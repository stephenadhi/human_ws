import math

import torch
from torch import nn
import numpy as np
import os

from social_motion_planner.models.AR_fast import TrajectoryGeneratorAR
from social_motion_planner.models.IAR_full_Tranf import TrajectoryGenerator

torch.autograd.set_detect_anomaly(True)
from data.utils import prepare_states, batched_Robot_coll_smoothed_loss, cart2pol, pol2cart, GaußNLL, actionXYtoROT


class Prediction_Model(nn.Module):
    def __init__(self, robot_params_dict, dt, feature_size,
                 hist_steps,predictions_steps, num_agent,
                 sample_batch, device='cuda'):
        super(Prediction_Model, self).__init__()
        self.robot_params_dict = robot_params_dict
        self.dt = dt
        self.feature_size = feature_size
        self.hist = hist_steps
        self.predictions_steps = predictions_steps
        self.num_agent = num_agent
        self.sample_batch = sample_batch
        self.device = device
        self.pred_model_iar, self.pred_model_ar = self.get_model(sample_batch)
        
    def get_model(self, sample_batch):

        _dir = os.path.dirname(__file__) or '.'
        _dir = _dir + "/weights/"
        checkpoint_path = _dir + 'SIMNoGoal-univ_fast_AR2/checkpoint_with_model.pt'

        checkpoint = torch.load(checkpoint_path, map_location=torch.device(self.device))
        model_ar = TrajectoryGeneratorAR(self.num_agent, self.robot_params_dict, self.dt,
                                         predictions_steps = self.predictions_steps,
                                         sample_batch=sample_batch,
                                         device=self.device)
        model_ar.load_state_dict(checkpoint["best_state"])
        if self.device == 'cuda':
            model_ar.cuda()
        else:
            model_ar.cpu()
        model_ar.eval()
        checkpoint_path = _dir + 'SIMNoGoal-univ_IAR_Full_trans/checkpoint_with_model.pt'
        checkpoint = torch.load(checkpoint_path, map_location=torch.device(self.device))

        model_iar = TrajectoryGenerator(self.robot_params_dict, self.dt, self.device, predictions_steps=self.predictions_steps, sample_batch=sample_batch)
        model_iar.load_state_dict(checkpoint["best_state"])
        if self.device == 'cuda':
            model_iar.cuda()
        else:
            model_iar.cpu()
        model_iar.eval()

        return model_iar, model_ar

    def calc_to_goal_cost(self, trajectory, goal, robot_state):
        """
            calc to goal cost with angle difference
        """
        dx = goal[:,0] - trajectory[-1, :, 0]
        dy = goal[:,1] - trajectory[-1, :, 1]
        error_angle = torch.atan2(dy, dx)
        cost_angle = error_angle - robot_state[:, 2]
        cost = abs(torch.atan2(torch.sin(cost_angle), torch.cos(cost_angle)))

        return cost

    def calc_cost_map_cost(self, trajectory, cost_map_obj, opt_count):
        cost =  cost_map_obj.get_cost_from_world_x_y(trajectory[:].cpu().numpy())
        if opt_count==4:
           cost = np.where(cost > 96, np.inf, cost)

        return cost.sum(0)

    def forward(self, data, z, goal=None, ar_step_or_DWA=True, calc_new=True, costmap_obj=None, opt_count=0):

        with torch.no_grad():

            obs_traj_pos, traj_rel, neigh_index, robot_idx, r_goal, r_pose = data[:6] # if CEM is used for bc, prep_state will produce an additional next state which we do not need here

            goal[robot_idx] = r_goal
            pred_traj_rel, mu, scale, r_pose = self.pred_model_iar(traj_rel, r_pose, robotID=robot_idx, z=z,
                                                ar_step_or_DWA=ar_step_or_DWA, calc_new=calc_new)

            pred_traj_abs = torch.cumsum(pred_traj_rel, dim=0) + obs_traj_pos[-1]

            goal_diff = (goal[robot_idx] - obs_traj_pos[-1, robot_idx])
            v_g, yaw_g = cart2pol(goal_diff[:, 0], goal_diff[:, 1])
            v_g = torch.clamp(v_g, max=3.0)
            x_g,y_g = pol2cart(v_g, yaw_g)
            goal_clamped = torch.cat([x_g, y_g], dim=-1) + obs_traj_pos[-1, robot_idx]

            # Calculate costs
            goal_cost = torch.sqrt(((goal_clamped - pred_traj_abs[0, robot_idx]) ** 2).sum(dim=-1))#.sum(0)
            # goal_cost = self.calc_to_goal_cost(pred_traj_abs[:, robot_idx], goal[robot_idx], robot_state )
            speed_cost = (0.5 - r_pose[:, 3])
            costmap_cost = torch.from_numpy(self.calc_cost_map_cost(pred_traj_abs[:, robot_idx], costmap_obj, opt_count).astype(np.float32, copy=False)).to(goal_cost)
            coll_cost = batched_Robot_coll_smoothed_loss(pred_traj_abs, self.sample_batch,
                                                         predictions_steps=self.predictions_steps,
                                                         batch=True, collision_dist=self.robot_params_dict["collision_dist"]).view(self.predictions_steps, -1).sum(0)
            nll = GaußNLL(mu[:, robot_idx], scale[:, robot_idx], pred_traj_rel[:, robot_idx])
            
        return goal_cost, coll_cost, nll, speed_cost, costmap_cost, pred_traj_rel[:, robot_idx]


class CEM_IAR(nn.Module):
    def __init__(self, robot_params_dict, dt, hist=8,
                 num_agent=5, obstacle_cost_gain=1000,
                 soft_update= False, bc = False,
                 device='cuda'):
        super(CEM_IAR, self).__init__()
        self.device = device
        self.sample_batch = 100
        self.predictions_steps = 12
        self.init_mean = torch.zeros([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.init_var = torch.ones([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.max_iters = 5 # eval without robot constrains 1 53 success, 2- 84.7, 3- 0.9548, 4- 0.9703, 5- 0.9857 , 6- same
        self.epsilon = 0.001
        self.alpha = 0.2
        self.num_elites = 6
        print(self.sample_batch)
        print(self.max_iters)
        print(self.num_elites)
        self.obstacle_cost_gain = obstacle_cost_gain
        self.soft_update = False #soft_update
        self.bc = bc
        self.hist = hist
        self.num_agent = num_agent
        self.device= device
        self.pred_model = Prediction_Model(robot_params_dict, dt, 16, self.hist,
                                           self.predictions_steps, self.num_agent,
                                           self.sample_batch, device=self.device,
                                           )
        for param in self.pred_model.parameters():
            param.requires_grad = False


    def predict(self, x, costmap_obj=None):
        with torch.no_grad():
            x = torch.as_tensor(np.array(x), dtype=torch.float, device=self.device)
            x = x.repeat(self.sample_batch, 1)
            x = prepare_states(x, self.hist, self.num_agent, bc=self.bc, device=self.device)
            pred_traj_fake_goal = torch.zeros_like(x[0][0])
            mean = self.init_mean.clone()
            var = self.init_var.clone()
            opt_count = 0
            calc_new = True
            while (opt_count < self.max_iters) and torch.max(var) > self.epsilon:
                samples = mean + var * torch.randn_like(var)
                ar_step_or_DWA = opt_count % 2 == 0
                goal_cost, coll_cost, nll, speed_cost, costmap_cost, pred_traj_fake = self.pred_model(x, samples,
                                                                                                      pred_traj_fake_goal,
                                                                                                      ar_step_or_DWA=True,
                                                                                                      opt_count=opt_count,
                                                                                                      calc_new=calc_new,
                                                                                                      costmap_obj=costmap_obj)
                calc_new = False
                cost = goal_cost + 0* speed_cost + 100.* costmap_cost + 1.*nll#+ self.obstacle_cost_gain*coll_cost #+ nll
                best_ids = torch.argsort(cost, descending=False)
                elites = samples[:, best_ids][:, :self.num_elites]
                new_mean = torch.mean(elites, dim=1)
                new_var = torch.std(elites, dim=1)
                if self.soft_update:
                    # soft update
                    mean = (self.alpha * mean[:,0] + (1. - self.alpha) * new_mean).unsqueeze(dim=1).repeat(1,self.sample_batch,1)
                    var = (self.alpha * var[:,0] + (1. - self.alpha) * new_var).unsqueeze(dim=1).repeat(1,self.sample_batch,1)
                else:
                    mean = new_mean.unsqueeze(dim=1).repeat(1, self.sample_batch,1)
                    var = new_var.unsqueeze(dim=1).repeat(1, self.sample_batch,1)
                opt_count += 1

            all_coll_check = coll_cost[best_ids]
            check_costmap_costs = costmap_cost[best_ids[0]]
            all_coll_check = (all_coll_check > 0.8).sum()

            if  check_costmap_costs == np.inf:
                u = np.array([0., 0.])  # stop robot if more than 50% of predicted states are collisions
               # print(u)
            else:
                #best_action = pred_traj_fake[:, best_ids[:self.num_elites]].mean(dim=1).cpu().numpy() # <--- better
                best_action = pred_traj_fake[0, best_ids[0]] #.cpu().numpy()
                u = actionXYtoROT(best_action.unsqueeze(0), x[5][0].unsqueeze(0), 0.4)[0].cpu().numpy()
               # print(u)
                if u[1] > 1.1:
                    print('Error')
                    u = np.array([0., 0.])
            self.pred_model.plot_list = []
            return u