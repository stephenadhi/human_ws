import math

import torch
from torch import nn
import numpy as np
import os

from nav2_soloco_controller.models.AR_fast import TrajectoryGeneratorAR
from nav2_soloco_controller.models.IAR_full_Tranf import TrajectoryGenerator

torch.autograd.set_detect_anomaly(True)
from nav2_soloco_controller.data.utils import prepare_states, batched_Robot_coll_smoothed_loss, cart2pol, pol2cart, GaußNLL, actionXYtoROT
from rclpy.logging import get_logger

class Prediction_Model(nn.Module):
    def __init__(self, robot_params_dict, dt, feature_size,
                 hist_steps,predictions_steps, num_agent,
                 sample_batch, AR_checkpoint,
                 IAR_checkpoint, device='cuda'):
        super(Prediction_Model, self).__init__()
        self.logger = get_logger("my_logger")
        self.robot_params_dict = robot_params_dict
        self.dt = dt
        self.feature_size = feature_size
        self.hist = hist_steps
        self.predictions_steps = predictions_steps
        self.num_agent = num_agent
        self.sample_batch = sample_batch
        self.AR_checkpoint = AR_checkpoint
        self.IAR_checkpoint = IAR_checkpoint
        self.device = device
        self.pred_model_iar, _ = self.get_model(sample_batch)
        self.plot_list = []
        self.pred_traj_abs = None


    def get_model(self, sample_batch):

        _dir = os.path.dirname(__file__) or '.'
        _dir = _dir + "/weights/"

        # checkpoint_path = _dir + 'Test_no_goal_loss/checkpoint_with_model.pt'#EXPERIMENT_NAME + '-univ/checkpoint_with_model.pt'
        # checkpoint_path = _dir + 'SIMNoGoal-univ_fast_AR2/checkpoint_with_model.pt'
        # checkpoint_path = '/home/martin/Sim2Goal/models/weights/SIMNoGoal-univ_fast_AR_debug2/checkpoint_with_model.pt'
        checkpoint = torch.load(self.AR_checkpoint, map_location=torch.device(self.device))
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
        # checkpoint_path = _dir + 'SIMNoGoal-univ_IAR_Full_trans/checkpoint_with_model.pt'
        # checkpoint_path = '/home/martin/Sim2Goal/models/weights/SIMNoGoal-univ_noSocialCRT/checkpoint_with_model.pt'
        # checkpoint_path = '/home/martin/Sim2Goal/models/weights/SIMNoGoal-univ_IAR_test/checkpoint_with_model.pt'
        # checkpoint_path = '/home/martin/Sim2Goal/models/weights/SIMNoGoal-univ_IAR_Transf_enc_test/checkpoint_with_model.pt'
        # checkpoint_path = '/home/martin/Sim2Goal/models/weights/SIMNoGoal-univ_IAR_Full_trans_debug8/checkpoint_with_model.pt'
        checkpoint = torch.load(self.IAR_checkpoint, map_location=torch.device(self.device))

        # checkpoint_sampler_path = _dir + 'GFLOW-ETHandUCY_sampler-univ/checkpoint_with_model.pt'
        # checkpoint_sampler = torch.load(checkpoint_sampler_path)
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
        cost = cost_map_obj.get_cost_from_world_x_y(trajectory[:4].cpu().numpy())
       # if opt_count == 4:
        cost = np.where(cost > 96, np.inf, cost)

        return cost.sum(0)

    def calc_cost_map_cost2(self, trajectory, cost_map_obj, opt_count):
        cost =  cost_map_obj.get_cost_from_world_x_y(trajectory[:].cpu().numpy())
       # if opt_count >=4:
       #    cost = np.where(cost >= 97, np.inf, cost)
        return np.max(cost, axis=0)
       # return cost.sum(0)

    def forward(self, data, z, goal=None, ar_step_or_DWA=True, calc_new=True, costmap_obj=None, opt_count=0):

        with torch.no_grad():

            obs_traj_pos, traj_rel, neigh_index, robot_idx, r_goal, r_pose = data[:6] # if CEM is used for bc, prep_state will produce an additional next state which we do not need here

            goal[robot_idx] = r_goal
            pred_traj_rel, mu, scale, self.pertu_actions_clamped, self.nll = self.pred_model_iar(traj_rel, r_pose, robotID=robot_idx, z=z,
                                                ar_step_or_DWA=ar_step_or_DWA, calc_new=calc_new)

            pred_traj_abs = torch.cumsum(pred_traj_rel, dim=0) + obs_traj_pos[-1]
            self.pred_traj_abs = pred_traj_abs
            goal_diff = (goal[robot_idx] - obs_traj_pos[-1, robot_idx])
            v_g, yaw_g = cart2pol(goal_diff[:, 0], goal_diff[:, 1])
            v_g = torch.clamp(v_g, max=3.0)
            x_g,y_g = pol2cart(v_g, yaw_g)
            goal_clamped = torch.cat([x_g, y_g], dim=-1) + obs_traj_pos[-1, robot_idx]
            # plot_traj(obs_traj_pos, pred_traj_abs, goal_clamped, robot_idx)
            self.plot_list.append([obs_traj_pos, pred_traj_abs, goal_clamped, robot_idx])
            goal_cost = torch.sqrt(((goal_clamped - pred_traj_abs[:, robot_idx]) ** 2).sum(dim=-1)).sum(0)
           # speed_cost = (0.5 - r_pose[:, 3])
            costmap_cost = self.calc_cost_map_cost(pred_traj_abs[:, robot_idx], costmap_obj, opt_count)
           # goal_cost = self.calc_to_goal_cost(pred_traj_abs[:, robot_idx], goal[robot_idx], r_pose )
            coll_cost = batched_Robot_coll_smoothed_loss(pred_traj_abs, self.sample_batch,
                                                         predictions_steps=self.predictions_steps,
                                                         batch=True, collision_dist=self.robot_params_dict["collision_dist"]).view(self.predictions_steps, -1).sum(0)
            nll = GaußNLL(mu[:, robot_idx], scale[:, robot_idx], pred_traj_rel[:, robot_idx])
        return goal_cost, coll_cost, nll, 0, costmap_cost, pred_traj_rel[:, robot_idx]


class CEM_IAR(nn.Module):
    def __init__(self, robot_params_dict, dt=0.4, hist=8, sample_batch=400,
                 num_agent=5, obstacle_cost_gain=1000, prediction_steps=12,
                 soft_update= False, bc = False,
                 AR_checkpoint='weights/SIMNoGoal-univ_fast_AR2/checkpoint_with_model.pt',
                 IAR_checkpoint='weights/SIMNoGoal-univ_IAR_Full_trans/checkpoint_with_model.pt',
                 device='cuda'):
        super(CEM_IAR, self).__init__()
        self.logger = get_logger("my_logger")
        self.device = device
        self.sample_batch = sample_batch
        self.predictions_steps = prediction_steps
        self.init_mean = torch.zeros([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.init_var = torch.ones([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.max_iters = 5 # eval without robot constrains 1 53 success, 2- 84.7, 3- 0.9548, 4- 0.9703, 5- 0.9857 , 6- same
        self.epsilon = 0.001
        self.alpha = 0.2
        self.num_elites = 20
        print(self.sample_batch)
        print(self.max_iters)
        print(self.num_elites)
        self.obstacle_cost_gain = obstacle_cost_gain
        self.soft_update = False #soft_update
        self.bc = bc
        self.hist = hist
        self.num_agent = num_agent
        self.AR_checkpoint = AR_checkpoint
        self.IAR_checkpoint = IAR_checkpoint
        self.device= device
        self.pred_model = Prediction_Model(robot_params_dict, dt, 16, self.hist,
                                           self.predictions_steps, self.num_agent,
                                           self.sample_batch, AR_checkpoint=self.AR_checkpoint,
                                           IAR_checkpoint=self.IAR_checkpoint,
                                           device=self.device, 
                                           )
        for param in self.pred_model.parameters():
            param.requires_grad = False


    def get_pred_traj_abs(self):
        agent_future = self.pred_model.pred_traj_abs.cpu().numpy().transpose(1, 0, 2)
        agent_future = agent_future.reshape(self.sample_batch, -1, self.predictions_steps, 2)
        agent_future = agent_future[self.best_id.cpu().numpy()].mean(axis=0)
        return agent_future

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
            while (opt_count < self.max_iters): # and torch.max(var) > self.epsilon:
                samples = mean + var * torch.randn_like(var)
                ar_step_or_DWA = opt_count % 2 == 0
                goal_cost, coll_cost, nll, speed_cost, costmap_cost, pred_traj_fake = self.pred_model(x, samples,
                                                                                                      pred_traj_fake_goal,
                                                                                                      ar_step_or_DWA=ar_step_or_DWA,
                                                                                                      opt_count=opt_count,
                                                                                                      calc_new=calc_new,
                                                                                                      costmap_obj=costmap_obj)
                calc_new = False
                costmap_cost = torch.tensor(costmap_cost, device=self.device)
                cost = 1* goal_cost + 1000.* coll_cost + 1* costmap_cost + 1.*nll#+ self.obstacle_cost_gain*coll_cost #+ nll
                best_ids = torch.argsort(cost, descending=False)
                elites = samples[:, best_ids][:, :self.num_elites]
                new_mean = torch.mean(elites, dim=1)
                new_var = torch.std(elites, dim=1)

                if self.soft_update:
                    # soft update
                    mean = (self.alpha * mean[:, 0] + (1. - self.alpha) * new_mean).unsqueeze(dim=1).repeat(1,self.sample_batch,1)
                    var = (self.alpha * var[:, 0] + (1. - self.alpha) * new_var).unsqueeze(dim=1).repeat(1,self.sample_batch,1)
                else:
                    mean = new_mean.unsqueeze(dim=1).repeat(1, self.sample_batch,1)
                    var = new_var.unsqueeze(dim=1).repeat(1, self.sample_batch,1)
                opt_count += 1

            all_coll_check = coll_cost[best_ids]
            check_costmap_costs = costmap_cost[best_ids[:self.num_elites]]
           # all_coll_check = (all_coll_check > 0.8).sum()
           # if all_coll_check / self.sample_batch > 0.8:
           #     u = np.array([0., 0.])  # stop robot if more than 50% of predicted states are collisions
           #     print('crash with human')
            self.best_id = best_ids[:self.num_elites]
            if  check_costmap_costs.sum() == np.inf:
                u = np.array([0., 0.])
                print('crash with obstacle')
            else:
                best_action = pred_traj_fake[:, best_ids[:self.num_elites]].mean(dim=1)#.cpu().numpy() # <--- better
                #best_action = pred_traj_fake[0, best_ids[0]] #.cpu().numpy()
                u = actionXYtoROT(best_action, x[5][0].unsqueeze(0), 0.4)[0].cpu().numpy()
           # print(u)
            if u[1] > 1.1:
                print('Error')
                u = np.array([0., 0.])
            self.pred_model.plot_list = []

            current_future = self.get_pred_traj_abs()
            
            return u, current_future