import torch
from torch import nn
import numpy as np
import os
from rclpy.logging import get_logger

from nav2_soloco_controller.models.AR_fast_goal import TrajectoryGeneratorAR_goal
from nav2_soloco_controller.models.IAR_full_Tranf_Goal import TrajectoryGeneratorGoalIAR
#from nav2_soloco_controller.data.social_nav_env import SocialNavEnv, evaluate
from nav2_soloco_controller.models.IAR_full_Tranf import TrajectoryGenerator

from nav2_soloco_controller.data.utils import prepare_states, batched_Robot_coll_smoothed_loss, cart2pol, pol2cart, GaußNLL

#import matplotlib.pyplot as plt




def _ensure_non_zero(cost, beta, factor):
    return torch.exp(-factor * (cost - beta))


class Parallel_MPPI(nn.Module):
    def __init__(self, robot_params_dict, human_reaction=False, costmap_obj=None, dt=0.4, hist=8,
                 num_agent=5, obstacle_cost_gain=3000,
                 soft_update=False, bc=False,
                 device='cuda'):
        super(Parallel_MPPI, self).__init__()
        self.logger = get_logger("my_logger")
        self.robot_params_dict = robot_params_dict
        self.dt = dt
        self.device = device
        self.costmap_obj = costmap_obj
     #   if self.costmap_obj is not None:
     #       self.real_robot = True
     #   else:
     #       self.real_robot = False
        self.real_robot = True
        self.num_threads = 1
        self.sample_batch_per_thread = 800
        self.predictions_steps = 12
        self.max_iters = 1
        self.epsilon = 0.001
        self.alpha = 0.2
        self.num_elites = 800
        self.lambda_ = .001
        self.sample_batch = self.sample_batch_per_thread * self.num_threads
        self.init_mean = torch.zeros([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.init_var = torch.ones([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.U_init = torch.zeros([self.predictions_steps, self.sample_batch, 2], device=self.device)
        self.human_reaction = human_reaction  # predict human reaction with ar model?
        self.index_serial = torch.arange(self.num_threads, device=self.device).unsqueeze(dim=-1).repeat(1,
                                                                                                        self.num_elites) * self.sample_batch_per_thread
        self.obstacle_cost_gain = obstacle_cost_gain
        self.soft_update = soft_update
        self.bc = bc
        self.hist = hist
        self.num_agent = num_agent
        self.get_model()

        self.num_ids_for_plot = 50
        self._ids_for_plot = None
#        for param in self.pred_model.parameters():
 #           param.requires_grad = False

    def calc_cost_map_cost(self, trajectory, cost_map_obj, opt_count):
        cost = cost_map_obj.get_cost_from_world_x_y(trajectory[:].cpu().numpy())
        # if opt_count == 4:
        cost = np.where(cost > 96, 100000, cost)

        return cost.sum(0).astype(int)

    def get_model(self):

        _dir = os.path.dirname(__file__) or '.'
        _dir = _dir + "/weights/"

        if self.human_reaction:
            checkpoint_path = _dir + 'SIM2Goal-univ_fast_AR/checkpoint_with_model.pt'
            checkpoint = torch.load(checkpoint_path, map_location=torch.device(self.device))
            self.model_g = TrajectoryGeneratorAR_goal(self.num_agent, self.robot_params_dict, self.dt,
                                                      predictions_steps=self.predictions_steps,
                                                      sample_batch=self.sample_batch,
                                                      device=self.device)
            self.model_g.load_state_dict(checkpoint["best_state"])

            if self.device == 'cuda':
                self.model_g.cuda()
            else:
                self.model_g.cpu()
        else:
            checkpoint_path = _dir + 'SIM2Goal-univ_IAR/checkpoint_with_model.pt'
            checkpoint = torch.load(checkpoint_path, map_location=torch.device(self.device))
            self.model_g = TrajectoryGeneratorGoalIAR(self.robot_params_dict, self.dt, self.device,
                                                      predictions_steps=self.predictions_steps,
                                                      sample_batch=self.sample_batch,
                                                      )
            self.model_g.load_state_dict(checkpoint["best_state"])

            if self.device == 'cuda':
                self.model_g.cuda()
            else:
                self.model_g.cpu()

        checkpoint_path = _dir + 'SIMNoGoal-univ_IAR_Full_trans/checkpoint_with_model.pt'
        checkpoint = torch.load(checkpoint_path, map_location=torch.device(self.device))
        self.model_iar = TrajectoryGenerator(self.robot_params_dict, self.dt, self.device,
                                             predictions_steps=self.predictions_steps, sample_batch=self.sample_batch)
        self.model_iar.load_state_dict(checkpoint["best_state"])
        if self.device == 'cuda':
            self.model_iar.cuda()
        else:
            self.model_iar.cpu()
        self.model_iar.eval()
    def calc_r_goal_clamped(self, r_goal, last_obs_r_pos, calc_new):
        if calc_new:
            goal_diff = (r_goal - last_obs_r_pos)
            v_g, yaw_g = cart2pol(goal_diff[:, 0], goal_diff[:, 1])
            v_g = torch.clamp(v_g, max=10.0)
            x_g, y_g = pol2cart(v_g.unsqueeze(-1), yaw_g.unsqueeze(-1))
            self.goal_clamped = torch.cat([x_g, y_g], dim=-1) + last_obs_r_pos
            return self.goal_clamped
        else:
            return self.goal_clamped

   
        plt.figure()

        for i in range(1*6):
            # Plot trajectory
            xs, ys = trajectories[:, i, 0], trajectories[:, i, 1]
            plt.plot(xs, ys, label=f'Trajectory {i + 1}')

            # Plot corresponding goal
            goal = goals[i]
            if i < 6:
                plt.plot(*goal, 'ro')  # 'ro' plots a red circle
                plt.text(goal[0], goal[1], f'Goal {i + 1}', fontsize=12, ha='right')

        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Trajectories and their Goals')
       # plt.legend()
        plt.grid(True)
        plt.show()
    def calc_cost(self, data, z, ar_step_or_DWA=True, calc_new=True, costmap_obj=None):

        with torch.no_grad():

            obs_traj_pos, traj_rel, neigh_index, robot_idx, r_goal, r_pose = data[:6]
           # self.logger.info("dfsdf:" + str(self.num_agent))
            if calc_new:
                pred_traj_rel, mu, scale, self.pertu_actions_clamped, self.nll = self.model_iar(traj_rel, r_pose,
                                                                                                     robotID=robot_idx,
                                                                                                     z=z,
                                                                                                     ar_step_or_DWA=False,
                                                                                                     calc_new=calc_new)
                
                pred_traj_abs = torch.cumsum(pred_traj_rel, dim=0) + obs_traj_pos[-1]
                
        #        if self.costmap_obj:
        #           costmap_cost_human = torch.Tensor(self.calc_cost_map_cost(pred_traj_abs, costmap_obj, 0))
        #           nll_h = ((pred_traj_rel - mu) ** 2).sum(0).sum(-1)  # .unsqueeze(-1)
        #           cost_h = (costmap_cost_human + nll_h).reshape(self.sample_batch, -1)
        #           ids_h = torch.argmin(cost_h, dim=0)
        #           test_ids = ids_h * (self.num_agent + 1) + torch.arange(self.num_agent + 1, device=self.device)
        ##           pred_traj_abs = pred_traj_abs[:, test_ids].repeat(1, 1, self.sample_batch, 1).permute(0, 2, 1,
        #                                                                                                 3).reshape(
        #               self.predictions_steps, -1, 2)

                self.goal = pred_traj_abs[-1]
             #   self.logger.info("dfsdf:" + str(self.goal.shape))
            self.goal[robot_idx] = r_goal # self.calc_r_goal_clamped(r_goal, obs_traj_pos[-1, robot_idx], calc_new)

            self.pred_traj_rel, self.pertu_actions_clamped, self.nll = self.model_g(traj_rel, obs_traj_pos,
                                                                                                   neigh_index,
                                                                                                   r_pose, self.goal,
                                                                                                   robotID=robot_idx, z=z,
                                                                                                   mean_pred = True)
            self.pred_traj_abs = torch.cumsum(self.pred_traj_rel, dim=0) + obs_traj_pos[-1]
            goal_cost = ((self.goal - self.pred_traj_abs) ** 2).sum(dim=-1)
            goal_cost = (goal_cost).mean(0).reshape(self.sample_batch, -1).sum(-1)

            coll_cost = batched_Robot_coll_smoothed_loss(self.pred_traj_abs, self.sample_batch,
                                                         predictions_steps=self.predictions_steps,
                                                         batch=True,
                                                         collision_dist=self.robot_params_dict["collision_dist"]).view(
                                                         self.predictions_steps, -1).sum(0)
            if costmap_obj:
                costmap_cost = self.calc_cost_map_cost(self.pred_traj_abs[:, robot_idx], costmap_obj, 0)
            else:
                costmap_cost = 0.
        return goal_cost, coll_cost, self.pertu_actions_clamped, costmap_cost, self.pred_traj_rel[:,
                                                                               robot_idx], self.nll

    def get_pred_traj_abs(self):
        agent_future = self.pred_traj_abs.cpu().numpy().transpose(1, 0, 2)
        agent_future = agent_future.reshape(self.num_threads, self.sample_batch_per_thread, -1, self.predictions_steps,
                                            2)
        agent_future = agent_future[
            self.min_thread_id, self._ids_for_plot[self.min_thread_id, :self.num_ids_for_plot]].mean(axis=0)
        return agent_future

        
    def predict(self, x, costmap_obj=None):
        self.costmap_obj = costmap_obj
        with torch.no_grad():
          #  self.logger.info("1:" + str(x.shape))
            x = torch.as_tensor(np.array(x), dtype=torch.float, device=self.device)
            x = x.repeat(self.sample_batch, 1)
            x = prepare_states(x, self.hist, self.num_agent, bc=self.bc, device=self.device)
            var = self.init_var.clone()
            opt_count = 0
            calc_new = True
            U = self.U_init
          #  self.logger.info("dfsdf:" + str(x[0].shape))
            while (opt_count < self.max_iters):
                ar_step_or_DWA = opt_count % 2 == 0
                if torch.max(var) < self.epsilon and not ar_step_or_DWA:
                    break

                noise = var * torch.randn_like(var)
                pertu_actions = U + noise

                goal_cost, coll_cost, pertu_actions_clamped, costmap_cost, pred_traj_fake, nll = self.calc_cost(x,
                                                                                                                 pertu_actions.float(),
                                                                                                                 ar_step_or_DWA=ar_step_or_DWA,
                                                                                                                 calc_new=calc_new,
                                                                                                                 costmap_obj=self.costmap_obj)
                calc_new = False
                noise_clamped = pertu_actions_clamped - U
                cost = goal_cost + self.obstacle_cost_gain * coll_cost + costmap_cost #+ nll #+ 0.1*nll #+ torch.abs(pertu_actions_clamped[:,:,1]).mean(0)

                cost_reshaped = cost.reshape(self.num_threads, self.sample_batch_per_thread)
                elite_cost, ids = torch.sort(cost_reshaped, descending=False)  # [:, :self.num_elites]
                cost_reshaped = elite_cost[:, :self.num_elites]
                self._ids_for_plot = ids[:, :self.num_ids_for_plot] + self.index_serial[:, :self.num_ids_for_plot]
                elite_ids = ids[:, :self.num_elites] + self.index_serial
                elite_ids = elite_ids.reshape(-1)
                elite_noise = noise_clamped[:, elite_ids].reshape(self.predictions_steps, self.num_threads,
                                                                  self.num_elites, 2)
                beta, batch_id = torch.min(cost_reshaped, dim=1)
                _, self.min_thread_id = torch.min(beta, dim=0)
                self.cost_total_non_zero = _ensure_non_zero(cost_reshaped, beta.unsqueeze(dim=-1), 1 / self.lambda_)
                eta = torch.sum(self.cost_total_non_zero, dim=1, keepdim=True)
                self.omega = ((1. / eta) * self.cost_total_non_zero).view(self.num_threads, self.num_elites, 1)
                perturbations = []
                for t in range(self.predictions_steps):
                    perturbations.append(torch.sum(self.omega * elite_noise[t], dim=1))
                perturbations = torch.stack(perturbations)
                std_new = torch.sqrt(torch.sum(self.omega * (elite_noise - perturbations.unsqueeze(dim=2)) ** 2, dim=2))
                var = std_new.unsqueeze(dim=2).repeat(1, 1, self.sample_batch_per_thread, 1).view(
                    self.predictions_steps, -1, 2)
                perturbations = perturbations.unsqueeze(dim=2).repeat(1, 1, self.sample_batch_per_thread, 1).view(
                    self.predictions_steps, -1, 2)

                U = U + perturbations
                opt_count += 1

            self.U_init = torch.roll(U, -1, 0)

            current_future = self.get_pred_traj_abs()

            if self.real_robot:
                return U[0, 0], current_future
            else:
                return [U[0, 0].cpu().numpy()]

    def reset(self):
        self.U_init = torch.zeros([self.predictions_steps, self.sample_batch, 2], device=self.device)
        return None

