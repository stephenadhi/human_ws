import torch
import torch.nn as nn
from soloco_planner.data.utils import dynamic_window, actionXYtoROT
from soloco_planner.models.utils import Hist_Encoder, Decoder_TF


class TrajectoryGenerator(nn.Module):
    def __init__(self,robot_params_dict, dt, device, collision_distance=0.2, obs_len=8,
                 predictions_steps=12, sample_batch=200):
        super(TrajectoryGenerator, self).__init__()

        self.device = device
        self.robot_params_dict = robot_params_dict
        self.sample_batch = sample_batch
        self.obs_len = obs_len
        self.predictions_steps = predictions_steps
        self.hist_encoder = Hist_Encoder(obs_len, self.device)
        self.decoder = Decoder_TF(self.device)
        self.collision_distance = collision_distance
        self.critic_imput = torch.zeros([12, self.obs_len, (5 + 1)*sample_batch, 2], device=self.device)
        self.critic_imput_test = torch.zeros([self.obs_len, (5 + 1) * sample_batch, 2], device=self.device)
        self.dt = dt

    def forward(self, traj_rel, robot_state, z=None, robotID=None,
                ar_step_or_DWA=True, calc_new=True):

        batch = traj_rel.shape[1] #/ 12

        noise_sampled = torch.randn(self.predictions_steps, batch, 2, device=self.device)
        noise_sampled[:, robotID] = z
        if calc_new:
            enc_hist = self.hist_encoder(traj_rel[: self.obs_len])
            mu, scale = self.decoder(noise_sampled, enc_hist)
            scale = torch.clamp(scale, min=-9, max=4)
            self.mu = mu
            self.scale = scale

        test_state = robot_state.clone()
        output_pred_sampled = self.mu + torch.exp(self.scale) * noise_sampled
       # output_pred_sampled[:, robotID] = z # TODO: REmove this is just a test
        if self.robot_params_dict['use_robot_model'] and ar_step_or_DWA:
            for i in range(self.predictions_steps):
                u = actionXYtoROT(output_pred_sampled[i, robotID], robot_state, self.dt)
                robot_state = dynamic_window(robot_state, u,
                                             self.robot_params_dict,
                                             self.dt)
                output_pred_sampled[i, robotID] = robot_state[:, :2] * self.dt
        #test = actionXYtoROT(output_pred_sampled[0, robotID, :2], test_state, self.dt)
        return output_pred_sampled[:self.predictions_steps], self.mu, self.scale, robot_state