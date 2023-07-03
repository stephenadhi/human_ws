import torch
import torch.nn as nn
from nav2_soloco_controller.data.utils import dynamic_window, actionXYtoROT
from nav2_soloco_controller.models.utils import Hist_Encoder, Decoder_TF


class TrajectoryGenerator(nn.Module):
    def __init__(self, robot_params_dict, dt, device, collision_distance=0.2, obs_len=8,
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
        # self.critic_imput = torch.zeros([12, self.obs_len, (5 + 1)*sample_batch, 2], device=self.device)
        # self.critic_imput_test = torch.zeros([self.obs_len, (5 + 1) * sample_batch, 2], device=self.device)
        self.dt = dt

    def forward(self, traj_rel, robot_state, z=None, robotID=None,
                ar_step_or_DWA=True, calc_new=True, optimize_latent_space=False, mean_pred=True):

        batch = traj_rel.shape[1]
        if mean_pred:
            noise_sampled = torch.zeros(self.predictions_steps, batch, 2, device=self.device)
        else:
            noise_sampled = torch.randn(self.predictions_steps, batch, 2, device=self.device)

        noise_sampled[:, robotID] = z

        if calc_new:
            num_h = batch // self.sample_batch
            enc_hist = self.hist_encoder(traj_rel[: self.obs_len, :num_h])
            mu, scale = self.decoder(torch.zeros_like(noise_sampled[:, :num_h]), enc_hist)
            scale = torch.clamp(scale, min=-9, max=4)
            self.mu = mu.repeat(1, self.sample_batch, 1)
            self.scale = scale.repeat(1, self.sample_batch, 1)

        output_pred_sampled = self.mu + torch.exp(self.scale) * noise_sampled
        clamped_action = []
        if self.robot_params_dict['use_robot_model'] and ar_step_or_DWA:
            for i in range(self.predictions_steps):
                if optimize_latent_space:
                    u = actionXYtoROT(output_pred_sampled[i, robotID], robot_state, self.dt)
                else:
                    u = z[i]
                robot_state = dynamic_window(robot_state, u,
                                             self.robot_params_dict,
                                             self.dt)
                clamped_action.append(robot_state[:, 3:].unsqueeze(dim=0))
                output_pred_sampled[i, robotID] = robot_state[:, :2] * self.dt
            clamped_action = torch.cat(clamped_action)
     #   else:
            # only in cem we are doing this non DWA split
      #      output_pred_sampled[:, robotID] = torch.clamp(output_pred_sampled[:, robotID], min=-self.robot_params_dict["max_speed"]*self.dt,
       #                              max=self.robot_params_dict["max_speed"]*self.dt)


        nll = ((output_pred_sampled[:, robotID] - self.mu[:, robotID]) ** 2).sum(-1).sum(0)
        return output_pred_sampled[:self.predictions_steps], self.mu, self.scale, clamped_action, nll
