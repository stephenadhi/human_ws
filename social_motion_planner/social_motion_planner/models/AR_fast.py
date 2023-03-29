import torch
import torch.nn as nn
import torch.nn.functional as F
from model.utils import Pooling_net, Hist_Encoder


class TrajectoryGeneratorAR(nn.Module):
    def __init__(self, num_agent, robot_params_dict, dt, collision_distance=0.2, obs_len=8,
                 predictions_steps=12, sample_batch=200, device='cuda'):
        super(TrajectoryGeneratorAR, self).__init__()
        self.num_agent = num_agent
        self.robot_params_dict = robot_params_dict
        self.sample_batch = sample_batch
        self.collision_distance = collision_distance
        self.obs_len = obs_len
        self.pred_len = predictions_steps
        traj_lstm_input_size = 2
        rela_embed_size = 16
        traj_lstm_hidden_size = 16
        self.device = device
        self.inputLayer_encoder = nn.Linear(traj_lstm_input_size, rela_embed_size)
        self.inputLayer_decoder = nn.Linear(traj_lstm_input_size + 16, rela_embed_size)
        self.pl_net = Pooling_net(self.device, self.num_agent, h_dim=traj_lstm_hidden_size, ar_model=True)
        self.traj_lstm_hidden_size = traj_lstm_hidden_size
        self.pred_lstm_hidden_size = self.traj_lstm_hidden_size
        self.hist_encoder = Hist_Encoder(obs_len, self.device)
        self.pred_lstm_model = nn.LSTMCell(rela_embed_size, 16)
        self.pred_hidden2pos = nn.Linear(traj_lstm_hidden_size, 2 * 2)
        self.dt = dt
        self.dropout = nn.Dropout(p=0.0)

    def forward(self, traj_rel, obs_traj_pos,
                nei_index, robot_state=None, z=None, robotID=None, proposed_robot_action=None, robot_pred=True):

        pred_traj_rel = []
        mu_robot_list, scale_robot_list = [], []

        batch = obs_traj_pos.shape[1]
        num_h = batch // self.sample_batch

        enc_hist = self.hist_encoder(traj_rel[: self.obs_len])[-1]
        pred_lstm_hidden = enc_hist

        pred_lstm_c_t = torch.zeros_like(pred_lstm_hidden)
        output = traj_rel[self.obs_len - 1]
        lstm_state_context = torch.zeros_like(pred_lstm_hidden)
        curr_pos_abs = obs_traj_pos[-1]

        for i in range(self.pred_len):

            input_cat = torch.cat([lstm_state_context, output], dim=-1).detach()
            input_embedded = self.dropout(F.relu(self.inputLayer_decoder(input_cat)))
            lstm_state = self.pred_lstm_model(
                input_embedded, (pred_lstm_hidden, pred_lstm_c_t)
            )
            pred_lstm_hidden = lstm_state[0]
            pred_lstm_c_t = lstm_state[1]

            test = curr_pos_abs.reshape(self.sample_batch, num_h, 2)  # now here repeate just the 6
            test_corr = test.unsqueeze(dim=1).expand(self.sample_batch, num_h, num_h, 2)
            test_corr_index = (test_corr.transpose(1, 2) - test_corr)
            test_corr_index = test_corr_index.reshape(-1, num_h, 2)

            lstm_state_context = self.pl_net(test_corr_index, nei_index, pred_lstm_hidden)
            concat_output = pred_lstm_hidden + lstm_state_context
            mu, scale = self.pred_hidden2pos(concat_output).chunk(2, 1)
            scale = torch.clamp(scale, min=-9, max=4)
            mu_robot_list.append(mu)
            scale_robot_list.append(torch.exp(scale))
            sample_noise = torch.randn_like(scale)
            # if robot_pred:
            #     sample_noise[robotID] = z[i]
            #     output_pred = mu + torch.exp(scale) * sample_noise
            #     if proposed_robot_action is not None:
            #         output_pred[robotID] = proposed_robot_action[i]  # get proposed actions here from other module
            #     if self.robot_params_dict['use_robot_model']:
            #         u = actionXYtoROT(output_pred[robotID], robot_state, self.dt)
            #         robot_state = dynamic_window(robot_state, u,
            #                                           self.robot_params_dict,
            #                                           self.dt)
            #         output_pred[robotID] = robot_state[:, :2] * self.dt
            # else:
            #     do =0
            output_pred = mu + torch.exp(scale) * sample_noise



            curr_pos_abs = (curr_pos_abs + output_pred).detach()
            pred_traj_rel += [output_pred]
            output = output_pred

        pred_traj_rel = torch.stack(pred_traj_rel)
        mu_robot_list = torch.stack(mu_robot_list)
        scale_robot_list = torch.stack(scale_robot_list)

        return pred_traj_rel, mu_robot_list, scale_robot_list
