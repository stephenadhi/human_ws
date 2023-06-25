import warnings
import math
import numpy as np
import torch
import os
from nav2_soloco_controller.models.config import ConfigRobot, ConfigSim, RobotType
from nav2_soloco_controller.models.AR_fast import TrajectoryGeneratorAR

warnings.filterwarnings("ignore", category=DeprecationWarning)


class DWA:
    def __init__(self, sim_config: ConfigSim, hist: int, num_agent: int, neigh_index, obstacle_cost_gain: float = 5.0):
        self.robot_model = sim_config.robot_model
        self.init_Flag = True
        self.num_agent = num_agent
        self.config = sim_config
        
        self.hist = hist
        self.neigh_index = neigh_index
        self.num_agent = num_agent
        #self.config.obstacle_cost_gain = obstacle_cost_gain
        
        # size of each batch
        self.sample_batch = 1
        
        
        # device to run the neural network (set to cuda if there is a cuda compatible cpu, else set to cpu)
        if torch.cuda.is_available():
            print("Found cuda device: ", torch.cuda.get_device_name(0))
            self.device = 'cuda'
        else:
            print("Using CPU")
            self.device = 'cpu'


        self.pred_time_steps = self.config.pred_time_steps + 1
        # initialize the model
        self.pred_model_ar = self.get_model(self.sample_batch)
      


    def get_model(self, sample_batch):
        _dir = os.path.dirname(__file__) or '.'
        _dir = _dir + "/model/weights/"

        # path to the pre-trained neural network's weights
        checkpoint_path = _dir + 'SIMNoGoal-univ_fast_AR2/checkpoint_with_model.pt'

        # load the weights
        if self.device == 'cpu':
            checkpoint = torch.load(checkpoint_path, map_location=torch.device('cpu'))
        else:
            checkpoint = torch.load(checkpoint_path)

        # initialize model
        model_ar = TrajectoryGeneratorAR(num_agent=self.num_agent,
                                         robot_params_dict=self.config.robot_model,
                                         dt=self.config.dt,
                                         collision_distance=0.2,  # not used inside the model?
                                         obs_len=self.hist + 1,  # agent_hist(always 7/no. frames into the past) + 1(for predicting robot)
                                         predictions_steps=self.pred_time_steps,
                                         sample_batch=sample_batch,
                                         device=self.device)

        # set the weights
        model_ar.load_state_dict(checkpoint["best_state"])

        # move the device to gpu
        if self.device == 'cuda':
            model_ar.cuda()

        # set the model to evaluation mode as we are not going to train it
        model_ar.eval()

        return model_ar

    def dwa_control(self, x, config, goal, ob, cost_map_obj):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x, config)

        u, trajectory = self.calc_control_and_trajectory(x, dw, config, goal, ob, cost_map_obj)
        return u, trajectory

    def motion(self, x, u, dt):
        """
        motion model
        """
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        # TODO: debug this function
        # u -> linear(x) and angular vel.(z) action / vel. -> 2 wheels needed to created curves
        # x -> current state for given time-step
        # publish u to cmd_vel
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x, config):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [config.min_speed, config.max_speed,
              -config.max_yaw_rate, config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - config.max_accel * config.dt,
              x[3] + config.max_accel * config.dt,
              x[4] - config.max_delta_yaw_rate * config.dt,
              x[4] + config.max_delta_yaw_rate * config.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def predict_trajectory(self, x_init, v, y, config):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            trajectory = np.vstack((trajectory, x))
            time += config.dt

        return trajectory

    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def calc_obstacle_cost(self, trajectory, ob, config):
        """
        calc obstacle cost inf: collision
        """
        
        """old
        ox = ob[:, 0]
        oy = ob[:, 1]

        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)
        """

        ox = ob[:, :, 0]
        oy = ob[:, :, 1]
        #print(ox)
        #print(ob)
        # if we found any pedestrian in fov
        #print(trajectory)      

        if ob.shape[0] > 1:
            # IMPORTANT: duplicate the first row of trajectory, because the first two rows of ox,oy
            # correspond to the obstacles in red and green area respectively, and the first row of the
            # trajectory contains the current position of the robot
            #trajectory = np.concatenate((np.expand_dims(trajectory[3], axis=0), trajectory), axis=0)
            trajectory = np.concatenate((np.expand_dims(trajectory[3], axis=0), trajectory), axis=0)
            
        
       # print(trajectory)      

        dx = trajectory[:, None, 0] - ox
        dy = trajectory[:, None, 1] - oy
        r = np.hypot(dx, dy)

        
        if config.robot_type == RobotType.rectangle:
            yaw = trajectory[:, 2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rot = np.transpose(rot, [2, 0, 1])
            local_ob = ob[:, None] - trajectory[:, 0:2]
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            local_ob = np.array([local_ob @ x for x in rot])
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            upper_check = local_ob[:, 0] <= config.robot_length / 2
            right_check = local_ob[:, 1] <= config.robot_width / 2
            bottom_check = local_ob[:, 0] >= -config.robot_length / 2
            left_check = local_ob[:, 1] >= -config.robot_width / 2
            if (np.logical_and(np.logical_and(upper_check, right_check),
                               np.logical_and(bottom_check, left_check))).any():
                return float("Inf")
        #elif config.robot_type == RobotType.circle:
            #if np.array(r <= config.robot_radius).any():
                #return float("Inf")

        min_r = np.nanmin(r)
        #print("min:" + str(min_r))
        return 1.0 / min_r  # OK

    def calc_control_and_trajectory(self, x, dw, config, goal, ob, cost_map_obj):
        """
        calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], config.v_resolution):
            for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y, config)
                # calc cost
                to_goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
                #ob_cost = config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, config)
                ob_cost = config.obstacle_cost_gain * self.calc_cost_map_cost(trajectory, cost_map_obj, config)
                #print("goal:" + str(to_goal_cost))
                #print("speed:" + str(speed_cost))
                #print("ob:" + str(ob_cost))
               # print("-------")
                final_cost = to_goal_cost + speed_cost + ob_cost
                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < config.robot_stuck_flag_cons \
                            and abs(x[3]) < config.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -config.max_yaw_rate  # Bug in original code? Corrected to max_yaw_rate instead
                        # max_delta_yaw_rate
        return best_u, best_trajectory

    def calc_cost_map_cost(self, trajectory, cost_map_obj, config):
        cost =  cost_map_obj.get_cost_from_world_x_y(trajectory[:, :2])
        cost_max = np.max(cost)
        if cost_max > 99:
            return np.inf
        else:
            return cost.sum()



    def cart2pol(self, x, y):
        """
        Function to convert cartesian coordinates into polar coordinates.
        """
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)
        return rho, phi

    def pol2cart(self, rho, phi):
        """
        Function to convert polar coordinates into cartesian coordinates.
        """
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return x, y

    def predict(self, obs_traj_pos: np.ndarray, obs_lidar, r_pos_xy: np.ndarray,
                r_goal_xy: np.ndarray, r_vel_state: np.ndarray, cost_map_obj: np.ndarray) -> np.ndarray:

        """
        # r_state -> v_x, v_y, orientation, a_x, v_orientation
        r_state = r_state
        # r_pos -> x, y
        r_state_xy = r_pos
        # r_goal
        r_goal = r_goal


        #neigh_hist = np.ones([self.num_agent, 2]) * -1000
        #neigh_hist[0:obs_traj_pos.shape[1]] = obs_traj_pos.squeeze(0)[:, :]

        #ob = neigh_hist
        #print(np.shape(ob))
        ob = obs_traj_pos
        #print(np.shape(ob))
        x = np.concatenate([r_state_xy, r_state[2:]])
        
        #print(r_state[2:])
        u, predicted_trajectory = self.dwa_control(x, self.config, r_goal, ob)
        #print("current robot position: ", r_pos)
        #print("going to: ", u)
        return np.array([u])
        """
        """
        Main function for Dynamic Window Approach that calls other necessary functions to predict trajectories and
        choose the best proposed action.\n
        :param obs_traj_pos: position of all obstacles given as: [[x(m), y(m)], ...]
        :param r_pos_xy: current position of the robot given as: [x(m), y(m)]
        :param r_goal_xy: goal of the robot given as: [x(m), y(m)]
        :param r_vel_state: current velocity and orientation of robot given as: [v_x(m/s), v_y(m/s), theta(rad),
        v_lin(m/s), v_ang(rad/s)]
        :return: best proposed action as a numpy array
        """
        obs_xy_in_fov = obs_traj_pos
        #print("--------")
        #print(obs_xy_in_fov)
        """
        yaw = r_vel_state[2]

        # init obstacles in vicinity with dummy obstacle far away
        obs_xy_in_proximity_list = [[-1000, -1000]]

        # filler array for num_agents
        obs_xy_in_fov = np.ones(obs_traj_pos.shape) * -1000

        # unit vector at origin, with the same direction as the robot
        r_norm = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float)
        for idx, ob in enumerate(obs_traj_pos[-1, :, :]):
            z = r_pos_xy - ob
            x = np.linalg.norm(r_pos_xy - ob)
            y = inFOV(ob, r_pos_xy, r_norm, self.fov)
            # proximity check
            if np.linalg.norm(r_pos_xy - ob) <= self.config.inner_proximity_radius:
                obs_xy_in_proximity_list.append(ob)

            # if pedestrian in fov and is within the outer proximity range
            if inFOV(ob, r_pos_xy, r_norm, self.fov) and np.linalg.norm(r_pos_xy - ob) <= self.config.outer_proximity_radius:
                # copy its data to the new array
                obs_xy_in_fov[:, idx, :] = obs_traj_pos[:, idx, :]
        """
        # array containing obstacles within a radius of the robot(according to LiDAR data)
        # expected size: (1, X, 2), where X is simply the number of obstacles
        # TODO: so far X is assumed to be num_agent, gotta change that(prob. feed obs_xy_in_proximity as a seperate parameter to dwa_control
        #if len(obs_xy_in_proximity_list) > 1:
        #    obs_xy_in_proximity = np.array(obs_xy_in_proximity_list[1:])
        #else:
        #    obs_xy_in_proximity = np.array(obs_xy_in_proximity_list)
        #np.expand_dims(obs_xy_in_proximity, axis=0)

        # filler array for obstacles in proximity(red area - according to LiDAR)
        #neigh_proximity = np.ones([1, self.num_agent, 2]) * -1000

        # copy the obstacles in proximity
        #neigh_proximity[0, 0:obs_xy_in_proximity.shape[0]] = obs_xy_in_proximity

        # if there is no pedestrian in fov, then no need to call the predict function
        if (1): #not (obs_xy_in_fov == -1000).all():
            with torch.no_grad():
                # convert arrays into tensors
                obs_traj_pos_fov = torch.from_numpy(obs_xy_in_fov).to(self.device)
                neigh_index = torch.from_numpy(self.neigh_index).to(self.device)

                # calculation of relative traj from obs_traj_pos(Martin's magical code)
                mask_rel = torch.where(obs_traj_pos_fov != 0, True, False)
                traj_rel = torch.zeros_like(obs_traj_pos_fov)
                mask_rel_first_element = mask_rel.logical_not() * 999.99
                obs_traj_pos_filled = obs_traj_pos_fov + mask_rel_first_element
                traj_rel[1:] = (obs_traj_pos_filled[1:] - obs_traj_pos_filled[:-1])
                traj_rel = torch.where(traj_rel < -300, torch.zeros_like(obs_traj_pos_fov), traj_rel) * mask_rel

                # forward propagation
                pred_traj_rel, _, _ = self.pred_model_ar(traj_rel=traj_rel.float(),
                                                         obs_traj_pos=obs_traj_pos_fov.float(),
                                                         nei_index=neigh_index)

            # conversion from relative to absolute trajectory
            pred_traj_abs = torch.cumsum(pred_traj_rel.cpu(), dim=0) + obs_traj_pos[-1]

            # filler array for prediction
            neigh_predicted = np.ones([self.pred_time_steps + 1, self.num_agent, 2]) * -1000

            # add current positions of pedestrians(green area - in FOV) to the array
            neigh_predicted[0, 0:obs_traj_pos_fov.shape[1]] = obs_traj_pos_fov[-1, :].cpu().numpy()

            # add predicted positions to the array
            neigh_predicted[1:, 0:obs_traj_pos_fov.shape[1]] = pred_traj_abs.cpu().numpy()

            #print(obs_traj_pos)
            # print(traj_rel)
            # print(pred_traj_abs)

            neigh = np.concatenate((obs_lidar, neigh_predicted), axis=0)


        else:
            neigh = obs_lidar
        
      #  print(neigh_predicted)
        all_obs = neigh

        #obs_xy_in_proximity = np.array(obs_xy_in_proximity)
        #rospy.loginfo("no obs. in vicinity: %d", obs_xy_in_proximity.shape[0])

        # current state of the robot given as: [x(m), y(m), theta(rad), v_lin(m/s), v_ang(rad/s)]
        r_curr_state = np.concatenate([r_pos_xy, r_vel_state[2:]])

        # best proposed action(here: u) and predicted trajectory calculated
        best_proposed_action, predicted_trajectory = self.dwa_control(r_curr_state, self.config, r_goal_xy, all_obs, cost_map_obj)

        return np.array(best_proposed_action)
        
        

if __name__ == '__main__':
    robot_config = ConfigRobot(robot_model="locobot", collision_dist=0.2)
    sim_config = ConfigSim(robot_config=robot_config)
    dwa = DWA(sim_config, -1, 30)
