"""
    Configurations File
"""
from enum import Enum


# simple class for robot types
class RobotType(Enum):
    circle = 0
    rectangle = 1


class ConfigRobot:
    """
    Class to configure the attributes of the robot for DWA. The attributes are contained as a dictionary.
    """
    def __init__(self, robot_model: str = "locobot", collision_dist: float = 0.35, robot_params: dict = None):
        """
        Simple constructor to initialize the configuration of the robot.\n
        :param robot_model: name of the robot model
        :param collision_dist: baseline distance to obstacles that the robot should always avoid. given in meters
        :param robot_params: dictionary containing robot's capabilities such as max/min-speed and max-accel for angular and linear velocity
        """
        if robot_model == "locobot":
            self.robot_params = {'max_linear_vel': .5,  # [m/s]
                                 'min_linear_vel': -.1,  # [m/s]
                                 'max_linear_acc': .5,  # [m/ss]
                                 'max_angular_vel': 1.0,  # [rad/s]
                                 'max_angular_acc': 3.2,  # [rad/ss] / collision_dist: [m]
                                 'use_robot_model': True,
                                 'collision_dist': collision_dist} if robot_params is None else robot_params

        else:
            raise ValueError("There is no other robot we are working with, just use locobot dummy")


class ConfigSim:
    """
    Class to configure the simulation. Contains cost gains and time tick for motion prediction. Uses configuration from
    ConfigRobot class.
    """
    def __init__(self, robot_config: ConfigRobot):
        """
        Simple constructor to initialize the configuration of the simulation.\n
        :param robot_config: ConfigRobot object containing parameters of
        """
        # robot parameters
        self.robot_model = robot_config.robot_params
        self.max_speed = self.robot_model['max_linear_vel']  # [m/s]
        self.min_speed = self.robot_model['min_linear_vel']  # [m/s]
        self.max_accel = self.robot_model['max_linear_acc']  # [m/ss]
        self.max_yaw_rate = self.robot_model['max_angular_vel']  # [rad/s]
        self.max_delta_yaw_rate = self.robot_model['max_angular_acc']  # [rad/ss]
        
        
        # the radius, in which we consider obstacles
        self.inner_proximity_radius = 2
        self.outer_proximity_radius = 10000

        # TODO: move below parameters somewhere else(e.g. class called Locobot with all these stuff)
        # robot attributes
        self.robot_type = RobotType.circle
        self.robot_radius = 0.35  # [m] for collision check
        self.robot_width = 0.35  # [m] for collision check
        self.robot_length = 0.35  # [m] for collision check

        # simulation params
        self.v_resolution = 0.025  # [m/s]
        self.yaw_rate_resolution = 0.025  # sim_granularity like in move_base
        self.dt = 0.4  # [s] Time tick for motion prediction
        self.pred_time_steps = 2
        self.predict_time = self.pred_time_steps * 0.4  # 1.7s for move_base we tested 4 * 0.4, but best results with 2 * 0.4
        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 4
        self.obstacle_cost_gain = 3
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stuck

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value
