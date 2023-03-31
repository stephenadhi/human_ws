#!/usr/bin/env python3
import rclpy
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2

# import functionalities from rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
# import message filters functionalities
from message_filters import Subscriber, ApproximateTimeSynchronizer

# import to find the share folder
from launch_ros.substitutions import FindPackageShare

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Goal
from visualization_msgs.msg import MarkerArray

from models.config import ConfigRobot, ConfigSim
from models.DWA import DWA
from models.uti
from models.CEM_policy_IAR import CEM_IAR
from datetime import datetime, timedelta

from occupancy_grid_manager import OccupancyGridManager
from ego_tracker import EgoTrack

class NeuralMotionPlanner(Node):
    def __init__(self):
        super().__init__('neural_motion_planner')
        # define quality of service
        self.pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        # Declare ROS parameters
        self.declare_ros_parameters()
        # Initialize node parameters
        self.initialize_node()
        # Setup publishers and subscribers
        self.setup_publishers_and_subscribers()
        
    def declare_ros_parameters(self):
        # Declare topic parameters
        self.declare_parameter('odom_topic', '/zed2/zed_node/odom')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('interpolated_hist_topic', '/locobot/interpolated_history')
        # Device to use: 'gpu' or 'cpu'
        self.declare_parameter('device', 'gpu') 
        # Define neural model
        self.declare_parameter('model_name', 'CEM_IAR')        
        # Declare robot parameters
        self.declare_parameter('use_robot_model', 'True')   # Flag for robot param constrains usage
        self.declare_parameter('max_speed', '0.5')          # [m/s] # 0.5 locobot
        self.declare_parameter('min_speed', '-0.1')         # [m/s]
        self.declare_parameter('max_yaw_rate', '1.0')       # [rad/s]
        self.declare_parameter('max_accel', '-0.5')         # [m/ss]
        self.declare_parameter('max_delta_yaw_rate', '3.2') # [rad/ss]
        self.declare_parameter('max_accel', '-0.5')         # [m]
        self.declare_parameter('collision_dist', '0.2')     # [m]
        # Declare maximum number of agents        
        self.declare_parameter('max_num_agents', '5')     # [maximum number of pedestrians]
        # Declare maximum history length
        self.declare_parameter('max_history_length', '12')     # [maximum number of pedestrians]
    
    def initialize_node(self):
        # Device to use
        self.device= self.get_parameter('device').get_parameter_value().string_value
        # Initialize model
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.model = self.switch_case_model(model_name)
        # Initialize robot params
        self.robot_params_dict = {}
        self.robot_params_dict['use_robot_model'] = self.get_parameter('use_robot_model').get_parameter_value().bool_value 
        self.robot_params_dict['max_speed'] = self.get_parameter('max_speed').get_parameter_value().double_value  
        self.robot_params_dict['min_speed'] = self.get_parameter('min_speed').get_parameter_value().double_value 
        self.robot_params_dict['max_yaw_rate'] = self.get_parameter('max_yaw_rate').get_parameter_value().double_value  
        self.robot_params_dict['max_accel'] = self.get_parameter('max_accel').get_parameter_value().double_value  
        self.robot_params_dict['max_delta_yaw_rate'] = self.get_parameter('max_delta_yaw_rate').get_parameter_value().double_value  
        self.robot_params_dict['collision_dist'] = self.get_parameter('collision_dist').get_parameter_value().double_value  
        # Initialize maximum number of agents
        self.max_num_agents = self.get_parameter('max_num_agents').get_parameter_value().interger_value  
        # Initialize maximum history length
        self.max_history_length = self.get_parameter('max_history_length').get_parameter_value().interger_value  
        # Initialize neighboring matrix
        self.neigh_matrix = np.ones((6, 6), int)
        np.fill_diagonal(self.neigh_matrix, 0)
        # Initialize current robot state
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        # Initialize current position of all pedestrians and goal pose
        self.ped_pos_xy_cem = np.ones((self.max_history_length + 1, self.max_num_agents + 1, 2)) #* (500)
        self.goal_pose = None
        self.ego_tracker_obj = None
        
    def switch_case_model(self, model_name):
        if model_name == 'CEM_IAR':
            return CEM_IAR(self.robot_params_dict, 0.4, hist=self.max_history_length, max_num_agents=5, device=self.device)
        else:
            raise Exception('An error occurred')
            
     def setup_publishers_and_subscribers(self):
        # Get ROS parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_pose_topic = self.get_parameter('goal_pose_topic').get_parameter_value().string_value
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        interpolated_hist_topic = self.get_parameter('interpolated_hist_topic').get_parameter_value().string_value
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, self.pose_qos)
        self.npArray_markerarray_publisher = self.create_publisher(interpolated_hist_topic, MarkerArray, self.pose_qos)
        # Subscriber for goal pose
        self.goal_pose_sub = self.create_subscription(Goal, goal_topic, self.goal_callback, self.pose_qos)
        # Subscriber for synced update
        self.ego_sub = Subscriber(self, Odometry, odom_topic)
        self.human_sub = Subscriber(self, TrackedPersons, human_topic)
        self.costmap_sub = Subscriber(self, OccupancyGrid, costmap_topic)

        # Sync messages with slop (max delay) = 0.1 seconds
        time_sync = ApproximateTimeSynchronizer([self.ego_sub, self.human_sub, self.costmap_sub, self.goal_pose_sub], queue_size=10, slop=0.1)
        time_sync.registerCallback(self.common_callback)

    def goal_callback(self, goal_msg):
        goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]
        self.goal_pose = np.array(goal)
        
    def common_callback(self, odom_msg, human_msg, costmap_msg):
            # update robot state
            self.r_state[0] = odom_msg.twist.twist.linear.x
            self.r_state[1] = odom_msg.twist.twist.linear.y
            self.r_state[2] = np.arctan2(odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.x)
            self.r_state[3] = np.sqrt(odom_msg.twist.twist.linear.x**2 + odom_msg.twist.twist.linear.y**2)
            self.r_state[4] = odom_msg.twist.twist.angular.z
            # Update robot track
            self.ego_tracker_obj = EgoTrack(robot_msg)
            self.ego_tracker_obj.interpolated_poses_cb(robot_msg)
            self.npArray_markerarray_publisher.publish([self.ego_tracker_obj.marker])
            # update pedestrians state
            interpoints = self.ego_tracker_obj.arr_interp_padded
            self.ped_pos_xy_cem[:, 0] = interpoints[::-1] 
            # Send costmap to occupancy grid manager
            self.ogm = OccupancyGridManager(costmap_msg, subscribe_to_updates=False)
            # Concatenate information about pedestrian track, robot state, and goal
            x = np.concatenate([self.ped_pos_xy_cem.flatten(), self.neigh_matrix.flatten(), self.r_state, self.goal_pose])
            
            if self.goal_pose is not None:
                # Get command from neural model forward pass, given costmap object
                u = self.model.predict(x, costmap_obj=self.ogm)
                # Publish resulting twist to cmd_vel topic
                cmd_vel = Twist()
                cmd_vel.linear.x = u[0]
                cmd_vel.angular.z = u[1]
                self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    try:
        rclpy.init(args=args)

        neural_motion_planner = NeuralMotionPlanner()
        rclpy.spin(neural_motion_planner)

        neural_motion_planner.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()