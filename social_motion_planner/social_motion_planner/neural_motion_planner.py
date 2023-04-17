#!/usr/bin/env python3
import rclpy
import numpy as np

# import functionalities from rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
# import message filters functionalities
from message_filters import Subscriber, ApproximateTimeSynchronizer

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from soloco_interfaces.msg import TrackedPersons

from social_motion_planner.models.DWA import DWA
from social_motion_planner.models.CEM_policy_IAR import CEM_IAR

from social_motion_planner.occupancy_grid_manager import OccupancyGridManager

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
        self.declare_parameter('odom_topic', '/locobot/odom')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/locobot/commands/velocity')
        self.declare_parameter('human_track_topic', '/human/tracks')
        # Device to use: 'gpu' or 'cpu'
        self.declare_parameter('device', 'cpu') 
        # Define neural model
        self.declare_parameter('model_name', 'CEM_IAR')        
        # Declare robot parameters
        self.declare_parameter('use_robot_model', True)   # Flag for robot param constrains usage
        self.declare_parameter('max_speed', 0.5)          # [m/s] # 0.5 locobot
        self.declare_parameter('min_speed', -0.1)         # [m/s]
        self.declare_parameter('max_yaw_rate', 1.0)       # [rad/s]
        self.declare_parameter('max_accel', 0.5)         # [m/s^2]
        self.declare_parameter('max_delta_yaw_rate', 3.2) # [rad/s^2]
        self.declare_parameter('collision_dist', 0.2)     # [m]
        # Declare maximum number of agents        
        self.declare_parameter('max_num_agents', 5)     # [maximum number of people]
        # Declare maximum history length
        self.declare_parameter('max_history_length', 7) # [maximum history length]
        self.declare_parameter('interp_interval', 0.4) # [interpolation interval]

    def initialize_node(self):
        # Device to use
        self.device= self.get_parameter('device').get_parameter_value().string_value
        # Initialize robot params
        self.robot_params_dict = {}
        self.robot_params_dict['use_robot_model'] = self.get_parameter('use_robot_model').get_parameter_value().bool_value 
        self.robot_params_dict['max_speed'] = self.get_parameter('max_speed').get_parameter_value().double_value  
        self.robot_params_dict['min_speed'] = self.get_parameter('min_speed').get_parameter_value().double_value 
        self.robot_params_dict['max_yaw_rate'] = self.get_parameter('max_yaw_rate').get_parameter_value().double_value  
        self.robot_params_dict['max_accel'] = self.get_parameter('max_accel').get_parameter_value().double_value  
        self.robot_params_dict['max_delta_yaw_rate'] = self.get_parameter('max_delta_yaw_rate').get_parameter_value().double_value  
        self.robot_params_dict['collision_dist'] = self.get_parameter('collision_dist').get_parameter_value().double_value
        # Initialize maximum number of agents, history length, and interpolation interval
        self.max_num_agents = self.get_parameter('max_num_agents').get_parameter_value().integer_value  
        self.max_history_length = self.get_parameter('max_history_length').get_parameter_value().integer_value  
        self.interp_interval = self.get_parameter('interp_interval').get_parameter_value().double_value
        # Initialize neighboring matrix
        self.neigh_matrix = np.ones((6, 6), int)
        np.fill_diagonal(self.neigh_matrix, 0)
        # Initialize current robot state
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        # Initialize current position of all people and goal pose
        self.ped_pos_xy_cem = np.ones((self.max_history_length + 1, self.max_num_agents + 1, 2)) * 500 # placeholder value
        self.goal_pose = None
        # Initialize model
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.model = self.switch_case_model(model_name)

    def switch_case_model(self, model_name):
        if model_name == 'CEM_IAR':
            return CEM_IAR(self.robot_params_dict, self.interp_interval, hist=self.max_history_length, num_agent=self.max_num_agents, device=self.device)
        else:
            raise Exception('An error occurred')
            
    def setup_publishers_and_subscribers(self):
        # Get ROS parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_pose_topic = self.get_parameter('goal_pose_topic').get_parameter_value().string_value
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, self.pose_qos)
        
        # Subscribers
        self.goal_pose_sub = self.create_subscription(PoseStamped, goal_pose_topic, self.goal_callback, self.pose_qos) 
        # self.human_sub = self.create_subscription(TrackedPersons, human_track_topic, self.human_callback, self.pose_qos)
        self.marker_sub = self.create_subscription(MarkerArray, '/visualization/tracks', self.marker_callback, self.pose_qos)
        
        # Subscriber for synced update
        self.ego_sub = Subscriber(self, Odometry, odom_topic)
        self.costmap_sub = Subscriber(self, OccupancyGrid, costmap_topic)
        
        # Sync messages with slop (max delay) = 0.1 seconds
        time_sync = ApproximateTimeSynchronizer([self.ego_sub, self.costmap_sub], queue_size=10, slop=0.1)
        time_sync.registerCallback(self.common_callback)

    def goal_callback(self, goal_msg):
        goal = [goal_msg.pose.position.x, goal_msg.pose.position.y]

        self.goal_pose = np.array(goal)

    def marker_callback(self, marker_msg):
        self.ped_pos_xy_cem = np.ones((self.max_history_length + 1, self.max_num_agents + 1, 2)) * 99
        for i, ped in enumerate(marker_msg.markers):
            coords_array = np.array([[e.x, e.y] for e in ped.points])
            self.ped_pos_xy_cem[:, i] = coords_array[::-1]

    # def human_callback(self, human_msg):
    #     # update people state
    #     for person_idx, person in enumerate(human_msg.tracks):
    #         for pos_idx, past_pos in enumerate(person.track):
    #             coords_array = np.array([past_pos.position.x, past_pos.position.y])
    #             self.ped_pos_xy_cem[pos_idx, person_idx, :] = coords_array[::-1]

    def common_callback(self, odom_msg, costmap_msg):
        robot_pose = Pose()
        robot_pose = odom_msg.pose.pose

        # update robot state
        self.r_state[0] = 0.0
        self.r_state[1] = 0.0
        self.r_state[2] = odom_msg.pose.pose.orientation.z
        self.r_state[3] = np.sqrt(odom_msg.twist.twist.linear.x**2 + odom_msg.twist.twist.linear.y**2)
        self.r_state[4] = odom_msg.twist.twist.angular.z
        
        # Send costmap to occupancy grid manager
        self.ogm = OccupancyGridManager(costmap_msg, subscribe_to_updates=False)
        
        if self.goal_pose is not None:
            # Concatenate information about people track, robot state, and goal
            x = np.concatenate([self.ped_pos_xy_cem.flatten(), self.neigh_matrix.flatten(), self.r_state, self.goal_pose])
            # Get command from neural model forward pass, given costmap object
            u = self.model.predict(x, costmap_obj=self.ogm)
            # Publish resulting twist to cmd_vel topic
            cmd_vel = Twist()
            cmd_vel.linear.x = float(u[0])
            cmd_vel.angular.z = float(u[1])
            self.cmd_vel_publisher.publish(cmd_vel)

    #         self.visualize_model_plan(mean_robot_action=mean_robot_action)
    
    # def visualize_model_plan(self, robot_pose, mean_robot_action):


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