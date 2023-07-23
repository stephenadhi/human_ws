#!/usr/bin/env python3
import os
import rclpy
from rclpy.action import ActionServer
import numpy as np
from math import hypot
from tf_transformations import euler_from_quaternion
# import functionalities from rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PointStamped, Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from soloco_interfaces.msg import TrackedPersons, EgoTrajectory
from soloco_interfaces.action import NavigateToXYGoal

from nav2_soloco_controller.models.DWA import DWA
from nav2_soloco_controller.models.CEM_policy_IAR import CEM_IAR
from nav2_soloco_controller.models.MPPI_policy_AR import Parallel_MPPI
from nav2_soloco_controller.occupancy_grid_manager import OccupancyGridManager
# from nav2_soloco_controller.utils import point_transform, pose_transform
from launch_ros.substitutions import FindPackageShare

class NeuralMotionPlanner(Node):
    def __init__(self):
        super().__init__('neural_motion_planner')
        self.pkg_name = 'nav2_soloco_controller'
        self.pkg_dir = FindPackageShare(package=self.pkg_name).find(self.pkg_name)
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
        # Setup action server
        self._action_server = ActionServer(
            self,
            NavigateToXYGoal,
            'navigate_to_xy_goal',
            self.execute_callback)
        
    def declare_ros_parameters(self):
        # Declare topic parameters
        self.declare_parameter('odom_topic', 'locobot/odom')
        self.declare_parameter('global_goal_topic', 'goal_pose')
        self.declare_parameter('subgoal_topic', 'subgoal_pose')
        self.declare_parameter('costmap_topic', 'local_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', 'locobot/commands/vedlocity')
        self.declare_parameter('human_track_topic', 'human/interpolated_history')
        self.declare_parameter('robot_track_topic', 'robot/ego_trajectory')      
        self.declare_parameter('future_topic', 'visualization/predicted_future')
        self.declare_parameter('pub_frame_id', 'locobot/odom')
        # Device to use: 'gpu' or 'cpu'
        self.declare_parameter('device', 'cpu') 
        # Define neural model
        self.declare_parameter('model_name', 'MPPI')
        self.declare_parameter('AR_checkpoint', 'models/weights/SIMNoGoal-univ_fast_AR2/checkpoint_with_model.pt')
        self.declare_parameter('IAR_checkpoint', 'models/weights/SIMNoGoal-univ_IAR_Full_trans/checkpoint_with_model.pt')     
        # Declare goal tolerance
        self.declare_parameter('goal_tolerance', 0.5)
        # Declare robot parameters
        self.declare_parameter('use_robot_model', True)   # Flag for robot param constrains usage
        self.declare_parameter('robot_model', 'differential_drive')
        self.declare_parameter('max_speed', 0.5)          # [m/s] # 0.5 locobot
        self.declare_parameter('min_speed', -0.1)         # [m/s]
        self.declare_parameter('max_yaw_rate', 1.0)       # [rad/s]
        self.declare_parameter('max_accel', 0.5)         # [m/s^2]
        self.declare_parameter('max_delta_yaw_rate', 3.2) # [rad/s^2]
        self.declare_parameter('collision_dist', 0.2)     # [m]
        # Declare maximum number of agents    and placeholder value     
        self.declare_parameter('max_num_agents', 5)     # [maximum number of people]
        self.declare_parameter('placeholder_value', 50.0) # [Placeholder value for people distance]
        # Declare maximum history length
        self.declare_parameter('max_history_length', 7) # [maximum history length]
        self.declare_parameter('prediction_steps', 12) # [maximum history length]
        self.declare_parameter('sample_batch', 200) # [maximum history length]
        self.declare_parameter('interp_interval', 0.4) # [interpolation interval]
        self.declare_parameter('controller_frequency', 20.0) # [controller frequency]
        self.declare_parameter('debug_log', False)
        
        self.debug_log = self.get_parameter('debug_log').get_parameter_value().bool_value

    def initialize_node(self):
        # Get ROS parameters
        self.device= self.get_parameter('device').get_parameter_value().string_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.placeholder_value = self.get_parameter('placeholder_value').get_parameter_value().double_value
        # Initialize robot params
        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
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
        self.prediction_steps = self.get_parameter('prediction_steps').get_parameter_value().integer_value
        self.interp_interval = self.get_parameter('interp_interval').get_parameter_value().double_value
        self.sample_batch = self.get_parameter('sample_batch').get_parameter_value().integer_value
        # Initialize neighboring matrix
        self.neigh_matrix = np.ones((self.max_num_agents+1, self.max_num_agents+1), int)
        np.fill_diagonal(self.neigh_matrix, 0)
        # Initialize current robot state
        self.r_pos_xy = np.zeros(2)
        self.robot_yaw = np.zeros(3)
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        # Initialize current position of all people and goal pose
        self.ped_pos_xy_cem = np.ones((self.max_history_length + 1, self.max_num_agents + 1, 2)) * self.placeholder_value
        self.ped_pos_xy_cem[:, 0] = np.ones((self.max_history_length + 1, 2))
        self.new_goal = False
        self.trajectory_received = False
        self.global_goal = None
        self.subgoal_pose = None
        self.ogm = None
        # Initialize model
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.AR_checkpoint = os.path.join(self.pkg_dir, self.pkg_name, self.get_parameter('AR_checkpoint').get_parameter_value().string_value)
        self.IAR_checkpoint = os.path.join(self.pkg_dir, self.pkg_name, self.get_parameter('IAR_checkpoint').get_parameter_value().string_value)
        self.model = self.switch_case_model(model_name)

    def switch_case_model(self, model_name):
        if model_name == 'CEM_IAR':
            self.AR_checkpoint = os.path.join(self.pkg_dir, self.pkg_name, 'models/weights/SIMNoGoal-univ_fast_AR2/checkpoint_with_model.pt')
            self.IAR_checkpoint = os.path.join(self.pkg_dir, self.pkg_name, 'models/weights/SIMNoGoal-univ_IAR_Full_trans/checkpoint_with_model.pt')
            return CEM_IAR(robot_params_dict=self.robot_params_dict, dt=self.interp_interval, sample_batch=self.sample_batch, hist=self.max_history_length, 
                           prediction_steps=self.prediction_steps, num_agent=self.max_num_agents, AR_checkpoint=self.AR_checkpoint,
                           IAR_checkpoint=self.IAR_checkpoint, device=self.device)
        elif model_name == 'MPPI':
            return Parallel_MPPI(robot_params_dict=self.robot_params_dict, dt=self.interp_interval, hist=self.max_history_length,
                           sample_batch_per_thread=self.sample_batch, num_threads=1, predictions_steps=self.prediction_steps, num_agent=self.max_num_agents,
                           device=self.device, human_reaction=False)
        else:
            raise Exception('An error occurred')
            
    def setup_publishers_and_subscribers(self):
        # Get ROS parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        global_goal_topic = self.get_parameter('global_goal_topic').get_parameter_value().string_value
        subgoal_topic = self.get_parameter('subgoal_topic').get_parameter_value().string_value
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value
        robot_track_topic = self.get_parameter('robot_track_topic').get_parameter_value().string_value
        future_topic = self.get_parameter('future_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, self.pose_qos)
        self.agent_future_publisher = self.create_publisher(MarkerArray, future_topic, self.pose_qos)
        # Subscribers
        self.global_goal_sub = self.create_subscription(PoseStamped, global_goal_topic, self.goal_callback, self.pose_qos) 
        self.subgoal_sub = self.create_subscription(PoseStamped, subgoal_topic, self.subgoal_callback, self.pose_qos) 
        self.human_sub = self.create_subscription(TrackedPersons, human_track_topic, self.human_callback, self.pose_qos)
        self.trajectory_sub = self.create_subscription(EgoTrajectory, robot_track_topic, self.trajectory_callback, self.pose_qos)
        
        self.costmap_sub = self.create_subscription(OccupancyGrid, costmap_topic, self.costmap_callback, self.pose_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, self.pose_qos)
        
        # controller_frequency = self.get_parameter('controller_frequency').get_parameter_value().double_value
        # self.timer_period = 1/controller_frequency
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def goal_callback(self, goal_msg):
        self.new_goal = True
        self.get_logger().info(f'New global goal received in {goal_msg.header.frame_id} frame.')
        curr_goal = PoseStamped()
        curr_goal = goal_msg

        ori = curr_goal.pose.orientation
        quat = (ori.x, ori.y, ori.z, ori.w)
        goal_yaw = euler_from_quaternion(quat)
        goal = [curr_goal.pose.position.x, curr_goal.pose.position.y, goal_yaw[2]]
        
        self.global_goal = np.array(goal)

    def subgoal_callback(self, goal_msg):
        ori = goal_msg.pose.orientation
        quat = (ori.x, ori.y, ori.z, ori.w)
        goal_yaw = euler_from_quaternion(quat)
        goal = [goal_msg.pose.position.x, goal_msg.pose.position.y, goal_yaw[2]]
        
        self.subgoal_pose = np.array(goal)

    def trajectory_callback(self, robot_msg):
        # self.get_logger().info('Robot trajectory received.')
        # Update robot state (Robot ID: 0)
        coords_array = np.array([[e.pose.position.x, e.pose.position.y] for e in robot_msg.track.poses])
        # Add small noise if odometry is giving zeros
        if coords_array.sum() == 0:
            coords_array = np.random.randn(self.max_history_length+1, 2)*0.01
        self.ped_pos_xy_cem[:, 0] = coords_array[::-1]
        self.trajectory_received = True

    '''update people state (ID 0 is reserved for robot)'''
    def human_callback(self, human_msg):
        # self.get_logger().info('Human trajectory received.')
        # Reset human data with placeholder value
        self.ped_pos_xy_cem[:, 1:] = np.ones((self.max_history_length + 1, self.max_num_agents, 2)) * self.placeholder_value
        if human_msg.tracks:
            for person_idx, person in enumerate(human_msg.tracks):
                coords_array = np.array([[e.pose.position.x, e.pose.position.y] for e in person.track.poses])
                self.ped_pos_xy_cem[:, person_idx+1] = coords_array[::-1]

    def costmap_callback(self, costmap_msg):
        # Send costmap to occupancy grid manager
        self.ogm = OccupancyGridManager(costmap_msg)

    def odom_callback(self, odom_msg):
        # update robot states
        self.r_pos_xy[0] = odom_msg.pose.pose.position.x
        self.r_pos_xy[1] = odom_msg.pose.pose.position.y
        ori = odom_msg.pose.pose.orientation
        quat = (ori.x, ori.y, ori.z, ori.w)                
        self.robot_yaw = euler_from_quaternion(quat)

        self.r_state[0] = 0.0
        self.r_state[1] = 0.0
        self.r_state[2] = self.robot_yaw[2]
        self.r_state[3] = hypot(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)
        self.r_state[4] = odom_msg.twist.twist.angular.z

    def execute_callback(self, goal_handle):
        if self.global_goal is not None and self.subgoal_pose is not None and self.ogm is not None and self.trajectory_received:
            # Calculate euclidian distance to goal
            distance_to_goal = hypot((self.r_pos_xy[0]-self.global_goal[0]), 
                                 (self.r_pos_xy[1]-self.global_goal[1]))

            # Concatenate information about people track, robot state, and goal
            x = np.concatenate([self.ped_pos_xy_cem.flatten(), self.neigh_matrix.flatten(), self.r_state, self.subgoal_pose[:2]])
            # Get command from neural model forward pass, given costmap object
            u, current_future = self.model.predict(x, costmap_obj=self.ogm)
            
            # Publish resulting twist to cmd_vel topic
            cmd_vel = Twist()
            if distance_to_goal > self.goal_tolerance:
                cmd_vel.linear.x = float(u[0])
                cmd_vel.angular.z = float(u[1])
                self.cmd_vel_publisher.publish(cmd_vel)
                if self.debug_log:
                    self.get_logger().info(f'Heading towards x:{self.global_goal[0]}, y: {self.global_goal[1]}')
                    self.get_logger().info(f'Navigating with velocity linear: {u[0]} and angular {u[1]}.')          
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_vel)
                self.get_logger().info(f'Distance to goal: {distance_to_goal} Goal pose achieved.')
                self.global_goal = None

            self.visualize_future(current_future)
        
        
        goal_handle.succeed()
        
        result = NavigateToXYGoal.Result()
        result.command_velocity = cmd_vel

        return result

    def visualize_future(self, current_future):
        agent_marker = MarkerArray()
        for i, track in enumerate(current_future):
            # Create a Marker message
            marker = Marker()
            marker.id = i + 100
            marker.header.frame_id = self.pub_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.lifetime = Duration(seconds=0.05).to_msg()
            marker.scale.x = 0.05
            if i == 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0
            # Set the pose of the Marker message
            marker.pose.orientation.w = 1.0

            # Get the last observed position for the current agent
            last_observed_pos = self.ped_pos_xy_cem[-1, i]
            last_point = Point()
            last_point.x = float(last_observed_pos[0])
            last_point.y = float(last_observed_pos[1])
            track_points = [last_point]
            
            # Append future
            for point in track:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                track_points.append(p)
            marker.points = track_points
            agent_marker.markers.append(marker)

        self.agent_future_publisher.publish(agent_marker) 


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