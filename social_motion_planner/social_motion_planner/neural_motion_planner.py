#!/usr/bin/env python3
import os
import rclpy
from rclpy.time import Time
import numpy as np
from math import hypot
from tf_transformations import euler_from_quaternion
# import functionalities from rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.duration import Duration
# import message filters functionalities
from message_filters import Subscriber, ApproximateTimeSynchronizer

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from soloco_interfaces.msg import TrackedPersons

from social_motion_planner.models.DWA import DWA
from social_motion_planner.models.CEM_policy_IAR import CEM_IAR
from social_motion_planner.occupancy_grid_manager import OccupancyGridManager
# from social_motion_planner.utils import point_transform, pose_transform
from launch_ros.substitutions import FindPackageShare

from tf2_geometry_msgs import do_transform_pose, do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException

class NeuralMotionPlanner(Node):
    def __init__(self):
        super().__init__('neural_motion_planner')
        self.pkg_name = 'social_motion_planner'
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
        
    def declare_ros_parameters(self):
        # Declare topic parameters
        self.declare_parameter('odom_topic', '/locobot/odom')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('subgoal_topic', '/subgoal_pose')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/locobot/commands/vedlocity')
        self.declare_parameter('human_track_topic', '/human/tracks')
        self.declare_parameter('tracks_marker_topic', '/visualization/tracks')       
        self.declare_parameter('future_topic', '/visualization/predicted_future')
        # Device to use: 'gpu' or 'cpu'
        self.declare_parameter('device', 'cpu') 
        # Define neural model
        self.declare_parameter('model_name', 'CEM_IAR')
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
        # Declare maximum number of agents        
        self.declare_parameter('max_num_agents', 5)     # [maximum number of people]
        # Declare maximum history length
        self.declare_parameter('max_history_length', 7) # [maximum history length]
        self.declare_parameter('interp_interval', 0.4) # [interpolation interval]

        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

    def initialize_node(self):
        # Device to use
        self.device= self.get_parameter('device').get_parameter_value().string_value
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
        self.interp_interval = self.get_parameter('interp_interval').get_parameter_value().double_value
        # Initialize neighboring matrix
        self.neigh_matrix = np.ones((6, 6), int)
        np.fill_diagonal(self.neigh_matrix, 0)
        # Initialize current robot state
        self.r_state = np.zeros(5)  # v_x, v_y, yaw, v_t, omega_t
        # Initialize current position of all people and goal pose
        self.ped_pos_xy_cem = np.ones((self.max_history_length + 1, self.max_num_agents + 1, 2)) * 500 # placeholder value
        self.new_goal = False
        self.global_goal = None
        self.goal_pose = None
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.ogm = None
        # Initialize model
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.AR_checkpoint = os.path.join(self.pkg_dir, self.pkg_name, self.get_parameter('AR_checkpoint').get_parameter_value().string_value)
        self.IAR_checkpoint = os.path.join(self.pkg_dir, self.pkg_name, self.get_parameter('IAR_checkpoint').get_parameter_value().string_value)
        self.model = self.switch_case_model(model_name)

    def switch_case_model(self, model_name):
        if model_name == 'CEM_IAR':
            return CEM_IAR(self.robot_params_dict, self.interp_interval, hist=self.max_history_length, 
                           num_agent=self.max_num_agents, AR_checkpoint=self.AR_checkpoint,
                           IAR_checkpoint=self.IAR_checkpoint, device=self.device)
        else:
            raise Exception('An error occurred')
            
    def setup_publishers_and_subscribers(self):
        # Get ROS parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_pose_topic = self.get_parameter('goal_pose_topic').get_parameter_value().string_value
        subgoal_topic = self.get_parameter('subgoal_topic').get_parameter_value().string_value
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value
        tracks_marker_topic = self.get_parameter('tracks_marker_topic').get_parameter_value().string_value
        future_topic = self.get_parameter('future_topic').get_parameter_value().string_value

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, self.pose_qos)
        self.agent_future_publisher = self.create_publisher(MarkerArray, future_topic, self.pose_qos)
        # Subscribers
        self.goal_pose_sub = self.create_subscription(PoseStamped, goal_pose_topic, self.goal_callback, self.pose_qos) 
        self.subgoal_sub = self.create_subscription(PoseStamped, subgoal_topic, self.subgoal_callback, self.pose_qos) 
        # self.human_sub = self.create_subscription(TrackedPersons, human_track_topic, self.human_callback, self.pose_qos)
        self.marker_sub = self.create_subscription(MarkerArray, tracks_marker_topic, self.marker_callback, self.pose_qos)
        
        self.costmap_sub = self.create_subscription(OccupancyGrid, costmap_topic, self.costmap_callback, self.pose_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, self.pose_qos)
        # Subscriber for synced update
        # self.ego_sub = Subscriber(self, Odometry, odom_topic)
        # self.costmap_sub = Subscriber(self, OccupancyGrid, costmap_topic)
        
        # Sync messages with slop (max delay) = 0.1 seconds
        # time_sync = ApproximateTimeSynchronizer([self.ego_sub, self.costmap_sub], queue_size=10, slop=0.1)
        # time_sync.registerCallback(self.common_callback)

    def goal_callback(self, goal_msg):
        self.new_goal = True
        self.get_logger().info(f'New global goal received in {goal_msg.header.frame_id} frame')
        curr_goal = PoseStamped()
        curr_goal.pose = self.pose_transform(goal_msg.pose, "locobot/odom", goal_msg.header.frame_id)
        curr_goal = goal_msg
        if curr_goal.pose:
            self.get_logger().info('Goal transformed')
            curr_goal.header.frame_id = 'locobot/odom'
            ori = curr_goal.pose.orientation
            quat = (ori.x, ori.y, ori.z, ori.w)
            goal_yaw = euler_from_quaternion(quat)
            goal = [curr_goal.pose.position.x, curr_goal.pose.position.y, goal_yaw[2]]
            self.global_goal = np.array(goal)
            self.get_logger().info(f'Heading towards x:{goal[0]}, y: {goal[1]}')

    def subgoal_callback(self, goal_msg):
        ori = goal_msg.pose.orientation
        quat = (ori.x, ori.y, ori.z, ori.w)
        goal_yaw = euler_from_quaternion(quat)
        goal = [goal_msg.pose.position.x, goal_msg.pose.position.y, goal_yaw[2]]
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

    def costmap_callback(self, costmap_msg):
        # Send costmap to occupancy grid manager
        self.ogm = OccupancyGridManager(costmap_msg)

    def odom_callback(self, odom_msg):
        ori = odom_msg.pose.pose.orientation
        quat = (ori.x, ori.y, ori.z, ori.w)                
        robot_yaw = euler_from_quaternion(quat)
        self.r_pos_xy = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y]
        # update robot state
        self.r_state[0] = 0.0
        self.r_state[1] = 0.0
        self.r_state[2] = robot_yaw[2]
        self.r_state[3] = hypot(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y)
        self.r_state[4] = odom_msg.twist.twist.angular.z

        if self.global_goal is not None and self.ogm is not None:
            if self.new_goal == True and self.robot_model == 'differential_drive':
                yaw_diff = self.global_goal[2] - robot_yaw[2]
                self.spin_robot_in_subgoal_direction(yaw_diff)
            # Calculate euclidian distance to subgoal

            distance_to_goal = hypot((odom_msg.pose.pose.position.x-self.global_goal[0]), 
                                 (odom_msg.pose.pose.position.y-self.global_goal[1]))
            # Concatenate information about people track, robot state, and goal
            x = np.concatenate([self.ped_pos_xy_cem.flatten(), self.neigh_matrix.flatten(), self.r_state, self.global_goal[:2]])
            # Get command from neural model forward pass, given costmap object
            u, current_future = self.model.predict(x, costmap_obj=self.ogm)
            cmd_vel = Twist()
            if distance_to_goal > self.goal_tolerance:
                # Publish resulting twist to cmd_vel topic
                cmd_vel.linear.x = float(u[0])
                cmd_vel.angular.z = float(u[1])
                self.cmd_vel_publisher.publish(cmd_vel)            
            else:
                # Publish resulting twist to cmd_vel topic
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_vel)
                self.get_logger().info(f'Distance to goal: {distance_to_goal} Goal pose achieved.')
                self.global_goal = None

            self.visualize_future(current_future)
    
    def spin_robot_in_subgoal_direction(self, yaw_diff):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.8 if yaw_diff > 0 else -0.8
        self.cmd_vel_publisher.publish(cmd_vel)
        if abs(yaw_diff) < 0.2:
            self.new_goal = False
            self.get_logger().info("Spin in goal direction completed.")

    def pose_transform(self, curr_pose, output_frame, input_frame):
        transformed_pose = Pose()
        success = False
        while not success:
            try:
                tf = self.tf_buffer.lookup_transform(output_frame, input_frame, Time())
                transformed_pose = do_transform_pose(curr_pose, tf)
            except TransformException as ex:
                self.get_logger().warning(f"Failed to transform: '{ex}'.")
            if transformed_pose: success = True
        
        return transformed_pose

    def point_transform(self, curr_point, output_frame, input_frame):
        transformed_point = Point()
        try:
            tf = self.tf_buffer.lookup_transform(output_frame, input_frame, Time())
            transformed_point = do_transform_point(curr_point, tf)
        except TransformException as ex:
            self.get_logger().warning(f"Failed to transform: '{ex}'.")
            
        return transformed_point if transformed_point else None

    def visualize_future(self, current_future):
        agent_marker = MarkerArray()
        for i, track in enumerate(current_future):
            # Create a Marker message
            marker = Marker()
            marker.id = i + 100
            marker.header.frame_id = "locobot/odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.lifetime = Duration(seconds=1.0).to_msg()
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
            track_points = []
            for point in track:
                p = PointStamped()
                p.point.x = float(point[0])
                p.point.y = float(point[1])
                pos = self.point_transform(p, "map", "locobot/odom")
                track_points.append(p.point)
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