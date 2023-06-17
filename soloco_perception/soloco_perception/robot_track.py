#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from soloco_interfaces.msg import EgoTrajectory

from tf2_ros import Buffer, TransformListener

from soloco_perception.utils import interpolatedTracklet, pose_transform

class RobotTrackPublisher(Node):
    def __init__(self):
        super().__init__('robot_track_publisher')
        # QoS settings
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # Declare parameters
        self.declare_parameter('robot_odom_topic', 'locobot/odom')
        self.declare_parameter('robot_track_topic', 'robot/ego_trajectory')
        self.declare_parameter('pub_frame_id', 'locobot/odom')
        self.declare_parameter('interp_interval', 0.4)
        self.declare_parameter('delay_tolerance', 2.0)
        self.declare_parameter('max_history_length', 7)

        # Get parameter values
        robot_odom_topic = self.get_parameter('robot_odom_topic').get_parameter_value().string_value
        robot_track_topic = self.get_parameter('robot_track_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        self.interp_interval = self.get_parameter("interp_interval").get_parameter_value().double_value
        self.delay_tolerance = self.get_parameter("delay_tolerance").get_parameter_value().double_value
        self.max_history_length = self.get_parameter("max_history_length").get_parameter_value().integer_value 
        
        # Create subscriber
        self.odom_sub = self.create_subscription(Odometry, robot_odom_topic, self.odom_callback, qos)
        
        # Create publisher
        self.robot_interpolated_track_pub = self.create_publisher(EgoTrajectory, robot_track_topic, qos)

        # Initialize ego trajectory to zeros
        self.init_track_to_zeros()
        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

    def init_track_to_zeros(self):
        # Initialize robot trajectory to zeros
        curr_time_ = self.get_clock().now()
        pose_xy = PoseStamped()
        pose_xy.pose.position.x = 0.0
        pose_xy.pose.position.y = 0.0
        pose_xy.header.frame_id = self.pub_frame_id
        pose_xy.header.stamp = curr_time_.to_msg()
        self.ego_trajectory = EgoTrajectory()
        for i in range(self.max_history_length + 1):
            self.ego_trajectory.track.poses.append(pose_xy)
        self.interpolated_tracklet = interpolatedTracklet(
            pose_xy, curr_time_, self.max_history_length, self.interp_interval)
        
    def odom_callback(self, msg):
        # Get current robot pose in publishing frame (default: 'odom')
        robot_in_pub_frame = PoseStamped()
        robot_in_pub_frame.header.stamp = msg.header.stamp
        if msg.header.frame_id != self.pub_frame_id:
            robot_in_pub_frame.pose = pose_transform(
                msg.pose.pose, self.pub_frame_id, msg.header.frame_id, self.tf_buffer)
        else:
            robot_in_pub_frame.pose = msg.pose.pose
        curr_time_ = self.get_clock().now()
        # Add odom point to track history
        self.interpolated_tracklet.add_interpolated_point(robot_in_pub_frame, curr_time_)
        self.update_robot_track()
        # Publish robot track
        self.robot_interpolated_track_pub.publish(self.ego_trajectory)

    def update_robot_track(self):
        for i in range(self.max_history_length + 1):
            pose_xy = PoseStamped()
            pose_xy.header.frame_id = self.pub_frame_id
            pose_xy.header.stamp.sec = int(self.interp_interval * i * 1000) # milliseconds
            pose_xy.pose.position.x = self.interpolated_tracklet.arr_interp_padded[i, 0]
            pose_xy.pose.position.y = self.interpolated_tracklet.arr_interp_padded[i, 1]
            self.ego_trajectory.track.poses[i] = pose_xy

        self.ego_trajectory.track.header.stamp = self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    robot_track_publisher = RobotTrackPublisher()
    rclpy.spin(robot_track_publisher)
    robot_track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()