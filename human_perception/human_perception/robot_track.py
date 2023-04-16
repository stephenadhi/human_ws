#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker

from collections import deque
import numpy as np

from human_perception.utils import interpolatedTracklet, pose_transform

class RobotTrackPublisher(Node):
    def __init__(self):
        super().__init__('robot_track_publisher')
        # QoS settings
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # Declare parameters
        self.declare_parameter('robot_odom_topic', '/zed2/zed_node/odom')
        self.declare_parameter('pose_marker_topic', '/visualization/robot_track')
        self.declare_parameter('bounding_box_topic', '/visualization/bounding_boxes')
        self.declare_parameter('pub_frame_id', 'map')
        self.declare_parameter('pub_frame_rate', 15.0)
        self.declare_parameter('interp_interval', 0.4)
        self.declare_parameter('delay_tolerance', 2.0)
        self.declare_parameter('max_history_length', 7)

        # Get parameter values
        robot_odom_topic = self.get_parameter('robot_odom_topic').get_parameter_value().string_value
        pose_marker_topic = self.get_parameter('pose_marker_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        self.pub_frame_rate = self.get_parameter("pub_frame_rate").get_parameter_value().integer_value
        self.interp_interval = self.get_parameter("interp_interval").get_parameter_value().double_value
        self.delay_tolerance = self.get_parameter("delay_tolerance").get_parameter_value().double_value
        self.max_history_length = self.get_parameter("max_history_length").get_parameter_value().integer_value 
        # Create subscribers
        self.odom_sub = self.create_subscription(Odometry, robot_odom_topic, self.odom_callback, qos)
        self.robot_track = Robot_track(self.max_history_length)

    def odom_callback(self, msg):
        # Get current robot pose in publishing frame (default: 'map')
        robot_in_map_frame = PoseStamped()
        robot_in_map_frame.pose = self.pose_transform(msg.pose.pose, self.pub_frame_id, msg.header.frame_id)
        robot_in_map_frame.header.stamp = msg.header.stamp

        if robot_in_map_frame.pose: # if transform exists, concatenate robot with people and publish
            self.robot_track.interpolated_pose(robot_in_map_frame)

class Robot_track:
    def __init__(self, max_history_length):
        self.max_history_length = max_history_length
        self.odoms = deque(maxlen=90)  # from odom data, not PoseStamped
        self.points_interp = deque(maxlen=self.max_history_length)
        self.interp_point = 0.4
        self.curr_time = 0.
        self.nano_factor = 10 ** 9
        self.arr_interp_padded = np.zeros([self.max_history_length + 1, 2])
        self.marker = Marker()
        self.marker.ns = "tracks"
        self.marker.id = 0
        self.marker.header.frame_id = "map"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.color.a = 1.0
        self.marker.color.g = 1.0
        self.marker.scale.x = 0.05
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.lifetime = Duration(seconds=0.5).to_msg()

    def interpolated_pose(self, pose_):
        self.curr_time = Time.from_msg(pose_.header.stamp)
        self.marker.header.stamp = pose_.header.stamp

        self.odoms.append([pose_.pose.position.x, pose_.pose.position.y, self.curr_time.nanoseconds/self.nano_factor])
       # self.odoms.append([pose_.twist.twist.linear.x, pose_.twist.twist.linear.y, self.curr_time.nanoseconds/self.nano_factor])

        num_t_quan_steps = int((self.curr_time.nanoseconds/self.nano_factor - self.odoms[0][2]) / self.interp_point)
        num_t_quan_steps = self.max_history_length if num_t_quan_steps > self.max_history_length else num_t_quan_steps
        
        np_odoms = np.array(self.odoms)
        inter_time_points = self.curr_time.nanoseconds/self.nano_factor - np.arange(num_t_quan_steps + 1) * self.interp_point
        x_interp = np.interp(inter_time_points, np_odoms[:, 2], np_odoms[:, 0])
        y_interp = np.interp(inter_time_points, np_odoms[:, 2], np_odoms[:, 1])

        self.arr_interp_padded[0:num_t_quan_steps + 1, 0] = x_interp
        self.arr_interp_padded[0:num_t_quan_steps + 1, 1] = y_interp

        if num_t_quan_steps != self.max_history_length:
            self.arr_interp_padded[num_t_quan_steps +1:, 0] = x_interp[-1]
            self.arr_interp_padded[num_t_quan_steps +1:, 1] = y_interp[-1]

        current_points = []
        for i in range(self.max_history_length + 1):
            point = Point()
            point.x = self.arr_interp_padded[i, 0]
            point.y = self.arr_interp_padded[i, 1]
            current_points.append(point)

        self.marker.points = current_points

def main(args=None):
    rclpy.init(args=args)
    robot_track_publisher = RobotTrackPublisher()
    rclpy.spin(robot_track_publisher)
    robot_track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()