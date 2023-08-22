#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import subprocess

class RosbagRecorderNode(Node):

    def __init__(self):
        super().__init__('rosbag_recorder_node')

        # Parameters
        self.declare_parameter('goal_tolerance', 0.2)
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # QoS profile for subscription
        qos_profile = QoSProfile(depth=10)

        # Subscribe to the odom topic
        self.odom_sub = self.create_subscription(
            Odometry,
            'locobot/odom',
            self.odom_callback,
            qos_profile)

        # Subscribe to the goal topic
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            1)

        self.robot_goal = None
        self.is_recording = False
        self.process = None
        self.timer = None

    def odom_callback(self, msg):
        if self.robot_goal:
            distance_to_goal = math.hypot(
                msg.pose.pose.position.x - self.robot_goal.pose.position.x,
                msg.pose.pose.position.y - self.robot_goal.pose.position.y)
            if distance_to_goal <= self.goal_tolerance:
                self.stop_recording()

    def goal_callback(self, msg):
        self.robot_goal = msg
        self.start_recording()

    def start_recording(self):
        if not self.is_recording:
            topics = [
                "/locobot/commands/velocity",
                "/locobot/odom",
                "/map",
                "/local_costmap/costmap",
                "/global_costmap/costmap",
                "/tf",
                "/tf_static",
                "/plan",
                "/visualization/predicted_future",
                "/visualization/human_tracks",
                "/visualization/robot_track",
                "/visualization/subgoal",
                "/visualization/human_bounding_boxes",
                "/goal_pose",
                "/locobot/robot_description",
                "/zed2/zed_node/left_raw/image_raw_color"
            ]
            cmd = ['ros2', 'bag', 'record'] + topics
            self.process = subprocess.Popen(cmd)
            self.is_recording = True
            self.get_logger().info('Started recording rosbag')
            self.timer = self.create_timer(120, self.stop_recording)  # Stop recording after 2 minutes

    def stop_recording(self):
        if self.is_recording and self.process:
            self.process.terminate()
            self.is_recording = False
            self.get_logger().info('Stopped recording rosbag')
            if self.timer:
                self.timer.cancel()  # Cancel the timer if goal is reached before 2 minutes

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
