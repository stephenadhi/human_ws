#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from soloco_interfaces.msg import EgoTrajectory
from soloco_interfaces.msg import TrackedPersons

class MultiTrackVisualizer(Node):
    def __init__(self):
        super().__init__('multi_track_visualizer')
        # QoS settings
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # Declare parameters
        self.declare_parameter('robot_odom_topic', 'locobot/odom')
        self.declare_parameter('robot_track_topic', 'robot/ego_trajectory')
        self.declare_parameter('robot_marker_topic', 'visualization/robot_track')
        self.declare_parameter('human_track_topic', 'human/interpolated_history')
        self.declare_parameter('pose_marker_topic', 'visualization/human_tracks')
        self.declare_parameter('bounding_box_topic', 'visualization/human_bounding_boxes')
        self.declare_parameter('pub_frame_id', 'locobot/odom')
        self.declare_parameter('visualize_bbox', True)

        # Get parameter values
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value
        pose_marker_topic = self.get_parameter('pose_marker_topic').get_parameter_value().string_value
        bounding_box_topic = self.get_parameter('bounding_box_topic').get_parameter_value().string_value
        robot_track_topic = self.get_parameter('robot_track_topic').get_parameter_value().string_value
        robot_marker_topic = self.get_parameter('robot_marker_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value

        self.visualize_bbox = self.get_parameter("visualize_bbox").get_parameter_value().bool_value    

        # Create subscribers
        self.robot_interpolated_track_sub = self.create_subscription(EgoTrajectory, robot_track_topic, self.egotraj_cb, qos)
        self.human_track_interpolated_sub = self.create_subscription(TrackedPersons, human_track_topic, self.human_tracks_cb, qos)
        # Create publishers
        self.robot_marker_pub = self.create_publisher(Marker, robot_marker_topic, 10)    
        self.pose_markers_pub = self.create_publisher(MarkerArray, pose_marker_topic, 10)
        self.bbox_markers_pub = self.create_publisher(MarkerArray, bounding_box_topic, 10)        
        
    def egotraj_cb(self, msg):
        current_points = []
        for i in range(len(msg.track.poses)):
            point = Point()
            point.x = msg.track.poses[i].pose.position.x
            point.y = msg.track.poses[i].pose.position.y
            current_points.append(point)
            # Create new marker
            marker = Marker()
            marker.ns = "robot"
            marker.id = 9000
            marker.header.frame_id = self.pub_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.scale.x = 0.05
            marker.points = current_points
            marker.lifetime = Duration(seconds=0.1).to_msg()
        # Publish marker
        self.robot_marker_pub.publish(marker)

    def human_tracks_cb(self, msg):
        pose_marker_array = MarkerArray() 
        bbox_marker_array = MarkerArray()
        # self.get_logger().info(f'There are {len(self.people.tracks)} person(s) detected')
        for person in msg.tracks:
            # self.get_logger().info(f'Person has {len(person.track.poses)} poses')
            person_marker_points = []
            for i in range(len(person.track.poses)):
                point = Point()
                point.x = person.track.poses[i].pose.position.x
                point.y = person.track.poses[i].pose.position.y
                # Append point to person_track
                person_marker_points.append(point)
            # Create new marker for each person track
            marker = Marker()
            marker.ns = "human"
            marker.id = person.track_id
            marker.header.frame_id = self.pub_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.scale.x = 0.05
            marker.points = person_marker_points
            marker.lifetime = Duration(seconds=0.2).to_msg()
            pose_marker_array.markers.append(marker)
            # Create bounding box marker for the person
            if self.visualize_bbox and person.current_pose.pose.position.x != 0.0:
                marker = Marker()
                marker.ns = "human"
                marker.id = person.track_id + 1000
                marker.header.frame_id = self.pub_frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.pose.position.x = person.current_pose.pose.position.x
                marker.pose.position.y = person.current_pose.pose.position.y
                marker.pose.position.z = 1.80 / 2.0
                marker.scale.x = 0.40
                marker.scale.y = 0.40
                marker.scale.z = 1.80
                marker.color.a = 1.0
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.lifetime = Duration(seconds=0.4).to_msg()
                bbox_marker_array.markers.append(marker)
        # Publish final marker array
        self.pose_markers_pub.publish(pose_marker_array)
        self.bbox_markers_pub.publish(bbox_marker_array)


def main(args=None):
    rclpy.init(args=args)
    multi_track_visualizer = MultiTrackVisualizer()
    rclpy.spin(multi_track_visualizer)
    multi_track_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()