#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from zed_interfaces.msg import ObjectsStamped
from soloco_interfaces.msg import TrackedPerson, TrackedPersons

from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener, TransformException

import numpy as np
from collections import deque

from robot_track import Robot_track

class HumanTrackPublisher(Node):
    def __init__(self):
        super().__init__('human_track_publisher')
        # QoS settings
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # Declare parameters
        self.declare_parameter('odom_topic', '/zed2/zed_node/odom')
        self.declare_parameter('detected_obj_topic', '/zed2/zed_node/obj_det/objects')
        self.declare_parameter('human_track_topic', '/human/tracks')
        self.declare_parameter('pose_marker_topic', '/visualization/tracks')
        self.declare_parameter('bounding_box_topic', '/visualization/bounding_boxes')
        self.declare_parameter('pub_frame_id', 'map')
        self.declare_parameter('pub_frame_rate', 15.0)
        self.declare_parameter('interp_interval', 0.4)
        self.declare_parameter('max_history_length', 12)
        self.declare_parameter('delay_tolerance', 3.0)
        self.declare_parameter('visualize_bbox', False) 
        self.declare_parameter('max_num_agents', 5)
        self.declare_parameter('track_timeout', 2.0)    

        # Get parameter values
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        detected_obj_topic = self.get_parameter('detected_obj_topic').get_parameter_value().string_value
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value
        pose_marker_topic = self.get_parameter('pose_marker_topic').get_parameter_value().string_value
        bounding_box_topic = self.get_parameter('bounding_box_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        self.pub_frame_rate = self.get_parameter("pub_frame_rate").get_parameter_value().integer_value
        self.interp_interval = self.get_parameter("interp_interval").get_parameter_value().double_value
        self.delay_tolerance = self.get_parameter("delay_tolerance").get_parameter_value().double_value
        self.max_history_length = self.get_parameter("max_history_length").get_parameter_value().integer_value
        self.visualize_bbox = self.get_parameter("visualize_bbox").get_parameter_value().bool_value
        self.max_num_agents = self.get_parameter('max_num_agents').get_parameter_value().integer_value
        self.track_timeout = self.get_parameter('track_timeout').get_parameter_value().double_value
        
        # Create subscribers
        self.zed_sub = self.create_subscription(ObjectsStamped, detected_obj_topic, self.zed_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, qos)
        
        # Create publishers
        self.human_track_interpolated_pub = self.create_publisher(TrackedPersons, human_track_topic, qos)
        self.pose_markers_pub = self.create_publisher(MarkerArray, pose_marker_topic, 10)
        self.bbox_markers_pub = self.create_publisher(MarkerArray, bounding_box_topic, 10)

        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        # Bool to check object ID
        self.new_object = False
        
        # Create cache for updating people and robot history
        self.idx = 0
        self.people = TrackedPersons()
        self.interpolated_tracklets = []
        self.robot_track = Robot_track(self.max_history_length)
        
    def odom_callback(self, msg):
        # Get current robot pose in publishing frame (default: 'map')
        robot_in_map_frame = PoseStamped()
        robot_in_map_frame.pose = self.pose_transform(msg.pose.pose, self.pub_frame_id, msg.header.frame_id)
        robot_in_map_frame.header.stamp = msg.header.stamp

        if robot_in_map_frame.pose: # if transform exists, concatenate robot with people and publish
            self.robot_track.interpolated_pose(robot_in_map_frame)

    def zed_callback(self, msg):
        # Loop through all detected objects, only consider valid tracking
        for obj in range(len(msg.objects)):
            if (msg.objects[obj].tracking_state == 1 and msg.objects[obj].label == "Person"):
                # Get object ID
                obj_id = msg.objects[obj].label_id
                # Position in camera frame, saved in poseStamped message format
                curr_pose_cam = Pose()
                curr_pose_cam.position.x = float(msg.objects[obj].position[0])
                curr_pose_cam.position.y = float(msg.objects[obj].position[1])
                curr_pose_cam.position.z = 0.0
                curr_pose_cam.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                # Convert pose to poseStamped in publishing frame (default: 'map')
                curr_pose_map = PoseStamped()
                curr_pose_map.header.stamp = msg.header.stamp
                curr_pose_map.header.frame_id = self.pub_frame_id
                curr_pose_map.pose = self.pose_transform(curr_pose_cam.pose, self.pub_frame_id, msg.header.frame_id)
                # if transformed pose exists
                if curr_pose_map.pose:             
                    self.new_object = True
                    self.idx = len(self.people.tracks) - 1
                    # Check for matching id in cache
                    if len(self.people.tracks) != 0:
                        for person in self.people.tracks:
                            idx = 0
                            if person.track_id == obj_id:
                                self.get_logger().info(f'Track ID Matched: {person.track_id}, idx: {idx}')
                                self.new_object = False
                                self.interpolated_tracklets[idx].add_interpolated_point(curr_pose_map, idx, self.get_clock().now())
                                person.track.header.stamp = self.get_clock().now().to_msg()
                                self.idx = idx
                                break
                            idx += 1
                    # In case this object does not belong to existing tracks
                    if self.new_object:
                        curr_time_ = self.get_clock().now()
                        # Create a new person and append current person pose in map frame
                        tracked_person = TrackedPerson()
                        tracked_person.track.header.stamp = curr_time_.to_msg()
                        tracked_person.tracking_state = 1
                        tracked_person.track_id = obj_id
                        tracked_person.current_pose = curr_pose_map
                        self.people.tracks.append(tracked_person)
                        self.get_logger().info(f'New person detected! ID: {obj_id}, idx: {idx}')
                        self.interpolated_tracklets.append(
                            interpolatedTracklet(curr_pose_map, idx, curr_time_, self.max_history_length, self.interp_interval))
                    # Update person track
                    self.update_person_track(person_idx=self.idx)

            # Delete entries of interpolated points older than 
            self.prune_old_interpolated_points(self.get_clock().now())

        # Visualize markers for detected person(s) track and robot track
        self.visualize_markers()

    def pose_transform(self, curr_pose, output_frame, input_frame):
        transformed_pose = Pose()
        try:
            tf = self.tf_buffer.lookup_transform(output_frame, input_frame, rclpy.time.Time())
            transformed_pose = do_transform_pose(curr_pose, tf)
        except TransformException as ex:
            self.get_logger().warning(f"Failed to transform: '{ex}'.")
            
        return transformed_pose if transformed_pose else None

    def update_person_track(self, person_idx):
        # Get interpolated tracklet
        interpolated_tracklet = self.interpolated_tracklets[person_idx]
        self.people.tracks[person_idx].track.header.stamp = self.get_clock().now().to_msg()
        for i in range(self.max_history_length + 1):
            pose_xy = Pose()
            pose_xy.position.x = interpolated_tracklet.arr_interp_padded[i, 0]
            pose_xy.position.y = interpolated_tracklet.arr_interp_padded[i, 1]
            if self.new_object:
                self.people.tracks[person_idx].track.poses.append(pose_xy)    
            else:
                self.people.tracks[person_idx].track.poses[i] = pose_xy 

    def prune_old_interpolated_points(self, timestamp_):
        track_to_delete = []
        for person in self.people.tracks:
            det_time = Time.from_msg(person.track.header.stamp)
            time_diff = (timestamp_.nanoseconds - det_time.nanoseconds)/10**9
            # self.get_logger().info(f'Track time difference: {time_diff}s')
            if ((time_diff) > self.track_timeout):
                track_to_delete.append(person)
        # Delete old tracks
        for person in track_to_delete:
            self.people.tracks.remove(person)
            self.get_logger().info(f'Deleted old track with ID: {person.track_id}')
            
    def visualize_markers(self):
        pose_marker_array, bbox_marker_array = MarkerArray()
        # Append current interpolated robot position to marker array
        pose_marker_array.markers.append(self.robot_track.marker)
        # Additionally append people to marker array
        if len(self.people.tracks) != 0:
            # self.get_logger().info(f'There are {len(self.people.tracks)} person(s) detected')
            for person in self.people.tracks:
                # self.get_logger().info(f'Person has {len(person.track.poses)} poses')
                person_marker_points = []
                for i in range(self.max_history_length + 1):
                    point = Point()
                    point.x = person.track.poses[i].position.x
                    point.y = person.track.poses[i].position.y
                    # Append point to person_track
                    person_marker_points.append(point)
                # Create new marker for each person track
                marker = Marker()
                marker.ns = "human"
                marker.id = person.track_id + 1  # 0 is reserved for the robot
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
                marker.lifetime = Duration(seconds=0.1).to_msg()
                pose_marker_array.markers.append(marker)
                # Create bounding box marker for the person
                if self.visualize_bbox:
                    marker = Marker()
                    marker.ns = "human"
                    marker.id = person.track_id + 1  # 0 is reserved for the robot
                    marker.header.frame_id = self.pub_frame_id
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = marker.CUBE
                    marker.action = marker.ADD
                    marker.pose.position.x = person.current_pose.pose.position.x
                    marker.pose.position.y = person.current_pose.pose.position.y
                    marker.pose.position.z = 1.80 / 2.0
                    marker.scale.x = 0.20
                    marker.scale.y = 0.20
                    marker.scale.z = 1.80
                    marker.color.a = 0.4
                    marker.color.r = 0.0
                    marker.color.g = 0.9
                    marker.color.b = 0.0
                    marker.lifetime = Duration(seconds=0.2).to_msg()
                    bbox_marker_array.markers.append(marker)
        # Publish final marker array
        self.pose_markers_pub.publish(pose_marker_array)
        self.bbox_markers_pub.publish(bbox_marker_array)

class interpolatedTracklet:
    def __init__(self, curr_pose, timestamp_, max_history_length, interp_interval):
        # Initialize variables
        self.max_history_length = max_history_length
        self.interp_interval = interp_interval
        # Deque object as cache for saving history
        self.track = deque(maxlen=90)
        # Create padding array to assign zeros for unknown future positions
        self.arr_interp_padded = np.zeros([self.max_history_length + 1, 2])
        # Add interpolated point
        self.add_interpolated_point(curr_pose, timestamp_)

    def add_interpolated_point(self, curr_pose, person_idx, timestamp_):
        curr_time_ = timestamp_.nanoseconds/10**9
        # get time of current pose
        det_time_ = Time.from_msg(curr_pose.header.stamp).nanoseconds/10**9
        self.track.append([curr_pose.pose.position.x, curr_pose.pose.position.y, det_time_])
        np_track = np.array(self.track)
        # Get number of steps depending on measured pose time and interpolation interval
        num_t_quan_steps = int((curr_time_ - self.track[0][2]) / self.interp_interval)
        num_t_quan_steps = self.max_history_length if num_t_quan_steps > self.max_history_length else num_t_quan_steps
        # Get the time points for interpolation
        inter_time_points = curr_time_ - np.arange(num_t_quan_steps + 1) * self.interp_interval
        # Interpolate using numpy library
        x_interp = np.interp(inter_time_points, np_track[:, 2], np_track[:, 0])
        y_interp = np.interp(inter_time_points, np_track[:, 2], np_track[:, 1])
        # Assign the interpolated points to the padded array
        self.arr_interp_padded[0:num_t_quan_steps + 1, 0] = x_interp
        self.arr_interp_padded[0:num_t_quan_steps + 1, 1] = y_interp    
        if num_t_quan_steps != self.max_history_length:
            self.arr_interp_padded[num_t_quan_steps + 1:, 0] = x_interp[-1]
            self.arr_interp_padded[num_t_quan_steps + 1:, 1] = y_interp[-1]

def main(args=None):
    rclpy.init(args=args)
    human_track_publisher = HumanTrackPublisher()
    rclpy.spin(human_track_publisher)
    human_track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()