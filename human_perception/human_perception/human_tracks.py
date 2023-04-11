#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from zed_interfaces.msg import ObjectsStamped
from soloco_interfaces.msg import TrackedPerson, TrackedPersons

from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

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
        self.declare_parameter('odom_topic', '/locobot/odom')
        self.declare_parameter('detected_obj_topic', '/zed2/zed_node/obj_det/objects')
        self.declare_parameter('human_track_topic', '/human/track')
        self.declare_parameter('pub_frame_id', 'map')
        self.declare_parameter('pub_frame_rate', 15.0)
        self.declare_parameter('max_interp_interval', 0.4)
        self.declare_parameter('max_history_length', 12)
        self.declare_parameter('delay_tolerance', 3.0)
        self.declare_parameter('visualize_bbox', False) 
        self.declare_parameter('max_num_agents', 5)
        self.declare_parameter('track_timeout', 3.0)    

        # Get parameter values
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        detected_obj_topic = self.get_parameter('detected_obj_topic').get_parameter_value().string_value
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        self.pub_frame_rate = self.get_parameter("pub_frame_rate").get_parameter_value().integer_value
        self.max_interp_interval = self.get_parameter("max_interp_interval").get_parameter_value().double_value
        self.delay_tolerance = self.get_parameter("delay_tolerance").get_parameter_value().double_value
        self.max_history_length = self.get_parameter("max_history_length").get_parameter_value().integer_value
        self.visualize_bbox = self.get_parameter("visualize_bbox").get_parameter_value().bool_value
        self.max_num_agents = self.get_parameter('max_num_agents').get_parameter_value().integer_value
        self.track_timeout = self.get_parameter('track_timeout').get_parameter_value().double_value
        
        # Create subscribers
        self.zed_sub = self.create_subscription(ObjectsStamped, detected_obj_topic, self.zed_callback, qos)
        self.odom_sub = self.create_subscription(PoseStamped, odom_topic, self.odom_callback, qos)
        
        # Create publishers
        self.human_track_interpolated_pub = self.create_publisher(TrackedPersons, human_track_topic, qos)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'track_markers', 10)
        self.bbox_pub = self.create_publisher(MarkerArray, 'human_bounding_boxes', 10)

        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        # People vector as cache for detected humans
        self.people = TrackedPersons()
        
    def odom_callback(self, msg):
        # Get current robot pose in publishing frame (default: 'map')
        robot_in_map_frame = self.pose_transform(msg.pose.pose, self.pub_frame_id, msg.header.frame_id)
        
        if robot_in_map_frame: # if transform exists, concatenate robot with people and publish
            self.robot_track.interpolated_pose(robot_in_map_frame, self.max_history_length)
            # Append current interpolaed robot position to marker array
            marker_array = MarkerArray()
            marker_array.markers.append(self.robot_track.marker)
            # dditionally append people to marker array
            if len(self.people.tracks) != 0:
                for person in self.people.tracks:
                    person_track = []
                    for i in range(self.max_history_length + 1):
                        point = Point()
                        point.x = person.track.poses[i].position.x
                        point.y = person.track.poses[i].position.y
                        # Append point to person_track
                        track.append(point)
                    # Create new marker for each person track
                    marker = Marker()
                    marker.id = person.track_id + 1  # 0 is reserved for the robot
                    marker.header.frame_id = pub_frame_id
                    marker.type = self.marker.LINE_STRIP
                    marker.action = self.marker.ADD
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.scale.x = 0.05
                    marker.points = person_track
                    marker.lifetime = Duration(seconds=0.1)
                    marker_array.markers.append(marker)
            # Publish final marker array
            self.marker_array_pub.publish(marker_array.markers)

    def zed_callback(self, msg):
        # Loop through all detected objects, only consider valid tracking
        if (msg.objects[count].tracking_state == 1 and msg.objects[count].label == "Person"):
            for obj in range(len(msg.objects)):
                obj_id = msg.objects[count].label_id
                # Get the person id and tracking state
                tracked_person = soloco_interfaces.msg.TrackedPerson()
                tracked_person.track_id = obj_id
                tracked_person.tracking_state = 1
                # Position in camera frame, saved in poseStamped message format
                curr_pose_cam = geometry_msgs.msg.PoseStamped()
                curr_pose_cam.header = msg.header
                curr_pose_cam.pose.position.x = msg.objects[count].position[0]
                curr_pose_cam.pose.position.y = msg.objects[count].position[1]
                curr_pose_cam.pose.position.z = 0.0
                curr_pose_cam.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                # Convert pose to publishing frame (default: 'map')
                curr_pose_map = self.pose_transform(msg.pose.pose, self.pub_frame_id, msg.header.frame_id)
                if curr_pose_map: # if transform exists               
                    curr_timestamp = self.get_clock().now()
                    new_object = True
                    people_count = len(self.people.tracks)
                    # Check for matching id from previous detections
                    if len(self.people.tracks) != 0:
                        for person in self.people.tracks:
                            if person.track_id == obj.id:
                                new_object = False
                                self.add_interpolated_point(curr_pose-map, obj_id, pub_frame_id, curr_timestamp)  
                    # In case this object does not belong to existing tracks
                    if new_object:
                        # Create a new person and append currebt map pose
                        person = TrackedPerson()
                        person.current_pose = curr_pose_map
                        person.track.header.stamp = curr_timestamp
                        person.track.poses.append(curr_pose_map)
            # Delete entries of interpolated points older than 
            self.prune_old_interpolated_points(curr_timestamp)

    def pose_transform(self, curr_pose, output_frame, input_frame):
        transformed_pose = PoseStamped()
        try:
            tf = m_tf_buffer.lookup_transform(output_frame, input_frame, self.get_clock().now())
            transformed_pose = tf2_geometry_msgs.do_transform_pose(curr_pose, tf)
        except tf2_ros.TransformException as ex:
            self.get_logger().error('%s', ex)
            
        return transformed_pose if transformed_pose else None

    def add_interpolated_point(self, curr_pose, person_id, pub_frame_id, timestamp_):
        # get time of current pose
        det_time = Time(curr_pose.header.stamp)
        # Create deque object for saving track
        track = deque(maxlen=90)
        track.append(curr_pose.poseposition.x, curr_pose.poseposition.y)
        np_track = np.array(track)
        # Create padding array to assign zeros for unknown future positions
        arr_interp_padded = np.zeros([self.max_history_length + 1, 2])
        # Get number of steps depending on measured pose time and interpolation interval
        num_t_quan_steps = int((timestamp_ - det_time) / self.max_interp_interval)
        num_t_quan_steps = self.max_history_length if num_t_quan_steps > self.max_history_length else num_t_quan_steps
        # Get the time points for interpolation
        inter_time_points = timestamp_ - np.arange(num_t_quan_steps + 1) * self.max_interp_interval
        # Interpolate using numpy library
        x_interp = np.interp(inter_time_points, det_time, np_track[:, 0])
        y_interp = np.interp(inter_time_points, det_time, np_track[:, 1])
        # Assign the interpolated points to the padded array
        arr_interp_padded[0:num_t_quan_steps + 1, 0] = x_interp
        arr_interp_padded[0:num_t_quan_steps + 1, 1] = y_interp    
        if num_t_quan_steps != self.max_history_length:
            arr_interp_padded[num_t_quan_steps + 1:, 0] = x_interp[-1]
            arr_interp_padded[num_t_quan_steps + 1:, 1] = y_interp[-1]
        # Renew person track
        self.people.tracks[person_id].track.header.stamp = now()
        self.people.tracks[person_id].track.poses.clear()
        for i in range(self.max_history_length + 1):
            pose_xy = Pose()
            pose_xy.position.x = arr_interp_padded[i, 0]
            pose_xy.position.y = arr_interp_padded[i, 1]
            self.people.tracks[person_id].track.poses.append(pose_xy)    

    def prune_old_interpolated_points(self, timestamp_):
        track_to_delete = []
        for person in self.people.tracks:
            det_time = Time(person.track.header.stamp)
            if ((timestamp_ - det_time) > self.track_timeout):
                track_to_delete.append(it)
        # Delete old tracks
        for it in track_to_delete:
            self.people.tracks.remove(person)
            

def main(args=None):
    rclpy.init(args=args)
    human_track_publisher = HumanTrackPublisher()
    rclpy.spin(human_track_publisher)
    human_track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()