import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from zed_interfaces.msg import ObjectsStamped
from soloco_interfaces.msg import TrackedPerson, TrackedPersons
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

import numpy as np
import math
from collections import deque

from robot_track import Robot_track

class HumanTrackPublisher(Node):
    def __init__(self):
        super().__init__('human_track_publisher')
        # QoS settings
        qos = QoSProfile(
            depth=10,
            reliability=QoSProfile.ReliabilityPolicy.RELIABLE,
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
        self.declare_parameter('visualize_bbox', True)

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
        
        # Create zed detected objects subscriber
        self.zed_sub = self.create_subscription(ObjectsStamped, detected_obj_topic, self.zed_callback, qos)
        # Create odom subscriber
        self.odom_sub = self.create_subscription(Goal, odom_topic, self.odom_callback, qos) 
        # Create human track publisher
        self.human_track_interpolated_pub = self.create_publisher(TrackedPersons, human_track_topic, qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'human_markers', 10)
        self.bbox_pub = self.create_publisher(MarkerArray, 'human_bounding_boxes', 10)

        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        # People vector as cache for detected humans
        self.people = TrackedPersons()
        
        self.person = []

    def odom_callback(self, msg):
        # In an ideal case, where the robot motion and sensors are perfectly accurate and precise, 
        # the transform between "odom" and "map" frames would be zero. However, in reality, there are always 
        # errors and uncertainties in robot motion and sensor readings, which can accumulate over time.
        # This results in drift between the "odom" and "map" frames. Therefore, it is important to use 
        # localization and mapping algorithms that can account for these errors and correct for drift 
        # in order to maintain accurate estimates of the robot position and orientation.
        
        # Get current robot pose in publishing frame (default: 'map')
        robot_in_map_frame = self.pose_transform(msg.pose.pose, self.pub_frame_id, msg.header.frame_id)
        self.robot_track.interpolated_pose(robot_in_map_frame, self.max_history_length)
        # Append current robot position to marker array
        marker_array = MarkerArray()
        marker_array.markers.append(self.robot_track.marker)
        # Append people to marker array
        if len(self.person) != 0:
            for pers in self.person:
                track = []
                for i in range(self.max_history_length + 1):
                    point = Point()
                    point.x = person.track.poses[i].position.x
                    point.y = person.track.poses[i].position.y
                    
                    track.append(point)
                    pers.marker.points = track
                    marker_array.markers.append(pers.marker)
        
        self.markerarray_publisher.publish(marker_array.markers)

    def zed_callback(self, msg):
        # Loop through all detected objects,
        # Only consider valid tracking
        if msg.objects[count].tracking_state == 1 and msg.objects[count].label == "Person":
            for count in range(len(msg.objects)):
                # Get the person id and tracking state
                tracked_person = soloco_interfaces.msg.TrackedPerson()
                tracked_person.track_id = "Person"
                tracked_person.tracking_state = 1
                # Position in camera frame
                person_x_pos = msg.objects[count].position[0]
                person_y_pos = msg.objects[count].position[1]
                # Save to current poseStamped message format
                currPose = geometry_msgs.msg.PoseStamped()
                currPose.header = msg.header
                currPose.pose.position.x = person_x_pos
                currPose.pose.position.y = person_y_pos
                currPose.pose.position.z = 0.0
                currPose.pose.orientation.x = 0.0
                currPose.pose.orientation.y = 0.0
                currPose.pose.orientation.z = 0.0
                currPose.pose.orientation.w = 1.0

                # First add new points and remove the ones that are too old
                current_timestamp = self.get_clock().now()
                self.generate_from_objects(self.objects, current_timestamp)
        
                # Delete old entries of interpolated points
                self.prune_old_interpolated_points(
                    current_timestamp)  


    def transform_array(self, transform):
        # Create a homogeneous transformation matrix from the transform
        translation = np.array([transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z])
        rotation = np.array([transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z,
                             transform.transform.rotation.w])
        matrix = tf_conversions.transformations.quaternion_matrix(rotation)
        matrix[:3, 3] = translation
        return matrix

    def generate_from_objects(self, objects, current_timestamp):
        for obj in objects.object_list:
            new_object = True
            for i in range(len(self.person)):
                if self.person[i].id == obj.id:
                    new_object = False
                    self.person[i].add_interpolated_point(obj, current_timestamp)  
            # In case this object does not belong to existing tracks
            if (new_object):
                # Create a new object of interpolatedTracklets
                self.person.append(interpolatedTracklet(obj, current_timestamp))

    def prune_old_interpolated_points(self, ts):
        track_to_delete = []
        for it in self.person:
            if ((ts - it.last_timestamp) > (3.)):
                track_to_delete.append(it)
                it.marker.action = 2

        for it in track_to_delete:
            self.person.remove(it)

    def pose_transform(obj, output_frame, input_frame):
        transformed_obj = PoseStamped()
        try:
            tf = m_tf_buffer.lookup_transform(output_frame, input_frame, self.get_clock().now())
            transformed_obj = tf2_geometry_msgs.do_transform_pose(obj, tf)
        except tf2_ros.TransformException as ex:
            self.get_logger().error('%s', ex)

        return transformed_obj if transformed_obj else None

#---------------------------------custom class made by krisna-----------------------------------------------#

class interpolatedTracklet:
    def __init__(self, obj_, timestamp_, max_history_length=7):
        self.id = obj_.id
        self.max_history_length = max_history_length
        self.odoms = deque(maxlen=90)  # from odom data, not PoseStamped
        self.points_interp = deque(maxlen=self.max_history_length)
        self.interp_point = 0.4
        self.curr_time = 0
        self.start_time = timestamp_
        self.arr_interp_padded = np.zeros([self.max_history_length + 1, 4])
        self.arr_interp_padded[:, -1] = 1.

        self.marker = Marker()
        self.marker.id = self.id + 1  # 0 is reserved for the robot
        self.marker.header.frame_id = "locobot/odom"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.scale.x = 0.05
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.lifetime = rospy.Duration.from_sec(0.1)

        self.add_interpolated_point(obj_, timestamp_)

    def add_interpolated_point(se
        num_t_quan_steps = int((self.curr_time - self.odoms[0][2]) / self.interp_point)
        num_t_quan_steps = self.max_history_length if num_t_quan_steps > self.max_history_length else num_t_quan_steps
        np_odoms = np.array(self.odoms)
        inter_time_points = self.curr_time - np.arange(num_t_quan_steps + 1) * self.interp_point
        x_interp = np.interp(inter_time_points, np_odoms[:, 2], np_odoms[:, 0])
        y_interp = np.interp(inter_time_points, np_odoms[:, 2], np_odoms[:, 1])
        # arr_interp = np.concatenate([x_interp, y_interp])
        self.arr_interp_padded[0:num_t_quan_steps + 1, 0] = x_interp
        self.arr_interp_padded[0:num_t_quan_steps + 1, 1] = y_interp
        if num_t_quan_steps != self.max_history_length:
            self.arr_interp_padded[num_t_quan_steps + 1:, 0] = x_interp[-1]
            self.arr_interp_padded[num_t_quan_steps + 1:, 1] = y_interp[-1]
        self.last_timestamp = timestamp_


def main(args=None):
    rclpy.init(args=args)
    human_track_publisher = HumanTrackPublisher()
    rclpy.spin(human_track_publisher)
    human_track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()









