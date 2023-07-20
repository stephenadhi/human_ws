#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from soloco_interfaces.msg import TrackedAgents
from soloco_interfaces.msg import TrackedPerson, TrackedPersons

from tf2_ros import Buffer, TransformListener

from soloco_perception.utils import interpolatedTracklet, pose_transform

class PedsimTrackPublisher(Node):
    def __init__(self):
        super().__init__('pedsim_track_publisher')
        # QoS settings
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        # Declare parameters
        self.declare_parameter('detected_agents_topic', 'human/simulated_agents')
        self.declare_parameter('human_track_topic', 'human/interpolated_history')
        self.declare_parameter('pub_frame_id', 'locobot/odom')
        self.declare_parameter('interp_interval', 0.4)
        self.declare_parameter('delay_tolerance', 0.05)
        self.declare_parameter('max_history_length', 7)
        self.declare_parameter('max_num_agents', 10)
        self.declare_parameter('track_timeout', 0.15)
        self.declare_parameter('pruning_rate', 20.0)

        # Get parameter values
        detected_agents_topic = self.get_parameter('detected_agents_topic').get_parameter_value().string_value
        human_track_topic = self.get_parameter('human_track_topic').get_parameter_value().string_value
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        self.interp_interval = self.get_parameter("interp_interval").get_parameter_value().double_value
        self.delay_tolerance = self.get_parameter("delay_tolerance").get_parameter_value().double_value
        self.max_history_length = self.get_parameter("max_history_length").get_parameter_value().integer_value
        self.max_num_agents = self.get_parameter('max_num_agents').get_parameter_value().integer_value
        self.track_timeout = self.get_parameter('track_timeout').get_parameter_value().double_value
        self.pruning_rate = self.get_parameter('pruning_rate').get_parameter_value().double_value
        # Create subscriber
        self.pedsim_sub = self.create_subscription(TrackedAgents, detected_agents_topic, self.det_agents_callback, qos)
        # Create publisher
        self.human_track_interpolated_pub = self.create_publisher(TrackedPersons, human_track_topic, qos)
        self.timer = self.create_timer(1/self.pruning_rate, self.timer_callback)
        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)
        # Bool to check object ID
        self.new_object = True
        # Create cache for updating people and robot history
        self.pedsim_tracker = TrackedAgents()
        self.people = TrackedPersons()
        self.idx = len(self.people.tracks) - 1
        self.interpolated_tracklets = []

    def det_agents_callback(self, msg):
        # Store tracked agents
        self.pedsim_tracker.header = msg.header
        self.pedsim_tracker.agents = msg.agents

    def timer_callback(self):
        time_now = self.get_clock().now()
        curr_agents = self.pedsim_tracker.agents
        # Loop through all detected objects, only consider valid tracking
        for obj in range(len(curr_agents)):
            self.new_object = True
            # Get object ID
            obj_id = curr_agents[obj].track_id
            # Position in camera frame, saved in poseStamped message format
            curr_pose_cam = Pose()
            curr_pose_cam.position.x = float(curr_agents[obj].current_pose.pose.position.x)
            curr_pose_cam.position.y = float(curr_agents[obj].current_pose.pose.position.y)
            curr_pose_cam.position.z = 0.0
            curr_pose_cam.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            # Convert pose to poseStamped in publishing frame (default: 'odom')
            curr_pose_pub_frame = PoseStamped()
            curr_pose_pub_frame.header.stamp = time_now.to_msg()
            curr_pose_pub_frame.header.frame_id = self.pub_frame_id
            curr_pose_pub_frame.pose = curr_pose_cam # pedsim relay node already publishes in the right frame
            # if transformed pose exists
            if curr_pose_pub_frame.pose:             
                self.idx = len(self.people.tracks) - 1
                # Check for matching id in cache
                idx = 0
                for person in self.people.tracks:
                    if person.track_id == obj_id:
                        # self.get_logger().info(f'Track ID Matched: {person.track_id}, idx: {idx}')
                        self.new_object = False
                        self.interpolated_tracklets[idx].add_interpolated_point(curr_pose_pub_frame, time_now)
                        # Update curent pose to the last interpolated pose
                        person.current_pose = curr_pose_pub_frame
                        # Stamp update time
                        person.track.header.stamp = time_now.to_msg()
                        self.idx = idx
                        break
                    idx += 1
                # In case this object does not belong to existing tracks
                if self.new_object:
                    self.idx += 1
                    # Create a new person and append current person pose in map frame
                    tracked_person = TrackedPerson()
                    tracked_person.track.header.frame_id = self.pub_frame_id
                    tracked_person.tracking_state = 1
                    tracked_person.track_id = obj_id
                    tracked_person.current_pose = curr_pose_pub_frame
                    self.people.tracks.append(tracked_person)
                    self.get_logger().info(f'New person detected! ID: {obj_id}, idx: {self.idx}')
                    self.interpolated_tracklets.append(
                        interpolatedTracklet(curr_pose_pub_frame, time_now, self.max_history_length, self.interp_interval))
                # Update person track
                self.update_person_track(person_idx=self.idx, time_now=time_now)
        # Delete entries of interpolated points
        self.prune_old_interpolated_points(time_now)
        # Publish human tracks
        self.people.header.stamp = time_now.to_msg()
        self.human_track_interpolated_pub.publish(self.people)

    def update_person_track(self, person_idx, time_now):
        # Get interpolated tracklet
        interpolated_tracklet = self.interpolated_tracklets[person_idx]

        for i in range(self.max_history_length + 1):
            pose_xy = PoseStamped()
            pose_xy.header.frame_id = self.pub_frame_id
            pose_xy.header.stamp.sec = int(self.interp_interval * i * 1000) # milliseconds
            pose_xy.pose.position.x = interpolated_tracklet.arr_interp_padded[i, 0]
            pose_xy.pose.position.y = interpolated_tracklet.arr_interp_padded[i, 1]
            if self.new_object:
                self.people.tracks[person_idx].track.poses.append(pose_xy)
            else:
                self.people.tracks[person_idx].track.poses[i] = pose_xy
        # Stamp update time
        self.people.tracks[person_idx].track.header.stamp = time_now.to_msg()

    def prune_old_interpolated_points(self, timestamp_):
        new_people_tracks = []
        new_interpolated_tracklets = []
        for person, tracklet in zip(self.people.tracks, self.interpolated_tracklets):
            det_time = Time.from_msg(person.track.header.stamp)
            time_diff = (timestamp_.nanoseconds - det_time.nanoseconds)/10**9
            if time_diff <= (self.track_timeout + self.delay_tolerance):
                new_people_tracks.append(person)
                new_interpolated_tracklets.append(tracklet)
        self.people.tracks = new_people_tracks
        self.interpolated_tracklets = new_interpolated_tracklets

def main(args=None):
    rclpy.init(args=args)
    pedsim_track_publisher = PedsimTrackPublisher()
    rclpy.spin(pedsim_track_publisher)
    pedsim_track_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
