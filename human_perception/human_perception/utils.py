#!/usr/bin/env python3

from rclpy.time import Time

import numpy as np
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
from geometry_msgs.msg import Pose

def pose_transform(self, curr_pose, output_frame, input_frame):
    transformed_pose = Pose()
    try:
        tf = self.tf_buffer.lookup_transform(output_frame, input_frame, Time())
        transformed_pose = do_transform_pose(curr_pose, tf)
    except TransformException as ex:
        self.get_logger().warning(f"Failed to transform: '{ex}'.")
        
    return transformed_pose if transformed_pose else None

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

    def add_interpolated_point(self, curr_pose, timestamp_):
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