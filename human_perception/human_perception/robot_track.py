#!/usr/bin/env python3
from rclpy.time import Time, Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from collections import deque
import numpy as np

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

