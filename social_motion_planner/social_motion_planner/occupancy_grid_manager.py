#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from itertools import product

"""
Class to deal with OccupancyGrid in Python
as in local / global costmaps.
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

class OccupancyGridManager(object):
    def __init__(self, costmap_msg, subscribe_to_updates=False):
        # OccupancyGrid starts on lower left corner
        self._grid_data = np.array(costmap_msg.data,
                                   dtype=np.uint8).reshape(data.info.height,
                                                           data.info.width)
        self._occ_grid_metadata = costmap_msg.info
        self._reference_frame = costmap_msg.header.frame_id

    @property
    def resolution(self):
        return self._occ_grid_metadata.resolution

    @property
    def width(self):
        return self._occ_grid_metadata.width

    @property
    def height(self):
        return self._occ_grid_metadata.height

    @property
    def origin(self):
        return self._occ_grid_metadata.origin

    @property
    def reference_frame(self):
        return self._reference_frame

    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.position.x
        world_y = costmap_y * self.resolution + self.origin.position.y
        return world_x, world_y

    def get_costmap_x_y(self, xy):
        costmap_xy = np.round((xy - np.array([self.origin.position.x, self.origin.position.y])) / self.resolution).astype(int)
        return costmap_xy

    def get_cost_from_world_x_y(self, xy):
        cxy = self.get_costmap_x_y(xy)
        try:
            test =  self.get_cost_from_costmap_x_y(cxy)
            return test
        except IndexError as e:
            raise IndexError(
                "Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                    self.reference_frame, x, y,
                    self.origin.position.x,
                    self.origin.position.x + self.height * self.resolution,
                    self.origin.position.y,
                    self.origin.position.y + self.width * self.resolution,
                    e))

    def pad_in_gridmap(self, xy):
        assert self.width == self.height
        xy = np.where(xy<0, 0, xy)
        xy = np.where(xy >= self.width, self.width-1, xy)
        return xy

    def get_cost_from_costmap_x_y(self, cxy):
        cxy = self.pad_in_gridmap(cxy)
        #cxy = np.flip(cxy, 1)

        return self._grid_data[cxy[:, :, 1], cxy[:, :, 0]]