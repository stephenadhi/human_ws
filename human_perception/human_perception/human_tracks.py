#!/usr/bin/env python3


import pyzed.sl as sl
from collections import deque
import numpy as np
import copy

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import tf2_ros
import tf_conversions
from robot_track import Robot_track

import time

#from scipy import interpolate

class people:
    def __init__(self):

        # init camera

        zed = sl.Camera()

        # Set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA
        init_params.camera_fps = 100
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD  # Use a right-handed Z-up x forward for ROS
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # ULTRA
        init_params.sdk_verbose = True

        # Open the camera
        err = zed.open(init_params)
        if (err != sl.ERROR_CODE.SUCCESS):
            print("camera initialization failed")
            exit()
        print("camera initialization finished")
        # ---------------------------------------------------------------------

        # -------------configurating object detection initialization and runtime parameter---------------------#
        # models can be found on https://github.com/stereolabs/zed-python-api/blob/master/src/pyzed/sl.pyx
        self.obj_param = sl.ObjectDetectionParameters()
        self.obj_param.detection_model = sl.DETECTION_MODEL.HUMAN_BODY_FAST
        self.obj_param.image_sync = True
        self.obj_param.enable_tracking = True
        self.obj_param.enable_body_fitting = False#True
        self.obj_param.prediction_timeout_s = 2.
        self.obj_param.batch_parameters.enable = True
        self.obj_param.allow_reduced_precision_inference = True
        # If you want to have object tracking you need to enable positional tracking first
        if self.obj_param.enable_tracking:
            positional_tracking_param = sl.PositionalTrackingParameters()
            initial_position = sl.Transform()
            initial_translation = sl.Translation()
            initial_translation.init_vector(0, 0, 0)  # in meter
            initial_position.set_translation(initial_translation)
            positional_tracking_param.set_initial_world_transform(initial_position)
            zed.enable_positional_tracking(positional_tracking_param)

        print("Object Detection: Loading Module...")
        err = zed.enable_object_detection(self.obj_param)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            zed.close()
            exit(1)
        print("finished")

        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.obj_runtime_param.detection_confidence_threshold = 40
        # ----------------------------------------------------------------------------------------------------#

        # --------------------preparing zed objects--------------------------#
        self.objects = sl.Objects()  # preparing the zed objects variable
        self.runtime_parameters = sl.RuntimeParameters()
        # runtime_parameters.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD
        self.runtime_parameters.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD

        zed_pose = sl.Pose()
        self.zed_pose = zed_pose
        self.zed = zed
        self.sl = sl

        # -------------------------------------------------------------------#

        print("creating node...")
        rospy.init_node("human_pose_publisher", anonymous=False)
        self.rate = rospy.Rate(60)
        # -------------------------------------------------------------------#

        self.person = []
        self.markerArray = MarkerArray()
        self.camera_pose = sl.Pose()
        self.robot_pose = Odometry()

     #   self.markerArray_corrected = MarkerArray()

        self.markerarray_publisher = rospy.Publisher("/interpolated_history_position", MarkerArray,
                                                     queue_size=5)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,)
        #rospy.sleep(2)

        robot_sub = rospy.Subscriber("/locobot/mobile_base/odom", Odometry, self.test, queue_size=1)
        self.robot_track = Robot_track()
        print('Init done!')
        self.zed_perception_loop()



    def zed_perception_loop(self):

        while not rospy.is_shutdown():


            t1 = time.time()

            # odom = rospy.wait_for_message("/locobot/mobile_base/odom", Odometry, timeout=None)
            # Locobot.update_interpolated_poses(odom)

            if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:  # Grab new frames and detect objects
                returned_state = self.zed.retrieve_objects(self.objects, self.obj_runtime_param)  # retrieve objects

                if (returned_state == sl.ERROR_CODE.SUCCESS and self.objects.is_new):
                    obj_array = self.objects.object_list
                    if (self.obj_param.enable_tracking) == True:
                        # First add new points and remove the ones that are too old
                        current_timestamp = time.time()
                        self.generate_from_objects(self.objects, current_timestamp)
                        self.prune_old_interpolated_points(
                            current_timestamp)  # here we delete old entries of interpolated points

            self.rate.sleep()  # wichtig! otherwise your program will not exit ctrl + c

            t2 = time.time()

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
        # Apply the transformation matrix to the array
      #  print(array)
       # homogeneous_array = np.concatenate([array, np.ones((array.shape[0], 1))], axis=1).T
      #  print(array.shape)
      #  print(matrix)
        #transformed_array = np.dot(matrix, array)

    def test(self, robot_pose):
        self.robot_track.interpolated_pose(robot_pose)
        marker_array = MarkerArray()
        marker_array.markers.append(self.robot_track.marker)
        if len(self.person) != 0:
            state = self.zed.get_position(self.zed_pose, self.sl.REFERENCE_FRAME.WORLD)
            #py_translation = self.sl.Translation()
            #rotation = self.zed_pose.get_rotation_vector()
            cam_pos_m = self.zed_pose.pose_data(self.sl.Transform()).m
            inv_Twc = np.linalg.inv(cam_pos_m)
            trans = self.tfBuffer.lookup_transform('locobot/odom', 'locobot/base_link', rospy.Time())

            robot_to_map_matrix = self.transform_array(trans)
            for pers in self.person:
                tranformed_to_cam_frame_points = (inv_Twc @ pers.arr_interp_padded.T).T
                tranformed_to_map_frame_points = (robot_to_map_matrix @ tranformed_to_cam_frame_points.T).T


                tranformed_to_cam_frame_list = []
                for i in range(pers.max_history_length + 1):
                    point = Point()
                    point.x = tranformed_to_map_frame_points[i, 0]
                    point.y = tranformed_to_map_frame_points[i, 1]
                    tranformed_to_cam_frame_list.append(point)
                pers.marker.points = tranformed_to_cam_frame_list
                marker_array.markers.append(pers.marker)
        self.markerarray_publisher.publish(marker_array.markers)
        self.rate.sleep()

    def generate_from_objects(self, objects, current_timestamp):
        t1 = time.time()

        for obj in objects.object_list:
           # print(obj.tracking_state)
            if ((obj.tracking_state != sl.OBJECT_TRACKING_STATE.OK) or (not np.isfinite(obj.position[0])) or (obj.id < 0)):
                if obj.tracking_state == sl.OBJECT_TRACKING_STATE.SEARCHING:
                    pass
                else:
                    continue

            new_object = True
            for i in range(len(self.person)):
                if self.person[i].id == obj.id:
                    new_object = False
                    self.person[i].add_interpolated_point(obj, current_timestamp)  # ---------------------------------here we add a new position to the created interpolatedTracklets

            # In case this object does not belong to existing tracks
            if (new_object):
                self.person.append(interpolatedTracklet(obj, current_timestamp))  # ----------------------here we create a new object of interpolatedTracklets
             #   self.fill_marker_array()

        t2 = time.time()
      #  print("one loop for generate_from_objects takes ", t2 - t1, " seconds")

    def fill_marker_array(self):
        for p in self.person:
            self.markerArray.markers.append(p.marker)


    def prune_old_interpolated_points(self, ts):
        track_to_delete = []
        for it in self.person:
            if ((ts - it.last_timestamp) > (3.)):
                track_to_delete.append(it)
                it.marker.action = 2

        for it in track_to_delete:
            self.person.remove(it)

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

    def add_interpolated_point(self, obj_, timestamp_):
        self.curr_time = timestamp_

        current_pos = obj_.position # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< THIS MAKES THE POINT AS IT IS
        self.odoms.append([current_pos[0], current_pos[1], timestamp_])

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




#
if __name__ == '__main__':

    People = people()








