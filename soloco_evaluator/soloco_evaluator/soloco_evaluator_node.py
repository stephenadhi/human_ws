#!/usr/bin/env python3

import numpy as np
import math
import sys
import os

import rclpy
from rclpy.node import Node
from soloco_evaluator import hunav_metrics

from soloco_interfaces.msg import EgoTrajectory, TrackedPersons
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

class SolocoEvaluatorNode(Node):

    def __init__(self):
        super().__init__("soloco_evaluator_node")
        
        name = 'hunav_evaluator'
        self.agents_list = []
        self.robot_list = []
        self.robot_goal = None
        self.metrics_to_compute = {}
        self.metrics_lists = {}

        self.declare_parameter('goal_tolerance', 0.2)
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # Two modes:
        # 1- The user start/stop the recording through the
        #    the service /hunav_trigger_recording
        # 2- The recording start/stop process is semi-automatic:
        #    It starts when the first topic is received or a navigation goal is received.
        #    It stops when a certain time pass without receiving data. 
        self.mode = self.declare_parameter('mode', 2).get_parameter_value().integer_value

        # Indicate the frequency of capturing the data  (it must be slower than data publishing).
        # If the value is set to zero, the data is captured at the same frequency than it is published.
        self.freq = self.declare_parameter('frequency', 0.0).get_parameter_value().double_value

        # base name of the result files
        self.declare_parameter('result_file', 'metrics')
        self.result_file_path = self.get_parameter('result_file').get_parameter_value().string_value

        # tag to identify the experiment
        self.declare_parameter('experiment_tag', '1')
        self.exp_tag = self.get_parameter('experiment_tag').get_parameter_value().string_value

        # optionally, the data recording can be started when a robot navigation goal is received
        self.declare_parameter('use_nav_goal_to_start', True)
        self.use_navgoal_to_start = self.get_parameter('use_nav_goal_to_start').get_parameter_value().bool_value

        # Read metrics
        for m in hunav_metrics.metrics.keys():
            ok = self.declare_parameter('metrics.'+m, True).get_parameter_value().bool_value
            if(ok):
                self.metrics_to_compute[m] = 0.0
        
        self.get_logger().info("Started Hunav evaluator:")
        self.get_logger().info("mode: %i" % self.mode)
        self.get_logger().info("freq: %.1f" % self.freq)
        self.get_logger().info("use_nav_goal_to_start: %i" % self.use_navgoal_to_start)
        self.get_logger().info("result_file: %s" % self.result_file_path)
        self.get_logger().info("experiment_tag: %s" % self.exp_tag)
        # self.get_logger().info("Metrics:")
        # for m in self.metrics_to_compute.keys():
        #     self.get_logger().info("m: %s, value: %s" % (m, self.metrics_to_compute[m]))

        if(self.freq > 0.0):
            self.agents = TrackedPersons()
            self.robot = EgoTrajectory()
            self.record_timer = self.create_timer(1/self.freq, self.timer_record_callback)

        if(self.mode == 1):
            self.recording = False
            self.recording_srv = self.create_service(Trigger, 'hunav_trigger_recording', self.recording_service)
        elif(self.mode == 2):
            if self.use_navgoal_to_start == True:
                self.recording = False
            else:
                self.recording = True
            self.timeout_period = 30.0  # seconds
            self.last_time = self.get_clock().now()
            self.init = False
            self.end_timer = self.create_timer(self.timeout_period, self.timer_end_callback)
            #self.end_timer.cancel()
        else:
            self.get_logger().error("Mode not recognized. Only modes 1 or 2 are allowed")

        # Define subscribers
        self.agent_sub = self.create_subscription(TrackedPersons, 'human/interpolated_history', self.human_callback, 1)
        self.robot_sub = self.create_subscription(EgoTrajectory, 'robot/ego_trajectory', self.robot_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)

    def human_callback(self, msg):
        if(self.mode == 2):
            self.init = True
            self.last_time = self.get_clock().now()
        if(self.recording == True):
            #self.get_logger().info("human received")
            if(self.freq == 0.0):
                self.agents_list.append(msg)
            else:
                self.agents = msg

    def robot_callback(self, msg):
        if(self.mode == 2):
            self.init = True
            self.last_time = self.get_clock().now()
        if(self.recording == True):
            #self.get_logger().info("robot received")
            robot_msg = msg
            if(self.robot_goal is not None):
                robot_msg.goals.clear()
                robot_msg.goals.append(self.robot_goal.pose)
                robot_msg.goal_radius = 0.2
            if(self.freq == 0.0):
                self.robot_list.append(robot_msg)
            else:
                self.robot = robot_msg

        # Stop recording and compute metrics 
        if(self.robot_goal is not None and self.recording):
            distance_to_goal = math.hypot(
            msg.current_pose.pose.position.x - self.robot_goal.pose.position.x,
            msg.current_pose.pose.position.y - self.robot_goal.pose.position.y
        )

            if distance_to_goal <= self.goal_tolerance:
                self.recording = False
                self.get_logger().info("Goal reached! Hunav evaluator stopped recording!")
                self.compute_metrics()

    def goal_callback(self, msg):
        self.robot_goal = msg
        if self.use_navgoal_to_start == True:
            self.get_logger().info("Goal received! Hunav evaluator started recording!")
            self.use_navgoal_to_start = False
            self.recording = True

    def recording_service(self, request, response):
        response.success = True
        status = "stopping" if self.recording else "started"
        self.get_logger().info(f"Hunav evaluator {status} recording!")
        self.recording = not self.recording
        response.message = f'Hunav recording {status}'
        if status == "stopping":
            self.compute_metrics()
        
    def timer_end_callback(self):
        if(self.init == True):
            secs = (self.get_clock().now() - self.last_time).to_msg().sec
            #self.get_logger().info("secs: %.2f" % secs)
            if(secs >= self.timeout_period):
                self.recording == False
                self.get_logger().info("Time period reached! Hunav evaluator stopped recording!")
                self.compute_metrics()

    def timer_record_callback(self):
        if(self.recording == True and self.init == True):
            #self.get_logger().info("Saving data...")
            self.agents_list.append(self.agents)
            self.robot_list.append(self.robot)

    def compute_metrics(self):
        self.check_data()
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        self.get_logger().info("Hunav evaluator. Collected %i messages of agents and %i of robot" % (agents_size, robot_size))
        self.get_logger().info("Computing metrics...")

        # compute metrics for all agents
        self.metrics_lists['time_stamps']=hunav_metrics.get_time_stamps(self.agents_list, self.robot_list)
        for m in self.metrics_to_compute.keys():
            metric = hunav_metrics.metrics[m](self.agents_list, self.robot_list)
            self.metrics_to_compute[m] = metric[0]
            if len(metric) > 1:
                self.metrics_lists[m]=metric[1]
            
        print('Metrics computed:')
        print(self.metrics_to_compute)
        self.store_metrics(self.result_file_path)

        self.destroy_node()
        sys.exit()
        #return

    def store_metrics(self, result_file):
        list_file = result_file
        # add extension if it does not have it
        if not result_file.endswith(".txt"):
            result_file += '.txt'
            list_file += '_steps_' + str(self.exp_tag) + '.txt'
        else:
            list_file = list_file[:-4]
            list_file += '_steps_' + str(self.exp_tag) + '.txt'

        file_was_created = os.path.exists(result_file)

        # open the file
        file = open(result_file,'a+')
        if(file is None):
            self.get_logger().error("RESULT METRICS FILE NOT CREATED! FILE: %s" % result_file)

        # if the file is new, create a header
        if file_was_created == False:
            file.write('experiment_tag')
            file.write('\t')
            for m in self.metrics_to_compute.keys():
                file.write(m)
                file.write('\t')
            file.write('\n')
        
        # write the data
        file.write(self.exp_tag)
        file.write('\t')
        for v in self.metrics_to_compute.values():
            file.write(str(v))
            file.write('\t')
        file.write('\n')
        file.close()

        # open and write the second file (metric for each step)
        file2 = open(list_file,'w')
        for m in self.metrics_lists.keys():
            file2.write(m)
            file2.write('\t')
        file2.write('\n')
        length = len(self.metrics_lists['time_stamps'])
        for i in range(length):
            for m in self.metrics_lists.keys():
                v = self.metrics_lists[m]
                file2.write(str(v[i]))
                file2.write('\t')
            file2.write('\n')
        file2.close()

    def check_data(self):
        #First check the number of messages
        agents_size = len(self.agents_list)
        robot_size = len(self.robot_list)
        if(abs(agents_size - robot_size) != 0):
            while(len(self.agents_list) > len(self.robot_list)):
                self.agents_list.pop()
            while(len(self.robot_list) > len(self.agents_list)):
                self.robot_list.pop()
            
        # check that the robot msg contains a goal?
        # check when the robot reaches the goal?


def main(args=None):
    rclpy.init(args=args)
    node = SolocoEvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()