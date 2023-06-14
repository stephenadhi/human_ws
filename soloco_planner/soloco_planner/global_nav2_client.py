#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry   
from nav2_msgs.srv import GetCostmap
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose

# import message filters functionalities
from message_filters import Subscriber, ApproximateTimeSynchronizer


class GlobalNav2Client(Node):
    def __init__(self):
        super().__init__('global_nav2_client')

        self.declare_parameter('pub_frame_id', 'locobot/odom')
        self.declare_parameter('global_goal_topic', 'global_goal_republished')
        self.declare_parameter('republish_global_goal', False)
        self.declare_parameter('republish_goal_period', 0.5)
        self.declare_parameter('invoke_global_planner', False)
        self.pub_frame_id = self.get_parameter("pub_frame_id").get_parameter_value().string_value
        global_goal_topic = self.get_parameter("global_goal_topic").get_parameter_value().string_value
        self.invoke_global_planner = self.get_parameter('invoke_global_planner').get_parameter_value().bool_value
        self.republish_global_goal = self.get_parameter('republish_global_goal').get_parameter_value().bool_value
        self.republish_goal_period = self.get_parameter('republish_goal_period').get_parameter_value().double_value
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10) 
        self.odom_sub = self.create_subscription(Odometry, 'locobot/odom', self.odom_callback, 10) 

        self.publisher = self.create_publisher(PoseStamped, global_goal_topic, 10)
        self.timer = self.create_timer(self.republish_goal_period, self.timer_callback)

        # Create ComputePathToPose Client
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        '/compute_path_to_pose')

        self.get_costmap_local_srv = self.create_client(GetCostmap, '/local_costmap/get_costmap')

        # Create tf buffer and transform listener   
        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

    def goal_callback(self, goal_msg):
        self.goal_pose = PoseStamped()
        if goal_msg.header.frame_id != self.pub_frame_id:
            self.goal_pose.pose = self.pose_transform(goal_msg.pose, self.pub_frame_id, goal_msg.header.frame_id)
            self.goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.goal_pose.header.frame_id = self.pub_frame_id
            self.get_logger().info('Goal transformed to publishing frame.')
        else:
            self.goal_pose = goal_msg
        
    def odom_callback(self, odom_msg):
        self.odom = odom_msg

    def getPath(self, start_pose):
        # Sends a `ComputePathToPose` action request
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        goal_msg.goal = self.goal_pose
        goal_msg.start = start_pose
        goal_msg.planner_id = "GridBased"

        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal=goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Get path was rejected!')
        #     return None
        # self.result_future = self.goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, self.result_future)
        # self.status = self.result_future.result().status
        # if self.status != GoalStatus.STATUS_SUCCEEDED:
        #     self.get_logger().warn('Getting path failed with status code: {0}'.format(self.status))
        #     #return None
        return None

    def timer_callback(self):
        try:
            self.goal_pose
            self.odom
        except AttributeError:
            # no message received yet
            return
        if self.republish_global_goal:
            self.publisher.publish(self.goal_pose)
        # self.get_logger().info('Getting path...')
        if self.invoke_global_planner:
            start_pose = PoseStamped()
            start_pose.pose = self.odom.pose.pose
            start_pose.header = self.odom.header
            self.getPath(start_pose)

    def pose_transform(self, curr_pose, output_frame, input_frame):
        transformed_pose = Pose()
        success = False
        while not success:
            try:
                tf = self.tf_buffer.lookup_transform(output_frame, input_frame, Time())
                transformed_pose = do_transform_pose(curr_pose, tf)
            except TransformException as ex:
                self.get_logger().warning(f"Failed to transform: '{ex}'.")
            if transformed_pose: success = True
        
        return transformed_pose
    
def main(args=None):
    rclpy.init(args=args)
    global_nav2_client = GlobalNav2Client()
    rclpy.spin(global_nav2_client)
    global_nav2_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
