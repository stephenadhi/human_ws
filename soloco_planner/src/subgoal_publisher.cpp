#include "soloco_planner/subgoal_publisher.hpp"

SubGoalPublisher::SubGoalPublisher()
    : Node("subgoal_publisher") {
    // Declare parameters
    std::string global_plan_topic, subgoal_topic, subgoal_marker_topic, odom_topic;
    this->declare_parameter<std::string>("odom_topic", "locobot/odom");
    this->declare_parameter<std::string>("global_plan_topic", "plan");
    this->declare_parameter<std::string>("subgoal_topic", "subgoal_pose");
    this->declare_parameter<std::string>("subgoal_marker_topic", "visualization/subgoal");
    this->declare_parameter<double>("lookahead_dist", 3.0);
    this->declare_parameter<double>("goal_tolerance", 0.5);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("global_plan_topic", global_plan_topic);
    this->get_parameter("subgoal_topic", subgoal_topic);
    this->get_parameter("subgoal_marker_topic", subgoal_marker_topic);
    this->get_parameter("lookahead_dist", lookahead_dist);
    this->get_parameter("goal_tolerance", goal_tolerance);
    //Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&SubGoalPublisher::odomCallback, this, std::placeholders::_1));
    global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        global_plan_topic, 10, std::bind(&SubGoalPublisher::globalPlanCallback, this, std::placeholders::_1));
    // Publisher
    subgoal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(subgoal_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(subgoal_marker_topic, 10);
}

void SubGoalPublisher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    robot_pose = msg->pose.pose;
}

void SubGoalPublisher::globalPlanCallback(const nav_msgs::msg::Path::SharedPtr msg){
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(
        msg->poses.begin(), msg->poses.end(), [&](const auto & ps) {
        return std::hypot(ps.pose.position.x- robot_pose.position.x, ps.pose.position.y - robot_pose.position.y) >= lookahead_dist;
        }
    );
    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == msg->poses.end()) {
        goal_pose_it = std::prev(msg->poses.end());
    }        
    geometry_msgs::msg::PoseStamped subgoal_pose;
    auto idx = std::distance(msg->poses.begin(), goal_pose_it);
    subgoal_pose = msg->poses[idx];
    subgoal_pose.header.stamp = this->get_clock()->now();
    //RCLCPP_INFO(this->get_logger(),"lookahead dist: %f", lookahead_dist);
    subgoal_pub_->publish(subgoal_pose);
    visualize_subgoal(subgoal_pose);
}

void SubGoalPublisher::visualize_subgoal(geometry_msgs::msg::PoseStamped subgoal){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "goals";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = goal_tolerance*2;
    marker.scale.y = goal_tolerance*2;
    marker.scale.z = 0.01;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 0.1;
    lifetime.nanosec = 0;
    marker.lifetime = lifetime;
    marker.pose.position.x = subgoal.pose.position.x;
    marker.pose.position.y = subgoal.pose.position.y;
    marker_pub_->publish(marker);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubGoalPublisher>());
    rclcpp::shutdown();
    return 0;
}