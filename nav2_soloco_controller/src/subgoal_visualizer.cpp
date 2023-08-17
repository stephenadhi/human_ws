#include "nav2_soloco_controller/subgoal_visualizer.hpp"

SubGoalVisualizer::SubGoalVisualizer()
    : Node("subgoal_visualizer") {
    // Declare parameters
    std::string subgoal_topic, subgoal_marker_topic;
    this->declare_parameter<std::string>("frame_id", "locobot/odom");
    this->declare_parameter<std::string>("subgoal_topic", "subgoal_pose");
    this->declare_parameter<std::string>("subgoal_marker_topic", "visualization/subgoal");
    this->declare_parameter<double>("goal_tolerance", 0.5);
    this->get_parameter("subgoal_topic", subgoal_topic);
    this->get_parameter("subgoal_marker_topic", subgoal_marker_topic);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("goal_tolerance", goal_tolerance_);
    //Subscribers
    subgoal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        subgoal_topic, 10, std::bind(&SubGoalVisualizer::visualize_subgoal, this, std::placeholders::_1));
    // Publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(subgoal_marker_topic, 10);
}

void SubGoalVisualizer::visualize_subgoal(const geometry_msgs::msg::PoseStamped::SharedPtr subgoal){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "goals";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = goal_tolerance_*2;
    marker.scale.y = goal_tolerance_*2;
    marker.scale.z = 0.01;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 0.1;
    lifetime.nanosec = 0;
    marker.lifetime = lifetime;
    marker.pose.position.x = subgoal->pose.position.x;
    marker.pose.position.y = subgoal->pose.position.y;
    marker_pub_->publish(marker);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubGoalVisualizer>());
    rclcpp::shutdown();
    return 0;
}