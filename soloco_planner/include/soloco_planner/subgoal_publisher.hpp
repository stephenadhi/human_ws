#ifndef SUBGOAL_PUBLISHER_HPP_
#define SUBGOAL_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <vector>

class SubGoalPublisher : public rclcpp::Node {
public:
    SubGoalPublisher();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void globalPlanCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void visualize_subgoal(geometry_msgs::msg::PoseStamped subgoal);

private:
    double lookahead_dist, goal_tolerance;
    geometry_msgs::msg::Pose robot_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr subgoal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

#endif // SUBGOAL_PUBLISHER_HPP_