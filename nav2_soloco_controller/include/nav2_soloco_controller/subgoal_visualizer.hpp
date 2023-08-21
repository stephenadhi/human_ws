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

class SubGoalVisualizer : public rclcpp::Node {
public:
    SubGoalVisualizer();

private:
    void visualize_subgoal(const geometry_msgs::msg::PoseStamped::SharedPtr subgoal);

private:
    std::string frame_id_;
    double goal_tolerance_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subgoal_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

#endif // SUBGOAL_PUBLISHER_HPP_