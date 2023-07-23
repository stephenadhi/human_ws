// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <chrono>
#include "nav2_soloco_controller/controller.hpp"

// #define BENCHMARK_TESTING

namespace nav2_soloco_controller
{

void SolocoController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(max_lookahead_dist_, "max_lookahead_dist", 0.4);
  getParam(min_lookahead_dist_, "min_lookahead_dist", 2.4);
  getParam(max_speed_, "max_speed", 0.5);
  // Configure composed objects
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());

  // Configure action client
  soloco_client_ptr_ = rclcpp_action::create_client<soloco_interfaces::action::NavigateToXYGoal>(
    node, "navigate_to_xy_goal");

  if (!soloco_client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(logger_, "Action server not available after waiting");
  }
  
  RCLCPP_INFO(logger_, "Configured Soloco Controller: %s", name_.c_str());
}

void SolocoController::cleanup()
{
  // optimizer_.shutdown();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up Soloco Controller: %s", name_.c_str());
}

void SolocoController::activate()
{
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated Soloco Controller: %s", name_.c_str());
}

void SolocoController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivated Soloco Controller: %s", name_.c_str());
}

// void SolocoController::reset()
// {
//   RCLCPP_INFO(logger_, "Reset Soloco Controller: %s", name_.c_str());
// }

geometry_msgs::msg::TwistStamped SolocoController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
  auto node = parent_.lock();
  std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  }

  double lookahead_dist = max_lookahead_dist_;
  lookahead_dist = fabs(robot_speed.linear.x) / max_speed_ * max_lookahead_dist_;

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return std::hypot(ps.pose.position.x- robot_pose.pose.position.x, 
        ps.pose.position.y - robot_pose.pose.position.y) >= lookahead_dist;
    }
  );

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  }

  geometry_msgs::msg::PoseStamped subgoal_pose;
  auto idx = std::distance(transformed_plan.poses.begin(), goal_pose_it);
  subgoal_pose = transformed_plan.poses[idx];
  subgoal_pose.header.frame_id = "locobot/odom";
  subgoal_pose.header.stamp = node->get_clock()->now();

  auto goal_msg = soloco_interfaces::action::NavigateToXYGoal::Goal();
  goal_msg.goal = subgoal_pose;

  auto send_goal_future = soloco_client_ptr_->async_send_goal(goal_msg);

  // Block until goal is accepted
  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Failed to get goal handle");
    // Handle the error...
    return geometry_msgs::msg::TwistStamped();  // Return an empty message
  }

  auto result_future = soloco_client_ptr_->async_get_result(goal_handle);

  // Block until result is ready
  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(logger_, "Result is null");
    // Handle the error...
    return geometry_msgs::msg::TwistStamped();  // Return an empty message
  }

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node->get_clock()->now();
  cmd_vel.twist = result.result->command_velocity;

  return cmd_vel;
}

void SolocoController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void SolocoController::setSpeedLimit(const double& limit, const bool& is_percentage)
{
  if (is_percentage) {
    max_speed_ = max_speed_ * limit / 100.0;
  } else {
    max_speed_ = limit;
  }
}

}  // namespace nav2_soloco_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_soloco_controller::SolocoController, nav2_core::Controller)