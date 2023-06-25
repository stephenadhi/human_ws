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

  // Configure composed objects
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());

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

// geometry_msgs::msg::TwistStamped SolocoController::computeVelocityCommands(
//   const geometry_msgs::msg::PoseStamped & robot_pose,
//   const geometry_msgs::msg::Twist & robot_speed,
//   nav2_core::GoalChecker * goal_checker)
// {
// #ifdef BENCHMARK_TESTING
//   auto start = std::chrono::system_clock::now();
// #endif

//   std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
//   nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

//   // Update for the current goal checker's state
//   geometry_msgs::msg::Pose pose_tolerance;
//   geometry_msgs::msg::Twist vel_tolerance;
//   if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
//     RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
//   } else {
//     goal_dist_tol_ = pose_tolerance.position.x;
//   }

// #ifdef BENCHMARK_TESTING
//   auto end = std::chrono::system_clock::now();
//   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//   RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
// #endif

//   if (visualize_) {
//     visualize(std::move(transformed_plan));
//   }

//   return cmd;
// }

void SolocoController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

}  // namespace nav2_soloco_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_soloco_controller::SolocoController, nav2_core::Controller)