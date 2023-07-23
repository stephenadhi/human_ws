
// Copyright 2023
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

/* Author: stephenadhi stephenadhi@gmail.com */

#ifndef PEDSIM_RELAY_NODE_HPP_
#define PEDSIM_RELAY_NODE_HPP_

#include <algorithm>
#include <vector>
#include <utility>
#include <queue> 
#include <math.h>
#include <angles/angles.h>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>
#include <soloco_interfaces/msg/tracked_agent.hpp>
#include <soloco_interfaces/msg/tracked_agents.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pedsim {
class PedsimRelayNode : public rclcpp::Node {
public:
  PedsimRelayNode();

private:
  int map_width = 8;
  int map_height = 8;
  double map_origin_x = 0.0;
  double map_origin_y = 0.0;
  double map_resolution = 0.1;
  
  int max_num_agents_ = 5;
  double field_of_view_ = 2.0944;
  double robot_theta_ = 0.0;
  geometry_msgs::msg::Pose robot_pose_;
  std::string pub_frame_id_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void agentsCallback(const pedsim_msgs::msg::AgentStates::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool check_agent_in_fov_and_costmap(double agent_x, double agent_y);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr pedsim_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<soloco_interfaces::msg::TrackedAgents>::SharedPtr det_agents_pub_;
};
}; // namespace pedsim

#endif /* PEDSIM_RELAY_NODE_HPP_ */
