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

#include "pedsim_relay/pedsim_relay_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

namespace pedsim {
PedsimRelayNode::PedsimRelayNode() : Node("pedsim_relay_node") {
  std::string costmap_topic, detected_agents_topic, odom_topic;
  this->declare_parameter<std::string>("odom_topic", "locobot/odom");
  this->declare_parameter<std::string>("pub_frame_id", "locobot/odom");
  this->declare_parameter<std::string>("costmap_topic", "global_costmap/costmap"); 
  this->declare_parameter<std::string>("detected_agents_topic", "human/simulated_agents");
  this->declare_parameter<double>("field_of_view", 2.0944);
  this->declare_parameter<int>("max_num_agents", 5);
  this->get_parameter("odom_topic", odom_topic);
  this->get_parameter("pub_frame_id", pub_frame_id_);
  this->get_parameter("costmap_topic", costmap_topic);
  this->get_parameter("detected_agents_topic", detected_agents_topic);
  this->get_parameter("field_of_view", field_of_view_);
  this->get_parameter("max_num_agents", max_num_agents_);
  // Subscribe to robot odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&PedsimRelayNode::odomCallback, this, std::placeholders::_1));
  // Subscribe to simulated agents
  pedsim_sub_ = create_subscription<pedsim_msgs::msg::AgentStates>(
      "pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS(),
      std::bind(&PedsimRelayNode::agentsCallback, this, std::placeholders::_1));
  // Subscribe to local costmap topic
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic, 10,
      std::bind(&PedsimRelayNode::costmapCallback, this, std::placeholders::_1));
  // Publish filtered agents
  det_agents_pub_ = this->create_publisher<soloco_interfaces::msg::TrackedAgents>(detected_agents_topic, 10);
}

void PedsimRelayNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg){
  robot_pose_ = msg->pose.pose;
  tf2::Quaternion q(
    robot_pose_.orientation.x, 
    robot_pose_.orientation.y, 
    robot_pose_.orientation.z, 
    robot_pose_.orientation.w);
  // Convert quaternion to roll, pitch, yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  robot_theta_ = yaw;
}

void PedsimRelayNode::agentsCallback(
    const pedsim_msgs::msg::AgentStates::SharedPtr msg) {
  soloco_interfaces::msg::TrackedAgents tracked_agents_;
  std::map<double, soloco_interfaces::msg::TrackedAgent> closest_agents;
  int count = 0;
  for (const auto &actor : msg->agent_states) {
    double pos_x = actor.pose.position.x;
    double pos_y = actor.pose.position.y;
    // Check if agent is insde local costmap and fov
    bool occ_flag = check_agent_in_fov_and_costmap(pos_x, pos_y);
    if (occ_flag) {
      // Calculate robot to agent distance
      double agent_distance = std::hypot(robot_pose_.position.x - pos_x, robot_pose_.position.y - pos_y);
      geometry_msgs::msg::PoseStamped ps_;
      soloco_interfaces::msg::TrackedAgent agent_;
      ps_.header.frame_id = pub_frame_id_;
      ps_.header.stamp = now();
      ps_.pose.position = actor.pose.position;
      ps_.pose.orientation = actor.pose.orientation;
      agent_.current_pose = ps_;
      agent_.track_id = count;
      closest_agents.insert(std::make_pair(agent_distance, agent_));
    }
    count++;
  }
  // Publish agents
  auto it = closest_agents.begin();
  for (int i = 0; i < max_num_agents_ && it != closest_agents.end(); ++i, ++it) {
    tracked_agents_.agents.push_back(it->second);
  }
  tracked_agents_.header.frame_id = pub_frame_id_;
  tracked_agents_.header.stamp = now();
  det_agents_pub_->publish(tracked_agents_);
}

void PedsimRelayNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg) {
  map_width = costmap_msg->info.width;
  map_height = costmap_msg->info.height;
  map_origin_x = costmap_msg->info.origin.position.x;
  map_origin_y = costmap_msg->info.origin.position.y;
  map_resolution = costmap_msg->info.resolution;
}

bool PedsimRelayNode::check_agent_in_fov_and_costmap(
    double agent_x, double agent_y) {
  // Calculate angle to agent
  double angle_to_agent = std::atan2(agent_y - robot_pose_.position.y, agent_x - robot_pose_.position.x);
  angle_to_agent = angles::normalize_angle(angle_to_agent);
  // Check if the agent is within the FOV
  double angle_diff = angles::normalize_angle(angle_to_agent - robot_theta_);
  bool agent_in_fov = std::abs(angle_diff) <= field_of_view_ / 2;
  int grid_x = (int)((agent_x - map_origin_x) / map_resolution);
  int grid_y = (int)((agent_y - map_origin_y) / map_resolution);
  bool in_costmap = grid_x >= 0 && grid_x < map_width && grid_y >= 0 && grid_y < map_height;

  return agent_in_fov && in_costmap;
}

}; // namespace pedsim

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pedsim::PedsimRelayNode>());
  rclcpp::shutdown();

  return 0;
}