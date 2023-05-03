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
/* Author: jginesclavero jonatan.gines@urjc.es */

/* Author: stephenadhi stephenadhi@gmail.com */

#include "pedsim_relay/pedsim_relay_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

namespace pedsim {
PedsimRelayNode::PedsimRelayNode() : Node("pedsim_relay_node") {
  // Subscribe to simulated agents
  sub_ = create_subscription<pedsim_msgs::msg::AgentStates>(
      "pedsim_simulator/simulated_agents", rclcpp::SensorDataQoS(),
      std::bind(&PedsimRelayNode::agentsCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<soloco_interfaces::msg::TrackedAgents>("human/simulated_agents", rclcpp::SensorDataQoS());
}

void PedsimRelayNode::agentsCallback(
    const pedsim_msgs::msg::AgentStates::SharedPtr msg) {
  soloco_interfaces::msg::TrackedAgents agents_;
  for (const auto &actor : msg->agent_states) {
    double pos_x = actor.pose.position.x;
    double pos_y = actor.pose.position.y;
    bool occ_flag = check_agent_in_local_costmap(pos_x, pos_y);
    if (occ_flag) {
      geometry_msgs::msg::PoseStamped ps_;
      soloco_interfaces::msg::TrackedAgent agent_;
      ps_.header.frame_id = "map";
      ps_.header.stamp = now();
      ps_.pose.position.x = pos_x;
      ps_.pose.position.y = pos_y;
      tf2::Quaternion qt;
      double theta = std::atan2(actor.pose.position.y, actor.pose.position.x);
      qt.setRPY(0.0, 0.0, theta);
      qt.normalize();
      ps_.pose.orientation = tf2::toMsg(qt);
      agent_.current_pose = ps_;
      agents_.agents.push_back(agent_);
    }
    pub_->publish(agents_);
  }
}

void PedsimRelayNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg) {
  map_width = costmap_msg->info.width;
  map_height = costmap_msg->info.height;
  map_origin_x = costmap_msg->info.origin.position.x;
  map_origin_y = costmap_msg->info.origin.position.y;
  map_resolution = costmap_msg->info.resolution;
}

bool PedsimRelayNode::check_agent_in_local_costmap(
    double agent_x, double agent_y) {
  int grid_x = (int)((agent_x - map_origin_x) / map_resolution);
  int grid_y = (int)((agent_y - map_origin_y) / map_resolution);
  if (grid_x >= 0 && grid_x < map_width && grid_y >= 0 && grid_y < map_height) {
    return true;
  }
  else { return false; }
}

}; // namespace pedsim

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pedsim::PedsimRelayNode>());
  rclcpp::shutdown();

  return 0;
}