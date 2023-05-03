
// Copyright 2020 Intelligent Robotics Lab
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

/* Original Author: jginesclavero jonatan.gines@urjc.es */

/* Modified by: stephenadhi stephenadhi@gmail.com */

#ifndef HUMAN_TF2_PUBLISHER_HPP_
#define HUMAN_TF2_PUBLISHER_HPP_

#include <algorithm>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <soloco_interfaces/msg/tracked_person.hpp>
#include <soloco_interfaces/msg/tracked_persons.hpp>

using namespace geometry_msgs::msg;

namespace human_tf2_publisher {
class HumanTF2Publisher : public rclcpp::Node {
public:
  HumanTF2Publisher(const std::string &name);
  void step();

private:
  void agentsCallback(const soloco_interfaces::msg::TrackedPersons::SharedPtr msg);
  bool getTFfromAgent(soloco_interfaces::msg::TrackedPerson actor, TransformStamped &tf);

  rclcpp::Subscription<soloco_interfaces::msg::TrackedPersons>::SharedPtr sub_;

  soloco_interfaces::msg::TrackedPersons::SharedPtr states_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
}; // namespace pedsim

#endif /* HUMAN_TF2_PUBLISHER_HPP_ */
