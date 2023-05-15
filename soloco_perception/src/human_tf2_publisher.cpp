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
/* Author: jginesclavero jonatan.gines@urjc.es */

/* Mantainer: jginesclavero jonatan.gines@urjc.es */

/* Modified by: stephenadhi stephenadhi@gmail.com */

#include "soloco_perception/human_tf2_publisher.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace geometry_msgs::msg;

namespace human_tf2_publisher {
HumanTF2Publisher::HumanTF2Publisher(const std::string &name) : Node(name), states_() {
  // Subscribe to tracked people
  sub_ = create_subscription<soloco_interfaces::msg::TrackedPersons>(
      "human/interpolated_history", rclcpp::SensorDataQoS(),
      std::bind(&HumanTF2Publisher::agentsCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void HumanTF2Publisher::agentsCallback(
    const soloco_interfaces::msg::TrackedPersons::SharedPtr msg) {
  states_ = msg;
}

bool HumanTF2Publisher::getTFfromAgent(soloco_interfaces::msg::TrackedPerson actor,
                               TransformStamped &tf) {
  double theta = std::atan2(actor.current_pose.pose.position.y, actor.current_pose.pose.position.x);
  tf2::Quaternion qt;
  qt.setRPY(0.0, 0.0, theta);
  qt.normalize();
  tf.transform.translation.x = actor.current_pose.pose.position.x;
  tf.transform.translation.y = actor.current_pose.pose.position.y;
  tf.transform.translation.z = actor.current_pose.pose.position.z;
  tf.transform.rotation = tf2::toMsg(qt);
  if (std::isnan(theta) || std::isnan(tf.transform.translation.x) ||
      std::isnan(tf.transform.translation.y) ||
      std::isnan(tf.transform.translation.z)) {
    return false;
  }
  return true;
}

void HumanTF2Publisher::step() {
  rclcpp::Rate loop_rate(50ms);
  while (rclcpp::ok()) {
    if (states_ != NULL) {
      for (auto actor : states_->tracks) {
        TransformStamped agent_tf;
        if (getTFfromAgent(actor, agent_tf)) {
          agent_tf.header.frame_id = "map";
          agent_tf.header.stamp = now();
          agent_tf.child_frame_id = "agent_" + std::to_string(actor.track_id);
          tf_broadcaster_->sendTransform(agent_tf);
        }
      }
    }
    rclcpp::spin_some(shared_from_this());
    loop_rate.sleep();
  }
}

} // namespace human_tf2_publisher

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto human_tf2 = std::make_shared<human_tf2_publisher::HumanTF2Publisher>("human_tf2_publisher");
  human_tf2->step();
  rclcpp::shutdown();

  return 0;
}