#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <soloco_interfaces/msg/ego_trajectory.hpp>

class EgoTrajectoryVisualizer : public rclcpp::Node
{
public:
  EgoTrajectoryVisualizer() : Node("ego_trajectory_visualizer")
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("odometry_markers", 10);
    ego_track_sub_ = this->create_subscription<soloco_interfaces::msg::EgoTrajectory>(
      "/robot/ego_trajectory", 10, std::bind(&EgoTrajectoryVisualizer::egoTrajectoryCallback, this, std::placeholders::_1));
  }

private:
  void egoTrajectoryCallback(const soloco_interfaces::msg::EgoTrajectory::SharedPtr msg)
  {
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < msg->track.size(); i++)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = msg->header.frame_id;
      marker.header.stamp = this->now();
      marker.ns = "trajectory";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = msg->track[i].pose.pose.position.x;
      marker.pose.position.y = msg->track[i].pose.pose.position.y;
      marker.pose.position.z = msg->track[i].pose.pose.position.z;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<soloco_interfaces::msg::EgoTrajectory>::SharedPtr ego_track_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoTrajectoryVisualizer>());
  rclcpp::shutdown();
  return 0;
}