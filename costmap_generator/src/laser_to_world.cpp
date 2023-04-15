#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <laser_geometry/laser_geometry.hpp>
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanTransformNode : public rclcpp::Node {
public:
  LaserScanTransformNode()
      : Node("laserscan_transform_node") {
    // Initialize the tf2 buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<std::string>("output_topic", "map_point_cloud");
    this->declare_parameter<std::string>("laser_frame_id", "laser_link");
    this->declare_parameter<std::string>("map_frame_id", "map");

    // Get parameters
    this->get_parameter("scan_topic", scan_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("laser_frame_id", laser_frame_id_);
    this->get_parameter("map_frame_id", map_frame_id_);

    // Subscribe to the laser scan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10,
        std::bind(&LaserScanTransformNode::scanCallback, this,
                  std::placeholders::_1));
    // Advertise the transformed point cloud topic
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
}

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // Wait for the transform from laser_link to map
    auto transform = tf_buffer_->lookupTransform(
        map_frame_id_, laser_frame_id_, scan_msg->header.stamp,
        std::chrono::duration_cast<std::chrono::seconds>(std::chrono::seconds(1)));

    // Convert LaserScan to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i) {
      double range = scan_msg->ranges[i];
      if (std::isfinite(range) && range > scan_msg->range_min &&
          range < scan_msg->range_max) {
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        pcl::PointXYZ point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        point.z = 0.0;
        laser_cloud->push_back(point);
      }
    }
    laser_cloud->header.frame_id = laser_frame_id_;
    laser_cloud->header.stamp = pcl_conversions::toPCL(scan_msg->header).stamp;

    // Transform LaserScan to map frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*laser_cloud, *map_cloud, transform);

    // Publish PointCloud
    sensor_msgs::msg::PointCloud2::SharedPtr map_cloud_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud, *map_cloud_msg);
    map_cloud_msg->header.frame_id = map_frame_id_;
    map_cloud_msg->header.stamp = scan_msg->header.stamp;
    map_cloud_pub_->publish(*map_cloud_msg);
  }

  private:
    std::string scan_topic_;
    std::string output_topic_;
    std::string laser_frame_id_;
    std::string map_frame_id_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::shutdown();
  return 0;
}