#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <soloco_interfaces/msg/ego_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define RAD2DEG 57.295779513

class RobotTrackPublisher : public rclcpp::Node {
public:
    RobotTrackPublisher()
        : Node("robot_track_publisher") {
        // QoS settings
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.reliable();
        qos.durability_volatile();
        // Declare parameters
        std::string odom_topic, robot_track_topic;
        this->declare_parameter<std::string>("odom_topic", "/zed2/zed_node/odom");
        this->declare_parameter<std::string>("robot_track_topic", "/robot/ego_trajectory");
        this->declare_parameter<std::string>("pub_frame_id", "map");
        this->declare_parameter<double>("pub_frame_rate", 15.0);
        this->declare_parameter<int64_t>("max_history_length", 12);
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("robot_track_topic", robot_track_topic);
        // Create odom subscriber
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                    odom_topic, qos,
                    std::bind(&RobotTrackPublisher::odomCallback, this, std::placeholders::_1) );
        // Create track publisher
        m_track_pub = this->create_publisher<soloco_interfaces::msg::EgoTrajectory>(
                        robot_track_topic, qos);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get ros parameters
        this->get_parameter("pub_frame_id", pub_frame_id);
        this->get_parameter("pub_frame_rate", pub_frame_rate);
        this->get_parameter("max_history_length", max_history_length);
        // Save current pose
        // nav_msgs::msg::Odometry odom;
        // create path variable to save odom pose to path (trajectory)
        soloco_interfaces::msg::EgoTrajectory traj;
        
        traj.track.push_back(*msg);
        // If tracked_person track length is bigger than max_history-length, remove oldest pose
        if(traj.track.size()>max_history_length){
            traj.track.erase(traj.track.begin());
        }
        // publish trajectory
        traj.header.frame_id = msg->header.frame_id;
        traj.header.stamp = now();
        m_track_pub->publish(traj);
    }

private:
    // Declare variables
    std::string pub_frame_id;
    double pub_frame_rate, delay_tolerance;
    long unsigned int max_history_length;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Publisher<soloco_interfaces::msg::EgoTrajectory>::SharedPtr m_track_pub;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotTrackPublisher>());
    rclcpp::shutdown();
  return 0;
}
