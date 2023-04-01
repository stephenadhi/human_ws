#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
        this->declare_parameter<double>("max_interp_interval", 0.4);
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
        // Visualization marker publisher
        m_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_markers", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Setup marker array for visualization
        visualization_msgs::msg::MarkerArray markers;
        // Get ros parameters
        this->get_parameter("pub_frame_id", pub_frame_id);
        this->get_parameter("pub_frame_rate", pub_frame_rate);
        this->get_parameter("max_interp_interval", max_interp_interval);
        this->get_parameter("max_history_length", max_history_length);
        // Save msg to odometry array (EgoTrajectory.msg)
        soloco_interfaces::msg::EgoTrajectory traj;
        nav_msgs::msg::Odometry odom;
        odom.header = msg-> header;
        odom.child_frame_id = msg->child_frame_id;
        odom.pose = msg->pose;
        odom.twist = msg->twist;
        traj.track.push_back(odom);
        // If robot track length is bigger than max_history-length, remove oldest pose
        if(traj.track.size()>max_history_length){
            traj.track.erase(traj.track.begin());
        }
        // get time
        double now_time = now().seconds();
        rclcpp::Time rec_time(traj.track[0].header.stamp);
        // Interpolate robot historical poses for same interval between poses
        interpolate_robot_pos(traj, now_time, rec_time.seconds(), max_interp_interval, markers);
        // publish past trajectory and visualization markers
        traj.header.frame_id = msg->header.frame_id;
        traj.header.stamp = now();
        m_track_pub->publish(traj);
        m_marker_pub->publish(markers);
    }
    
    void interpolate_robot_pos(
            soloco_interfaces::msg::EgoTrajectory traj,
            double curr_time, 
            double det_time, 
            double max_interp_interval,
            visualization_msgs::msg::MarkerArray markers) {
        // Setup marker for visualization
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = traj.header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "robot_trajectory";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        // get number of steps depending on interpolation frequency, maximum size is history length
        int num_t_quan_steps = int((curr_time - det_time) / max_interp_interval);
        if(num_t_quan_steps > int(max_history_length)){
            num_t_quan_steps = int(max_history_length); 
        }
        // get interpolated time points
        Eigen::VectorXd inter_time_points = Eigen::VectorXd::LinSpaced(num_t_quan_steps, 0.0, max_interp_interval*num_t_quan_steps);
        // Get the current number of poses inside path message
        int num_poses = traj.track.size();
        // Vector for saving person historical track and recorded time
        Eigen::VectorXd x(num_poses), y(num_poses), t(num_poses);
        for (int k = 0; k < num_poses; k++){
            rclcpp::Time rec_time (traj.track[k].header.stamp);
            x[k] = traj.track[k].pose.pose.position.x;
            y[k] = traj.track[k].pose.pose.position.y;
            t[k] = rec_time.seconds();
        }
        // 
        for (int i = 0; i <= num_t_quan_steps; i++){
            int j = 0;
            while (j < t.size() - 1 && t[j + 1] < inter_time_points[i]){
            j++;
            double alpha = (inter_time_points[i] - t[j]) / (t[j + 1] - t[j]);
            double x_interp = x[j] * (1 - alpha) + x[j + 1] * alpha;
            double y_interp = y[j] * (1 - alpha) + y[j + 1] * alpha;
            traj.track[i].pose.pose.position.x = x_interp;
            traj.track[i].pose.pose.position.y = y_interp;
            // Visualize using markers
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = x_interp;
            marker.pose.position.y = y_interp;      
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            markers.markers.push_back(marker);
            }
        }
    }

private:
    // Declare variables
    std::string pub_frame_id;
    double pub_frame_rate, delay_tolerance, max_interp_interval;
    long unsigned int max_history_length;
    // Odometry susbcriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    // Robot track publisher
    rclcpp::Publisher<soloco_interfaces::msg::EgoTrajectory>::SharedPtr m_track_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;   
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotTrackPublisher>());
    rclcpp::shutdown();
  return 0;
}
