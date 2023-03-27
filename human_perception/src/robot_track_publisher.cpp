#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define RAD2DEG 57.295779513

class OdomTrajectory : public rclcpp::Node {
public:
    // create path variable to save odom pose to path (trajectory)
    nav_msgs::msg::Path traj;
    // minimum distance in metres to add odom pose to path 
    double concatenate_threshold = 1.0;
    // initialize variables
    geometry_msgs::msg::PoseStamped prevPose;
    double dist = 0.0;
    
    OdomTrajectory()
        : Node("trajectory_visualizer") {
        /* Note: it is very important to use a QOS profile for the subscriber that is compatible
         * with the QOS profile of the publisher.
         */// https://github.com/ros2/ros2/wiki/About-Quality-of-Service-Settings
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.reliable();
        qos.durability_volatile();
        // Create odom subscriber
        mOdomSub = this->create_subscription<nav_msgs::msg::Odometry>(
                    "odometry/filtered", qos,
                    std::bind(&OdomTrajectory::odomCallback, this, std::placeholders::_1) );
        // Create path publisher
        mTrajPub = this->create_publisher<nav_msgs::msg::Path>(
                        "visualization/ego_trajectory", qos);
    }

private:
    // Function to calculate distance
    double distance(double x2, double y2, double x1, double y1){
        // Calculating distance
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Camera position in map frame
        //double tx = msg->pose.pose.position.x;
        //double ty = msg->pose.pose.position.y;
        //double tz = msg->pose.pose.position.z;
        // Orientation quaternion
        tf2::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
        // 3x3 Rotation matrix from quaternion
        tf2::Matrix3x3 m(q);
        // Roll Pitch and Yaw from rotation matrix
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // Output the received message
        //RCLCPP_INFO(get_logger(), "Received odometry in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f - Timestamp: %u.%u sec ",
        //         msg->header.frame_id.c_str(),
        //         tx, ty, tz,
        //         roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG,
        //            msg->header.stamp.sec,msg->header.stamp.nanosec);
        
        // Save current pose
        geometry_msgs::msg::PoseStamped currPose;
        currPose.header = msg->header;
        currPose.pose = msg->pose.pose;
            
        // If traj empty push initial pose
        if(traj.poses.empty()){
            traj.poses.push_back(currPose);
            // set currentPose as prevPose
            prevPose.pose = currPose.pose;
        }
        // concatenate to trajectory if dist > x metres
        else{
            // calculate distance between prevPose and CurrPose
            dist = distance(currPose.pose.position.x, currPose.pose.position.y,
                prevPose.pose.position.x, prevPose.pose.position.y);
            if(dist > concatenate_threshold){ 
                traj.poses.push_back(currPose);
                // set currentPose as prevPose
                prevPose.pose = currPose.pose;
            }
        }
        // publish trajectory
        traj.header.frame_id = msg->header.frame_id;
        traj.header.stamp = now();
        mTrajPub->publish(traj);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mOdomSub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mTrajPub;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTrajectory>());
    rclcpp::shutdown();
  return 0;
}
