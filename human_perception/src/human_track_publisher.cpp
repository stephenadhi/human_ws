#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <soloco_interfaces/msg/tracked_person.hpp>
#include <soloco_interfaces/msg/tracked_persons.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <experimental/optional>

class HumanTrackPublisher : public rclcpp::Node {
public:    
    HumanTrackPublisher()
        : Node("human_track_publisher") {
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.reliable();
        qos.durability_volatile();
        // Create zed human pose subscriber
        m_zed_sub = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
                    "/zed2/zed_node/obj_det/objects", qos,
                    std::bind(&HumanTrackPublisher::zedCallback, this, std::placeholders::_1) );
        // Create path publisher
        m_human_track_interpolated_pub = this->create_publisher<soloco_interfaces::msg::TrackedPersons>(
                        "/human/interpolated_track", qos);
        // Create tf buffer and transform listener   
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    }

private:
    void zedCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) {
        bool new_object = true;
        geometry_msgs::msg::PoseStamped pose_out;
        // Do for all detected humans
        for(long unsigned int count=0; count < msg->objects.size(); count++){
            // Position in camera frame
            double tx = msg->objects[count].position[0];
            double ty = msg->objects[count].position[1];
            double tz = msg->objects[count].position[2];
            // Save to current poseStamped message format
            geometry_msgs::msg::PoseStamped currPose;
            currPose.header = msg->header;
            currPose.pose.position.x = tx;
            currPose.pose.position.y = ty;
            currPose.pose.position.z = tz;
            // If tf transform exists, convert current pose to world space
            pose_out = pose_transform(currPose, msg->header.frame_id, pub_frame_id);
            
            // If we already know the person id
            for(long unsigned int k=0; k < people.tracks.size(); k++){
                if(people.tracks[k].track_id == msg->objects[count].label_id){
                    people.tracks[k].track.poses.push_back(pose_out);
                    // If tracked_person track length is bigger than max_history-length, remove oldest pose
                    if(people.tracks[k].track.poses.size()>max_history_length){
                        people.tracks[k].track.poses.erase(people.tracks[k].track.poses.begin());
                    }
                    new_object = false;
                    break;
                }
            }
            // Add to people vector if the person is new
            if(new_object){
                // create a new person object;
                soloco_interfaces::msg::TrackedPerson tracked_person;
                tracked_person.track_id = msg->objects[count].label_id;
                tracked_person.tracking_state = msg->objects[count].tracking_state;
                tracked_person.track.poses.push_back(pose_out);
                // Save current pose
                tracked_person.pose.pose = pose_out.pose;
                //tracked_person.pose.covariance = msg->objects[count].position_covariance;
                // Save to people vector
                people.tracks.push_back(tracked_person);
            }
        }
        
        // publish track history for all persons in one ROS message
        people.header.frame_id = pub_frame_id;
        people.header.stamp = now();
        m_human_track_interpolated_pub->publish(people);
    }
    
    geometry_msgs::msg::PoseStamped pose_transform(const geometry_msgs::msg::PoseStamped & obj, const std::string & output_frame, const std::string & input_frame)
    {   using namespace std::chrono_literals;
        geometry_msgs::msg::PoseStamped transformed_obj;
        try {
            // rclcpp::Time now = this->get_clock()->now();
            auto tf = m_tf_buffer->lookupTransform(output_frame, input_frame, tf2::TimePointZero);
            tf2::doTransform(obj, transformed_obj, tf);
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            
        }
        return transformed_obj;
    }

private:
    // tf buffer and transform listener
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener {nullptr};
    // Human pose subscriber
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr m_zed_sub;
    // Human track publisher
    rclcpp::Publisher<soloco_interfaces::msg::TrackedPersons>::SharedPtr m_human_track_interpolated_pub;
    // people vector as cache for detected humans
    soloco_interfaces::msg::TrackedPersons people;
    // initialize variables
    long unsigned int max_history_length = 12;
    std::string pub_frame_id = "map";
};  

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanTrackPublisher>());
    rclcpp::shutdown();
  return 0;
}
