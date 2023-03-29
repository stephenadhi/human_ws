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

using namespace std::chrono_literals;

class HumanTrackPublisher : public rclcpp::Node {
public:    
    HumanTrackPublisher()
        : Node("human_track_publisher") {
        // QoS settings
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.best_effort();
        qos.durability_volatile();
        // Declare parameters
        std::string human_track_topic, detected_obj_topic;
        this->declare_parameter<std::string>("detected_obj_topic", "/zed2/zed_node/obj_det/objects");
        this->declare_parameter<std::string>("human_track_topic", "/human/track");
        this->declare_parameter<std::string>("pub_frame_id", "map");
        this->declare_parameter<double>("pub_frame_rate", 15.0);
        this->declare_parameter<int64_t>("max_history_length", 12);
        this->declare_parameter<double>("delay_tolerance", 3.0);
        this->get_parameter("detected_obj_topic", detected_obj_topic);
        this->get_parameter("human_track_topic", human_track_topic);

        // Create zed detected objects subscriber
        m_zed_sub = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
                    detected_obj_topic, qos,
                    std::bind(&HumanTrackPublisher::zedCallback, this, std::placeholders::_1) );
        // Create human track publisher
        m_human_track_interpolated_pub = this->create_publisher<soloco_interfaces::msg::TrackedPersons>(
                    human_track_topic, qos);
        // Create tf buffer and transform listener   
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    }

private:
    void zedCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) {
        // Initialize new_object to true
        bool new_object = true;
        // get ros parameters
        this->get_parameter("pub_frame_id", pub_frame_id);
        this->get_parameter("pub_frame_rate", pub_frame_rate);
        this->get_parameter("delay_tolerance", delay_tolerance);
        this->get_parameter("max_history_length", max_history_length);
        double tolerance = 1/pub_frame_rate + delay_tolerance;
        
        // variable to save transformed pose in the pub_frame_id space
        geometry_msgs::msg::PoseStamped pose_out;
        // Loop all detected objects
        for(long unsigned int count=0; count < msg->objects.size(); count++){
            // Get the person id and tracking state
            soloco_interfaces::msg::TrackedPerson tracked_person;
            tracked_person.track_id = msg->objects[count].label_id;
            tracked_person.tracking_state = 1; //msg->objects[count].tracking_state;
            // Only consider valid tracking
            if(tracked_person.tracking_state == 1 && msg->objects[count].label == "Person"){
                // Position in camera frame
                double tx = msg->objects[count].position[0];
                double ty = msg->objects[count].position[1];
                // Save to current poseStamped message format
                geometry_msgs::msg::PoseStamped currPose;
                currPose.header = msg->header;
                currPose.pose.position.x = tx;
                currPose.pose.position.y = ty;
                currPose.pose.position.z = 0.0;
                currPose.pose.orientation.x = 0.0;
                currPose.pose.orientation.y = 0.0;
                currPose.pose.orientation.z = 0.0;
                currPose.pose.orientation.w = 1.0;
                // If tf transform exists, convert current pose to world space
                pose_out = pose_transform(currPose, pub_frame_id, msg->header.frame_id);
                long unsigned int people_count = people.tracks.size();
                for(long unsigned int person_idx=0; person_idx < people_count; person_idx++){
                // Get last detection time for person k in people vector
                rclcpp::Time det_time(people.tracks[person_idx].header.stamp);
                        // Check whether person is known by id
                        if(people.tracks[person_idx].track_id == msg->objects[count].label_id){
                        // RCLCPP_INFO(this->get_logger(), "Matched person ID %i.", people.tracks[person_idx].track_id);
                        // Add person to people vector
                        people.tracks[person_idx].track.poses.push_back(pose_out);  
                        // stamp the time of update
                        people.tracks[person_idx].header.stamp = now();
                        new_object = false;
                    }
                    // If tracked_person track length is bigger than max_history-length, remove oldest pose
                    if(people.tracks[person_idx].track.poses.size()>max_history_length){
                        people.tracks[person_idx].track.poses.erase(people.tracks[person_idx].track.poses.begin());
                        // RCLCPP_INFO(this->get_logger(), "Track too long, removing oldest pose");
                    }
                    // Delete older person object longer than tolerance time
                    double time_diff = now().seconds() - det_time.seconds();
                    if (time_diff > tolerance){
                        people.tracks.erase(people.tracks.begin() + person_idx);
                        person_idx--;
                        people_count--;
                        RCLCPP_INFO(this->get_logger(), "Erased old data from %f seconds ago.", time_diff);
                    }
                }
                if(new_object){
                    // Push curent pose to history vector;
                    tracked_person.track.poses.push_back(pose_out);
                    // Save current pose
                    tracked_person.current_pose.pose = pose_out.pose;
                    // Save to people vector
                    people.tracks.push_back(tracked_person);
                    people.tracks.back().header.stamp = now();
                    RCLCPP_INFO(this->get_logger(), "New person detected! I heard there are %li people", people.tracks.size());
                }
            }
        // publish track history for all persons in one ROS message
        people.header.frame_id = pub_frame_id;
        people.header.stamp = now();
        m_human_track_interpolated_pub->publish(people);
        }
    }
    
    geometry_msgs::msg::PoseStamped pose_transform(const geometry_msgs::msg::PoseStamped & obj, const std::string & output_frame, const std::string & input_frame){   
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
    // Declare variables
    std::string pub_frame_id;
    double pub_frame_rate, delay_tolerance;
    long unsigned int max_history_length;
    // tf buffer and transform listener
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener {nullptr};
    // Human pose subscriber
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr m_zed_sub;
    // Human track publisher
    rclcpp::Publisher<soloco_interfaces::msg::TrackedPersons>::SharedPtr m_human_track_interpolated_pub;
    // people vector as cache for detected humans
    soloco_interfaces::msg::TrackedPersons people;
};  

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanTrackPublisher>());
    rclcpp::shutdown();
  return 0;
}
