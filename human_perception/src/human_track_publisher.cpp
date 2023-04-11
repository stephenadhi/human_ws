#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
        qos.reliable();
        qos.durability_volatile();
        // Declare parameters
        std::string human_track_topic, detected_obj_topic;
        this->declare_parameter<std::string>("detected_obj_topic", "/zed2/zed_node/obj_det/objects");
        this->declare_parameter<std::string>("human_track_topic", "/human/track");
        this->declare_parameter<std::string>("pub_frame_id", "map");
        this->declare_parameter<double>("pub_frame_rate", 15.0);
        this->declare_parameter<double>("max_interp_interval", 0.4);
        this->declare_parameter<int64_t>("max_history_length", 12);
        this->declare_parameter<double>("delay_tolerance", 3.0);
        this->declare_parameter<bool>("visualize_bbox", true);
        this->get_parameter("detected_obj_topic", detected_obj_topic);
        this->get_parameter("human_track_topic", human_track_topic);

        // Create zed detected objects subscriber
        m_zed_sub = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
                    detected_obj_topic, qos,
                    std::bind(&HumanTrackPublisher::zedCallback, this, std::placeholders::_1) );
        // Create human track publisher
        m_human_track_interpolated_pub = this->create_publisher<soloco_interfaces::msg::TrackedPersons>(
                    human_track_topic, qos);
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("human_markers", 10);
        bbox_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("human_bounding_boxes", 10);
        // Create tf buffer and transform listener   
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_transform_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
    }

private:
    void zedCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) {
        // get ros parameters
        this->get_parameter("pub_frame_id", pub_frame_id);
        this->get_parameter("pub_frame_rate", pub_frame_rate);
        this->get_parameter("max_interp_interval", max_interp_interval);
        this->get_parameter("delay_tolerance", delay_tolerance);
        this->get_parameter("max_history_length", max_history_length);
        this->get_parameter("visualize_bbox", visualize_bbox);
        double tolerance = 1/pub_frame_rate + delay_tolerance;

        // Setup new visualization marker arrays
        visualization_msgs::msg::MarkerArray human_markers;
        visualization_msgs::msg::MarkerArray human_bboxes;
        
        // Loop through all detected objects
        for(long unsigned int count=0; count < msg->objects.size(); count++){
            // Get the person id and tracking state
            soloco_interfaces::msg::TrackedPerson tracked_person;
            tracked_person.track_id = msg->objects[count].label_id;
            tracked_person.tracking_state = 1; //msg->objects[count].tracking_state;
            // Only consider valid tracking
            if(tracked_person.tracking_state == 1 && msg->objects[count].label == "Person"){
                // Position in camera frame
                double person_x_pos = msg->objects[count].position[0];
                double person_y_pos = msg->objects[count].position[1];
                // Save to current poseStamped message format
                geometry_msgs::msg::PoseStamped currPose;
                currPose.header = msg->header;
                currPose.pose.position.x = person_x_pos;
                currPose.pose.position.y = person_y_pos;
                currPose.pose.position.z = 0.0;
                currPose.pose.orientation.x = 0.0;
                currPose.pose.orientation.y = 0.0;
                currPose.pose.orientation.z = 0.0;
                currPose.pose.orientation.w = 1.0;
                
                // Get bounding box dimensions
                //tracked_person.bbox_x_length = msg->objects[count].dimensions_3d[0];
                //tracked_person.bbox_y_length = msg->objects[count].dimensions_3d[1];
                //tracked_person.bbox_z_length = msg->objects[count].dimensions_3d[2];
                
                // If tf transform exists, convert current pose to world space
                if (auto pose_out = pose_transform(currPose, pub_frame_id, msg->header.frame_id)){
                    // variable to save transformed pose in the pub_frame_id space
                    geometry_msgs::msg::PoseStamped pose_world = pose_out.value();
                    
                    if (visualize_bbox){ visualize_person_bbox(tracked_person, pose_world, human_bboxes); }
                    
                    long unsigned int people_count = people.tracks.size();
                    bool new_object = check_and_update_people_vector(tracked_person, pose_world, people_count, tolerance, human_markers);

                    if(new_object){
                        // Push curent pose to history vector and stamp detection time
                        tracked_person.track.poses.push_back(pose_world);
                        tracked_person.track.poses.back().header.stamp = now();
                        // Save current pose
                        tracked_person.current_pose.pose = pose_world.pose;
                        // Save to people vector and stamp time of person track
                        people.tracks.push_back(tracked_person);
                        RCLCPP_INFO(this->get_logger(), "New person detected! I heard there are %li people", people.tracks.size());
                    }
                }
            }
        // publish track history for all persons in one ROS message
        people.header.frame_id = pub_frame_id;
        people.header.stamp = now();
        m_human_track_interpolated_pub->publish(people);
        
        marker_pub->publish(human_markers);
        bbox_pub->publish(human_bboxes);
        }
    }
    
    std::experimental::optional<geometry_msgs::msg::PoseStamped> pose_transform(
            const geometry_msgs::msg::PoseStamped & obj, 
            const std::string & output_frame, 
            const std::string & input_frame){   
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
    
    void interpolate_person_pos(
            soloco_interfaces::msg::TrackedPerson person,
            double curr_time, 
            double det_time, 
            double max_interp_interval,
            visualization_msgs::msg::MarkerArray markers) {
        // Setup marker for visualization
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = pub_frame_id;
        marker.header.stamp = this->now();
        marker.ns = "human_trajectory";
        marker.id = person.track_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        // get number of steps depending on interpolation frequency, maximum size is history length
        int num_t_quan_steps = int((curr_time - det_time) / max_interp_interval);
        if(num_t_quan_steps > int(max_history_length)){
            num_t_quan_steps = int(max_history_length); 
        }
        RCLCPP_INFO(this->get_logger(), "num_t_quan_steps %i.", num_t_quan_steps + 1);
        // get interpolated time points
        Eigen::VectorXd inter_time_points = Eigen::VectorXd::LinSpaced(num_t_quan_steps + 1, 0.0, max_interp_interval*num_t_quan_steps);
        // Get the current number of poses inside path message
        int num_poses = person.track.poses.size();
        // Vector for saving person historical track and recorded time
        Eigen::VectorXd x(num_poses), y(num_poses), t(num_poses);
        for (int k = 0; k < num_poses; k++){
            rclcpp::Time rec_time (person.track.poses[k].header.stamp);
            x[k] = person.track.poses[k].pose.position.x;
            y[k] = person.track.poses[k].pose.position.y;
            t[k] = rec_time.seconds();
        }
        // Rewrite person historical track and recorded time
        for (int i = 0; i <= num_t_quan_steps; i++){
            int j = 0;
            while (j < t.size() - 1 && t[j + 1] < inter_time_points[i]){
            j++;
            double alpha = (inter_time_points[i] - t[j]) / (t[j + 1] - t[j]);
            double x_interp = x[j] * (1 - alpha) + x[j + 1] * alpha;
            double y_interp = y[j] * (1 - alpha) + y[j + 1] * alpha;
            person.track.poses[i].pose.position.x = x_interp;
            person.track.poses[i].pose.position.y = y_interp;
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
    
    bool check_and_update_people_vector(
            soloco_interfaces::msg::TrackedPerson tracked_person,
            geometry_msgs::msg::PoseStamped pose_world,
            long unsigned int people_count,
            double tolerance,
            visualization_msgs::msg::MarkerArray human_markers) {
        // initialize new object to true
        bool new_object = true; 
        // Loop through all person in people vector
        for(long unsigned int person_idx=0; person_idx < people_count; person_idx++){
            soloco_interfaces::msg::TrackedPerson person = people.tracks[person_idx];
            // Get first detection time for person k in people vector
            rclcpp::Time det_time(person.track.poses[0].header.stamp);
            // Check whether person is known by id. MATCHED ID --> Add person pose to person track
            if(person.track_id == tracked_person.track_id){
                // RCLCPP_INFO(this->get_logger(), "Matched person ID %i.", people.tracks[person_idx].track_id);
                person.track.poses.push_back(pose_world);  
                // If tracked_person track length is bigger than max_history-length, remove oldest pose
                if(person.track.poses.size()>max_history_length){
                    person.track.poses.erase(person.track.poses.begin());
                    // RCLCPP_INFO(this->get_logger(), "Track too long, removing oldest pose");
                }
                // Interpolate person historical poses for exact time diff between poses
                if (person.track.poses.size() > 1){
                    interpolate_person_pos(person, now().seconds(), det_time.seconds(), max_interp_interval, human_markers);
                }
                // stamp the time of person track update
                new_object = false;
            }
            // PRUNE: Delete older person object longer than tolerance time
            double time_diff = now().seconds() - det_time.seconds();
            if (time_diff > tolerance){
                people.tracks.erase(people.tracks.begin() + person_idx);
                person_idx--;
                people_count--;
                RCLCPP_INFO(this->get_logger(), "Erased old data from %f seconds ago.", time_diff);
            }
        }
        return new_object;    
    }
    
    void visualize_person_bbox(
            soloco_interfaces::msg::TrackedPerson tracked_person, 
            geometry_msgs::msg::PoseStamped pose_world,
            visualization_msgs::msg::MarkerArray human_bboxes){
        // Setup box marker center point
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = pose_world.header.frame_id;
        marker.header.stamp = this->now();
        marker.ns = "human_boxes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pose_world.pose.position.x;
        marker.pose.position.y = pose_world.pose.position.y;
        marker.pose.position.z = 1.80 / 2.0;
        //marker.scale.x = tracked_person.bbox_x_length;
        //marker.scale.y = tracked_person.bbox_y_length;
        marker.scale.z = 1.80;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        human_bboxes.markers.push_back(marker);
    }

private:
    // Declare variables
    std::string pub_frame_id;
    double pub_frame_rate, delay_tolerance, max_interp_interval;
    long unsigned int max_history_length;
    bool visualize_bbox;
    
    // tf buffer and transform listener
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener {nullptr};
    
    // Human pose subscriber
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr m_zed_sub;
    // Human track publisher
    rclcpp::Publisher<soloco_interfaces::msg::TrackedPersons>::SharedPtr m_human_track_interpolated_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;  
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bbox_pub;  
    
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