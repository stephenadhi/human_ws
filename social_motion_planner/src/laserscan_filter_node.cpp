#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <zed_interfaces/msg/bounding_box3_d.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <algorithm>
#include <array>

class LaserScanFilterNode : public rclcpp::Node {
public:
    LaserScanFilterNode()
        : Node("laser_scan_filter") {
        // Declare parameters
        std::string human_track_topic, detected_obj_topic, scan_topic, filtered_scan_topic;
        this->declare_parameter<std::string>("detected_obj_topic", "/zed2/zed_node/obj_det/objects");
        this->declare_parameter<std::string>("scan_topic", "scan/map");
        this->declare_parameter<std::string>("filtered_scan_topic", "scan/filtered");
        this->declare_parameter<int64_t>("max_num_agents", 20);
        this->get_parameter("detected_obj_topic", detected_obj_topic);
        this->get_parameter("scan_topic", scan_topic);
        this->get_parameter("filtered_scan_topic", filtered_scan_topic);       
        this->get_parameter("max_num_agents", max_num_agents);
        //Subscribers
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&LaserScanFilterNode::laserScanCallback, this, std::placeholders::_1));
        zed_objects_sub_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
            detected_obj_topic, 10, std::bind(&LaserScanFilterNode::zedObjectsCallback, this, std::placeholders::_1));
        // Publisher
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(filtered_scan_topic, 10);
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        auto filtered_scan = *msg; // make a copy of the original scan
        // Filter the laser scan points that are inside the bounding box
        for (long unsigned int i = 0; i < msg->ranges.size(); i++){
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = msg->ranges[i] * cos(angle);
            float y = msg->ranges[i] * sin(angle);
            if (!bounding_boxes_.empty()) {
                if (isPointInsideBoundingBox(x, y, bounding_boxes_)) {
                    filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        filtered_scan_pub_->publish(filtered_scan);
    }

    void zedObjectsCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg){
    // Store the 2D bounding box information for use in filtering the laserscan
        int num_agents = std::min(static_cast<int>(msg->objects.size()), max_num_agents);
        bounding_boxes_.clear();
        for (int i = 0; i < num_agents; i++) {
            const auto& corners = msg->objects[i].bounding_box_3d.corners;
            std::array  <float, 8> bounding_box;
            std::copy(std::begin(corners), std::end(corners), std::begin(bounding_box));
            bounding_boxes_.push_back(bounding_box);
            
        }
    }
    
    bool isPointInsideBoundingBox(float x, float y, const std::vector<std::array<float, 8>>& bounding_boxes){
        // Check if the point is inside any of the bounding boxes
        for (const auto& bounding_box : bounding_boxes) {
            // Find the min and max x and y values of the bounding box
            float min_x = std::min({bounding_box[0], bounding_box[2], bounding_box[4], bounding_box[6]});
            float max_x = std::max({bounding_box[0], bounding_box[2], bounding_box[4], bounding_box[6]});
            float min_y = std::min({bounding_box[1], bounding_box[3], bounding_box[5], bounding_box[7]});
            float max_y = std::max({bounding_box[1], bounding_box[3], bounding_box[5], bounding_box[7]});

            // Check if the point is inside the bounding box
            if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) {
                return true;
            }
        }

        return false;
    }

private:
    int max_num_agents = 10;
    std::vector<std::array<float, 8>> bounding_boxes_ = {};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr zed_objects_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanFilterNode>());
    rclcpp::shutdown();
    return 0;
}