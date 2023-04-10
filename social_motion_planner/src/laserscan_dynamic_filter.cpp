#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <zed_interfaces/msg/bounding_box_3d.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <algorithm>
#include <array>

class LaserScanFilterNode : public rclcpp::Node
{
public:
    LaserScanFilterNode()
        : Node("laser_scan_filter") {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LaserScanFilterNode::laserScanCallback, this, std::placeholders::_1));
        zed_objects_sub_ = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
            "/zed2/zed_node/obj_det/objects", 10, std::bind(&LaserScanFilterNode::zedObjectsCallback, this, std::placeholders::_1));
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){{
        auto filtered_scan = *msg; // make a copy of the original scan
        // Filter the laser scan points that are inside the bounding box
        for (size_t i = 0; i <= msg->ranges.size(); i++){
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = msg->ranges[i] * cos(angle);
            float y = msg->ranges[i] * sin(angle);
            for (size_t k = 0; k <= bounding_box_.size(0); k++) {
                if (isPointInsideBoundingBox(x, y, bounding_box_[k])){
                    filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        filtered_scan_pub_->publish(filtered_scan);
    }

    void zedObjectsCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg){
    // Store the 2D bounding box information for use in filtering the laserscan
        for (int k = 0; k <= msg->objects.size(); k++) {
            // Save bounding box if object is moving
            if (msg->objects.action_state == 2) {
                bbox = msg->objects[k].bounding_box_3d;
                for (int i = 0; i <= 8; i++){
                    bounding_box_[i] = bbox.corners[i];
                }
            }
        }
    }
    
    bool isPointInsideBoundingBox(float x, float y, const std::array<float, 8>& bounding_box){
        // Find the min and max x and y values of the bounding box
        float min_x = std::min({bounding_box[0], bounding_box[2], bounding_box[4], bounding_box[6]});
        float max_x = std::max({bounding_box[0], bounding_box[2], bounding_box[4], bounding_box[6]});
        float min_y = std::min({bounding_box[1], bounding_box[3], bounding_box[5], bounding_box[7]});
        float max_y = std::max({bounding_box[1], bounding_box[3], bounding_box[5], bounding_box[7]});
    
        // Check if the point is inside the bounding box
        return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
    }

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