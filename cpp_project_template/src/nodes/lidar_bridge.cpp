#include "nodes/lidar_bridge.hpp"

using namespace nodes;

LidarBridge::LidarBridge()
    : Node("lidar_bridge")
{
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/scan");
    this->declare_parameter<std::string>("output_topic", "/bpc_prp_robot/lidar");
    
    // Get parameters
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    
    // Create subscriber for input topic (usually /scan from LD19 driver)
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic_,
        10,
        std::bind(&LidarBridge::scan_callback, this, std::placeholders::_1)
    );
    
    // Create publisher for output topic
    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        output_topic_,
        10
    );
    
    RCLCPP_INFO(this->get_logger(), "Lidar Bridge initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_.c_str());
}

void LidarBridge::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Simply republish the message to the output topic
    lidar_publisher_->publish(*msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Lidar data forwarded: %zu ranges", msg->ranges.size());
}
