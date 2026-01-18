#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace nodes {

    /**
     * Lidar Bridge Node
     * 
     * Bridge node ktorý preposiela lidar dáta z /scan topicu
     * na /bpc_prp_robot/lidar topic pre kompatibilitu s existujúcim kódom.
     */
    class LidarBridge : public rclcpp::Node {
    public:
        LidarBridge();

    private:
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
        
        std::string input_topic_;
        std::string output_topic_;
    };

}
