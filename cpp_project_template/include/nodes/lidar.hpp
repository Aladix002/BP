#ifndef NODES_LIDAR_SECTORS_NODE_HPP
#define NODES_LIDAR_SECTORS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace nodes {

/**
 * Odbera /scan, pocita len front (40 st.), publikuje /lidar_sectors [front] v metroch.
 */
class LidarSectorsNode : public rclcpp::Node {
public:
    LidarSectorsNode();

private:
    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_sectors_;
    float lidar_front_offset_rad_ = 0.f;
};

}  // namespace nodes

#endif
