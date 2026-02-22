#ifndef NODES_LIDAR_AUTO_NODE_HPP
#define NODES_LIDAR_AUTO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nodes/lidar_sectors.hpp"

namespace nodes {

/**
 * Auto režim podľa lidaru: 3 sektory 60° (front, left, right).
 * - Predok pod threshold_stop: zastaví a točí sa smerom kde je viac miesta.
 * - V koridore: ide dopredu, pridaním pravého motoru keď blízko pravej steny, ľavého keď blízko ľavej.
 */
class LidarAutoNode : public rclcpp::Node {
public:
    LidarAutoNode();

private:
    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    float lidar_front_offset_rad_ = 0.f;
    float threshold_stop_m_ = 0.30f;   // pod touto vzdialenosťou vpredu zastaví a točí
    float threshold_wall_m_ = 0.45f;    // pod touto vzdialenosťou bokom pridá k motoru (koridor)
    float base_speed_ = 0.4f;
    float turn_speed_ = 0.35f;
    float side_correction_ = 0.12f;
};

}  // namespace nodes

#endif
