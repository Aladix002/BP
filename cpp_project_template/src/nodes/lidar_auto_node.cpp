#include "nodes/lidar_auto_node.hpp"
#include <algorithm>

namespace nodes {

LidarAutoNode::LidarAutoNode() : Node("lidar_auto") {
    this->declare_parameter<double>("lidar_front_offset_rad", 0.0);
    lidar_front_offset_rad_ = static_cast<float>(this->get_parameter("lidar_front_offset_rad").as_double());
    this->declare_parameter<double>("threshold_stop_m", 0.30);
    threshold_stop_m_ = static_cast<float>(this->get_parameter("threshold_stop_m").as_double());
    this->declare_parameter<double>("threshold_wall_m", 0.45);
    threshold_wall_m_ = static_cast<float>(this->get_parameter("threshold_wall_m").as_double());
    this->declare_parameter<double>("base_speed", 0.4);
    base_speed_ = static_cast<float>(this->get_parameter("base_speed").as_double());
    this->declare_parameter<double>("turn_speed", 0.35);
    turn_speed_ = static_cast<float>(this->get_parameter("turn_speed").as_double());
    this->declare_parameter<double>("side_correction", 0.12);
    side_correction_ = static_cast<float>(this->get_parameter("side_correction").as_double());

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LidarAutoNode::scan_cb, this, std::placeholders::_1));
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/auto_motor_cmd", 10);

    RCLCPP_INFO(this->get_logger(),
        "Lidar auto: 3×60° sektory | stop < %.2fm | koridor wall < %.2fm | base=%.2f turn=%.2f",
        threshold_stop_m_, threshold_wall_m_, base_speed_, turn_speed_);
}

void LidarAutoNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    LidarSectors60 sec;
    fill_sectors_60(*msg, lidar_front_offset_rad_, sec);

    double L = 0.0, R = 0.0;

    if (!sec.valid) {
        std_msgs::msg::Float64MultiArray stop;
        stop.data = {0.0, 0.0};
        pub_cmd_->publish(stop);
        return;
    }

    // Predok blízko -> zastav a toč smerom kde je viac miesta
    if (sec.front_min < threshold_stop_m_) {
        if (sec.left_min > sec.right_min)
            L = -static_cast<double>(turn_speed_), R = static_cast<double>(turn_speed_);  // toč vľavo
        else
            L = static_cast<double>(turn_speed_), R = -static_cast<double>(turn_speed_);  // toč vpravo
    } else {
        // Koridor: dopredu + korekcia podľa boku
        L = base_speed_;
        R = base_speed_;
        if (sec.right_min < threshold_wall_m_)
            R += side_correction_;   // blízko pravej steny -> pridaj pravému (uhni doľava)
        if (sec.left_min < threshold_wall_m_)
            L += side_correction_;  // blízko ľavej steny -> pridaj ľavému (uhni doprava)
    }

    L = std::max(-1.0, std::min(1.0, L));
    R = std::max(-1.0, std::min(1.0, R));

    std_msgs::msg::Float64MultiArray out;
    out.data = { L, R };
    pub_cmd_->publish(out);
}

}  // namespace nodes
