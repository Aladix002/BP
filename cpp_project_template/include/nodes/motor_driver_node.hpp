#ifndef NODES_MOTOR_DRIVER_NODE_HPP
#define NODES_MOTOR_DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace nodes {

/** Odoberá /motor_cmd (left, right v rozsahu -1..1) a posiela na ESP32 cez HTTP (T:1, L, R). */
class MotorDriverNode : public rclcpp::Node {
public:
    MotorDriverNode();
    ~MotorDriverNode();

private:
    void motor_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void send_http(double left, double right);

    std::string esp32_url_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

}  // namespace nodes

#endif
