#ifndef NODES_MOTOR_DRIVER_NODE_HPP
#define NODES_MOTOR_DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace nodes {

/** Odbera /motor_cmd: data[0]=LAVY, data[1]=PRAVY (-1..1); posiela na ESP32 HTTP T:1, L=lavy, R=pravy. UGV: L->Motor A (lavy), R->Motor B (pravy). */
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
