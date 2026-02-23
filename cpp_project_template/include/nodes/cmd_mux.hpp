#ifndef NODES_CMD_MUX_NODE_HPP
#define NODES_CMD_MUX_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <memory>

namespace nodes {

/** Prepina medzi /manual_motor_cmd a /auto_motor_cmd podla rezimu; publikuje /motor_cmd. */
class CmdMuxNode : public rclcpp::Node {
public:
    explicit CmdMuxNode(std::shared_ptr<std::atomic<bool>> manual_mode);

private:
    void manual_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void auto_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void timer_cb();
    void publish_cmd(double left, double right);

    std::shared_ptr<std::atomic<bool>> manual_mode_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_manual_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_auto_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::array<double, 2> last_manual_{0.0, 0.0};
    std::array<double, 2> last_auto_{0.0, 0.0};
};

}  // namespace nodes

#endif
