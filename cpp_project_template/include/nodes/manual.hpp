#ifndef NODES_MANUAL_TELEOP_NODE_HPP
#define NODES_MANUAL_TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <atomic>
#include <memory>
#include <chrono>

/** Manualne ovladanie WASD/sipky; publikuje /manual_motor_cmd (left, right -1..1). Rezim z mainu. */
class ManualTeleopNode : public rclcpp::Node {
public:
    explicit ManualTeleopNode(std::shared_ptr<std::atomic<bool>> manual_mode = nullptr);

private:
    void timer_cb();

    std::shared_ptr<std::atomic<bool>> manual_mode_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string esp32_ip_;
    double speed_;
    double left_trim_;
    double right_trim_;
    int key_release_timeout_ms_;
    std::chrono::steady_clock::time_point last_up_{};
    std::chrono::steady_clock::time_point last_down_{};
    std::chrono::steady_clock::time_point last_left_{};
    std::chrono::steady_clock::time_point last_right_{};
};

#endif
