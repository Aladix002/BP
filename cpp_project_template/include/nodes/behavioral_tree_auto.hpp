#ifndef NODES_BT_AUTO_NODE_HPP
#define NODES_BT_AUTO_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <atomic>
#include <mutex>

namespace nodes {

enum class Behavior : int { Stop = 0, Turn = 1, Forward = 2 };

/**
 * Auto: len front z lidaru. 1) Osoba -> Stop. 2) Prekazka vpredu -> toc na timer (trvanie nastavitelne, napr. na 80 st.).
 * 3) Inak -> jazda dopredu. IMU sa nepouziva.
 */
class BtAutoNode : public rclcpp::Node {
public:
    BtAutoNode();

private:
    void sectors_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void people_cb(const std_msgs::msg::Int32::SharedPtr msg);
    void timer_cb();

    void publish_cmd(double left, double right);
    void publish_behavior(Behavior b);
    static double pwm_to_float(float pwm);
    void action_stop();
    /** Tocenie ako v manuale: L/R = +-turn_in_place. */
    void action_turn(bool turn_left);
    void action_forward();

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_sectors_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_people_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_behavior_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex data_mutex_;
    float front_ = 1e9f;
    std::atomic<int> people_count_{0};
    bool has_sectors_ = false;

    float threshold_stop_m_ = 0.20f;
    float threshold_go_m_ = 0.50f;
    float base_speed_pwm_ = 153.f;
    double turn_in_place_ = 0.4;    // rovnako ako manual (sipka do lava/doprava)
    bool turning_ = false;
    double turn_duration_sec_ = 2.0;
    rclcpp::Time turn_start_time_;
    bool turn_left_ = true;
    Behavior last_published_behavior_ = static_cast<Behavior>(-1);
};

}  // namespace nodes

#endif
