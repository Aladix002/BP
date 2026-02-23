#include "nodes/behavioral_tree_auto.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

static constexpr float PWM_NEUTRAL = 128.f;
static constexpr float NO_DATA_THRESHOLD = 1e8f;

double BtAutoNode::pwm_to_float(float pwm) {
    return (static_cast<double>(std::max(0.f, std::min(255.f, pwm))) - PWM_NEUTRAL) / PWM_NEUTRAL;
}

BtAutoNode::BtAutoNode() : Node("bt_auto") {
    this->declare_parameter<double>("threshold_stop_m", 0.20);
    threshold_stop_m_ = static_cast<float>(this->get_parameter("threshold_stop_m").as_double());
    this->declare_parameter<double>("threshold_go_m", 0.50);
    threshold_go_m_ = static_cast<float>(this->get_parameter("threshold_go_m").as_double());
    this->declare_parameter<int>("base_speed_level", 3);
    int level = std::max(1, std::min(10, static_cast<int>(this->get_parameter("base_speed_level").as_int())));
    base_speed_pwm_ = 128.f + (static_cast<float>(level) / 10.f) * 127.f;
    this->declare_parameter<double>("turn_in_place", 0.4);
    turn_in_place_ = std::max(0.1, std::min(1.0, this->get_parameter("turn_in_place").as_double()));
    this->declare_parameter<double>("turn_duration_sec", 1.5);
    turn_duration_sec_ = this->get_parameter("turn_duration_sec").as_double();
    this->declare_parameter<bool>("turn_left", true);
    turn_left_ = this->get_parameter("turn_left").as_bool();

    sub_sectors_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/lidar_sectors", 10, std::bind(&BtAutoNode::sectors_cb, this, std::placeholders::_1));
    sub_people_ = this->create_subscription<std_msgs::msg::Int32>(
        "/detected_people", 10, std::bind(&BtAutoNode::people_cb, this, std::placeholders::_1));
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/auto_motor_cmd", 10);
    pub_behavior_ = this->create_publisher<std_msgs::msg::Int32>("/bt_auto/current_behavior", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&BtAutoNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(),
        "BT Auto: osoba->stop, prekazka vpredu->turning %g s (L/R=+-%.2f ako manual), inak dopredu", turn_duration_sec_, turn_in_place_);
}

void BtAutoNode::sectors_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.empty()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    front_ = static_cast<float>(msg->data[0]);
    has_sectors_ = true;
}

void BtAutoNode::people_cb(const std_msgs::msg::Int32::SharedPtr msg) {
    people_count_.store(msg->data);
}

void BtAutoNode::publish_cmd(double left, double right) {
    std_msgs::msg::Float64MultiArray out;
    out.data = { left, right };
    pub_cmd_->publish(out);
}

void BtAutoNode::publish_behavior(Behavior b) {
    if (b == last_published_behavior_) return;
    last_published_behavior_ = b;
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int>(b);
    pub_behavior_->publish(msg);
}

void BtAutoNode::action_stop() {
    publish_cmd(0.0, 0.0);
    publish_behavior(Behavior::Stop);
}

void BtAutoNode::action_turn(bool turn_left) {
    if (turn_left)
        publish_cmd(-turn_in_place_, turn_in_place_);
    else
        publish_cmd(turn_in_place_, -turn_in_place_);
    publish_behavior(Behavior::Turn);
}

void BtAutoNode::action_forward() {
    double v = pwm_to_float(base_speed_pwm_);
    publish_cmd(v, v);
    publish_behavior(Behavior::Forward);
}

void BtAutoNode::timer_cb() {
    if (people_count_.load() > 0) {
        action_stop();
        return;
    }

    float f;
    bool has;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        f = front_;
        has = has_sectors_;
    }

    if (!has || f >= NO_DATA_THRESHOLD || std::isnan(f)) {
        action_stop();
        return;
    }

    // Prekazka vpredu: toc na timer (trvanie turn_duration_sec), potom dopredu
    if (f < threshold_stop_m_) {
        if (!turning_) {
            turning_ = true;
            turn_start_time_ = this->now();
        }
    }
    if (turning_) {
        double elapsed = (this->now() - turn_start_time_).seconds();
        if (elapsed >= turn_duration_sec_) {
            turning_ = false;
        } else {
            action_turn(turn_left_);
            return;
        }
    }

    action_forward();
}

}  // namespace nodes
