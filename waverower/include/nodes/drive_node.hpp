#ifndef NODES_DRIVE_NODE_HPP
#define NODES_DRIVE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

namespace nodes {

class Pca9685;

class DriveNode : public rclcpp::Node {
public:
  explicit DriveNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DriveNode() override;

private:
  // Prevedie normalizovaný príkaz [-1, 1] na PWM duty [0, 4095].
  // Lineárne mapovanie: 0 → stop, pwm_min pri prvom pohybe, pwm_max pri plnom vstupe.
  uint16_t to_duty(double cmd) const;

  void timer_cb();
  void teleop_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleop_corrected_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void apply_manual_twist(const geometry_msgs::msg::Twist& msg);

  std::unique_ptr<Pca9685> hat_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_corrected_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;

  std::mutex mu_;

  // Cieľové hodnoty (z callbackov)
  double target_l_{0.0};
  double target_r_{0.0};
  std::chrono::steady_clock::time_point last_cmd_{};
  bool have_cmd_{false};

  // Vyhladené hodnoty (aktualizované v timer_cb)
  double smooth_l_{0.0};
  double smooth_r_{0.0};

  // IMU PID stav
  double imu_yaw_rate_{0.0};
  double imu_integral_{0.0};
  double imu_prev_error_{0.0};
  std::chrono::steady_clock::time_point imu_pid_time_{};
};

}  // namespace nodes

#endif
