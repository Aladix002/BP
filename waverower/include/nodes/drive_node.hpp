#ifndef NODES_DRIVE_NODE_HPP
#define NODES_DRIVE_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cstdint>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>

namespace nodes {

class Pca9685;

enum class DriveMode : std::uint8_t { Manual = 0, Auto = 1 };

class DriveNode : public rclcpp::Node {
public:
  explicit DriveNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DriveNode() override;

  void prepare_terminal();
  void start_input_thread();

private:
  void timer_cb();
  void timer_cb_manual(const std::chrono::steady_clock::time_point& now, double boost, double snap,
                       double snap_turn, double in_place_boost, double alpha, double alpha_spin);
  void timer_cb_auto(double boost, double snap, double snap_turn, double in_place_boost, double alpha,
                     double alpha_spin);
  void input_loop();
  void touch_fb(double v);
  void touch_tr(double v);
  void full_stop_keys();
  void bump_speed(double delta);
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void teleop_twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void reset_motion_state();
  static DriveMode parse_control_mode(const std::string& s);
  rcl_interfaces::msg::SetParametersResult on_param_change(const std::vector<rclcpp::Parameter>& parameters);

  void apply_tank(double l_cmd, double r_cmd, double base, double boost, double snap_fwd,
                  double snap_turn, bool low_forward, double in_place_boost);

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);

  // IMU yaw PID (bez enkoderov)
  double imu_yaw_rate_{0.0};
  double imu_yaw_integral_{0.0};
  double imu_yaw_prev_error_{0.0};
  std::chrono::steady_clock::time_point imu_pid_last_time_{};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_imu_debug_;

  std::unique_ptr<Pca9685> hat_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::mutex mu_;
  double fb_{0.0};
  double tr_{0.0};
  std::chrono::steady_clock::time_point last_fb_{};
  std::chrono::steady_clock::time_point last_tr_{};
  double base_speed_{1.0};
  double turn_scale_{0.55};
  int decay_ms_{700};
  double pwm_freq_hz_{800.0};
  double smooth_l_{0.0};
  double smooth_r_{0.0};

  double twist_linear_x_{0.0};
  double twist_angular_z_{0.0};
  std::chrono::steady_clock::time_point last_cmd_steady_{};
  bool have_cmd_vel_{false};

  int cmd_vel_timeout_ms_{250};
  double wheel_separation_m_{0.20};
  double max_wheel_linear_m_s_{0.35};

  std::atomic<DriveMode> control_mode_{DriveMode::Manual};

  int i2c_bus_{1};
  int i2c_addr_{0x40};

  std::atomic<bool> running_{false};
  std::thread input_thread_;
  bool tty_ok_{false};
  struct termios tty_saved_{};
};

}  // namespace nodes

#endif
