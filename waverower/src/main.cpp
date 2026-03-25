#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nodes/wasd_motor_hat.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  // Pozor: automatically_declare_parameters_from_overrides(true) by zdvojilo declare v uzle (base_speed atď.).
  node_options.parameter_overrides({
      rclcpp::Parameter("control_mode", std::string("manual")),
      rclcpp::Parameter("pwm_boost", 2.35),
      rclcpp::Parameter("snap_threshold", 0.22),
      rclcpp::Parameter("smooth_alpha", 1.0),
      rclcpp::Parameter("base_speed", 1.0),
  });

  try {
    auto node = std::make_shared<nodes::WasdMotorHatNode>(node_options);
    // Keyboard input is intentionally disabled.
    // Control is expected via ROS topics (e.g. /teleop_cmd_vel or /cmd_vel).
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waverower"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
