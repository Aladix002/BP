#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nodes/drive_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;

  try {
    auto node = std::make_shared<nodes::DriveNode>(node_options);
    // Klavesnica v uzle vypnuta: ovladanie len cez topicy (teleop, cmd_vel).
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waverower"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
