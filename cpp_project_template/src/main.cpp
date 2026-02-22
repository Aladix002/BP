#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "nodes/manual_teleop_node.hpp"
#include "nodes/lidar_auto_node.hpp"
#include "nodes/cmd_mux_node.hpp"
#include "nodes/motor_driver_node.hpp"
#include <atomic>
#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Režim: true = manuál (WASD), false = auto (lidar 3×60°). Prepínanie klávesom 'm' v manual_teleop.
    auto manual_mode = std::make_shared<std::atomic<bool>>(true);

    auto manual_teleop = std::make_shared<ManualTeleopNode>(manual_mode);
    auto lidar_auto = std::make_shared<nodes::LidarAutoNode>();
    auto cmd_mux = std::make_shared<nodes::CmdMuxNode>(manual_mode);
    auto motor_driver = std::make_shared<nodes::MotorDriverNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manual_teleop);
    executor.add_node(lidar_auto);
    executor.add_node(cmd_mux);
    executor.add_node(motor_driver);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
