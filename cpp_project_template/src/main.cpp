#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "nodes/manual.hpp"
#include "nodes/lidar.hpp"
#include "nodes/behavioral_tree_auto.hpp"
#include "nodes/cmd_mux.hpp"
#include "nodes/motor_driver.hpp"
#include "nodes/camera.hpp"
#include "nodes/imu.hpp"
#include <atomic>
#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Rezim: true = manual (WASD), false = auto (BT: person->stop, blocked->turn, else corridor). Prepinanie 'm' v manual_teleop.
    auto manual_mode = std::make_shared<std::atomic<bool>>(true);

    auto manual_teleop = std::make_shared<ManualTeleopNode>(manual_mode);
    auto lidar_sectors = std::make_shared<nodes::LidarSectorsNode>();
    auto bt_auto = std::make_shared<nodes::BtAutoNode>();
    auto cmd_mux = std::make_shared<nodes::CmdMuxNode>(manual_mode);
    auto motor_driver = std::make_shared<nodes::MotorDriverNode>();
    auto camera = std::make_shared<nodes::CameraNode>();
    auto imu_http = std::make_shared<nodes::ImuHttpNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(manual_teleop);
    executor.add_node(lidar_sectors);
    executor.add_node(bt_auto);
    executor.add_node(cmd_mux);
    executor.add_node(motor_driver);
    executor.add_node(camera);
    executor.add_node(imu_http);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
