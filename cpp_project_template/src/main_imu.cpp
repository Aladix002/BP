#include <rclcpp/rclcpp.hpp>
#include "nodes/imu_i2c.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<nodes::ImuI2cNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("imu_i2c"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
