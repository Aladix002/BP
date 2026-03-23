#include <rclcpp/rclcpp.hpp>
#include "nodes/camera.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nodes::CameraNode>());
    rclcpp::shutdown();
    return 0;
}
