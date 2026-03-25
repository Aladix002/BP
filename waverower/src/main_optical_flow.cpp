#include <rclcpp/rclcpp.hpp>
#include "nodes/optical_flow.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nodes::OpticalFlowNode>());
    rclcpp::shutdown();
    return 0;
}
