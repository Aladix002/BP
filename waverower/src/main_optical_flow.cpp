// Samostatny binarny uzol optical flow (CMake target optical_flow) - pouziva sa z runtime_stack.
#include <rclcpp/rclcpp.hpp>
#include "nodes/optical_flow.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nodes::OpticalFlowNode>());
    rclcpp::shutdown();
    return 0;
}
