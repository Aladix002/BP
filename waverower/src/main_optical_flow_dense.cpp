#include <rclcpp/rclcpp.hpp>
#include "nodes/optical_flow_dense.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nodes::OpticalFlowDenseNode>());
    rclcpp::shutdown();
    return 0;
}
