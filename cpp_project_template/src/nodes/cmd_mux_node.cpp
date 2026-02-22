#include "nodes/cmd_mux_node.hpp"
#include <chrono>

namespace nodes {

CmdMuxNode::CmdMuxNode(std::shared_ptr<std::atomic<bool>> manual_mode)
    : Node("cmd_mux"), manual_mode_(std::move(manual_mode)) {
    pub_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor_cmd", 10);

    sub_manual_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/manual_motor_cmd", 10, std::bind(&CmdMuxNode::manual_cb, this, std::placeholders::_1));
    sub_auto_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/auto_motor_cmd", 10, std::bind(&CmdMuxNode::auto_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CmdMuxNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "Cmd mux: manual_mode=true -> /manual_motor_cmd, false -> /auto_motor_cmd -> /motor_cmd");
}

void CmdMuxNode::manual_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        last_manual_[0] = msg->data[0];
        last_manual_[1] = msg->data[1];
    }
}

void CmdMuxNode::auto_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        last_auto_[0] = msg->data[0];
        last_auto_[1] = msg->data[1];
    }
}

void CmdMuxNode::timer_cb() {
    bool manual = manual_mode_ && manual_mode_->load();
    if (manual)
        publish_cmd(last_manual_[0], last_manual_[1]);
    else
        publish_cmd(last_auto_[0], last_auto_[1]);
}

void CmdMuxNode::publish_cmd(double left, double right) {
    std_msgs::msg::Float64MultiArray out;
    out.data = { left, right };
    pub_cmd_->publish(out);
}

}  // namespace nodes
