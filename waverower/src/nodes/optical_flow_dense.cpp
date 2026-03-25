#include "nodes/optical_flow_dense.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

OpticalFlowDenseNode::OpticalFlowDenseNode() : Node("optical_flow_dense_node") {
    declare_parameter<double>("correction_gain",   1.5);
    declare_parameter<double>("max_correction",    0.3);
    declare_parameter<double>("forward_threshold", 0.05);
    declare_parameter<double>("steer_deadzone",    0.12);
    declare_parameter<std::string>("image_topic",  "/camera/camera_node/image_raw/compressed");
    declare_parameter<std::string>("teleop_topic", "/teleop_cmd_vel");
    declare_parameter<std::string>("output_topic", "/teleop_cmd_vel_corrected");

    correction_gain_   = get_parameter("correction_gain").as_double();
    max_correction_    = get_parameter("max_correction").as_double();
    forward_threshold_ = get_parameter("forward_threshold").as_double();
    steer_deadzone_    = get_parameter("steer_deadzone").as_double();
    const auto img_topic    = get_parameter("image_topic").as_string();
    const auto teleop_topic = get_parameter("teleop_topic").as_string();
    const auto out_topic    = get_parameter("output_topic").as_string();

    sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        img_topic, rclcpp::SensorDataQoS(),
        std::bind(&OpticalFlowDenseNode::image_cb, this, std::placeholders::_1));

    sub_teleop_ = create_subscription<geometry_msgs::msg::Twist>(
        teleop_topic, rclcpp::QoS(10),
        std::bind(&OpticalFlowDenseNode::teleop_cb, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(out_topic, rclcpp::QoS(10));

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&OpticalFlowDenseNode::timer_cb, this));

    RCLCPP_INFO(get_logger(),
                "OpticalFlowDense (Farneback): image=%s teleop=%s out=%s gain=%.2f",
                img_topic.c_str(), teleop_topic.c_str(), out_topic.c_str(), correction_gain_);
}

void OpticalFlowDenseNode::image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (frame.empty()) return;

    // Farneback pracuje na menšom rozlíšení kvôli výkonu
    if (frame.cols > 320) {
        cv::resize(frame, frame, cv::Size(320, frame.rows * 320 / frame.cols));
    }

    std::lock_guard<std::mutex> lock(mu_);

    if (prev_gray_.empty()) {
        prev_gray_ = frame;
        return;
    }

    // Farneback dense optical flow
    // params: pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(prev_gray_, frame, flow,
                                 0.5,   // pyr_scale
                                 3,     // levels
                                 15,    // winsize
                                 3,     // iterations
                                 5,     // poly_n
                                 1.2,   // poly_sigma
                                 0);    // flags

    // flow je 2-kanálový Mat (každý pixel má dx, dy)
    // spočítaj priemer horizontálnej zložky (kanál 0 = x)
    const cv::Scalar mean_flow = cv::mean(flow);
    const double mean_dx = mean_flow[0];  // priemer dx cez všetky pixely

    // normalizuj šírkou → [-0.5, 0.5] zhruba
    const double mean_dx_norm = mean_dx / frame.cols;
    const double raw = -mean_dx_norm * correction_gain_;
    flow_correction_ = std::clamp(raw, -max_correction_, max_correction_);

    prev_gray_ = frame;
}

void OpticalFlowDenseNode::teleop_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    latest_teleop_ = *msg;
}

void OpticalFlowDenseNode::timer_cb() {
    geometry_msgs::msg::Twist out;
    double correction = 0.0;

    {
        std::lock_guard<std::mutex> lock(mu_);
        out = latest_teleop_;
        correction = flow_correction_;
    }

    const bool moving_forward = std::abs(out.linear.x) > forward_threshold_;
    const bool user_steering  = std::abs(out.angular.z) > steer_deadzone_;

    if (moving_forward && !user_steering) {
        out.angular.z = std::clamp(out.angular.z + correction, -1.5, 1.5);
    }

    pub_cmd_->publish(out);
}

}  // namespace nodes
