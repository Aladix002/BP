// Sparse optical flow: Lucas–Kanade + image pyramids (calcOpticalFlowPyrLK, goodFeaturesToTrack).
// Theory and API align with the OpenCV tutorial on optical flow:
// https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
#include "nodes/optical_flow.hpp"
#include <algorithm>
#include <cmath>

namespace nodes {

OpticalFlowNode::OpticalFlowNode() : Node("optical_flow_node") {
    declare_parameter<double>("correction_gain",   1.5);
    declare_parameter<double>("max_correction",    0.3);
    declare_parameter<double>("forward_threshold", 0.05);
    declare_parameter<double>("steer_deadzone",    0.12);
    declare_parameter<int>   ("min_features",      15);
    declare_parameter<std::string>("image_topic",  "/camera/camera_node/image_raw/compressed");
    declare_parameter<std::string>("teleop_topic", "/teleop_cmd_vel");
    declare_parameter<std::string>("output_topic", "/teleop_cmd_vel_corrected");

    correction_gain_    = get_parameter("correction_gain").as_double();
    max_correction_     = get_parameter("max_correction").as_double();
    forward_threshold_  = get_parameter("forward_threshold").as_double();
    steer_deadzone_     = get_parameter("steer_deadzone").as_double();
    min_features_       = get_parameter("min_features").as_int();
    const auto img_topic    = get_parameter("image_topic").as_string();
    const auto teleop_topic = get_parameter("teleop_topic").as_string();
    const auto out_topic    = get_parameter("output_topic").as_string();

    sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        img_topic, rclcpp::SensorDataQoS(),
        std::bind(&OpticalFlowNode::image_cb, this, std::placeholders::_1));

    sub_teleop_ = create_subscription<geometry_msgs::msg::Twist>(
        teleop_topic, rclcpp::QoS(10),
        std::bind(&OpticalFlowNode::teleop_cb, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(out_topic, rclcpp::QoS(10));

    // publikuj na 20 Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&OpticalFlowNode::timer_cb, this));

    RCLCPP_INFO(get_logger(),
                "OpticalFlow: image=%s teleop=%s out=%s gain=%.2f max_corr=%.2f",
                img_topic.c_str(), teleop_topic.c_str(), out_topic.c_str(),
                correction_gain_, max_correction_);
}

void OpticalFlowNode::image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // dekóduj JPEG
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (frame.empty()) return;

    // zmenši na max 320px šírku kvôli výkonu na RPi
    if (frame.cols > 320) {
        cv::resize(frame, frame, cv::Size(320, frame.rows * 320 / frame.cols));
    }

    std::lock_guard<std::mutex> lock(mu_);

    if (prev_gray_.empty()) {
        prev_gray_ = frame;
        return;
    }

    // detekcia rohových bodov v predchádzajúcom snímku
    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(prev_gray_, prev_pts, 100, 0.01, 10);

    if (static_cast<int>(prev_pts.size()) < min_features_) {
        prev_gray_ = frame;
        return;
    }

    // Lucas-Kanade sledovanie
    std::vector<cv::Point2f> curr_pts;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev_gray_, frame, prev_pts, curr_pts, status, err);

    // priemer horizontálneho pohybu (dx) len úspešne sledovaných bodov
    double sum_dx = 0.0;
    int count = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            sum_dx += curr_pts[i].x - prev_pts[i].x;
            ++count;
        }
    }

    if (count >= min_features_) {
        // normalizuj šírkou snímku → [-0.5, 0.5] zhruba
        const double mean_dx_norm = (sum_dx / count) / frame.cols;
        // scéna sa posunula doľava (mean_dx < 0) → robot driftuje doprava
        // koriguj: otočiť doľava = kladné angular.z
        const double raw = -mean_dx_norm * correction_gain_;
        flow_correction_ = std::clamp(raw, -max_correction_, max_correction_);
    } else {
        flow_correction_ = 0.0;
    }

    prev_gray_ = frame;
}

void OpticalFlowNode::teleop_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mu_);
    latest_teleop_ = *msg;
}

void OpticalFlowNode::timer_cb() {
    geometry_msgs::msg::Twist out;
    double correction = 0.0;

    {
        std::lock_guard<std::mutex> lock(mu_);
        out = latest_teleop_;
        correction = flow_correction_;
    }

    // korekcia len keď robot ide dopredu a užívateľ aktívne nestočuje
    const bool moving_forward  = std::abs(out.linear.x) > forward_threshold_;
    const bool user_steering   = std::abs(out.angular.z) > steer_deadzone_;

    if (moving_forward && !user_steering) {
        out.angular.z = std::clamp(out.angular.z + correction, -1.5, 1.5);
    }

    pub_cmd_->publish(out);
}

}  // namespace nodes
