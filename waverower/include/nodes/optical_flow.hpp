#ifndef NODES_OPTICAL_FLOW_HPP
#define NODES_OPTICAL_FLOW_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace nodes {

// Riedky Lucas-Kanade optical flow na JPEG obrazoch z kamery.
// Odhad horizontalneho driftu sceny -> pridava angular.z k teleopu (ked ide vpred a netoci uzivatel).
// /optical_flow_debug (Float64MultiArray): [mean_dx_px, mean_dx_norm, flow_corr, n_good, n_corners].
// Volitelne debug_publish_image: JPEG na topic (rqt_image_view na PC, rovnaky ROS_DOMAIN_ID).
class OpticalFlowNode : public rclcpp::Node {
public:
    OpticalFlowNode();

private:
    void image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void teleop_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_cb();
    void publish_flow_debug(double mean_dx_px, double mean_dx_norm, double flow_corr,
                            double n_good, double n_corners);

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_viz_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mu_;
    cv::Mat prev_gray_;
    double flow_correction_{0.0};
    geometry_msgs::msg::Twist latest_teleop_;

    double correction_gain_;
    double max_correction_;
    double forward_threshold_;
    double steer_deadzone_;
    int min_features_;
};

}  // namespace nodes

#endif
