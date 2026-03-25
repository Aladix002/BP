#ifndef NODES_OPTICAL_FLOW_HPP
#define NODES_OPTICAL_FLOW_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace nodes {

/**
 * Optical flow node (Lucas-Kanade sparse).
 * Odbera /camera/camera_node/image_raw/compressed a /teleop_cmd_vel.
 * Publikuje korigovaný Twist na /teleop_cmd_vel_corrected.
 * Korekcia: ak kamera detekuje horizontálny drift scény, pridá korekciu angular.z.
 * Aktívna len keď robot ide dopredu a užívateľ aktívne nestočuje.
 */
class OpticalFlowNode : public rclcpp::Node {
public:
    OpticalFlowNode();

private:
    void image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void teleop_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timer_cb();

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_teleop_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mu_;
    cv::Mat prev_gray_;
    double flow_correction_{0.0};   // angular.z korekcia [rad/s]
    geometry_msgs::msg::Twist latest_teleop_;

    // parametre
    double correction_gain_;
    double max_correction_;
    double forward_threshold_;
    double steer_deadzone_;
    int min_features_;
};

}  // namespace nodes

#endif
