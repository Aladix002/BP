#ifndef NODES_OPTICAL_FLOW_DENSE_HPP
#define NODES_OPTICAL_FLOW_DENSE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace nodes {

/**
 * Dense optical flow node (Farneback).
 * Rovnaké rozhranie ako OpticalFlowNode (LK sparse), iný algoritmus.
 * Vypočíta flow pre každý pixel → priemer horizontálnej zložky → korekcia angular.z.
 * Presnejší ako LK sparse, ale vyššia záťaž CPU (~2-3x pomalší).
 */
class OpticalFlowDenseNode : public rclcpp::Node {
public:
    OpticalFlowDenseNode();

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
    double flow_correction_{0.0};
    geometry_msgs::msg::Twist latest_teleop_;

    double correction_gain_;
    double max_correction_;
    double forward_threshold_;
    double steer_deadzone_;
};

}  // namespace nodes

#endif
