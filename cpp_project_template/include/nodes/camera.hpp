#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <string>

namespace nodes {

/**
 * Odbera obrazok z ROS (Image alebo CompressedImage), YOLO detekcia (ONNX).
 * Publikuje /detected_objects, /detected_people, /camera/compressed.
 */
class CameraNode : public rclcpp::Node {
public:
    CameraNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void process_frame(cv::Mat& frame);
    void detect_yolo(cv::Mat& frame, std::vector<cv::Rect>& boxes, std::vector<int>& class_ids, std::vector<float>& scores);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_objects_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_people_count_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;

    std::string image_topic_;
    bool use_compressed_ = false;
    bool publish_compressed_ = true;
    float conf_threshold_ = 0.45f;
    float nms_threshold_ = 0.4f;
    cv::dnn::Net net_;
    std::vector<std::string> coco_names_;
    int input_size_ = 640;
    bool model_loaded_ = false;
};

}  // namespace nodes

#endif
