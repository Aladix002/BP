#include "nodes/optical_flow.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

// opencv2/opencv.hpp uz zahrnuje highgui, imgcodecs a vsetky ostatne hlavicky cez optical_flow.hpp

namespace nodes {

OpticalFlowNode::OpticalFlowNode() : Node("optical_flow_node") {
    declare_parameter<double>("correction_gain",   1.5);
    declare_parameter<double>("max_correction",    0.3);
    declare_parameter<double>("forward_threshold", 0.05);
    declare_parameter<double>("steer_deadzone",    0.12);
    declare_parameter<int>   ("min_features",      15);
    declare_parameter<bool>  ("enabled",           false);
    declare_parameter<std::string>("image_topic",  "/camera/camera_node/image_raw/compressed");
    declare_parameter<std::string>("teleop_topic", "/teleop_cmd_vel");
    declare_parameter<std::string>("output_topic", "/teleop_cmd_vel_corrected");
    declare_parameter<std::string>("debug_topic", "/optical_flow_debug");
    declare_parameter<bool>("debug_show", false);
    declare_parameter<bool>("debug_publish_image", false);
    declare_parameter<std::string>("debug_image_topic", "/optical_flow/viz/compressed");
    declare_parameter<int>("debug_window_scale", 2);
    declare_parameter<std::string>("debug_window_name", "optical_flow");

    correction_gain_    = get_parameter("correction_gain").as_double();
    max_correction_     = get_parameter("max_correction").as_double();
    forward_threshold_  = get_parameter("forward_threshold").as_double();
    steer_deadzone_     = get_parameter("steer_deadzone").as_double();
    min_features_       = get_parameter("min_features").as_int();
    const auto img_topic    = get_parameter("image_topic").as_string();
    const auto teleop_topic = get_parameter("teleop_topic").as_string();
    const auto out_topic    = get_parameter("output_topic").as_string();
    const auto dbg_topic    = get_parameter("debug_topic").as_string();
    const auto viz_topic    = get_parameter("debug_image_topic").as_string();

    sub_image_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        img_topic, rclcpp::SensorDataQoS(),
        std::bind(&OpticalFlowNode::image_cb, this, std::placeholders::_1));

    sub_teleop_ = create_subscription<geometry_msgs::msg::Twist>(
        teleop_topic, rclcpp::QoS(10),
        std::bind(&OpticalFlowNode::teleop_cb, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(out_topic, rclcpp::QoS(10));
    pub_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>(dbg_topic, rclcpp::QoS(10));
    pub_viz_ = create_publisher<sensor_msgs::msg::CompressedImage>(viz_topic, rclcpp::SensorDataQoS());

    // 20 Hz: staci na korekciu; obraz moze byt 15-30 Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&OpticalFlowNode::timer_cb, this));

    RCLCPP_INFO(get_logger(),
                "OpticalFlow: image=%s teleop=%s out=%s gain=%.2f max_corr=%.2f debug=%s viz=%s",
                img_topic.c_str(), teleop_topic.c_str(), out_topic.c_str(),
                correction_gain_, max_correction_, dbg_topic.c_str(), viz_topic.c_str());
}

void OpticalFlowNode::publish_flow_debug(double mean_dx_px, double mean_dx_norm, double flow_corr,
                                         double n_good, double n_corners) {
    std_msgs::msg::Float64MultiArray m;
    m.data.resize(5);
    m.data[0] = mean_dx_px;
    m.data[1] = mean_dx_norm;
    m.data[2] = flow_corr;
    m.data[3] = n_good;
    m.data[4] = n_corners;
    pub_debug_->publish(m);
}

void OpticalFlowNode::image_cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    if (!get_parameter("enabled").as_bool()) {
        std::lock_guard<std::mutex> lock(mu_);
        prev_gray_.release();
        flow_correction_ = 0.0;
        return;
    }

    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_GRAYSCALE);
    if (frame.empty()) return;

    if (frame.cols > 320) {
        cv::resize(frame, frame, cv::Size(320, frame.rows * 320 / frame.cols));
    }

    const int W = frame.cols;
    const bool want_show = get_parameter("debug_show").as_bool();
    const bool want_pub  = get_parameter("debug_publish_image").as_bool();

    double mean_dx_px = 0.0;
    double mean_dx_norm = 0.0;
    double dbg_corr = 0.0;
    int n_corners = 0;
    int n_good = 0;

    cv::Mat frame_viz;
    std::vector<cv::Point2f> prev_pts_copy;
    std::vector<cv::Point2f> curr_pts_copy;
    std::vector<uchar> status_copy;
    bool show_this_frame = false;

    {
        std::lock_guard<std::mutex> lock(mu_);

        if (prev_gray_.empty()) {
            prev_gray_ = frame;
            publish_flow_debug(0.0, 0.0, 0.0, 0.0, 0.0);
            return;
        }

        std::vector<cv::Point2f> prev_pts;
        cv::goodFeaturesToTrack(prev_gray_, prev_pts, 100, 0.01, 10);
        n_corners = static_cast<int>(prev_pts.size());

        if (n_corners < min_features_) {
            prev_gray_ = frame;
            flow_correction_ = 0.0;
            publish_flow_debug(0.0, 0.0, 0.0, 0.0, static_cast<double>(n_corners));
            return;
        }

        std::vector<cv::Point2f> curr_pts;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prev_gray_, frame, prev_pts, curr_pts, status, err);

        double sum_dx = 0.0;
        int count = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                sum_dx += static_cast<double>(curr_pts[i].x - prev_pts[i].x);
                ++count;
            }
        }
        n_good = count;

        if (count > 0) {
            mean_dx_px = sum_dx / static_cast<double>(count);
            mean_dx_norm = mean_dx_px / static_cast<double>(W);
        }

        if (count >= min_features_) {
            const double raw = -mean_dx_norm * correction_gain_;
            flow_correction_ = std::clamp(raw, -max_correction_, max_correction_);
            dbg_corr = flow_correction_;
        } else {
            flow_correction_ = 0.0;
            dbg_corr = 0.0;
        }

        if ((want_show || want_pub) && count > 0) {
            frame_viz = frame.clone();
            prev_pts_copy = prev_pts;
            curr_pts_copy = curr_pts;
            status_copy = status;
            show_this_frame = true;
        }

        publish_flow_debug(mean_dx_px, mean_dx_norm, dbg_corr,
                           static_cast<double>(n_good), static_cast<double>(n_corners));

        prev_gray_ = frame;
    }

    if (show_this_frame) {
        cv::Mat vis;
        cv::cvtColor(frame_viz, vis, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < status_copy.size(); ++i) {
            if (!status_copy[i]) continue;
            const cv::Point2f& a = prev_pts_copy[i];
            const cv::Point2f& b = curr_pts_copy[i];
            cv::line(vis, a, b, cv::Scalar(0, 255, 120), 1, cv::LINE_AA);
            cv::circle(vis, a, 2, cv::Scalar(255, 80, 0), -1, cv::LINE_AA);
            cv::circle(vis, b, 2, cv::Scalar(200, 0, 255), -1, cv::LINE_AA);
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "mean_dx_px " << mean_dx_px << "  norm " << mean_dx_norm
            << "  corr " << dbg_corr << "  ok " << n_good << "/" << n_corners;
        cv::putText(vis, oss.str(), cv::Point(6, 18), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

        const int sc = std::clamp(
            static_cast<int>(get_parameter("debug_window_scale").as_int()), 1, 8);
        if (sc > 1) {
            cv::Mat big;
            cv::resize(vis, big, cv::Size(), static_cast<double>(sc), static_cast<double>(sc),
                        cv::INTER_NEAREST);
            vis = big;
        }
        if (want_pub) {
            std::vector<uchar> jpeg;
            const std::vector<int> enc_params = {cv::IMWRITE_JPEG_QUALITY, 80};
            if (cv::imencode(".jpg", vis, jpeg, enc_params)) {
                sensor_msgs::msg::CompressedImage out_img;
                out_img.header = msg->header;
                if (out_img.header.stamp.sec == 0 && out_img.header.stamp.nanosec == 0U) {
                    out_img.header.stamp = now();
                }
                out_img.format = "jpeg";
                out_img.data = std::move(jpeg);
                pub_viz_->publish(out_img);
            }
        }

        if (want_show) {
            const std::string win = get_parameter("debug_window_name").as_string();
            cv::imshow(win, vis);
            cv::waitKey(1);
        }
    }
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

    if (!get_parameter("enabled").as_bool()) {
        pub_cmd_->publish(out);
        return;
    }

    const bool moving_forward  = std::abs(out.linear.x) > forward_threshold_;
    const bool user_steering   = std::abs(out.angular.z) > steer_deadzone_;

    if (moving_forward && !user_steering) {
        out.angular.z = std::clamp(out.angular.z + correction, -1.5, 1.5);
    }

    pub_cmd_->publish(out);
}

}  // namespace nodes
