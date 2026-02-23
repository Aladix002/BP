#include "nodes/lidar.hpp"
#include <cmath>

namespace nodes {

// Hodnota pre "ziadna stena / neplatne" v metroch
static constexpr double NO_DATA = 1e9;

// Len front: uhol +/-45 st. (90 st. spolu)
static float normalize_angle(float a) {
    while (a > static_cast<float>(M_PI)) a -= 2.f * static_cast<float>(M_PI);
    while (a < -static_cast<float>(M_PI)) a += 2.f * static_cast<float>(M_PI);
    return a;
}

static void fill_front_only(const sensor_msgs::msg::LaserScan& msg, float lidar_front_offset_rad, float& front_min, bool& valid) {
    front_min = 1e9f;
    valid = false;
    if (msg.ranges.empty()) return;
    float angle_min = msg.angle_min;
    float inc = msg.angle_increment;
    float front_lo = -static_cast<float>(M_PI) / 4.f;   // -45 st.
    float front_hi =  static_cast<float>(M_PI) / 4.f;   // +45 st.

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
        float r = msg.ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < msg.range_min || r > msg.range_max) continue;
        float angle = normalize_angle(angle_min + static_cast<float>(i) * inc - lidar_front_offset_rad);
        if (angle >= front_lo && angle <= front_hi) {
            if (r < front_min) front_min = r;
        }
    }
    valid = (front_min < 1e8f);
}

// --- Node ---

LidarSectorsNode::LidarSectorsNode() : Node("lidar_sectors") {
    // Default 90 st. dolava: ak je lidarova 0 na pravom boku, front = smer jazdy
    this->declare_parameter<double>("lidar_front_offset_rad", M_PI / 2.0);
    lidar_front_offset_rad_ = static_cast<float>(this->get_parameter("lidar_front_offset_rad").as_double());

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LidarSectorsNode::scan_cb, this, std::placeholders::_1));
    pub_sectors_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lidar_sectors", 10);

    RCLCPP_INFO(this->get_logger(), "Lidar: len front 90 st. /scan -> /lidar_sectors [front] m");
}

void LidarSectorsNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float front_min;
    bool valid;
    fill_front_only(*msg, lidar_front_offset_rad_, front_min, valid);

    std_msgs::msg::Float64MultiArray out;
    out.data = { valid ? static_cast<double>(front_min) : NO_DATA };
    pub_sectors_->publish(out);
}

}  // namespace nodes
