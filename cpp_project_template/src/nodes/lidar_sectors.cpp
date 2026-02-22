#include "nodes/lidar_sectors.hpp"
#include <cmath>

namespace nodes {

static float normalize_angle(float a) {
    while (a > M_PI) a -= 2.f * static_cast<float>(M_PI);
    while (a < -M_PI) a += 2.f * static_cast<float>(M_PI);
    return a;
}

void fill_sectors_60(const sensor_msgs::msg::LaserScan& msg, float lidar_front_offset_rad, LidarSectors60& out) {
    out.front_min = 1e9f;
    out.left_min = 1e9f;
    out.right_min = 1e9f;
    out.valid = false;

    if (msg.ranges.empty()) return;

    float angle_min = msg.angle_min;
    float inc = msg.angle_increment;
    float front_lo = -M_PI / 6.f;   // -30°
    float front_hi =  M_PI / 6.f;   // +30°
    float left_lo   =  M_PI / 6.f;
    float left_hi   =  M_PI / 2.f;  // 90°
    float right_lo  = -M_PI / 2.f;
    float right_hi  = -M_PI / 6.f;

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
        float r = msg.ranges[i];
        if (std::isnan(r) || std::isinf(r) || r < msg.range_min || r > msg.range_max) continue;
        float angle = normalize_angle(angle_min + static_cast<float>(i) * inc - lidar_front_offset_rad);

        if (angle >= front_lo && angle <= front_hi) {
            if (r < out.front_min) out.front_min = r;
        } else if (angle >= left_lo && angle <= left_hi) {
            if (r < out.left_min) out.left_min = r;
        } else if (angle >= right_lo && angle <= right_hi) {
            if (r < out.right_min) out.right_min = r;
        }
    }

    out.valid = (out.front_min < 1e8f || out.left_min < 1e8f || out.right_min < 1e8f);
}

}  // namespace nodes
