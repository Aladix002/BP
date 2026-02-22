#ifndef NODES_LIDAR_SECTORS_HPP
#define NODES_LIDAR_SECTORS_HPP

#include <sensor_msgs/msg/laser_scan.hpp>

namespace nodes {

/** Sektory po 60°: predok ±30°, ľavý 30°–90°, pravý -90° až -30° (v rad súradniciach po odčítaní offsetu). */
struct LidarSectors60 {
    float front_min = 1e9f;
    float left_min = 1e9f;
    float right_min = 1e9f;
    bool valid = false;
};

/** Vyplní sektory z LaserScan; angle_min + i*angle_increment - lidar_front_offset_rad. */
void fill_sectors_60(const sensor_msgs::msg::LaserScan& msg, float lidar_front_offset_rad, LidarSectors60& out);

}  // namespace nodes

#endif
