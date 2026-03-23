#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cstdint>

namespace nodes {

/**
 * Čítanie IMU priamo cez Linux i2c-dev (bez ESP32).
 *
 * Podporovaný čip: MPU-6050 (I2C 0x68 / 0x69).
 * Publikuje sensor_msgs/Imu na /imu.
 *
 * Parametre:
 *   i2c_bus          – číslo I2C zbernice (default 1 → /dev/i2c-1)
 *   i2c_address      – adresa čipu, hex (default 0x68)
 *   publish_rate_hz  – frekvencia publikovania (default 100.0)
 *   gyro_range_dps   – rozsah gyroskopu: 250 | 500 | 1000 | 2000 (default 250)
 *   accel_range_g    – rozsah akcelerometra: 2 | 4 | 8 | 16 (default 2)
 *   frame_id         – TF frame (default "imu_link")
 *   imu_topic        – výstupný topic (default "/imu")
 */
class ImuI2cNode : public rclcpp::Node {
public:
    ImuI2cNode();
    ~ImuI2cNode() override;

private:
    void timer_cb();
    bool init_device();
    bool read_raw(int16_t& ax, int16_t& ay, int16_t& az,
                  int16_t& gx, int16_t& gy, int16_t& gz);

    int i2c_fd_{-1};
    int i2c_bus_{1};
    int i2c_address_{0x68};
    double publish_rate_hz_{100.0};
    double accel_scale_{1.0 / 16384.0 * 9.80665};  // ±2g
    double gyro_scale_{1.0 / 131.0 * M_PI / 180.0}; // ±250°/s → rad/s
    std::string frame_id_{"imu_link"};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nodes
