#include "nodes/imu_i2c.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

// MPU-6050 register addresses
namespace {
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // 14 bytes: accel(6) + temp(2) + gyro(6)
constexpr uint8_t REG_WHO_AM_I     = 0x75;   // should read 0x68

// Scale faktory podľa konfigurácie
// GYRO_CONFIG: FS_SEL 0=250°/s, 1=500°/s, 2=1000°/s, 3=2000°/s
// ACCEL_CONFIG: AFS_SEL 0=±2g, 1=±4g, 2=±8g, 3=±16g
double gyro_lsb_per_dps(int range_dps) {
    switch (range_dps) {
        case 500:  return 65.5;
        case 1000: return 32.8;
        case 2000: return 16.4;
        default:   return 131.0;  // 250 dps
    }
}
uint8_t gyro_fs_sel(int range_dps) {
    switch (range_dps) {
        case 500:  return 0x08;
        case 1000: return 0x10;
        case 2000: return 0x18;
        default:   return 0x00;
    }
}
double accel_lsb_per_g(int range_g) {
    switch (range_g) {
        case 4:  return 8192.0;
        case 8:  return 4096.0;
        case 16: return 2048.0;
        default: return 16384.0;  // ±2g
    }
}
uint8_t accel_afs_sel(int range_g) {
    switch (range_g) {
        case 4:  return 0x08;
        case 8:  return 0x10;
        case 16: return 0x18;
        default: return 0x00;
    }
}
}  // namespace

namespace nodes {

ImuI2cNode::ImuI2cNode() : Node("imu_i2c_node") {
    i2c_bus_         = declare_parameter<int>("i2c_bus", 1);
    i2c_address_     = declare_parameter<int>("i2c_address", 0x68);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    frame_id_        = declare_parameter<std::string>("frame_id", "imu_link");
    int gyro_range   = declare_parameter<int>("gyro_range_dps", 250);
    int accel_range  = declare_parameter<int>("accel_range_g", 2);
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu");

    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 100.0;

    // Scale faktory
    gyro_scale_  = (1.0 / gyro_lsb_per_dps(gyro_range))  * (M_PI / 180.0);  // rad/s
    accel_scale_ = (1.0 / accel_lsb_per_g(accel_range))  * 9.80665;          // m/s²

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());

    if (!init_device()) {
        throw std::runtime_error("IMU I2C: nepodarilo sa inicializovat /dev/i2c-"
                                 + std::to_string(i2c_bus_)
                                 + " addr=0x" + std::to_string(i2c_address_));
    }

    // Nakonfiguruj rozsahy
    uint8_t buf[2];
    buf[0] = REG_GYRO_CONFIG; buf[1] = gyro_fs_sel(gyro_range);
    if (write(i2c_fd_, buf, 2) != 2) {
        RCLCPP_WARN(get_logger(), "IMU: zapis GYRO_CONFIG zlyhal");
    }
    buf[0] = REG_ACCEL_CONFIG; buf[1] = accel_afs_sel(accel_range);
    if (write(i2c_fd_, buf, 2) != 2) {
        RCLCPP_WARN(get_logger(), "IMU: zapis ACCEL_CONFIG zlyhal");
    }

    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1.0e6 / publish_rate_hz_));
    timer_ = create_wall_timer(period, [this] { timer_cb(); });

    RCLCPP_INFO(get_logger(),
        "IMU I2C: /dev/i2c-%d addr=0x%02x  gyro±%d°/s  accel±%dg  %.0fHz → %s",
        i2c_bus_, i2c_address_, gyro_range, accel_range, publish_rate_hz_, imu_topic.c_str());
}

ImuI2cNode::~ImuI2cNode() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

bool ImuI2cNode::init_device() {
    const std::string dev = "/dev/i2c-" + std::to_string(i2c_bus_);
    i2c_fd_ = open(dev.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "IMU: open %s zlyhalo: %s", dev.c_str(), strerror(errno));
        return false;
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
        RCLCPP_ERROR(get_logger(), "IMU: ioctl I2C_SLAVE 0x%02x zlyhalo: %s",
                     i2c_address_, strerror(errno));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // WHO_AM_I check (MPU-6050 vráti 0x68, MPU-6500 / MPU-9250 vráti 0x70/0x71)
    uint8_t reg = REG_WHO_AM_I;
    uint8_t who = 0;
    if (write(i2c_fd_, &reg, 1) == 1 && read(i2c_fd_, &who, 1) == 1) {
        RCLCPP_INFO(get_logger(), "IMU WHO_AM_I = 0x%02x", who);
    }

    // Prebuď čip: PWR_MGMT_1 = 0x01 (clock z gyro X, najstabilnejší)
    uint8_t buf[2] = { REG_PWR_MGMT_1, 0x01 };
    if (write(i2c_fd_, buf, 2) != 2) {
        RCLCPP_ERROR(get_logger(), "IMU: prebúdzanie zlyhalo");
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }
    return true;
}

bool ImuI2cNode::read_raw(int16_t& ax, int16_t& ay, int16_t& az,
                           int16_t& gx, int16_t& gy, int16_t& gz) {
    if (i2c_fd_ < 0) return false;

    // Nastav register pointer na ACCEL_XOUT_H
    uint8_t reg = REG_ACCEL_XOUT_H;
    if (write(i2c_fd_, &reg, 1) != 1) return false;

    // Prečítaj 14 bytov: ax_h ax_l ay_h ay_l az_h az_l temp_h temp_l gx_h gx_l gy_h gy_l gz_h gz_l
    uint8_t buf[14];
    if (read(i2c_fd_, buf, 14) != 14) return false;

    ax = static_cast<int16_t>((buf[0]  << 8) | buf[1]);
    ay = static_cast<int16_t>((buf[2]  << 8) | buf[3]);
    az = static_cast<int16_t>((buf[4]  << 8) | buf[5]);
    // buf[6..7] = temperature, skip
    gx = static_cast<int16_t>((buf[8]  << 8) | buf[9]);
    gy = static_cast<int16_t>((buf[10] << 8) | buf[11]);
    gz = static_cast<int16_t>((buf[12] << 8) | buf[13]);
    return true;
}

void ImuI2cNode::timer_cb() {
    int16_t ax, ay, az, gx, gy, gz;
    if (!read_raw(ax, ay, az, gx, gy, gz)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU: citanie zlyhalo");
        return;
    }

    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = now();
    msg.header.frame_id = frame_id_;

    msg.linear_acceleration.x = ax * accel_scale_;
    msg.linear_acceleration.y = ay * accel_scale_;
    msg.linear_acceleration.z = az * accel_scale_;

    msg.angular_velocity.x = gx * gyro_scale_;
    msg.angular_velocity.y = gy * gyro_scale_;
    msg.angular_velocity.z = gz * gyro_scale_;

    // Orientácia nie je k dispozícii z raw dát (potrebný filter); nastav kovarianciu na -1
    msg.orientation_covariance[0]         = -1.0;
    msg.angular_velocity_covariance[0]    = 1e-4;
    msg.linear_acceleration_covariance[0] = 1e-3;

    pub_imu_->publish(msg);
}

}  // namespace nodes
