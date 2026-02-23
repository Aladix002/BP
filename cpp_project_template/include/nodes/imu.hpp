#ifndef NODES_IMU_HTTP_NODE_HPP
#define NODES_IMU_HTTP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace nodes {

/** Stahuje IMU z ESP32 cez HTTP (/js, T:126) a publikuje sensor_msgs/Imu na /imu. */
class ImuHttpNode : public rclcpp::Node {
public:
    ImuHttpNode();

private:
    void timer_cb();
    bool fetch_imu_http(std::string* response_out);
    bool parse_imu_json(const std::string& json, sensor_msgs::msg::Imu& imu_out);
    static void euler_to_quaternion(double roll, double pitch, double yaw,
        double& qx, double& qy, double& qz, double& qw);

    std::string esp32_url_;
    double publish_rate_hz_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
};

}  // namespace nodes

#endif
