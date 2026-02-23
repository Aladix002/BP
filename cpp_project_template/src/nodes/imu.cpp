#include "nodes/imu.hpp"
#include <curl/curl.h>
#include <chrono>
#include <cmath>
#include <cstring>

namespace nodes {

static size_t write_callback(void* ptr, size_t size, size_t nmemb, void* user) {
    size_t len = size * nmemb;
    std::string* s = static_cast<std::string*>(user);
    s->append(static_cast<const char*>(ptr), len);
    return len;
}

// Vytiahnutie double z JSON podla kluca "key": hladam "key": a potom cislo.
static bool json_get_double(const std::string& json, const char* key, double& out) {
    std::string search = "\"";
    search += key;
    search += "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return false;
    pos += search.size();
    const char* p = json.c_str() + pos;
    char* end = nullptr;
    out = std::strtod(p, &end);
    return end != p;
}

void ImuHttpNode::euler_to_quaternion(double roll, double pitch, double yaw,
                                      double& qx, double& qy, double& qz, double& qw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

ImuHttpNode::ImuHttpNode() : Node("imu_http_node") {
    this->declare_parameter<std::string>("esp32_ip", "192.168.0.224");
    this->declare_parameter<double>("publish_rate", 50.0);
    std::string ip = this->get_parameter("esp32_ip").as_string();
    esp32_url_ = "http://" + ip + "/js";
    publish_rate_hz_ = this->get_parameter("publish_rate").as_double();
    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 50.0;

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(static_cast<int64_t>(1e6 / publish_rate_hz_)),
        std::bind(&ImuHttpNode::timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "IMU HTTP: %s (T:126), rate %.0f Hz -> /imu", esp32_url_.c_str(), publish_rate_hz_);
}

bool ImuHttpNode::fetch_imu_http(std::string* response_out) {
    const std::string json_cmd = "{\"T\":126}";
    CURL* curl = curl_easy_init();
    if (!curl || !response_out) return false;
    response_out->clear();

    char* encoded = curl_easy_escape(curl, json_cmd.c_str(), static_cast<int>(json_cmd.size()));
    std::string url = esp32_url_ + "?json=" + std::string(encoded);
    curl_free(encoded);

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, response_out);
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    return (res == CURLE_OK && !response_out->empty());
}

bool ImuHttpNode::parse_imu_json(const std::string& json, sensor_msgs::msg::Imu& imu_out) {
    double r = 0, p = 0, y = 0;
    double ax = 0, ay = 0, az = 0;
    double gx = 0, gy = 0, gz = 0;
    if (!json_get_double(json, "r", r)) return false;
    if (!json_get_double(json, "p", p)) return false;
    if (!json_get_double(json, "y", y)) return false;
    json_get_double(json, "ax", ax);
    json_get_double(json, "ay", ay);
    json_get_double(json, "az", az);
    json_get_double(json, "gx", gx);
    json_get_double(json, "gy", gy);
    json_get_double(json, "gz", gz);

    imu_out.header.stamp = this->now();
    imu_out.header.frame_id = "imu_link";

    double qx, qy, qz, qw;
    euler_to_quaternion(r, p, y, qx, qy, qz, qw);
    imu_out.orientation.x = qx;
    imu_out.orientation.y = qy;
    imu_out.orientation.z = qz;
    imu_out.orientation.w = qw;

    imu_out.angular_velocity.x = gx;
    imu_out.angular_velocity.y = gy;
    imu_out.angular_velocity.z = gz;

    imu_out.linear_acceleration.x = ax;
    imu_out.linear_acceleration.y = ay;
    imu_out.linear_acceleration.z = az;

    imu_out.orientation_covariance[0] = -1.0;
    imu_out.angular_velocity_covariance[0] = -1.0;
    imu_out.linear_acceleration_covariance[0] = -1.0;
    return true;
}

void ImuHttpNode::timer_cb() {
    std::string response;
    if (!fetch_imu_http(&response)) return;
    sensor_msgs::msg::Imu imu_msg;
    if (parse_imu_json(response, imu_msg))
        pub_imu_->publish(imu_msg);
}

}  // namespace nodes
