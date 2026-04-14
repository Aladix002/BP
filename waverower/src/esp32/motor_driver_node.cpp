// ROS 2 uzol: prijima /motor_cmd [left, right] a posiela HTTP GET na ESP32 (/js?json=...).
// Pouziva sa ako alternativa k PCA9685 I2C, ked motor driver bezi na ESP32.
#include "nodes/motor_driver_node.hpp"
#include <curl/curl.h>
#include <sstream>
#include <iomanip>

namespace nodes {

MotorDriverNode::MotorDriverNode() : Node("motor_driver") {
    this->declare_parameter<std::string>("esp32_ip", "192.168.0.224");
    std::string ip = this->get_parameter("esp32_ip").as_string();
    esp32_url_ = "http://" + ip + "/js";

    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/motor_cmd", 10, std::bind(&MotorDriverNode::motor_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Motor driver: /motor_cmd -> HTTP %s", esp32_url_.c_str());
}

MotorDriverNode::~MotorDriverNode() {
    send_http(0.0, 0.0);
}

// Prijme [left, right] v -1..1 a posle na ESP32
void MotorDriverNode::motor_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 2) return;
    double left = std::max(-1.0, std::min(1.0, msg->data[0]));
    double right = std::max(-1.0, std::min(1.0, msg->data[1]));
    send_http(left, right);
}

// Zostaví JSON {T:1, L:left, R:right} a posle cez libcurl GET na esp32_url
void MotorDriverNode::send_http(double left, double right) {
    std::ostringstream json;
    json << std::fixed << std::setprecision(2);
    json << "{\"T\":1,\"L\":" << left << ",\"R\":" << right << "}";
    std::string json_str = json.str();

    CURL* curl = curl_easy_init();
    if (!curl) return;

    char* encoded = curl_easy_escape(curl, json_str.c_str(), static_cast<int>(json_str.size()));
    std::string url = esp32_url_ + "?json=" + std::string(encoded);
    curl_free(encoded);

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);
    curl_easy_perform(curl);
    curl_easy_cleanup(curl);
}

}  // namespace nodes
