#include "nodes/esp32_bridge.hpp"
#include <curl/curl.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <regex>
#include <cmath>

using namespace nodes;

Esp32Bridge::~Esp32Bridge() {
    curl_global_cleanup();
}

// Callback for curl write function
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

Esp32Bridge::Esp32Bridge() 
    : Node("esp32_bridge") {
    
    // Declare parameters
    this->declare_parameter<std::string>("esp32_ip", "192.168.0.224");
    this->declare_parameter<int>("imu_poll_rate_ms", 50); // 20 Hz default
    
    // Get parameters
    esp32_ip_ = this->get_parameter("esp32_ip").as_string();
    imu_poll_rate_ms_ = this->get_parameter("imu_poll_rate_ms").as_int();
    
    esp32_url_ = "http://" + esp32_ip_ + "/js";
    
    RCLCPP_INFO(this->get_logger(), "ESP32 Bridge initialized");
    RCLCPP_INFO(this->get_logger(), "ESP32 IP: %s", esp32_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "ESP32 URL: %s", esp32_url_.c_str());
    
    // Initialize curl
    curl_global_init(CURL_GLOBAL_DEFAULT);
    
    // Create subscribers for left and right side motor control
    // Hodnoty 0-255, kde 127 = stoj, 0-126 = dozadu, 128-255 = dopredu
    cmd_motor_left_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/bpc_prp_robot/cmd_motor_left",
        10,
        std::bind(&Esp32Bridge::on_cmd_motor_left, this, std::placeholders::_1)
    );
    
    cmd_motor_right_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/bpc_prp_robot/cmd_motor_right",
        10,
        std::bind(&Esp32Bridge::on_cmd_motor_right, this, std::placeholders::_1)
    );
    
    // Initialize last motor values - obe strany na stoj (127)
    last_motor_left_ = 127;  // 127 = stoj
    last_motor_right_ = 127; // 127 = stoj
    
    // Initialize flags - žiadna strana nie je aktívna
    left_side_active_ = false;
    right_side_active_ = false;
    
    // Create IMU publisher
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu",
        10
    );
    
    // Create timer for IMU polling
    imu_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(imu_poll_rate_ms_),
        std::bind(&Esp32Bridge::poll_imu_data, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "ESP32 Bridge ready");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /bpc_prp_robot/cmd_motor_left");
    RCLCPP_INFO(this->get_logger(), "Subscribed to /bpc_prp_robot/cmd_motor_right");
}

void Esp32Bridge::send_json_command(const std::string& json_cmd) {
    CURL *curl;
    CURLcode res;
    
    curl = curl_easy_init();
    if (curl) {
        // ESP32 očakáva GET request s query parametrom json=...
        // curl -G --data-urlencode 'json=...' vytvorí GET request s URL-encoded query parametrom
        // Použijeme curl_easy_escape pre URL encoding a vytvoríme URL s query parametrom
        char* encoded = curl_easy_escape(curl, json_cmd.c_str(), json_cmd.length());
        std::string url_with_param = esp32_url_ + "?json=" + std::string(encoded);
        curl_free(encoded);
        
        curl_easy_setopt(curl, CURLOPT_URL, url_with_param.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        
        res = curl_easy_perform(curl);
        
        if (res != CURLE_OK) {
            RCLCPP_WARN(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
        } else {
            long response_code;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            RCLCPP_INFO(this->get_logger(), "HTTP GET sent: %s (response: %ld)", url_with_param.c_str(), response_code);
        }
        
        curl_easy_cleanup(curl);
    }
}

std::string Esp32Bridge::send_json_command_with_response(const std::string& json_cmd) {
    CURL *curl;
    CURLcode res;
    std::string readBuffer;
    
    curl = curl_easy_init();
    if (curl) {
        // ESP32 očakáva GET request s query parametrom json=...
        // curl -G --data-urlencode 'json=...' vytvorí GET request s URL-encoded query parametrom
        char* encoded = curl_easy_escape(curl, json_cmd.c_str(), json_cmd.length());
        std::string url_with_param = esp32_url_ + "?json=" + std::string(encoded);
        curl_free(encoded);
        
        curl_easy_setopt(curl, CURLOPT_URL, url_with_param.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        
        res = curl_easy_perform(curl);
        
        if (res != CURLE_OK) {
            RCLCPP_WARN(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
            readBuffer = "";
        }
        
        curl_easy_cleanup(curl);
    }
    
    return readBuffer;
}

// Left and right side motor callbacks
// Hodnoty 0-255, kde 127 = stoj
void Esp32Bridge::on_cmd_motor_left(const std_msgs::msg::UInt8::SharedPtr msg) {
    last_motor_left_ = msg->data;
    left_side_active_ = true;
    // Keď zmeníme ľavú stranu, ak pravá nie je aktívna, nastavíme ju na stoj
    if (!right_side_active_) {
        last_motor_right_ = 127; // stoj
    }
    send_motor_command();
    RCLCPP_DEBUG(this->get_logger(), "Motor Left: %d (127=stoj)", msg->data);
}

void Esp32Bridge::on_cmd_motor_right(const std_msgs::msg::UInt8::SharedPtr msg) {
    last_motor_right_ = msg->data;
    right_side_active_ = true;
    // Keď zmeníme pravú stranu, ak ľavá nie je aktívna, nastavíme ju na stoj
    if (!left_side_active_) {
        last_motor_left_ = 127; // stoj
    }
    send_motor_command();
    RCLCPP_DEBUG(this->get_logger(), "Motor Right: %d (127=stoj)", msg->data);
}

// Helper function to convert motor value (0-255, 127=stoj) to speed (-1.0 to +1.0)
float Esp32Bridge::convert_motor_value_to_speed(uint8_t value) {
    // 0-255 -> -1.0 až +1.0, kde 127 = 0.0 (stoj)
    // Formula: (value - 127) / 127.0
    // 0 -> -1.0, 127 -> 0.0, 255 -> +1.0
    float speed = (static_cast<float>(value) - 127.0f) / 127.0f;
    return std::clamp(speed, -1.0f, 1.0f);
}

void Esp32Bridge::send_motor_command() {
    // Konvertuj hodnoty 0-255 (127=stoj) na speed -1.0 až +1.0
    float left_speed = convert_motor_value_to_speed(last_motor_left_);
    float right_speed = convert_motor_value_to_speed(last_motor_right_);
    
    // Reset flags ak obe strany sú na stoj
    if (last_motor_left_ == 127 && last_motor_right_ == 127) {
        left_side_active_ = false;
        right_side_active_ = false;
    }
    
    // Pošli speed príkaz na ESP32 ako GET request s json=... parametrom
    // Z web_page.h: {"T":1,"L":left_speed,"R":right_speed}
    // Použijeme std::fixed a std::setprecision(1) pre konzistentný formát (ako curl)
    std::ostringstream json_stream;
    json_stream << std::fixed << std::setprecision(1);
    json_stream << "{\"T\":1,\"L\":" << left_speed << ",\"R\":" << right_speed << "}";
    std::string json_cmd = json_stream.str();
    send_json_command(json_cmd);
    
    RCLCPP_INFO(this->get_logger(), "Motors: Left=%d (%.3f), Right=%d (%.3f)", 
                 last_motor_left_, left_speed, last_motor_right_, right_speed);
    RCLCPP_INFO(this->get_logger(), "Sending JSON: %s", json_cmd.c_str());
}

void Esp32Bridge::poll_imu_data() {
    // CMD_GET_IMU_DATA: {"T":126}
    // Response: {"T":1002,"ax":...,"ay":...,"az":...,"gx":...,"gy":...,"gz":...,"mx":...,"my":...,"mz":...}
    
    std::string json_str = "{\"T\":126}";
    std::string response = send_json_command_with_response(json_str);
    
    if (response.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "No response from ESP32 for IMU data");
        return;
    }
    
    // Simple JSON parsing using regex (for basic parsing)
    // Look for T:1002 pattern
    std::regex t_pattern(R"("T"\s*:\s*(\d+))");
    std::smatch t_match;
    
    if (std::regex_search(response, t_match, t_pattern)) {
        int t_value = std::stoi(t_match[1].str());
        
        // Check if this is IMU data response (T=1002)
        if (t_value == 1002) {
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = "imu_link";
            
            // Parse values using regex
            std::regex value_pattern("\"([agm][xyz])\"\\s*:\\s*([+-]?\\d*\\.?\\d+)");
            std::sregex_iterator iter(response.begin(), response.end(), value_pattern);
            std::sregex_iterator end;
            
            for (; iter != end; ++iter) {
                std::smatch match = *iter;
                std::string key = match[1].str();
                double value = std::stod(match[2].str());
                
                if (key == "ax") imu_msg.linear_acceleration.x = value;
                else if (key == "ay") imu_msg.linear_acceleration.y = value;
                else if (key == "az") imu_msg.linear_acceleration.z = value;
                else if (key == "gx") imu_msg.angular_velocity.x = value;
                else if (key == "gy") imu_msg.angular_velocity.y = value;
                else if (key == "gz") imu_msg.angular_velocity.z = value;
            }
            
            imu_publisher_->publish(imu_msg);
        }
    }
}

std::string Esp32Bridge::build_json_cmd(int cmd_type, const std::map<std::string, std::string>& params) {
    std::ostringstream json_stream;
    json_stream << "{\"T\":" << cmd_type;
    
    for (const auto& [key, value] : params) {
        json_stream << ",\"" << key << "\":\"" << value << "\"";
    }
    
    json_stream << "}";
    return json_stream.str();
}

void Esp32Bridge::parse_imu_response(const std::string& json_response) {
    // This is handled in poll_imu_data()
}

