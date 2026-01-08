#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <memory>
#include <chrono>
#include <map>

namespace nodes {

    /**
     * ESP32 Bridge Node
     * 
     * Komunikuje s ESP32 cez HTTP a poskytuje ROS 2 topics pre:
     * - Motor control (speed, PWM, ROS ctrl)
     * - IMU data publishing
     */
    class Esp32Bridge : public rclcpp::Node {
    public:
        Esp32Bridge();
        ~Esp32Bridge() override;

    private:
        // ESP32 configuration
        std::string esp32_ip_;
        std::string esp32_url_;
        int imu_poll_rate_ms_;
        
        // HTTP client (using curl or similar)
        void send_json_command(const std::string& json_cmd);
        std::string send_json_command_with_response(const std::string& json_cmd);
        
        // Left and right side motor control subscribers
        // Hodnoty 0-255, kde 127 = stoj, 0-126 = dozadu, 128-255 = dopredu
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_motor_left_subscriber_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_motor_right_subscriber_;
        
        // Store last motor values for left and right side
        // Hodnoty 0-255 (127 = stoj)
        uint8_t last_motor_left_;
        uint8_t last_motor_right_;
        
        // Flags to track which side was last updated
        bool left_side_active_;
        bool right_side_active_;
        
        // IMU publisher
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        
        // Timer for IMU polling
        rclcpp::TimerBase::SharedPtr imu_timer_;
        
        // Callbacks
        void on_cmd_motor_left(const std_msgs::msg::UInt8::SharedPtr msg);
        void on_cmd_motor_right(const std_msgs::msg::UInt8::SharedPtr msg);
        void poll_imu_data();
        
        // Helper function to convert motor value (0-255, 127=stoj) to speed (-1.0 to +1.0)
        float convert_motor_value_to_speed(uint8_t value);
        void send_motor_command();
        
        // Helper functions
        std::string build_json_cmd(int cmd_type, const std::map<std::string, std::string>& params);
        void parse_imu_response(const std::string& json_response);
    };
}

