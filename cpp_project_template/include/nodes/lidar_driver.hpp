#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <fstream>

namespace nodes {

    /**
     * LIDAR Driver Node
     * 
     * Číta dáta z USB LIDARu cez seriový port a publikuje ich ako LaserScan
     * Podporuje YDLIDAR a RPLIDAR protokoly
     */
    class LidarDriverNode : public rclcpp::Node {
    public:
        LidarDriverNode();
        ~LidarDriverNode() override;

    private:
        // Konfigurácia
        std::string serial_port_;
        int baud_rate_;
        std::string lidar_type_; // "ydlidar", "rplidar", alebo "d300"
        
        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
        
        // Thread pre čítanie dát zo seriového portu
        std::thread read_thread_;
        std::atomic<bool> running_;
        
        // Funkcie
        void read_serial_data();
        bool open_serial_port();
        void close_serial_port();
        void parse_ydlidar_data(const std::vector<uint8_t>& data);
        void parse_rplidar_data(const std::vector<uint8_t>& data);
        void parse_d300_data(const std::vector<uint8_t>& data);
        void publish_laser_scan(const std::vector<float>& ranges, float angle_min, float angle_max, 
                               float range_min, float range_max, float scan_time);
        
        // Seriový port file descriptor
        int serial_fd_;
        
        // LIDAR parametre
        float angle_min_;
        float angle_max_;
        float range_min_;
        float range_max_;
        int num_readings_;
    };

}

