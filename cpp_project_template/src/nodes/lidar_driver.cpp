#include "nodes/lidar_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cmath>
#include <algorithm>

using namespace nodes;

LidarDriverNode::LidarDriverNode()
    : Node("lidar_driver"),
      serial_port_("/dev/ttyUSB0"),
      baud_rate_(230400),
      lidar_type_("d300"),
      running_(false),
      serial_fd_(-1),
      angle_min_(-M_PI),
      angle_max_(M_PI),
      range_min_(0.1f),
      range_max_(12.0f),
      num_readings_(360)
{
    // Deklaruj parametre
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 230400);
    this->declare_parameter<std::string>("lidar_type", "d300");
    this->declare_parameter<float>("angle_min", -M_PI);
    this->declare_parameter<float>("angle_max", M_PI);
    this->declare_parameter<float>("range_min", 0.1f);
    this->declare_parameter<float>("range_max", 12.0f);
    this->declare_parameter<int>("num_readings", 360);
    
    // Získaj parametre
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    lidar_type_ = this->get_parameter("lidar_type").as_string();
    angle_min_ = this->get_parameter("angle_min").as_double();
    angle_max_ = this->get_parameter("angle_max").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    num_readings_ = this->get_parameter("num_readings").as_int();
    
    RCLCPP_INFO(this->get_logger(), "LIDAR Driver inicializovaný");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
    RCLCPP_INFO(this->get_logger(), "LIDAR typ: %s", lidar_type_.c_str());
    
    // Vytvor publisher
    lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/bpc_prp_robot/lidar",
        10
    );
    
    // Otvor seriový port
    if (open_serial_port()) {
        RCLCPP_INFO(this->get_logger(), "Seriový port úspešne otvorený");
        running_ = true;
        read_thread_ = std::thread(&LidarDriverNode::read_serial_data, this);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Nepodarilo sa otvoriť seriový port %s", serial_port_.c_str());
        RCLCPP_WARN(this->get_logger(), "Skúste: ls -la /dev/ttyUSB* /dev/ttyACM*");
        RCLCPP_WARN(this->get_logger(), "Alebo použite existujúci ROS 2 driver (napr. ydlidar_ros2)");
    }
}

LidarDriverNode::~LidarDriverNode() {
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    close_serial_port();
}

bool LidarDriverNode::open_serial_port() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return false;
    }
    
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    // Nastav baud rate
    speed_t speed;
    switch (baud_rate_) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        default: speed = B115200; break;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // Nastav 8N1 (8 bitov, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Raw mode
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    tty.c_oflag &= ~OPOST;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; // 1 sekunda timeout
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    return true;
}

void LidarDriverNode::close_serial_port() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

void LidarDriverNode::read_serial_data() {
    std::vector<uint8_t> buffer;
    buffer.reserve(8192);
    
    while (rclcpp::ok() && running_) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(serial_fd_, &read_fds);
        
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        int select_result = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        
        if (select_result > 0 && FD_ISSET(serial_fd_, &read_fds)) {
            uint8_t temp_buffer[256];
            ssize_t bytes_read = read(serial_fd_, temp_buffer, sizeof(temp_buffer));
            
            if (bytes_read > 0) {
                buffer.insert(buffer.end(), temp_buffer, temp_buffer + bytes_read);
                
                // Parsuj dáta podľa typu LIDARu
                if (lidar_type_ == "ydlidar") {
                    parse_ydlidar_data(buffer);
                } else if (lidar_type_ == "rplidar") {
                    parse_rplidar_data(buffer);
                } else if (lidar_type_ == "d300") {
                    parse_d300_data(buffer);
                }
            } else if (bytes_read < 0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Chyba pri čítaní zo seriového portu");
            }
        } else if (select_result == 0) {
            // Timeout - žiadne dáta
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Čakám na dáta z LIDARu...");
        }
        
        // Obmedz veľkosť buffera
        if (buffer.size() > 16384) {
            buffer.clear();
        }
    }
}

void LidarDriverNode::parse_ydlidar_data(const std::vector<uint8_t>& data) {
    // TODO: Implementuj YDLIDAR protokol parsing
    // YDLIDAR používa vlastný protokol, ktorý musí byť parsovaný
    // Pre teraz vytvoríme jednoduchú simuláciu
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "YDLIDAR parsing nie je implementovaný. Použite ydlidar_ros2 driver.");
    
    // Pre testovanie vytvoríme prázdne dáta
    std::vector<float> ranges(num_readings_, std::numeric_limits<float>::quiet_NaN());
    publish_laser_scan(ranges, angle_min_, angle_max_, range_min_, range_max_, 0.1f);
}

void LidarDriverNode::parse_rplidar_data(const std::vector<uint8_t>& data) {
    // TODO: Implementuj RPLIDAR protokol parsing
    // RPLIDAR používa vlastný protokol, ktorý musí byť parsovaný
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "RPLIDAR parsing nie je implementovaný. Použite rplidar_ros2 driver.");
    
    // Pre testovanie vytvoríme prázdne dáta
    std::vector<float> ranges(num_readings_, std::numeric_limits<float>::quiet_NaN());
    publish_laser_scan(ranges, angle_min_, angle_max_, range_min_, range_max_, 0.1f);
}

void LidarDriverNode::parse_d300_data(const std::vector<uint8_t>& data) {
    // D300 používa podobný protokol ako LD19
    // Protokol: Start byte 0x54, data length, data, checksum
    // Odporúčame použiť ldlidar_stl_ros2 driver namiesto vlastného parsera
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "D300 parsing nie je implementovaný. Použite ldlidar_stl_ros2 driver.");
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                        "D300 dáta prijaté: %zu bajtov", data.size());
    
    // Pre testovanie vytvoríme prázdne dáta
    std::vector<float> ranges(num_readings_, std::numeric_limits<float>::quiet_NaN());
    publish_laser_scan(ranges, angle_min_, angle_max_, range_min_, range_max_, 0.1f);
}

void LidarDriverNode::publish_laser_scan(const std::vector<float>& ranges, 
                                         float angle_min, float angle_max,
                                         float range_min, float range_max, 
                                         float scan_time) {
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = "lidar_link";
    
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = (angle_max - angle_min) / ranges.size();
    scan_msg.time_increment = scan_time / ranges.size();
    scan_msg.scan_time = scan_time;
    scan_msg.range_min = range_min;
    scan_msg.range_max = range_max;
    scan_msg.ranges = ranges;
    
    lidar_publisher_->publish(scan_msg);
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publikované LaserScan: %zu čítaní", ranges.size());
}

