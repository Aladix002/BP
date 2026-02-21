#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

// Bridge node pre komunikáciu medzi ROS2 a ESP32
// Prijíma ROS2 správy a odosiela JSON príkazy cez Serial/UART na ESP32

class MotorESP32BridgeNode : public rclcpp::Node {
public:
    MotorESP32BridgeNode() : Node("motor_esp32_bridge") {
        // Parameter pre serial port (default: auto - automaticky hľadá)
        this->declare_parameter<std::string>("serial_port", "auto");
        std::string serial_port = this->get_parameter("serial_port").as_string();
        
        // Parameter pre baud rate (default: 115200)
        this->declare_parameter<int>("baud_rate", 115200);
        int baud_rate = this->get_parameter("baud_rate").as_int();
        
        // Ak je "auto", skús nájsť dostupný serial port
        if (serial_port == "auto") {
            serial_port = findAvailableSerialPort();
            if (serial_port.empty()) {
                RCLCPP_WARN(this->get_logger(), "No serial port found, will work in simulation mode");
                RCLCPP_WARN(this->get_logger(), "Available ports: /dev/ttyUSB0, /dev/ttyACM0, /dev/ttyAMA0, /dev/ttyS0");
                RCLCPP_WARN(this->get_logger(), "Or specify manually: --ros-args -p serial_port:=/dev/ttyAMA10");
                serial_fd_ = -1;
            } else {
                RCLCPP_INFO(this->get_logger(), "Auto-detected serial port: %s", serial_port.c_str());
            }
        }
        
        // Otvor serial port (ak nie je auto alebo sa našiel port)
        if (serial_fd_ == -1 && !serial_port.empty() && serial_port != "auto") {
            serial_fd_ = openSerialPort(serial_port, baud_rate);
        }
        
        if (serial_fd_ < 0) {
            if (!serial_port.empty() && serial_port != "auto") {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port.c_str());
                RCLCPP_ERROR(this->get_logger(), "Make sure ESP32 is connected and you have permissions");
                RCLCPP_ERROR(this->get_logger(), "Try: sudo chmod 666 %s", serial_port.c_str());
                RCLCPP_WARN(this->get_logger(), "Continuing in simulation mode (commands will be logged only)");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Serial port opened: %s at %d baud", 
                       serial_port.c_str(), baud_rate);
        }
        
        // Vytvor subscriber pre motor commands
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 10,
            std::bind(&MotorESP32BridgeNode::motor_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Motor ESP32 Bridge Node started");
        RCLCPP_INFO(this->get_logger(), "Listening on /bpc_prp_robot/set_motor_speeds");
        if (serial_fd_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Sending JSON commands to ESP32 via %s", serial_port.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Running in simulation mode - commands will be logged only");
        }
    }
    
    ~MotorESP32BridgeNode() {
        // Zastav motory pred ukončením
        sendMotorCommand(0, 0);
        
        // Zatvor serial port
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
    }

private:
    std::string findAvailableSerialPort() {
        // Zoznam možných serial portov (v poradí priority)
        std::vector<std::string> possible_ports = {
            "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
            "/dev/ttyACM0", "/dev/ttyACM1",
            "/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyAMA10",
            "/dev/ttyS0", "/dev/ttyS1"
        };
        
        for (const auto& port : possible_ports) {
            // Skontroluj, či port existuje
            struct stat st;
            if (stat(port.c_str(), &st) == 0) {
                // Skús otvoriť port (len na test)
                int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
                if (fd >= 0) {
                    close(fd);
                    RCLCPP_INFO(this->get_logger(), "Found available port: %s", port.c_str());
                    return port;
                }
            }
        }
        return "";
    }
    
    int openSerialPort(const std::string& port, int baud_rate) {
        // Otvor serial port
        int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            return -1;
        }
        
        // Konfiguruj termios
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            return -1;
        }
        
        // Nastav baud rate
        speed_t speed;
        switch (baud_rate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            default: speed = B115200; break;
        }
        
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);
        
        // Nastav raw mode
        tty.c_cflag &= ~PARENB;        // No parity
        tty.c_cflag &= ~CSTOPB;        // 1 stop bit
        tty.c_cflag &= ~CSIZE;         // Clear size bits
        tty.c_cflag |= CS8;            // 8 data bits
        tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem controls
        
        tty.c_lflag &= ~ICANON;        // Disable canonical mode
        tty.c_lflag &= ~ECHO;          // Disable echo
        tty.c_lflag &= ~ECHOE;         // Disable erasure
        tty.c_lflag &= ~ECHONL;        // Disable new-line echo
        tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        tty.c_oflag &= ~OPOST;         // Disable post-processing
        tty.c_oflag &= ~ONLCR;         // Disable conversion of newline to carriage return/line feed
        
        // Nastav timeout
        tty.c_cc[VMIN] = 0;            // Non-blocking read
        tty.c_cc[VTIME] = 10;          // 1 second timeout
        
        // Aplikuj nastavenia
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            return -1;
        }
        
        return fd;
    }
    
    void motor_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            uint8_t left_speed = msg->data[0];
            uint8_t right_speed = msg->data[1];
            
            // Konvertuj z 0-255 formátu (128 = stop) na -255 až 255
            int left_pwm = static_cast<int>(left_speed) - 128;
            int right_pwm = static_cast<int>(right_speed) - 128;
            
            RCLCPP_INFO(this->get_logger(), "Received command: L=%d, R=%d -> PWM: L=%d, R=%d", 
                        left_speed, right_speed, left_pwm, right_pwm);
            
            sendMotorCommand(left_pwm, right_pwm);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid motor command: expected 2 values, got %zu", 
                       msg->data.size());
        }
    }
    
    void sendMotorCommand(int left_pwm, int right_pwm) {
        // Obmedz hodnoty na rozsah -255 až 255
        left_pwm = std::max(-255, std::min(255, left_pwm));
        right_pwm = std::max(-255, std::min(255, right_pwm));
        
        // Vytvor JSON príkaz podľa ESP32 formátu
        // CMD_PWM_INPUT = 11: {"T":11,"L":164,"R":164}
        std::ostringstream json_stream;
        json_stream << "{\"T\":11,\"L\":" << left_pwm << ",\"R\":" << right_pwm << "}\n";
        
        std::string json_cmd = json_stream.str();
        
        // Ak je serial port otvorený, odosli cez neho
        if (serial_fd_ >= 0) {
            // Odstráň \n pre logovanie (bez newline)
            std::string json_cmd_no_nl = json_cmd;
            if (!json_cmd_no_nl.empty() && json_cmd_no_nl.back() == '\n') {
                json_cmd_no_nl.pop_back();
            }
            RCLCPP_INFO(this->get_logger(), "Sending JSON: %s", json_cmd_no_nl.c_str());
            
            ssize_t bytes_written = write(serial_fd_, json_cmd.c_str(), json_cmd.length());
            
            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", strerror(errno));
            } else if (static_cast<size_t>(bytes_written) != json_cmd.length()) {
                RCLCPP_WARN(this->get_logger(), "Partial write: %zd/%zu bytes", bytes_written, json_cmd.length());
            } else {
                RCLCPP_INFO(this->get_logger(), "Successfully sent %zd bytes to ESP32", bytes_written);
            }
            
            // Flush output
            fsync(serial_fd_);
        } else {
            // Simulation mode - len loguj
            RCLCPP_INFO(this->get_logger(), "SIMULATION: Would send JSON: %s", json_cmd.c_str());
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscriber_;
    int serial_fd_ = -1;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorESP32BridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

