#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <thread>
#include <atomic>
#include <map>
#include <cmath>
#include <cstdlib>
#include <string>

// Motor control node pre Waveshare auto
// Motor driver je pripojený priamo na RPi GPIO piny
// GPIO piny podľa ugv_config.h z ESP32 kódu

#define PWMA 25         // Motor A PWM control  
#define AIN2 17         // Motor A input 2     
#define AIN1 21         // Motor A input 1     
#define BIN1 22         // Motor B input 1       
#define BIN2 23         // Motor B input 2       
#define PWMB 26         // Motor B PWM control  

class MotorSubscriberNode : public rclcpp::Node {
public:
    MotorSubscriberNode() : Node("motor_subscriber") {
        subscriber_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 10,
            std::bind(&MotorSubscriberNode::motor_callback, this, std::placeholders::_1));
        
        // Inicializuj GPIO piny
        initGPIO();
        
        // Spusti PWM threads pre software PWM
        pwm_duty_cycles_[PWMA] = 0;
        pwm_duty_cycles_[PWMB] = 0;
        pwm_threads_[PWMA] = std::thread(&MotorSubscriberNode::pwmThread, this, PWMA);
        pwm_threads_[PWMB] = std::thread(&MotorSubscriberNode::pwmThread, this, PWMB);
        
        RCLCPP_INFO(this->get_logger(), "Motor Subscriber Node started");
        RCLCPP_INFO(this->get_logger(), "Listening on /bpc_prp_robot/set_motor_speeds");
        RCLCPP_INFO(this->get_logger(), "GPIO pins: PWMA=%d, AIN1=%d, AIN2=%d, PWMB=%d, BIN1=%d, BIN2=%d",
                    PWMA, AIN1, AIN2, PWMB, BIN1, BIN2);
    }
    
    ~MotorSubscriberNode() {
        // Zastav motory
        setMotorSpeed(0, 0);
        
        // Zastav PWM threads
        for (auto& [pin, thread] : pwm_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

private:
    void motor_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            uint8_t left_speed = msg->data[0];
            uint8_t right_speed = msg->data[1];
            
            // Konvertuj z 0-255 formátu (128 = stop) na -255 až 255
            int left_pwm = static_cast<int>(left_speed) - 128;
            int right_pwm = static_cast<int>(right_speed) - 128;
            
            RCLCPP_INFO(this->get_logger(), "Received: L=%d, R=%d -> PWM: L=%d, R=%d", 
                        left_speed, right_speed, left_pwm, right_pwm);
            
            setMotorSpeed(left_pwm, right_pwm);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid motor command: expected 2 values, got %zu", 
                       msg->data.size());
        }
    }
    
    void initGPIO() {
        // Inicializuj všetky GPIO piny
        setupGPIO(AIN1);
        setupGPIO(AIN2);
        setupGPIO(BIN1);
        setupGPIO(BIN2);
        setupGPIO(PWMA);
        setupGPIO(PWMB);
        
        // Nastav všetky na LOW na začiatku
        setGPIO(AIN1, false);
        setGPIO(AIN2, false);
        setGPIO(BIN1, false);
        setGPIO(BIN2, false);
    }
    
    void setupGPIO(int pin) {
        // Použi gpioset z libgpiod
        std::string cmd = "gpioset gpiochip4 " + std::to_string(pin) + "=0";
        system(cmd.c_str());
    }
    
    void setGPIO(int pin, bool value) {
        std::string cmd = "gpioset gpiochip4 " + std::to_string(pin) + "=" + (value ? "1" : "0");
        int result = system(cmd.c_str());
        if (result != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set GPIO %d to %d", pin, value);
        }
    }
    
    void pwmThread(int pin) {
        // Software PWM thread - 1 kHz frequency
        const int pwm_period_us = 1000;  // 1 ms = 1000 Hz
        
        RCLCPP_INFO(this->get_logger(), "PWM thread started for pin %d", pin);
        
        while (rclcpp::ok()) {
            int duty_cycle = pwm_duty_cycles_[pin].load();
            
            if (duty_cycle >= 255) {
                // 100% duty cycle - drž GPIO HIGH nepretržite
                setGPIO(pin, true);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else if (duty_cycle > 0) {
                // PWM: toggle podľa duty cycle
                int high_time_us = (duty_cycle * pwm_period_us) / 255;
                int low_time_us = pwm_period_us - high_time_us;
                
                setGPIO(pin, true);
                std::this_thread::sleep_for(std::chrono::microseconds(high_time_us));
                
                if (low_time_us > 0) {
                    setGPIO(pin, false);
                    std::this_thread::sleep_for(std::chrono::microseconds(low_time_us));
                }
            } else {
                // 0% duty cycle - drž GPIO LOW
                setGPIO(pin, false);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void setMotorSpeed(int pwmA, int pwmB) {
        // Podľa switchPortCtrlA a switchPortCtrlB z ESP32 kódu
        // pwmA, pwmB: -255 až 255 (0 = stop)
        
        // Left motor (Motor A)
        if (abs(pwmA) < 1) {
            // Stop
            setGPIO(AIN1, false);
            setGPIO(AIN2, false);
            setGPIO(PWMA, false);  // PWM LOW
            pwm_duty_cycles_[PWMA] = 0;
        } else if (pwmA > 0) {
            // Forward: AIN1=LOW, AIN2=HIGH
            setGPIO(AIN1, false);
            setGPIO(AIN2, true);
            // Pre test: nastav PWM na HIGH (100% duty cycle)
            setGPIO(PWMA, true);
            pwm_duty_cycles_[PWMA] = 255;  // Max pre software PWM
        } else {
            // Reverse: AIN1=HIGH, AIN2=LOW
            setGPIO(AIN1, true);
            setGPIO(AIN2, false);
            setGPIO(PWMA, true);
            pwm_duty_cycles_[PWMA] = 255;
        }
        
        // Right motor (Motor B)
        if (abs(pwmB) < 1) {
            // Stop
            setGPIO(BIN1, false);
            setGPIO(BIN2, false);
            setGPIO(PWMB, false);  // PWM LOW
            pwm_duty_cycles_[PWMB] = 0;
        } else if (pwmB > 0) {
            // Forward: BIN1=LOW, BIN2=HIGH
            setGPIO(BIN1, false);
            setGPIO(BIN2, true);
            setGPIO(PWMB, true);
            pwm_duty_cycles_[PWMB] = 255;
        } else {
            // Reverse: BIN1=HIGH, BIN2=LOW
            setGPIO(BIN1, true);
            setGPIO(BIN2, false);
            setGPIO(PWMB, true);
            pwm_duty_cycles_[PWMB] = 255;
        }
        
        RCLCPP_INFO(this->get_logger(), "Motor A: pwm=%d -> AIN1=%d AIN2=%d PWMA=%d", 
                    pwmA, (pwmA > 0 ? 0 : (pwmA < 0 ? 1 : 0)), 
                    (pwmA > 0 ? 1 : 0), (abs(pwmA) > 0 ? 1 : 0));
        RCLCPP_INFO(this->get_logger(), "Motor B: pwm=%d -> BIN1=%d BIN2=%d PWMB=%d", 
                    pwmB, (pwmB > 0 ? 0 : (pwmB < 0 ? 1 : 0)), 
                    (pwmB > 0 ? 1 : 0), (abs(pwmB) > 0 ? 1 : 0));
    }
    
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscriber_;
    
    // Software PWM
    std::map<int, std::thread> pwm_threads_;
    std::map<int, std::atomic<int>> pwm_duty_cycles_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
