#include <rclcpp/rclcpp.hpp>
#include <curl/curl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <chrono>
#include <sys/select.h>
#include <sstream>
#include <iomanip>

// Priame ovládanie motorov cez HTTP na ESP32.
// Rýchlosť oboch strán cez 1-9; W+A / W+D = jedna strana pomalšie (zábačka); kontrola častá (10 ms).
// Výstup: T:1 float -1..1 alebo T:11 PWM 0-255 (128=stop, 128+ dopredu, 127- dozadu).

namespace {

enum class KeyDir { None, Up, Down, Left, Right, Stop, SpeedUp, SpeedDown, SpeedDigit,
    LeftTrimUp, LeftTrimDown, RightTrimUp, RightTrimDown };
int g_speed_digit = 5;  // 1-9, used when KeyDir::SpeedDigit

int setup_raw_stdin() {
    int fd = STDIN_FILENO;
    if (!isatty(fd)) return -1;
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;
    tty.c_lflag &= ~(ICANON | ECHO);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
    return 0;
}

KeyDir read_key_nonblock() {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    struct timeval tv = { 0, 0 };
    if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0 || !FD_ISSET(STDIN_FILENO, &fds))
        return KeyDir::None;

    char buf[8];
    int n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) return KeyDir::None;

    // Šípky: štandard (vľavo = vľavo, vpravo = vpravo)
    if (n >= 3 && buf[0] == '\x1b' && (buf[1] == '[' || buf[1] == 'O')) {
        switch (buf[2]) {
            case 'A': return KeyDir::Up;
            case 'B': return KeyDir::Down;
            case 'C': return KeyDir::Right;  // šípka vpravo
            case 'D': return KeyDir::Left;   // šípka vľavo
        }
    }
    if (n == 1) {
        if (buf[0] == 'q' || buf[0] == 'Q' || buf[0] == ' ' || buf[0] == 3) return KeyDir::Stop;
        if (buf[0] == '+' || buf[0] == '=') return KeyDir::SpeedUp;
        if (buf[0] == '-') return KeyDir::SpeedDown;
        if (buf[0] >= '1' && buf[0] <= '9') { g_speed_digit = buf[0] - '0'; return KeyDir::SpeedDigit; }
        // WASD: štandard A=vľavo, D=vpravo
        if (buf[0] == 'w' || buf[0] == 'W') return KeyDir::Up;
        if (buf[0] == 's' || buf[0] == 'S') return KeyDir::Down;
        if (buf[0] == 'a' || buf[0] == 'A') return KeyDir::Left;
        if (buf[0] == 'd' || buf[0] == 'D') return KeyDir::Right;
        // Trim ľavá/pravá strana: u/i = ľavý motor -/+, o/p = pravý motor -/+
        if (buf[0] == 'u' || buf[0] == 'U') return KeyDir::LeftTrimDown;
        if (buf[0] == 'i' || buf[0] == 'I') return KeyDir::LeftTrimUp;
        if (buf[0] == 'o' || buf[0] == 'O') return KeyDir::RightTrimDown;
        if (buf[0] == 'p' || buf[0] == 'P') return KeyDir::RightTrimUp;
    }
    return KeyDir::None;
}

} // namespace

class MotorHttpTeleopNode : public rclcpp::Node {
public:
    MotorHttpTeleopNode() : Node("motor_http_teleop") {
        this->declare_parameter<std::string>("esp32_ip", "192.168.0.224");
        esp32_ip_ = this->get_parameter("esp32_ip").as_string();
        esp32_url_ = "http://" + esp32_ip_ + "/js";

        this->declare_parameter<double>("speed", 0.5);
        speed_ = std::max(0.1, std::min(1.0, this->get_parameter("speed").as_double()));

        this->declare_parameter<double>("left_trim", 1.0);
        left_trim_ = std::max(0.5, std::min(1.5, this->get_parameter("left_trim").as_double()));
        this->declare_parameter<double>("right_trim", 1.0);
        right_trim_ = std::max(0.5, std::min(1.5, this->get_parameter("right_trim").as_double()));

        this->declare_parameter<bool>("use_pwm_255", false);
        use_pwm_255_ = this->get_parameter("use_pwm_255").as_bool();

        // Timeout pustenia: väčší než oneskorenie key repeat (~500 ms), aby pri držaní nepreseklo
        this->declare_parameter<int>("key_release_timeout_ms", 550);
        key_release_timeout_ms_ = static_cast<int>(this->get_parameter("key_release_timeout_ms").as_int());

        // Veľmi častá kontrola (100 Hz): W+A / W+D musia byť čo najčastejšie vyhodnotené
        this->declare_parameter<int>("control_period_ms", 10);
        int period_ms = std::max(5, std::min(100, static_cast<int>(this->get_parameter("control_period_ms").as_int())));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&MotorHttpTeleopNode::timer_cb, this));

        if (setup_raw_stdin() != 0) {
            RCLCPP_WARN(this->get_logger(), "Spustite v termináli (nie cez IDE).");
        }

        curl_global_init(CURL_GLOBAL_DEFAULT);

        RCLCPP_INFO(this->get_logger(), "Motory HTTP teleop: %s", esp32_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "WASD/šípky = smer | u/i = ľavý motor -/+ | o/p = pravý motor -/+ | +/- 1-9 = rýchlosť");
        RCLCPP_INFO(this->get_logger(), "Trim L=%.2f R=%.2f | výstup: %s", left_trim_, right_trim_, use_pwm_255_ ? "PWM 0-255 (T:11)" : "float -1..1 (T:1)");
    }

    ~MotorHttpTeleopNode() {
        send_motors(0.0, 0.0);
        curl_global_cleanup();
    }

private:
    void timer_cb() {
        auto now = std::chrono::steady_clock::now();
        // Spracuj všetky čakajúce klávesy (W+D naraz = obe stlačené, zábačka hneď)
        for (;;) {
            KeyDir k = read_key_nonblock();
            if (k == KeyDir::None) break;

            if (k == KeyDir::Stop) {
                last_up_ = last_down_ = last_left_ = last_right_ = std::chrono::steady_clock::time_point{};
            } else if (k == KeyDir::SpeedUp) {
                speed_ = std::min(1.0, speed_ + 0.1);
                RCLCPP_INFO(this->get_logger(), "Rýchlosť: %.2f", speed_);
            } else if (k == KeyDir::SpeedDown) {
                speed_ = std::max(0.1, speed_ - 0.1);
                RCLCPP_INFO(this->get_logger(), "Rýchlosť: %.2f", speed_);
            } else if (k == KeyDir::SpeedDigit) {
                speed_ = g_speed_digit / 9.0;
                RCLCPP_INFO(this->get_logger(), "Rýchlosť: %.2f (úroveň %d)", speed_, g_speed_digit);
            } else if (k == KeyDir::LeftTrimUp) {
                left_trim_ = std::min(1.5, left_trim_ + 0.1);
                RCLCPP_INFO(this->get_logger(), "Trim ľavý: %.2f", left_trim_);
            } else if (k == KeyDir::LeftTrimDown) {
                left_trim_ = std::max(0.5, left_trim_ - 0.1);
                RCLCPP_INFO(this->get_logger(), "Trim ľavý: %.2f", left_trim_);
            } else if (k == KeyDir::RightTrimUp) {
                right_trim_ = std::min(1.5, right_trim_ + 0.1);
                RCLCPP_INFO(this->get_logger(), "Trim pravý: %.2f", right_trim_);
            } else if (k == KeyDir::RightTrimDown) {
                right_trim_ = std::max(0.5, right_trim_ - 0.1);
                RCLCPP_INFO(this->get_logger(), "Trim pravý: %.2f", right_trim_);
            } else {
                if (k == KeyDir::Up) last_up_ = now;
                if (k == KeyDir::Down) last_down_ = now;
                if (k == KeyDir::Left) last_left_ = now;
                if (k == KeyDir::Right) last_right_ = now;
            }
        }

        auto ms = [&now](const std::chrono::steady_clock::time_point& t) {
            if (t == std::chrono::steady_clock::time_point{}) return 999999;
            return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count());
        };

        bool up_ok = ms(last_up_) < key_release_timeout_ms_;
        bool down_ok = ms(last_down_) < key_release_timeout_ms_;
        bool left_ok = ms(last_left_) < key_release_timeout_ms_;
        bool right_ok = ms(last_right_) < key_release_timeout_ms_;

        // Konflikt dopredu/dozadu: prvý stlačený vyhrá (starší čas = skorší stlačený)
        int vertical = 0;  // -1 dozadu, 0 žiadny, 1 dopredu
        if (up_ok && down_ok) {
            if (last_up_ < last_down_) vertical = 1; else vertical = -1;
        } else if (up_ok) vertical = 1;
        else if (down_ok) vertical = -1;

        // horizontal: 1 = left (L slower pri zábačke), -1 = right (R slower) – ako ugv A/D
        int horizontal = 0;
        if (left_ok && right_ok) {
            if (last_left_ < last_right_) horizontal = 1; else horizontal = -1;
        } else if (left_ok)  horizontal = 1;
        else if (right_ok) horizontal = -1;

        // Zábačky: base 0.5, curve 0.3 (ako ugv). Otáčanie na mieste: miernejšie 0.7 ako predtým.
        const double base = 0.5 * speed_;
        const double curve = 0.3 * speed_;
        const double turn_in_place = 0.4;  // konštantná rýchlosť otáčania na mieste (~2× pomalšie)
        double L = 0.0, R = 0.0;

        bool fwd = (vertical == 1);
        bool bwd = (vertical == -1);
        bool left = (horizontal == 1);
        bool right = (horizontal == -1);

        if (!up_ok && !down_ok && !left_ok && !right_ok) {
            L = 0; R = 0;
        } else if (fwd && !bwd && !left && !right) {
            L = base; R = base;
        } else if (bwd && !left && !right) {
            L = -base; R = -base;
        } else if (!fwd && !bwd && left && !right) {
            L = -turn_in_place; R = turn_in_place;   // otáčanie vľavo na mieste
        } else if (!fwd && !bwd && !left && right) {
            L = turn_in_place; R = -turn_in_place;   // otáčanie vpravo na mieste
        } else if (fwd && !bwd && left && !right) {
            L = curve; R = base;   // forward+left
        } else if (fwd && !bwd && !left && right) {
            L = base; R = curve;   // forward+right
        } else if (bwd && left && !right) {
            L = -curve; R = -base; // back+left
        } else if (bwd && !left && right) {
            L = -base; R = -curve; // back+right
        } else {
            // Konflikt alebo obe horizontálne: preferencia ako v ugv (prvý stlačený)
            if (fwd && left) { L = curve; R = base; }
            else if (fwd && right) { L = base; R = curve; }
            else if (bwd && left) { L = -curve; R = -base; }
            else if (bwd && right) { L = -base; R = -curve; }
            else if (left) { L = -turn_in_place; R = turn_in_place; }
            else if (right) { L = turn_in_place; R = -turn_in_place; }
            else { L = 0; R = 0; }
        }

        // Nezávislé nastavenie ľavej/pravej strany (trim)
        L *= left_trim_;
        R *= right_trim_;
        L = std::max(-1.0, std::min(1.0, L));
        R = std::max(-1.0, std::min(1.0, R));

        send_motors(L, R);
    }

    void send_motors(double left, double right) {
        std::ostringstream json;
        if (use_pwm_255_) {
            // 128 = stop, 128+ = dopredu, 127- = dozadu (0-255)
            int L255 = static_cast<int>(128.0 + left * 127.0 + 0.5);
            int R255 = static_cast<int>(128.0 + right * 127.0 + 0.5);
            L255 = std::max(0, std::min(255, L255));
            R255 = std::max(0, std::min(255, R255));
            json << "{\"T\":11,\"L\":" << L255 << ",\"R\":" << R255 << "}";
        } else {
            json << std::fixed << std::setprecision(2);
            json << "{\"T\":1,\"L\":" << left << ",\"R\":" << right << "}";
        }
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

    std::string esp32_ip_;
    std::string esp32_url_;
    double speed_;
    double left_trim_;
    double right_trim_;
    bool use_pwm_255_;
    int key_release_timeout_ms_;
    std::chrono::steady_clock::time_point last_up_{};
    std::chrono::steady_clock::time_point last_down_{};
    std::chrono::steady_clock::time_point last_left_{};
    std::chrono::steady_clock::time_point last_right_{};
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorHttpTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
