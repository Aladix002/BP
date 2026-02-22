#include <rclcpp/rclcpp.hpp>
#include <curl/curl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <chrono>
#include <sys/select.h>
#include <sstream>
#include <iomanip>

// Priame ovládanie motorov cez HTTP na ESP32 (bez esp32_bridge).
// Držanie = jazda; dopredu+doprava = zábačka; dopredu+dozadu = prvý stlačený vyhrá.

namespace {

enum class KeyDir { None, Up, Down, Left, Right, Stop, SpeedUp, SpeedDown, SpeedDigit };
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

    // Šípky: vpravo/vľavo prehodené (C=vpravo, D=vľavo na klávesnici)
    if (n >= 3 && buf[0] == '\x1b' && (buf[1] == '[' || buf[1] == 'O')) {
        switch (buf[2]) {
            case 'A': return KeyDir::Up;
            case 'B': return KeyDir::Down;
            case 'C': return KeyDir::Left;   // šípka vpravo → Left (prehodené)
            case 'D': return KeyDir::Right;  // šípka vľavo → Right
        }
    }
    if (n == 1) {
        if (buf[0] == 'q' || buf[0] == 'Q' || buf[0] == ' ' || buf[0] == 3) return KeyDir::Stop;
        if (buf[0] == '+' || buf[0] == '=') return KeyDir::SpeedUp;
        if (buf[0] == '-') return KeyDir::SpeedDown;
        if (buf[0] >= '1' && buf[0] <= '9') { g_speed_digit = buf[0] - '0'; return KeyDir::SpeedDigit; }
        // WASD: A a D prehodené
        if (buf[0] == 'w' || buf[0] == 'W') return KeyDir::Up;
        if (buf[0] == 's' || buf[0] == 'S') return KeyDir::Down;
        if (buf[0] == 'a' || buf[0] == 'A') return KeyDir::Right;  // A → vpravo
        if (buf[0] == 'd' || buf[0] == 'D') return KeyDir::Left;    // D → vľavo
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

        // Dlhý timeout: pri držaní často nepríde opakovanie klávesov (SSH/terminál), takže
        // berieme „stlačené“ až do tohto času; zastavenie = medzerník alebo q
        this->declare_parameter<int>("key_release_timeout_ms", 10000);
        key_release_timeout_ms_ = static_cast<int>(this->get_parameter("key_release_timeout_ms").as_int());

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorHttpTeleopNode::timer_cb, this));

        if (setup_raw_stdin() != 0) {
            RCLCPP_WARN(this->get_logger(), "Spustite v termináli (nie cez IDE).");
        }

        curl_global_init(CURL_GLOBAL_DEFAULT);

        RCLCPP_INFO(this->get_logger(), "Motory HTTP teleop: %s", esp32_url_.c_str());
        RCLCPP_INFO(this->get_logger(), "Drž = jazda (10 s alebo medzerník/q = stop) | 2 šípky = zábačka | +/- 1-9 = rýchlosť");
    }

    ~MotorHttpTeleopNode() {
        send_motors(0.0, 0.0);
        curl_global_cleanup();
    }

private:
    void timer_cb() {
        auto now = std::chrono::steady_clock::now();
        KeyDir k = read_key_nonblock();

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
        } else {
            if (k == KeyDir::Up) last_up_ = now;
            if (k == KeyDir::Down) last_down_ = now;
            if (k == KeyDir::Left) last_left_ = now;
            if (k == KeyDir::Right) last_right_ = now;
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

        // horizontal: 1 = šípka vpravo, -1 = šípka vľavo (prehodené ak na robote L/R sedí opačne)
        int horizontal = 0;
        if (left_ok && right_ok) {
            if (last_left_ < last_right_) horizontal = 1; else horizontal = -1;
        } else if (left_ok)  horizontal = 1;
        else if (right_ok) horizontal = -1;

        // Menšie točenie: len 25 % rozdiel (0.25), nie 50 % – plynulejšia zábačka
        const double turn_reduce = 0.25;
        double L = vertical * speed_;
        double R = vertical * speed_;
        if (vertical != 0 && horizontal != 0) {
            if (horizontal > 0) L *= (1.0 - turn_reduce);
            else               R *= (1.0 - turn_reduce);
        } else if (vertical == 0 && horizontal != 0) {
            // Otáčanie na mieste tiež miernejšie: 70 % rýchlosti
            const double turn_in_place = 0.7;
            L =  horizontal * speed_ * turn_in_place;
            R = -horizontal * speed_ * turn_in_place;
        }
        L = std::max(-speed_, std::min(speed_, L));
        R = std::max(-speed_, std::min(speed_, R));

        send_motors(L, R);
    }

    void send_motors(double left, double right) {
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

    std::string esp32_ip_;
    std::string esp32_url_;
    double speed_;
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
