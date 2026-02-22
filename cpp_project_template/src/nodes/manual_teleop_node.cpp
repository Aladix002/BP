#include "nodes/manual_teleop_node.hpp"
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <chrono>

namespace {

enum class KeyDir { None, Up, Down, Left, Right, Stop, SpeedUp, SpeedDown, SpeedDigit,
    LeftTrimUp, LeftTrimDown, RightTrimUp, RightTrimDown, ToggleMode };
int g_speed_digit = 5;

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
    if (n >= 3 && buf[0] == '\x1b' && (buf[1] == '[' || buf[1] == 'O')) {
        switch (buf[2]) {
            case 'A': return KeyDir::Up;
            case 'B': return KeyDir::Down;
            case 'C': return KeyDir::Right;
            case 'D': return KeyDir::Left;
        }
    }
    if (n == 1) {
        if (buf[0] == 'q' || buf[0] == 'Q' || buf[0] == ' ' || buf[0] == 3) return KeyDir::Stop;
        if (buf[0] == '+' || buf[0] == '=') return KeyDir::SpeedUp;
        if (buf[0] == '-') return KeyDir::SpeedDown;
        if (buf[0] >= '1' && buf[0] <= '9') { g_speed_digit = buf[0] - '0'; return KeyDir::SpeedDigit; }
        if (buf[0] == 'w' || buf[0] == 'W') return KeyDir::Up;
        if (buf[0] == 's' || buf[0] == 'S') return KeyDir::Down;
        if (buf[0] == 'a' || buf[0] == 'A') return KeyDir::Left;
        if (buf[0] == 'd' || buf[0] == 'D') return KeyDir::Right;
        if (buf[0] == 'u' || buf[0] == 'U') return KeyDir::LeftTrimDown;
        if (buf[0] == 'i' || buf[0] == 'I') return KeyDir::LeftTrimUp;
        if (buf[0] == 'o' || buf[0] == 'O') return KeyDir::RightTrimDown;
        if (buf[0] == 'p' || buf[0] == 'P') return KeyDir::RightTrimUp;
        if (buf[0] == 'm' || buf[0] == 'M') return KeyDir::ToggleMode;
    }
    return KeyDir::None;
}

}  // namespace

ManualTeleopNode::ManualTeleopNode(std::shared_ptr<std::atomic<bool>> manual_mode)
    : Node("manual_teleop"), manual_mode_(std::move(manual_mode)) {
    this->declare_parameter<double>("speed", 0.5);
    speed_ = std::max(0.1, std::min(1.0, this->get_parameter("speed").as_double()));
    this->declare_parameter<double>("left_trim", 1.0);
    left_trim_ = std::max(0.5, std::min(1.5, this->get_parameter("left_trim").as_double()));
    this->declare_parameter<double>("right_trim", 1.0);
    right_trim_ = std::max(0.5, std::min(1.5, this->get_parameter("right_trim").as_double()));
    this->declare_parameter<int>("key_release_timeout_ms", 550);
    key_release_timeout_ms_ = static_cast<int>(this->get_parameter("key_release_timeout_ms").as_int());
    this->declare_parameter<int>("control_period_ms", 10);
    int period_ms = std::max(5, std::min(100, static_cast<int>(this->get_parameter("control_period_ms").as_int())));

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/manual_motor_cmd", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&ManualTeleopNode::timer_cb, this));

    if (setup_raw_stdin() != 0)
        RCLCPP_WARN(this->get_logger(), "Spustite v termináli (nie cez IDE).");

    RCLCPP_INFO(this->get_logger(), "Manual teleop: WASD/šípky -> /manual_motor_cmd | u/i o/p = trim | m = prepínať režim (v main)");
}

void ManualTeleopNode::timer_cb() {
    auto now = std::chrono::steady_clock::now();
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
            RCLCPP_INFO(this->get_logger(), "Rýchlosť: %.2f", speed_);
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
        } else if (k == KeyDir::ToggleMode && manual_mode_) {
            bool new_val = !manual_mode_->load();
            manual_mode_->store(new_val);
            RCLCPP_INFO(this->get_logger(), "Režim: %s", new_val ? "MANUÁL" : "AUTO (lidar)");
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

    int vertical = 0;
    if (up_ok && down_ok) { if (last_up_ < last_down_) vertical = 1; else vertical = -1; }
    else if (up_ok) vertical = 1;
    else if (down_ok) vertical = -1;
    int horizontal = 0;
    if (left_ok && right_ok) { if (last_left_ < last_right_) horizontal = 1; else horizontal = -1; }
    else if (left_ok) horizontal = 1;
    else if (right_ok) horizontal = -1;

    const double base = 0.5 * speed_;
    const double curve = 0.3 * speed_;
    const double turn_in_place = 0.4;
    double L = 0.0, R = 0.0;
    bool fwd = (vertical == 1), bwd = (vertical == -1), left = (horizontal == 1), right = (horizontal == -1);

    if (!up_ok && !down_ok && !left_ok && !right_ok) {
        L = 0; R = 0;
    } else if (fwd && !bwd && !left && !right) {
        L = base; R = base;
    } else if (bwd && !left && !right) {
        L = -base; R = -base;
    } else if (!fwd && !bwd && left && !right) {
        L = -turn_in_place; R = turn_in_place;
    } else if (!fwd && !bwd && !left && right) {
        L = turn_in_place; R = -turn_in_place;
    } else if (fwd && !bwd && left && !right) {
        L = curve; R = base;
    } else if (fwd && !bwd && !left && right) {
        L = base; R = curve;
    } else if (bwd && left && !right) {
        L = -curve; R = -base;
    } else if (bwd && !left && right) {
        L = -base; R = -curve;
    } else {
        if (fwd && left) { L = curve; R = base; }
        else if (fwd && right) { L = base; R = curve; }
        else if (bwd && left) { L = -curve; R = -base; }
        else if (bwd && right) { L = -base; R = -curve; }
        else if (left) { L = -turn_in_place; R = turn_in_place; }
        else if (right) { L = turn_in_place; R = -turn_in_place; }
        else { L = 0; R = 0; }
    }

    L *= left_trim_;
    R *= right_trim_;
    L = std::max(-1.0, std::min(1.0, L));
    R = std::max(-1.0, std::min(1.0, R));

    std_msgs::msg::Float64MultiArray msg;
    msg.data = { L, R };
    pub_->publish(msg);
}
