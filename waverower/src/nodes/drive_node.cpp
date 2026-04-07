#include "nodes/drive_node.hpp"

#include "nodes/pca9685.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <sys/select.h>
#include <unistd.h>

namespace nodes {

namespace {

std::string lower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

}  // namespace

DriveNode::DriveNode(const rclcpp::NodeOptions& options)
    : Node("motor_hat_node", options) {
  i2c_bus_ = declare_parameter<int>("i2c_bus", 1);
  i2c_addr_ = declare_parameter<int>("i2c_address", 0x40);
  base_speed_ = declare_parameter<double>("base_speed", 1.0);
  turn_scale_ = declare_parameter<double>("turn_scale", 1.0);
  decay_ms_ = declare_parameter<int>("input_decay_ms", 700);
  declare_parameter<double>("smooth_alpha", 1.0);
  pwm_freq_hz_ = declare_parameter<double>("pwm_freq_hz", 800.0);
  declare_parameter<double>("pwm_boost", 2.35);
  // 1.0 = bez "snap to full"; plynulé využitie celej PWM škály.
  declare_parameter<double>("snap_threshold", 1.0);
  declare_parameter<double>("turn_snap_threshold", 0.0);
  declare_parameter<double>("in_place_turn_pwm_boost", 1.0);
  declare_parameter<double>("smooth_alpha_spin", 1.0);

  declare_parameter<std::string>("control_mode", "manual");
  cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 250);
  wheel_separation_m_ = declare_parameter<double>("wheel_separation_m", 0.20);
  max_wheel_linear_m_s_ = declare_parameter<double>("max_wheel_linear_m_s", 0.35);
  const std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  const std::string teleop_topic = declare_parameter<std::string>("manual_twist_topic", "teleop_cmd_vel");
  declare_parameter<double>("teleop_max_linear_m_s", 0.5);
  declare_parameter<double>("teleop_max_angular_rad_s", 1.8);
  declare_parameter<bool>("teleop_invert_linear", true);
  declare_parameter<bool>("cmd_vel_invert_linear", false);
  declare_parameter<bool>("cmd_vel_invert_angular", false);

  // IMU yaw PID (bez enkoderov)
  declare_parameter<bool>("imu_correction", false);
  declare_parameter<double>("imu_yaw_kp", 0.15);
  declare_parameter<double>("imu_yaw_ki", 0.05);
  declare_parameter<double>("imu_yaw_kd", 0.01);
  declare_parameter<double>("imu_yaw_deadband", 0.02);
  declare_parameter<double>("imu_yaw_integral_limit", 0.3);
  imu_pid_last_time_ = std::chrono::steady_clock::now();

  base_speed_ = std::clamp(base_speed_, 0.2, 1.0);
  turn_scale_ = std::clamp(turn_scale_, 0.1, 2.0);
  pwm_freq_hz_ = std::clamp(pwm_freq_hz_, 50.0, 1000.0);
  wheel_separation_m_ = std::max(wheel_separation_m_, 0.01);
  max_wheel_linear_m_s_ = std::max(max_wheel_linear_m_s_, 0.05);

  control_mode_ = parse_control_mode(get_parameter("control_mode").as_string());

  hat_ = std::make_unique<Pca9685>(i2c_bus_, static_cast<uint8_t>(i2c_addr_));
  hat_->set_pwm_freq_hz(pwm_freq_hz_);

  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic, rclcpp::QoS(10),
      std::bind(&DriveNode::cmd_vel_cb, this, std::placeholders::_1));

  if (!teleop_topic.empty()) {
    sub_teleop_ = create_subscription<geometry_msgs::msg::Twist>(
        teleop_topic, rclcpp::QoS(10),
        std::bind(&DriveNode::teleop_twist_cb, this, std::placeholders::_1));
  }

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&DriveNode::imu_cb, this, std::placeholders::_1));

  param_cb_ = add_on_set_parameters_callback(
      std::bind(&DriveNode::on_param_change, this, std::placeholders::_1));

  pub_imu_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "imu_correction_debug", rclcpp::QoS(5));

  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { timer_cb(); });

  last_cmd_steady_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(),
              "DriveNode  mode=%s  i2c-%d 0x%02x  %.0fHz  cmd_vel=%s  teleop=%s",
              control_mode_ == DriveMode::Auto ? "auto" : "manual", i2c_bus_, i2c_addr_,
              pwm_freq_hz_, cmd_topic.c_str(),
              teleop_topic.empty() ? "(off)" : teleop_topic.c_str());
}

DriveNode::~DriveNode() {
  running_ = false;
  if (input_thread_.joinable()) {
    input_thread_.join();
  }
  if (hat_) {
    hat_->motor_stop(0);
    hat_->motor_stop(1);
  }
  if (tty_ok_) {
    tcsetattr(STDIN_FILENO, TCSANOW, &tty_saved_);
  }
}

DriveMode DriveNode::parse_control_mode(const std::string& s) {
  const std::string k = lower(s);
  if (k == "auto" || k == "autonomous") {
    return DriveMode::Auto;
  }
  return DriveMode::Manual;
}

rcl_interfaces::msg::SetParametersResult DriveNode::on_param_change(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult out;
  out.successful = true;
  for (const rclcpp::Parameter& p : parameters) {
    if (p.get_name() == "control_mode") {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        out.successful = false;
        out.reason = "control_mode musi byt retazec: manual | auto";
        return out;
      }
      const std::string v = lower(p.as_string());
      if (v != "manual" && v != "auto" && v != "autonomous") {
        out.successful = false;
        out.reason = "control_mode: manual alebo auto";
        return out;
      }
      DriveMode m = (v == "manual") ? DriveMode::Manual : DriveMode::Auto;
      control_mode_.store(m, std::memory_order_relaxed);
      reset_motion_state();
      RCLCPP_INFO(get_logger(), "control_mode -> %s", m == DriveMode::Auto ? "auto" : "manual");
    }
  }
  return out;
}

void DriveNode::reset_motion_state() {
  std::lock_guard<std::mutex> lock(mu_);
  fb_ = 0.0;
  tr_ = 0.0;
  last_fb_ = last_tr_ = std::chrono::steady_clock::time_point{};
  twist_linear_x_ = 0.0;
  twist_angular_z_ = 0.0;
  have_cmd_vel_ = false;
  last_cmd_steady_ = std::chrono::steady_clock::now();
  smooth_l_ = 0.0;
  smooth_r_ = 0.0;
}

void DriveNode::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mu_);
  const bool inv_l = get_parameter("cmd_vel_invert_linear").as_bool();
  const bool inv_w = get_parameter("cmd_vel_invert_angular").as_bool();
  twist_linear_x_ = inv_l ? -msg->linear.x : msg->linear.x;
  twist_angular_z_ = inv_w ? -msg->angular.z : msg->angular.z;
  have_cmd_vel_ = true;
  last_cmd_steady_ = std::chrono::steady_clock::now();
}

void DriveNode::teleop_twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (control_mode_.load(std::memory_order_relaxed) != DriveMode::Manual) {
    return;
  }
  double max_lin = std::max(get_parameter("teleop_max_linear_m_s").as_double(), 0.05);
  double max_ang = std::max(get_parameter("teleop_max_angular_rad_s").as_double(), 0.1);
  const double lx =
      get_parameter("teleop_invert_linear").as_bool() ? -msg->linear.x : msg->linear.x;
  const double fb = std::clamp(lx / max_lin, -1.0, 1.0);
  const double tr = std::clamp(msg->angular.z / max_ang, -1.0, 1.0);
  const auto t = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mu_);
  fb_ = fb;
  tr_ = tr;
  last_fb_ = t;
  last_tr_ = t;
}

void DriveNode::prepare_terminal() {
  if (!isatty(STDIN_FILENO)) {
    RCLCPP_WARN(get_logger(), "stdin nie je TTY");
    return;
  }
  if (tcgetattr(STDIN_FILENO, &tty_saved_) != 0) {
    return;
  }
  termios t = tty_saved_;
  t.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &t) != 0) {
    return;
  }
  tty_ok_ = true;
}

void DriveNode::start_input_thread() {
  running_ = true;
  input_thread_ = std::thread([this] { input_loop(); });
}

void DriveNode::touch_fb(double v) {
  std::lock_guard<std::mutex> lock(mu_);
  fb_ = std::clamp(v, -1.0, 1.0);
  last_fb_ = std::chrono::steady_clock::now();
}

void DriveNode::touch_tr(double v) {
  std::lock_guard<std::mutex> lock(mu_);
  tr_ = std::clamp(v, -1.0, 1.0);
  last_tr_ = std::chrono::steady_clock::now();
}

void DriveNode::full_stop_keys() {
  std::lock_guard<std::mutex> lock(mu_);
  fb_ = 0.0;
  tr_ = 0.0;
  last_fb_ = last_tr_ = std::chrono::steady_clock::time_point{};
  smooth_l_ = 0.0;
  smooth_r_ = 0.0;
}

void DriveNode::bump_speed(double delta) {
  double new_base = 0.0;
  {
    std::lock_guard<std::mutex> lock(mu_);
    base_speed_ = std::clamp(base_speed_ + delta, 0.25, 1.0);
    new_base = base_speed_;
  }
  (void)set_parameter(rclcpp::Parameter("base_speed", new_base));
  RCLCPP_INFO(get_logger(), "base_speed = %.2f", new_base);
}

void DriveNode::input_loop() {
  std::array<char, 8> esc_buf{};
  int esc_len = 0;

  while (running_ && rclcpp::ok()) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    if (select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
      continue;
    }

    char c = 0;
    if (read(STDIN_FILENO, &c, 1) != 1) {
      break;
    }

    if (esc_len > 0) {
      if (esc_len < static_cast<int>(esc_buf.size())) {
        esc_buf[esc_len++] = c;
      }
      if (esc_len >= 3 && esc_buf[0] == '\x1b' && esc_buf[1] == '[') {
        switch (esc_buf[2]) {
          case 'A': touch_fb(-1.0); break;
          case 'B': touch_fb(1.0);  break;
          case 'C': touch_tr(1.0);  break;
          case 'D': touch_tr(-1.0); break;
          default: break;
        }
        esc_len = 0;
      } else if (esc_len >= 2 && esc_buf[0] == '\x1b' && esc_buf[1] != '[') {
        esc_len = 0;
      }
      continue;
    }

    if (c == '\x1b') {
      esc_buf[0] = c;
      esc_len = 1;
      continue;
    }

    switch (c) {
      case 'w': case 'W': touch_fb(-1.0); break;
      case 's': case 'S': touch_fb(1.0);  break;
      case 'a': case 'A': touch_tr(1.0);  break;
      case 'd': case 'D': touch_tr(-1.0); break;
      case 'm': case 'M': {
        const bool go_auto = (control_mode_.load(std::memory_order_relaxed) == DriveMode::Manual);
        const std::string next = go_auto ? "auto" : "manual";
        (void)set_parameters({ rclcpp::Parameter("control_mode", next) });
        break;
      }
      case ' ': case 'x': case 'X': full_stop_keys(); break;
      case '+': case '=': bump_speed(0.05);  break;
      case '-': case '_': bump_speed(-0.05); break;
      case 'q': case 'Q':
        RCLCPP_INFO(get_logger(), "Quit (Q)");
        rclcpp::shutdown();
        running_ = false;
        break;
      default: break;
    }
  }
}

void DriveNode::timer_cb() {
  using clock = std::chrono::steady_clock;
  const auto now = clock::now();
  const double boost = std::clamp(get_parameter("pwm_boost").as_double(), 1.0, 2.5);
  const double snap = std::clamp(get_parameter("snap_threshold").as_double(), 0.0, 1.0);
  const double turn_snap_raw = get_parameter("turn_snap_threshold").as_double();
  const double snap_turn = (turn_snap_raw <= 0.0)
                               ? snap
                               : std::clamp(turn_snap_raw, snap, 1.0);
  const double in_place_boost =
      std::clamp(get_parameter("in_place_turn_pwm_boost").as_double(), 1.0, 1.6);
  const double alpha = std::clamp(get_parameter("smooth_alpha").as_double(), 0.05, 1.0);
  const double alpha_spin = std::clamp(get_parameter("smooth_alpha_spin").as_double(), 0.05, 1.0);

  if (control_mode_.load(std::memory_order_relaxed) == DriveMode::Auto) {
    timer_cb_auto(boost, snap, snap_turn, in_place_boost, alpha, alpha_spin);
  } else {
    timer_cb_manual(now, boost, snap, snap_turn, in_place_boost, alpha, alpha_spin);
  }
}

void DriveNode::timer_cb_manual(const std::chrono::steady_clock::time_point& now, double boost,
                                double snap, double snap_turn, double in_place_boost, double alpha,
                                double alpha_spin) {
  double fb = 0.0;
  double tr = 0.0;
  double base = 1.0;
  int decay = 700;
  double turn_sc = 1.0;
  double sl = 0.0;
  double sr = 0.0;

  {
    std::lock_guard<std::mutex> lock(mu_);
    base = base_speed_;
    decay = decay_ms_;
    turn_sc = turn_scale_;
    const auto zt = std::chrono::steady_clock::time_point{};
    if (last_fb_ != zt &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fb_).count() <= decay) {
      fb = fb_;
    }
    if (last_tr_ != zt &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tr_).count() <= decay) {
      tr = tr_;
    }

    const double l_tgt = std::clamp(fb - tr * turn_sc, -1.0, 1.0);
    const double r_tgt = std::clamp(fb + tr * turn_sc, -1.0, 1.0);
    const bool spin_only = std::abs(fb) < 0.12 && std::abs(tr) > 0.05;
    const double a = spin_only ? alpha_spin : alpha;
    smooth_l_ += a * (l_tgt - smooth_l_);
    smooth_r_ += a * (r_tgt - smooth_r_);
    sl = smooth_l_;
    sr = smooth_r_;
  }

  const bool low_forward = std::abs(fb) < 0.12;
  apply_tank(sl, sr, base, boost, snap, snap_turn, low_forward, in_place_boost);
}

void DriveNode::timer_cb_auto(double boost, double snap, double snap_turn, double in_place_boost,
                              double alpha, double alpha_spin) {
  cmd_vel_timeout_ms_ = get_parameter("cmd_vel_timeout_ms").as_int();
  wheel_separation_m_ = std::max(get_parameter("wheel_separation_m").as_double(), 0.01);
  max_wheel_linear_m_s_ = std::max(get_parameter("max_wheel_linear_m_s").as_double(), 0.05);

  double v = 0.0;
  double w = 0.0;
  bool timed_out = false;
  double base = 1.0;
  {
    std::lock_guard<std::mutex> lock(mu_);
    base = base_speed_;
    const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - last_cmd_steady_)
                           .count();
    if (!have_cmd_vel_ || dt_ms > cmd_vel_timeout_ms_) {
      timed_out = true;
      twist_linear_x_ = 0.0;
      twist_angular_z_ = 0.0;
    }
    v = twist_linear_x_;
    w = twist_angular_z_;
  }

  const double half_track = 0.5 * wheel_separation_m_;
  const double denom = max_wheel_linear_m_s_;
  double v_l = (v - w * half_track) / denom;
  double v_r = (v + w * half_track) / denom;
  v_l = std::clamp(v_l, -1.0, 1.0);
  v_r = std::clamp(v_r, -1.0, 1.0);

  double sl = 0.0;
  double sr = 0.0;
  {
    std::lock_guard<std::mutex> lock(mu_);
    const bool spin_only = std::abs(v) < 1e-6 && std::abs(w) > 1e-6;
    const double a = (spin_only && !timed_out) ? alpha_spin : alpha;
    smooth_l_ += a * (v_l - smooth_l_);
    smooth_r_ += a * (v_r - smooth_r_);
    sl = smooth_l_;
    sr = smooth_r_;
  }

  const bool low_forward = std::abs(v) < 0.02;
  apply_tank(sl, sr, base, boost, snap, snap_turn, low_forward, in_place_boost);
}

void DriveNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mu_);
  constexpr double kAlpha = 0.25;
  imu_yaw_rate_ += kAlpha * (msg->angular_velocity.z - imu_yaw_rate_);
}

void DriveNode::apply_tank(double l_cmd, double r_cmd, double base, double boost,
                           double snap_fwd, double snap_turn, bool low_forward,
                           double in_place_boost) {
  double corr = 0.0;
  double dbg_yaw_rate = 0.0, dbg_error = 0.0;
  double dbg_p = 0.0, dbg_i = 0.0, dbg_d = 0.0;
  const bool correction_on = get_parameter("imu_correction").as_bool();

  if (correction_on && !low_forward) {
    const double kp           = get_parameter("imu_yaw_kp").as_double();
    const double ki           = get_parameter("imu_yaw_ki").as_double();
    const double kd           = get_parameter("imu_yaw_kd").as_double();
    const double deadband     = get_parameter("imu_yaw_deadband").as_double();
    const double windup_limit = get_parameter("imu_yaw_integral_limit").as_double();

    {
      std::lock_guard<std::mutex> lock(mu_);
      dbg_yaw_rate = imu_yaw_rate_;
    }

    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - imu_pid_last_time_).count();
    imu_pid_last_time_ = now;

    if (std::abs(dbg_yaw_rate) <= deadband) {
      dbg_error = 0.0;
    } else {
      dbg_error = dbg_yaw_rate > 0.0 ? dbg_yaw_rate - deadband : dbg_yaw_rate + deadband;
    }

    if (dt > 0.001 && dt < 0.5) {
      imu_yaw_integral_ += dbg_error * dt;
      imu_yaw_integral_ = std::clamp(imu_yaw_integral_, -windup_limit, windup_limit);
      const double d_error = (dbg_error - imu_yaw_prev_error_) / dt;
      dbg_p = kp * dbg_error;
      dbg_i = ki * imu_yaw_integral_;
      dbg_d = kd * d_error;
      corr  = dbg_p + dbg_i + dbg_d;
    }
    imu_yaw_prev_error_ = dbg_error;
  } else {
    imu_yaw_integral_   = 0.0;
    imu_yaw_prev_error_ = 0.0;
    imu_pid_last_time_  = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mu_);
    dbg_yaw_rate = imu_yaw_rate_;
  }

  const double snap_eff = low_forward ? snap_turn : snap_fwd;
  const double extra = low_forward ? in_place_boost : 1.0;
  auto to_pct = [&](double cmd) -> int {
    double m = std::abs(cmd);
    if (snap_eff > 0.0 && m >= snap_eff) {
      m = 1.0;
    }
    const double raw = m * base * 100.0 * boost * extra;
    return std::min(100, static_cast<int>(std::lround(raw)));
  };

  int pl = to_pct(l_cmd);
  int pr = to_pct(r_cmd);
  int corr_pct = 0;
  if (corr != 0.0) {
    corr_pct = static_cast<int>(std::lround(corr * boost * 100.0));
    pl = std::clamp(pl - corr_pct, 0, 100);
    pr = std::clamp(pr + corr_pct, 0, 100);
  }

  // Publish debug info (~100 Hz, rosbridge throttles na strane klienta)
  // Layout: [active, low_fwd, yaw_rate, error, p, i, d, total, corr_pct, pwm_l, pwm_r]
  std_msgs::msg::Float64MultiArray dbg_msg;
  dbg_msg.data = {
    correction_on && !low_forward ? 1.0 : 0.0,
    low_forward ? 1.0 : 0.0,
    dbg_yaw_rate,
    dbg_error,
    dbg_p,
    dbg_i,
    dbg_d,
    corr,
    static_cast<double>(corr_pct),
    static_cast<double>(pl),
    static_cast<double>(pr),
  };
  pub_imu_debug_->publish(dbg_msg);

  if (pl <= 0 && pr <= 0) {
    hat_->motor_stop(0);
    hat_->motor_stop(1);
    return;
  }
  hat_->apply_drive(pl, l_cmd > 0.0, pr, r_cmd > 0.0);
}

}  // namespace nodes
