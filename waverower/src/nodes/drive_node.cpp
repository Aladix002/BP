#include "nodes/drive_node.hpp"
#include "nodes/pca9685.hpp"

#include <algorithm>
#include <cmath>

namespace nodes {

// ---------------------------------------------------------------------------
// Konštruktor – deklaruje všetky parametre s rozumnými predvolenými hodnotami
// ---------------------------------------------------------------------------
DriveNode::DriveNode(const rclcpp::NodeOptions& options)
    : Node("motor_hat_node", options)
{
  // Hardware
  const int    bus  = declare_parameter<int>("i2c_bus", 1);
  const int    addr = declare_parameter<int>("i2c_address", 0x40);
  const double freq = declare_parameter<double>("pwm_freq_hz", 800.0);

  // PWM rozsah
  // pwm_min: minimálne duty kedy sa motor začne točiť (anti-stall)
  // pwm_max: maximálne duty (plná rýchlosť)
  declare_parameter<int>("pwm_min", 400);
  declare_parameter<int>("pwm_max", 4095);

  // Mŕtva zóna vstupu – pod touto hodnotou (zlomok 0–1) motor stojí
  declare_parameter<double>("deadzone", 0.05);

  // Vyhladzovanie – 1.0 = okamžitá odozva, 0.1 = pomalý nábeh (~150 ms)
  declare_parameter<double>("smooth_alpha", 0.15);

  // Režim: "manual" počúva /teleop_cmd_vel, "auto" počúva /cmd_vel
  declare_parameter<std::string>("control_mode", "manual");

  // Manuálny režim – normalizácia teleopu
  // Nastav teleop_max_linear na maximálnu rýchlosť ktorú teleop_twist_keyboard posiela
  // (napr. 1.0 ak chceš škálu 0–1.0 m/s = plný rozsah PWM)
  declare_parameter<double>("teleop_max_linear",  1.0);
  declare_parameter<double>("teleop_max_angular", 2.0);
  declare_parameter<bool>("invert_linear",  false);
  declare_parameter<bool>("invert_angular", false);

  // Automatický režim – diferenciálna kinematika
  declare_parameter<int>("cmd_vel_timeout_ms", 300);
  declare_parameter<double>("max_wheel_speed", 0.4);  // fyzická max. rýchlosť kolesa [m/s]
  declare_parameter<double>("wheel_base",      0.20); // rozchod kolies [m]
  // Wander / ball_follow / Nav2: ak +linear.x ide fyzicky vzad, zapni true
  declare_parameter<bool>("cmd_vel_invert_linear", false);

  // IMU korekcia priamej jazdy (PID na yaw rate)
  declare_parameter<bool>("imu_correction", false);
  declare_parameter<double>("imu_kp",       0.30);
  declare_parameter<double>("imu_ki",       0.05);
  declare_parameter<double>("imu_kd",       0.01);
  declare_parameter<double>("imu_deadband", 0.02);  // [rad/s] pod ktorou sa neopravuje
  declare_parameter<double>("imu_windup",   0.30);  // limit integrátora
  declare_parameter<double>("imu_sign",    -1.0);   // smer korekcie: +1 alebo -1
  // ωz z /imu: EMA + podlahová mŕtva zóna (malé vykyvy pri státí → 0 pre PID aj drive_debug)
  declare_parameter<double>("imu_yaw_lowpass_alpha", 0.18);
  declare_parameter<double>("imu_yaw_noise_floor_rad_s", 0.025);

  hat_ = std::make_unique<Pca9685>(bus, static_cast<uint8_t>(addr));
  hat_->set_pwm_freq_hz(freq);

  sub_teleop_ = create_subscription<geometry_msgs::msg::Twist>(
      "teleop_cmd_vel", rclcpp::QoS(10),
      std::bind(&DriveNode::teleop_cb, this, std::placeholders::_1));

  sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&DriveNode::cmd_vel_cb, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&DriveNode::imu_cb, this, std::placeholders::_1));

  // Debug topic: [pwm_l%, pwm_r%, left_cmd, right_cmd, imu_corr]
  pub_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>("drive_debug", rclcpp::QoS(5));

  imu_pid_time_ = std::chrono::steady_clock::now();
  last_cmd_     = std::chrono::steady_clock::now();

  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { timer_cb(); });

  RCLCPP_INFO(get_logger(),
              "DriveNode ready  i2c-%d 0x%02x  %.0fHz  mode=%s",
              bus, addr, freq,
              get_parameter("control_mode").as_string().c_str());
}

DriveNode::~DriveNode() {
  if (hat_) {
    hat_->motor_stop(0);
    hat_->motor_stop(1);
  }
}

// ---------------------------------------------------------------------------
// to_duty – srdce ovládania
//
// cmd ∈ [-1, 1] (normalizovaný príkaz)
// Výstup: duty ∈ [0, 4095]
//
// Mapovanie:
//   |cmd| ≤ deadzone          → 0      (motor stojí)
//   |cmd| = deadzone + epsilon → pwm_min (motor sa práve pohne)
//   |cmd| = 1.0               → pwm_max (plná rýchlosť)
//   Medzi tým: lineárne
// ---------------------------------------------------------------------------
uint16_t DriveNode::to_duty(double cmd) const {
  const double m    = std::abs(cmd);
  const double dz   = std::clamp(get_parameter("deadzone").as_double(), 0.0, 0.49);
  const int    pmin = std::clamp(static_cast<int>(get_parameter("pwm_min").as_int()),  0, 4095);
  const int    pmax = std::clamp(static_cast<int>(get_parameter("pwm_max").as_int()),  pmin, 4095);

  if (m <= dz) return 0;

  // Normalizácia: dz..1.0  →  0..1
  const double t = (m - dz) / (1.0 - dz);

  const double duty = static_cast<double>(pmin) + t * static_cast<double>(pmax - pmin);
  return static_cast<uint16_t>(std::clamp(static_cast<int>(std::lround(duty)), 0, 4095));
}

// ---------------------------------------------------------------------------
// teleop_cb – manuálny režim
//
// Normalizuje Twist podľa teleop_max_linear / teleop_max_angular.
// Tank mixing: left = fb - tr,  right = fb + tr
// ---------------------------------------------------------------------------
void DriveNode::teleop_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (get_parameter("control_mode").as_string() != "manual") return;

  const double max_lin = std::max(get_parameter("teleop_max_linear").as_double(),  0.01);
  const double max_ang = std::max(get_parameter("teleop_max_angular").as_double(), 0.01);

  const double lx = get_parameter("invert_linear").as_bool()  ? -msg->linear.x  : msg->linear.x;
  const double az = get_parameter("invert_angular").as_bool() ? -msg->angular.z : msg->angular.z;

  const double fb = std::clamp(lx / max_lin, -1.0, 1.0);
  const double tr = std::clamp(az / max_ang, -1.0, 1.0);

  // Tank mix pre tento podvozok: oproti štandardnému (L=fb−tr, R=fb+tr) sú oba smery prehodené.
  std::lock_guard<std::mutex> lk(mu_);
  target_l_  = std::clamp(-fb + tr, -1.0, 1.0);
  target_r_  = std::clamp(-fb - tr, -1.0, 1.0);
  last_cmd_  = std::chrono::steady_clock::now();
  have_cmd_  = true;
}

// ---------------------------------------------------------------------------
// cmd_vel_cb – automatický režim (nav2, ball_follower, wander)
//
// Diferenciálna kinematika (unicycle → tank), výsledok v [-1, 1].
// Delíme teleop_max_linear (nie max_wheel_speed): Twist.linear.x je v m/s rovnako
// ako pri teleope; pri max_wheel_speed ~0.4 a v~0.5 m/s sa inak nasýtilo na plný výkon.
// ---------------------------------------------------------------------------
void DriveNode::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (get_parameter("control_mode").as_string() == "manual") return;

  const double max_v  = std::max(get_parameter("teleop_max_linear").as_double(), 0.01);
  const double half_b = 0.5 * std::max(get_parameter("wheel_base").as_double(),  0.01);

  double v = msg->linear.x;
  if (get_parameter("cmd_vel_invert_linear").as_bool()) {
    v = -v;
  }
  const double w = msg->angular.z;
  const double v_l = std::clamp((v - w * half_b) / max_v, -1.0, 1.0);
  const double v_r = std::clamp((v + w * half_b) / max_v, -1.0, 1.0);

  std::lock_guard<std::mutex> lk(mu_);
  target_l_  = v_l;
  target_r_  = v_r;
  last_cmd_  = std::chrono::steady_clock::now();
  have_cmd_  = true;
}

// ---------------------------------------------------------------------------
// imu_cb – nízkoúrovňový filter yaw rate (EMA)
// ---------------------------------------------------------------------------
void DriveNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double a = std::clamp(get_parameter("imu_yaw_lowpass_alpha").as_double(), 0.02, 1.0);
  const double floor_rad =
      std::max(0.0, get_parameter("imu_yaw_noise_floor_rad_s").as_double());
  double z = msg->angular_velocity.z;
  std::lock_guard<std::mutex> lk(mu_);
  imu_yaw_rate_ += a * (z - imu_yaw_rate_);
  if (std::abs(imu_yaw_rate_) < floor_rad) {
    imu_yaw_rate_ = 0.0;
  }
}

// ---------------------------------------------------------------------------
// timer_cb – 100 Hz hlavná slučka
//
// 1. Kontrola timeoutu
// 2. Vyhladenie (EMA)
// 3. Voliteľná IMU korekcia (PID na yaw rate)
// 4. Prevod na duty a zápis do motorov
// ---------------------------------------------------------------------------
void DriveNode::timer_cb() {
  const auto now = std::chrono::steady_clock::now();

  const double alpha   = std::clamp(get_parameter("smooth_alpha").as_double(), 0.01, 1.0);
  const int    timeout = get_parameter("cmd_vel_timeout_ms").as_int();
  const bool   is_auto = (get_parameter("control_mode").as_string() != "manual");

  // --- Timeout ---
  double tl, tr;
  {
    std::lock_guard<std::mutex> lk(mu_);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_cmd_).count();
    const bool timed_out = !have_cmd_ || (elapsed_ms > timeout);
    if (timed_out) {
      target_l_ = 0.0;
      target_r_ = 0.0;
    }
    tl = target_l_;
    tr = target_r_;
  }

  // --- Vyhladzovanie (EMA) ---
  smooth_l_ += alpha * (tl - smooth_l_);
  smooth_r_ += alpha * (tr - smooth_r_);

  double cl = smooth_l_;
  double cr = smooth_r_;

  // --- IMU korekcia priamej jazdy ---
  double corr = 0.0;
  const bool imu_on = get_parameter("imu_correction").as_bool();
  // Korekcia sa aplikuje len keď ideme priamo (malý rozdiel L/R, aspoň minimálna rýchlosť)
  const double avg_speed = 0.5 * (std::abs(cl) + std::abs(cr));
  const bool going_straight = avg_speed > 0.1 && std::abs(cl - cr) < 0.15;

  if (imu_on && going_straight) {
    const double kp = get_parameter("imu_kp").as_double();
    const double ki = get_parameter("imu_ki").as_double();
    const double kd = get_parameter("imu_kd").as_double();
    const double db = get_parameter("imu_deadband").as_double();
    const double wl = get_parameter("imu_windup").as_double();

    double yaw_rate;
    { std::lock_guard<std::mutex> lk(mu_); yaw_rate = imu_yaw_rate_; }

    const double dt = std::chrono::duration<double>(now - imu_pid_time_).count();
    imu_pid_time_ = now;

    // Aplikuj mŕtvu zónu
    const double error = std::abs(yaw_rate) <= db ? 0.0
        : (yaw_rate > 0.0 ? yaw_rate - db : yaw_rate + db);

    if (dt > 0.001 && dt < 0.5) {
      imu_integral_ += error * dt;
      imu_integral_ = std::clamp(imu_integral_, -wl, wl);
      const double d_err = (error - imu_prev_error_) / dt;
      corr = kp * error + ki * imu_integral_ + kd * d_err;
    }
    imu_prev_error_ = error;

    const double sign = std::clamp(get_parameter("imu_sign").as_double(), -1.0, 1.0);
    corr *= sign;
  } else {
    imu_integral_   = 0.0;
    imu_prev_error_ = 0.0;
    imu_pid_time_   = now;
  }

  cl = std::clamp(cl - corr, -1.0, 1.0);
  cr = std::clamp(cr + corr, -1.0, 1.0);

  // --- PWM ---
  const uint16_t dl = to_duty(cl);
  const uint16_t dr = to_duty(cr);

  // Debug: [pwm_l%, pwm_r%, left_cmd, right_cmd, imu_corr, imu_yaw_rate]
  {
    double yaw_dbg = 0.0;
    { std::lock_guard<std::mutex> lk(mu_); yaw_dbg = imu_yaw_rate_; }
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {
      static_cast<double>(dl) * 100.0 / 4095.0,
      static_cast<double>(dr) * 100.0 / 4095.0,
      cl, cr, corr, yaw_dbg
    };
    pub_debug_->publish(dbg);
  }

  // --- Motory ---
  if (dl == 0 && dr == 0) {
    hat_->motor_stop(0);
    hat_->motor_stop(1);
    return;
  }
  hat_->apply_drive(dl, cl < 0.0, dr, cr < 0.0);
}

}  // namespace nodes
