#ifndef NODES_MOTOR_HAT_I2C_HPP
#define NODES_MOTOR_HAT_I2C_HPP

#include <cstdint>

namespace nodes {

/** PCA9685 + TB6612 na Waveshare Motor Driver HAT (I2C). */
class MotorHatI2c {
public:
  explicit MotorHatI2c(int i2c_bus, uint8_t addr7);
  ~MotorHatI2c();

  MotorHatI2c(const MotorHatI2c&) = delete;
  MotorHatI2c& operator=(const MotorHatI2c&) = delete;

  void set_pwm_freq_hz(double freq);
  void motor_stop(int motor);
  void apply_drive(int pct_left, bool fwd_left, int pct_right, bool fwd_right);

private:
  void write_reg(uint8_t reg, uint8_t val);
  uint8_t read_reg(uint8_t reg);
  void set_pwm_channel(int channel, uint16_t on, uint16_t off);
  void set_channel_full_on(int channel);
  void set_channel_full_off(int channel);
  void set_duty_percent(int channel, int percent);
  void set_level(int channel, bool high);

  int fd_{-1};
};

}  // namespace nodes

#endif
