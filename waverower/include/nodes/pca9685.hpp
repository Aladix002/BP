#ifndef NODES_PCA9685_HPP
#define NODES_PCA9685_HPP

#include <cstdint>

namespace nodes {

// Ovladanie PCA9685 (PWM driver) + logika TB6612 kanalov na Waveshare Motor HAT (I2C).
// Kanaly 0-2 = motor A (PWM + IN1 + IN2), 3-5 = motor B - pozri pca9685.cpp.
class Pca9685 {
public:
  explicit Pca9685(int i2c_bus, uint8_t addr7);
  ~Pca9685();

  Pca9685(const Pca9685&) = delete;
  Pca9685& operator=(const Pca9685&) = delete;

  void set_pwm_freq_hz(double freq);
  void motor_stop(int motor);
  void apply_drive(uint16_t duty_left, bool fwd_left, uint16_t duty_right, bool fwd_right);

private:
  void write_reg(uint8_t reg, uint8_t val);
  uint8_t read_reg(uint8_t reg);
  void set_pwm_channel(int channel, uint16_t on, uint16_t off);
  void set_channel_full_on(int channel);
  void set_channel_full_off(int channel);
  void set_duty_12bit(int channel, uint16_t duty);
  void set_duty_percent(int channel, int percent);
  void set_level(int channel, bool high);

  int fd_{-1};
};

}  // namespace nodes

#endif
