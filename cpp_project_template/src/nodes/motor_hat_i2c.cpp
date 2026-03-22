#include "nodes/motor_hat_i2c.hpp"

#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace nodes {

namespace {

constexpr uint8_t kPcaMode1 = 0x00;
constexpr uint8_t kPcaPrescale = 0xFE;
constexpr uint8_t kPcaLed0OnL = 0x06;

constexpr int kPwma = 0;
constexpr int kAin1 = 1;
constexpr int kAin2 = 2;
constexpr int kPwmb = 5;
constexpr int kBin1 = 3;
constexpr int kBin2 = 4;

constexpr uint8_t kPcaFullOnHigh = 0x10;
constexpr uint8_t kPcaFullOffHigh = 0x10;

}  // namespace

MotorHatI2c::MotorHatI2c(int i2c_bus, uint8_t addr7) {
  std::string path = "/dev/i2c-" + std::to_string(i2c_bus);
  fd_ = open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Nepodarilo sa otvorit " + path + " (i2c alebo sudo?)");
  }
  if (ioctl(fd_, I2C_SLAVE, static_cast<int>(addr7)) < 0) {
    close(fd_);
    fd_ = -1;
    throw std::runtime_error("I2C_SLAVE zlyhalo");
  }
  write_reg(kPcaMode1, 0x00);
}

MotorHatI2c::~MotorHatI2c() {
  if (fd_ >= 0) {
    close(fd_);
  }
}

void MotorHatI2c::write_reg(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  if (write(fd_, buf, sizeof(buf)) != static_cast<ssize_t>(sizeof(buf))) {
    throw std::runtime_error("I2C write zlyhal");
  }
}

uint8_t MotorHatI2c::read_reg(uint8_t reg) {
  if (write(fd_, &reg, 1) != 1) {
    throw std::runtime_error("I2C read addr zlyhal");
  }
  uint8_t v = 0;
  if (read(fd_, &v, 1) != 1) {
    throw std::runtime_error("I2C read zlyhal");
  }
  return v;
}

void MotorHatI2c::set_pwm_freq_hz(double freq) {
  double prescaleval = 25000000.0 / 4096.0 / freq - 1.0;
  auto prescale = static_cast<uint8_t>(std::floor(prescaleval + 0.5));
  if (prescale < 3) {
    prescale = 3;
  }
  uint8_t oldmode = read_reg(kPcaMode1);
  uint8_t newmode = static_cast<uint8_t>((oldmode & 0x7F) | 0x10);
  write_reg(kPcaMode1, newmode);
  write_reg(kPcaPrescale, prescale);
  write_reg(kPcaMode1, oldmode);
  usleep(5000);
  write_reg(kPcaMode1, static_cast<uint8_t>(oldmode | 0x80));
}

void MotorHatI2c::set_pwm_channel(int channel, uint16_t on, uint16_t off) {
  uint8_t base = static_cast<uint8_t>(kPcaLed0OnL + 4 * channel);
  write_reg(base + 0, static_cast<uint8_t>(on & 0xFF));
  write_reg(base + 1, static_cast<uint8_t>((on >> 8) & 0xFF));
  write_reg(base + 2, static_cast<uint8_t>(off & 0xFF));
  write_reg(base + 3, static_cast<uint8_t>((off >> 8) & 0xFF));
}

void MotorHatI2c::set_channel_full_on(int channel) {
  uint8_t base = static_cast<uint8_t>(kPcaLed0OnL + 4 * channel);
  write_reg(base + 0, 0);
  write_reg(base + 1, kPcaFullOnHigh);
  write_reg(base + 2, 0);
  write_reg(base + 3, 0);
}

void MotorHatI2c::set_channel_full_off(int channel) {
  uint8_t base = static_cast<uint8_t>(kPcaLed0OnL + 4 * channel);
  write_reg(base + 0, 0);
  write_reg(base + 1, 0);
  write_reg(base + 2, 0);
  write_reg(base + 3, kPcaFullOffHigh);
}

void MotorHatI2c::set_duty_percent(int channel, int percent) {
  percent = std::clamp(percent, 0, 100);
  if (percent <= 0) {
    set_channel_full_off(channel);
    return;
  }
  if (percent >= 100) {
    set_channel_full_on(channel);
    return;
  }
  const uint16_t off = static_cast<uint16_t>(
      std::lround(static_cast<double>(percent) * 4095.0 / 100.0));
  set_pwm_channel(channel, 0, off);
}

void MotorHatI2c::set_level(int channel, bool high) {
  if (high) {
    set_channel_full_on(channel);
  } else {
    set_channel_full_off(channel);
  }
}

void MotorHatI2c::motor_stop(int motor) {
  set_level(motor == 0 ? kAin1 : kBin1, false);
  set_level(motor == 0 ? kAin2 : kBin2, false);
  set_duty_percent(motor == 0 ? kPwma : kPwmb, 0);
}

void MotorHatI2c::apply_drive(int pct_left, bool fwd_left, int pct_right, bool fwd_right) {
  pct_left = std::clamp(pct_left, 0, 100);
  pct_right = std::clamp(pct_right, 0, 100);

  if (pct_left <= 0) {
    set_level(kAin1, false);
    set_level(kAin2, false);
  } else {
    set_level(kAin1, !fwd_left);
    set_level(kAin2, fwd_left);
  }
  if (pct_right <= 0) {
    set_level(kBin1, false);
    set_level(kBin2, false);
  } else {
    set_level(kBin1, !fwd_right);
    set_level(kBin2, fwd_right);
  }
  set_duty_percent(kPwma, pct_left);
  set_duty_percent(kPwmb, pct_right);
}

}  // namespace nodes
