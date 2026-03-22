"""
Motor driver abstraction for Wave Rover PCA9685 HAT.

Hardware layout (matches motor_hat_i2c.cpp):
  PCA9685 I2C 0x40, PWM 800 Hz
  PWMA=ch0  AIN1=ch1  AIN2=ch2   -> Motor A (left wheel)
  PWMB=ch5  BIN1=ch3  BIN2=ch4   -> Motor B (right wheel)

Usage:
    driver = create_driver(use_mock=False, i2c_bus=1)
    driver.set_speed(left_pct=-50.0, right_pct=50.0)   # spin left
    driver.stop()
    driver.close()
"""

import abc
import logging
import math
import time

logger = logging.getLogger(__name__)

# PCA9685 register map
_MODE1 = 0x00
_MODE2 = 0x01
_PRESCALE = 0xFE
_LED0_ON_L = 0x06
_LED0_ON_H = 0x07
_LED0_OFF_L = 0x08
_LED0_OFF_H = 0x09
_ALL_LED_OFF_H = 0xFD

# Channel assignments
_PWMA = 0
_AIN1 = 1
_AIN2 = 2
_BIN1 = 3
_BIN2 = 4
_PWMB = 5

_MAX_PWM = 4095


class MotorDriver(abc.ABC):
    """Abstract motor driver interface."""

    @abc.abstractmethod
    def set_speed(self, left_pct: float, right_pct: float) -> None:
        """Set motor speeds.

        Args:
            left_pct:  Left wheel  [-100, 100]  positive = forward
            right_pct: Right wheel [-100, 100]  positive = forward
        """

    @abc.abstractmethod
    def stop(self) -> None:
        """Coast stop – release both motors."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release I2C bus / file descriptors."""


class MockMotorDriver(MotorDriver):
    """Software-only driver for development without hardware."""

    def __init__(self) -> None:
        logger.info("MockMotorDriver: running in software-only mode")

    def set_speed(self, left_pct: float, right_pct: float) -> None:
        logger.debug("MockMotor  L=%+6.1f%%  R=%+6.1f%%", left_pct, right_pct)

    def stop(self) -> None:
        logger.debug("MockMotor  STOP")

    def close(self) -> None:
        logger.debug("MockMotor  close")


class PCA9685MotorDriver(MotorDriver):
    """PCA9685 HAT driver via smbus2 – mirrors motor_hat_i2c.cpp behaviour."""

    def __init__(
        self,
        i2c_bus: int = 1,
        address: int = 0x40,
        pwm_freq_hz: float = 800.0,
    ) -> None:
        import smbus2  # type: ignore[import]

        self._bus = smbus2.SMBus(i2c_bus)
        self._addr = address
        self._reset()
        self.set_freq(pwm_freq_hz)
        logger.info(
            "PCA9685MotorDriver: bus=%d addr=0x%02X freq=%.0fHz",
            i2c_bus, address, pwm_freq_hz,
        )

    # ------------------------------------------------------------------
    # Low-level register access
    # ------------------------------------------------------------------

    def _write(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self._addr, reg, value & 0xFF)

    def _read(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)

    def _reset(self) -> None:
        self._write(_MODE1, 0x00)

    def set_freq(self, freq_hz: float) -> None:
        """Change PWM frequency (requires brief sleep)."""
        prescale = int(math.floor(25_000_000.0 / (4096.0 * freq_hz) - 1 + 0.5))
        prescale = max(3, min(255, prescale))
        old_mode = self._read(_MODE1)
        self._write(_MODE1, (old_mode & 0x7F) | 0x10)   # SLEEP bit
        self._write(_PRESCALE, prescale)
        self._write(_MODE1, old_mode)
        time.sleep(0.005)
        self._write(_MODE1, old_mode | 0xA0)             # AI + ALLCALL

    def _set_channel(self, ch: int, on: int, off: int) -> None:
        base = _LED0_ON_L + 4 * ch
        self._write(base + 0, on & 0xFF)
        self._write(base + 1, (on >> 8) & 0x0F)
        self._write(base + 2, off & 0xFF)
        self._write(base + 3, (off >> 8) & 0x0F)

    def _set_pwm(self, ch: int, duty: int) -> None:
        """duty: 0-4095"""
        duty = max(0, min(_MAX_PWM, duty))
        self._set_channel(ch, 0, duty)

    def _set_digital(self, ch: int, high: bool) -> None:
        if high:
            self._set_channel(ch, 4096, 0)
        else:
            self._set_channel(ch, 0, 4096)

    # ------------------------------------------------------------------
    # Motor helpers
    # ------------------------------------------------------------------

    def _drive_a(self, pct: float) -> None:
        """Motor A = left wheel.  pct in [-100, 100]."""
        duty = int(abs(pct) / 100.0 * _MAX_PWM)
        fwd = pct >= 0.0
        self._set_pwm(_PWMA, duty)
        self._set_digital(_AIN1, fwd)
        self._set_digital(_AIN2, not fwd)

    def _drive_b(self, pct: float) -> None:
        """Motor B = right wheel.  pct in [-100, 100]."""
        duty = int(abs(pct) / 100.0 * _MAX_PWM)
        fwd = pct >= 0.0
        self._set_pwm(_PWMB, duty)
        self._set_digital(_BIN1, fwd)
        self._set_digital(_BIN2, not fwd)

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def set_speed(self, left_pct: float, right_pct: float) -> None:
        left_pct = max(-100.0, min(100.0, left_pct))
        right_pct = max(-100.0, min(100.0, right_pct))
        self._drive_a(left_pct)
        self._drive_b(right_pct)
        logger.debug("PCA9685  L=%+6.1f%%  R=%+6.1f%%", left_pct, right_pct)

    def stop(self) -> None:
        # Coast: zero PWM + release direction pins
        for ch in (_PWMA, _PWMB):
            self._set_pwm(ch, 0)
        for ch in (_AIN1, _AIN2, _BIN1, _BIN2):
            self._set_digital(ch, False)
        logger.debug("PCA9685  STOP")

    def close(self) -> None:
        self.stop()
        self._bus.close()
        logger.debug("PCA9685  close")


def create_driver(
    use_mock: bool = False,
    i2c_bus: int = 1,
    address: int = 0x40,
    pwm_freq_hz: float = 800.0,
) -> MotorDriver:
    """Factory: return PCA9685 driver, or mock as fallback.

    If use_mock=False but smbus2/hardware is unavailable, automatically
    falls back to MockMotorDriver with a warning.
    """
    if use_mock:
        return MockMotorDriver()
    try:
        return PCA9685MotorDriver(i2c_bus, address, pwm_freq_hz)
    except Exception as exc:
        logger.warning(
            "PCA9685 init failed (%s) – falling back to MockMotorDriver", exc
        )
        return MockMotorDriver()
