#!/usr/bin/env python3
"""
Motor controller node – Wave Rover differential drive.

Subscribes:
  /cmd_vel  (geometry_msgs/Twist)   – velocity commands from mux/Nav2

Publishes:
  /motor_pwm  (std_msgs/Float32MultiArray)  – debug: [left_pct, right_pct]

Parameters (all runtime-adjustable):
  wheel_base          [m]     default 0.20   (track width)
  max_wheel_speed     [m/s]   default 0.35
  max_pwm             [%]     default 100.0  (0-100 scale sent to driver)
  pwm_boost           [-]     default 2.35   (multiplier to overcome stiction)
  pwm_freq_hz         [Hz]    default 800.0  (PCA9685 frequency)
  smooth_alpha        [-]     default 0.8    (EMA filter, 1.0 = no filter)
  cmd_vel_timeout_ms  [ms]    default 500    (watchdog: stop if stale)
  i2c_bus             [-]     default 1
  i2c_address         [-]     default 0x40
  use_mock            [bool]  default false  (set true if no HAT present)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from wave_rover_utils.motor_driver import create_driver


class MotorControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('motor_controller_node')

        # ── declare parameters ──────────────────────────────────────────
        self.declare_parameter('wheel_base', 0.20)
        self.declare_parameter('max_wheel_speed', 0.35)
        self.declare_parameter('max_pwm', 100.0)
        self.declare_parameter('pwm_boost', 2.35)
        self.declare_parameter('pwm_freq_hz', 800.0)
        self.declare_parameter('smooth_alpha', 0.8)
        self.declare_parameter('cmd_vel_timeout_ms', 500)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('use_mock', False)

        self._load_params()

        # ── hardware driver ──────────────────────────────────────────────
        self._driver = create_driver(
            use_mock=self.get_parameter('use_mock').value,
            i2c_bus=self.get_parameter('i2c_bus').value,
            address=self.get_parameter('i2c_address').value,
            pwm_freq_hz=self.get_parameter('pwm_freq_hz').value,
        )

        # ── state ────────────────────────────────────────────────────────
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._smooth_l = 0.0
        self._smooth_r = 0.0
        self._last_cmd_ns = self.get_clock().now().nanoseconds

        # ── ROS interfaces ───────────────────────────────────────────────
        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        self._pub_pwm = self.create_publisher(
            Float32MultiArray, 'motor_pwm', 10)

        self._timer = self.create_timer(0.01, self._loop)   # 100 Hz

        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'MotorController ready  wheel_base=%.3fm  max_speed=%.2fm/s  '
            'mock=%s',
            self._wheel_base, self._max_speed,
            self.get_parameter('use_mock').value,
        )

    # ── parameter helpers ────────────────────────────────────────────────

    def _load_params(self) -> None:
        self._wheel_base = max(0.01, self.get_parameter('wheel_base').value)
        self._max_speed  = max(0.05, self.get_parameter('max_wheel_speed').value)
        self._max_pwm    = max(10.0, min(100.0, self.get_parameter('max_pwm').value))
        self._boost      = max(1.0,  min(3.0,   self.get_parameter('pwm_boost').value))
        self._alpha      = max(0.05, min(1.0,   self.get_parameter('smooth_alpha').value))
        self._timeout_ms = max(100,  self.get_parameter('cmd_vel_timeout_ms').value)

    def _on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    # ── callbacks ────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._linear_x   = msg.linear.x
        self._angular_z  = msg.angular.z
        self._last_cmd_ns = self.get_clock().now().nanoseconds

    # ── main control loop ────────────────────────────────────────────────

    def _loop(self) -> None:
        # Timeout guard
        elapsed_ms = (self.get_clock().now().nanoseconds - self._last_cmd_ns) / 1e6
        if elapsed_ms > self._timeout_ms:
            self._linear_x  = 0.0
            self._angular_z = 0.0

        # Differential drive kinematics:
        #   v_l = (v - ω * L/2) / v_max
        #   v_r = (v + ω * L/2) / v_max
        half = 0.5 * self._wheel_base
        v_l = (self._linear_x - self._angular_z * half) / self._max_speed
        v_r = (self._linear_x + self._angular_z * half) / self._max_speed
        v_l = max(-1.0, min(1.0, v_l))
        v_r = max(-1.0, min(1.0, v_r))

        # Exponential smoothing (EMA)
        self._smooth_l += self._alpha * (v_l - self._smooth_l)
        self._smooth_r += self._alpha * (v_r - self._smooth_r)

        # Scale to PWM percentage, apply boost
        pwm_l = max(-100.0, min(100.0, self._smooth_l * self._max_pwm * self._boost))
        pwm_r = max(-100.0, min(100.0, self._smooth_r * self._max_pwm * self._boost))

        # Drive
        if abs(pwm_l) < 1.0 and abs(pwm_r) < 1.0:
            self._driver.stop()
            self._smooth_l = 0.0
            self._smooth_r = 0.0
        else:
            self._driver.set_speed(pwm_l, pwm_r)

        # Debug publish
        out = Float32MultiArray()
        out.data = [float(pwm_l), float(pwm_r)]
        self._pub_pwm.publish(out)

        self.get_logger().debug(
            'cmd(%.3f, %.3f)  pwm(%.1f, %.1f)',
            self._linear_x, self._angular_z, pwm_l, pwm_r,
        )

    # ── cleanup ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._driver.stop()
        self._driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
