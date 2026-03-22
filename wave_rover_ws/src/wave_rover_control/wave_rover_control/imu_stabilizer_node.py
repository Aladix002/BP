#!/usr/bin/env python3
"""
IMU stabilizer node – corrects angular drift without wheel encoders.

When the robot receives a turn command, the actual yaw rate measured by
the IMU may differ from the commanded rate (due to slip, uneven surface,
motor mismatch).  This node applies a P-controller correction.

Subscribes:
  /cmd_vel       (geometry_msgs/Twist)   – desired velocity
  /imu/data      (sensor_msgs/Imu)       – measured angular velocity

Publishes:
  /cmd_vel_stable (geometry_msgs/Twist)  – corrected velocity

Parameters (runtime-adjustable):
  enabled    [bool]   default true     – disable to pass cmd_vel straight through
  Kp         [float]  default 0.3      – proportional gain
  Ki         [float]  default 0.01     – integral gain (anti-windup at ±0.5)
  Kd         [float]  default 0.05     – derivative gain
  max_correction [float]  default 0.5  – clamp on angular correction [rad/s]
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class ImuStabilizerNode(Node):

    def __init__(self) -> None:
        super().__init__('imu_stabilizer_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('Kp', 0.3)
        self.declare_parameter('Ki', 0.01)
        self.declare_parameter('Kd', 0.05)
        self.declare_parameter('max_correction', 0.5)

        self._load_params()

        # State
        self._desired_w  = 0.0
        self._measured_w = 0.0
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = self.get_clock().now()

        # ROS interfaces
        self._sub_cmd = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, 10)
        self._sub_imu = self.create_subscription(
            Imu, 'imu/data', self._imu_cb, 10)
        self._pub = self.create_publisher(
            Twist, 'cmd_vel_stable', 10)

        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'ImuStabilizer ready  enabled=%s  Kp=%.3f  Ki=%.4f  Kd=%.4f',
            self._enabled, self._Kp, self._Ki, self._Kd,
        )

    def _load_params(self) -> None:
        self._enabled      = self.get_parameter('enabled').value
        self._Kp           = self.get_parameter('Kp').value
        self._Ki           = self.get_parameter('Ki').value
        self._Kd           = self.get_parameter('Kd').value
        self._max_corr     = abs(self.get_parameter('max_correction').value)

    def _on_params(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    def _imu_cb(self, msg: Imu) -> None:
        self._measured_w = msg.angular_velocity.z

    def _cmd_cb(self, msg: Twist) -> None:
        if not self._enabled:
            self._pub.publish(msg)
            return

        self._desired_w = msg.angular.z

        # Time delta
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds / 1e9
        self._prev_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.01   # guard against large dt on startup

        # PID
        error = self._desired_w - self._measured_w
        self._integral += error * dt
        # Anti-windup
        self._integral = max(-0.5, min(0.5, self._integral))
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        correction = (
            self._Kp * error
            + self._Ki * self._integral
            + self._Kd * derivative
        )
        correction = max(-self._max_corr, min(self._max_corr, correction))

        out = Twist()
        out.linear.x  = msg.linear.x
        out.angular.z = msg.angular.z + correction

        self.get_logger().debug(
            'desired_w=%.3f  measured_w=%.3f  err=%.3f  corr=%.3f',
            self._desired_w, self._measured_w, error, correction,
        )

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuStabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
