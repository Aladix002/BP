#!/usr/bin/env python3
"""1D Kalman filter pre každú zložku linear_acceleration a angular_velocity (Imu).

Orientáciu nemení (prepíše z poslednej správy); na plný „AHRS Kalman“ by bol potrebný
Quaternion EKF — bežne sa v ROS 2 rieši cez imu_filter_madgwick alebo robot_localization.

Parametre Q/R ladíš podľa šumu senzora (vyššie R = viac dôvery modelu / hladšie).
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu

_IMU_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def _double_param(node: Node, name: str, default: float) -> float:
    pv = node.get_parameter(name).get_parameter_value()
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if pv.type == ParameterType.PARAMETER_STRING and pv.string_value:
        return float(pv.string_value.strip())
    return default


class ScalarKalman:
    """Konštantný model: x_k ≈ x_{k-1} + w, meranie z = x + v."""

    __slots__ = ("_x", "_p", "_q", "_r", "_has_state")

    def __init__(self, q: float, r: float) -> None:
        self._q = q
        self._r = r
        self._x = 0.0
        self._p = 1.0
        self._has_state = False

    def update(self, z: float) -> float:
        if not self._has_state:
            self._x = z
            self._has_state = True
            return self._x
        self._p += self._q
        k = self._p / (self._p + self._r)
        self._x += k * (z - self._x)
        self._p *= 1.0 - k
        return self._x


class ImuKalmanFilter(Node):
    def __init__(self) -> None:
        super().__init__("imu_kalman_filter")

        self.declare_parameter("input_topic", "/imu")
        self.declare_parameter("output_topic", "/imu/filtered")
        self.declare_parameter("process_noise_accel", 0.001)
        self.declare_parameter("process_noise_gyro", 1.0e-6)
        self.declare_parameter("measurement_noise_accel", 0.05)
        self.declare_parameter("measurement_noise_gyro", 0.001)
        self.declare_parameter("use_best_effort_qos", False)

        in_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        qa = _double_param(self, "process_noise_accel", 0.001)
        qg = _double_param(self, "process_noise_gyro", 1.0e-6)
        ra = _double_param(self, "measurement_noise_accel", 0.05)
        rg = _double_param(self, "measurement_noise_gyro", 0.001)
        use_be = self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value

        self._filters_ax = ScalarKalman(qa, ra)
        self._filters_ay = ScalarKalman(qa, ra)
        self._filters_az = ScalarKalman(qa, ra)
        self._filters_gx = ScalarKalman(qg, rg)
        self._filters_gy = ScalarKalman(qg, rg)
        self._filters_gz = ScalarKalman(qg, rg)

        qos = qos_profile_sensor_data if use_be else _IMU_QOS
        self._pub = self.create_publisher(Imu, out_topic, qos)
        self.create_subscription(Imu, in_topic, self._cb, qos)

        self.get_logger().info(f"Kalman IMU: {in_topic} → {out_topic} (6× 1D Kalman na a, ω)")

    def _cb(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.linear_acceleration.x = self._filters_ax.update(msg.linear_acceleration.x)
        out.linear_acceleration.y = self._filters_ay.update(msg.linear_acceleration.y)
        out.linear_acceleration.z = self._filters_az.update(msg.linear_acceleration.z)
        out.angular_velocity.x = self._filters_gx.update(msg.angular_velocity.x)
        out.angular_velocity.y = self._filters_gy.update(msg.angular_velocity.y)
        out.angular_velocity.z = self._filters_gz.update(msg.angular_velocity.z)
        out.orientation = msg.orientation
        out.orientation_covariance = list(msg.orientation_covariance)
        out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)
        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        self._pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuKalmanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
