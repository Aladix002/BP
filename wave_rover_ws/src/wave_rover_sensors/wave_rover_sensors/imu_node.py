#!/usr/bin/env python3
"""
IMU node – publishes sensor_msgs/Imu from the Wave Rover IMU.

Supports two backends (selected by `backend` parameter):

  http  – Polls the ESP32 JSON API (same as the existing C++ ImuHttpNode)
          GET http://<esp32_ip>/js?json={"T":126}
          → { "T":126, "r":<roll>, "p":<pitch>, "y":<yaw>,
              "ax":<ax>, "ay":<ay>, "az":<az>,
              "gx":<gx>, "gy":<gy>, "gz":<gz> }

  i2c   – Reads IMU directly over I2C (MPU6050/QMI8658).
          Requires `smbus2` and `mpu6050-raspberrypi` (or similar).

Publishes:
  /imu/data   (sensor_msgs/Imu)    – full IMU message
  /imu/raw    (sensor_msgs/Imu)    – identical (legacy topic alias)

Parameters:
  backend         [string]  default "http"      – "http" | "i2c"
  rate_hz         [float]   default 50.0
  frame_id        [string]  default "imu_link"

  # HTTP backend
  esp32_ip        [string]  default "192.168.0.224"
  esp32_port      [int]     default 80
  http_timeout_s  [float]   default 0.1

  # I2C backend
  i2c_bus         [int]     default 1
  i2c_address     [int]     default 0x68   (MPU6050 default)
"""

import math
import json

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# Optional imports – only loaded when the respective backend is used
try:
    import urllib.request
    _HAS_HTTP = True
except ImportError:
    _HAS_HTTP = False


def _euler_to_quat(roll: float, pitch: float, yaw: float):
    """Convert Euler angles (rad) to quaternion (x,y,z,w)."""
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # x
        cr * sp * cy + sr * cp * sy,   # y
        cr * cp * sy - sr * sp * cy,   # z
        cr * cp * cy + sr * sp * sy,   # w
    )


class _HttpBackend:
    def __init__(self, ip: str, port: int, timeout: float) -> None:
        self._url     = f'http://{ip}:{port}/js?json={{"T":126}}'
        self._timeout = timeout

    def read(self) -> dict | None:
        try:
            with urllib.request.urlopen(self._url, timeout=self._timeout) as r:
                return json.loads(r.read().decode())
        except Exception:
            return None


class _I2CBackend:
    """Minimal MPU6050 reader via smbus2 (no external library required)."""

    _PWR_MGMT_1 = 0x6B
    _ACCEL_XOUT_H = 0x3B
    _GYRO_XOUT_H  = 0x43
    _ACCEL_SCALE  = 16384.0   # ±2g range
    _GYRO_SCALE   = 131.0     # ±250 °/s range → rad/s after conversion

    def __init__(self, bus: int, address: int) -> None:
        import smbus2
        self._bus  = smbus2.SMBus(bus)
        self._addr = address
        # Wake up MPU6050
        self._bus.write_byte_data(self._addr, self._PWR_MGMT_1, 0x00)

    def _read_raw(self, reg: int) -> int:
        h = self._bus.read_byte_data(self._addr, reg)
        l = self._bus.read_byte_data(self._addr, reg + 1)
        val = (h << 8) | l
        return val - 65536 if val >= 32768 else val

    def read(self) -> dict | None:
        try:
            ax = self._read_raw(self._ACCEL_XOUT_H) / self._ACCEL_SCALE * 9.80665
            ay = self._read_raw(self._ACCEL_XOUT_H + 2) / self._ACCEL_SCALE * 9.80665
            az = self._read_raw(self._ACCEL_XOUT_H + 4) / self._ACCEL_SCALE * 9.80665
            gx = math.radians(self._read_raw(self._GYRO_XOUT_H)     / self._GYRO_SCALE)
            gy = math.radians(self._read_raw(self._GYRO_XOUT_H + 2) / self._GYRO_SCALE)
            gz = math.radians(self._read_raw(self._GYRO_XOUT_H + 4) / self._GYRO_SCALE)
            return {'ax': ax, 'ay': ay, 'az': az,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'r': 0.0, 'p': 0.0, 'y': 0.0}
        except Exception:
            return None


class ImuNode(Node):

    def __init__(self) -> None:
        super().__init__('imu_node')

        self.declare_parameter('backend',        'http')
        self.declare_parameter('rate_hz',         50.0)
        self.declare_parameter('frame_id',        'imu_link')
        self.declare_parameter('esp32_ip',        '192.168.0.224')
        self.declare_parameter('esp32_port',       80)
        self.declare_parameter('http_timeout_s',   0.1)
        self.declare_parameter('i2c_bus',          1)
        self.declare_parameter('i2c_address',      0x68)

        backend  = self.get_parameter('backend').value.lower()
        rate_hz  = max(1.0, self.get_parameter('rate_hz').value)
        self._frame = self.get_parameter('frame_id').value

        if backend == 'i2c':
            self._backend = _I2CBackend(
                self.get_parameter('i2c_bus').value,
                self.get_parameter('i2c_address').value,
            )
            self.get_logger().info('IMU backend: I2C')
        else:
            self._backend = _HttpBackend(
                self.get_parameter('esp32_ip').value,
                self.get_parameter('esp32_port').value,
                self.get_parameter('http_timeout_s').value,
            )
            self.get_logger().info(
                'IMU backend: HTTP %s', self.get_parameter('esp32_ip').value)

        self._pub      = self.create_publisher(Imu, 'imu/data', 10)
        self._pub_raw  = self.create_publisher(Imu, 'imu/raw',  10)
        self._timer    = self.create_timer(1.0 / rate_hz, self._tick)
        self._errors   = 0

    def _tick(self) -> None:
        data = self._backend.read()
        if data is None:
            self._errors += 1
            if self._errors % 20 == 1:
                self.get_logger().warn('IMU read failed (%d times)', self._errors)
            return
        self._errors = 0

        roll  = float(data.get('r',  0.0))
        pitch = float(data.get('p',  0.0))
        yaw   = float(data.get('y',  0.0))
        ax    = float(data.get('ax', 0.0))
        ay    = float(data.get('ay', 0.0))
        az    = float(data.get('az', 0.0))
        gx    = float(data.get('gx', 0.0))
        gy    = float(data.get('gy', 0.0))
        gz    = float(data.get('gz', 0.0))

        qx, qy, qz, qw = _euler_to_quat(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance[0] = 4e-8
        msg.angular_velocity_covariance[4] = 4e-8
        msg.angular_velocity_covariance[8] = 4e-8

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance[0] = 2.89e-4
        msg.linear_acceleration_covariance[4] = 2.89e-4
        msg.linear_acceleration_covariance[8] = 2.89e-4

        self._pub.publish(msg)
        self._pub_raw.publish(msg)

        self.get_logger().debug(
            'IMU r=%.2f p=%.2f y=%.2f  gz=%.4f  ax=%.3f',
            math.degrees(roll), math.degrees(pitch), math.degrees(yaw), gz, ax,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
