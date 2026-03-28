#!/usr/bin/env python3
"""Arduino USB → sensor_msgs/Imu: CSV z MPU6050 sketchu (15 alebo 20 polí).

15 polí:
  ts_ms,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,temp_c,gx_dps,gy_dps,gz_dps

20 polí (komplementárny filter na Arduine):
  ...,gz_dps, roll_acc,pitch_acc, roll_f,pitch_f,yaw_gyro
  indexy 17–19 = roll_f, pitch_f, yaw_gyro [deg] → quaternion (yaw_gyro driftuje bez magneta)
  indexy 15–16 = roll_acc, pitch_acc → voliteľne /imu/arduino_extra
"""

import math
import queue
import subprocess
import sys
import termios
import threading
from typing import BinaryIO, List, Optional

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

_IMU_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

GRAVITY = 9.80665
DEG2RAD = math.pi / 180.0


def _baud_from_param(node: Node) -> int:
    pv = node.get_parameter("baud_rate").get_parameter_value()
    if pv.type == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if pv.type == ParameterType.PARAMETER_STRING and pv.string_value:
        return int(pv.string_value.strip())
    return 115200


def _quaternion_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple:
    """RPY v stupňoch, rotácia ZYX (bežná pre mobilnú platformu). Vráti (x, y, z, w)."""
    r = roll_deg * DEG2RAD
    p = pitch_deg * DEG2RAD
    y = yaw_deg * DEG2RAD
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yq = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, yq, z, w)


class ImuSerialFusionBridge(Node):
    def __init__(self) -> None:
        super().__init__("imu_serial_fusion_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic", "/imu")
        self.declare_parameter("run_stty", True)
        self.declare_parameter("use_best_effort_qos", False)
        self.declare_parameter("publish_extra_fields", True)
        self.declare_parameter("extra_topic", "/imu/arduino_extra")
        self.declare_parameter("fill_orientation_from_fusion", True)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = _baud_from_param(self)
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        run_stty = self.get_parameter("run_stty").get_parameter_value().bool_value
        use_be = self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        self._pub_extra = self.get_parameter("publish_extra_fields").get_parameter_value().bool_value
        extra_topic = self.get_parameter("extra_topic").get_parameter_value().string_value
        self._fill_ori = self.get_parameter("fill_orientation_from_fusion").get_parameter_value().bool_value

        if run_stty:
            subprocess.run(
                ["stty", "-F", port, str(baud), "raw", "-echo"],
                check=False,
                capture_output=True,
            )

        try:
            self._fp: BinaryIO = open(port, "rb")
        except OSError as ex:
            self.get_logger().fatal(f"Nepodarilo sa otvoriť {port}: {ex}")
            raise

        try:
            termios.tcflush(self._fp.fileno(), termios.TCIFLUSH)
        except (OSError, termios.error):
            pass

        qos = qos_profile_sensor_data if use_be else _IMU_QOS
        self._pub = self.create_publisher(Imu, topic, qos)
        self._pub_ex = (
            self.create_publisher(Float64MultiArray, extra_topic, 10)
            if self._pub_extra
            else None
        )
        self._queue: queue.Queue = queue.Queue(maxsize=256)
        self._queue_ex: queue.Queue = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        self.create_timer(0.001, self._flush_publish)

        self.get_logger().info(
            f"IMU fusion serial → {topic} ({port} @ {baud}, 15/20 CSV, frame_id={self._frame_id})"
        )

    def _flush_publish(self) -> None:
        for _ in range(64):
            try:
                msg = self._queue.get_nowait()
            except queue.Empty:
                break
            self._pub.publish(msg)
        if self._pub_ex:
            for _ in range(8):
                try:
                    ex = self._queue_ex.get_nowait()
                except queue.Empty:
                    break
                self._pub_ex.publish(ex)

    def _read_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            try:
                raw = self._fp.readline()
            except OSError:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("ERR") or "READ_FAIL" in line:
                continue
            parsed = self._parse_line(line)
            if parsed is None:
                continue
            imu_msg, extra = parsed
            try:
                self._queue.put_nowait(imu_msg)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self._queue.put_nowait(imu_msg)
                except queue.Full:
                    pass
            if extra is not None and self._pub_ex:
                try:
                    self._queue_ex.put_nowait(extra)
                except queue.Full:
                    pass

    def _parse_line(self, line: str) -> Optional[tuple]:
        parts = [p.strip() for p in line.split(",")]
        n = len(parts)
        if n not in (15, 20):
            if len(parts) > 0 and parts[0].isalpha():
                return None
            return None
        try:
            f = [float(x) for x in parts]
        except ValueError:
            return None
        if any(f[i] == -1.0 for i in range(1, 8)):
            return None

        ax_g, ay_g, az_g = f[8], f[9], f[10]
        gx_dps, gy_dps, gz_dps = f[12], f[13], f[14]

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._frame_id

        out.linear_acceleration.x = ax_g * GRAVITY
        out.linear_acceleration.y = ay_g * GRAVITY
        out.linear_acceleration.z = az_g * GRAVITY

        out.angular_velocity.x = gx_dps * DEG2RAD
        out.angular_velocity.y = gy_dps * DEG2RAD
        out.angular_velocity.z = gz_dps * DEG2RAD

        extra_msg: Optional[Float64MultiArray] = None

        if n == 20 and self._fill_ori:
            # Zodpovedá hlavičke: roll_f, pitch_f, yaw_gyro (nie roll_acc/pitch_acc z 15–16)
            roll_deg, pitch_deg, yaw_deg = f[17], f[18], f[19]
            qx, qy, qz, qw = _quaternion_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
            out.orientation.x = qx
            out.orientation.y = qy
            out.orientation.z = qz
            out.orientation.w = qw
            for i in range(9):
                out.orientation_covariance[i] = 0.01
            if self._pub_extra:
                extra_msg = Float64MultiArray()
                extra_msg.data = [float(f[15]), float(f[16])]
        else:
            out.orientation_covariance[0] = -1.0

        return (out, extra_msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        if hasattr(self, "_reader") and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if hasattr(self, "_fp") and self._fp:
            try:
                self._fp.close()
            except OSError:
                pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ImuSerialFusionBridge()
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
