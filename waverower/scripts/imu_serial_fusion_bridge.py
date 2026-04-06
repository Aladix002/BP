#!/usr/bin/env python3
"""Arduino USB -> sensor_msgs/Imu (CSV MPU6050, 15 alebo 20 poli).

20-polovy format: roll_f, pitch_f, yaw_gyro [deg] -> quaternion.
"""

import math
import os
import queue
import subprocess
import sys
import termios
import threading
from glob import glob
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
    """RPY [deg], ZYX, vrati quaternion (x,y,z,w)."""
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


def _resolve_serial_port(requested_port: str) -> str:
    """
    Resolve serial device path robustly.
    - If requested exists, use it.
    - If requested is missing (/dev/ttyACM0), try Arduino by-id first, then ttyACM*.
    """
    if requested_port and os.path.exists(requested_port):
        return requested_port

    by_id_candidates = sorted(
        p for p in glob("/dev/serial/by-id/*") if "arduino" in os.path.basename(p).lower()
    )
    for path in by_id_candidates:
        if os.path.exists(path):
            return path

    acm_candidates = sorted(glob("/dev/ttyACM*"))
    if acm_candidates:
        return acm_candidates[0]

    usb_candidates = sorted(glob("/dev/ttyUSB*"))
    if usb_candidates:
        return usb_candidates[0]

    return requested_port


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
        self.declare_parameter("zero_yaw_on_start", True)

        requested_port = self.get_parameter("serial_port").get_parameter_value().string_value
        port = _resolve_serial_port(requested_port)
        baud = _baud_from_param(self)
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        run_stty = self.get_parameter("run_stty").get_parameter_value().bool_value
        use_be = self.get_parameter("use_best_effort_qos").get_parameter_value().bool_value
        self._pub_extra = self.get_parameter("publish_extra_fields").get_parameter_value().bool_value
        extra_topic = self.get_parameter("extra_topic").get_parameter_value().string_value
        self._fill_ori = self.get_parameter("fill_orientation_from_fusion").get_parameter_value().bool_value
        self._zero_yaw = self.get_parameter("zero_yaw_on_start").get_parameter_value().bool_value
        self._yaw_offset: Optional[float] = None

        if port != requested_port:
            self.get_logger().warn(
                f"Port {requested_port} nedostupny, pouzivam {port}"
            )

        if run_stty:
            subprocess.run(
                ["stty", "-F", port, str(baud), "raw", "-echo"],
                check=False,
                capture_output=True,
            )

        try:
            self._fp: BinaryIO = open(port, "rb")
        except OSError as ex:
            self.get_logger().fatal(f"Nepodarilo otvorit {port}: {ex}")
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
            f"IMU fusion serial -> {topic} ({port} @ {baud}, 15/20 CSV, frame_id={self._frame_id})"
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

    def _apply_yaw_offset(self, yaw_deg: float) -> float:
        """Yaw offset pri prvom merani."""
        if not self._zero_yaw:
            return yaw_deg
        if self._yaw_offset is None:
            self._yaw_offset = yaw_deg
            self.get_logger().info(f"Yaw vynulovany, pociatok: {yaw_deg:.2f} deg")
        return yaw_deg - self._yaw_offset

    def _parse_line(self, line: str) -> Optional[tuple]:
        parts = [p.strip() for p in line.split(",")]
        n = len(parts)
        if n not in (15, 16, 20):
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
            # roll_f, pitch_f, yaw_gyro z CSV
            roll_deg, pitch_deg, yaw_deg = f[17], f[18], f[19]
            yaw_deg = self._apply_yaw_offset(yaw_deg)
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
        elif n == 16 and self._fill_ori:
            # 16 poli: 15 + yaw_gyro
            yaw_deg = self._apply_yaw_offset(f[15])
            qx, qy, qz, qw = _quaternion_from_rpy_deg(0.0, 0.0, yaw_deg)
            out.orientation.x = qx
            out.orientation.y = qy
            out.orientation.z = qz
            out.orientation.w = qw
            for i in range(9):
                out.orientation_covariance[i] = 0.05  # len yaw
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
