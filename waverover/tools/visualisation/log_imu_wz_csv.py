#!/usr/bin/env python3
# /imu omega_z -> CSV.

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuWzLogger(Node):
    def __init__(self) -> None:
        super().__init__("imu_wz_logger")
        self.declare_parameter("output_file", str(Path.home() / "imu_log.csv"))
        self.declare_parameter("topic", "/imu")
        out = Path(self.get_parameter("output_file").value).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        self._path = out
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(["t_sec", "imu_wz_rad_s"])
        self._t0 = None
        topic = self.get_parameter("topic").value
        self.create_subscription(Imu, topic, self._cb, 50)
        self.get_logger().info(f"Logujem {topic} -> {self._path}")

    def _cb(self, msg: Imu) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        self._w.writerow([f"{t - self._t0:.6f}", f"{msg.angular_velocity.z:.8f}"])
        self._f.flush()

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = ImuWzLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
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
