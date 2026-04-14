#!/usr/bin/env python3
# Vizualizacia/meranie : pozri tools/visualisation/README.md
# Nahrava /imu + /drive_debug do CSV pre matplotlib (plot_imu_drive_csv.py).
#   ros2 run waverower log_imu_drive_csv.py --ros-args -p output_file:=/tmp/imu_drive.csv
# Stlpce drive_debug: pwm_l%%, pwm_r%%, cl, cr, imu_corr, yaw_filt (drive_node.cpp)

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray


class ImuDriveLogger(Node):
    def __init__(self) -> None:
        super().__init__("imu_drive_logger")
        self.declare_parameter("output_file", str(Path.home() / "imu_drive_log.csv"))
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("drive_debug_topic", "/drive_debug")
        self.declare_parameter("rate_hz", 30.0)

        out = Path(self.get_parameter("output_file").value).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        self._path = out
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(
            [
                "t_sec",
                "imu_wz_rad_s",
                "pwm_l_pct",
                "pwm_r_pct",
                "cl",
                "cr",
                "imu_corr",
                "yaw_filt_rad_s",
            ]
        )
        self._t0 = None

        self._imu_wz = float("nan")
        self._dbg = [float("nan")] * 6

        imu_t = self.get_parameter("imu_topic").value
        dbg_t = self.get_parameter("drive_debug_topic").value
        self.create_subscription(Imu, imu_t, self._imu_cb, 50)
        self.create_subscription(Float64MultiArray, dbg_t, self._dbg_cb, 50)

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(f"CSV: {imu_t} + {dbg_t} -> {self._path} @ {rate:.1f} Hz")

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_wz = float(msg.angular_velocity.z)

    def _dbg_cb(self, msg: Float64MultiArray) -> None:
        d = list(msg.data)
        for i in range(6):
            self._dbg[i] = float(d[i]) if i < len(d) else float("nan")

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0
        row = [f"{rel:.6f}", f"{self._imu_wz:.8f}"]
        row.extend(f"{x:.8f}" if math.isfinite(x) else "" for x in self._dbg)
        self._w.writerow(row)
        self._f.flush()

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = ImuDriveLogger()
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
        # Po Ctrl+C uz moze byt kontext vypnuty : druhe shutdown hodi RCLError
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
