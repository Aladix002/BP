#!/usr/bin/env python3
# Vizualizacia/meranie — tools/visualisation/README.md
# Nahrava /teleop_cmd_vel + /teleop_cmd_vel_corrected (+ volitelne /drive_debug) do CSV.
#   ros2 run waverower log_optical_flow_csv.py --ros-args -p output_file:=/tmp/flow.csv
# Spusti pri correction_mode:=optical_flow a zapnutom optical_flow_node (enabled).

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class OpticalFlowLogger(Node):
    def __init__(self) -> None:
        super().__init__("optical_flow_logger")
        self.declare_parameter("output_file", str(Path.home() / "optical_flow_log.csv"))
        self.declare_parameter("teleop_topic", "/teleop_cmd_vel")
        self.declare_parameter("corrected_topic", "/teleop_cmd_vel_corrected")
        self.declare_parameter("drive_debug_topic", "/drive_debug")
        self.declare_parameter("log_drive_debug", True)
        self.declare_parameter("rate_hz", 30.0)

        out = Path(self.get_parameter("output_file").value).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        self._path = out
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._log_dbg = bool(self.get_parameter("log_drive_debug").value)
        header = [
            "t_sec",
            "teleop_linear_x",
            "teleop_angular_z",
            "out_linear_x",
            "out_angular_z",
            "flow_delta_angular_z",
        ]
        if self._log_dbg:
            header.extend(["pwm_l_pct", "pwm_r_pct"])
        self._w.writerow(header)
        self._t0 = None

        self._tlx = float("nan")
        self._taz = float("nan")
        self._olx = float("nan")
        self._oaz = float("nan")
        self._pwm_l = float("nan")
        self._pwm_r = float("nan")

        tp = self.get_parameter("teleop_topic").value
        co = self.get_parameter("corrected_topic").value
        self.create_subscription(Twist, tp, self._teleop_cb, 50)
        self.create_subscription(Twist, co, self._corrected_cb, 50)
        if self._log_dbg:
            dd = self.get_parameter("drive_debug_topic").value
            self.create_subscription(Float64MultiArray, dd, self._dbg_cb, 50)

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

        extra = f" + {self.get_parameter('drive_debug_topic').value}" if self._log_dbg else ""
        self.get_logger().info(f"CSV: {tp} + {co}{extra} -> {self._path} @ {rate:.1f} Hz")

    def _teleop_cb(self, msg: Twist) -> None:
        self._tlx = float(msg.linear.x)
        self._taz = float(msg.angular.z)

    def _corrected_cb(self, msg: Twist) -> None:
        self._olx = float(msg.linear.x)
        self._oaz = float(msg.angular.z)

    def _dbg_cb(self, msg: Float64MultiArray) -> None:
        d = list(msg.data)
        self._pwm_l = float(d[0]) if len(d) > 0 else float("nan")
        self._pwm_r = float(d[1]) if len(d) > 1 else float("nan")

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0
        delta = self._oaz - self._taz if math.isfinite(self._oaz) and math.isfinite(self._taz) else float("nan")
        row = [
            f"{rel:.6f}",
            f"{self._tlx:.8f}" if math.isfinite(self._tlx) else "",
            f"{self._taz:.8f}" if math.isfinite(self._taz) else "",
            f"{self._olx:.8f}" if math.isfinite(self._olx) else "",
            f"{self._oaz:.8f}" if math.isfinite(self._oaz) else "",
            f"{delta:.8f}" if math.isfinite(delta) else "",
        ]
        if self._log_dbg:
            row.extend(
                [
                    f"{self._pwm_l:.8f}" if math.isfinite(self._pwm_l) else "",
                    f"{self._pwm_r:.8f}" if math.isfinite(self._pwm_r) else "",
                ]
            )
        self._w.writerow(row)
        self._f.flush()

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = OpticalFlowLogger()
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
