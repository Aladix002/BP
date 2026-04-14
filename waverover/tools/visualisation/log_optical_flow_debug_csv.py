#!/usr/bin/env python3
# Nahrava /optical_flow_debug (Float64MultiArray) do CSV pre graf mean_dx.
#   ros2 run waverover log_optical_flow_debug_csv.py --ros-args -p output_file:=/tmp/flow_debug.csv
# Stlpce: mean_dx_px, mean_dx_norm, flow_corr, n_good, n_corners (optical_flow.cpp)

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class OpticalFlowDebugLogger(Node):
    def __init__(self) -> None:
        super().__init__("optical_flow_debug_logger")
        self.declare_parameter("output_file", str(Path.home() / "optical_flow_debug.csv"))
        self.declare_parameter("debug_topic", "/optical_flow_debug")
        self.declare_parameter("rate_hz", 60.0)

        out = Path(self.get_parameter("output_file").value).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        self._path = out
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(
            [
                "t_sec",
                "mean_dx_px",
                "mean_dx_norm",
                "flow_corr",
                "n_good",
                "n_corners",
            ]
        )
        self._t0 = None
        self._last = [float("nan")] * 5

        topic = self.get_parameter("debug_topic").value
        self.create_subscription(Float64MultiArray, topic, self._cb, 50)

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(f"CSV: {topic} -> {self._path} @ {rate:.1f} Hz")

    def _cb(self, msg: Float64MultiArray) -> None:
        d = list(msg.data)
        for i in range(5):
            self._last[i] = float(d[i]) if i < len(d) else float("nan")

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0
        row = [f"{rel:.6f}"]
        row.extend(f"{x:.8f}" if math.isfinite(x) else "" for x in self._last)
        self._w.writerow(row)
        self._f.flush()

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = OpticalFlowDebugLogger()
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
