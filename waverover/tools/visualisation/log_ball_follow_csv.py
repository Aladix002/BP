#!/usr/bin/env python3
# CSV z /ball_follower/debug_signals (+ volitelne /drive_debug = PWM motory, + BT).
#   ros2 run waverover log_ball_follow_csv.py --ros-args -p output_file:=/tmp/ball_follow.csv
# Spusti pocas ball_follow_standalone; pre pwm_* musi bezat waverover_motor (drive_debug).

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class BallFollowLogger(Node):
    def __init__(self) -> None:
        super().__init__("ball_follow_logger")
        self.declare_parameter("output_file", str(Path.home() / "ball_follow_log.csv"))
        self.declare_parameter("signals_topic", "/ball_follower/debug_signals")
        self.declare_parameter("drive_debug_topic", "/drive_debug")
        self.declare_parameter("log_drive_debug", True)
        self.declare_parameter("bt_topic", "/ball_follow_bt/active_behaviour")
        self.declare_parameter("log_bt", True)
        self.declare_parameter("rate_hz", 30.0)

        out = Path(self.get_parameter("output_file").value).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        self._path = out
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._log_bt = bool(self.get_parameter("log_bt").value)
        self._log_motor = bool(self.get_parameter("log_drive_debug").value)
        header = [
            "t_sec",
            "state",
            "x_err",
            "radius_px",
            "conf",
            "err_shaped",
            "pid_integral",
            "d_err",
            "eff_kp",
            "pid_out",
            "diff_mix",
            "v_ln",
            "v_rn",
            "cmd_lin",
            "cmd_ang",
        ]
        if self._log_motor:
            header.extend(["pwm_l_pct", "pwm_r_pct"])
        if self._log_bt:
            header.append("bt_active")
        self._w.writerow(header)
        self._t0 = None
        self._sig = [float("nan")] * 14
        self._pwm_l = float("nan")
        self._pwm_r = float("nan")
        self._bt = ""

        st = self.get_parameter("signals_topic").value
        self.create_subscription(Float64MultiArray, st, self._sig_cb, 50)
        if self._log_motor:
            dd = self.get_parameter("drive_debug_topic").value
            self.create_subscription(Float64MultiArray, dd, self._dbg_cb, 50)
        if self._log_bt:
            self.create_subscription(
                String,
                self.get_parameter("bt_topic").value,
                self._bt_cb,
                10,
            )

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)

        parts = [st]
        if self._log_motor:
            parts.append(self.get_parameter("drive_debug_topic").value)
        if self._log_bt:
            parts.append("BT")
        self.get_logger().info(f"CSV: {' + '.join(parts)} -> {self._path} @ {rate:.1f} Hz")

    def _sig_cb(self, msg: Float64MultiArray) -> None:
        d = list(msg.data)
        for i in range(14):
            self._sig[i] = float(d[i]) if i < len(d) else float("nan")

    def _dbg_cb(self, msg: Float64MultiArray) -> None:
        d = list(msg.data)
        self._pwm_l = float(d[0]) if len(d) > 0 else float("nan")
        self._pwm_r = float(d[1]) if len(d) > 1 else float("nan")

    def _bt_cb(self, msg: String) -> None:
        self._bt = (msg.data or "").strip()

    def _tick(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0
        row = [f"{rel:.6f}"]
        for x in self._sig:
            row.append(f"{x:.8f}" if math.isfinite(x) else "")
        if self._log_motor:
            row.extend(
                [
                    f"{self._pwm_l:.8f}" if math.isfinite(self._pwm_l) else "",
                    f"{self._pwm_r:.8f}" if math.isfinite(self._pwm_r) else "",
                ]
            )
        if self._log_bt:
            row.append(self._bt.replace(",", ";"))
        self._w.writerow(row)
        self._f.flush()

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = BallFollowLogger()
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
