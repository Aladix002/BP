#!/usr/bin/env python3
"""Autonómne bludenie – 2 stavy: FWD a STOP.

Stavy:
  FWD  – jazdí dopredu; ak predok ≤ threshold → STOP
  STOP – zastane, zvolí smer (L/R/180°), otočí sa,
         IMU potvrdí ± 15° od cieľa → späť do FWD

Dynamická rekonf.:
  ros2 param set /lidar_wander_node enabled false
  ros2 param set /lidar_wander_node threshold_m 0.30
  ros2 param set /lidar_wander_node turn_speed 1.8
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

_TOLERANCE_RAD = math.radians(15.0)   # ±15° tolerancia


class LidarWanderNode(Node):
    _FWD  = 0
    _STOP = 1

    def __init__(self) -> None:
        super().__init__("lidar_wander_node")

        self.declare_parameter("enabled",             True)
        self.declare_parameter("threshold_m",         0.30)
        self.declare_parameter("forward_speed",       0.10)
        self.declare_parameter("turn_speed",          1.80)
        self.declare_parameter("turn_timeout_s",       6.0)
        self.declare_parameter("lidar_rotation_deg", 90.0)
        self.declare_parameter("cmd_topic",       "/cmd_vel")

        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self._pub      = self.create_publisher(Twist, cmd_topic, 10)
        self._sub_scan = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, rclpy.qos.qos_profile_sensor_data
        )
        self._sub_imu  = self.create_subscription(
            Imu, "/imu", self._imu_cb, rclpy.qos.qos_profile_sensor_data
        )

        self._state:       int   = self._FWD
        self._turn_dir:    float = 1.0
        self._turn_target: float = math.radians(90.0)
        self._turn_accum:  float = 0.0
        self._turn_start         = None
        self._last_imu_t         = None

        self._d_front: float = math.inf
        self._d_left:  float = math.inf
        self._d_right: float = math.inf

        self.create_timer(0.1, self._ctrl_cb)
        self.get_logger().info(
            f"LidarWander → {cmd_topic}  "
            f"[threshold={self.get_parameter('threshold_m').get_parameter_value().double_value:.2f} m]"
        )

    # ── LiDAR ─────────────────────────────────────────────────────────────────

    def _sector_min(self, ranges, angle_min, angle_inc,
                    lo_deg, hi_deg, rotation_rad) -> float:
        lo, hi = math.radians(lo_deg), math.radians(hi_deg)
        best = math.inf
        for i, d in enumerate(ranges):
            if not math.isfinite(d) or d < 0.02:
                continue
            a = angle_min + i * angle_inc
            a_robot = math.atan2(math.sin(a + rotation_rad),
                                 math.cos(a + rotation_rad))
            if lo <= a_robot <= hi:
                best = min(best, d)
        return best

    def _scan_cb(self, msg: LaserScan) -> None:
        rot = math.radians(
            self.get_parameter("lidar_rotation_deg").get_parameter_value().double_value
        )
        self._d_front = self._sector_min(msg.ranges, msg.angle_min,
                                         msg.angle_increment, -45.0, +45.0, rot)
        self._d_left  = self._sector_min(msg.ranges, msg.angle_min,
                                         msg.angle_increment, +45.0, +135.0, rot)
        self._d_right = self._sector_min(msg.ranges, msg.angle_min,
                                         msg.angle_increment, -135.0, -45.0, rot)

    # ── IMU ───────────────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu) -> None:
        now = self.get_clock().now()
        if self._last_imu_t is not None and self._state == self._STOP:
            dt = (now - self._last_imu_t).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._turn_accum += abs(msg.angular_velocity.z) * dt
        self._last_imu_t = now

    # ── Otočenie ──────────────────────────────────────────────────────────────

    def _start_turn(self, direction: float, target_deg: float, reason: str) -> None:
        self._turn_dir    = direction
        self._turn_target = math.radians(target_deg)
        self._turn_accum  = 0.0
        self._turn_start  = self.get_clock().now()
        side = "vľavo" if direction > 0 else "vpravo"
        self.get_logger().info(
            f"{reason} → otáčam {side} o {target_deg:.0f}°  "
            f"(F={self._d_front:.2f}  L={self._d_left:.2f}  R={self._d_right:.2f})"
        )

    def _choose_turn(self, reason: str) -> None:
        thr        = self.get_parameter("threshold_m").get_parameter_value().double_value
        left_free  = self._d_left  > thr
        right_free = self._d_right > thr

        if left_free and not right_free:
            self._start_turn(+1.0, 90.0, reason)
        elif right_free and not left_free:
            self._start_turn(-1.0, 90.0, reason)
        elif left_free and right_free:
            if self._d_left >= self._d_right:
                self._start_turn(+1.0, 90.0, reason)
            else:
                self._start_turn(-1.0, 90.0, reason)
        else:
            self._start_turn(+1.0, 180.0, f"{reason} – zablokovaný")

    # ── Riadiaci cyklus ───────────────────────────────────────────────────────

    def _ctrl_cb(self) -> None:
        if not self.get_parameter("enabled").get_parameter_value().bool_value:
            self._pub.publish(Twist())
            self._state = self._FWD
            self._turn_start = None
            return

        thr  = self.get_parameter("threshold_m").get_parameter_value().double_value
        fwd  = self.get_parameter("forward_speed").get_parameter_value().double_value
        spd  = self.get_parameter("turn_speed").get_parameter_value().double_value
        tmax = self.get_parameter("turn_timeout_s").get_parameter_value().double_value

        cmd = Twist()

        if self._state == self._FWD:
            if self._d_front <= thr:
                self._state = self._STOP
                self._turn_start = None   # zatiaľ nevieme smer, vyberieme nižšie
                self.get_logger().info(
                    f"Prekážka {self._d_front:.2f} m → STOP"
                )
            else:
                # Motor má opačný smer – negujeme linear.x
                cmd.linear.x = -fwd

        if self._state == self._STOP:
            if self._turn_start is None:
                # Práve sme vstúpili – vyber smer a začni otáčanie
                self._choose_turn(f"Prekážka {self._d_front:.2f} m")
            else:
                # Prebieha otáčanie
                elapsed = (self.get_clock().now() - self._turn_start).nanoseconds * 1e-9
                min_rad = self._turn_target - _TOLERANCE_RAD   # cieľ − 15°
                done_angle   = self._turn_accum >= min_rad
                done_timeout = elapsed > tmax

                if done_angle or done_timeout:
                    reason = "uhol" if done_angle else "timeout"
                    self.get_logger().info(
                        f"Otočenie [{reason}]  "
                        f"{math.degrees(self._turn_accum):.1f}° / "
                        f"{math.degrees(self._turn_target):.0f}°  "
                        f"{elapsed:.1f}s → FWD"
                    )
                    self._state      = self._FWD
                    self._turn_start = None
                else:
                    # Motor má opačný smer otáčania – negujeme angular.z
                    cmd.angular.z = -(self._turn_dir * spd)

        self._pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = LidarWanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
