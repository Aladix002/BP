#!/usr/bin/env python3
"""Autonómne bludenie – 4 LiDAR kvadranty, prah 30 cm.

LiDAR je rozdelený na 4 kvadranty (v robot frame):
  Front:  −45° …  +45°
  Left:   +45° … +135°
  Back:  +135° … +225°  (alebo −135° … −135°)
  Right: −135° …  −45°

Logika:
  1. Jazdi dopredu.
  2. Ak Front ≤ threshold → zastav, porovnaj Left vs Right.
     • Left voľný (> threshold) → otáčaj vľavo o 90°
     • Right voľný (> threshold) → otáčaj vpravo o 90°
     • Obe zablokované → otáčaj vpravo o 180°
  3. Po otočení → späť na krok 1.

Dynamická rekonf.:
  ros2 param set /lidar_wander_node enabled false
  ros2 param set /lidar_wander_node threshold_m 0.30
  ros2 param set /lidar_wander_node lidar_rotation_deg 180.0
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class LidarWanderNode(Node):
    _FWD = 0
    _TRN = 1

    def __init__(self) -> None:
        super().__init__("lidar_wander_node")

        self.declare_parameter("enabled",             True)
        self.declare_parameter("threshold_m",         0.30)   # prah pre všetky kvadranty [m]
        self.declare_parameter("forward_speed",       0.10)   # rýchlosť dopredu [m/s]
        self.declare_parameter("turn_speed",          1.00)   # uhlová rýchlosť [rad/s]
        self.declare_parameter("turn_timeout_s",       6.0)   # záchranný timeout
        self.declare_parameter("lidar_rotation_deg",   0.0)   # montážna rotácia LiDARu [°]
        self.declare_parameter("cmd_topic",        "/cmd_vel")

        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value

        self._pub      = self.create_publisher(Twist, cmd_topic, 10)
        self._sub_scan = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, rclpy.qos.qos_profile_sensor_data
        )
        self._sub_imu  = self.create_subscription(
            Imu, "/imu", self._imu_cb, rclpy.qos.qos_profile_sensor_data
        )

        self._state:      int   = self._FWD
        self._turn_dir:   float = 1.0
        self._turn_target: float = math.radians(90.0)
        self._turn_accum: float = 0.0
        self._turn_start        = None
        self._last_imu_t        = None

        # Minimálne vzdialenosti pre každý kvadrant
        self._d_front: float = math.inf
        self._d_left:  float = math.inf
        self._d_right: float = math.inf

        self.create_timer(0.1, self._ctrl_cb)
        self.get_logger().info(
            f"LidarWander → {cmd_topic}  "
            f"[threshold={self.get_parameter('threshold_m').get_parameter_value().double_value:.2f} m  "
            f"lidar_rotation={self.get_parameter('lidar_rotation_deg').get_parameter_value().double_value:.0f}°]"
        )

    # ── LiDAR ─────────────────────────────────────────────────────────────────

    def _sector_min(
        self,
        ranges: list,
        angle_min: float,
        angle_inc: float,
        lo_deg: float,
        hi_deg: float,
        rotation_rad: float,
    ) -> float:
        lo   = math.radians(lo_deg)
        hi   = math.radians(hi_deg)
        best = math.inf
        for i, d in enumerate(ranges):
            if not math.isfinite(d) or d < 0.02:
                continue
            a_lidar = angle_min + i * angle_inc
            a_robot = math.atan2(
                math.sin(a_lidar + rotation_rad),
                math.cos(a_lidar + rotation_rad),
            )
            if lo <= a_robot <= hi:
                best = min(best, d)
        return best

    def _scan_cb(self, msg: LaserScan) -> None:
        rot = math.radians(
            self.get_parameter("lidar_rotation_deg").get_parameter_value().double_value
        )
        self._d_front = self._sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -45.0,  +45.0, rot,
        )
        self._d_left  = self._sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            +45.0, +135.0, rot,
        )
        self._d_right = self._sector_min(
            msg.ranges, msg.angle_min, msg.angle_increment,
            -135.0, -45.0, rot,
        )

    # ── IMU ───────────────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu) -> None:
        now = self.get_clock().now()
        if self._last_imu_t is not None and self._state == self._TRN:
            dt = (now - self._last_imu_t).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                self._turn_accum += abs(msg.angular_velocity.z) * dt
        self._last_imu_t = now

    # ── Riadiaci cyklus ───────────────────────────────────────────────────────

    def _start_turn(self, direction: float, target_deg: float, reason: str) -> None:
        self._turn_dir    = direction
        self._turn_target = math.radians(target_deg)
        self._turn_accum  = 0.0
        self._turn_start  = self.get_clock().now()
        self._state       = self._TRN
        side = "vľavo" if direction > 0 else "vpravo"
        self.get_logger().info(
            f"{reason} → otáčam {side} o {target_deg:.0f}°  "
            f"(F={self._d_front:.2f}  L={self._d_left:.2f}  R={self._d_right:.2f})"
        )

    def _ctrl_cb(self) -> None:
        if not self.get_parameter("enabled").get_parameter_value().bool_value:
            self._pub.publish(Twist())
            self._state = self._FWD
            return

        thr  = self.get_parameter("threshold_m").get_parameter_value().double_value
        fwd  = self.get_parameter("forward_speed").get_parameter_value().double_value
        spd  = self.get_parameter("turn_speed").get_parameter_value().double_value
        tmax = self.get_parameter("turn_timeout_s").get_parameter_value().double_value

        cmd = Twist()

        if self._state == self._FWD:
            if self._d_front <= thr:
                left_free  = self._d_left  > thr
                right_free = self._d_right > thr

                if left_free and not right_free:
                    self._start_turn(+1.0, 90.0, f"Prekážka vpredu {self._d_front:.2f} m")
                elif right_free and not left_free:
                    self._start_turn(-1.0, 90.0, f"Prekážka vpredu {self._d_front:.2f} m")
                elif left_free and right_free:
                    # Obe voľné → otoč sa na stranu s väčšou voľnosťou
                    if self._d_left >= self._d_right:
                        self._start_turn(+1.0, 90.0, f"Prekážka vpredu {self._d_front:.2f} m")
                    else:
                        self._start_turn(-1.0, 90.0, f"Prekážka vpredu {self._d_front:.2f} m")
                else:
                    # Obe zablokované → 180°
                    self._start_turn(+1.0, 180.0, f"Zablokované zo všetkých strán")
            else:
                cmd.linear.x = fwd

        if self._state == self._TRN:
            elapsed      = (self.get_clock().now() - self._turn_start).nanoseconds * 1e-9
            done_angle   = self._turn_accum >= self._turn_target
            done_timeout = elapsed > tmax
            if done_angle or done_timeout:
                reason = "uhol" if done_angle else "timeout"
                self.get_logger().info(
                    f"Otočenie [{reason}]  {math.degrees(self._turn_accum):.1f}°  {elapsed:.1f}s"
                )
                self._state = self._FWD
            else:
                cmd.angular.z = self._turn_dir * spd

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
