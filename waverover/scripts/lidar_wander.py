#!/usr/bin/env python3
# Jednoduche bludenie: z /scan vezme vzdialenosti v sektoroch, pri prekazke zastavi a otoci sa.
# Uhol otocenia pocita z IMU (integracia omega_z), nie z encodera.
# Vypnut: ros2 param set /lidar_wander_node enabled false

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

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
        # Cielovy uhol otocenia (IMU integracia omega_z); zablokovany = obe strany pod threshold
        self.declare_parameter("turn_target_deg",        90.0)
        # Ukonci otacanie ked integral uhla >= turn_target_deg - turn_tolerance_deg (mensia = blizsie k plnemu uhlu)
        self.declare_parameter("turn_tolerance_deg",      2.0)
        self.declare_parameter("turn_blocked_deg",    120.0)
        self.declare_parameter("lidar_rotation_deg", -90.0)
        self.declare_parameter("cmd_topic",       "/cmd_vel")
        self.declare_parameter("imu_angular_z_sign", -1.0)
        # Ak predok LiDARu ukaze volno pocas otacania, ukonci skor (menej zavisle od gyro integracie)
        self.declare_parameter("lidar_early_exit",          False)
        self.declare_parameter("lidar_early_exit_min_deg",   40.0)
        # Nasobenie omega pred integraciou (1.0 = default); >1 zrychli narast integrala (skorsi koniec otacania)
        self.declare_parameter("imu_integration_scale",       1.0)
        # Poslednych turn_ramp_deg pred cielom: linearne znizuje |angular.z| az po turn_ramp_min_scale * turn_speed; 0 = vypnute
        self.declare_parameter("turn_ramp_deg", 28.0)
        self.declare_parameter("turn_ramp_min_scale",         0.22)

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
        self._imu_sign = self.get_parameter(
            "imu_angular_z_sign"
        ).get_parameter_value().double_value

        self._d_front: float = math.inf
        self._d_left:  float = math.inf
        self._d_right: float = math.inf

        self.create_timer(0.1, self._ctrl_cb)
        self.get_logger().info(
            f"LidarWander -> {cmd_topic} "
            f"[threshold={self.get_parameter('threshold_m').get_parameter_value().double_value:.2f} m]"
        )

    def _sector_min(self, ranges, angle_min, angle_inc,
                    lo_deg, hi_deg, rotation_rad) -> float:
        # Najde minimalnu vzdialenost v useku uhlov (v ramci robota po odcitani rotacie lidaru)
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

    def _imu_cb(self, msg: Imu) -> None:
        now = self.get_clock().now()
        if (
            self._last_imu_t is not None
            and self._state == self._STOP
            and self._turn_start is not None
        ):
            dt = (now - self._last_imu_t).nanoseconds * 1e-9
            if 0.0 < dt < 0.5:
                scale = self.get_parameter(
                    "imu_integration_scale"
                ).get_parameter_value().double_value
                omega = msg.angular_velocity.z * self._imu_sign * scale
                self._turn_accum += omega * dt
        self._last_imu_t = now

    def _start_turn(self, direction: float, target_deg: float, reason: str) -> None:
        self._turn_dir    = direction
        self._turn_target = math.radians(target_deg)
        self._turn_accum  = 0.0
        self._turn_start  = self.get_clock().now()
        self._last_imu_t = None
        side = "vlavo" if direction > 0 else "vpravo"
        self.get_logger().info(
            f"{reason} -> otacam {side} o {target_deg:.0f} deg "
            f"(F={self._d_front:.2f} L={self._d_left:.2f} R={self._d_right:.2f})"
        )

    def _choose_turn(self, reason: str) -> None:
        thr        = self.get_parameter("threshold_m").get_parameter_value().double_value
        tgt        = self.get_parameter("turn_target_deg").get_parameter_value().double_value
        blk        = self.get_parameter("turn_blocked_deg").get_parameter_value().double_value
        left_free  = self._d_left  > thr
        right_free = self._d_right > thr

        if left_free and not right_free:
            self._start_turn(+1.0, tgt, reason)
        elif right_free and not left_free:
            self._start_turn(-1.0, tgt, reason)
        elif left_free and right_free:
            if self._d_left >= self._d_right:
                self._start_turn(+1.0, tgt, reason)
            else:
                self._start_turn(-1.0, tgt, reason)
        else:
            self._start_turn(+1.0, blk, f"{reason} - zablokovany")

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
                self._turn_start = None
                self.get_logger().info(f"Prekazka {self._d_front:.2f} m -> STOP")
            else:
                # Zaporna linear.x: u tohto podvozku a drive_node znamena jazdu dopredu (zhoda s cmd_vel_invert)
                cmd.linear.x = -fwd

        if self._state == self._STOP:
            if self._turn_start is None:
                self._choose_turn(f"Prekazka {self._d_front:.2f} m")
            else:
                elapsed = (self.get_clock().now() - self._turn_start).nanoseconds * 1e-9
                progress = abs(self._turn_accum)
                tol = math.radians(
                    self.get_parameter("turn_tolerance_deg").get_parameter_value().double_value
                )
                min_ok = self._turn_target - tol
                done_angle = progress >= min_ok
                done_timeout = elapsed > tmax
                early_exit = self.get_parameter("lidar_early_exit").get_parameter_value().bool_value
                early_min = math.radians(
                    self.get_parameter("lidar_early_exit_min_deg").get_parameter_value().double_value
                )
                lidar_clear = early_exit and self._d_front > thr and progress >= early_min

                if lidar_clear or done_angle or done_timeout:
                    if lidar_clear:
                        reason = "lidar_predok"
                    elif done_angle:
                        reason = "uhol"
                    else:
                        reason = "timeout"
                    self.get_logger().info(
                        f"Otocenie [{reason}] "
                        f"{math.degrees(abs(self._turn_accum)):.1f} deg / "
                        f"{math.degrees(self._turn_target):.0f} deg "
                        f"{elapsed:.1f}s -> FWD"
                    )
                    self._state      = self._FWD
                    self._turn_start = None
                else:
                    remaining = max(0.0, min_ok - progress)
                    ramp_deg = self.get_parameter("turn_ramp_deg").get_parameter_value().double_value
                    ramp_rad = math.radians(ramp_deg) if ramp_deg > 0.0 else 0.0
                    if ramp_rad <= 0.0:
                        turn_scale = 1.0
                    elif remaining >= ramp_rad:
                        turn_scale = 1.0
                    else:
                        lo = max(0.01, min(1.0, self.get_parameter(
                            "turn_ramp_min_scale"
                        ).get_parameter_value().double_value))
                        turn_scale = max(lo, remaining / ramp_rad)
                    cmd.angular.z = -(self._turn_dir * spd * turn_scale)

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
