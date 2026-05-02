#!/usr/bin/env python3
# Jednoduchy wander: z /scan meria vzdialenosti v sektoroch (predok, lavo, pravo),
# pri prekazke zastavi, vyberie stranu otocenia a otoci sa podla integralu omega_z z /imu.
# Publikuje Twist na cmd_topic (typicky /cmd_vel) pre motor_hat_node v rezime auto.

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan


class LidarWanderNode(Node):
    # Stavovy automat: FWD jazda vpred, STOP rozhoduje o smere, TURN integruje uhol z IMU.
    _FWD  = 0
    _STOP = 1
    _TURN = 2

    _TURN_LEFT  = "LEFT"
    _TURN_RIGHT = "RIGHT"
    _TURN_FULL  = "FULL"  # obe strany zablokovane -> velky uhol (napr. 180 deg)

    def __init__(self) -> None:
        super().__init__("lidar_wander_node")

        # enabled: v launch false v manual rezime (motor berie len teleop)
        self.declare_parameter("enabled",             True)
        # threshold_m: ak predok blizsie -> prechod do STOP a vyber otocenia
        self.declare_parameter("threshold_m",         0.30)
        self.declare_parameter("forward_speed",       0.10)
        self.declare_parameter("turn_speed",          1.80)
        self.declare_parameter("turn_timeout_s",       6.0)
        self.declare_parameter("turn_target_deg",        90.0)
        self.declare_parameter("turn_tolerance_deg",      2.0)
        self.declare_parameter("turn_blocked_deg",    180.0)
        # lidar_rotation_deg: ak je lidar natoceny voci base_link, posun uhlov sektorov
        self.declare_parameter("lidar_rotation_deg", -90.0)
        self.declare_parameter("cmd_topic",       "/cmd_vel")
        # imu_angular_z_sign: doladenie znamienka gyroskopu voci skutocnemu otacaniu robota
        self.declare_parameter("imu_angular_z_sign", -1.0)
        # lidar_early_exit: ak sa pocas otacenia predok otvori, mozno skoncit skor (bez cakania na uhol)
        self.declare_parameter("lidar_early_exit",          False)
        self.declare_parameter("lidar_early_exit_min_deg",   40.0)
        self.declare_parameter("imu_integration_scale",       1.0)
        # turn_ramp: pri konci otacenia zmensi angular.z (plynulejsi dojazd na cielovy uhol)
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
        self._turn_mode:   str   = self._TURN_LEFT
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
        # Najblizsi platny bod v uhlovom pasme v suradniciach robota (rotation_rad = offset lidaru).
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
        # Po case zaciatku otacenia integruj omega*dt do _turn_accum = natocenie od startu (rad).
        now = self.get_clock().now()
        if (
            self._last_imu_t is not None
            and self._state == self._TURN
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

    def _start_turn(self, direction: float, target_deg: float, reason: str, mode: str) -> None:
        # direction +1 = dolava (smer musi sediet so znamienkom cmd.angular.z a integraciou v _imu_cb).
        self._turn_dir    = direction
        self._turn_target = math.radians(target_deg)
        self._turn_accum  = 0.0
        self._turn_mode   = mode
        self._turn_start  = self.get_clock().now()
        self._last_imu_t = None
        self._state = self._TURN
        side = "vlavo" if direction > 0 else "vpravo"
        self.get_logger().info(
            f"{reason} -> otacam {side} o {target_deg:.0f} deg "
            f"[{self._turn_mode}] "
            f"(F={self._d_front:.2f} L={self._d_left:.2f} R={self._d_right:.2f})"
        )

    def _choose_turn(self, reason: str) -> None:
        # Vyber volnejsiu stranu podla sektorov L/R; ak obe zablokovane, velky "FULL" uhol.
        thr        = self.get_parameter("threshold_m").get_parameter_value().double_value
        tgt        = self.get_parameter("turn_target_deg").get_parameter_value().double_value
        blk        = self.get_parameter("turn_blocked_deg").get_parameter_value().double_value
        left_free  = self._d_left  > thr
        right_free = self._d_right > thr

        if left_free and not right_free:
            self._start_turn(+1.0, tgt, reason, self._TURN_LEFT)
        elif right_free and not left_free:
            self._start_turn(-1.0, tgt, reason, self._TURN_RIGHT)
        elif left_free and right_free:
            if self._d_left >= self._d_right:
                self._start_turn(+1.0, tgt, reason, self._TURN_LEFT)
            else:
                self._start_turn(-1.0, tgt, reason, self._TURN_RIGHT)
        else:
            self._start_turn(+1.0, blk, f"{reason} - zablokovany", self._TURN_FULL)

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
                cmd.linear.x = fwd

        elif self._state == self._STOP:
            if self._turn_start is None:
                self._choose_turn(f"Prekazka {self._d_front:.2f} m")

        elif self._state == self._TURN and self._turn_start is not None:
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
                    f"Otocenie [{self._turn_mode}/{reason}] "
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
                # Znamienko musi sediet s integraciou v _imu_cb (otacanie v zvolenom smere).
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
