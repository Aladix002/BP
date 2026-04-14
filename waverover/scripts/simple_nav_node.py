#!/usr/bin/env python3
# simple_nav_node: otoc sa podla IMU smerom k cielu (v map_frame), potom jazdi vpred.
# Pozicia a vzdialenost k cielu: TF map->base_link (SLAM).
# ROTATING: dorovnaj heading podla IMU; hotovo pri ~+-10 deg (netreba presne na stupen).
# DRIVING: pri malom kurze ide rovno (deadband), inak jemna korekcia z map TF.
# Topics: sub /goal_pose, /imu; pub /cmd_vel, /nav_status

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def imu_msg_yaw_rad(msg: Imu) -> float:
    # Yaw z ROS Imu orientation (rovnaky postup ako cmd_vel_odom)
    q = msg.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return -math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SimpleNavNode(Node):
    IDLE = "IDLE"
    ROTATING = "ROTATING"
    DRIVING = "DRIVING"
    DONE = "DONE"

    def __init__(self):
        super().__init__("simple_nav_node")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_topic", "/imu")

        # Musi sediet s motorom (teleop_max_angular). Rychlost otacania = teleop_max_angular * rotate_angular_scale
        # (napr. 2.0 * 0.8 = 1.6 rad/s ako 80 % „angular“ slidera v manuale).
        self.declare_parameter("teleop_max_angular", 2.0)
        self.declare_parameter("rotate_angular_scale", 0.8)
        self.declare_parameter("rotate_angular_sign", -1.0)   # 1.0 alebo -1.0 – smer otacania
        # Ak > 0, pevny strop [rad/s] namiesto vypoctu z teleop * scale
        self.declare_parameter("rotate_speed_max_override", -1.0)
        self.declare_parameter("rotate_kp", 4.5)
        # Hotovo otacanie (IMU): |chyba| < tol [deg] (~+-10 deg = netreba presne na stupen)
        self.declare_parameter("rotate_done_deg", 10.0)
        # Nad tymto uhlom plny |omega| (nie pomaly P) – uzsie = rychlejsie otacanie celkovo
        self.declare_parameter("rotate_fast_deg", 12.0)
        # Min |omega| pri P-faze; aspon cast max_om aby kolesa netocili prilis pomaly
        self.declare_parameter("rotate_omega_min", 1.0)
        # linear.x pri ROTATING: default 0 (cista otacka na mieste; odom/RViz inak ukazuju jazdu vpred).
        # Ak koleso len bzuci, skus zvysit angular / deadzone motora, nie nudge.
        self.declare_parameter("rotate_linear_nudge", 0.0)
        self.declare_parameter("rotate_done_streak", 2)
        self.declare_parameter("rotate_stuck_sec", 22.0)
        self.declare_parameter("rotate_stuck_max_deg", 22.0)

        self.declare_parameter("drive_speed", 0.75)
        self.declare_parameter("drive_forward_scale", 1.5)
        self.declare_parameter("drive_steer_kp", 0.7)
        self.declare_parameter("drive_steer_max", 0.25)
        # Pod tymto uhlom voci cielu ide rovno (angular.z=0), len vpred
        self.declare_parameter("drive_steer_deadband_deg", 10.0)
        self.declare_parameter("rerotate_threshold_rad", 0.52)
        self.declare_parameter("goal_tolerance_m", 0.35)
        self.declare_parameter("approach_slowdown_m", 0.60)

        self.declare_parameter("loop_hz", 20.0)
        self.declare_parameter("tf_timeout_sec", 0.15)

        self.tf_buf = Buffer()
        self._tf_listener = TransformListener(self.tf_buf, self)

        self._imu_yaw: float | None = None
        imu_topic = self.get_parameter("imu_topic").value
        self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)

        self.create_subscription(PoseStamped, "/goal_pose", self._goal_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_status = self.create_publisher(String, "/nav_status", 10)

        self.state = self.IDLE
        self.goal_x = 0.0
        self.goal_y = 0.0
        self._rotate_ok_streak = 0
        self._rotate_t0 = None
        # imu_yaw + _imu_to_map_yaw == map yaw (v okamihu sync pri vstupe do ROTATING)
        self._imu_to_map_yaw = 0.0
        self._have_imu_sync = False

        hz = self.get_parameter("loop_hz").value
        self.create_timer(1.0 / max(1.0, float(hz)), self._loop)

        self.get_logger().info(
            f"simple_nav: IMU otacanie ({imu_topic}), jazda podla map TF; ciel /goal_pose"
        )

    def _imu_cb(self, msg: Imu) -> None:
        self._imu_yaw = imu_msg_yaw_rad(msg)

    def _goal_cb(self, msg: PoseStamped):
        mf = self.get_parameter("map_frame").value
        gx, gy = self._goal_xy_in_map(msg, mf)
        self.goal_x = gx
        self.goal_y = gy
        self.state = self.ROTATING
        self._rotate_ok_streak = 0
        self._rotate_t0 = self.get_clock().now()
        self._have_imu_sync = False
        self.get_logger().info(
            f"Novy ciel map=({self.goal_x:.2f}, {self.goal_y:.2f}) m → ROTATING (IMU kurz)"
        )

    def _goal_xy_in_map(self, msg: PoseStamped, map_frame: str):
        fid = (msg.header.frame_id or "").strip()
        if not fid or fid == map_frame:
            return msg.pose.position.x, msg.pose.position.y
        try:
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                when = Time()
            else:
                when = Time.from_msg(msg.header.stamp)
            t = self.tf_buf.lookup_transform(
                map_frame, fid, when, timeout=Duration(seconds=1.0))
            out = do_transform_pose(msg, t)
            return out.pose.position.x, out.pose.position.y
        except Exception as e:
            self.get_logger().error(
                f"TF {fid!r} -> {map_frame!r}: {e}; pouzivam surove suradnice z spravy."
            )
            return msg.pose.position.x, msg.pose.position.y

    def _get_pose(self):
        mf = self.get_parameter("map_frame").value
        bf = self.get_parameter("base_frame").value
        t = float(self.get_parameter("tf_timeout_sec").value)
        try:
            tf = self.tf_buf.lookup_transform(mf, bf, Time(), timeout=Duration(seconds=t))
            p = tf.transform.translation
            q = tf.transform.rotation
            return p.x, p.y, yaw_from_quat(q)
        except Exception:
            return None

    def _yaw_from_imu_map(self) -> float | None:
        if self._imu_yaw is None:
            return None
        return math.atan2(
            math.sin(self._imu_yaw + self._imu_to_map_yaw),
            math.cos(self._imu_yaw + self._imu_to_map_yaw),
        )

    def _stop(self):
        self.pub_cmd.publish(Twist())

    def _loop(self):
        if self.state in (self.IDLE, self.DONE):
            self._publish_status(0.0, 0.0)
            return

        pose = self._get_pose()
        if pose is None:
            self.get_logger().warn(
                "TF map->base_link nedostupny – cakam na SLAM...",
                throttle_duration_sec=3.0,
            )
            return

        rx, ry, ryaw_map = pose
        dx = self.goal_x - rx
        dy = self.goal_y - ry
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if dist > 1e-9 else 0.0

        goal_tol = float(self.get_parameter("goal_tolerance_m").value)
        if dist < goal_tol:
            self._stop()
            self.state = self.DONE
            self.get_logger().info(f"Ciel dosiahnuty dist={dist:.3f} m → DONE")
            self._publish_status(dist, 0.0)
            return

        if self.state == self.ROTATING:
            if not self._have_imu_sync:
                if self._imu_yaw is None:
                    self.get_logger().warn(
                        "Cakam na /imu pre otacanie podla IMU...",
                        throttle_duration_sec=2.0,
                    )
                    return
                self._imu_to_map_yaw = angle_diff(ryaw_map, self._imu_yaw)
                self._have_imu_sync = True
                self.get_logger().info(
                    "IMU↔map yaw sync: otacam podla IMU smerom k cielu."
                )

            yaw_imu_map = self._yaw_from_imu_map()
            if yaw_imu_map is None:
                return
            h_err = angle_diff(bearing, yaw_imu_map)

            kp = float(self.get_parameter("rotate_kp").value)
            max_a = max(0.1, float(self.get_parameter("teleop_max_angular").value))
            scale = max(0.05, min(1.0, float(self.get_parameter("rotate_angular_scale").value)))
            ov = float(self.get_parameter("rotate_speed_max_override").value)
            if ov > 1e-6:
                max_om = ov
            else:
                max_om = max_a * scale
            omega_min = max(
                float(self.get_parameter("rotate_omega_min").value),
                0.45 * max_om,
            )
            done_rad = math.radians(float(self.get_parameter("rotate_done_deg").value))
            fast_rad = math.radians(float(self.get_parameter("rotate_fast_deg").value))
            need_streak = max(1, int(self.get_parameter("rotate_done_streak").value))
            stuck_sec = float(self.get_parameter("rotate_stuck_sec").value)
            stuck_deg = float(self.get_parameter("rotate_stuck_max_deg").value)
            stuck_rad = math.radians(stuck_deg)

            if abs(h_err) > fast_rad:
                omega = math.copysign(max_om, h_err)
            else:
                omega = clamp(kp * h_err, -max_om, max_om)
                if abs(h_err) >= done_rad and abs(omega) < omega_min:
                    omega = math.copysign(omega_min, h_err)

            rot_sign = float(self.get_parameter("rotate_angular_sign").value)
            nudge = max(0.0, float(self.get_parameter("rotate_linear_nudge").value))
            cmd = Twist()
            cmd.linear.x = nudge
            cmd.angular.z = rot_sign * omega
            self.pub_cmd.publish(cmd)

            force_drive = False
            if stuck_sec > 0.5 and self._rotate_t0 is not None and abs(h_err) < stuck_rad:
                elapsed = (self.get_clock().now() - self._rotate_t0).nanoseconds * 1e-9
                if elapsed >= stuck_sec:
                    force_drive = True
                    self.get_logger().warn(
                        f"ROTATING {elapsed:.0f}s, |err|={math.degrees(abs(h_err)):.1f}° → DRIVING"
                    )

            if abs(h_err) < done_rad:
                self._rotate_ok_streak += 1
            else:
                self._rotate_ok_streak = 0

            if force_drive or self._rotate_ok_streak >= need_streak:
                self._rotate_ok_streak = 0
                self.state = self.DRIVING
                self.get_logger().info(
                    f"Otoceny (IMU/map err={math.degrees(h_err):.1f}°) dist={dist:.2f} m → DRIVING"
                )

            self._publish_status(dist, h_err)
            return

        # DRIVING – kurz z map TF (scan / SLAM)
        if self.state == self.DRIVING:
            h_err = angle_diff(bearing, ryaw_map)
            rerot = float(self.get_parameter("rerotate_threshold_rad").value)

            if abs(h_err) > rerot:
                self._stop()
                self.state = self.ROTATING
                self._rotate_ok_streak = 0
                self._rotate_t0 = self.get_clock().now()
                self._have_imu_sync = False
                self.get_logger().info(
                    f"Kurz {math.degrees(h_err):.1f}° > {math.degrees(rerot):.0f}° → ROTATING"
                )
                self._publish_status(dist, h_err)
                return

            spd = float(self.get_parameter("drive_speed").value)
            kp_st = float(self.get_parameter("drive_steer_kp").value)
            max_st = float(self.get_parameter("drive_steer_max").value)
            slow_m = float(self.get_parameter("approach_slowdown_m").value)
            if slow_m > 0.05 and dist < slow_m:
                spd *= max(dist / slow_m, 0.12)

            fwd_scale = max(0.0, float(self.get_parameter("drive_forward_scale").value))
            lin_x = min(1.0, spd * fwd_scale)

            deadband = math.radians(float(self.get_parameter("drive_steer_deadband_deg").value))
            if abs(h_err) < deadband:
                omega = 0.0
            else:
                omega = clamp(kp_st * h_err, -max_st, max_st)

            cmd = Twist()
            cmd.linear.x = lin_x
            cmd.angular.z = omega
            self.pub_cmd.publish(cmd)

        self._publish_status(dist, angle_diff(bearing, ryaw_map))

    def _publish_status(self, dist: float, herr: float):
        msg = String()
        msg.data = (
            f"{self.state} dist={dist:.2f}m heading_err={math.degrees(herr):.1f}deg "
            f"goal=({self.goal_x:.2f},{self.goal_y:.2f})"
        )
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
