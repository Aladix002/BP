#!/usr/bin/env python3
# Fiktivna odometria: integruje cmd_vel do x,y,yaw a publikuje /odom + TF odom->base_link.
# Pocuva na oba zdroje naraz:
#   cmd_topic      (/teleop_cmd_vel) - manual/web, linear.x kladne = dopredu
#   cmd_topic_auto (/cmd_vel)        - wander/auto, linear.x zaporne = dopredu (motor invertuje)
# Pouzije zdroj, ktory mal posledny nenulovy prikaz (v ramci timeout).
# use_imu_yaw=true: yaw z IMU kvaterniona namiesto integracie angular.z.

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class CmdVelOdom(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_odom")
        self.declare_parameter("cmd_topic",      "/teleop_cmd_vel")
        self.declare_parameter("cmd_topic_auto", "/cmd_vel")
        self.declare_parameter("odom_topic",     "/odom")
        self.declare_parameter("odom_frame",     "odom")
        self.declare_parameter("base_frame",     "base_link")
        self.declare_parameter("publish_tf",     True)
        self.declare_parameter("timeout_sec",    0.4)
        self.declare_parameter("update_rate_hz", 30.0)
        self.declare_parameter("use_imu_yaw",    False)
        self.declare_parameter("imu_topic",      "/imu")
        self.declare_parameter("linear_scale",   1.0)

        cmd_topic      = self.get_parameter("cmd_topic").value
        cmd_topic_auto = self.get_parameter("cmd_topic_auto").value
        self.odom_topic       = self.get_parameter("odom_topic").value
        self.odom_frame       = self.get_parameter("odom_frame").value
        self.base_frame       = self.get_parameter("base_frame").value
        self.publish_tf       = bool(self.get_parameter("publish_tf").value)
        self.timeout_sec      = float(self.get_parameter("timeout_sec").value)
        self.update_rate_hz   = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.use_imu_yaw      = bool(self.get_parameter("use_imu_yaw").value)
        imu_topic             = self.get_parameter("imu_topic").value
        self._linear_scale    = float(self.get_parameter("linear_scale").value)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self._imu_yaw: float | None = None

        # Manual zdroj (teleop / web): linear.x kladne = dopredu
        self._cmd_manual      = Twist()
        self._time_manual     = self.get_clock().now()
        self._manual_nonzero  = False  # ci posledny manual prikaz bol nenulovy

        # Auto zdroj (wander): linear.x zaporne = dopredu (invertujeme)
        self._cmd_auto        = Twist()
        self._time_auto       = self.get_clock().now()
        self._auto_nonzero    = False

        self.last_update_time = self.get_clock().now()

        self.create_subscription(Twist, cmd_topic,      self._cb_manual, 20)
        self.create_subscription(Twist, cmd_topic_auto, self._cb_auto,   20)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        if self.use_imu_yaw:
            self.create_subscription(Imu, imu_topic, self._imu_cb, 20)
            self.get_logger().info(f"cmd_vel_odom: yaw z IMU ({imu_topic})")

        self.create_timer(1.0 / self.update_rate_hz, self.tick)
        self.get_logger().info(
            f"cmd_vel_odom: manual={cmd_topic} auto={cmd_topic_auto} -> {self.odom_topic}"
        )

    def _cb_manual(self, msg: Twist) -> None:
        self._cmd_manual  = msg
        self._time_manual = self.get_clock().now()
        self._manual_nonzero = abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4

    def _cb_auto(self, msg: Twist) -> None:
        self._cmd_auto  = msg
        self._time_auto = self.get_clock().now()
        self._auto_nonzero = abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4

    def _imu_cb(self, msg: Imu) -> None:
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._imu_yaw = -math.atan2(siny_cosp, cosy_cosp)

    def tick(self) -> None:
        now = self.get_clock().now()
        dt  = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        age_manual = (now - self._time_manual).nanoseconds * 1e-9
        age_auto   = (now - self._time_auto).nanoseconds * 1e-9
        fresh_manual = age_manual <= self.timeout_sec
        fresh_auto   = age_auto   <= self.timeout_sec

        # Prednost: auto zdroj ak ma cerstvy NENULOVY prikaz (wander bezi)
        # inak pouzijeme manual (teleop / web)
        if fresh_auto and self._auto_nonzero:
            # lidar_wander posiela zaporne linear.x pre pohyb dopredu -> invertujeme
            vx = -self._linear_scale * float(self._cmd_auto.linear.x)
            wz =  float(self._cmd_auto.angular.z)
        elif fresh_manual:
            vx = self._linear_scale * float(self._cmd_manual.linear.x)
            wz = float(self._cmd_manual.angular.z)
        else:
            vx = 0.0
            wz = 0.0

        if self.use_imu_yaw and self._imu_yaw is not None:
            self.yaw = self._imu_yaw
        else:
            self.yaw += wz * dt

        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp        = now.to_msg()
        odom.header.frame_id     = self.odom_frame
        odom.child_frame_id      = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp    = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = CmdVelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
