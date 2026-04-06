#!/usr/bin/env python3
"""Dead reckoning: /cmd_vel + /imu -> /odom + TF odom->base_link.

Yaw z IMU gyro Z; poloha (x,y) z cmd_vel linear.x (bez enkoderov).
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    h = yaw * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)


class CmdVelOdometry(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_odometry")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        rate = float(self.get_parameter("publish_rate").value)
        self._timeout = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._prev_tick = self.get_clock().now()

        self._imu_wz = 0.0
        self._imu_available = False

        self._pub = self.create_publisher(Odometry, "odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)

        period = 1.0 / max(rate, 1.0)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"cmd_vel_odometry: odom={self._odom_frame} base={self._base_frame} "
            f"rate={rate:.0f} Hz  yaw=IMU gyro (fallback: cmd_vel)"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _on_imu(self, msg: Imu) -> None:
        self._imu_wz = msg.angular_velocity.z
        self._imu_available = True

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._prev_tick).nanoseconds / 1e9
        self._prev_tick = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / 50.0

        stale = (now - self._last_cmd_time).nanoseconds / 1e9 > self._timeout
        v = 0.0 if stale else float(self._last_cmd.linear.x)

        if self._imu_available:
            w = self._imu_wz
        else:
            w = 0.0 if stale else float(self._last_cmd.angular.z)

        self._yaw += w * dt
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt

        qx, qy, qz, qw = yaw_to_quat(self._yaw)
        stamp = now.to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self._odom_frame
        o.child_frame_id = self._base_frame
        o.pose.pose.position.x = self._x
        o.pose.pose.position.y = self._y
        o.pose.pose.position.z = 0.0
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        self._pub.publish(o)


def main() -> None:
    rclpy.init()
    node = CmdVelOdometry()
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
