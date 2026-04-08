#!/usr/bin/env python3
# Fiktivna odometria: integruje /teleop_cmd_vel do x,y,yaw a publikuje /odom + TF odom->base_link.
# Bez enkoderov je to len vizualizacia v RViz, nie pravdiva poloha.

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class CmdVelOdom(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_odom")
        self.declare_parameter("cmd_topic", "/teleop_cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("timeout_sec", 0.4)
        self.declare_parameter("update_rate_hz", 30.0)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))

        self.latest_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.sub = self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 20)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.tick)
        self.get_logger().info(f"cmd_vel_odom: {self.cmd_topic} -> {self.odom_topic}")

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age > self.timeout_sec:
            # Ak neprisiel novy prikaz (Wi-Fi lag), netlacime stare velocity do integracie
            vx = 0.0
            wz = 0.0
        else:
            vx = float(self.latest_cmd.linear.x)
            wz = float(self.latest_cmd.angular.z)

        # Jednoduchy unicycle model v 2D (bez slipu kolies)
        self.yaw += wz * dt
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt

        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
