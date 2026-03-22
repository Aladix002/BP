#!/usr/bin/env python3
"""
Mode manager node – switches between MANUAL and AUTONOMOUS control.

Subscribes:
  /mode   (std_msgs/String)   – "manual" | "auto"

Publishes:
  /current_mode   (std_msgs/String)   – active mode (latched)
  /cmd_vel        (geometry_msgs/Twist) – gated output to motor controller
  cmd_vel_nav    (geometry_msgs/Twist) – incoming Nav2 command (rebridged)
  cmd_vel_teleop (geometry_msgs/Twist) – incoming teleop command (rebridged)

The node acts as a software mux:
  MANUAL mode → forward /cmd_vel_teleop → /cmd_vel, block /cmd_vel_nav
  AUTO   mode → forward /cmd_vel_nav    → /cmd_vel, block /cmd_vel_teleop

Keyboard shortcut: publish "toggle" to /mode to flip the current mode.

Parameters:
  initial_mode  [string]  default "manual"
  teleop_topic  [string]  default "cmd_vel_teleop"
  nav_topic     [string]  default "cmd_vel_nav"
  output_topic  [string]  default "cmd_vel"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import String


_LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeManagerNode(Node):

    _MANUAL = 'manual'
    _AUTO   = 'auto'

    def __init__(self) -> None:
        super().__init__('mode_manager_node')

        self.declare_parameter('initial_mode', self._MANUAL)
        self.declare_parameter('teleop_topic',  'cmd_vel_teleop')
        self.declare_parameter('nav_topic',     'cmd_vel_nav')
        self.declare_parameter('output_topic',  'cmd_vel')

        teleop_topic  = self.get_parameter('teleop_topic').value
        nav_topic     = self.get_parameter('nav_topic').value
        output_topic  = self.get_parameter('output_topic').value
        initial       = self.get_parameter('initial_mode').value.lower()
        self._mode    = self._AUTO if initial == 'auto' else self._MANUAL

        # Publishers
        self._pub_cmd   = self.create_publisher(Twist,  output_topic, 10)
        self._pub_mode  = self.create_publisher(String, 'current_mode', _LATCHED)

        # Subscriptions
        self._sub_mode   = self.create_subscription(
            String, 'mode', self._mode_cb, 10)
        self._sub_teleop = self.create_subscription(
            Twist, teleop_topic, self._teleop_cb, 10)
        self._sub_nav    = self.create_subscription(
            Twist, nav_topic, self._nav_cb, 10)

        self.add_on_set_parameters_callback(self._on_params)

        self._publish_mode()
        self.get_logger().info(
            'ModeManager ready  mode=%s  teleop=%s  nav=%s  out=%s',
            self._mode, teleop_topic, nav_topic, output_topic,
        )

    # ── mode switching ────────────────────────────────────────────────────

    def _set_mode(self, new_mode: str) -> None:
        if new_mode not in (self._MANUAL, self._AUTO):
            self.get_logger().warn('Unknown mode "%s" ignored', new_mode)
            return
        if new_mode == self._mode:
            return
        self._mode = new_mode
        self._publish_mode()
        # Publish zero velocity on mode switch for safety
        self._pub_cmd.publish(Twist())
        self.get_logger().info('Mode → %s', self._mode)

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = self._mode
        self._pub_mode.publish(msg)

    # ── callbacks ─────────────────────────────────────────────────────────

    def _mode_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()
        if cmd == 'toggle':
            self._set_mode(self._AUTO if self._mode == self._MANUAL else self._MANUAL)
        else:
            self._set_mode(cmd)

    def _teleop_cb(self, msg: Twist) -> None:
        if self._mode == self._MANUAL:
            self._pub_cmd.publish(msg)
            self.get_logger().debug(
                'MANUAL fwd: lin=%.3f ang=%.3f', msg.linear.x, msg.angular.z)

    def _nav_cb(self, msg: Twist) -> None:
        if self._mode == self._AUTO:
            self._pub_cmd.publish(msg)
            self.get_logger().debug(
                'AUTO fwd: lin=%.3f ang=%.3f', msg.linear.x, msg.angular.z)

    def _on_params(self, params):
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
