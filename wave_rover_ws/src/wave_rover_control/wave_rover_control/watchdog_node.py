#!/usr/bin/env python3
"""
Watchdog (safety) node – publishes zero velocity if /cmd_vel goes stale.

If no Twist message arrives on /cmd_vel within `timeout_ms` milliseconds,
the watchdog publishes a zero Twist to /cmd_vel_safe to bring the robot
to a stop.  The motor_controller_node should subscribe to /cmd_vel_safe
rather than /cmd_vel directly when this node is in use.

  /cmd_vel  ──► watchdog ──► /cmd_vel_safe
                   |
                 timeout? → zero Twist

Publishes:
  /cmd_vel_safe  (geometry_msgs/Twist)

Subscribes:
  /cmd_vel       (geometry_msgs/Twist)

Parameters:
  timeout_ms   [int]    default 500   – watchdog timeout
  rate_hz      [float]  default 50.0  – output publish rate
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist


class WatchdogNode(Node):

    def __init__(self) -> None:
        super().__init__('watchdog_node')

        self.declare_parameter('timeout_ms', 500)
        self.declare_parameter('rate_hz', 50.0)

        self._timeout_ms = max(100, self.get_parameter('timeout_ms').value)
        rate_hz          = max(10.0, self.get_parameter('rate_hz').value)

        self._last_msg: Twist | None = None
        self._last_time_ns = self.get_clock().now().nanoseconds

        self._sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, 10)
        self._pub = self.create_publisher(
            Twist, 'cmd_vel_safe', 10)

        self._timer = self.create_timer(1.0 / rate_hz, self._tick)

        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'Watchdog ready  timeout=%dms  rate=%.0fHz',
            self._timeout_ms, rate_hz,
        )

    def _on_params(self, params):
        for p in params:
            if p.name == 'timeout_ms':
                self._timeout_ms = max(100, int(p.value))
        return SetParametersResult(successful=True)

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_msg     = msg
        self._last_time_ns = self.get_clock().now().nanoseconds

    def _tick(self) -> None:
        elapsed_ms = (
            self.get_clock().now().nanoseconds - self._last_time_ns
        ) / 1e6

        if elapsed_ms > self._timeout_ms:
            if self._last_msg is not None:
                self.get_logger().warn(
                    'Watchdog: cmd_vel stale (%.0f ms > %d ms) – STOP',
                    elapsed_ms, self._timeout_ms,
                )
                self._last_msg = None
            self._pub.publish(Twist())   # zero velocity
        elif self._last_msg is not None:
            self._pub.publish(self._last_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
