#!/usr/bin/env python3
"""
Shutdown node – safe RPi power-off from ROS 2 or hardware button.

Triggers:
  1. ROS topic  /shutdown  (std_msgs/String)  – publish any message to shutdown
  2. GPIO pin   (optional) – physical button connected to a GPIO pin
     Connect a push-button between GPIO_PIN and GND (uses internal pull-up).

How to call from command line:
  ros2 topic pub --once /shutdown std_msgs/msg/String "data: 'halt'"

Parameters:
  method          [string]  default "systemctl"   – "systemctl" | "shutdown"
  gpio_enabled    [bool]    default false          – enable GPIO button
  gpio_pin        [int]     default 21             – BCM pin number (pin 40 on header)
  gpio_debounce_s [float]   default 1.0            – ignore bounces for this many seconds

Sudoers setup (run once if method=shutdown):
  echo "$(whoami) ALL=(ALL) NOPASSWD: /sbin/shutdown" | sudo tee /etc/sudoers.d/ros_shutdown
"""

import subprocess
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class ShutdownNode(Node):

    def __init__(self) -> None:
        super().__init__('shutdown_node')

        self.declare_parameter('method',          'systemctl')
        self.declare_parameter('gpio_enabled',    False)
        self.declare_parameter('gpio_pin',        21)
        self.declare_parameter('gpio_debounce_s', 1.0)

        self._method   = self.get_parameter('method').value
        self._last_shutdown = 0.0

        # ROS topic trigger
        self._sub = self.create_subscription(
            String, '/shutdown', self._shutdown_cb, 10)

        # GPIO trigger (optional)
        if self.get_parameter('gpio_enabled').value:
            self._setup_gpio()
        else:
            self._gpio_chip = None

        self.get_logger().info(
            'ShutdownNode ready  method=%s  gpio=%s',
            self._method,
            'enabled pin=%d' % self.get_parameter('gpio_pin').value
            if self.get_parameter('gpio_enabled').value else 'disabled',
        )
        self.get_logger().info(
            'Trigger: ros2 topic pub --once /shutdown std_msgs/msg/String "data: halt"')

    # ── GPIO setup ────────────────────────────────────────────────────────

    def _setup_gpio(self) -> None:
        try:
            import gpiod  # type: ignore[import]
            pin   = self.get_parameter('gpio_pin').value
            chip  = gpiod.Chip('gpiochip0')
            line  = chip.get_line(pin)
            line.request(consumer='shutdown_node',
                         type=gpiod.LINE_REQ_EV_FALLING_EDGE,
                         flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
            self._gpio_chip = chip
            self._gpio_line = line
            # Poll GPIO in a timer callback (non-blocking)
            self._gpio_timer = self.create_timer(0.1, self._gpio_poll)
            self.get_logger().info('GPIO shutdown button on BCM pin %d', pin)
        except ImportError:
            self.get_logger().warn(
                'gpiod not installed – GPIO shutdown disabled. '
                'Install: pip3 install gpiod')
            self._gpio_chip = None
        except Exception as exc:
            self.get_logger().warn('GPIO setup failed: %s', exc)
            self._gpio_chip = None

    def _gpio_poll(self) -> None:
        if not hasattr(self, '_gpio_line'):
            return
        try:
            if self._gpio_line.event_wait(sec=0):
                self._gpio_line.event_read()
                self.get_logger().warn('GPIO button pressed – initiating shutdown')
                self._do_shutdown('gpio_button')
        except Exception:
            pass

    # ── callbacks ────────────────────────────────────────────────────────

    def _shutdown_cb(self, msg: String) -> None:
        self.get_logger().warn(
            'Shutdown requested via ROS topic: "%s"', msg.data)
        self._do_shutdown(f'ros_topic({msg.data})')

    # ── shutdown logic ────────────────────────────────────────────────────

    def _do_shutdown(self, reason: str) -> None:
        now = time.monotonic()
        debounce = self.get_parameter('gpio_debounce_s').value
        if now - self._last_shutdown < debounce:
            return
        self._last_shutdown = now

        self.get_logger().warn('=== SYSTEM SHUTDOWN triggered by: %s ===', reason)
        # Small delay so the log message can be published
        self.create_timer(0.5, self._execute_shutdown)

    def _execute_shutdown(self) -> None:
        method = self._method
        try:
            if method == 'systemctl':
                # Works without sudo via polkit on Ubuntu
                subprocess.run(['systemctl', 'poweroff'], check=True)
            else:
                # Requires sudoers entry (see docstring)
                subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
        except Exception as exc:
            self.get_logger().error('Shutdown command failed: %s', exc)

    def destroy_node(self) -> None:
        if self._gpio_chip is not None:
            try:
                self._gpio_chip.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ShutdownNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
