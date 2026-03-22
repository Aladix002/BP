#!/usr/bin/env python3
"""
Node Manager – dynamically starts and stops task-specific ROS2 nodes.

Instead of launching SLAM + Nav2 at the same time (waste of CPU/RAM on RPi),
this node manages which navigation stack is running based on the current task.

Always running (in bringup.launch.py):
  robot_state_publisher, imu_node, motor_controller, watchdog,
  imu_stabilizer, mode_manager, shutdown_node, THIS node

Started/stopped on demand:
  "slam"  → slam_toolbox  +  robot_localization EKF
  "nav"   → nav2 (amcl + planner + controller + costmaps)  +  EKF  +  map_server
  "off"   → nothing extra (pure teleop)

Topics:
  SUB  /nav_mode  (std_msgs/String)  – "slam" | "nav" | "off"
  PUB  /nav_mode_status (std_msgs/String) – current active task

Parameters:
  slam_launch_pkg   default "wave_rover_navigation"
  slam_launch_file  default "slam_only.launch.py"
  nav_launch_pkg    default "wave_rover_navigation"
  nav_launch_file   default "nav_only.launch.py"
  map_path          default ""   (required for nav mode)
  shutdown_timeout  default 8.0  (seconds to wait for process to die)
"""

import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class NodeManagerNode(Node):

    VALID_MODES = ('slam', 'nav', 'off')

    def __init__(self) -> None:
        super().__init__('node_manager_node')

        self.declare_parameter('slam_launch_pkg',   'wave_rover_navigation')
        self.declare_parameter('slam_launch_file',  'slam_only.launch.py')
        self.declare_parameter('nav_launch_pkg',    'wave_rover_navigation')
        self.declare_parameter('nav_launch_file',   'nav_only.launch.py')
        self.declare_parameter('map_path',          '')
        self.declare_parameter('shutdown_timeout',   8.0)

        self._current_task: str = 'off'
        self._proc: subprocess.Popen | None = None
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            String, '/nav_mode', self._nav_mode_cb, 10)
        self._pub = self.create_publisher(String, '/nav_mode_status', 10)

        # Publish current status at 1 Hz so UI can show it
        self._timer = self.create_timer(1.0, self._publish_status)

        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'NodeManager ready – publish to /nav_mode: "slam" | "nav" | "off"')

    # ── status publisher ──────────────────────────────────────────────────

    def _publish_status(self) -> None:
        msg = String()
        msg.data = self._current_task
        self._pub.publish(msg)

    # ── nav mode callback ─────────────────────────────────────────────────

    def _nav_mode_cb(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode not in self.VALID_MODES:
            self.get_logger().warn(
                'Unknown nav_mode "%s" – valid: %s', mode, self.VALID_MODES)
            return
        if mode == self._current_task:
            return
        # Run in thread so we don't block the spin loop
        threading.Thread(
            target=self._switch_task, args=(mode,), daemon=True).start()

    # ── task switching ────────────────────────────────────────────────────

    def _switch_task(self, new_task: str) -> None:
        with self._lock:
            self._stop_current()
            if new_task != 'off':
                self._start_task(new_task)
            self._current_task = new_task
            self._publish_status()

    def _stop_current(self) -> None:
        if self._proc is None:
            return
        timeout = self.get_parameter('shutdown_timeout').value
        self.get_logger().info(
            'Stopping task "%s" (pid=%d)...', self._current_task, self._proc.pid)
        try:
            # Send SIGINT first (ROS2 nodes handle it gracefully)
            self._proc.send_signal(2)  # SIGINT
            self._proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Process did not stop – sending SIGKILL')
            self._proc.kill()
            self._proc.wait()
        except Exception as exc:
            self.get_logger().error('Error stopping process: %s', exc)
        finally:
            self._proc = None
        self.get_logger().info('Task "%s" stopped.', self._current_task)

    def _start_task(self, task: str) -> None:
        if task == 'slam':
            pkg  = self.get_parameter('slam_launch_pkg').value
            file = self.get_parameter('slam_launch_file').value
            extra = []
        else:  # nav
            pkg  = self.get_parameter('nav_launch_pkg').value
            file = self.get_parameter('nav_launch_file').value
            map_path = self.get_parameter('map_path').value
            extra = [f'map:={map_path}'] if map_path else []

        cmd = ['ros2', 'launch', pkg, file] + extra
        self.get_logger().info('Starting task "%s": %s', task, ' '.join(cmd))
        try:
            # Start as new process group so we can kill the whole tree
            self._proc = subprocess.Popen(
                cmd,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            # Brief pause to catch immediate startup failures
            time.sleep(1.5)
            if self._proc.poll() is not None:
                self.get_logger().error(
                    'Task "%s" exited immediately (rc=%d)', task, self._proc.returncode)
                self._proc = None
            else:
                self.get_logger().info(
                    'Task "%s" started (pid=%d)', task, self._proc.pid)
        except FileNotFoundError:
            self.get_logger().error(
                'ros2 not found – is ROS2 sourced in this shell?')

    def _on_params(self, params):
        return SetParametersResult(successful=True)

    def destroy_node(self) -> None:
        with self._lock:
            self._stop_current()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NodeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
