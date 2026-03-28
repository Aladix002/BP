#!/usr/bin/env python3
"""ROS2 Action klient: NavigateToPose pre autonómnu navigáciu WaveRovera.

Ukážka použitia ROS2 Actions (NavigateToPose) – BP demo.
Nav2 action server spracuje goal asynchrónne a posiela priebežný feedback.

Použitie:
  ros2 run waverower send_goal.py -- <x> <y> [yaw_deg]
  ros2 run waverower send_goal.py -- 1.0 2.0 90

Argumenty:
  x       – cieľová poloha X v metroch (frame: map)
  y       – cieľová poloha Y v metroch (frame: map)
  yaw_deg – orientácia v cieli v stupňoch (default: 0)
"""

import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("nav_goal_client")
        self._client: ActionClient = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def send_goal(self, x: float, y: float, yaw_rad: float) -> bool:
        self.get_logger().info("Čakám na navigate_to_pose action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server nedostupný – je spustený nav.launch.py?")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info(
            f"Posielam cieľ → x={x:.2f} m  y={y:.2f} m  yaw={math.degrees(yaw_rad):.1f}°"
        )

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Cieľ odmietnutý Nav2!")
            return False

        self.get_logger().info("Cieľ prijatý – navigácia prebieha...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Cieľ dosiahnutý!")
            return True

        self.get_logger().warn(f"Navigácia zlyhala (status={status})")
        return False

    def _on_feedback(self, feedback_msg: NavigateToPose.Impl.FeedbackMessage) -> None:
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"  zostatok: {dist:.2f} m", throttle_duration_sec=2.0)


def main() -> None:
    rclpy.init()

    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    x   = float(sys.argv[1])
    y   = float(sys.argv[2])
    yaw = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0

    node = NavGoalClient()
    ok = node.send_goal(x, y, yaw)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
