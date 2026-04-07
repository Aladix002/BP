#!/usr/bin/env python3
"""py_trees BT runner -> /follow_ball ActionServer.

Terminal 1: ros2 launch waverower ball_follower_action.launch.py
Terminal 2: ros2 run waverower ball_follow_bt_runner.py

Zavislost: ros-jazzy-py-trees
"""

import sys
import time

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from waverower.action import FollowBall


class FollowBallBehaviour(py_trees.behaviour.Behaviour):
    """py_trees uzol: goal na /follow_ball, caka na vysledok."""

    def __init__(self, node: Node, ball_color: str, max_duration_sec: float):
        super().__init__("FollowBall")
        self._node        = node
        self._client      = ActionClient(node, FollowBall, "follow_ball")
        self._ball_color  = ball_color
        self._max_dur     = max_duration_sec
        self._goal_handle = None
        self._result = None
        self._sent        = False

    def setup(self, **kwargs):
        self._node.get_logger().info("Cakam na /follow_ball server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "/follow_ball nedostupny - spusti: ros2 launch waverower ball_follower_action.launch.py"
            )
        self._node.get_logger().info("Server /follow_ball OK")

    def initialise(self):
        self._sent        = False
        self._goal_handle = None
        self._result      = None

    def update(self) -> py_trees.common.Status:
        if not self._sent:
            goal = FollowBall.Goal(
                ball_color=self._ball_color,
                max_duration_sec=float(self._max_dur),
            )
            fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            fut.add_done_callback(self._on_goal_response)
            self._sent = True
            self._node.get_logger().info(
                f"Goal odoslany color='{self._ball_color}' max_dur={self._max_dur}s"
            )
            return py_trees.common.Status.RUNNING

        if self._result is None:
            return py_trees.common.Status.RUNNING

        if self._result.success:
            self._node.get_logger().info(f"[BT] SUCCESS - {self._result.message}")
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warn(
            f"[BT] FAILURE - {self._result.message} code={self._result.reason_code}"
        )
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        if new_status == py_trees.common.Status.INVALID and self._goal_handle is not None:
            self._node.get_logger().info("[BT] terminate -> cancel goal")
            self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._node.get_logger().error("Goal odmietnuty!")
            self._result = FollowBall.Result(success=False, reason_code=3, message="rejected")
            return
        self._goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._result = future.result().result

    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self._node.get_logger().info(
            f"  [feedback] x_err={fb.x_error:+.2f}  r={fb.radius_px:.0f}px",
            throttle_duration_sec=1.0,
        )


def main():
    rclpy.init(args=sys.argv)
    node = Node("ball_follow_bt_runner")

    node.declare_parameter("ball_color",       "orange")
    node.declare_parameter("max_duration_sec", 0.0)
    node.declare_parameter("tick_rate_hz",     10.0)

    ball_color  = node.get_parameter("ball_color").get_parameter_value().string_value
    max_dur     = node.get_parameter("max_duration_sec").get_parameter_value().double_value
    tick_hz     = max(node.get_parameter("tick_rate_hz").get_parameter_value().double_value, 1.0)
    tick_period = 1.0 / tick_hz

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    root = py_trees.composites.Sequence(name="BallFollowSeq", memory=True)
    root.add_child(FollowBallBehaviour(node, ball_color, max_dur))
    tree = py_trees.trees.BehaviourTree(root)

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    try:
        tree.setup(timeout=15)
    except RuntimeError as e:
        node.get_logger().error(str(e))
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(f"BT startuje tick {tick_hz:.0f} Hz")
    print(py_trees.display.ascii_tree(tree.root))

    try:
        while rclpy.ok():
            tree.tick()
            st = tree.root.status
            if st == py_trees.common.Status.SUCCESS:
                node.get_logger().info("=== BT: SUCCESS ===")
                break
            if st == py_trees.common.Status.FAILURE:
                node.get_logger().warn("=== BT: FAILURE ===")
                break
            executor.spin_once(timeout_sec=tick_period)
    except KeyboardInterrupt:
        print("Ctrl-C -> cancel + stop")
        tree.root.stop(py_trees.common.Status.INVALID)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
