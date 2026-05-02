#!/usr/bin/env python3
# BT: FindBall, FollowBall, Celebrate.

import sys
import time

import py_trees
import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from waverover.action import FollowBall


class _ActionBehaviour(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        node: Node,
        ball_color: str,
        max_duration_sec: float,
        stop_when_found: bool,
        fail_on_lost_sec: float,
    ) -> None:
        super().__init__(name)
        self._node            = node
        self._ball_color      = ball_color
        self._max_dur         = max_duration_sec
        self._stop_when_found = stop_when_found
        self._fail_on_lost    = fail_on_lost_sec
        self._client          = ActionClient(node, FollowBall, "follow_ball")
        self._goal_handle     = None
        self._result          = None
        self._sent            = False

    def setup(self, **kwargs) -> None:
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/follow_ball action server not available")

    def initialise(self) -> None:
        self._sent        = False
        self._goal_handle = None
        self._result      = None

    def update(self) -> py_trees.common.Status:
        if not self._sent:
            goal = FollowBall.Goal(
                ball_color=self._ball_color,
                max_duration_sec=float(self._max_dur),
                stop_when_found=self._stop_when_found,
                fail_on_lost_sec=float(self._fail_on_lost),
            )
            fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
            fut.add_done_callback(self._on_goal_response)
            self._sent = True
            self._node.get_logger().info(
                f"[{self.name}] goal sent"
                f"  stop_when_found={self._stop_when_found}"
                f"  fail_on_lost={self._fail_on_lost}s"
            )
            return py_trees.common.Status.RUNNING

        if self._result is None:
            return py_trees.common.Status.RUNNING

        if self._result.success:
            self._node.get_logger().info(f"[{self.name}] SUCCESS: {self._result.message}")
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warn(f"[{self.name}] FAILURE: {self._result.message}")
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status == py_trees.common.Status.INVALID and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._node.get_logger().error(f"[{self.name}] goal rejected")
            self._result = FollowBall.Result(success=False, reason_code=3, message="rejected")
            return
        self._goal_handle.get_result_async().add_done_callback(
            lambda f: setattr(self, "_result", f.result().result)
        )

    def _on_feedback(self, fb_msg) -> None:
        fb = fb_msg.feedback
        self._node.get_logger().info(
            f"[{self.name}] x={fb.x_error:+.2f}  r={fb.radius_px:.0f}px",
            throttle_duration_sec=1.0,
        )


class _CelebrateBehaviour(py_trees.behaviour.Behaviour):
    SPIN_SPEED   = 8.0   # rad/s
    PHASE_SEC    = 0.35  # seconds per direction
    TOTAL_SEC    = 4.2   # total celebrate duration

    def __init__(self, node: Node) -> None:
        super().__init__("Celebrate")
        self._node  = node
        self._pub   = node.create_publisher(Twist, "/cmd_vel", 10)
        self._start = 0.0

    def initialise(self) -> None:
        self._start = time.monotonic()
        self._node.get_logger().info("[Celebrate] found the ball!")

    def update(self) -> py_trees.common.Status:
        elapsed = time.monotonic() - self._start
        if elapsed >= self.TOTAL_SEC:
            self._pub.publish(Twist())
            return py_trees.common.Status.SUCCESS
        phase = int(elapsed / self.PHASE_SEC)
        cmd = Twist()
        cmd.angular.z = self.SPIN_SPEED if phase % 2 == 0 else -self.SPIN_SPEED
        self._pub.publish(cmd)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self._pub.publish(Twist())


def main() -> None:
    rclpy.init(args=sys.argv)
    node = Node("ball_follow_bt_runner")

    node.declare_parameter("ball_color",       "orange")
    node.declare_parameter("tick_rate_hz",     10.0)
    node.declare_parameter("find_timeout_sec",  0.0)
    node.declare_parameter("fail_on_lost_sec",  0.0)
    node.declare_parameter("retry_on_lost",    True)

    color        = node.get_parameter("ball_color").value
    tick_hz      = max(float(node.get_parameter("tick_rate_hz").value), 1.0)
    find_timeout = float(node.get_parameter("find_timeout_sec").value)
    fail_on_lost = max(0.0, float(node.get_parameter("fail_on_lost_sec").value))
    retry_on_lost = bool(node.get_parameter("retry_on_lost").value)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    status_pub = node.create_publisher(String, "/ball_follow_bt/active_behaviour", 10)

    # memory=True: after FindBall succeeds, skip it on restart
    sequence = py_trees.composites.Sequence(name="BallFollowSeq", memory=True)
    find_ball = _ActionBehaviour("FindBall",   node, color, find_timeout, stop_when_found=True,  fail_on_lost_sec=0.0)
    follow_ball = _ActionBehaviour("FollowBall", node, color, 0.0,        stop_when_found=False, fail_on_lost_sec=fail_on_lost)
    sequence.add_children([
        find_ball,
        follow_ball,
        _CelebrateBehaviour(node),
    ])

    tree = py_trees.trees.BehaviourTree(sequence)
    py_trees.logging.level = py_trees.logging.Level.INFO

    try:
        tree.setup(timeout=15)
    except RuntimeError as e:
        node.get_logger().error(str(e))
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(
        f"BT ready  tick={tick_hz:.0f} Hz  color={color}  fail_on_lost={fail_on_lost:.1f} s"
        f"  retry_on_lost={retry_on_lost}"
    )
    node.get_logger().info("tree:\n" + py_trees.display.ascii_tree(tree.root))

    try:
        while rclpy.ok():
            # Najprv spracuj callbacky (action client / feedback), potom BT tick
            executor.spin_once(timeout_sec=0.0)
            executor.spin_once(timeout_sec=0.0)
            tree.tick()
            active = _running_name(sequence)
            status_pub.publish(String(data=active or sequence.name))
            st = tree.root.status
            if st == py_trees.common.Status.SUCCESS:
                node.get_logger().info("=== SUCCESS – ball reached ===")
                break
            if st == py_trees.common.Status.FAILURE:
                # Ak sa lopta stratila v FollowBall, resetni strom a znova hladaj cez FindBall.
                if retry_on_lost and follow_ball.status == py_trees.common.Status.FAILURE:
                    node.get_logger().warn("=== FOLLOW FAILED: reset BT -> FindBall ===")
                    tree.root.stop(py_trees.common.Status.INVALID)
                    continue
                node.get_logger().warn("=== FAILURE ===")
                break
            executor.spin_once(timeout_sec=max(0.01, 1.0 / tick_hz))
    except KeyboardInterrupt:
        tree.root.stop(py_trees.common.Status.INVALID)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _running_name(b: py_trees.behaviour.Behaviour) -> str:
    if b.status != py_trees.common.Status.RUNNING:
        return ""
    for c in getattr(b, "children", []):
        if c.status == py_trees.common.Status.RUNNING:
            return _running_name(c) or c.name
    return b.name


if __name__ == "__main__":
    main()
