#!/usr/bin/env python3
# Behavior tree (py_trees): sekvencia FindBall -> FollowBall -> Celebrate.
# FindBall posle action goal s stop_when_found=True (kratky uspech). FollowBall plne sledovanie.

import sys
import time

import py_trees
import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from waverover.action import FollowBall


def _running_behaviour_name(behaviour: py_trees.behaviour.Behaviour) -> str:
    if behaviour.status != py_trees.common.Status.RUNNING:
        return ""
    children = getattr(behaviour, "children", None)
    if children:
        for c in children:
            if c.status == py_trees.common.Status.RUNNING:
                sub = _running_behaviour_name(c)
                return sub if sub else c.name
        return behaviour.name
    return behaviour.name


class ActionBehaviour(py_trees.behaviour.Behaviour):
    # Wrapper: async send_goal -> caka na result; py_trees vola update() dokym RUNNING
    def __init__(self, name: str, node: Node, ball_color: str,
                 max_duration_sec: float, stop_when_found: bool, fail_on_lost_sec: float):
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

    def setup(self, **kwargs):
        if not self._client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/follow_ball server nedostupny")

    def initialise(self):
        self._sent = False
        self._goal_handle = None
        self._result = None

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
                f"[{self.name}] Goal odoslany"
                f" stop_when_found={self._stop_when_found}"
                f" fail_on_lost={self._fail_on_lost}s"
            )
            return py_trees.common.Status.RUNNING

        if self._result is None:
            return py_trees.common.Status.RUNNING

        if self._result.success:
            self._node.get_logger().info(f"[{self.name}] SUCCESS: {self._result.message}")
            return py_trees.common.Status.SUCCESS

        self._node.get_logger().warn(f"[{self.name}] FAILURE: {self._result.message}")
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        # INVALID = strom zastaveny (Ctrl+C) -> zrus goal
        if new_status == py_trees.common.Status.INVALID and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._node.get_logger().error(f"[{self.name}] Goal odmietnuty!")
            self._result = FollowBall.Result(success=False, reason_code=3, message="rejected")
            return
        self._goal_handle.get_result_async().add_done_callback(
            lambda f: setattr(self, "_result", f.result().result)
        )

    def _on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self._node.get_logger().info(
            f"  [{self.name}] x_err={fb.x_error:+.2f}  r={fb.radius_px:.0f}px",
            throttle_duration_sec=1.0,
        )


class CelebrateBehaviour(py_trees.behaviour.Behaviour):
    # Po uspechu kratke otocenie na mieste (vizualna "radost")
    def __init__(self, node: Node, spin_sec: float = 1.2):
        super().__init__("Celebrate")
        self._node     = node
        self._spin_sec = spin_sec
        self._pub      = node.create_publisher(Twist, "/cmd_vel", 10)
        self._start    = None

    def initialise(self):
        self._start = time.monotonic()
        self._node.get_logger().info("[Celebrate] Nasiel som loptu!")

    def update(self) -> py_trees.common.Status:
        if time.monotonic() - self._start < self._spin_sec:
            cmd = Twist()
            cmd.angular.z = 2.0
            self._pub.publish(cmd)
            return py_trees.common.Status.RUNNING
        self._pub.publish(Twist())
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        self._pub.publish(Twist())


def main():
    rclpy.init(args=sys.argv)
    node = Node("ball_follow_bt_runner")

    node.declare_parameter("ball_color",       "orange")
    node.declare_parameter("tick_rate_hz",     10.0)
    node.declare_parameter("find_timeout_sec", 30.0)
    node.declare_parameter("fail_on_lost_sec", 0.0)

    color        = node.get_parameter("ball_color").get_parameter_value().string_value
    tick_hz      = max(node.get_parameter("tick_rate_hz").get_parameter_value().double_value, 1.0)
    find_timeout = node.get_parameter("find_timeout_sec").get_parameter_value().double_value
    fail_on_lost = max(0.0, node.get_parameter("fail_on_lost_sec").get_parameter_value().double_value)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    bt_status_pub = node.create_publisher(String, "/ball_follow_bt/active_behaviour", 10)

    # memory=True: po uspesnom FindBall sa uz nevracia na zaciatok sekvencie
    sequence = py_trees.composites.Sequence(name="BallFollowSeq", memory=True)
    sequence.add_children([
        ActionBehaviour("FindBall",   node, color, find_timeout, stop_when_found=True,  fail_on_lost_sec=0.0),
        ActionBehaviour("FollowBall", node, color, 0.0,          stop_when_found=False, fail_on_lost_sec=fail_on_lost),
        CelebrateBehaviour(node),
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
        f"BT start  tick={tick_hz:.0f}Hz  color={color}  fail_on_lost_sec={fail_on_lost:.1f}"
    )
    node.get_logger().info("BT strom:\n" + py_trees.display.ascii_tree(tree.root))

    try:
        while rclpy.ok():
            tree.tick()
            active = _running_behaviour_name(sequence)
            bt_status_pub.publish(String(data=active if active else sequence.name))
            st = tree.root.status
            if st == py_trees.common.Status.SUCCESS:
                node.get_logger().info("=== USPECH - lopta dosiahnuta ===")
                break
            if st == py_trees.common.Status.FAILURE:
                node.get_logger().warn("=== ZLYHANIE ===")
                break
            executor.spin_once(timeout_sec=1.0 / tick_hz)
    except KeyboardInterrupt:
        tree.root.stop(py_trees.common.Status.INVALID)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
