#!/usr/bin/env python3
"""Spustenie FollowBall action (jednoduchý „list“ pre orchestráciu / vlastný BT).

Predvolené: jeden goal na /follow_ball a čakanie na výsledok (bez py_trees).

Voliteľné: --py-trees  (sudo apt install ros-${ROS_DISTRO}-py-trees ros-${ROS_DISTRO}-py-trees-ros)

Príklad:
  ros2 run waverower ball_follow_bt_runner.py
  ros2 run waverower ball_follow_bt_runner.py -- --duration 60 --color white
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from waverower.action import FollowBall


def run_simple_client(args: argparse.Namespace) -> int:
    rclpy.init()
    node = Node("ball_follow_bt_runner")
    client = ActionClient(node, FollowBall, "follow_ball")
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("Action /follow_ball nedostupný – spusti: ros2 run waverower ball_follower_action.py")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    goal = FollowBall.Goal()
    goal.ball_color = args.color or ""
    goal.max_duration_sec = float(args.duration)
    node.get_logger().info(
        f"FollowBall goal: color='{goal.ball_color}' max_duration={goal.max_duration_sec}s"
    )

    send_fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_fut)
    gh = send_fut.result()
    if not gh.accepted:
        node.get_logger().error("Goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    result_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    res = result_fut.result().result
    node.get_logger().info(f"Výsledok: success={res.success} reason={res.reason_code} msg={res.message}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if res.success else 3


def run_py_trees(args: argparse.Namespace) -> int:
    import os

    try:
        import py_trees
        from py_trees.common import Status
    except ImportError:
        dist = os.environ.get("ROS_DISTRO", "jazzy")
        print(
            "Chýba py_trees. Inštaluj:\n"
            f"  sudo apt install ros-{dist}-py-trees ros-{dist}-py-trees-ros",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = Node("ball_follow_bt_runner")

    class SendFollowBall(py_trees.behaviour.Behaviour):
        def __init__(self):
            super().__init__("FollowBall")
            self._client = ActionClient(node, FollowBall, "follow_ball")
            self._goal_future = None
            self._result_future = None
            self._phase = "wait_server"

        def initialise(self):
            self._goal_future = None
            self._result_future = None
            self._phase = "wait_server"

        def update(self):
            if self._phase == "wait_server":
                if not self._client.wait_for_server(timeout_sec=0.2):
                    return Status.RUNNING
                g = FollowBall.Goal()
                g.ball_color = args.color or ""
                g.max_duration_sec = float(args.duration)
                self._goal_future = self._client.send_goal_async(g)
                self._phase = "accept"
                return Status.RUNNING
            if self._phase == "accept":
                if not self._goal_future.done():
                    return Status.RUNNING
                gh = self._goal_future.result()
                if not gh.accepted:
                    return Status.FAILURE
                self._result_future = gh.get_result_async()
                self._phase = "result"
                return Status.RUNNING
            if self._phase == "result":
                if not self._result_future.done():
                    return Status.RUNNING
                r = self._result_future.result().result
                return Status.SUCCESS if r.success else Status.FAILURE
            return Status.FAILURE

    root = py_trees.composites.Sequence(name="root", memory=True)
    root.add_children([SendFollowBall()])

    from rclpy.executors import MultiThreadedExecutor

    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)

    def tick():
        st = root.tick()
        if st != Status.RUNNING:
            rclpy.shutdown()

    timer = node.create_timer(0.05, tick)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--py-trees", action="store_true", help="Použiť py_trees Sequence (vyžaduje apt balíky)")
    p.add_argument("--duration", type=float, default=120.0, help="max_duration_sec v goal")
    p.add_argument("--color", type=str, default="", help="ball_color (prázdne = default servera)")
    args, _ = p.parse_known_args()

    if args.py_trees:
        sys.exit(run_py_trees(args))
    sys.exit(run_simple_client(args))


if __name__ == "__main__":
    main()
