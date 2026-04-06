#!/usr/bin/env python3
"""FollowBall ActionServer - logika ako ball_follower, len pocas goal (cancel = stop).

success_hold_ticks: kolko po sebe iducich kontrol (~0.08 s) musi byt r >= stop_radius a cerstva detekcia,
  aby goal uspel (jednoduchy debounce bez YOLO).
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ball_follower_base import BallFollowerBase
from waverower.action import FollowBall


class BallFollowerActionNode(BallFollowerBase):
    RC_OK, RC_CANCEL, RC_TIMEOUT, RC_ABORT = 0, 1, 2, 3

    def __init__(self) -> None:
        super().__init__()
        # Koľkokrát po sebe (sleep v slučke ~0.08 s) musí platiť „dosť blízko“ pred success — proti náhodnej ruke
        self.declare_parameter("success_hold_ticks", 8)
        self._following_active = False
        self._goal_in_progress = False
        self._cbg = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            FollowBall,
            "follow_ball",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cbg,
        )
        self.get_logger().info("FollowBall action server /follow_ball (py_trees BT, ball_follow_bt_runner, ...)")

    def _goal_cb(self, goal_request):
        if self._goal_in_progress:
            self.get_logger().warn("Novy goal odmietnuty - uz bezi iny.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        self._goal_in_progress = True
        stop_r_override = float(req.stop_radius_px) if hasattr(req, "stop_radius_px") and req.stop_radius_px > 0 else None

        self._following_active = True
        deadline = None
        if float(req.max_duration_sec) > 0.0:
            deadline = time.monotonic() + float(req.max_duration_sec)

        hold_need = max(1, int(self.get_parameter("success_hold_ticks").get_parameter_value().integer_value))
        close_ticks = 0
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._following_active = False
                    self._pub.publish(Twist())
                    goal_handle.canceled(FollowBall.Result(success=False, reason_code=self.RC_CANCEL, message="canceled"))
                    return FollowBall.Result(success=False, reason_code=self.RC_CANCEL, message="canceled")

                if deadline is not None and time.monotonic() > deadline:
                    self._following_active = False
                    self._pub.publish(Twist())
                    goal_handle.abort(FollowBall.Result(success=False, reason_code=self.RC_TIMEOUT, message="timeout"))
                    return FollowBall.Result(success=False, reason_code=self.RC_TIMEOUT, message="timeout")

                stop_r = stop_r_override or self.get_parameter("stop_radius_px").get_parameter_value().double_value
                fresh_close = (
                    self._ever_seen
                    and self._last_radius >= stop_r
                    and (time.monotonic() - self._last_det_time < 0.3)
                )
                if fresh_close:
                    close_ticks += 1
                    if close_ticks >= hold_need:
                        self._following_active = False
                        self._pub.publish(Twist())
                        res = FollowBall.Result(success=True, reason_code=self.RC_OK, message="close_enough")
                        goal_handle.succeed(res)
                        return res
                else:
                    close_ticks = 0

                fb = FollowBall.Feedback()
                fb.x_error   = float(self._last_x_err)
                fb.radius_px = float(self._last_radius)
                goal_handle.publish_feedback(fb)
                time.sleep(0.08)
        finally:
            self._following_active = False
            self._goal_in_progress = False
            self._pub.publish(Twist())

        res = FollowBall.Result(success=False, reason_code=self.RC_ABORT, message="shutdown")
        goal_handle.abort(res)
        return res


def main() -> None:
    rclpy.init()
    node = BallFollowerActionNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
