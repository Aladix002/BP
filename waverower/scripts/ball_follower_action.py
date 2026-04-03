#!/usr/bin/env python3
"""FollowBall ActionServer – rovnaká logika ako ball_follower, spustenie len počas goal (cancel = stop)."""

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
        self.get_logger().info("FollowBall action server na /follow_ball (klient: Nav2 BT alebo ros2 action send_goal)")

    def _goal_cb(self, goal_request):
        if self._goal_in_progress:
            self.get_logger().warn("Nový goal odmietnutý – už beží iný.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        self._goal_in_progress = True
        self._goal_stopped_close = False
        c = (req.ball_color or "").strip()
        self._goal_color_override = c if c else None

        self._following_active = True
        deadline = None
        if float(req.max_duration_sec) > 0.0:
            deadline = time.monotonic() + float(req.max_duration_sec)

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._following_active = False
                    self._goal_color_override = None
                    self._pub.publish(Twist())
                    res = FollowBall.Result(
                        success=False,
                        reason_code=self.RC_CANCEL,
                        message="canceled",
                    )
                    goal_handle.canceled(res)
                    return res

                if deadline is not None and time.monotonic() > deadline:
                    self._following_active = False
                    self._goal_color_override = None
                    self._pub.publish(Twist())
                    res = FollowBall.Result(
                        success=False,
                        reason_code=self.RC_TIMEOUT,
                        message="timeout",
                    )
                    goal_handle.abort(res)
                    return res

                if self._goal_stopped_close:
                    self._following_active = False
                    self._goal_color_override = None
                    self._pub.publish(Twist())
                    res = FollowBall.Result(
                        success=True,
                        reason_code=self.RC_OK,
                        message="close_enough",
                    )
                    goal_handle.succeed(res)
                    return res

                fb = FollowBall.Feedback()
                fb.state = self._last_fb_state
                fb.x_error = self._last_fb_x_err
                fb.radius_px = self._last_fb_radius_px
                goal_handle.publish_feedback(fb)
                time.sleep(0.08)
        finally:
            self._following_active = False
            self._goal_color_override = None
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
