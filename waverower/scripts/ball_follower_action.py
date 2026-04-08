#!/usr/bin/env python3
# Action server FollowBall: rovnaka detekcia ako BallFollowerBase, ale riadenie len pocas aktivneho goal.
# Cancel = zastav a nuluj cmd_vel. Jeden goal naraz (dalsi REJECT kym bezi execute).

import collections
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ball_follower_base import BallFollowerBase
from waverower.action import FollowBall


class BallFollowerActionNode(BallFollowerBase):
    RC_OK, RC_CANCEL, RC_TIMEOUT, RC_ABORT, RC_LOST = 0, 1, 2, 3, 4

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("success_hold_ticks", 8)
        self._following_active = False
        self._goal_in_progress = False
        self._active_goal_handle = None
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
        self.get_logger().info("FollowBall action server /follow_ball ready")

    def _goal_cb(self, goal_request):
        if self._goal_in_progress:
            if self._active_goal_handle is not None and not self._active_goal_handle.is_active:
                self.get_logger().warn("Reset staleho _goal_in_progress flagu.")
                self._goal_in_progress = False
            else:
                self.get_logger().warn("Novy goal odmietnuty - uz bezi iny.")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, cancel_request):
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        self._goal_in_progress = True
        self._active_goal_handle = goal_handle
        # Base._tick kontroluje tuto flag - ked True, publikuje cmd_vel z PID
        self._following_active = True

        stop_when_found = bool(req.stop_when_found)
        fail_on_lost    = float(req.fail_on_lost_sec)

        deadline = None
        if float(req.max_duration_sec) > 0.0:
            deadline = time.monotonic() + float(req.max_duration_sec)

        # Blizko ciela: musi platit N krat v case success_window (anti nahodny flash detekcie)
        success_count  = max(1, int(self.get_parameter("success_hold_ticks").get_parameter_value().integer_value))
        success_window = 1.0
        close_times: collections.deque = collections.deque()

        entered_track    = False
        track_lost_start = None

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

                lost_timeout = self.get_parameter("detection_lost_sec").get_parameter_value().double_value
                is_fresh = (
                    self._last_det_time > 0.0
                    and (time.monotonic() - self._last_det_time) < lost_timeout
                )

                # FindBall (BT prva faza): staci vidiet loptu -> SUCCESS, netreba dojazd
                if stop_when_found and is_fresh:
                    self._following_active = False
                    self._pub.publish(Twist())
                    res = FollowBall.Result(success=True, reason_code=self.RC_OK, message="ball_found")
                    goal_handle.succeed(res)
                    return res

                # FollowBall + fail_on_lost: ak sme uz sledovali a lopta zmizela dlhsie ako fail_on_lost -> ABORT
                if fail_on_lost > 0.0:
                    if is_fresh:
                        entered_track    = True
                        track_lost_start = None
                    elif entered_track:
                        if track_lost_start is None:
                            track_lost_start = time.monotonic()
                        elif time.monotonic() - track_lost_start > fail_on_lost:
                            self._following_active = False
                            self._pub.publish(Twist())
                            res = FollowBall.Result(success=False, reason_code=self.RC_LOST, message="ball_lost")
                            goal_handle.abort(res)
                            return res

                # Normalny koniec: polomer v px >= stop_radius_px drzane success_count-krat v okne 1s
                if not stop_when_found:
                    stop_r = self.get_parameter("stop_radius_px").get_parameter_value().double_value
                    now_t  = time.monotonic()
                    if self._ever_seen and self._last_radius >= stop_r and is_fresh:
                        close_times.append(now_t)
                    while close_times and (now_t - close_times[0]) > success_window:
                        close_times.popleft()
                    if len(close_times) >= success_count:
                        self._following_active = False
                        self._pub.publish(Twist())
                        res = FollowBall.Result(success=True, reason_code=self.RC_OK, message="close_enough")
                        goal_handle.succeed(res)
                        return res

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
