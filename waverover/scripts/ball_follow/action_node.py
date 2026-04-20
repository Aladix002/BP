#!/usr/bin/env python3
import collections
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
import rclpy.qos
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32

from detector import Detection, DetectorConfig, OrangeDetector
from controller import BallController, ControllerConfig
from waverover.action import FollowBall


class BallFollowerNode(Node):
    RC_OK, RC_CANCEL, RC_TIMEOUT, RC_ABORT, RC_LOST = 0, 1, 2, 3, 4

    def __init__(self) -> None:
        super().__init__("ball_follower")
        self._declare_params()

        self._bridge = CvBridge()
        self._det  = OrangeDetector(self._detector_cfg())
        self._ctrl = BallController(self._controller_cfg())

        cmd_topic = self.get_parameter("cmd_topic").value
        self._cmd_pub  = self.create_publisher(Twist,           cmd_topic,                                1)
        self._dbg_pub  = self.create_publisher(Image,           "/ball_follower/debug_image",             1)
        self._mask_pub = None
        if bool(self.get_parameter("publish_debug_mask").value):
            self._mask_pub = self.create_publisher(
                CompressedImage, "/ball_follower/debug_mask/compressed", 1
            )
        self._conf_pub = self.create_publisher(Float32,         "/ball_follower/detection_confidence",   10)

        self._last_det:   Optional[Detection] = None
        self._last_det_t: float = 0.0
        self._last_frm_t: float = 0.0
        self._last_dbg_pub_t: float = 0.0
        self._active:     bool  = False
        self._goal_busy:  bool  = False

        self._img_sub = None
        self._setup_image_subscription()

        rate = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(4.0,        self._warn_no_frames)

        cbg = ReentrantCallbackGroup()
        ActionServer(
            self, FollowBall, "follow_ball",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cbg,
        )
        comp = self.get_parameter("subscribe_compressed").value
        self.get_logger().info(
            f"BallFollower ready  compressed={comp}  topic={self.get_parameter('image_topic').value}"
        )

    def _setup_image_subscription(self) -> None:
        qos = rclpy.qos.qos_profile_sensor_data
        topic = self.get_parameter("image_topic").value
        use_comp = bool(self.get_parameter("subscribe_compressed").value)
        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None
        if use_comp:
            self._img_sub = self.create_subscription(
                CompressedImage, topic, self._compressed_cb, qos
            )
        else:
            self._img_sub = self.create_subscription(Image, topic, self._image_cb, qos)

    # ── parameter declaration ─────────────────────────────────────────────────

    def _declare_params(self) -> None:
        dp = self.declare_parameter
        # camera_ros: JPEG na .../compressed (rovnako ako optical_flow v runtime_stack)
        dp("image_topic", "/camera/camera_node/image_raw/compressed")
        dp("subscribe_compressed", True)
        dp("cmd_topic",         "/cmd_vel")
        dp("control_rate_hz",   20.0)
        dp("success_hold_ticks", 1)
        # HSV orange
        dp("orange_h_min", 10);  dp("orange_h_max", 24)
        dp("orange_s_min", 90);  dp("orange_s_max", 255)
        dp("orange_v_min", 90);  dp("orange_v_max", 255)
        # Pre-processing
        dp("blur_ksize",   11)
        dp("erode_iters",   2);  dp("dilate_iters",  2)
        # Detection filters
        dp("min_radius_px",    10.0);  dp("max_radius_px",    145.0)
        dp("min_circularity",  0.76);  dp("min_solidity",     0.84)
        dp("min_fill_ratio",   0.64);  dp("max_area_ratio",   0.12)
        dp("max_aspect_ratio", 1.20);  dp("min_confidence",   0.70)
        dp("max_jump_frac",    0.28);  dp("jump_reset_sec",   0.45)
        # Motion
        dp("forward_speed",     0.72)
        dp("stop_radius_px",   40.0)
        dp("detection_lost_sec", 1.5)
        # Differential drive → cmd_vel
        dp("wheel_base_cmd",     2.0);  dp("max_linear",         1.0)
        dp("invert_linear",      True)
        dp("side_gain",          1.02); dp("mix_max",            0.68)
        dp("min_wheel_fwd",      0.12)
        dp("side_slowdown_gain", 0.50); dp("side_slowdown_min",  0.40)
        dp("err_exp", 0.85)
        # Approach brake
        dp("brake_band_px",  14.0);  dp("turn_blend_min",  0.45)
        # Smoothing & burst search
        dp("smooth_alpha",   0.36)
        dp("burst_speed",   10.0)
        dp("burst_on_sec",   0.45);  dp("burst_off_sec",   0.90)
        # Debug do rqt: plny Image kazdy frame je na RPi drahy — obmedz frekvenciu alebo vypni
        dp("publish_debug_image", True)
        dp("publish_debug_mask", False)
        dp("debug_image_max_hz", 10.0)

    def _detector_cfg(self) -> DetectorConfig:
        g = lambda n: self.get_parameter(n).value
        return DetectorConfig(
            h_min=g("orange_h_min"),  h_max=g("orange_h_max"),
            s_min=g("orange_s_min"),  s_max=g("orange_s_max"),
            v_min=g("orange_v_min"),  v_max=g("orange_v_max"),
            blur_ksize=g("blur_ksize"),
            erode_iters=g("erode_iters"),   dilate_iters=g("dilate_iters"),
            min_radius=g("min_radius_px"),  max_radius=g("max_radius_px"),
            min_circularity=g("min_circularity"),
            min_solidity=g("min_solidity"),
            min_fill_ratio=g("min_fill_ratio"),
            max_area_ratio=g("max_area_ratio"),
            max_aspect_ratio=g("max_aspect_ratio"),
            min_confidence=g("min_confidence"),
            max_jump_frac=g("max_jump_frac"),
            jump_reset_sec=g("jump_reset_sec"),
        )

    def _controller_cfg(self) -> ControllerConfig:
        g = lambda n: self.get_parameter(n).value
        return ControllerConfig(
            forward_speed=g("forward_speed"),
            stop_radius_px=g("stop_radius_px"),
            detection_lost_sec=g("detection_lost_sec"),
            wheel_base=g("wheel_base_cmd"),   max_linear=g("max_linear"),
            invert_linear=g("invert_linear"),
            side_gain=g("side_gain"),         mix_max=g("mix_max"),
            min_wheel_fwd=g("min_wheel_fwd"),
            side_slowdown_gain=g("side_slowdown_gain"),
            side_slowdown_min=g("side_slowdown_min"),
            err_exp=g("err_exp"),
            brake_band_px=g("brake_band_px"),
            turn_blend_min=g("turn_blend_min"),
            smooth_alpha=g("smooth_alpha"),
            burst_speed=g("burst_speed"),
            burst_on_sec=g("burst_on_sec"),
            burst_off_sec=g("burst_off_sec"),
        )

    # ── image processing ──────────────────────────────────────────────────────

    def _compressed_cb(self, msg: CompressedImage) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self._last_frm_t = time.monotonic()
        self._process_frame(frame)

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return
        self._last_frm_t = time.monotonic()
        self._process_frame(frame)

    def _process_frame(self, frame: np.ndarray) -> None:
        det = self._det.detect(frame)
        self._last_det = det
        if det is not None:
            self._last_det_t = time.monotonic()

        self._conf_pub.publish(Float32(data=float(det.confidence) if det else 0.0))

        now = time.monotonic()
        want_img = bool(self.get_parameter("publish_debug_image").value)
        want_msk = bool(self.get_parameter("publish_debug_mask").value)
        max_hz = max(0.5, float(self.get_parameter("debug_image_max_hz").value))
        if (want_img or want_msk) and (now - self._last_dbg_pub_t) >= (1.0 / max_hz):
            self._last_dbg_pub_t = now
            if want_img:
                dbg = self._det.make_debug_image(frame, det)
                try:
                    self._dbg_pub.publish(self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
                except Exception:
                    pass
            if want_msk and self._mask_pub is not None:
                mask_bytes = self._det.make_mask_jpg(frame)
                if mask_bytes:
                    cm = CompressedImage()
                    cm.header.stamp = self.get_clock().now().to_msg()
                    cm.format = "jpeg"
                    cm.data = mask_bytes
                    self._mask_pub.publish(cm)

    # ── control tick ──────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if not self._active:
            return
        now = time.monotonic()
        has_frame = (now - self._last_frm_t) < 1.0
        lost_sec  = float(self.get_parameter("detection_lost_sec").value)
        det = self._last_det if (self._last_det_t > 0 and now - self._last_det_t < lost_sec) else None
        cmd, state = self._ctrl.update(det, has_frame)
        self._cmd_pub.publish(cmd)
        if det:
            self.get_logger().info(
                f"[{state.name}]  r={det.radius_px:.0f}px  x={det.x_err:+.2f}"
                f"  c={det.confidence:.2f}  lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}",
                throttle_duration_sec=0.5,
            )
        else:
            self.get_logger().info(
                f"[{state.name}]  lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}",
                throttle_duration_sec=0.5,
            )

    def _warn_no_frames(self) -> None:
        if self._last_frm_t == 0 or time.monotonic() - self._last_frm_t > 4.0:
            self.get_logger().warn(
                "No camera frames for 4 s – check image_topic and QoS.",
                throttle_duration_sec=4.0,
            )

    # ── action server ─────────────────────────────────────────────────────────

    def _goal_cb(self, _goal) -> GoalResponse:
        if self._goal_busy:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        self._goal_busy = True
        self._active    = True

        deadline = (time.monotonic() + float(req.max_duration_sec)) if req.max_duration_sec > 0 else None
        hold = max(1, int(self.get_parameter("success_hold_ticks").value))
        close_times: collections.deque = collections.deque()
        entered_track = False
        track_lost_t:  Optional[float] = None

        RC = FollowBall.Result
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._deactivate()
                    goal_handle.canceled(RC(success=False, reason_code=self.RC_CANCEL, message="canceled"))
                    return RC(success=False, reason_code=self.RC_CANCEL, message="canceled")

                if deadline and time.monotonic() > deadline:
                    self._deactivate()
                    goal_handle.abort(RC(success=False, reason_code=self.RC_TIMEOUT, message="timeout"))
                    return RC(success=False, reason_code=self.RC_TIMEOUT, message="timeout")

                now      = time.monotonic()
                lost_sec = float(self.get_parameter("detection_lost_sec").value)
                fresh    = self._last_det_t > 0 and now - self._last_det_t < lost_sec

                # FindBall mode: succeed as soon as ball is visible
                if req.stop_when_found and fresh:
                    self._deactivate()
                    res = RC(success=True, reason_code=self.RC_OK, message="ball_found")
                    goal_handle.succeed(res)
                    return res

                # FollowBall mode with fail_on_lost: abort if tracking lost for too long
                if float(req.fail_on_lost_sec) > 0:
                    if fresh:
                        entered_track = True
                        track_lost_t  = None
                    elif entered_track:
                        if track_lost_t is None:
                            track_lost_t = now
                        elif now - track_lost_t > float(req.fail_on_lost_sec):
                            self._deactivate()
                            res = RC(success=False, reason_code=self.RC_LOST, message="ball_lost")
                            goal_handle.abort(res)
                            return res

                # FollowBall success: ball radius held at stop_radius for `hold` ticks within 1 s
                if not req.stop_when_found and fresh:
                    stop_r = float(self.get_parameter("stop_radius_px").value)
                    det    = self._last_det
                    if det and det.radius_px >= stop_r:
                        close_times.append(now)
                    while close_times and now - close_times[0] > 1.0:
                        close_times.popleft()
                    if len(close_times) >= hold:
                        self._deactivate()
                        res = RC(success=True, reason_code=self.RC_OK, message="close_enough")
                        goal_handle.succeed(res)
                        return res

                fb           = FollowBall.Feedback()
                fb.state     = int(self._ctrl.state)
                fb.x_error   = float(self._last_det.x_err)    if self._last_det else 0.0
                fb.radius_px = float(self._last_det.radius_px) if self._last_det else 0.0
                goal_handle.publish_feedback(fb)
                time.sleep(0.08)

        finally:
            self._active    = False
            self._goal_busy = False
            self._cmd_pub.publish(Twist())

        goal_handle.abort(RC(success=False, reason_code=self.RC_ABORT, message="shutdown"))
        return RC(success=False, reason_code=self.RC_ABORT, message="shutdown")

    def _deactivate(self) -> None:
        self._active = False
        self._cmd_pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = BallFollowerNode()
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
