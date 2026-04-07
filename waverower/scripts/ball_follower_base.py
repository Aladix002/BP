#!/usr/bin/env python3
"""Sledovanie lopty - stavovy automat (TRACK / SEARCH_*).

Parametre: image_topic, cmd_topic, ball_color, forward_speed, angular_speed,
stop_radius_px, min_radius_px, max_radius_px, min_circularity,
max_contour_area_ratio, gaussian_blur_ksize, mask_erode_iters, mask_dilate_iters,
max_bbox_aspect_ratio, min_solidity, subscribe_compressed, image_use_best_effort_qos,
detection_max_center_jump_frac (0=vypnuté; zahodí skok stredu medzi snímkami), jump_reset_lost_sec.

Maska inspirovana Shawn Hymel blob_tracker.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32


class BallFollowerBase(Node):
    def __init__(self) -> None:
        super().__init__("ball_follower")

        self.declare_parameter("image_topic", "/camera/camera_node/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("ball_color", "orange")
        self.declare_parameter("forward_speed", 0.20)
        self.declare_parameter("angular_speed", 0.65)
        self.declare_parameter("stop_radius_px", 100.0)
        self.declare_parameter("min_radius_px", 15.0)
        self.declare_parameter("max_radius_px", 110.0)
        self.declare_parameter("min_circularity", 0.68)
        self.declare_parameter("max_contour_area_ratio", 0.18)
        self.declare_parameter("gaussian_blur_ksize", 11)
        self.declare_parameter("mask_erode_iters", 2)
        self.declare_parameter("mask_dilate_iters", 2)
        self.declare_parameter("max_bbox_aspect_ratio", 1.42)
        self.declare_parameter("min_solidity", 0.82)
        self.declare_parameter("min_fill_ratio", 0.58)
        self.declare_parameter("min_detection_confidence", 0.80)
        self.declare_parameter("forward_confirm_sec", 2.0)
        self.declare_parameter("forward_speed_scale_after_detect", 0.75)
        self.declare_parameter("subscribe_compressed", False)
        self.declare_parameter("image_use_best_effort_qos", True)
        self.declare_parameter("auto_switch_image_topic", True)
        self.declare_parameter("no_frame_stop_only", True)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("search_burst_speed", 1.0)
        self.declare_parameter("burst_on_sec", 0.10)
        self.declare_parameter("burst_off_sec", 0.90)
        self.declare_parameter("detection_max_center_jump_frac", 0.28)
        self.declare_parameter("jump_reset_lost_sec", 0.45)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        cmd_topic   = self.get_parameter("cmd_topic").get_parameter_value().string_value

        self.declare_parameter("debug_image", True)

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self._debug_pub = self.create_publisher(Image, "/ball_follower/debug_image", 1)
        self._conf_pub = self.create_publisher(Float32, "/ball_follower/detection_confidence", 10)

        self._last_det_time = 0.0
        self._last_frame_time = 0.0
        self._last_x_err = 0.0
        self._last_radius = 0.0
        self._last_confidence = 0.0
        self._prev_radius = 0.0
        self._ever_seen = False
        self._search_dir = 1.0  # +1 vlavo, -1 vpravo pri hlade
        self._last_accept_cx: float | None = None  # px, naposledy akceptovaný stred X (pre filter skokov)
        self._seen_streak_start: float | None = None
        self._image_sub = None
        self._current_image_topic = image_topic
        self._image_candidates = []
        self._candidate_idx = 0

        self._frame_count      = 0
        self._following_active = True
        self._burst_phase_start = time.monotonic()
        self._burst_spinning = True

        use_be = self.get_parameter("image_use_best_effort_qos").get_parameter_value().bool_value
        qos = rclpy.qos.qos_profile_sensor_data if use_be else QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        use_comp = self.get_parameter("subscribe_compressed").get_parameter_value().bool_value
        self._image_candidates = self._build_image_candidates(image_topic, use_comp)
        self._subscribe_to_image_topic(self._image_candidates[0], qos)

        rate   = max(self.get_parameter("control_rate_hz").get_parameter_value().double_value, 1.0)
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(4.0, self._warn_no_frames)
        self.create_timer(2.0, self._maybe_switch_image_topic)

        self.get_logger().info(
            f"BallFollower ready: topic={self._current_image_topic} color={self.get_parameter('ball_color').get_parameter_value().string_value}"
        )

    def _build_image_candidates(self, image_topic: str, use_comp: bool) -> list[str]:
        base = image_topic.rstrip("/")
        candidates: list[str] = []

        def _add(topic: str) -> None:
            if topic and topic not in candidates:
                candidates.append(topic)

        if "compressed" in base or use_comp:
            _add(base if "compressed" in base else f"{base}/compressed")
            _add(base.replace("/camera/camera_node/image_raw/compressed", "/camera/image_raw/compressed"))
            _add(base.replace("/camera/image_raw/compressed", "/camera/camera_node/image_raw/compressed"))
            _add(base.replace("/camera/camera_node/image_raw/compressed", "/camera/camera_node/image_raw"))
            _add(base.replace("/camera/image_raw/compressed", "/camera/image_raw"))
        else:
            _add(base)
            _add(base.replace("/camera/camera_node/image_raw", "/camera/image_raw"))
            _add(base.replace("/camera/image_raw", "/camera/camera_node/image_raw"))
            _add(f"{base}/compressed")
        return candidates if candidates else [image_topic]

    def _subscribe_to_image_topic(self, topic: str, qos: QoSProfile) -> None:
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
        self._current_image_topic = topic
        if "compressed" in topic:
            self._image_sub = self.create_subscription(CompressedImage, topic, self._compressed_cb, qos)
        else:
            self._image_sub = self.create_subscription(Image, topic, self._image_cb, qos)
        self.get_logger().info(f"Image subscription -> {topic}")

    def _maybe_switch_image_topic(self) -> None:
        auto_switch = self.get_parameter("auto_switch_image_topic").get_parameter_value().bool_value
        if not auto_switch or len(self._image_candidates) < 2:
            return
        # Ak nechodia framy, skus dalsi kandidat.
        if self._frame_count > 0:
            return
        use_be = self.get_parameter("image_use_best_effort_qos").get_parameter_value().bool_value
        qos = rclpy.qos.qos_profile_sensor_data if use_be else QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._candidate_idx = (self._candidate_idx + 1) % len(self._image_candidates)
        self._subscribe_to_image_topic(self._image_candidates[self._candidate_idx], qos)

    def _compressed_cb(self, msg: CompressedImage) -> None:
        arr   = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self._frame_count += 1
            self._last_frame_time = time.monotonic()
            self._process(frame)

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return
        self._frame_count += 1
        self._last_frame_time = time.monotonic()
        self._process(frame)

    def _warn_no_frames(self) -> None:
        if self._frame_count == 0:
            self.get_logger().warn("Za 4s ziadny obrazok - skontroluj image_topic a QoS.")

    def _hsv_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        c = (color or "white").strip().lower()
        if c == "orange":
            return cv2.inRange(
                hsv,
                np.array([8, 120, 80], dtype=np.uint8),
                np.array([30, 255, 255], dtype=np.uint8),
            )
        if c == "pink":
            return cv2.inRange(
                hsv,
                np.array([138, 70, 70], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            )
        if c == "white":
            # Biela lopta ma casto tienenu spodnu cast (nizsie V, o nieco vyssie S).
            # Kombinujeme jasnu bielu + tienovanu bielu.
            bright = cv2.inRange(
                hsv,
                np.array([0, 0, 165], dtype=np.uint8),
                np.array([180, 45, 255], dtype=np.uint8),
            )
            shaded = cv2.inRange(
                hsv,
                np.array([0, 0, 110], dtype=np.uint8),
                np.array([180, 55, 220], dtype=np.uint8),
            )
            return cv2.bitwise_or(bright, shaded)
        # Default fallback: white
        bright = cv2.inRange(
            hsv,
            np.array([0, 0, 165], dtype=np.uint8),
            np.array([180, 45, 255], dtype=np.uint8),
        )
        shaded = cv2.inRange(
            hsv,
            np.array([0, 0, 110], dtype=np.uint8),
            np.array([180, 55, 220], dtype=np.uint8),
        )
        return cv2.bitwise_or(bright, shaded)

    def _binary_mask(self, frame: np.ndarray) -> tuple[np.ndarray, int, int]:
        """Polovicne rozlisenie: blur, HSV, morfologia (blob_tracker styl)."""
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (w // 2, h // 2))
        kv = int(self.get_parameter("gaussian_blur_ksize").get_parameter_value().integer_value)
        if kv >= 3 and kv % 2 == 1:
            small = cv2.GaussianBlur(small, (kv, kv), 0)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        color = self.get_parameter("ball_color").get_parameter_value().string_value
        mask = self._hsv_mask(hsv, color)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        ei = max(0, int(self.get_parameter("mask_erode_iters").get_parameter_value().integer_value))
        di = max(0, int(self.get_parameter("mask_dilate_iters").get_parameter_value().integer_value))
        if ei > 0:
            mask = cv2.erode(mask, None, iterations=ei)
        if di > 0:
            mask = cv2.dilate(mask, None, iterations=di)
        return mask, w, h

    def _detect(self, frame: np.ndarray):
        """Vrati (x_err, radius_px, confidence) alebo None."""
        h, w = frame.shape[:2]
        mask, _, _ = self._binary_mask(frame)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        min_r = self.get_parameter("min_radius_px").get_parameter_value().double_value
        max_r = self.get_parameter("max_radius_px").get_parameter_value().double_value
        circ_thr = float(
            np.clip(self.get_parameter("min_circularity").get_parameter_value().double_value, 0.2, 0.99)
        )
        max_area_r = self.get_parameter("max_contour_area_ratio").get_parameter_value().double_value
        max_ar = self.get_parameter("max_bbox_aspect_ratio").get_parameter_value().double_value
        min_sol = self.get_parameter("min_solidity").get_parameter_value().double_value
        min_fill = self.get_parameter("min_fill_ratio").get_parameter_value().double_value
        min_conf = self.get_parameter("min_detection_confidence").get_parameter_value().double_value
        sh, sw = mask.shape[:2]
        mask_area = float(sw * sh)
        best = None
        for c in contours:
            area = cv2.contourArea(c)
            if area < 50:
                continue
            if max_area_r > 0.0 and (area / mask_area) > max_area_r:
                continue
            _bx, _by, bbw, bbh = cv2.boundingRect(c)
            if bbw > 0 and bbh > 0 and max_ar > 0.0:
                aspect = max(bbw, bbh) / float(min(bbw, bbh))
                if aspect > max_ar:
                    continue
            solidity = 1.0
            if min_sol > 0.0:
                hull = cv2.convexHull(c)
                ha = cv2.contourArea(hull)
                if ha <= 0.0:
                    continue
                solidity = area / ha
                if solidity < min_sol:
                    continue
            peri = cv2.arcLength(c, True)
            if peri == 0:
                continue
            circularity = 4.0 * np.pi * area / (peri * peri)
            if circularity < circ_thr:
                continue
            (cx, _cy), r = cv2.minEnclosingCircle(c)
            r_full = r * 2.0
            if r_full < min_r:
                continue
            if max_r > 0.0 and r_full > max_r:
                continue
            circle_area = float(np.pi * r * r)
            if circle_area <= 1.0:
                continue
            fill_ratio = area / circle_area
            if min_fill > 0.0 and fill_ratio < min_fill:
                continue
            area_norm = float(np.clip(area / (mask_area * 0.06), 0.0, 1.0))
            conf = float(np.clip(0.40 * circularity + 0.35 * solidity + 0.15 * area_norm + 0.10 * np.clip(fill_ratio, 0.0, 1.0), 0.0, 1.0))
            if min_conf > 0.0 and conf < min_conf:
                continue
            if best is None or conf > best[2]:
                best = (cx * 2.0, r_full, conf)

        if best is None:
            return None

        cx_full, r_full, conf = best
        x_err = (cx_full - w / 2.0) / (w / 2.0)
        return x_err, r_full, conf

    def _process(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        now = time.monotonic()
        result = self._detect(frame)

        if result is not None:
            x_err_try, _r, _c = result
            cx_full = (x_err_try + 1.0) * w / 2.0
            jump_lim = self.get_parameter("detection_max_center_jump_frac").get_parameter_value().double_value
            if jump_lim > 0.0 and self._last_accept_cx is not None:
                if abs(cx_full - self._last_accept_cx) > jump_lim * float(w):
                    result = None

        if result is None:
            lost_sec = self.get_parameter("jump_reset_lost_sec").get_parameter_value().double_value
            if (
                self._last_accept_cx is not None
                and self._last_det_time > 0.0
                and (now - self._last_det_time) > max(lost_sec, 0.05)
            ):
                self._last_accept_cx = None

        if self.get_parameter("debug_image").get_parameter_value().bool_value:
            dbg = frame.copy()
            mask, _, _ = self._binary_mask(frame)
            mask_full = cv2.resize(mask, (w, h))
            dbg[mask_full > 0] = (dbg[mask_full > 0] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            if result is not None:
                cx = int((result[0] + 1.0) * w / 2.0)
                r  = int(result[1])
                cv2.circle(dbg, (cx, h // 2), r, (0, 0, 255), 2)
                cv2.circle(dbg, (cx, h // 2), 4, (0, 0, 255), -1)
            cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
            cv2.putText(dbg, f"r={self._last_radius:.0f}px  x={self._last_x_err:+.2f}  conf={self._last_confidence:.2f}",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            try:
                self._debug_pub.publish(self._bridge.cv2_to_imgmsg(dbg, encoding="bgr8"))
            except Exception:
                pass

        if result is None:
            self._last_x_err = 0.0
            self._last_radius = 0.0
            self._last_confidence = 0.0
            self._conf_pub.publish(Float32(data=0.0))
            return
        x_err, radius, conf = result
        self._last_accept_cx = (x_err + 1.0) * w / 2.0
        self._last_x_err    = x_err
        self._last_radius   = radius
        self._last_confidence = conf
        self._last_det_time = now
        if self._seen_streak_start is None:
            self._seen_streak_start = now
        self._ever_seen     = True
        self._conf_pub.publish(Float32(data=float(conf)))
        if abs(x_err) > 0.05:
            self._search_dir = -1.0 if x_err > 0 else 1.0

    def _tick(self) -> None:
        if not self._following_active:
            self._pub.publish(Twist())
            return

        fwd_spd = self.get_parameter("forward_speed").get_parameter_value().double_value
        ang_spd = self.get_parameter("angular_speed").get_parameter_value().double_value
        stop_r  = self.get_parameter("stop_radius_px").get_parameter_value().double_value

        now   = time.monotonic()
        fresh = self._last_det_time > 0.0 and (now - self._last_det_time < 0.3)
        has_frames = self._last_frame_time > 0.0 and (now - self._last_frame_time < 1.0)

        cmd = Twist()

        no_frame_stop_only = self.get_parameter("no_frame_stop_only").get_parameter_value().bool_value
        if no_frame_stop_only and not has_frames:
            self._pub.publish(cmd)
            self.get_logger().warn(
                "[NO_FRAME] stop-only mode (kamera neposiela obraz)",
                throttle_duration_sec=2.0,
            )
            return

        if fresh:
            confirm_sec = max(0.0, self.get_parameter("forward_confirm_sec").get_parameter_value().double_value)
            seen_for = 0.0 if self._seen_streak_start is None else (now - self._seen_streak_start)
            if seen_for < confirm_sec:
                state = f"VERIFY {seen_for:.1f}/{confirm_sec:.1f}s"
                self._prev_radius = self._last_radius
                self._pub.publish(cmd)
                self.get_logger().info(
                    f"[{state}]  r={self._last_radius:.0f}px  x_err={self._last_x_err:+.2f}"
                    f"  conf={self._last_confidence:.2f}  lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}",
                    throttle_duration_sec=0.5,
                )
                return
            if stop_r > 0 and self._last_radius >= stop_r:
                state = "STOP"
            else:
                fwd_scale = float(np.clip(
                    self.get_parameter("forward_speed_scale_after_detect").get_parameter_value().double_value,
                    0.10,
                    1.0,
                ))
                cmd.linear.x  = fwd_spd * fwd_scale
                cmd.angular.z = float(np.clip(-ang_spd * self._last_x_err, -ang_spd, ang_spd))
                state = "TRACK"
            self._prev_radius = self._last_radius
        else:
            self._seen_streak_start = None
            burst_spd = self.get_parameter("search_burst_speed").get_parameter_value().double_value
            on_sec    = self.get_parameter("burst_on_sec").get_parameter_value().double_value
            off_sec   = self.get_parameter("burst_off_sec").get_parameter_value().double_value
            direction = self._search_dir if self._ever_seen else 1.0

            elapsed = now - self._burst_phase_start
            if self._burst_spinning:
                if elapsed >= on_sec:
                    self._burst_spinning    = False
                    self._burst_phase_start = now
                    elapsed = 0.0
            else:
                if elapsed >= off_sec:
                    self._burst_spinning    = True
                    self._burst_phase_start = now
                    elapsed = 0.0

            if self._burst_spinning:
                cmd.angular.z = direction * burst_spd
            state = f"{'SEARCH_D' if self._ever_seen else 'SEARCH_R'} {'ON' if self._burst_spinning else 'off'}"

        self._pub.publish(cmd)
        self.get_logger().info(
            f"[{state}]  r={self._last_radius:.0f}px  x_err={self._last_x_err:+.2f}"
            f"  conf={self._last_confidence:.2f}  lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}",
            throttle_duration_sec=0.5,
        )
