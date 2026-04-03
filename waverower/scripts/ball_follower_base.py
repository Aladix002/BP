#!/usr/bin/env python3
"""Sledovanie lopty: OpenCV HSV + riadenie cez časovač (inšp. joshnewans/follow_ball).

Callback len počíta polohu lopty; Twist sa posiela fixnou frekvenciou s vyhladením.

Parametre: image_topic, cmd_topic, ball_color, subscribe_compressed, image_use_best_effort_qos,
  control_rate_hz, filter_alpha, rcv_timeout_secs (bez detekcie = hľadanie),
  max_radius_px (zahodiť príliš veľké biele plochy), angular_kp, linear_kp, ...
"""

import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def _image_qos_reliable():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


# feedback state (FollowBall.action feedback.state)
ST_IDLE, ST_SEARCH, ST_TRACK, ST_STOP = 0, 1, 2, 3


class BallFollowerBase(Node):
    """Jadro sledovania; BallFollower = vždy aktívne, Action variant = len počas goal."""
    def __init__(self) -> None:
        super().__init__("ball_follower")

        self.declare_parameter("image_topic", "/camera/camera_node/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("angular_kp", 0.55)
        self.declare_parameter("linear_kp", 0.35)
        self.declare_parameter("target_radius_px", 80.0)
        self.declare_parameter("min_radius_px", 15.0)
        # Kontúry s väčším obkolesujúcim kruhom ako táto hodnota nie sú typická lopta v zábere (sedák, skrinka…)
        self.declare_parameter("max_radius_px", 100.0)
        self.declare_parameter("linear_max", 0.15)
        self.declare_parameter("angular_max", 1.0)
        self.declare_parameter("stop_if_lost", True)
        self.declare_parameter("search_speed", 0.55)
        self.declare_parameter("min_contour_area", 80)
        self.declare_parameter("ball_color", "white")
        self.declare_parameter("subscribe_compressed", False)
        self.declare_parameter("image_use_best_effort_qos", True)

        # follow_ball štýl: výstup oddelený od FPS kamery
        self.declare_parameter("control_rate_hz", 25.0)
        # filtered = alpha*old + (1-alpha)*new; vyššie alpha = viac vyhladenia, väčšie oneskorenie
        self.declare_parameter("filter_alpha", 0.45)
        # Ak počas tohto času neprišla platná detekcia → režim hľadania (s)
        self.declare_parameter("rcv_timeout_secs", 0.22)
        # Normalizovaný „polomer" (r / (šírka/2)): pod týmto prahom považuj loptu za dosť veľkú → menej vpred
        self.declare_parameter("max_approach_radius_norm", 0.22)
        # Základ prahu zastavenia (px); skutočný prah = stop_radius_px * stop_radius_scale
        self.declare_parameter("stop_radius_px", 72.0)
        # Koeficient (napr. 1.5 = lopta musí byť v zábere ~1,5× väčšia než samotný stop_radius_px)
        self.declare_parameter("stop_radius_scale", 1.5)
        # Ak robot pri korekcii smeru točí opačne ako očakávaš, nastav True
        self.declare_parameter("invert_angular", False)
        # Zrkadliť obrázok vodorovne (opačná „ľavá/pravá" ak je kamera otočená / mirror)
        self.declare_parameter("mirror_camera_x", False)
        # Normalizovaný x_err prah: kým |x_err| > center_tol, len otáčame; potom aj vpred
        self.declare_parameter("center_tol", 0.12)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self._search_dir = 1.0
        self._frame_count = 0
        self._warned_no_frames = False

        # Merania; pri zlyhaní snímku držíme posledné hodnoty až do rcv_timeout (krátke výpadky)
        self._last_det_time = 0.0
        self._raw_x_err = 0.0
        self._raw_radius_px = 0.0
        self._img_w = 640
        self._img_h = 480

        # Vyhladené stavy (aktualizuje časovač)
        self._filt_x_err = 0.0
        self._filt_radius_norm = 0.0

        self._following_active = True
        self._goal_stopped_close = False
        self._goal_color_override: Optional[str] = None
        self._last_fb_state = ST_IDLE
        self._last_fb_x_err = 0.0
        self._last_fb_radius_px = 0.0

        use_be = self.get_parameter("image_use_best_effort_qos").get_parameter_value().bool_value
        qos = rclpy.qos.qos_profile_sensor_data if use_be else _image_qos_reliable()
        qos_label = "BestEffort" if use_be else "Reliable/10"
        use_comp = self.get_parameter("subscribe_compressed").get_parameter_value().bool_value

        if "compressed" in image_topic or use_comp:
            t = image_topic if "compressed" in image_topic else image_topic.rstrip("/") + "/compressed"
            self.create_subscription(CompressedImage, t, self._compressed_cb, qos)
            comp_l = "compressed "
        else:
            self.create_subscription(Image, image_topic, self._image_cb, qos)
            comp_l = ""

        rate = float(self.get_parameter("control_rate_hz").get_parameter_value().double_value)
        period = 1.0 / max(rate, 1.0)
        self.create_timer(period, self._control_tick)

        self.get_logger().info(
            f"BallFollower: {comp_l}{image_topic}, QoS={qos_label}, "
            f"control={rate:.0f}Hz, farba={self.get_parameter('ball_color').get_parameter_value().string_value}, "
            f"→ {cmd_topic}"
        )
        self.create_timer(4.0, self._warn_if_no_images)

    def _ball_color_effective(self) -> str:
        if self._goal_color_override:
            return self._goal_color_override
        return self.get_parameter("ball_color").get_parameter_value().string_value

    def _warn_if_no_images(self) -> None:
        if self._warned_no_frames or self._frame_count > 0:
            return
        self._warned_no_frames = True
        be = self.get_parameter("image_use_best_effort_qos").get_parameter_value().bool_value
        self.get_logger().warn(
            "Za 4 s neprišiel žiadny obrázok. Skús: "
            f"ros2 param set /ball_follower image_use_best_effort_qos {'false' if be else 'true'} "
            f"alebo skontroluj topic."
        )

    def _compressed_cb(self, msg: CompressedImage) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        if self.get_parameter("mirror_camera_x").get_parameter_value().bool_value:
            frame = cv2.flip(frame, 1)
        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info(f"Prvý obrázok (compressed): {frame.shape[1]}x{frame.shape[0]} px")
        self._vision_update(frame)

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if frame.ndim == 3 and frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            except Exception as e:
                self.get_logger().warn(f"cv_bridge: {e}")
                return
        if self.get_parameter("mirror_camera_x").get_parameter_value().bool_value:
            frame = cv2.flip(frame, 1)
        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info(f"Prvý obrázok: {frame.shape[1]}x{frame.shape[0]} px")
        self._vision_update(frame)

    def _hsv_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        c = (color or "yellow").strip().lower()
        if c == "white":
            lower = np.array([0, 0, 120], dtype=np.uint8)
            upper = np.array([180, 80, 255], dtype=np.uint8)
            return cv2.inRange(hsv, lower, upper)
        if c == "orange":
            lower = np.array([5, 100, 100], dtype=np.uint8)
            upper = np.array([25, 255, 255], dtype=np.uint8)
            return cv2.inRange(hsv, lower, upper)
        m1 = cv2.inRange(hsv, np.array([15, 40, 80], dtype=np.uint8), np.array([45, 255, 255], dtype=np.uint8))
        m2 = cv2.inRange(hsv, np.array([0, 40, 80], dtype=np.uint8), np.array([15, 255, 255], dtype=np.uint8))
        return cv2.bitwise_or(m1, m2)

    def _detect_ball(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        scale = 0.5
        small = cv2.resize(frame, (int(w * scale), int(h * scale)))
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        color = self._ball_color_effective()
        mask = self._hsv_mask(hsv, color)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        min_area = int(self.get_parameter("min_contour_area").get_parameter_value().integer_value)
        max_r = float(self.get_parameter("max_radius_px").get_parameter_value().double_value)
        tgt_r = float(self.get_parameter("target_radius_px").get_parameter_value().double_value)

        candidates: list[tuple[float, float, float, float]] = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.35:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(c)
            r_full = float(radius / scale)
            if r_full > max_r:
                continue
            # Najbližšia veľkosť k očakávanej lopte — nie „najväčšia biela plocha" (sedák, podlaha)
            score = abs(r_full - tgt_r)
            candidates.append((score, circularity, cx / scale, cy / scale, r_full))

        if not candidates:
            return None
        # Primárne najmenšia odchýlka od target_radius; pri remíze vyššia kruhovitosť
        candidates.sort(key=lambda t: (t[0], -t[1]))
        _, _, cx, cy, r_full = candidates[0]
        return (cx, cy, r_full)

    def _vision_update(self, frame: np.ndarray) -> None:
        """Len detekcia + uloženie meraní — žiadny publish."""
        h, w = frame.shape[:2]
        self._img_w, self._img_h = w, h
        result = self._detect_ball(frame)
        min_r = self.get_parameter("min_radius_px").get_parameter_value().double_value

        if result is None or result[2] < min_r:
            return

        cx, cy, radius = result
        x_err = (cx - w / 2.0) / (w / 2.0)
        if abs(x_err) > 0.08:
            self._search_dir = -1.0 if x_err > 0 else 1.0

        self._raw_x_err = float(x_err)
        self._raw_radius_px = float(radius)
        self._last_det_time = time.monotonic()

    def _control_tick(self) -> None:
        if not self._following_active:
            self._pub.publish(Twist())
            self._last_fb_state = ST_IDLE
            return

        now = time.monotonic()
        timeout = float(self.get_parameter("rcv_timeout_secs").get_parameter_value().double_value)
        alpha = float(self.get_parameter("filter_alpha").get_parameter_value().double_value)
        alpha = float(np.clip(alpha, 0.0, 0.98))

        tgt_r = self.get_parameter("target_radius_px").get_parameter_value().double_value
        kp_ang = self.get_parameter("angular_kp").get_parameter_value().double_value
        kp_lin = self.get_parameter("linear_kp").get_parameter_value().double_value
        max_lin = self.get_parameter("linear_max").get_parameter_value().double_value
        max_ang = self.get_parameter("angular_max").get_parameter_value().double_value
        search_spd = self.get_parameter("search_speed").get_parameter_value().double_value
        max_rn = self.get_parameter("max_approach_radius_norm").get_parameter_value().double_value
        stop_r = float(self.get_parameter("stop_radius_px").get_parameter_value().double_value)
        stop_scale = float(self.get_parameter("stop_radius_scale").get_parameter_value().double_value)
        stop_scale = max(stop_scale, 0.01)
        effective_stop_r = (stop_r * stop_scale) if stop_r > 0.0 else 0.0
        invert_a = self.get_parameter("invert_angular").get_parameter_value().bool_value
        w = max(self._img_w, 1)
        cmd = Twist()

        fresh = self._last_det_time > 0.0 and (now - self._last_det_time <= timeout)

        if not fresh:
            # Stratili sme loptu (alebo ešte žiadna detekcia)
            self._last_fb_state = ST_SEARCH
            self._last_fb_x_err = float(self._filt_x_err)
            self._last_fb_radius_px = float(self._raw_radius_px)
            cmd.angular.z = self._search_dir * search_spd
            if invert_a:
                cmd.angular.z *= -1.0
            self._pub.publish(cmd)
            self.get_logger().info(
                "Hľadám loptu…",
                throttle_duration_sec=2.0,
            )
            return

        # Normalizovaná veľkosť v obraze (0…~0.5)
        r_norm = self._raw_radius_px / (w / 2.0)

        self._filt_x_err = alpha * self._filt_x_err + (1.0 - alpha) * self._raw_x_err
        self._filt_radius_norm = alpha * self._filt_radius_norm + (1.0 - alpha) * r_norm

        x_e = self._filt_x_err
        r_px_smooth = float(self._filt_radius_norm * (w / 2.0))
        if effective_stop_r > 0.0 and r_px_smooth >= effective_stop_r:
            self._goal_stopped_close = True
            self._last_fb_state = ST_STOP
            self._last_fb_x_err = float(x_e)
            self._last_fb_radius_px = float(self._raw_radius_px)
            self._pub.publish(cmd)
            self.get_logger().info(
                f"Zastavené – lopta veľká v zábere (r≈{r_px_smooth:.0f}px ≥ {effective_stop_r:.0f}px "
                f"= {stop_r:.0f}×{stop_scale:.2f})",
                throttle_duration_sec=0.8,
            )
            return

        r_err = tgt_r - self._raw_radius_px
        center_tol = float(self.get_parameter("center_tol").get_parameter_value().double_value)
        sgn = -1.0 if not invert_a else 1.0
        cmd.angular.z = float(np.clip(sgn * kp_ang * x_e, -max_ang, max_ang))

        if abs(x_e) > center_tol:
            # Lopta nie je vycentrovaná — iba otáčame, nepôjdeme vpred
            cmd.linear.x = 0.0
            phase = "centrujem"
        else:
            # Vycentrovaná — ideme za loptou (ak nie je príliš blízko)
            if self._filt_radius_norm < max_rn:
                cmd.linear.x = float(np.clip(kp_lin * r_err / tgt_r, -max_lin, max_lin))
            else:
                cmd.linear.x = 0.0
            phase = "sledujem"

        self._last_fb_state = ST_TRACK
        self._last_fb_x_err = float(x_e)
        self._last_fb_radius_px = float(self._raw_radius_px)

        self._pub.publish(cmd)
        self.get_logger().info(
            f"[{phase}]  x_err={x_e:+.2f}  r={self._raw_radius_px:.0f}px  "
            f"cmd lin={cmd.linear.x:+.3f} ang={cmd.angular.z:+.3f}",
            throttle_duration_sec=0.4,
        )
