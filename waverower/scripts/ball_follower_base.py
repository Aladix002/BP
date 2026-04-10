#!/usr/bin/env python3
# Detekcia farebnej gule: HSV maska, kontury, PID na uhol, volitelne diferencial L/R cez dva "virtualne" kolesa.
# Stavovy automat: TRACK (sledovanie), SEARCH (rotacia pri strate), burst rezim pri dlhom hladani.

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Float64MultiArray


def _smoothstep01(t: float) -> float:
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


class BallFollowerBase(Node):
    def __init__(self) -> None:
        super().__init__("ball_follower")

        self.declare_parameter("image_topic", "/camera/camera_node/image_raw")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("ball_color", "orange")
        # HSV range for orange ball; tuned for bright tangerine-like orange.
        self.declare_parameter("orange_h_min", 10)
        self.declare_parameter("orange_h_max", 24)
        self.declare_parameter("orange_s_min", 90)
        self.declare_parameter("orange_s_max", 255)
        self.declare_parameter("orange_v_min", 90)
        self.declare_parameter("orange_v_max", 255)
        self.declare_parameter("forward_speed", 0.72)
        self.declare_parameter("angular_speed", 2.0)
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
        self.declare_parameter("forward_confirm_sec", 0.0)
        self.declare_parameter("forward_speed_scale_after_detect", 1.0)
        # PID regulacia pre sledovanie lopty (angular.z)
        self.declare_parameter("pid_kp", 1.5)
        self.declare_parameter("pid_ki", 0.05)
        self.declare_parameter("pid_kd", 0.15)
        self.declare_parameter("pid_error_exponent", 0.75)
        self.declare_parameter("pid_min_turn_abs", 0.30)
        # IBVS adaptivny zisk: kp sa skalie podla radius_px / pid_radius_ref
        # Vacsi radius (lopta blizko) -> vacsi gain -> prudsie zatacanie
        self.declare_parameter("pid_radius_ref",  30.0)   # [px] referencia kde gain = kp
        self.declare_parameter("pid_radius_scale_min", 1.0)  # spodny limit skalovania
        self.declare_parameter("pid_radius_scale_max", 2.0)  # horny limit skalovania
        self.declare_parameter("subscribe_compressed", False)
        self.declare_parameter("image_use_best_effort_qos", True)
        self.declare_parameter("auto_switch_image_topic", True)
        self.declare_parameter("no_frame_stop_only", True)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("search_burst_speed", 10.0)
        self.declare_parameter("burst_on_sec", 0.15)
        self.declare_parameter("burst_off_sec", 0.90)
        self.declare_parameter("detection_max_center_jump_frac", 0.28)
        self.declare_parameter("jump_reset_lost_sec", 0.45)
        self.declare_parameter("detection_lost_sec", 0.6)
        # Plynulost: EMA na cmd_vel (0.2–0.45 typicky; 1.0 = bez filtra)
        self.declare_parameter("cmd_smooth_alpha", 0.30)
        # Pred dosiahnutim stop_radius: v tomto pasme [px] zmierni dopredu + mierne aj zatocenie
        self.declare_parameter("approach_brake_band_px", 14.0)
        # Pri plnom priblizeni (koniec pasma) ostane tato cast otacania (0.35–0.55)
        self.declare_parameter("approach_turn_blend_min", 0.45)
        # Diferencial L/R (obe kolesa dopredu) — musi sediet s motorom (drive_node)
        self.declare_parameter("use_differential_track_cmd", True)
        self.declare_parameter("wheel_base_cmd", 2.0)
        self.declare_parameter("teleop_max_linear_cmd", 1.0)
        self.declare_parameter("cmd_vel_invert_linear", True)
        self.declare_parameter("differential_side_gain", 0.92)
        self.declare_parameter("differential_mix_max", 0.58)
        self.declare_parameter("min_wheel_forward_norm", 0.10)
        # Pri velkej bocnej chybe zmierni base: gain * |x_err|; min = spodna hranica (vyssie = citatelnejsie)
        self.declare_parameter("side_error_slowdown_gain", 0.52)
        self.declare_parameter("side_error_slowdown_min", 0.38)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        cmd_topic   = self.get_parameter("cmd_topic").get_parameter_value().string_value

        self.declare_parameter("debug_image", True)
        self.declare_parameter("debug_mask_compressed", True)
        self.declare_parameter("debug_mask_topic", "/ball_follower/debug_mask/compressed")
        self.declare_parameter("publish_debug_signals", True)
        self.declare_parameter("debug_signals_topic", "/ball_follower/debug_signals")

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self._debug_pub = self.create_publisher(Image, "/ball_follower/debug_image", 1)
        self._debug_mask_pub = self.create_publisher(
            CompressedImage,
            self.get_parameter("debug_mask_topic").get_parameter_value().string_value,
            1,
        )
        self._signals_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("debug_signals_topic").get_parameter_value().string_value,
            10,
        )
        self._conf_pub = self.create_publisher(Float32, "/ball_follower/detection_confidence", 10)

        self._last_det_time = 0.0
        self._last_frame_time = 0.0
        self._last_x_err = 0.0
        self._last_radius = 0.0
        self._last_confidence = 0.0
        self._ever_seen = False
        self._search_dir = 1.0  # +1 vpravo, -1 vlavo pri hlade
        self._last_accept_cx: float | None = None  # px, naposledy akceptovany stred X (pre filter skokov)
        self._seen_streak_start: float | None = None
        self._image_sub = None
        self._current_image_topic = image_topic
        self._image_candidates = []
        self._candidate_idx = 0

        self._following_active = True
        self._burst_phase_start = time.monotonic()
        self._burst_spinning = True

        # PID stav pre angular regulaciu
        self._pid_integral: float = 0.0
        self._pid_last_err: float = 0.0
        self._pid_last_tick_time: float | None = None
        self._pid_reset: bool = True  # True = resetni integral pri dalsom TRACK tiku
        self._cmd_lin_f: float = 0.0
        self._cmd_ang_f: float = 0.0
        self._had_fresh_track: bool = False  # predchadzajuci tick mal platnu detekciu

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

    # Zoznam alternativnych topicov (raw vs compressed, camera vs camera/camera_node) pre auto_switch
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
        # Ak este neprisiel ani jeden frame z tohto topicu, skus dalsi kandidat.
        if self._last_frame_time > 0.0:
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
            self._last_frame_time = time.monotonic()
            self._process(frame)

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}", throttle_duration_sec=5.0)
            return
        self._last_frame_time = time.monotonic()
        self._process(frame)

    def _warn_no_frames(self) -> None:
        # Pouzivame cas posledneho framu (nie pocitadlo) - varuje aj pri neskorsom vypadku kamery
        no_frame = self._last_frame_time == 0.0 or (time.monotonic() - self._last_frame_time) > 4.0
        if no_frame:
            self.get_logger().warn(
                "Za 4s ziadny obrazok - skontroluj image_topic a QoS.",
                throttle_duration_sec=4.0,
            )

    # Pre vybranu farbu vrati binarnu masku v HSV priestore (OpenCV inRange)
    def _hsv_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        c = (color or "white").strip().lower()
        if c == "orange":
            oh_min = int(np.clip(self.get_parameter("orange_h_min").get_parameter_value().integer_value, 0, 179))
            oh_max = int(np.clip(self.get_parameter("orange_h_max").get_parameter_value().integer_value, 0, 179))
            os_min = int(np.clip(self.get_parameter("orange_s_min").get_parameter_value().integer_value, 0, 255))
            os_max = int(np.clip(self.get_parameter("orange_s_max").get_parameter_value().integer_value, 0, 255))
            ov_min = int(np.clip(self.get_parameter("orange_v_min").get_parameter_value().integer_value, 0, 255))
            ov_max = int(np.clip(self.get_parameter("orange_v_max").get_parameter_value().integer_value, 0, 255))
            if oh_min > oh_max:
                oh_min, oh_max = oh_max, oh_min
            if os_min > os_max:
                os_min, os_max = os_max, os_min
            if ov_min > ov_max:
                ov_min, ov_max = ov_max, ov_min
            return cv2.inRange(
                hsv,
                np.array([oh_min, os_min, ov_min], dtype=np.uint8),
                np.array([oh_max, os_max, ov_max], dtype=np.uint8),
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
        # Polovicne rozlisenie = rychlost; open/close odstranuju sum a diery v maske
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
        # Najlepsi kontur podla kruhovosti, solidity, fill; x_err je -1..1 od stredu obrazu
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
        # Volane z obrazoveho callbacku: detekcia + volitelny filter skoku stredu + debug publisher
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

        want_dbg = self.get_parameter("debug_image").get_parameter_value().bool_value
        want_msk = self.get_parameter("debug_mask_compressed").get_parameter_value().bool_value
        mask_full = None
        if want_dbg or want_msk:
            mask, _, _ = self._binary_mask(frame)
            mask_full = cv2.resize(mask, (w, h))
        if want_msk and mask_full is not None:
            ok_j, enc = cv2.imencode(".jpg", mask_full, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
            if ok_j:
                cm = CompressedImage()
                cm.header.stamp = self.get_clock().now().to_msg()
                cm.format = "jpeg"
                cm.data = enc.tobytes()
                self._debug_mask_pub.publish(cm)
        if want_dbg and mask_full is not None:
            dbg = frame.copy()
            dbg[mask_full > 0] = (dbg[mask_full > 0] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
            if result is not None:
                cx = int((result[0] + 1.0) * w / 2.0)
                r = int(result[1])
                cv2.circle(dbg, (cx, h // 2), r, (0, 0, 255), 2)
                cv2.circle(dbg, (cx, h // 2), 4, (0, 0, 255), -1)
            cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
            cv2.putText(
                dbg,
                f"r={self._last_radius:.0f}px  x={self._last_x_err:+.2f}  conf={self._last_confidence:.2f}",
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
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
            self._search_dir = 1.0 if x_err > 0 else -1.0

    def _publish_debug_signals(
        self,
        cmd_lin: float,
        cmd_ang: float,
        *,
        state_id: float,
        err_shaped: float = 0.0,
        pid_integral: float = 0.0,
        d_err: float = 0.0,
        effective_kp: float = 0.0,
        pid_out_unclipped: float = 0.0,
        diff_mix: float = 0.0,
        v_ln: float = 0.0,
        v_rn: float = 0.0,
    ) -> None:
        """Float64MultiArray pre rqt_plot: 0=state 1=x_err 2=r 3=conf 4=err_shaped 5=integral 6=d_err
        7=effective_kp 8=pid_out_pre_clip 9=mix 10=v_ln 11=v_rn 12=cmd_lin 13=cmd_ang.
        state: 0 idle 1 no_frame 2 verify 3 stop 4 track_diff 5 track_pid 6 approach_diff 7 approach_pid 8 search."""
        if not self.get_parameter("publish_debug_signals").get_parameter_value().bool_value:
            return
        msg = Float64MultiArray()
        msg.data = [
            float(state_id),
            float(self._last_x_err),
            float(self._last_radius),
            float(self._last_confidence),
            float(err_shaped),
            float(pid_integral),
            float(d_err),
            float(effective_kp),
            float(pid_out_unclipped),
            float(diff_mix),
            float(v_ln),
            float(v_rn),
            float(cmd_lin),
            float(cmd_ang),
        ]
        self._signals_pub.publish(msg)

    def _tick(self) -> None:
        # Perioda riadenia: stav TRACK/SEARCH, PID na uhol, diferencial alebo twist na cmd_topic
        dbg_state_id = 8.0
        dbg_err_shaped = 0.0
        dbg_d_err = 0.0
        dbg_eff_kp = 0.0
        dbg_pid_out = 0.0
        dbg_mix = 0.0
        dbg_vln = 0.0
        dbg_vrn = 0.0

        if not self._following_active:
            self._pub.publish(Twist())
            self._publish_debug_signals(0.0, 0.0, state_id=0.0)
            return

        fwd_spd = self.get_parameter("forward_speed").get_parameter_value().double_value
        ang_spd = self.get_parameter("angular_speed").get_parameter_value().double_value
        stop_r  = self.get_parameter("stop_radius_px").get_parameter_value().double_value

        now   = time.monotonic()
        lost_timeout = self.get_parameter("detection_lost_sec").get_parameter_value().double_value
        fresh = self._last_det_time > 0.0 and (now - self._last_det_time < lost_timeout)
        has_frames = self._last_frame_time > 0.0 and (now - self._last_frame_time < 1.0)

        cmd = Twist()

        no_frame_stop_only = self.get_parameter("no_frame_stop_only").get_parameter_value().bool_value
        if no_frame_stop_only and not has_frames:
            self._pub.publish(cmd)
            self._publish_debug_signals(0.0, 0.0, state_id=1.0)
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
                self._pub.publish(cmd)
                self._publish_debug_signals(0.0, 0.0, state_id=2.0)
                self.get_logger().info(
                    f"[{state}]  r={self._last_radius:.0f}px  x_err={self._last_x_err:+.2f}"
                    f"  conf={self._last_confidence:.2f}  lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}",
                    throttle_duration_sec=0.5,
                )
                return
            if stop_r > 0 and self._last_radius >= stop_r:
                state = "STOP"
                self._pid_reset = True
                dbg_state_id = 3.0
            else:
                fwd_scale = float(np.clip(
                    self.get_parameter("forward_speed_scale_after_detect").get_parameter_value().double_value,
                    0.10,
                    1.0,
                ))
                err_abs = float(np.clip(abs(self._last_x_err), 0.0, 1.0))
                sd_gain = float(np.clip(
                    self.get_parameter("side_error_slowdown_gain").get_parameter_value().double_value,
                    0.0,
                    1.2,
                ))
                sd_min = float(np.clip(
                    self.get_parameter("side_error_slowdown_min").get_parameter_value().double_value,
                    0.08,
                    1.0,
                ))
                turn_slowdown = float(np.clip(1.0 - sd_gain * err_abs, sd_min, 1.0))
                use_diff = self.get_parameter("use_differential_track_cmd").get_parameter_value().bool_value

                if use_diff:
                    # Obe kolesa dopredu: v_l/v_r v [0,1] ako v drive_node; lopta vpravo (x_err>0) -> vacsie v_r.
                    base = float(np.clip(fwd_spd * fwd_scale * turn_slowdown, 0.08, 1.0))
                    exp = float(np.clip(
                        self.get_parameter("pid_error_exponent").get_parameter_value().double_value,
                        0.45,
                        1.2,
                    ))
                    err_raw = self._last_x_err
                    err_shaped = float(np.sign(err_raw) * (abs(err_raw) ** exp))
                    sg = float(np.clip(
                        self.get_parameter("differential_side_gain").get_parameter_value().double_value,
                        0.0,
                        2.5,
                    ))
                    mm = float(np.clip(
                        self.get_parameter("differential_mix_max").get_parameter_value().double_value,
                        0.05,
                        0.85,
                    ))
                    mix = float(np.clip(sg * err_shaped, -mm, mm))
                    v_ln = base * (1.0 - mix)
                    v_rn = base * (1.0 + mix)
                    mwf = float(np.clip(
                        self.get_parameter("min_wheel_forward_norm").get_parameter_value().double_value,
                        0.0,
                        0.5,
                    ))
                    mn = min(v_ln, v_rn)
                    if mn < mwf:
                        dlt = mwf - mn
                        v_ln += dlt
                        v_rn += dlt
                    mxw = max(v_ln, v_rn)
                    if mxw > 1.0:
                        s = 1.0 / mxw
                        v_ln *= s
                        v_rn *= s

                    state = "TRACK"
                    brake_band = max(0.0, self.get_parameter("approach_brake_band_px").get_parameter_value().double_value)
                    if stop_r > 0.0 and brake_band > 0.0 and self._last_radius < stop_r:
                        low = stop_r - brake_band
                        if self._last_radius >= low:
                            t = (self._last_radius - low) / brake_band
                            sm = _smoothstep01(t)
                            blend_lin = 1.0 - sm
                            tmin = float(np.clip(
                                self.get_parameter("approach_turn_blend_min").get_parameter_value().double_value,
                                0.15,
                                1.0,
                            ))
                            blend_ang = tmin + (1.0 - tmin) * (1.0 - sm)
                            v_ln *= blend_lin
                            v_rn *= blend_lin
                            vm = 0.5 * (v_ln + v_rn)
                            vd = 0.5 * (v_ln - v_rn)
                            vd *= blend_ang
                            v_ln = vm + vd
                            v_rn = vm - vd
                            state = "APPROACH"

                    max_v = max(0.05, self.get_parameter("teleop_max_linear_cmd").get_parameter_value().double_value)
                    wb = max(0.05, self.get_parameter("wheel_base_cmd").get_parameter_value().double_value)
                    invert_lin = self.get_parameter("cmd_vel_invert_linear").get_parameter_value().bool_value
                    v_c = max_v * (v_ln + v_rn) * 0.5
                    w_c = max_v * (v_rn - v_ln) / wb
                    cmd.linear.x = -v_c if invert_lin else v_c
                    cmd.angular.z = w_c
                    self._pid_reset = True
                    dbg_err_shaped = err_shaped
                    dbg_mix = mix
                    dbg_vln = v_ln
                    dbg_vrn = v_rn
                    dbg_state_id = 6.0 if state == "APPROACH" else 4.0
                else:
                    cmd.linear.x = -(fwd_spd * fwd_scale * turn_slowdown)
                    tick_now = time.monotonic()
                    if self._pid_reset or self._pid_last_tick_time is None:
                        dt = 0.0
                        self._pid_integral = 0.0
                        self._pid_last_err = self._last_x_err
                        self._pid_reset = False
                    else:
                        dt = tick_now - self._pid_last_tick_time
                    self._pid_last_tick_time = tick_now

                    err = self._last_x_err
                    exp = float(np.clip(
                        self.get_parameter("pid_error_exponent").get_parameter_value().double_value,
                        0.45,
                        1.2,
                    ))
                    err_shaped = float(np.sign(err) * (abs(err) ** exp))
                    if dt > 0.0:
                        self._pid_integral = float(np.clip(
                            self._pid_integral + err_shaped * dt, -1.0, 1.0
                        ))
                        d_err = (err_shaped - self._pid_last_err) / dt
                    else:
                        d_err = 0.0
                    self._pid_last_err = err_shaped

                    kp = self.get_parameter("pid_kp").get_parameter_value().double_value
                    ki = self.get_parameter("pid_ki").get_parameter_value().double_value
                    kd = self.get_parameter("pid_kd").get_parameter_value().double_value
                    r_ref = max(1.0, self.get_parameter("pid_radius_ref").get_parameter_value().double_value)
                    r_smin = self.get_parameter("pid_radius_scale_min").get_parameter_value().double_value
                    r_smax = self.get_parameter("pid_radius_scale_max").get_parameter_value().double_value
                    radius_scale = float(np.clip(self._last_radius / r_ref, r_smin, r_smax))
                    effective_kp = kp * radius_scale
                    pid_out = effective_kp * err_shaped + ki * self._pid_integral + kd * d_err
                    dbg_err_shaped = err_shaped
                    dbg_d_err = d_err
                    dbg_eff_kp = effective_kp
                    dbg_pid_out = float(pid_out)
                    cmd.angular.z = -float(np.clip(pid_out, -ang_spd, ang_spd))
                    min_turn_abs = max(0.0, self.get_parameter("pid_min_turn_abs").get_parameter_value().double_value)
                    if abs(err_shaped) > 0.06 and min_turn_abs > 0.0:
                        turn_dir = -1.0 if err_shaped > 0 else 1.0
                        cmd.angular.z = float(
                            turn_dir * max(abs(cmd.angular.z), min(min_turn_abs, ang_spd))
                        )

                    brake_band = max(0.0, self.get_parameter("approach_brake_band_px").get_parameter_value().double_value)
                    if stop_r > 0.0 and brake_band > 0.0 and self._last_radius < stop_r:
                        low = stop_r - brake_band
                        if self._last_radius >= low:
                            t = (self._last_radius - low) / brake_band
                            sm = _smoothstep01(t)
                            blend_lin = 1.0 - sm
                            tmin = float(np.clip(
                                self.get_parameter("approach_turn_blend_min").get_parameter_value().double_value,
                                0.15,
                                1.0,
                            ))
                            blend_ang = tmin + (1.0 - tmin) * (1.0 - sm)
                            cmd.linear.x *= blend_lin
                            cmd.angular.z *= blend_ang
                            state = "APPROACH"
                            dbg_state_id = 7.0
                        else:
                            state = "TRACK"
                            dbg_state_id = 5.0
                    else:
                        state = "TRACK"
                        dbg_state_id = 5.0
            else:
            if self._had_fresh_track:
                self._cmd_lin_f = 0.0
                self._cmd_ang_f = 0.0
            self._had_fresh_track = False
            self._seen_streak_start = None
            # Reset PID - loptu sme stratili
            self._pid_reset = True
            self._pid_integral = 0.0
            self._pid_last_tick_time = None
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

        if fresh:
            self._had_fresh_track = True

        # EMA na cmd_vel — plynulejsia jazda; po prechode na SEARCH vyssie reset filtrov
        alpha = float(np.clip(
            self.get_parameter("cmd_smooth_alpha").get_parameter_value().double_value,
            0.05,
            1.0,
        ))
        self._cmd_lin_f += alpha * (cmd.linear.x - self._cmd_lin_f)
        self._cmd_ang_f += alpha * (cmd.angular.z - self._cmd_ang_f)
        cmd.linear.x = self._cmd_lin_f
        cmd.angular.z = self._cmd_ang_f

        self._publish_debug_signals(
            cmd.linear.x,
            cmd.angular.z,
            state_id=dbg_state_id,
            err_shaped=dbg_err_shaped,
            pid_integral=self._pid_integral,
            d_err=dbg_d_err,
            effective_kp=dbg_eff_kp,
            pid_out_unclipped=dbg_pid_out,
            diff_mix=dbg_mix,
            v_ln=dbg_vln,
            v_rn=dbg_vrn,
        )
        self._pub.publish(cmd)
        self.get_logger().info(
            f"[{state}]  r={self._last_radius:.0f}px  x_err={self._last_x_err:+.2f}"
            f"  conf={self._last_confidence:.2f}  lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}",
            throttle_duration_sec=0.5,
        )
