#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist

from detector import Detection


def _smoothstep(t: float) -> float:
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


class State(IntEnum):
    SEARCH   = 0
    TRACK    = 1
    APPROACH = 2
    STOP     = 3


@dataclass
class ControllerConfig:
    # Motion limits
    forward_speed: float = 0.72
    stop_radius_px: float = 40.0
    detection_lost_sec: float = 1.5
    # Differential drive: both wheels forward, L/R ratio steers → cmd_vel
    wheel_base: float = 2.0
    max_linear: float = 1.0
    invert_linear: bool = True
    side_gain: float = 1.02
    mix_max: float = 0.68
    min_wheel_fwd: float = 0.12
    side_slowdown_gain: float = 0.50
    side_slowdown_min: float = 0.40
    # Nonlinear lateral error: sign(e) * |e|^err_exp
    err_exp: float = 0.85
    # Approach brake zone just before stop_radius
    brake_band_px: float = 14.0
    turn_blend_min: float = 0.45
    # EMA smoothing on published cmd_vel
    smooth_alpha: float = 0.36
    # Burst search: spin in place when ball is lost
    burst_speed: float = 10.0
    burst_on_sec: float = 0.45
    burst_off_sec: float = 0.90


class BallController:
    """Pure-Python control state machine.

    Call update() at a fixed rate; it returns a Twist and the current State.
    The caller publishes the Twist to cmd_vel.
    """

    def __init__(self, cfg: ControllerConfig) -> None:
        self._cfg = cfg
        self.state = State.SEARCH

        self._search_dir: float = 1.0
        self._ever_seen: bool = False
        self._burst_on: bool = True
        self._burst_start: float = time.monotonic()

        self._ema_lin: float = 0.0
        self._ema_ang: float = 0.0
        self._prev_had_det: bool = False

    def update_config(self, cfg: ControllerConfig) -> None:
        self._cfg = cfg

    def update(self, det: Optional[Detection], has_frame: bool) -> tuple[Twist, State]:
        """Compute next cmd_vel.

        det:       most recent Detection, or None (ball not visible / stale).
        has_frame: True if a camera frame arrived recently (~1 s).
        """
        if not has_frame:
            self._ema_lin = 0.0
            self._ema_ang = 0.0
            return Twist(), State.SEARCH

        now = time.monotonic()
        cfg = self._cfg

        if det is None:
            if self._prev_had_det:      # just lost ball – don't carry forward momentum
                self._ema_lin = 0.0
                self._ema_ang = 0.0
            self._prev_had_det = False
            raw = self._search_cmd(cfg, now)
            state = State.SEARCH
        else:
            self._prev_had_det = True
            self._ever_seen = True
            if abs(det.x_err) > 0.05:
                self._search_dir = 1.0 if det.x_err > 0 else -1.0

            if det.radius_px >= cfg.stop_radius_px:
                raw = Twist()
                state = State.STOP
            else:
                raw, in_brake = self._track_cmd(det, cfg)
                state = State.APPROACH if in_brake else State.TRACK

        self.state = state
        self._apply_ema(raw, cfg.smooth_alpha)
        return raw, state

    # ── private ──────────────────────────────────────────────────────────────

    def _search_cmd(self, cfg: ControllerConfig, now: float) -> Twist:
        elapsed = now - self._burst_start
        if self._burst_on:
            if elapsed >= cfg.burst_on_sec:
                self._burst_on = False
                self._burst_start = now
        else:
            if elapsed >= cfg.burst_off_sec:
                self._burst_on = True
                self._burst_start = now

        cmd = Twist()
        if self._burst_on:
            direction = self._search_dir if self._ever_seen else 1.0
            cmd.angular.z = direction * cfg.burst_speed
        return cmd

    def _track_cmd(self, det: Detection, cfg: ControllerConfig) -> tuple[Twist, bool]:
        err_abs = float(np.clip(abs(det.x_err), 0.0, 1.0))
        slowdown = float(np.clip(1.0 - cfg.side_slowdown_gain * err_abs, cfg.side_slowdown_min, 1.0))
        base = float(np.clip(cfg.forward_speed * slowdown, 0.08, 1.0))

        cmd = self._diff_cmd(det, base, cfg)

        in_brake = False
        if cfg.brake_band_px > 0 and cfg.stop_radius_px > 0:
            low = max(0.0, cfg.stop_radius_px - cfg.brake_band_px)
            if det.radius_px >= low:
                t = (det.radius_px - low) / cfg.brake_band_px
                sm = _smoothstep(t)
                blend_lin = 1.0 - sm
                blend_ang = cfg.turn_blend_min + (1.0 - cfg.turn_blend_min) * (1.0 - sm)
                cmd.linear.x *= blend_lin
                cmd.angular.z *= blend_ang
                in_brake = True

        return cmd, in_brake

    def _diff_cmd(self, det: Detection, base: float, cfg: ControllerConfig) -> Twist:
        exp = float(np.clip(cfg.err_exp, 0.45, 1.2))
        err_s = float(np.sign(det.x_err) * (abs(det.x_err) ** exp))
        mix = float(np.clip(cfg.side_gain * err_s, -cfg.mix_max, cfg.mix_max))
        v_l = base * (1.0 - mix)
        v_r = base * (1.0 + mix)

        mn = min(v_l, v_r)
        if mn < cfg.min_wheel_fwd:
            d = cfg.min_wheel_fwd - mn
            v_l += d
            v_r += d
        mx = max(v_l, v_r)
        if mx > 1.0:
            s = 1.0 / mx
            v_l *= s
            v_r *= s

        wb = max(0.05, cfg.wheel_base)
        mv = max(0.05, cfg.max_linear)
        v_c = mv * (v_l + v_r) * 0.5
        w_c = mv * (v_r - v_l) / wb

        cmd = Twist()
        cmd.linear.x = -v_c if cfg.invert_linear else v_c
        cmd.angular.z = w_c
        return cmd

    def _apply_ema(self, cmd: Twist, alpha: float) -> None:
        self._ema_lin += alpha * (cmd.linear.x - self._ema_lin)
        self._ema_ang += alpha * (cmd.angular.z - self._ema_ang)
        cmd.linear.x = self._ema_lin
        cmd.angular.z = self._ema_ang

