#!/usr/bin/env python3
# Stavovy riadic sledovania gule z detekcie (bbox): hladanie otacanim, sledovanie, priblizenie,
# zastavenie pri velkej gule, kratky "coast" po strate detekcie. Vystup je Twist pre cmd_vel.

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist

from detector import Detection


def _smoothstep(t: float) -> float:
    # Hermitova interpolacia 0..1 pre makke prechody (brake pasma okolo stop_radius).
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


class State(IntEnum):
    SEARCH   = 0   # ziadna gula: burst otacanie
    TRACK    = 1   # gula videnie, riadenie strany a dopredu
    APPROACH = 2   # v brake pasme pred STOP (spomalenie + zmensenie zatacania)
    STOP     = 3   # gula dost velka v obraze -> stat
    COAST    = 4   # kratko drz poslednu linear.x po strate detekcie


@dataclass
class ControllerConfig:
    forward_speed: float = 0.72      # zakladna "rychlost kolies" 0..1 pred mapovanim na twist
    min_radius_px: float = 10.0      # spodna hranica velkosti gule v px (mala = daleko)
    stop_radius_px: float = 40.0     # ak radius >= toto, STOP (blizko ciela)
    detection_lost_sec: float = 1.5  # rezervovane / konzistencia s logovanim
    wheel_base: float = 2.0          # normalizovana "sira" pre v_l, v_r -> angular (nie metre!)
    max_linear: float = 1.0          # skalovanie linear.x z priemeru kolies
    side_gain: float = 1.02        # sila diferencialu podla horizontalnej chyby
    min_wheel_fwd: float = 0.12      # spodny clip kazdeho kolesa (necouvat na mieste prilis)
    err_exp: float = 0.85            # nelinearita |x_err|^exp pre citlivejsi stred
    brake_band_px: float = 14.0      # sirka pasma pred stop_radius kde sa zmensuje rychlost
    turn_blend_min: float = 0.45     # pri plnom brzdeni angular zmenseny na tento podiel
    smooth_alpha: float = 0.36     # EMA na linear.x a angular.z pred publikovanim
    coast_sec: float = 1.0           # ako dlho po strate detekcie drzat _last_coast_lin
    burst_speed: float = 10.0      # velka angular pri burst hladani
    burst_on_sec: float = 0.45       # dlzka pulzu otacania
    burst_off_sec: float = 0.90      # pauza medzi pulzmi


class BallController:

    def __init__(self, cfg: ControllerConfig) -> None:
        self._cfg = cfg
        self.state = State.SEARCH

        self._search_dir: float = 1.0   # smer hladania po strate / podla poslednej chyby
        self._ever_seen: bool = False
        self._burst_on: bool = True
        self._burst_start: float = time.monotonic()

        self._ema_lin: float = 0.0
        self._ema_ang: float = 0.0
        self._prev_had_det: bool = False
        self._last_det_t: float = 0.0
        self._last_coast_lin: float = 0.0

    def update_config(self, cfg: ControllerConfig) -> None:
        self._cfg = cfg

    def update(self, det: Optional[Detection], has_frame: bool) -> tuple[Twist, State]:
        # Bez snimky vynuluj a SEARCH (bezpecnost / ziadny vystup).
        if not has_frame:
            self._ema_lin = 0.0
            self._ema_ang = 0.0
            return Twist(), State.SEARCH

        now = time.monotonic()
        cfg = self._cfg

        if det is None:
            coast_elapsed = now - self._last_det_t
            if self._prev_had_det:
                self._burst_on = True
                self._burst_start = now
            self._prev_had_det = False

            if self._last_det_t > 0 and coast_elapsed < cfg.coast_sec:
                raw = Twist()
                raw.linear.x = self._last_coast_lin
                state = State.COAST
            else:
                raw = self._search_cmd(cfg, now)
                state = State.SEARCH
        else:
            self._prev_had_det = True
            self._ever_seen = True
            self._last_det_t = now
            if abs(det.x_err) > 0.05:
                self._search_dir = 1.0 if det.x_err > 0 else -1.0

            if det.radius_px >= cfg.stop_radius_px:
                raw = Twist()
                state = State.STOP
                self._last_coast_lin = 0.0
            else:
                raw, in_brake = self._track_cmd(det, cfg)
                state = State.APPROACH if in_brake else State.TRACK
                self._last_coast_lin = raw.linear.x

        self.state = state
        self._apply_ema(raw, cfg.smooth_alpha)
        return raw, state

    def _search_cmd(self, cfg: ControllerConfig, now: float) -> Twist:
        # Stridave burst otacanie (on/off) aby sa robot neotacal nekonecne jednym smerom.
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
        cmd = self._diff_cmd(det, cfg.forward_speed, cfg)

        in_brake = False
        if cfg.brake_band_px > 0 and cfg.stop_radius_px > 0:
            low = max(0.0, cfg.stop_radius_px - cfg.brake_band_px)
            if det.radius_px >= low:
                t = (det.radius_px - low) / cfg.brake_band_px
                sm = _smoothstep(t)
                cmd.linear.x  *= 1.0 - sm
                cmd.angular.z *= cfg.turn_blend_min + (1.0 - cfg.turn_blend_min) * (1.0 - sm)
                in_brake = True

        return cmd, in_brake

    def _diff_cmd(self, det: Detection, base: float, cfg: ControllerConfig) -> Twist:
        # x_err -1..1: diferencial v_l/v_r; vacsi radius -> vacsi "side" lebo gula je blizsie.
        exp = float(np.clip(cfg.err_exp, 0.45, 1.2))
        err_s = float(np.sign(det.x_err) * (abs(det.x_err) ** exp))

        r_range = max(1.0, cfg.stop_radius_px - cfg.min_radius_px)
        r_scale = 1.0 + float(np.clip((det.radius_px - cfg.min_radius_px) / r_range, 0.0, 1.0))
        add = float(np.clip(abs(err_s) * cfg.side_gain * r_scale, 0.0, 1.0))

        if err_s >= 0:
            v_l = base - add
            v_r = base + add
        else:
            v_l = base + add
            v_r = base - add

        v_l = float(np.clip(v_l, cfg.min_wheel_fwd, 1.0))
        v_r = float(np.clip(v_r, cfg.min_wheel_fwd, 1.0))

        wb = max(0.05, cfg.wheel_base)
        mv = max(0.05, cfg.max_linear)
        cmd = Twist()
        cmd.linear.x  = mv * (v_l + v_r) * 0.5
        cmd.angular.z = mv * (v_r - v_l) / wb
        return cmd

    def _apply_ema(self, cmd: Twist, alpha: float) -> None:
        # Upravi cmd in-place: vystup je vyhladeny (menej trhania motorov).
        self._ema_lin += alpha * (cmd.linear.x  - self._ema_lin)
        self._ema_ang += alpha * (cmd.angular.z - self._ema_ang)
        cmd.linear.x  = self._ema_lin
        cmd.angular.z = self._ema_ang
