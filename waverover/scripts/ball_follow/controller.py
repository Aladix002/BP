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


# Stavy regulatora
class State(IntEnum):
    SEARCH   = 0  # hlada loptu – otacanie na mieste
    TRACK    = 1  # sleduje loptu – jazda vpred s korekciou
    APPROACH = 2  # priblihovanie – postupne brzdenie
    STOP     = 3  # lopta dost blizko – zastavenie
    COAST    = 4  # lopta stratena – este sekunda vpred pred hladanim


@dataclass
class ControllerConfig:
    # Zakladna rychlost vpred
    forward_speed: float = 0.72
    # Polomer lopty v pixeloch: minimalny a stop
    min_radius_px: float = 10.0
    stop_radius_px: float = 40.0
    # Po kolkych sekundach bez detekcie sa lopta povazuje za stratenu
    detection_lost_sec: float = 1.5
    # Kinematika diferentialneho podvozku
    wheel_base: float = 2.0
    max_linear: float = 1.0
    # Bocne riadenie: side_gain urcuje agresivitu zatacania
    side_gain: float = 1.02
    min_wheel_fwd: float = 0.12   # minimalny dopredu pre vnutorne koleso
    # Nelinearita bocnej chyby: sign(e)*|e|^err_exp  (<1 = citlivejsie pri malych chybach)
    err_exp: float = 0.85
    # Pasmo plynuleho brzdenia pred stop_radius_px
    brake_band_px: float = 14.0
    turn_blend_min: float = 0.45  # min. podiel uhlovej zlozky pocas brzdenia
    # Exponencialny klzavy priemer na vyhladzenie cmd_vel
    smooth_alpha: float = 0.36
    # Coast: po strate lopty este coast_sec sekund ide vpred (potom burst)
    coast_sec: float = 1.0
    # Burst hladanie: kratke impulzy otacania na mieste
    burst_speed: float = 10.0
    burst_on_sec: float = 0.45
    burst_off_sec: float = 0.90


class BallController:

    def __init__(self, cfg: ControllerConfig) -> None:
        self._cfg = cfg
        self.state = State.SEARCH

        self._search_dir: float = 1.0    # smer posledneho otacania pri hladani
        self._ever_seen: bool = False    # ci bola lopta aspon raz videna
        self._burst_on: bool = True
        self._burst_start: float = time.monotonic()

        self._ema_lin: float = 0.0
        self._ema_ang: float = 0.0
        self._prev_had_det: bool = False
        self._last_det_t: float = 0.0        # cas poslednej uspesnej detekcie
        self._last_coast_lin: float = 0.0    # rychlost pri poslednom sledovani (pre coast)

    def update_config(self, cfg: ControllerConfig) -> None:
        self._cfg = cfg

    def update(self, det: Optional[Detection], has_frame: bool) -> tuple[Twist, State]:
        # Bez obrazu nerobi nic
        if not has_frame:
            self._ema_lin = 0.0
            self._ema_ang = 0.0
            return Twist(), State.SEARCH

        now = time.monotonic()
        cfg = self._cfg

        if det is None:
            coast_elapsed = now - self._last_det_t
            if self._prev_had_det:
                # Prave sme stratili loptu – resetuj burst timer aby zacal az po coaste
                self._burst_on = True
                self._burst_start = now
            self._prev_had_det = False

            if self._last_det_t > 0 and coast_elapsed < cfg.coast_sec:
                # Coast faza: ide stale vpred poslednou rychlostou
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
            # Zapamata smer pre burst hladanie
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

    # ── sukromne metody ───────────────────────────────────────────────────────

    def _search_cmd(self, cfg: ControllerConfig, now: float) -> Twist:
        # Striedanie: burst_on_sec otacanie / burst_off_sec pauza
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

        # Plynule brzdenie v pasme tesne pred stop_radius_px
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
        # Nelinearna bocna chyba – tlmi agresivitu pri malych odchylkach
        exp = float(np.clip(cfg.err_exp, 0.45, 1.2))
        err_s = float(np.sign(det.x_err) * (abs(det.x_err) ** exp))

        # Cim vacsie r (blizsia lopta), tym agresivnejsie zatacanie (r_scale 1..2)
        r_range = max(1.0, cfg.stop_radius_px - cfg.min_radius_px)
        r_scale = 1.0 + float(np.clip((det.radius_px - cfg.min_radius_px) / r_range, 0.0, 1.0))
        add = float(np.clip(abs(err_s) * cfg.side_gain * r_scale, 0.0, 1.0))

        # Vonkajsie koleso +add, vnutorne -add (symetricke diferencialne riadenie)
        if err_s >= 0:   # lopta vpravo → prave koleso rychlejsie
            v_l = base - add
            v_r = base + add
        else:            # lopta vlavo → lave koleso rychlejsie
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
        # Exponencialny klzavy priemer – tlmi skokove zmeny z detekcie
        self._ema_lin += alpha * (cmd.linear.x  - self._ema_lin)
        self._ema_ang += alpha * (cmd.angular.z - self._ema_ang)
        cmd.linear.x  = self._ema_lin
        cmd.angular.z = self._ema_ang
