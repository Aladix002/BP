#!/usr/bin/env python3
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass
class Detection:
    x_err: float       # [-1, 1] horizontalna chyba, kladna = lopta vpravo od stredu
    radius_px: float   # polomer lopty v pixeloch (full-res)
    confidence: float  # [0, 1] spolahlivos detekcie


@dataclass
class DetectorConfig:
    # HSV rozsah orandzovej farby (OpenCV H: 0-180)
    h_min: int = 10;  h_max: int = 24
    s_min: int = 90;  s_max: int = 255
    v_min: int = 90;  v_max: int = 255
    # Predspracovanie (half-res pre rychlost)
    blur_ksize: int = 11
    erode_iters: int = 2
    dilate_iters: int = 2
    # Prahy kvality kontury
    min_radius: float = 10.0
    max_radius: float = 145.0
    min_circularity: float = 0.76
    min_solidity: float = 0.84
    min_fill_ratio: float = 0.64
    max_area_ratio: float = 0.12      # max podiel plochy obrazu – odfiltruje velke bloby
    max_aspect_ratio: float = 1.20
    min_confidence: float = 0.70
    # Anti-jitter filter – odfiltruje skoky stredu lopty
    max_jump_frac: float = 0.28       # max posun stredu ako zlomok sirky obrazu
    jump_reset_sec: float = 0.45      # po tejto dobe bez detekcie sa historia resetuje


class OrangeDetector:

    def __init__(self, cfg: DetectorConfig) -> None:
        self._cfg = cfg
        self._last_cx: Optional[float] = None  # posledna akceptovana X-suradnica stredu (full-res)
        self._last_det_t: float = 0.0

    def update_config(self, cfg: DetectorConfig) -> None:
        self._cfg = cfg

    def detect(self, frame: np.ndarray) -> Optional[Detection]:
        h, w = frame.shape[:2]
        mask = self._make_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self._on_miss()
            return None

        cfg = self._cfg
        mask_area = float(mask.shape[0] * mask.shape[1])
        best: Optional[tuple[float, float, float]] = None  # (cx_full, r_full, conf)

        for c in contours:
            area = cv2.contourArea(c)
            if area < 50:
                continue
            # Odfiltruj prilis velke bloby (stena, podlaha)
            if cfg.max_area_ratio > 0 and area / mask_area > cfg.max_area_ratio:
                continue
            # Odfiltruj neokruhle tvary podla pomeru stran
            _, _, bw, bh = cv2.boundingRect(c)
            if bw > 0 and bh > 0 and max(bw, bh) / float(min(bw, bh)) > cfg.max_aspect_ratio:
                continue
            # Solidita – pomer plochy ku konvexnemu obalu
            hull = cv2.convexHull(c)
            ha = cv2.contourArea(hull)
            if ha <= 0:
                continue
            solidity = area / ha
            if solidity < cfg.min_solidity:
                continue
            # Kruhovitost
            peri = cv2.arcLength(c, True)
            if peri == 0:
                continue
            circ = 4.0 * np.pi * area / (peri * peri)
            if circ < cfg.min_circularity:
                continue
            # Polomer (half-res → full-res x2)
            (cx_h, _), r_h = cv2.minEnclosingCircle(c)
            r_full = r_h * 2.0
            if r_full < cfg.min_radius or (cfg.max_radius > 0 and r_full > cfg.max_radius):
                continue
            # Fill ratio – podiel plochy kontury voci kruznici
            fill = area / max(1.0, np.pi * r_h * r_h)
            if fill < cfg.min_fill_ratio:
                continue
            # Celkova spolahlivos (vazeny priemer metrik)
            area_norm = float(np.clip(area / (mask_area * 0.06), 0.0, 1.0))
            conf = float(np.clip(
                0.40 * circ + 0.35 * solidity + 0.15 * area_norm + 0.10 * np.clip(fill, 0.0, 1.0),
                0.0, 1.0,
            ))
            if conf < cfg.min_confidence:
                continue
            if best is None or conf > best[2]:
                best = (cx_h * 2.0, r_full, conf)

        if best is None:
            self._on_miss()
            return None

        cx_full, r_full, conf = best

        # Anti-jitter: zahodime detekciu ak stred prilis skocil
        if cfg.max_jump_frac > 0 and self._last_cx is not None:
            if abs(cx_full - self._last_cx) > cfg.max_jump_frac * w:
                self._on_miss()
                return None

        self._last_cx = cx_full
        self._last_det_t = time.monotonic()
        return Detection(
            x_err=(cx_full - w / 2.0) / (w / 2.0),
            radius_px=r_full,
            confidence=conf,
        )

    def make_debug_image(self, frame: np.ndarray, det: Optional[Detection]) -> np.ndarray:
        h, w = frame.shape[:2]
        mask_full = cv2.resize(self._make_mask(frame), (w, h))
        dbg = frame.copy()
        # Zeleny overlay na maskovanej oblasti
        dbg[mask_full > 0] = (dbg[mask_full > 0] * 0.5 + np.array([0, 255, 0]) * 0.5).astype(np.uint8)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        if det is not None:
            cx = int((det.x_err + 1.0) * w / 2.0)
            r  = max(1, int(det.radius_px))
            cv2.circle(dbg, (cx, h // 2), r, (0, 0, 255), 2)
            cv2.circle(dbg, (cx, h // 2), 4, (0, 0, 255), -1)
            cv2.putText(
                dbg,
                f"r={det.radius_px:.0f}px  x={det.x_err:+.2f}  c={det.confidence:.2f}",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
            )
        return dbg

    # ── sukromne metody ───────────────────────────────────────────────────────

    def _make_mask(self, frame: np.ndarray) -> np.ndarray:
        # Spracovanie na half-res pre znizenie CPU zataze
        h, w = frame.shape[:2]
        small = cv2.resize(frame, (w // 2, h // 2))
        cfg = self._cfg
        kv = cfg.blur_ksize
        if kv >= 3 and kv % 2 == 1:
            small = cv2.GaussianBlur(small, (kv, kv), 0)
        hsv  = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([cfg.h_min, cfg.s_min, cfg.v_min], dtype=np.uint8),
            np.array([cfg.h_max, cfg.s_max, cfg.v_max], dtype=np.uint8),
        )
        k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        if cfg.erode_iters  > 0:
            mask = cv2.erode(mask, None, iterations=cfg.erode_iters)
        if cfg.dilate_iters > 0:
            mask = cv2.dilate(mask, None, iterations=cfg.dilate_iters)
        return mask

    def _on_miss(self) -> None:
        # Po uplynom case bez detekcie resetuj historiu (jump filter)
        if self._last_cx is not None and self._last_det_t > 0:
            if time.monotonic() - self._last_det_t > self._cfg.jump_reset_sec:
                self._last_cx = None
