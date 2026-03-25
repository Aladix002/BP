#!/usr/bin/env python3
"""
Benchmark: Lucas-Kanade (sparse) vs Farneback (dense) optical flow.

Zbiera snímky z ROS kamery (alebo video súboru), spustí oba algoritmy
na rovnakých snímkach a vypíše porovnanie výkonu.

Použitie:
  # živá kamera (zberie 200 snímkov):
  python3 benchmark_flow.py

  # video súbor:
  python3 benchmark_flow.py --video /cesta/k/videu.mp4

  # zmeniť počet snímkov a topic:
  python3 benchmark_flow.py --frames 300 --topic /camera/camera_node/image_raw/compressed
"""

import argparse
import sys
import time
import csv
import statistics
from pathlib import Path
from datetime import datetime

import cv2
import numpy as np

# ROS 2 import – voliteľný (nie je potrebný pri --video)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


# ── Konfigurácia algoritmov ───────────────────────────────────────────────────

LK_PARAMS = dict(
    max_corners   = 100,
    quality_level = 0.01,
    min_distance  = 10,
)

LK_FLOW_PARAMS = dict(
    winSize   = (15, 15),
    maxLevel  = 2,
    criteria  = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)

FB_PARAMS = dict(
    pyr_scale  = 0.5,
    levels     = 3,
    winsize    = 15,
    iterations = 3,
    poly_n     = 5,
    poly_sigma = 1.2,
    flags      = 0,
)

MIN_LK_FEATURES = 15   # minimum tracked bodov pre LK
RESIZE_WIDTH    = 320  # zmenšenie snímku (rovnaké ako v C++ node)


# ── Implementácie algoritmov ──────────────────────────────────────────────────

def run_lk(prev_gray: np.ndarray, curr_gray: np.ndarray) -> float | None:
    """Vráti normalizovaný priemer dx ([-1,1]) alebo None ak málo features."""
    prev_pts = cv2.goodFeaturesToTrack(prev_gray, **LK_PARAMS)
    if prev_pts is None or len(prev_pts) < MIN_LK_FEATURES:
        return None

    curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_pts, None,
                                                    **LK_FLOW_PARAMS)
    good_prev = prev_pts[status.ravel() == 1]
    good_curr = curr_pts[status.ravel() == 1]

    if len(good_prev) < MIN_LK_FEATURES:
        return None

    dx_values = good_curr[:, 0, 0] - good_prev[:, 0, 0]
    return float(np.mean(dx_values)) / curr_gray.shape[1]


def run_farneback(prev_gray: np.ndarray, curr_gray: np.ndarray) -> float:
    """Vráti normalizovaný priemer dx ([-1,1])."""
    flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, **FB_PARAMS)
    mean_dx = float(cv2.mean(flow)[0])
    return mean_dx / curr_gray.shape[1]


# ── Meranie na zozname snímkov ────────────────────────────────────────────────

def benchmark(frames: list[np.ndarray]) -> dict:
    """
    Spustí oba algoritmy na všetkých snímkoch a vráti výsledky.
    frames: list of grayscale numpy arrays (už zmenšené na RESIZE_WIDTH).
    """
    results = {
        "lk":  {"times_ms": [], "corrections": [], "skipped": 0},
        "fb":  {"times_ms": [], "corrections": []},
    }

    n_pairs = len(frames) - 1
    print(f"\nMeriam {n_pairs} párov snímkov...\n")

    for i in range(n_pairs):
        prev = frames[i]
        curr = frames[i + 1]

        # Lucas-Kanade
        t0 = time.perf_counter()
        lk_dx = run_lk(prev, curr)
        lk_ms = (time.perf_counter() - t0) * 1000.0

        if lk_dx is not None:
            results["lk"]["times_ms"].append(lk_ms)
            results["lk"]["corrections"].append(lk_dx)
        else:
            results["lk"]["skipped"] += 1

        # Farneback
        t0 = time.perf_counter()
        fb_dx = run_farneback(prev, curr)
        fb_ms = (time.perf_counter() - t0) * 1000.0

        results["fb"]["times_ms"].append(fb_ms)
        results["fb"]["corrections"].append(fb_dx)

        if (i + 1) % 50 == 0:
            print(f"  {i+1}/{n_pairs} párov spracovaných...")

    return results


# ── Výpis výsledkov ───────────────────────────────────────────────────────────

def stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "mean": 0, "min": 0, "max": 0, "std": 0, "p95": 0}
    return {
        "count": len(values),
        "mean":  statistics.mean(values),
        "min":   min(values),
        "max":   max(values),
        "std":   statistics.stdev(values) if len(values) > 1 else 0.0,
        "p95":   float(np.percentile(values, 95)),
    }


def print_report(results: dict, n_frames: int, width: int, height: int):
    lk = results["lk"]
    fb = results["fb"]
    lk_t = stats(lk["times_ms"])
    fb_t = stats(fb["times_ms"])
    lk_c = stats(lk["corrections"])
    fb_c = stats(fb["corrections"])

    sep = "─" * 72

    print(f"\n{'═'*72}")
    print(f"  VÝSLEDKY BENCHMARKU — Optical Flow")
    print(f"{'═'*72}")
    print(f"  Snímky:     {n_frames}  (párov: {n_frames-1})")
    print(f"  Rozlíšenie: {width}×{height} px  (zmenšené na max {RESIZE_WIDTH}px šírku)")
    print(f"  Dátum:      {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'═'*72}\n")

    # čas spracovania
    print(f"  ČAS SPRACOVANIA NA PÁR SNÍMKOV [ms]")
    print(f"  {sep}")
    print(f"  {'Algoritmus':<22} {'Počet':>6} {'Priem':>7} {'Min':>7} {'Max':>7} {'Std':>7} {'P95':>7}")
    print(f"  {sep}")
    print(f"  {'Lucas-Kanade (sparse)':<22} {lk_t['count']:>6} "
          f"{lk_t['mean']:>7.2f} {lk_t['min']:>7.2f} {lk_t['max']:>7.2f} "
          f"{lk_t['std']:>7.2f} {lk_t['p95']:>7.2f}")
    print(f"  {'Farneback (dense)':<22} {fb_t['count']:>6} "
          f"{fb_t['mean']:>7.2f} {fb_t['min']:>7.2f} {fb_t['max']:>7.2f} "
          f"{fb_t['std']:>7.2f} {fb_t['p95']:>7.2f}")
    print(f"  {sep}")

    if lk_t["mean"] > 0:
        ratio = fb_t["mean"] / lk_t["mean"]
        print(f"  Farneback je {ratio:.1f}× pomalší ako Lucas-Kanade")

    # korekcia
    print(f"\n  VYPOČÍTANÁ KOREKCIA angular.z (normalizované dx)")
    print(f"  {sep}")
    print(f"  {'Algoritmus':<22} {'Počet':>6} {'Priem':>8} {'Std':>8} {'|Max|':>8}")
    print(f"  {sep}")
    print(f"  {'Lucas-Kanade (sparse)':<22} {lk_c['count']:>6} "
          f"{lk_c['mean']:>8.4f} {lk_c['std']:>8.4f} {max(abs(lk_c['min']), abs(lk_c['max'])):>8.4f}")
    print(f"  {'Farneback (dense)':<22} {fb_c['count']:>6} "
          f"{fb_c['mean']:>8.4f} {fb_c['std']:>8.4f} {max(abs(fb_c['min']), abs(fb_c['max'])):>8.4f}")
    print(f"  {sep}")

    if lk["skipped"] > 0:
        print(f"\n  LK preskočených párov (málo features): {lk['skipped']}")

    # odporúčanie
    print(f"\n  ODPORÚČANIE")
    print(f"  {sep}")
    if lk_t["mean"] > 0 and lk_t["p95"] < 20:
        print(f"  ✓ Lucas-Kanade: vhodný pre RPi ({lk_t['mean']:.1f} ms priemer, P95={lk_t['p95']:.1f} ms)")
    else:
        print(f"  ✗ Lucas-Kanade: pomalý ({lk_t['mean']:.1f} ms) — skontroluj záťaž systému")

    if fb_t["p95"] < 40:
        print(f"  ✓ Farneback: použiteľný na RPi ({fb_t['mean']:.1f} ms priemer, P95={fb_t['p95']:.1f} ms)")
    else:
        print(f"  ✗ Farneback: príliš pomalý na real-time ({fb_t['mean']:.1f} ms) — zvýš RESIZE_WIDTH alebo znož fps")

    print(f"{'═'*72}\n")


def save_csv(results: dict, path: Path):
    lk_t = results["lk"]["times_ms"]
    fb_t = results["fb"]["times_ms"]
    lk_c = results["lk"]["corrections"]
    fb_c = results["fb"]["corrections"]

    n = max(len(lk_t), len(fb_t))
    # zarovnaj na rovnakú dĺžku
    while len(lk_t) < n:
        lk_t.append(float("nan"))
    while len(lk_c) < n:
        lk_c.append(float("nan"))

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["pair_idx", "lk_ms", "lk_dx_norm", "fb_ms", "fb_dx_norm"])
        for i in range(n):
            writer.writerow([
                i,
                f"{lk_t[i]:.4f}" if i < len(lk_t) else "",
                f"{lk_c[i]:.6f}" if i < len(lk_c) else "",
                f"{fb_t[i]:.4f}" if i < len(fb_t) else "",
                f"{fb_c[i]:.6f}" if i < len(results['fb']['corrections']) else "",
            ])
    print(f"  CSV uložený: {path}")


# ── Zber snímkov z ROS ────────────────────────────────────────────────────────

class FrameCollector(Node):
    def __init__(self, topic: str, target: int):
        super().__init__("flow_benchmark_collector")
        self.target = target
        self.frames: list[np.ndarray] = []
        self._sub = self.create_subscription(
            CompressedImage, topic, self._cb, 10)
        self.get_logger().info(
            f"Zberam {target} snímkov z {topic}...")

    def _cb(self, msg: CompressedImage):
        if len(self.frames) >= self.target:
            return
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if frame is None:
            return
        if frame.shape[1] > RESIZE_WIDTH:
            h = frame.shape[0] * RESIZE_WIDTH // frame.shape[1]
            frame = cv2.resize(frame, (RESIZE_WIDTH, h))
        self.frames.append(frame)
        if len(self.frames) % 50 == 0:
            self.get_logger().info(
                f"  Zozbierané: {len(self.frames)}/{self.target}")

    def done(self) -> bool:
        return len(self.frames) >= self.target


# ── Zber snímkov z video súboru ───────────────────────────────────────────────

def collect_from_video(path: str, target: int) -> tuple[list[np.ndarray], int, int]:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"Chyba: nemôžem otvoriť {path}", file=sys.stderr)
        sys.exit(1)

    frames = []
    orig_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    orig_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Video: {path}  ({orig_w}×{orig_h})")
    print(f"Zberam {target} snímkov...")

    while len(frames) < target:
        ret, frame = cap.read()
        if not ret:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if gray.shape[1] > RESIZE_WIDTH:
            h = gray.shape[0] * RESIZE_WIDTH // gray.shape[1]
            gray = cv2.resize(gray, (RESIZE_WIDTH, h))
        frames.append(gray)

    cap.release()
    print(f"Zozbierané: {len(frames)} snímkov")
    return frames, orig_w, orig_h


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Benchmark LK vs Farneback optical flow")
    parser.add_argument("--video",  type=str, default=None,
                        help="Cesta k video súboru (ak nie je zadané, použije sa ROS kamera)")
    parser.add_argument("--frames", type=int, default=200,
                        help="Počet snímkov (default: 200)")
    parser.add_argument("--topic",  type=str,
                        default="/camera/camera_node/image_raw/compressed",
                        help="ROS topic (default: /camera/camera_node/image_raw/compressed)")
    parser.add_argument("--csv",    type=str, default=None,
                        help="Cesta pre CSV výstup (default: benchmark_<timestamp>.csv)")
    args = parser.parse_args()

    frames: list[np.ndarray] = []
    orig_w, orig_h = RESIZE_WIDTH, RESIZE_WIDTH

    if args.video:
        frames, orig_w, orig_h = collect_from_video(args.video, args.frames)
    else:
        if not HAS_ROS:
            print("Chyba: rclpy nie je dostupné. Použi --video.", file=sys.stderr)
            sys.exit(1)

        rclpy.init()
        collector = FrameCollector(args.topic, args.frames)
        print(f"Čakám na snímky z kamery... (Ctrl+C na prerušenie)")
        print(f"Uisti sa že kamera beží: ros2 launch waverower manual_bringup.launch.py use_camera:=true\n")

        try:
            while rclpy.ok() and not collector.done():
                rclpy.spin_once(collector, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass

        frames = collector.frames
        collector.destroy_node()
        rclpy.shutdown()

    if len(frames) < 2:
        print("Chyba: málo snímkov na benchmark.", file=sys.stderr)
        sys.exit(1)

    frame_h, frame_w = frames[0].shape[:2]
    results = benchmark(frames)
    print_report(results, len(frames), orig_w, orig_h)

    csv_path = Path(args.csv) if args.csv else Path(
        f"benchmark_flow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    save_csv(results, csv_path)


if __name__ == "__main__":
    main()
