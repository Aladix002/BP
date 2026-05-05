#!/usr/bin/env python3
# Porovnanie 3 behov v jednom grafe a jednom CSV:
# - bez korekcie
# - IMU
# - optical flow
#
# Z kazdeho CSV sa vezme stredne okno (default 5 s), signal |wz| sa vyhladi
# a linearne interpoluje na spolocnu casovu os.

import argparse
import bisect
import csv
import math
import statistics
from pathlib import Path


def _fcell(x: str) -> float:
    x = (x or "").strip()
    if not x:
        return float("nan")
    try:
        return float(x)
    except ValueError:
        return float("nan")


def load_signal(csv_file: Path, use_imu_raw: bool):
    t = []
    imu = []
    yaw_f = []

    with open(csv_file, newline="", encoding="utf-8") as fp:
        r = csv.DictReader(fp)
        cols = set(r.fieldnames or [])
        if "imu_wz_rad_s" not in cols and "yaw_filt_rad_s" not in cols:
            raise SystemExit(
                f"CSV {csv_file} nema imu_wz_rad_s ani yaw_filt_rad_s. "
                "Pouzi log_imu_drive_csv.py."
            )
        for row in r:
            t.append(_fcell(row.get("t_sec", "")))
            imu.append(_fcell(row.get("imu_wz_rad_s", "")))
            yaw_f.append(_fcell(row.get("yaw_filt_rad_s", "")))

    pick_yaw = (not use_imu_raw) and any(math.isfinite(v) for v in yaw_f)
    sig = yaw_f if pick_yaw else imu

    out_t = []
    out_s = []
    for ti, si in zip(t, sig):
        if math.isfinite(ti) and math.isfinite(si):
            out_t.append(ti)
            out_s.append(si)
    if not out_t:
        raise SystemExit(f"CSV {csv_file} nema platne data po filtrovani.")
    return out_t, out_s


def middle_window(t, s, window_sec: float):
    t0 = t[0]
    t1 = t[-1]
    dur = t1 - t0
    if dur <= 0.0:
        raise SystemExit("Neplatny casovy rozsah v CSV.")
    if dur <= window_sec:
        start_s = t0
        end_s = t1
    else:
        mid = 0.5 * (t0 + t1)
        half = 0.5 * window_sec
        start_s = mid - half
        end_s = mid + half

    ot = []
    os = []
    for ti, si in zip(t, s):
        if start_s <= ti <= end_s:
            ot.append(ti - start_s)  # relativny cas od 0
            os.append(si)
    if len(ot) < 2:
        raise SystemExit("Po vybere stredneho okna zostalo malo dat.")
    return ot, os


def smooth_abs_signal(sig, window: int):
    vals = [abs(v) for v in sig]
    if window <= 1 or len(vals) < 3:
        return vals
    if window % 2 == 0:
        window += 1
    half = window // 2
    out = []
    for i in range(len(vals)):
        lo = max(0, i - half)
        hi = min(len(vals), i + half + 1)
        out.append(statistics.fmean(vals[lo:hi]))
    return out


def interp_series(t, y, grid):
    out = []
    for gx in grid:
        if gx <= t[0]:
            out.append(y[0])
            continue
        if gx >= t[-1]:
            out.append(y[-1])
            continue
        i = bisect.bisect_right(t, gx)
        x0, x1 = t[i - 1], t[i]
        y0, y1 = y[i - 1], y[i]
        if x1 <= x0:
            out.append(y0)
        else:
            a = (gx - x0) / (x1 - x0)
            out.append(y0 + a * (y1 - y0))
    return out


def main():
    ap = argparse.ArgumentParser(
        description="Porovnanie odchylky jazdy (bez/IMU/flow) v jednom grafe a jednom CSV."
    )
    ap.add_argument("--no-corr", required=True, type=Path, help="CSV bez korekcie")
    ap.add_argument("--imu", required=True, type=Path, help="CSV IMU korekcie")
    ap.add_argument("--flow", required=True, type=Path, help="CSV optical flow korekcie")
    ap.add_argument("-o", "--output-image", required=True, type=Path, help="Vystupny PNG")
    ap.add_argument("--output-csv", required=True, type=Path, help="Vystupny spojeny CSV")
    ap.add_argument("--window-sec", type=float, default=5.0, help="Dlzka stredneho okna [s]")
    ap.add_argument("--smooth-window", type=int, default=9, help="Okno vyhladenia |wz|")
    ap.add_argument("--y-max", type=float, default=0.2, help="Horna hranica osi y")
    ap.add_argument("--samples", type=int, default=200, help="Pocet bodov spolocnej osi")
    ap.add_argument("--use-imu-raw", action="store_true", help="Pouzi imu_wz_rad_s miesto yaw_filt_rad_s")
    ap.add_argument("--dpi", type=int, default=170)
    args = ap.parse_args()

    for p in (args.no_corr, args.imu, args.flow):
        if not p.is_file():
            raise SystemExit(f"Subor neexistuje: {p}")

    n_t, n_s = load_signal(args.no_corr, args.use_imu_raw)
    i_t, i_s = load_signal(args.imu, args.use_imu_raw)
    f_t, f_s = load_signal(args.flow, args.use_imu_raw)

    n_t, n_s = middle_window(n_t, n_s, args.window_sec)
    i_t, i_s = middle_window(i_t, i_s, args.window_sec)
    f_t, f_s = middle_window(f_t, f_s, args.window_sec)

    n_y = smooth_abs_signal(n_s, args.smooth_window)
    i_y = smooth_abs_signal(i_s, args.smooth_window)
    f_y = smooth_abs_signal(f_s, args.smooth_window)

    common_end = min(n_t[-1], i_t[-1], f_t[-1], max(0.1, args.window_sec))
    m = max(10, int(args.samples))
    dt = common_end / (m - 1)
    grid_t = [k * dt for k in range(m)]

    n_c = interp_series(n_t, n_y, grid_t)
    i_c = interp_series(i_t, i_y, grid_t)
    f_c = interp_series(f_t, f_y, grid_t)

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output_csv, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["t_sec", "no_corr_abs_wz", "imu_abs_wz", "flow_abs_wz"])
        for t, a, b, c in zip(grid_t, n_c, i_c, f_c):
            w.writerow([f"{t:.6f}", f"{a:.8f}", f"{b:.8f}", f"{c:.8f}"])

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise SystemExit("Chyba importu matplotlib. Nainstaluj: python3 -m pip install --user matplotlib") from e

    for style in ("seaborn-v0_8-whitegrid", "ggplot", "bmh"):
        try:
            plt.style.use(style)
            break
        except OSError:
            continue

    fig, ax = plt.subplots(1, 1, figsize=(10.2, 4.8))
    fig.subplots_adjust(bottom=0.14, top=0.90)
    ax.plot(grid_t, n_c, color="#8e8e8e", linewidth=1.8, label="Bez korekcie")
    ax.plot(grid_t, i_c, color="#1f77b4", linewidth=1.8, label="IMU korekcia")
    ax.plot(grid_t, f_c, color="#d35400", linewidth=1.8, label="Optical flow")
    ax.set_xlabel("čas [s]")
    ax.set_ylabel("|wz| [rad/s]")
    ax.set_ylim(0.0, max(0.01, float(args.y_max)))
    ax.legend(loc="upper right", fontsize=9)

    args.output_image.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output_image, dpi=args.dpi, bbox_inches="tight")

    print(f"Ulozene CSV: {args.output_csv.resolve()}")
    print(f"Ulozeny obrazok: {args.output_image.resolve()}")


if __name__ == "__main__":
    main()
