#!/usr/bin/env python3
# Jednoduchy graf odchylky jazdy pre jeden beh.
# Spusti 3x: bez korekcie, IMU, optical flow.
#
# Ockava CSV z log_imu_drive_csv.py:
#  - t_sec
#  - imu_wz_rad_s
#  - yaw_filt_rad_s (volitelne)

import argparse
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


def crop_window(t, s, start_s, end_s):
    ot = []
    os = []
    for ti, si in zip(t, s):
        if ti < start_s:
            continue
        if end_s >= 0.0 and ti > end_s:
            continue
        ot.append(ti)
        os.append(si)
    if not ot:
        raise SystemExit("Po orezani casoveho okna nezostali ziadne data.")
    return ot, os


def metrics(sig):
    a = [abs(x) for x in sig if math.isfinite(x)]
    n = len(a)
    if n == 0:
        return None
    srt = sorted(a)
    p95_i = int(round(0.95 * (n - 1)))
    return {
        "n": n,
        "mean_abs": statistics.fmean(a),
        "rmse": math.sqrt(statistics.fmean(v * v for v in a)),
        "p95_abs": srt[p95_i],
        "max_abs": max(a),
    }


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


def main():
    ap = argparse.ArgumentParser(
        description="Graf odchýlky jazdy pre jeden beh (spusti 3x pre porovnanie režimov)."
    )
    ap.add_argument("csv_file", type=Path, help="CSV z log_imu_drive_csv.py")
    ap.add_argument("-o", "--output", type=Path, help="Uložiť PNG")
    ap.add_argument("--label", default="beh", help="Názov režimu v titulku (napr. bez_korekcie)")
    ap.add_argument("--start-s", type=float, default=0.0, help="Začiatok analýzy [s]")
    ap.add_argument("--end-s", type=float, default=-1.0, help="Koniec analýzy [s], -1 = do konca")
    ap.add_argument("--use-imu-raw", action="store_true", help="Použi imu_wz_rad_s namiesto yaw_filt_rad_s")
    ap.add_argument("--smooth-window", type=int, default=9, help="Veľkosť okna kĺzavého priemeru pre |wz|")
    ap.add_argument("--dpi", type=int, default=160)
    args = ap.parse_args()

    if not args.csv_file.is_file():
        raise SystemExit(f"Subor neexistuje: {args.csv_file}")

    t, s = load_signal(args.csv_file, args.use_imu_raw)
    t, s = crop_window(t, s, args.start_s, args.end_s)
    m = metrics(s)

    print(f"[{args.label}] samples={m['n']} mean|wz|={m['mean_abs']:.5f} rmse={m['rmse']:.5f} p95={m['p95_abs']:.5f} max={m['max_abs']:.5f}")

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise SystemExit("Chyba importu matplotlib. Nainštaluj: python3 -m pip install --user matplotlib") from e

    for style in ("seaborn-v0_8-whitegrid", "ggplot", "bmh"):
        try:
            plt.style.use(style)
            break
        except OSError:
            continue

    fig, ax = plt.subplots(1, 1, figsize=(10, 4.8))
    fig.subplots_adjust(bottom=0.14, top=0.84)

    abs_s = [abs(v) for v in s]
    smooth_abs = smooth_abs_signal(s, max(1, int(args.smooth_window)))
    ax.plot(t, abs_s, color="#f6ad55", linewidth=0.9, alpha=0.45, label="|wz| (merané)")
    ax.plot(t, smooth_abs, color="#d35400", linewidth=1.8, label=f"|wz| (vyhladené, okno={max(1, int(args.smooth_window))})")
    ax.set_ylabel("|wz| [rad/s]")
    ax.set_xlabel("čas [s]")
    ax.legend(loc="upper right", fontsize=8)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
        print(f"Ulozene: {args.output.resolve()}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

