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


def main():
    ap = argparse.ArgumentParser(
        description="Graf odchylky jazdy pre jeden beh (spusti 3x pre porovnanie rezimov)."
    )
    ap.add_argument("csv_file", type=Path, help="CSV z log_imu_drive_csv.py")
    ap.add_argument("-o", "--output", type=Path, help="Ulozit PNG")
    ap.add_argument("--label", default="beh", help="Nazov rezimu v titulku (napr. bez_korekcie)")
    ap.add_argument("--start-s", type=float, default=0.0, help="Zaciatok analyzy [s]")
    ap.add_argument("--end-s", type=float, default=-1.0, help="Koniec analyzy [s], -1 = do konca")
    ap.add_argument("--use-imu-raw", action="store_true", help="Pouzi imu_wz_rad_s namiesto yaw_filt_rad_s")
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
        raise SystemExit("Chyba importu matplotlib. Nainstaluj: python3 -m pip install --user matplotlib") from e

    for style in ("seaborn-v0_8-whitegrid", "ggplot", "bmh"):
        try:
            plt.style.use(style)
            break
        except OSError:
            continue

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    fig.subplots_adjust(hspace=0.25, bottom=0.12, top=0.88)

    ax0.plot(t, s, color="#2980b9", linewidth=1.0, label="wz")
    ax0.axhline(0.0, color="#2c3e50", linewidth=0.7, linestyle="--", alpha=0.7)
    ax0.set_ylabel("wz [rad/s]")
    ax0.set_title(f"{args.label}: uhlova odchylka okolo osi z")
    ax0.legend(loc="upper right", fontsize=8)

    abs_s = [abs(v) for v in s]
    ax1.plot(t, abs_s, color="#d35400", linewidth=1.0, label="|wz|")
    ax1.set_ylabel("|wz| [rad/s]")
    ax1.set_xlabel("cas [s]")
    ax1.set_title(
        f"mean|wz|={m['mean_abs']:.4f}, rmse={m['rmse']:.4f}, p95={m['p95_abs']:.4f}, max={m['max_abs']:.4f}"
    )
    ax1.legend(loc="upper right", fontsize=8)

    fig.suptitle(f"Odchylka jazdy: {args.csv_file.name}", fontsize=10, y=0.98)

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
        print(f"Ulozene: {args.output.resolve()}")
    else:
        plt.show()


if __name__ == "__main__":
    main()

