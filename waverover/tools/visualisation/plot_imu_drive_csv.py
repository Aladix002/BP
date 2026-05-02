#!/usr/bin/env python3
# Graf z CSV: plot_imu_drive_csv.py meranie.csv -o out.png

import argparse
import csv
from pathlib import Path


def load_csv(path: Path):
    t = []
    imu_wz = []
    pwm_l = []
    pwm_r = []
    cl = []
    cr = []
    corr = []
    yaw_f = []

    def fcell(x: str) -> float:
        x = (x or "").strip()
        if not x:
            return float("nan")
        try:
            return float(x)
        except ValueError:
            return float("nan")

    with open(path, newline="", encoding="utf-8") as fp:
        r = csv.DictReader(fp)
        for row in r:
            t.append(fcell(row.get("t_sec", "")))
            imu_wz.append(fcell(row.get("imu_wz_rad_s", "")))
            pwm_l.append(fcell(row.get("pwm_l_pct", "")))
            pwm_r.append(fcell(row.get("pwm_r_pct", "")))
            cl.append(fcell(row.get("cl", "")))
            cr.append(fcell(row.get("cr", "")))
            corr.append(fcell(row.get("imu_corr", "")))
            yaw_f.append(fcell(row.get("yaw_filt_rad_s", "")))
    return t, imu_wz, pwm_l, pwm_r, cl, cr, corr, yaw_f


def main() -> None:
    ap = argparse.ArgumentParser(description="Graf IMU + drive_debug z CSV")
    ap.add_argument("csv_file", type=Path, help="CSV z log_imu_drive_csv.py")
    ap.add_argument("-o", "--output", type=Path, help="Ulozit PNG (napr. obrazok.png)")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    if not args.csv_file.is_file():
        raise SystemExit(f"Subor neexistuje: {args.csv_file}")

    t, imu_wz, pwm_l, pwm_r, _cl, _cr, corr, yaw_f = load_csv(args.csv_file)

    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        raise SystemExit(
            "Chyba importu matplotlib. Nainstaluj: python3 -m pip install --user matplotlib"
        ) from e

    for s in ("seaborn-v0_8-whitegrid", "ggplot", "bmh"):
        try:
            plt.style.use(s)
            break
        except OSError:
            continue
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    fig.subplots_adjust(hspace=0.22, bottom=0.18, top=0.88)

    ax0, ax1 = axes
    ax0.plot(t, imu_wz, label=r"$\omega_z$ /imu (surove) [rad/s]", color="#c0392b", linewidth=1.0)
    ax0.plot(t, yaw_f, label="yaw_filt (motor) [rad/s]", color="#2980b9", linewidth=1.0, alpha=0.85)
    ax0.plot(t, corr, label="imu_corr (PID L/R)", color="#27ae60", linewidth=1.0, alpha=0.9)
    ax0.set_ylabel("uhlova rychlost / korekcia")
    ax0.legend(loc="upper right", fontsize=8)
    ax0.set_title("IMU a korekcia motorom")
    ax0.axhline(0.0, color="#7f8c8d", linewidth=0.6, linestyle="--", alpha=0.7)

    ax1.plot(t, pwm_l, label="PWM lave [% max]", color="#8e44ad", linewidth=1.0)
    ax1.plot(t, pwm_r, label="PWM prave [% max]", color="#d35400", linewidth=1.0)
    ax1.set_ylabel("PWM [%]")
    ax1.set_xlabel("cas [s]")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title("PWM po to_duty")

    fig.suptitle(f"Data: {args.csv_file.name}", fontsize=10, y=0.98)
    fig.text(
        0.5,
        0.01,
        (
            r"$\omega_z$: ROS +Z hore; znamienko zavisi od montaze IMU a imu_sign. "
            "PID: lave koleso -corr, prave +corr."
        ),
        ha="center",
        va="bottom",
        fontsize=7.5,
    )

    if args.output:
        fig.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
        print(f"Ulozene: {args.output.resolve()}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
