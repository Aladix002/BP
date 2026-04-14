#!/usr/bin/env python3
# Graf z CSV (bez ROS): python3 plot_optical_flow_csv.py meranie.csv -o graf.png

import argparse
import csv
from pathlib import Path


def load_csv(path: Path):
    t = []
    tlx = []
    taz = []
    olx = []
    oaz = []
    dlt = []
    pwm_l = []
    pwm_r = []

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
            tlx.append(fcell(row.get("teleop_linear_x", "")))
            taz.append(fcell(row.get("teleop_angular_z", "")))
            olx.append(fcell(row.get("out_linear_x", "")))
            oaz.append(fcell(row.get("out_angular_z", "")))
            dlt.append(fcell(row.get("flow_delta_angular_z", "")))
            pwm_l.append(fcell(row.get("pwm_l_pct", "")))
            pwm_r.append(fcell(row.get("pwm_r_pct", "")))
    return t, tlx, taz, olx, oaz, dlt, pwm_l, pwm_r


def _any_finite(xs) -> bool:
    import math

    return any(math.isfinite(x) for x in xs)


def main() -> None:
    ap = argparse.ArgumentParser(description="Graf optical flow korekcie z CSV (log_optical_flow_csv.py)")
    ap.add_argument("csv_file", type=Path)
    ap.add_argument("-o", "--output", type=Path, help="Ulozit PNG")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    if not args.csv_file.is_file():
        raise SystemExit(f"Subor neexistuje: {args.csv_file}")

    t, tlx, taz, _olx, oaz, dlt, pwm_l, pwm_r = load_csv(args.csv_file)
    has_pwm = _any_finite(pwm_l) or _any_finite(pwm_r)

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

    nrows = 3 if has_pwm else 2
    fig, axes = plt.subplots(nrows, 1, figsize=(10, 2.2 + 2.3 * nrows), sharex=True)
    fig.subplots_adjust(hspace=0.22, bottom=0.20, top=0.90)
    if nrows == 2:
        ax0, ax1 = axes
    else:
        ax0, ax1, ax2 = axes

    # V optical_flow_node sa linear.x len kopiruje; staci jedna krivka
    ax0.plot(t, tlx, label="linear.x [norm] (teleop = vystup)", color="#2980b9", linewidth=1.0)
    ax0.set_ylabel("linear.x")
    ax0.legend(loc="upper right", fontsize=8)
    ax0.set_title("Prikaz vpred/vzad (korekcia sa aplikuje len na angular.z)")
    ax0.axhline(0.0, color="#7f8c8d", linewidth=0.6, linestyle="--", alpha=0.6)

    ax1.plot(t, taz, label="teleop angular.z [norm]", color="#8e44ad", linewidth=1.0)
    ax1.plot(t, oaz, label="angular.z po korekcii", color="#d35400", linewidth=1.0, alpha=0.9)
    ax1.plot(t, dlt, label="delta = korekcia (pridane k angular.z)", color="#27ae60", linewidth=1.0, alpha=0.9)
    ax1.set_ylabel("angular.z / delta")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title("Otacanie: vstup, vystup, prispevok optical flow")
    ax1.axhline(0.0, color="#7f8c8d", linewidth=0.6, linestyle="--", alpha=0.6)

    if has_pwm:
        ax2.plot(t, pwm_l, label="PWM lave [% max]", color="#8e44ad", linewidth=1.0)
        ax2.plot(t, pwm_r, label="PWM prave [% max]", color="#d35400", linewidth=1.0)
        ax2.set_ylabel("PWM [%]")
        ax2.set_xlabel("cas [s]")
        ax2.legend(loc="upper right", fontsize=8)
        ax2.set_title("PWM (drive_debug) — nasledok po drive_node")
    else:
        ax1.set_xlabel("cas [s]")

    fig.suptitle(f"Data: {args.csv_file.name}", fontsize=10, y=0.98)
    fig.text(
        0.5,
        0.01,
        (
            "Optical flow (Lucas–Kanade): k vystupnemu angular.z sa prida korekcia z horizontalneho posunu obrazu, "
            "len ak |linear.x| > forward_threshold a zaroven |teleop angular.z| < steer_deadzone (uzivatel netoci). "
            "Parametre: correction_gain, max_correction — viz optical_flow_node."
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
