#!/usr/bin/env python3
# Graf mean_dx z CSV: python3 plot_optical_flow_debug_csv.py meranie.csv -o graf.png

import argparse
import csv
from pathlib import Path


def load_csv(path: Path):
    t = []
    mpx = []
    mn = []
    corr = []
    ng = []
    nc = []

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
            mpx.append(fcell(row.get("mean_dx_px", "")))
            mn.append(fcell(row.get("mean_dx_norm", "")))
            corr.append(fcell(row.get("flow_corr", "")))
            ng.append(fcell(row.get("n_good", "")))
            nc.append(fcell(row.get("n_corners", "")))
    return t, mpx, mn, corr, ng, nc


def main() -> None:
    ap = argparse.ArgumentParser(description="Graf optical flow debug (mean_dx) z CSV")
    ap.add_argument("csv_file", type=Path)
    ap.add_argument("-o", "--output", type=Path, help="Ulozit PNG")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    if not args.csv_file.is_file():
        raise SystemExit(f"Subor neexistuje: {args.csv_file}")

    t, mpx, mn, corr, ng, nc = load_csv(args.csv_file)

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

    ax0.plot(t, mpx, label="mean_dx [px/frame] (priemer horizontalneho posunu)", color="#2980b9", linewidth=1.0)
    ax0.set_ylabel("mean_dx [px]")
    ax0.legend(loc="upper right", fontsize=8)
    ax0.set_title("Horizontalna zlozka optickeho toku (Lucas–Kanade)")
    ax0.axhline(0.0, color="#7f8c8d", linewidth=0.6, linestyle="--", alpha=0.7)

    ax1.plot(t, mn, label="mean_dx / sirka_obrazu (norm)", color="#8e44ad", linewidth=1.0, alpha=0.85)
    ax1.plot(t, corr, label="flow_corr (po gain + clamp)", color="#27ae60", linewidth=1.0, alpha=0.9)
    ax1.set_ylabel("norm / korekcia")
    ax1.set_xlabel("cas [s]")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title("Normalizovany tok a vystup do angular.z (pred timer_cb)")
    ax1.axhline(0.0, color="#7f8c8d", linewidth=0.6, linestyle="--", alpha=0.7)

    fig.suptitle(f"Data: {args.csv_file.name}", fontsize=10, y=0.98)
    fig.text(
        0.5,
        0.01,
        (
            "mean_dx_norm * (-correction_gain) sa oreze na +/- max_correction -> flow_corr. "
            "K cmd_vel sa prida len pri jazde vpred a bez silneho teleop otacania. "
            "Ladiace okno: optical_flow_node debug_show:=true (DISPLAY musi byt k dispozicii)."
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
