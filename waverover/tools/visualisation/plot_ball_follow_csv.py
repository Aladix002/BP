#!/usr/bin/env python3
# Grafy z CSV (bez ROS): python3 plot_ball_follow_csv.py meranie.csv -o graf.png
# Zdoraznuje: odchylka lopty vlravo/vpravo (x_err) a rychlost lava/prava motor (PWM alebo v_ln/v_rn).

import argparse
import csv
from pathlib import Path


def load_csv(path: Path):
    t = []
    state = []
    x_err = []
    rpx = []
    conf = []
    esh = []
    pint = []
    derr = []
    ekp = []
    pout = []
    mix = []
    vln = []
    vrn = []
    clin = []
    cang = []
    bt = []
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
        cols = r.fieldnames or []
        read_bt = "bt_active" in cols
        read_pwm = "pwm_l_pct" in cols and "pwm_r_pct" in cols
        for row in r:
            t.append(fcell(row.get("t_sec", "")))
            state.append(fcell(row.get("state", "")))
            x_err.append(fcell(row.get("x_err", "")))
            rpx.append(fcell(row.get("radius_px", "")))
            conf.append(fcell(row.get("conf", "")))
            esh.append(fcell(row.get("err_shaped", "")))
            pint.append(fcell(row.get("pid_integral", "")))
            derr.append(fcell(row.get("d_err", "")))
            ekp.append(fcell(row.get("eff_kp", "")))
            pout.append(fcell(row.get("pid_out", "")))
            mix.append(fcell(row.get("diff_mix", "")))
            vln.append(fcell(row.get("v_ln", "")))
            vrn.append(fcell(row.get("v_rn", "")))
            clin.append(fcell(row.get("cmd_lin", "")))
            cang.append(fcell(row.get("cmd_ang", "")))
            bt.append((row.get("bt_active") or "").strip() if read_bt else "")
            pwm_l.append(fcell(row.get("pwm_l_pct", "")) if read_pwm else float("nan"))
            pwm_r.append(fcell(row.get("pwm_r_pct", "")) if read_pwm else float("nan"))
    return t, state, x_err, rpx, conf, esh, pint, derr, ekp, pout, mix, vln, vrn, clin, cang, bt, pwm_l, pwm_r, read_pwm


def _any_finite(xs) -> bool:
    import math

    return any(math.isfinite(x) for x in xs)


def main() -> None:
    ap = argparse.ArgumentParser(description="Graf ball follow: odchylka vlavo/vpravo + motory (log_ball_follow_csv.py)")
    ap.add_argument("csv_file", type=Path)
    ap.add_argument("-o", "--output", type=Path, help="Ulozit PNG")
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    if not args.csv_file.is_file():
        raise SystemExit(f"Subor neexistuje: {args.csv_file}")

    t, _st, x_err, rpx, conf, esh, _pi, _de, _ek, _po, mix, vln, vrn, clin, cang, _bt, pwm_l, pwm_r, has_pwm = load_csv(
        args.csv_file
    )
    use_pwm = has_pwm and (_any_finite(pwm_l) or _any_finite(pwm_r))

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

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.2), sharex=True)
    fig.subplots_adjust(hspace=0.28, bottom=0.16, top=0.88)
    ax0, ax1 = axes

    # --- Horny: bocna odchylka od stredu (vlavo / vpravo) + volitelne polomer na druhej osi ---
    ax0.fill_between(t, -1.05, 0.0, alpha=0.08, color="#3498db", label="oblast: lopta vlavo od stredu")
    ax0.fill_between(t, 0.0, 1.05, alpha=0.08, color="#e67e22", label="oblast: lopta vpravo od stredu")
    ax0.plot(t, x_err, color="#c0392b", linewidth=1.2, label="x_err (0 = stred obrazu)")
    ax0.axhline(0.0, color="#2c3e50", linewidth=0.8, linestyle="-", alpha=0.7)
    ax0.set_ylabel("x_err [-1..1]")
    ax0.set_title(
        "Poloha lopty voci stredu obrazu: zaporna odchylka = lopta vlavo, kladna = vpravo"
    )
    ax0.set_ylim(-1.05, 1.05)
    ax0.legend(loc="upper left", fontsize=7)

    ax0b = ax0.twinx()
    ax0b.plot(t, rpx, color="#2980b9", linewidth=0.9, alpha=0.75, linestyle="--", label="radius_px (blizsie ~ vacsi)")
    ax0b.set_ylabel("polomer [px]")
    ax0b.legend(loc="upper right", fontsize=7)

    # --- Dolny: PWM lava/prava (skutocny motor) alebo v_ln/v_rn z ball_followera ---
    if use_pwm:
        ax1.plot(t, pwm_l, label="PWM lave [% max]", color="#8e44ad", linewidth=1.1)
        ax1.plot(t, pwm_r, label="PWM prave [% max]", color="#d35400", linewidth=1.1)
        ax1.set_ylabel("PWM [%]")
        ax1.set_title("Rychlost motorov (drive_debug po cmd_vel) — vyssie PWM = rychlejsie koleso")
    else:
        ax1.plot(t, vln, label="v_ln norm (virtual lave pred cmd_vel)", color="#8e44ad", linewidth=1.1)
        ax1.plot(t, vrn, label="v_rn norm (virtual prave pred cmd_vel)", color="#d35400", linewidth=1.1)
        ax1.set_ylabel("norm 0..1")
        ax1.set_title(
            "Prikaz kolesam z ball_followera (bez /drive_debug v CSV — spusti log s log_drive_debug:=true)"
        )
    ax1.plot(t, mix, label="diff_mix", color="#27ae60", linewidth=0.85, alpha=0.85)
    ax1.set_xlabel("cas [s]")
    ax1.legend(loc="upper right", fontsize=7)
    ax1.axhline(0.0, color="#7f8c8d", linewidth=0.5, linestyle="--", alpha=0.5)

    fig.suptitle(f"Ball follow: {args.csv_file.name}", fontsize=10, y=0.98)
    fig.text(
        0.5,
        0.02,
        (
            "Pri lopta vpravo (x_err>0) diferencial zvysuje v_rn oproti v_ln — robot sa zataca za loptou. "
            "Stav state: 4 track_diff, 6 approach_diff, … (maska len ak publish_debug_mask.)"
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
