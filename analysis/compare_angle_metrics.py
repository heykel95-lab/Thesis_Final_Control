#!/usr/bin/env python3
"""Compare the two ways of reporting how far the tool aligned.

  python3 analysis/compare_angle_metrics.py [--trial DIR] [--out-dir DIR]

Two quantities describe the same event and are measured from different data.

  alignment  the angle between the tool axis and the calibrated plane. It has
             an absolute zero, so a curve can be read as "flat" or "not flat",
             but it carries the tool-axis and plane calibration with it.

  tip        the rotation of the end-effector since first contact. It comes
             from joint angles alone, so no calibration enters, but it only
             says how far the tool turned, not where it ended up.

The left panel overlays them for one trial, with tip subtracted from the
alignment at contact so both start at the same value. The right panel puts the
magnitude of the alignment gain against the tip for every archived trial, split
by whether the trial improved or worsened the alignment.

The magnitudes agree, which shows the calibration does not corrupt the reported
effect. Only the alignment carries the sign, because tip is the length of a
rotation and cannot tell a tool turning onto the plane from one turning off it.
"""

import argparse
import csv
import glob
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "experiments", "results")
METRICS = os.path.join(HERE, "..", "experiments", "derived", "metrics.csv")

SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"
SERIES_BLUE = "#0057b8"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Latin Modern Roman", "CMU Serif", "cmr10", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.size": 9,
    "axes.edgecolor": "#1a1a1a",
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "axes.grid.axis": "y",
    "grid.alpha": 0.3,
    "grid.linewidth": 0.6,
    "lines.linewidth": 1.25,
    "lines.markersize": 4.5,
    "legend.frameon": False,
    "legend.fontsize": 8,
})

SETUP_PHASE = 2  # ControlPhase::kSetup


def load_trial(trial_dir):
    """Return setup-phase time, alignment angle, and tip angle for one trial."""
    matches = glob.glob(os.path.join(trial_dir, "logs", "*.csv"))
    if not matches:
        raise SystemExit(f"no log csv under {trial_dir}")
    d = {}
    with open(matches[0]) as f:
        reader = csv.DictReader(f)
        cols = reader.fieldnames
        for name in cols:
            d[name] = []
        for row in reader:
            for name in cols:
                d[name].append(float(row[name]) if row[name] else np.nan)
    for name in cols:
        d[name] = np.array(d[name])

    setup = d["phase"] == SETUP_PHASE
    t = d["time"][setup]
    alignment = d["alignment_angle_deg"][setup]
    # Calculating the rotation since first contact, which the setup phase holds
    # as its frozen orientation reference [deg].
    tip = np.degrees(np.sqrt(d["e_R_x"][setup] ** 2 +
                             d["e_R_y"][setup] ** 2 +
                             d["e_R_z"][setup] ** 2))
    return t - t[0], alignment, tip


def load_metrics():
    """Return the alignment gain and tip of every trial that reported both."""
    gain, tip, label = [], [], []
    with open(METRICS) as f:
        for row in csv.DictReader(f):
            if not row.get("align_gain_deg") or not row.get("tip_deg"):
                continue
            gain.append(float(row["align_gain_deg"]))
            tip.append(float(row["tip_deg"]))
            label.append(row["run_id"])
    return np.array(gain), np.array(tip), label


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trial",
                   default=os.path.join(RESULTS, "V_best_check", "r02"))
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    t, alignment, tip = load_trial(args.trial)
    gain, tip_final, _ = load_metrics()

    fig, axes = plt.subplots(1, 2, figsize=(6.9, 3.0))

    # Left: both descriptions of the same trial, on one absolute scale.
    axes[0].plot(t, alignment, color=SERIES_BLACK, label="alignment")
    axes[0].plot(t, alignment[0] - tip, color=SERIES_RED, linestyle="--",
                 label=r"alignment at contact $-$ tip")
    axes[0].axhline(0.0, color="#888888", linewidth=0.8, zorder=0)
    axes[0].set_xlabel("Time from first contact [s]")
    axes[0].set_ylabel(r"Angle [$^\circ$]")
    axes[0].legend(loc="upper right")
    axes[0].set_title(os.path.basename(os.path.dirname(args.trial)) + " " +
                      os.path.basename(args.trial), fontsize=8)

    # Right: the same comparison reduced to one point per archived trial. The
    # gain is taken as a magnitude, because tip has no sign to compare against.
    limit = max(np.abs(gain).max(), tip_final.max()) * 1.05
    axes[1].plot([0.0, limit], [0.0, limit], color="#888888",
                 linewidth=0.8, zorder=0)
    improved = gain >= 0.0
    axes[1].plot(tip_final[improved], gain[improved], linestyle="none",
                 marker="o", color=SERIES_BLUE, markerfacecolor="white",
                 markeredgewidth=1.0, label="alignment improved")
    axes[1].plot(tip_final[~improved], -gain[~improved], linestyle="none",
                 marker="s", color=SERIES_RED, markerfacecolor="white",
                 markeredgewidth=1.0, label="alignment worsened")
    axes[1].set_xlabel(r"Tip since contact [$^\circ$]")
    axes[1].set_ylabel(r"Alignment change, magnitude [$^\circ$]")
    axes[1].set_title(f"{len(gain)} archived trials", fontsize=8)
    axes[1].legend(loc="upper left")

    residual = np.abs(gain) - tip_final
    print(f"trials compared        : {len(gain)}")
    print(f"improved / worsened    : {improved.sum()} / {(~improved).sum()}")
    print(f"mean |gain| - tip      : {residual.mean():+.3f} deg")
    print(f"median |gain| - tip    : {np.median(residual):+.3f} deg")
    print(f"within 0.5 deg         : {(np.abs(residual) <= 0.5).sum()} of {len(gain)}")
    worst = int(np.argmax(np.abs(residual)))
    print(f"largest disagreement   : {residual[worst]:+.3f} deg")

    fig.tight_layout()
    out = os.path.join(args.out_dir, "ANGLE_metric_comparison.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
