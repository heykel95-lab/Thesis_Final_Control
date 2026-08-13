#!/usr/bin/env python3
"""Draw the tool alignment falling from its commanded tilt during set-up.

  python3 analysis/plot_angle_descent.py [TRIAL=LABEL ...] [--out-dir DIR]

Each argument names an archived trial directory relative to experiments/results
and the label it carries in the legend. Without arguments the two verification
trials are drawn, which differ only in the compliance centre.

The angle is the one between the tool axis and the calibrated plane, so its
zero is the plane itself and a curve can be read as how flat the tool ended.
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

SERIES_COLOURS = ("#000000", "#c00000", "#0057b8", "#e0ad00")

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
    "lines.linewidth": 1.4,
    "legend.frameon": False,
    "legend.fontsize": 8,
})

SETUP_PHASE = 2  # ControlPhase::kSetup

DEFAULT_TRIALS = [
    ("V_orient_stiff/r01", "no lever"),
    ("V_best_check/r02", r"120 mm along the tool axis"),
]


def load(trial):
    """Return set-up time from contact and the tool-to-plane angle [s, deg]."""
    matches = glob.glob(os.path.join(RESULTS, trial, "logs", "*.csv"))
    if not matches:
        raise SystemExit(f"no log csv under {trial}")
    time, phase, angle = [], [], []
    with open(matches[0]) as f:
        for row in csv.DictReader(f):
            time.append(float(row["time"]))
            phase.append(float(row["phase"]))
            angle.append(float(row["alignment_angle_deg"]))
    time = np.array(time)
    setup = np.array(phase) == SETUP_PHASE
    t = time[setup]
    return t - t[0], np.array(angle)[setup]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trials", nargs="*", metavar="TRIAL=LABEL")
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    selected = ([tuple(a.split("=", 1)) for a in args.trials]
                if args.trials else DEFAULT_TRIALS)

    fig, ax = plt.subplots(figsize=(5.6, 3.1))
    for (trial, label), colour in zip(selected, SERIES_COLOURS):
        t, angle = load(trial)
        ax.plot(t, angle, color=colour, label=label)
        # Marking the value the trial ended on, which is the residual tilt.
        ax.plot(t[-1], angle[-1], marker="o", color=colour,
                markerfacecolor="white", markeredgewidth=1.2, markersize=5)
        ax.annotate(f"{angle[-1]:.2f}", (t[-1], angle[-1]),
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=8, color=colour)
        print(f"{trial:24s} {angle[0]:5.2f} -> {angle[-1]:4.2f} deg")

    ax.axhline(0.0, color="#888888", linewidth=0.8, zorder=0)
    ax.set_xlabel("Time from first contact [s]")
    ax.set_ylabel(r"Tool-to-plane angle [$^\circ$]")
    ax.set_ylim(bottom=-0.4)
    ax.legend(loc="upper right")

    fig.tight_layout()
    out = os.path.join(args.out_dir, "ANGLE_descent.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
