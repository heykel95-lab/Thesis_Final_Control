#!/usr/bin/env python3
"""Draw the tool-to-plane angle split into its two surface-frame components.

  python3 analysis/plot_alignment_components.py [TRIAL=LABEL ...] [--out-dir DIR]

The total angle is the length of a vector, so it hides any rotation across the
commanded tilt: a degree gained on the second tangent moves a 9.35 degree total
to 9.40. The components carry that motion with its sign, and they separate the
correction the lever is meant to produce from the sideways rotation it is meant
to prevent.

The dashed line is the commanded tangent, the solid line the one across it.
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

SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"

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

# Archives written before the rename carry the alignment_ column names.
COLUMN_ALIASES = {
    "angular_deviation_deg": "alignment_angle_deg",
    "angular_deviation_t1_deg": "alignment_error_t1_deg",
    "angular_deviation_t2_deg": "alignment_error_t2_deg",
}


def column(row, name):
    """Return one value by its current name, falling back to the archived one."""
    if name in row:
        return float(row[name])
    return float(row[COLUMN_ALIASES[name]])


DEFAULT_TRIALS = [
    ("S1_none_t1_10deg/r01", "no lever"),
    ("S5_normal_p090/r01", "90 mm along the tool axis"),
]


def load(trial):
    """Return set-up time and the two in-plane alignment components [s, deg]."""
    matches = glob.glob(os.path.join(RESULTS, trial, "logs", "*.csv"))
    if not matches:
        raise SystemExit(f"no log csv under {trial}")
    time, phase, t1, t2 = [], [], [], []
    with open(matches[0]) as f:
        for row in csv.DictReader(f):
            time.append(float(row["time"]))
            phase.append(float(row["phase"]))
            t1.append(column(row, "angular_deviation_t1_deg"))
            t2.append(column(row, "angular_deviation_t2_deg"))
    setup = np.array(phase) == SETUP_PHASE
    t = np.array(time)[setup]
    return t - t[0], np.array(t1)[setup], np.array(t2)[setup]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trials", nargs="*", metavar="TRIAL=LABEL")
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    selected = ([tuple(a.split("=", 1)) for a in args.trials]
                if args.trials else DEFAULT_TRIALS)

    fig, axes = plt.subplots(1, len(selected), figsize=(6.9, 3.1), sharey=True)
    axes = np.atleast_1d(axes)

    for ax, (trial, label) in zip(axes, selected):
        t, t1, t2 = load(trial)
        ax.plot(t, t1, color=SERIES_BLACK, linestyle="--",
                label=r"$t_1$, the commanded tilt")
        ax.plot(t, t2, color=SERIES_RED,
                label=r"$t_2$, across it")
        ax.axhline(0.0, color="#888888", linewidth=0.8, zorder=0)
        ax.set_xlabel("Time from first contact [s]")
        ax.set_title(label, fontsize=8)
        print(f"{trial:24s} t1 {t1[0]:+6.2f} -> {t1[-1]:+6.2f} | "
              f"t2 {t2[0]:+6.2f} -> {t2[-1]:+6.2f}")

    axes[0].set_ylabel(r"Alignment error [$^\circ$]")
    axes[0].legend(loc="lower right")

    fig.tight_layout()
    out = os.path.join(args.out_dir, "ANGLE_components.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
