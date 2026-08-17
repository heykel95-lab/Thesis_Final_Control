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
import sys

import numpy as np
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from figure_style import (apply_style, reference_line, shared_legend,  # noqa: E402
                          thin, SERIES_COLOURS)

RESULTS = os.path.join(HERE, "..", "experiments", "results")

apply_style()

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
    ("S5_normal_p090/r01", r"90 mm along the tool axis"),
]


def load(trial):
    """Return time from set-up entry and the tool-to-plane angle [s, deg]."""
    matches = glob.glob(os.path.join(RESULTS, trial, "logs", "*.csv"))
    if not matches:
        raise SystemExit(f"no log csv under {trial}")
    time, phase, angle = [], [], []
    with open(matches[0]) as f:
        for row in csv.DictReader(f):
            time.append(float(row["time"]))
            phase.append(float(row["phase"]))
            angle.append(column(row, "angular_deviation_deg"))
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
        t, angle = thin(*load(trial))
        ax.plot(t, angle, color=colour, label=label)
        # Marking the value the trial ended on, which is the residual tilt.
        ax.plot(t[-1], angle[-1], marker="o", color=colour,
                markerfacecolor="white", markeredgewidth=1.2, markersize=5)
        ax.annotate(f"{angle[-1]:.2f}", (t[-1], angle[-1]),
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=8, color=colour)
        print(f"{trial:24s} {angle[0]:5.2f} -> {angle[-1]:4.2f} deg")

    reference_line(ax)
    ax.set_xlabel("Time from start of set-up [s]")
    ax.set_ylabel(r"Angular deviation [$^\circ$]")
    ax.set_ylim(bottom=-0.4)
    shared_legend(fig, [ax], ncol=2)

    out = os.path.join(args.out_dir, "MAIN_DQ_descent.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
