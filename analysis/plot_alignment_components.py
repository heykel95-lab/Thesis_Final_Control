#!/usr/bin/env python3
"""Draw the angular deviation split into its two surface-frame components.

  python3 analysis/plot_alignment_components.py [TRIAL=LABEL ...] [--out-dir DIR]

The total deviation is the length of a vector, so it hides rotation across the
commanded tilt: a degree gained on the second tangent moves 9.35 to 9.40. The
components carry that motion with its sign.

Both conditions share one panel, with the commanded tangent drawn for every
condition first and the tangent across it after, so the pair that barely
differs and the pair that carries the whole effect read against each other.
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
    "angular_deviation_t1_deg": "alignment_error_t1_deg",
    "angular_deviation_t2_deg": "alignment_error_t2_deg",
}

DEFAULT_TRIALS = [
    ("S1_none_t1_10deg/r01", "no lever"),
    ("S5_normal_p090/r01", "90 mm along the tool axis"),
]


def column(row, name):
    """Return one value by its current name, falling back to the archived one."""
    if name in row:
        return float(row[name])
    return float(row[COLUMN_ALIASES[name]])


def load(trial):
    """Return set-up time and the two in-plane deviation components [s, deg]."""
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

    fig, ax = plt.subplots(figsize=(5.6, 3.4))

    loaded = [(label, thin(*load(trial))) for trial, label in selected]
    colour = iter(SERIES_COLOURS)
    for index, name in ((1, r"$t_1$"), (2, r"$t_2$")):
        for label, (t, t1, t2) in loaded:
            ax.plot(t, t1 if index == 1 else t2, color=next(colour),
                    label=f"{name}, {label}")

    reference_line(ax)
    ax.set_xlabel("Time from first contact [s]")
    ax.set_ylabel(r"Angular deviation [$^\circ$]")
    shared_legend(fig, [ax], ncol=2, bottom=0.17)

    for label, (_, t1, t2) in loaded:
        print(f"{label:26s} t1 {t1[0]:+6.2f} -> {t1[-1]:+6.2f} | "
              f"t2 {t2[0]:+6.2f} -> {t2[-1]:+6.2f}")

    out = os.path.join(args.out_dir, "ANGLE_components.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
