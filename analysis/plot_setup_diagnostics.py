#!/usr/bin/env python3
"""Draw the surface-frame rotation and commanded wrench during set-up.

  python3 analysis/plot_setup_diagnostics.py [TRIAL=LABEL ...] [--out-dir DIR]

One column per trial, three rows sharing the same time axis, so the rotation can
be read against the load that produced it.

  rotation   the turn since the start of set-up, resolved along the surface axes. It
             comes from joint angles alone, so no tool axis enters it.
  force      the controller-commanded Cartesian force.
  moment     the controller-commanded Cartesian moment at the TCP.

The wrench is resolved along the surface axes rather than the base axes, so a
moment about a tangent sits in the same row as the rotation about it.
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
from extract_metrics import surface_frame, read_params  # noqa: E402
from figure_style import (apply_style, reference_line,  # noqa: E402
                          thin, SERIES_COLOURS)

RESULTS = os.path.join(HERE, "..", "experiments", "results")

apply_style()

AXIS_NAMES = (r"$t_1$", r"$t_2$", r"$n$")


SETUP_PHASE = 2  # ControlPhase::kSetup

DEFAULT_TRIALS = [
    ("P2_t1_pos_m040/r01", "centre -40 mm"),
    ("P2_t1_pos_p040/r01", "centre +40 mm"),
]


def vec(row, prefix):
    return [float(row[f"{prefix}_{a}"]) for a in "xyz"]


def load(trial):
    """Return time, rotation, commanded force and moment in surface axes."""
    directory = os.path.join(RESULTS, trial)
    logs = glob.glob(os.path.join(directory, "logs", "*.csv"))
    if not logs:
        raise SystemExit(f"no log csv under {trial}")
    params = read_params(os.path.join(directory, "params_effective"))
    frame = surface_frame(float(params["surface_tilt_x_deg"]),
                          float(params["surface_tilt_y_deg"]))

    time, rotation, force, moment = [], [], [], []
    with open(logs[0]) as f:
        for row in csv.DictReader(f):
            if float(row["phase"]) != SETUP_PHASE:
                continue
            time.append(float(row["time"]))
            rotation.append(vec(row, "e_R"))
            force.append(vec(row, "f"))
            moment.append(vec(row, "m"))

    t = np.array(time)
    return (t - t[0],
            np.degrees(np.array(rotation)) @ frame,
            np.array(force) @ frame,
            np.array(moment) @ frame)


def draw_axes(ax, t, values, ylabel):
    """Draw the three surface-frame components of one vector quantity."""
    for i, name in enumerate(AXIS_NAMES):
        ax.plot(t, values[:, i], color=SERIES_COLOURS[i], label=name)
    # Zero separates a restoring component from a driving one here, so the
    # line means something; the press axis below is left without one.
    reference_line(ax)
    ax.margins(y=0.30)
    ax.legend(loc="upper right", ncol=3, columnspacing=0.8,
              handlelength=1.0, fontsize=7)
    if ylabel:
        ax.set_ylabel(ylabel)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trials", nargs="*", metavar="TRIAL=LABEL")
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    selected = ([tuple(a.split("=", 1)) for a in args.trials]
                if args.trials else DEFAULT_TRIALS)

    fig, axes = plt.subplots(3, len(selected), figsize=(6.9, 6.2),
                             sharex=True, squeeze=False)

    for column, (trial, label) in enumerate(selected):
        t, rotation, force, moment = thin(*load(trial))
        first = column == 0
        draw_axes(axes[0][column], t, rotation,
                  r"Set-up rotation [$^\circ$]" if first else "")
        draw_axes(axes[1][column], t, force,
                  r"$F_{\mathrm{cmd}}$ [N]" if first else "")
        draw_axes(axes[2][column], t, moment,
                  r"$M_{\mathrm{cmd}}$ [N m]" if first else "")
        axes[2][column].set_xlabel(f"Time from start of set-up [s]\n{label}")

        print(f"{trial:24s} rotation t1 {rotation[-1, 0]:+6.2f} deg | "
              f"F_cmd n {force[-1, 2]:+6.1f} N | "
              f"M_cmd t1 {moment[-1, 0]:+6.2f} N m")

    fig.tight_layout()
    out = os.path.join(args.out_dir, "MAIN_E_diagnostics.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
