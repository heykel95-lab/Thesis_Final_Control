#!/usr/bin/env python3
"""Draw one centre-of-compliance case in the fixed three-panel order.

  python3 analysis/plot_coc_case.py TRIAL=DETAIL [...] --axis t1 --out NAME

Every case is read the same way, top to bottom:

  1  set-up rotation   the signed rotation since the start of set-up about the
                       commanded surface tangent.
  2  normal force      the controller-commanded press along n_s.
  3  alignment moment  the controller-commanded moment about the commanded
                       surface tangent.

The set-up rotation is the same controller-response quantity used by every
case-comparison plot. It comes from the robot orientation error referenced at
the clearance transition and is resolved on the calibrated surface axes. It therefore has
no absolute flat-tool zero and is not affected by play between tool and
gripper.

The commanded wrench is used consistently for force and moment. The force is
the commanded Cartesian force resolved along the surface normal. The moment is
the commanded Cartesian moment at the TCP resolved about the selected tangent.
No model-estimated wrench is mixed into this controller-response comparison.

The normal force is negative while the tool presses. n_s points out of the
plate, so the commanded press runs along -n_s.

Each panel carries its own legend. The legend states the measured attitude at
the start of set-up, so the figure does not substitute the nominal commanded offset
for the orientation the robot actually reached.
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
from extract_metrics import parse_report, surface_frame, read_params  # noqa: E402
from figure_style import (apply_style, reference_line,  # noqa: E402
                          thin, SERIES_COLOURS)

RESULTS = os.path.join(HERE, "..", "experiments", "results")

apply_style()

SETUP_PHASE = 2  # ControlPhase::kSetup

AXIS_COLUMN = {"t1": 0, "t2": 1, "n": 2}
AXIS_LABEL = {"t1": r"$t_1$", "t2": r"$t_2$", "n": r"$n$"}
# The axis rides in the subscript, so a panel names the component it carries
# rather than describing it: M_{t1,cmd} instead of "M_cmd about t1".
AXIS_SUBSCRIPT = {"t1": "t_1", "t2": "t_2", "n": "n"}


def vec(row, prefix):
    return np.array([float(row[f"{prefix}_{a}"]) for a in "xyz"])


def initial_label(directory, axis, detail):
    """Name a curve by its measured signed orientation before set-up."""
    report = parse_report(os.path.join(directory, "terminal.log"))
    key = f"deviation_before_{axis}"
    try:
        angle = float(report[key])
    except (KeyError, ValueError):
        raise SystemExit(f"no {key} in the set-up report under {directory}")
    tangent = "t_1" if axis == "t1" else "t_2"
    suffix = f", {detail}" if detail else ""
    return rf"initial ${angle:+.2f}^\circ$ about ${tangent}${suffix}"


def load(trial, axis):
    """Return time, set-up rotation, commanded force and commanded moment."""
    directory = os.path.join(RESULTS, trial)
    logs = glob.glob(os.path.join(directory, "logs", "*.csv"))
    if not logs:
        raise SystemExit(f"no log csv under {trial}")
    params = read_params(os.path.join(directory, "params_effective"))
    frame = surface_frame(float(params["surface_tilt_x_deg"]),
                          float(params["surface_tilt_y_deg"]))
    normal = frame[:, 2]
    tilt_axis = frame[:, AXIS_COLUMN[axis]]

    with open(logs[0]) as f:
        rows = [r for r in csv.DictReader(f)
                if float(r["phase"]) == SETUP_PHASE]

    time, rotation, fn_cmd, m_cmd = [], [], [], []
    for row in rows:
        time.append(float(row["time"]))
        rotation.append(float(np.degrees(vec(row, "e_R")) @ tilt_axis))
        fn_cmd.append(float(normal @ vec(row, "f")))
        m_cmd.append(float(tilt_axis @ vec(row, "m")))

    t = np.array(time)
    return (t - t[0], np.array(rotation), np.array(fn_cmd), np.array(m_cmd))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trials", nargs="+", metavar="TRIAL=DETAIL")
    p.add_argument("--axis", default="t1", choices=sorted(AXIS_COLUMN))
    p.add_argument("--out", default="COC_case")
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    selected = [tuple(a.split("=", 1)) for a in args.trials]
    fig, axes = plt.subplots(3, 1, figsize=(5.8, 6.2), sharex=True)

    for (trial, detail), colour in zip(selected, SERIES_COLOURS):
        directory = os.path.join(RESULTS, trial)
        label = initial_label(directory, args.axis, detail)
        t, rotation, fn_cmd, m_cmd = thin(*load(trial, args.axis))
        for ax, series in zip(axes, (rotation, fn_cmd, m_cmd)):
            ax.plot(t, series, color=colour, label=label)
        print(f"{trial:26s} Delta_theta_set {rotation[-1]:+6.2f} deg | "
              f"Fn_cmd {fn_cmd[-1]:7.1f} N | M_cmd {m_cmd[-1]:+6.2f} N m")

    sub = AXIS_SUBSCRIPT[args.axis]
    labels = [rf"$\Delta\theta_{{\mathrm{{set}},{sub}}}$ [$^\circ$]",
              r"$F_{n,\mathrm{cmd}}$ [N]",
              rf"$M_{{{sub},\mathrm{{cmd}}}}$ [N m]"]
    # The deviation panel keeps the upper right corner, which its curves leave
    # free and which the start-value annotations at the left edge do not reach.
    # The rest take whichever corner is clearest.
    corners = ["upper right", "best", "best", "best", "best"]
    for ax, text, corner in zip(axes, labels, corners):
        ax.set_ylabel(text)
        # Zero separates a flat tool from a tilted one, and a restoring moment
        # from a driving one. The press panels are left without a line.
        if "F_" not in text:
            reference_line(ax)
        # Headroom so the corner legend sits above the data rather than on it.
        # The labels name the condition in full, so they are wide and need
        # more room than a bare series name would.
        ax.margins(y=1.15 if ax is axes[0] else 0.45)
        # A legend printed over the data is worse than one in a different
        # corner of the same panel.
        ax.legend(loc=corner, fontsize=7, labelspacing=0.3)
    axes[-1].set_xlabel("Time from start of set-up [s]")
    fig.tight_layout()
    out = os.path.join(args.out_dir, f"{args.out}.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
