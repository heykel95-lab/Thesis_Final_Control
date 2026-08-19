#!/usr/bin/env python3
"""Compare the two ways of reporting how far the tool aligned.

  python3 analysis/compare_angle_metrics.py [--trial DIR] [--out-dir DIR]

Two quantities describe the same event and are measured from different data.

  alignment  the angle between the tool axis and the calibrated plane. It has
             an absolute zero, so a curve can be read as "flat" or "not flat",
             but it carries the tool-axis and plane calibration with it.

  angular    the deviation from the orientation held at the start of set-up. It
  deviation  comes from joint angles alone, so no calibration enters, but it
             only says how far the tool turned, not where it ended up.

The left panel overlays them for one trial, with the deviation subtracted from
the alignment at the beginning of set-up so both start at the same value. The right panel puts
the magnitude of the alignment gain against the deviation for every archived
trial, split by whether the trial improved or worsened the alignment.

The magnitudes agree, which shows the calibration does not corrupt the reported
effect. Only the alignment carries the sign, because the deviation is the
length of a rotation and cannot tell a tool turning onto the plane from one
turning off it.
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "..", "experiments", "results")
METRICS = os.path.join(HERE, "..", "experiments", "derived", "metrics.csv")

SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"
SERIES_BLUE = "#0057b8"

sys.path.insert(0, HERE)
from figure_style import (apply_style, reference_line, shared_legend,  # noqa: E402
                          thin, SERIES_BLACK, SERIES_RED, SERIES_BLUE)

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



def load_trial(trial_dir):
    """Return set-up time, alignment angle, and angular deviation for a trial."""
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
    total = ("angular_deviation_deg" if "angular_deviation_deg" in d
             else "alignment_angle_deg")
    alignment = d[total][setup]
    # Calculating the deviation from the orientation captured at the geometric
    # clearance transition and held during set-up [deg].
    deviation = np.degrees(np.sqrt(d["e_R_x"][setup] ** 2 +
                             d["e_R_y"][setup] ** 2 +
                             d["e_R_z"][setup] ** 2))
    return t - t[0], alignment, deviation


def load_metrics():
    """Return the gain and angular deviation of every trial with both."""
    gain, deviation, label = [], [], []
    with open(METRICS) as f:
        for row in csv.DictReader(f):
            if not row.get("deviation_gain_deg") or not row.get("end_effector_deviation_deg"):
                continue
            gain.append(float(row["deviation_gain_deg"]))
            deviation.append(float(row["end_effector_deviation_deg"]))
            label.append(row["run_id"])
    return np.array(gain), np.array(deviation), label


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trial",
                   default=os.path.join(RESULTS, "V_best_check", "r02"))
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    t, alignment, deviation = load_trial(args.trial)
    gain, deviation_final, _ = load_metrics()

    fig, axes = plt.subplots(1, 2, figsize=(6.9, 3.0))

    # Left: both descriptions of the same trial, on one absolute scale.
    t, alignment, deviation = thin(t, alignment, deviation)
    axes[0].plot(t, alignment, color=SERIES_BLACK,
                 label="Alignment angle from end-effector pose")
    axes[0].plot(t, alignment[0] - deviation, color=SERIES_RED,
                 label=(r"deviation at start of set-up $-$ "
                        r"end-effector rotation"))
    reference_line(axes[0])
    axes[0].set_xlabel("Time from start of set-up [s]")
    # Both curves are the alignment angle, obtained two ways; a bare "Angle"
    # leaves the reader to guess which angle is plotted.
    axes[0].set_ylabel(r"Alignment angle $\theta_{\mathrm{align}}$ [$^\circ$]")
    axes[0].set_title("(a)")

    # Right: the same comparison reduced to one point per archived trial. The
    # gain is a magnitude here, because the deviation carries no sign.
    limit = max(np.abs(gain).max(), deviation_final.max()) * 1.05
    axes[1].plot([0.0, limit], [0.0, limit], color="#888888",
                 linewidth=0.8, zorder=0)
    improved = gain >= 0.0
    axes[1].plot(deviation_final[improved], gain[improved], linestyle="none",
                 marker="o", color=SERIES_BLUE, markerfacecolor="white",
                 markeredgewidth=1.0, label="deviation reduced")
    axes[1].plot(deviation_final[~improved], -gain[~improved], linestyle="none",
                 marker="s", color=SERIES_RED, markerfacecolor="white",
                 markeredgewidth=1.0, label="deviation increased")
    axes[1].set_xlabel(r"Set-up rotation angle $\phi_{\mathrm{set}}$ [$^\circ$]")
    axes[1].set_ylabel(r"$|\Delta\theta_{\mathrm{align}}|$ [$^\circ$]")
    axes[1].set_title("(b)")


    residual = np.abs(gain) - deviation_final
    print(f"trials compared        : {len(gain)}")
    print(f"improved / worsened    : {improved.sum()} / {(~improved).sum()}")
    print(f"mean |gain| - deviation  : {residual.mean():+.3f} deg")
    print(f"median |gain| - deviation: {np.median(residual):+.3f} deg")
    print(f"within 0.5 deg         : {(np.abs(residual) <= 0.5).sum()} of {len(gain)}")
    worst = int(np.argmax(np.abs(residual)))
    print(f"largest disagreement   : {residual[worst]:+.3f} deg")

    shared_legend(fig, list(axes), ncol=2, bottom=0.20)
    out = os.path.join(args.out_dir, "MAIN_DQ_metric_comparison.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
