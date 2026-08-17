#!/usr/bin/env python3
"""Draw the centre-of-compliance case figures from experiments/derived/metrics.csv.

  python3 analysis/make_coc_figures.py [--out-dir DIR]

One figure per case, matched to the thesis figure style: Latin Modern with
Computer Modern maths, the black-red-blue-yellow series order, open markers
with a white face, a horizontal grid, and no dashed line anywhere.

  D  the contact alone on each tangent, at both commanded signs
  E  the centre swept along the assisting tangent, all four groups together
  G  the centre swept along the tool axis
  H  the same positions at half the commanded offset

Case F is a two-condition comparison and is reported as a table rather than
drawn. Every case uses the signed set-up rotation about the commanded tangent.
This controller-response metric does not depend on the reconstructed tool
normal or its uncertain absolute zero.
"""

import argparse
import collections
import csv
import os
import statistics
import sys

import numpy as np
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
METRICS = os.path.join(HERE, "..", "experiments", "derived", "metrics.csv")

sys.path.insert(0, HERE)
from figure_style import (apply_style, reference_line,  # noqa: E402
                          SERIES_COLOURS, SERIES_MARKERS)

apply_style()

ROTATION_LABEL = (r"Set-up rotation about the commanded tangent"
                  r" [$^\circ$]")

# The four groups of the sweep, in the order they are drawn. Their legend
# labels are formed from the measured orientation at first contact below; the
# nominal commanded offset is deliberately not used as a substitute for it.
GROUPS = [
    ("P2_t1_pos", "t1"),
    ("P2_t1_neg", "t1"),
    ("P2_t2_pos", "t2"),
    ("P2_t2_neg", "t2"),
]

# Positions the campaign carried [mm]. 80 was measured before the magnitude
# was capped and is kept out of the sweeps so every group spans one range.
POSITIONS = [-40, -20, -10, 0, 10, 20, 40]


def load():
    """Return {run_id: {column: [values]}} for the campaign trials."""
    groups = collections.defaultdict(lambda: collections.defaultdict(list))
    with open(METRICS) as f:
        for row in csv.DictReader(f):
            run = row["run_id"]
            if not run.startswith(("A_", "B_", "C_", "P2_", "P3_",
                                   "P4_", "P5_", "P6_")):
                continue
            for key in ("deviation_gain_deg", "deviation_after_deg",
                        "deviation_before_deg",
                        "deviation_before_t1", "deviation_before_t2",
                        "contact_rotation_t1_deg", "contact_rotation_t2_deg"):
                try:
                    groups[run][key].append(float(row[key]))
                except (KeyError, ValueError):
                    pass
    return groups


ROTATION_KEY = {"rotation_t1": "contact_rotation_t1_deg",
                "rotation_t2": "contact_rotation_t2_deg"}


def stat(groups, run, key):
    """Mean and sample standard deviation, or None when the run is missing."""
    values = groups.get(run, {}).get(ROTATION_KEY.get(key, key), [])
    if not values:
        return None
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return statistics.mean(values), sd


def tag(position):
    return f"{'m' if position < 0 else 'p'}{abs(position):03d}"


def initial_label(groups, runs, axis, linebreak=False):
    """Name a series by its measured signed orientation before set-up."""
    key = f"deviation_before_{axis}"
    values = [value for run in runs for value in groups.get(run, {}).get(key, [])]
    if not values:
        raise ValueError(f"no {key} values for {runs}")
    separator = "\n" if linebreak else " "
    tangent = "t_1" if axis == "t1" else "t_2"
    return (f"initial{separator}${statistics.mean(values):+.2f}^\\circ$ "
            f"about ${tangent}$")


def sweep(groups, prefix, positions, key):
    """Return positions and signed set-up-rotation statistics for one group."""
    x, y, err = [], [], []
    for position in positions:
        value = stat(groups, f"{prefix}_{tag(position)}", key)
        if value is None:
            continue
        x.append(position)
        y.append(value[0])
        err.append(value[1])
    return np.array(x), np.array(y), np.array(err)


def draw_sweep(entries, xlabel, out_path, figsize=(5.8, 3.4),
               ylabel=ROTATION_LABEL, headroom=0.30):
    fig, ax = plt.subplots(figsize=figsize)
    for (x, y, err, label), colour, marker in zip(entries, SERIES_COLOURS,
                                                  SERIES_MARKERS):
        ax.errorbar(x, y, yerr=err, color=colour, marker=marker,
                    markerfacecolor="white", markeredgewidth=1.1,
                    capsize=2.5, label=label)
    reference_line(ax)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    # Headroom so the corner legend sits above the data rather than on it.
    ax.margins(y=headroom)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path)
    fig.savefig(out_path.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out_path)}")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    groups = load()
    out = lambda name: os.path.join(args.out_dir, name)

    # D -- the contact alone, one bar per group at the zero position.
    fig, ax = plt.subplots(figsize=(5.6, 3.2))
    labels, values, errors = [], [], []
    for prefix, axis in GROUPS:
        run = f"{prefix}_p000"
        value = stat(groups, run, f"rotation_{axis}")
        if value is None:
            continue
        labels.append(initial_label(groups, [run], axis, linebreak=True))
        values.append(value[0])
        errors.append(value[1])
    ax.bar(range(len(values)), values, yerr=errors, capsize=3,
           color=SERIES_COLOURS[0], edgecolor="#1a1a1a", linewidth=0.8,
           width=0.55)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels)
    reference_line(ax)
    ax.set_ylabel(ROTATION_LABEL)
    fig.tight_layout()
    fig.savefig(out("MAIN_D_contact.pdf"))
    fig.savefig(out("MAIN_D_contact.png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out('MAIN_D_contact.pdf'))}")

    # A -- the rotational entry, drawn as the rotation the robot made.
    for case, key, xlabel, values, name in (
            ("A", "A_rot", r"Rotational stiffness about the commanded tangent"
             " [N m/rad]", [5, 15, 50], "MAIN_A_KR.pdf"),
            ("B", "B_trans", r"Translational stiffness across the commanded"
             " tangent [N/m]", [300, 800, 2000], "MAIN_B_KP.pdf")):
        entries = []
        for axis in ("t1", "t2"):
            x, y, err, runs = [], [], [], []
            for v in values:
                # The shared value of every other case is the Case-D trial.
                run = (f"P2_{axis}_pos_p000" if v in (5, 2000)
                       else f"{key}_{axis}_{v:02d}" if case == "A"
                       else f"{key}_{axis}_{v:04d}")
                stats = stat(groups, run, f"rotation_{axis}")
                if stats is None:
                    continue
                x.append(v)
                y.append(stats[0])
                err.append(stats[1])
                runs.append(run)
            if x:
                entries.append((np.array(x), np.array(y), np.array(err),
                                initial_label(groups, runs, axis)))
        if entries:
            draw_sweep(entries, xlabel, out(name),
                       ylabel=ROTATION_LABEL)

    # E -- the tangential sweep, all four groups on one panel.
    entries = []
    for prefix, axis in GROUPS:
        x, y, err = sweep(groups, prefix, POSITIONS, f"rotation_{axis}")
        if len(x):
            runs = [f"{prefix}_{tag(position)}" for position in x]
            entries.append((x, y, err, initial_label(groups, runs, axis)))
    draw_sweep(entries, "Centre position along the assisting tangent [mm]",
               out("MAIN_E_sign.pdf"), figsize=(5.8, 3.8))

    # G -- the tool-axis sweep. Its zero is the tool-frame trial of case E.
    x, y, err = sweep(groups, "P3_axis", POSITIONS, "rotation_t1")
    zero = stat(groups, "P2_t1_pos_p000", "rotation_t1")
    if zero is not None:
        x = np.append(x, 0)
        y = np.append(y, zero[0])
        err = np.append(err, zero[1])
        order = np.argsort(x)
        x, y, err = x[order], y[order], err[order]
    runs = [f"P3_axis_{tag(position)}" for position in POSITIONS if position]
    runs.append("P2_t1_pos_p000")
    draw_sweep([(x, y, err, initial_label(groups, runs, "t1"))],
               "Centre position along the tool axis [mm]",
               out("MAIN_G_toolaxis.pdf"))

    # H -- the same two positions at both commanded magnitudes.
    entries = []
    for axis in ("t1", "t2"):
        x, y, err, runs = [], [], [], []
        for position, prefix in ((0, f"P5_mag_{axis}_p000"),
                                 (40, f"P5_mag_{axis}_p040")):
            value = stat(groups, prefix, f"rotation_{axis}")
            if value is None:
                continue
            x.append(position)
            y.append(value[0])
            err.append(value[1])
            runs.append(prefix)
        if x:
            entries.append((np.array(x), np.array(y), np.array(err),
                            initial_label(groups, runs, axis)))
    for axis in ("t1", "t2"):
        x, y, err, runs = [], [], [], []
        for position in (0, 40):
            run = f"P2_{axis}_pos_{tag(position)}"
            value = stat(groups, run, f"rotation_{axis}")
            if value is None:
                continue
            x.append(position)
            y.append(value[0])
            err.append(value[1])
            runs.append(run)
        if x:
            entries.append((np.array(x), np.array(y), np.array(err),
                            initial_label(groups, runs, axis)))
    draw_sweep(entries, "Centre position along the assisting tangent [mm]",
               out("MAIN_H_magnitude.pdf"), figsize=(5.8, 3.8), headroom=0.75)


if __name__ == "__main__":
    main()
