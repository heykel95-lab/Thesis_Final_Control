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
drawn. The sweeps carry the sign of the change, because a condition that makes
the alignment worse is as much a result as one that improves it.
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
from figure_style import (apply_style, reference_line, shared_legend,  # noqa: E402
                          SERIES_COLOURS, SERIES_MARKERS)

apply_style()

GAIN_LABEL = r"Change in angular deviation [$^\circ$]"

# The four groups of the sweep, in the order they are drawn.
GROUPS = [
    ("P2_t1_pos", r"$+10^\circ$ about $t_1$"),
    ("P2_t1_neg", r"$-10^\circ$ about $t_1$"),
    ("P2_t2_pos", r"$+10^\circ$ about $t_2$"),
    ("P2_t2_neg", r"$-10^\circ$ about $t_2$"),
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
            if not run.startswith(("P2_", "P3_", "P4_", "P5_")):
                continue
            for key in ("deviation_gain_deg", "deviation_after_deg",
                        "deviation_before_deg"):
                try:
                    groups[run][key].append(float(row[key]))
                except (KeyError, ValueError):
                    pass
    return groups


def stat(groups, run, key):
    """Mean and sample standard deviation, or None when the run is missing."""
    values = groups.get(run, {}).get(key, [])
    if not values:
        return None
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return statistics.mean(values), sd


def tag(position):
    return f"{'m' if position < 0 else 'p'}{abs(position):03d}"


def sweep(groups, prefix, positions):
    """Return the positions, means and deviations present for one group."""
    x, y, err = [], [], []
    for position in positions:
        value = stat(groups, f"{prefix}_{tag(position)}", "deviation_gain_deg")
        if value is None:
            continue
        x.append(position)
        y.append(value[0])
        err.append(value[1])
    return np.array(x), np.array(y), np.array(err)


def draw_sweep(entries, xlabel, out_path, figsize=(5.8, 3.4)):
    fig, ax = plt.subplots(figsize=figsize)
    for (x, y, err, label), colour, marker in zip(entries, SERIES_COLOURS,
                                                  SERIES_MARKERS):
        ax.errorbar(x, y, yerr=err, color=colour, marker=marker,
                    markerfacecolor="white", markeredgewidth=1.1,
                    capsize=2.5, label=label)
    reference_line(ax)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(GAIN_LABEL)
    shared_legend(fig, [ax], ncol=2, bottom=0.20)
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
    for prefix, label in GROUPS:
        value = stat(groups, f"{prefix}_p000", "deviation_gain_deg")
        if value is None:
            continue
        labels.append(label)
        values.append(value[0])
        errors.append(value[1])
    ax.bar(range(len(values)), values, yerr=errors, capsize=3,
           color=SERIES_COLOURS[0], edgecolor="#1a1a1a", linewidth=0.8,
           width=0.55)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels)
    reference_line(ax)
    ax.set_ylabel(GAIN_LABEL)
    fig.tight_layout()
    fig.savefig(out("MAIN_D_contact.pdf"))
    fig.savefig(out("MAIN_D_contact.png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out('MAIN_D_contact.pdf'))}")

    # E -- the tangential sweep, all four groups on one panel.
    entries = []
    for prefix, label in GROUPS:
        x, y, err = sweep(groups, prefix, POSITIONS)
        if len(x):
            entries.append((x, y, err, label))
    draw_sweep(entries, "Centre position along the assisting tangent [mm]",
               out("MAIN_E_sign.pdf"), figsize=(5.8, 3.8))

    # G -- the tool-axis sweep. Its zero is the tool-frame trial of case E.
    x, y, err = sweep(groups, "P3_axis", POSITIONS)
    zero = stat(groups, "P2_t1_pos_p000", "deviation_gain_deg")
    if zero is not None:
        x = np.append(x, 0)
        y = np.append(y, zero[0])
        err = np.append(err, zero[1])
        order = np.argsort(x)
        x, y, err = x[order], y[order], err[order]
    draw_sweep([(x, y, err, r"$+10^\circ$ about $t_1$")],
               "Centre position along the tool axis [mm]",
               out("MAIN_G_toolaxis.pdf"))

    # H -- the same two positions at both commanded magnitudes.
    entries = []
    for axis, marker_label in (("t1", r"$t_1$"), ("t2", r"$t_2$")):
        x, y, err = [], [], []
        for position, prefix in ((0, f"P5_mag_{axis}_p000"),
                                 (40, f"P5_mag_{axis}_p040")):
            value = stat(groups, prefix, "deviation_gain_deg")
            if value is None:
                continue
            x.append(position)
            y.append(value[0])
            err.append(value[1])
        if x:
            entries.append((np.array(x), np.array(y), np.array(err),
                            rf"$+5^\circ$ about {marker_label}"))
    for axis, marker_label in (("t1", r"$t_1$"), ("t2", r"$t_2$")):
        x, y, err = [], [], []
        for position in (0, 40):
            value = stat(groups, f"P2_{axis}_pos_{tag(position)}",
                         "deviation_gain_deg")
            if value is None:
                continue
            x.append(position)
            y.append(value[0])
            err.append(value[1])
        if x:
            entries.append((np.array(x), np.array(y), np.array(err),
                            rf"$+10^\circ$ about {marker_label}"))
    draw_sweep(entries, "Centre position along the assisting tangent [mm]",
               out("MAIN_H_magnitude.pdf"), figsize=(5.8, 3.8))


if __name__ == "__main__":
    main()
