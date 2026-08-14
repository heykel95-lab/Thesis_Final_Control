#!/usr/bin/env python3
"""Draw the Case J to N figures from experiments/derived/metrics.csv.

  python3 analysis/make_case_figures.py [--out-dir DIR]

Writes one vector PDF per case, matched to the thesis figure style: Latin
Modern with Computer Modern maths, the black-red-blue-yellow series order, open
markers with a white face, and a horizontal grid only.

Cases J and K share one figure, because the comparison between them is what
either one is read for. L is a two-condition comparison and is drawn beside
them. M and N are position sweeps and get one panel each.
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
                          SERIES_BLACK, SERIES_RED, SERIES_BLUE)

apply_style()

GAIN_LABEL = r"Change in angular deviation [$^\circ$]"

OFFSETS = [("00deg", "none"), ("t1_05deg", r"$5^\circ$ $t_1$"),
           ("t1_10deg", r"$10^\circ$ $t_1$"), ("t2_05deg", r"$5^\circ$ $t_2$"),
           ("t2_10deg", r"$10^\circ$ $t_2$")]


def load():
    stats = collections.defaultdict(list)
    with open(METRICS) as f:
        for row in csv.DictReader(f):
            if row.get("deviation_gain_deg"):
                stats[row["run_id"]].append(float(row["deviation_gain_deg"]))
    return {k: (statistics.mean(v),
                statistics.stdev(v) if len(v) > 1 else 0.0)
            for k, v in stats.items()}


def series(stats, prefix, keys):
    mean, err, seen = [], [], []
    for key, label in keys:
        entry = stats.get(f"{prefix}{key}")
        if entry is None:
            continue
        mean.append(entry[0])
        err.append(entry[1])
        seen.append(label)
    return seen, mean, err


def plot_offsets(stats, out_path):
    """Cases J, K and L against the commanded offset."""
    fig, ax = plt.subplots(figsize=(5.6, 3.1))
    x = np.arange(len(OFFSETS))

    # The surface-frame arm was sampled at two of the five offsets. Joining
    # those two would draw a line through a condition it never visited, so it
    # is left as markers.
    for prefix, colour, marker, style, label in (
            ("S1_none_", SERIES_BLACK, "o", "-", "no lever"),
            ("S2_tool_", SERIES_RED, "s", "-", "tool frame"),
            ("S3_surface_", SERIES_BLUE, "^", "none", "surface frame")):
        labels, mean, err = series(stats, prefix, OFFSETS)
        idx = [i for i, (k, _) in enumerate(OFFSETS)
               if f"{prefix}{k}" in stats]
        ax.errorbar(idx, mean, yerr=err, color=colour, marker=marker,
                    linestyle=style, markerfacecolor="white",
                    markeredgewidth=1.1, capsize=2.5, label=label)

    reference_line(ax)
    ax.set_xticks(x)
    ax.set_xticklabels([lbl for _, lbl in OFFSETS])
    ax.set_xlabel("Commanded tool orientation offset")
    ax.set_ylabel(GAIN_LABEL)
    shared_legend(fig, [ax], ncol=3, bottom=0.13)
    fig.savefig(out_path)
    plt.close(fig)


def plot_sweep(stats, prefix, positions, xlabel, out_path, series_label):
    fig, ax = plt.subplots(figsize=(5.6, 3.0))
    mean = [stats[f"{prefix}{k}"][0] for k, _ in positions]
    err = [stats[f"{prefix}{k}"][1] for k, _ in positions]
    x = [v for _, v in positions]

    ax.errorbar(x, mean, yerr=err, color=SERIES_BLACK, marker="o",
                markerfacecolor="white", markeredgewidth=1.1, capsize=2.5,
                label=series_label)
    reference_line(ax)
    ax.set_xticks(x)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(GAIN_LABEL)
    shared_legend(fig, [ax], ncol=1, bottom=0.13)
    fig.savefig(out_path)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stats = load()

    out = lambda name: os.path.join(args.out_dir, name)
    plot_offsets(stats, out("MAIN_JKL_frame.pdf"))
    plot_sweep(stats, "S4_tangential_",
               [("m050", -50), ("m020", -20), ("p000", 0),
                ("p020", 20), ("p050", 50)],
               r"In-plane compliance-centre position [mm]",
               out("MAIN_M_inplane.pdf"),
               r"tool frame, $10^\circ$ about $t_1$")
    plot_sweep(stats, "S5_normal_",
               [("m090", -90), ("m060", -60), ("m040", -40), ("p000", 0),
                ("p040", 40), ("p060", 60), ("p090", 90)],
               r"Tool-axis compliance-centre position [mm]",
               out("MAIN_N_toolaxis.pdf"),
               r"tool frame, $10^\circ$ about $t_1$")
    print(f"wrote three figures to {os.path.abspath(args.out_dir)}")


if __name__ == "__main__":
    main()
