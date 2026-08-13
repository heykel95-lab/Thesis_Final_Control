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

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
METRICS = os.path.join(HERE, "..", "experiments", "derived", "metrics.csv")

FONT_STYLE = "latex"
SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"
SERIES_BLUE = "#0057b8"
SERIES_COLOURS = (SERIES_BLACK, SERIES_RED, SERIES_BLUE, "#e0ad00")

_FONT_STYLES = {
    "latex": {"font.serif": ["Latin Modern Roman", "CMU Serif", "cmr10",
                             "DejaVu Serif"],
              "mathtext.fontset": "cm"},
}

plt.rcParams.update({
    "font.family": "serif",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.size": 9,
    "axes.prop_cycle": matplotlib.cycler(color=SERIES_COLOURS),
    "axes.edgecolor": "#1a1a1a",
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "axes.grid.axis": "y",
    "grid.alpha": 0.3,
    "grid.linewidth": 0.6,
    "xtick.direction": "out",
    "ytick.direction": "out",
    "lines.linewidth": 1.25,
    "lines.markersize": 5.5,
    "legend.frameon": False,
    "legend.fontsize": 8,
    "legend.handlelength": 1.6,
    "legend.handletextpad": 0.5,
    "legend.columnspacing": 1.2,
})
plt.rcParams.update(_FONT_STYLES[FONT_STYLE])

GAIN_LABEL = r"Alignment change $\Delta\theta_{\mathrm{align}}$ [$^\circ$]"

OFFSETS = [("00deg", "none"), ("t1_05deg", r"$5^\circ$ $t_1$"),
           ("t1_10deg", r"$10^\circ$ $t_1$"), ("t2_05deg", r"$5^\circ$ $t_2$"),
           ("t2_10deg", r"$10^\circ$ $t_2$")]


def load():
    stats = collections.defaultdict(list)
    with open(METRICS) as f:
        for row in csv.DictReader(f):
            if row.get("align_gain_deg"):
                stats[row["run_id"]].append(float(row["align_gain_deg"]))
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

    ax.axhline(0.0, color="#888888", linewidth=0.8, zorder=0)
    ax.set_xticks(x)
    ax.set_xticklabels([lbl for _, lbl in OFFSETS])
    ax.set_xlabel("Commanded tool orientation offset")
    ax.set_ylabel(GAIN_LABEL)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3,
               bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.11, 1, 1))
    fig.savefig(out_path)
    plt.close(fig)


def plot_sweep(stats, prefix, positions, xlabel, out_path):
    fig, ax = plt.subplots(figsize=(5.6, 3.0))
    mean = [stats[f"{prefix}{k}"][0] for k, _ in positions]
    err = [stats[f"{prefix}{k}"][1] for k, _ in positions]
    x = [v for _, v in positions]

    ax.errorbar(x, mean, yerr=err, color=SERIES_BLACK, marker="o",
                markerfacecolor="white", markeredgewidth=1.1, capsize=2.5)
    ax.axhline(0.0, color="#888888", linewidth=0.8, zorder=0)
    ax.set_xticks(x)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(GAIN_LABEL)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    stats = load()

    out = lambda name: os.path.join(args.out_dir, name)
    plot_offsets(stats, out("CASE_JKL_frame.pdf"))
    plot_sweep(stats, "S4_tangential_",
               [("m050", -50), ("m020", -20), ("p000", 0),
                ("p020", 20), ("p050", 50)],
               r"In-plane compliance-centre position [mm]",
               out("CASE_M_inplane.pdf"))
    plot_sweep(stats, "S5_normal_",
               [("m090", -90), ("m060", -60), ("m040", -40), ("p000", 0),
                ("p040", 40), ("p060", 60), ("p090", 90)],
               r"Tool-axis compliance-centre position [mm]",
               out("CASE_N_toolaxis.pdf"))
    print(f"wrote three figures to {os.path.abspath(args.out_dir)}")


if __name__ == "__main__":
    main()
