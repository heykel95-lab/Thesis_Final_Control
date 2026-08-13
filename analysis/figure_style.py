"""Shared drawing conventions for the thesis figures.

Follows MyOwn-thesis/FIGURE_STYLE.md, which owns the rules:

  Latin Modern with Computer Modern maths, so a figure carries the document's
  faces. Categorical colours begin black, red, blue, yellow, with grey reserved
  for reference lines. Horizontal grid only. No internal title, because the
  caption already names the figure. One shared legend below a multi-panel
  figure, assembled from every panel so no series is dropped.

Two conventions are added here. Every line is solid: a broken line reads as a
different kind of quantity, and the panels distinguish their series by colour.
Time series are drawn at a bounded number of points, because a set-up log holds
about five thousand samples and a panel is a few centimetres wide, so the full
rate paints a band rather than a curve.
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Categorical order fixed by the style guide.
SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"
SERIES_BLUE = "#0057b8"
SERIES_YELLOW = "#e0ad00"
SERIES_COLOURS = (SERIES_BLACK, SERIES_RED, SERIES_BLUE, SERIES_YELLOW)

# Grey belongs to reference lines and nothing else.
REFERENCE_GREY = "#888888"

# Marker shapes repeat the series distinction for monochrome printing.
SERIES_MARKERS = ("o", "s", "^", "D")

# Points kept when drawing a time series, so a curve stays a curve.
MAX_TIME_SERIES_POINTS = 900


def apply_style():
    """Set the drawing conventions for every figure in this repository."""
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Latin Modern Roman", "CMU Serif", "cmr10",
                       "DejaVu Serif"],
        "mathtext.fontset": "cm",
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


def thin(*arrays):
    """Return the arrays sampled down to a drawable number of points."""
    length = len(arrays[0])
    step = max(1, length // MAX_TIME_SERIES_POINTS)
    return tuple(a[::step] for a in arrays)


def shared_legend(fig, axes, ncol=3, bottom=0.13):
    """Place one legend below the panels, assembled from all of them.

    Entries are collected in the order the panels were drawn and repeated
    labels are dropped, so a series appearing in several panels is listed once.
    """
    handles, labels = [], []
    for ax in axes:
        for handle, label in zip(*ax.get_legend_handles_labels()):
            if label not in labels:
                handles.append(handle)
                labels.append(label)
    if not handles:
        return
    fig.legend(handles, labels, loc="lower center", ncol=ncol,
               bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, bottom, 1, 1))


def reference_line(ax, value=0.0):
    """Draw one horizontal reference line where the value means something."""
    ax.axhline(value, color=REFERENCE_GREY, linewidth=0.8, zorder=0)
