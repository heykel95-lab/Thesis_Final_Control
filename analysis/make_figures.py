#!/usr/bin/env python3
"""Build the thesis figures from experiments/derived/metrics.csv.

  python3 experiments/analysis/make_figures.py

Writes PDFs into experiments/figures/. Each figure corresponds to a specific
table or claim in the thesis, named in FIGURES below, so it is obvious which
result a plot is meant to support.

Flags come in two kinds and are treated differently.

DATA flags (not-converged, no-setup-phase, no-general-log, tip-mismatch,
task-disturbed, tool-play) mean the numbers themselves are untrustworthy for
the primary physical-tool claim. Those runs are excluded from every mean and
from the generated plots.

PROVENANCE flags -- currently just dirty-tree -- mean the repository had an
uncommitted change when the run was recorded. That says nothing about the
measurement: every run archives the exact configuration it used in
params_effective/, so it remains reproducible. Excluding these would have
dropped all twelve B3 runs, the largest effect in the campaign, on a
bookkeeping technicality. They are included, and marked in the caption.
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.normpath(os.path.join(HERE, ".."))
METRICS = os.path.join(EXP, "derived", "metrics.csv")
FIGURES = os.path.join(EXP, "figures")

# Text is matched to the thesis rather than to matplotlib's defaults. usetex
# is deliberately not used: it needs dvipng for the Agg backend and dvipng is
# not installed, and it would make every figure depend on a preamble kept
# somewhere else. Naming the same faces gets the same look without that.
#
#   "latex" -- Latin Modern Roman with Computer Modern maths, the face plain
#              LaTeX sets when the preamble loads no font package.
#   "times" -- Liberation Serif, the installed metric-compatible Times face,
#              with STIX maths. This is what newtxtext or mathptmx give.
#
# Switch here if the thesis loads a font package; nothing else needs changing.
FONT_STYLE = "latex"

# The categorical palette begins with the agreed black, red, blue and yellow.
# Further distinct colours are available when a plot genuinely needs more than
# four curves. Marker shape duplicates colour for monochrome print.
SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"
SERIES_BLUE = "#0057b8"
SERIES_YELLOW = "#e0ad00"
SERIES_GREEN = "#008450"
SERIES_PURPLE = "#7b3294"
SERIES_CYAN = "#008c95"
SERIES_ORANGE = "#e66100"
SERIES_COLOURS = (
    SERIES_BLACK,
    SERIES_RED,
    SERIES_BLUE,
    SERIES_YELLOW,
    SERIES_GREEN,
    SERIES_PURPLE,
    SERIES_CYAN,
    SERIES_ORANGE,
)

ALIGNMENT_IMPROVEMENT_LABEL = (
    r"Alignment improvement "
    r"$\theta_{\mathrm{initial}}-\theta_{\mathrm{final}}$ [$^\circ$]"
)
INITIAL_MISALIGNMENT_LABEL = (
    r"Initial misalignment $\theta_{\mathrm{initial}}$ [$^\circ$]"
)
FINAL_MISALIGNMENT_LABEL = (
    r"Final misalignment $\theta_{\mathrm{final}}$ [$^\circ$]"
)
STEADY_ESTIMATED_LOAD_LABEL = r"Steady estimated load $f$ [N]"
ALIGNMENT_TIME_LABEL = r"90% Alignment time $t_{90}$ [s]"
FEATURE_TRAVEL_LABEL = r"Selected-feature travel $s_g$ [mm]"
ROTATIONAL_STIFFNESS_LABEL = (
    "Rotation-axis stiffness\n"
    r"$K_{R,t_1}$ / $K_{R,t_2}$ "
    r"[$\mathrm{N\,m/rad}$]"
)
TRANSLATIONAL_STIFFNESS_LABEL = (
    "Perpendicular translational stiffness\n"
    r"$K_{p,t_2}$ / $K_{p,t_1}$ "
    r"[$\mathrm{N/m}$]"
)
COMPLIANCE_LEVER_LABEL = (
    "Compliance-centre lever\n"
    r"$r_{c,t_2}$ / $r_{c,t_1}$ [mm]"
)
LEVER_MAGNITUDE_LABEL = (
    r"Compliance-centre lever magnitude $\rho_c=|r_{c,t}|$ [mm]"
)

_FONT_STYLES = {
    "latex": {
        "font.serif": ["Latin Modern Roman", "CMU Serif", "cmr10",
                       "DejaVu Serif"],
        "mathtext.fontset": "cm",
    },
    "times": {
        "font.serif": ["Liberation Serif", "Times New Roman", "Times",
                       "Nimbus Roman"],
        "mathtext.fontset": "stix",
    },
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
    "xtick.major.width": 0.8,
    "ytick.major.width": 0.8,
    "lines.linewidth": 1.25,
    "lines.markersize": 5.5,
    # One legend everywhere: no box, no shadow, the same size relative to the
    # body text, and tight enough not to crowd the data.
    "legend.frameon": False,
    "legend.fontsize": 8,
    "legend.handlelength": 1.6,
    "legend.handletextpad": 0.5,
    "legend.labelspacing": 0.3,
    "legend.columnspacing": 1.2,
    "legend.borderaxespad": 0.4,
})
plt.rcParams.update(_FONT_STYLES[FONT_STYLE])


def load_metrics(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
        rows = []
        for line in f:
            if line.strip():
                rows.append(dict(zip(header, line.rstrip("\n").split(","))))
    return rows


def fnum(row, key):
    v = row.get(key, "")
    if v in ("", "nan", "None"):
        return np.nan
    try:
        return float(v)
    except ValueError:
        return np.nan


def errorbar_from_buckets(ax, buckets, label, color, marker="o"):
    xs = sorted(k for k in buckets if not np.isnan(k))
    if not xs:
        return False
    means, errs, plotted = [], [], []
    for x in xs:
        vals = buckets[x]["good"]
        if not vals:
            continue
        plotted.append(x)
        means.append(np.mean(vals))
        errs.append(np.std(vals, ddof=1) if len(vals) > 1 else 0.0)
    if plotted:
        ax.errorbar(plotted, means, yerr=errs, marker=marker, capsize=3,
                    label=label, color=color, linewidth=1.25,
                    elinewidth=1.0, capthick=1.0, markersize=5.5,
                    markerfacecolor="white", markeredgecolor=color,
                    markeredgewidth=1.1)
    return bool(plotted)


def sweep_axis(ax, values, pad=0.12):
    """A linear sweep axis ticked only where a setting was tested.

    Three settings do not justify a log scale, and a log decade fills itself
    with minor labels that collide at the printed width. Linear spacing with
    ticks at the tested values shows the spacing of the sample honestly and
    leaves the grid to the horizontal lines.
    """
    ticks = sorted(set(values))
    span = ticks[-1] - ticks[0]
    ax.set_xlim(ticks[0] - pad * span, ticks[-1] + pad * span)
    ax.set_xticks(ticks)
    ax.set_xticklabels([f"{v:g}" for v in ticks])


# Figures carry no internal title: the thesis caption identifies each one, and
# a title inside the axes repeats it on the page.
def _legend_items_in_colour_order(handles, labels, ncol):
    """Sort categorical colours and preserve row order."""
    colour_rank = {
        matplotlib.colors.to_hex(colour).lower(): rank
        for rank, colour in enumerate(SERIES_COLOURS)
    }

    def handle_colour(handle):
        candidate = handle
        if hasattr(handle, "lines") and handle.lines:
            candidate = handle.lines[0]
        if not hasattr(candidate, "get_color"):
            return ""
        try:
            return matplotlib.colors.to_hex(candidate.get_color()).lower()
        except (TypeError, ValueError):
            return ""

    items = list(zip(handles, labels))
    items.sort(
        key=lambda item: colour_rank.get(
            handle_colour(item[0]), len(SERIES_COLOURS)
        )
    )

    # Matplotlib fills a multi-column legend down each column. Reorder the
    # handles so that reading across the rendered rows still gives
    # the declared palette order.
    count = len(items)
    columns = min(max(1, ncol), count)
    if columns > 1:
        ordered = []
        for column in range(columns):
            rows_in_column = count // columns + (column < count % columns)
            for row in range(rows_in_column):
                index = row * columns + column
                if index < count:
                    ordered.append(items[index])
        items = ordered
    return [item[0] for item in items], [item[1] for item in items]


def axis_legend(ax, ncol=1, **kwargs):
    handles, labels = ax.get_legend_handles_labels()
    handles, labels = _legend_items_in_colour_order(handles, labels, ncol)
    if handles:
        ax.legend(handles, labels, ncol=ncol, **kwargs)


def figure_legend(fig, axes, ncol=3):
    """One legend under a multi-panel figure.

    An in-axes legend on a narrow subplot lands on the y label or the data.
    Below the figure it belongs to every panel at once, which is what a shared
    series list means anyway, and bbox_inches="tight" keeps it in the crop.
    """
    handles, labels = [], []
    for ax in np.atleast_1d(axes).flat:
        axis_handles, axis_labels = ax.get_legend_handles_labels()
        for handle, label in zip(axis_handles, axis_labels):
            if label and label not in labels:
                handles.append(handle)
                labels.append(label)
    if not handles:
        return
    handles, labels = _legend_items_in_colour_order(handles, labels, ncol)
    fig.legend(handles, labels, loc="lower center", ncol=ncol,
               bbox_to_anchor=(0.5, 0.01), fontsize=8)
    rows = int(np.ceil(len(handles) / min(ncol, len(handles))))
    fig._thesis_legend_bottom = 0.10 + 0.06 * (rows - 1)


def save(fig, name):
    os.makedirs(FIGURES, exist_ok=True)
    path = os.path.join(FIGURES, name)
    bottom = getattr(fig, "_thesis_legend_bottom", 0.0)
    top = 1.0 if fig._suptitle is None else 0.94
    fig.tight_layout(rect=(0.0, bottom, 1.0, top))
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {os.path.relpath(path, EXP)}")
    return path


# ---------------------------------------------------------------------------


def fig_a2_stiffness(rows):
    """Replaces the invented Table 5.1 (candidate stiffness settings)."""
    sub = [r for r in rows if r["run_id"].startswith("A2_KRtan")]
    if not sub:
        return None
    for r in sub:
        r["_kr"] = float(r["run_id"].split("_")[-1])

    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.0))
    for ax, key, ylabel in (
        (axes[0], "align_gain_deg", "alignment gain [deg]"),
        (axes[1], "force_steady_N", "steady contact force [N]"),
        (axes[2], "tau_norm_max_Nm", "peak commanded torque [Nm]"),
    ):
        buckets = {}
        for r in sub:
            y = fnum(r, key)
            if np.isnan(y):
                continue
            buckets.setdefault(r["_kr"], {"good": [], "bad": []})
            buckets[r["_kr"]]["good" if not data_suspect(r) else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.set_xlabel(r"$K_{R,t_1}=K_{R,t_2}$ [Nm/rad]")
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes)
    fig.suptitle("Rotational stiffness sweep", fontsize=10)
    return save(fig, "A2_stiffness_sweep.pdf")


def _axis_study_buckets(rows, prefix, xkey, ykey):
    buckets = {}
    for row in rows:
        if not row["run_id"].startswith(prefix):
            continue
        x, y = fnum(row, xkey), fnum(row, ykey)
        if np.isnan(x) or np.isnan(y):
            continue
        buckets.setdefault(x, {"good": [], "bad": []})
        bucket = "bad" if data_suspect(row) else "good"
        buckets[x][bucket].append(y)
    return buckets


def fig_d_axis_stiffness(rows):
    """Independent t1/t2 stiffness effects at a fixed 10-degree mismatch."""
    if not any(r["run_id"].startswith(("D1_", "D2_")) for r in rows):
        return None

    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    panels = (
        ("align_gain_deg", "physical-plane improvement [deg]"),
        ("alignment_time90_s", "90% alignment time [s]"),
        ("force_steady_N", "steady estimated normal load [N]"),
    )
    for ax, (key, ylabel) in zip(axes, panels):
        t1 = _axis_study_buckets(rows, "D1_KRt1_", "setup_KR_t1", key)
        t2 = _axis_study_buckets(rows, "D2_KRt2_", "setup_KR_t2", key)
        errorbar_from_buckets(ax, t1, r"$t_1$ excitation", "C0", marker="o")
        errorbar_from_buckets(ax, t2, r"$t_2$ excitation", "C1", marker="s")
        ax.set_xlabel(r"excited-axis $K_R$ [Nm/rad]")
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes)
    fig.suptitle("Axis-specific rotational stiffness", fontsize=10)
    return save(fig, "D_axis_stiffness.pdf")


def fig_d_initial_angle(rows):
    """Alignment response at 0, 5 and 10 degrees for each tangent axis."""
    selected = [
        row
        for row in rows
        if row["run_id"] == "D0_flat_00deg"
        or row["run_id"].startswith("D3_angle_")
        or row["run_id"] in ("D1_KRt1_05", "D2_KRt2_05")
    ]
    if not selected:
        return None

    fig, ax = plt.subplots(figsize=(5.4, 3.5))
    for axis, prefixes, xkey, color, marker in (
        (r"$t_1$", ("D0_", "D3_angle_t1_", "D1_KRt1_05"),
         "tool_offset_t1_deg", "C0", "o"),
        (r"$t_2$", ("D0_", "D3_angle_t2_", "D2_KRt2_05"),
         "tool_offset_t2_deg", "C1", "s"),
    ):
        buckets = {}
        for row in selected:
            if not row["run_id"].startswith(prefixes):
                continue
            x = abs(fnum(row, xkey))
            y = fnum(row, "align_gain_deg")
            if np.isnan(x) or np.isnan(y):
                continue
            buckets.setdefault(x, {"good": [], "bad": []})
            bucket = "bad" if data_suspect(row) else "good"
            buckets[x][bucket].append(y)
        errorbar_from_buckets(
            ax, buckets, f"offset about {axis}", color, marker=marker
        )
    ax.set_xlabel("initial tool-plane angle [deg]")
    ax.set_ylabel("physical-plane improvement [deg]")
    axis_legend(ax)
    ax.set_title("D0/D3: initial-angle response", fontsize=10)
    return save(fig, "D_initial_angle.pdf")


def _main_rows(rows, prefixes):
    return [
        row for row in rows
        if row["run_id"].startswith(prefixes)
    ]


def fig_main_initial_angle(rows):
    selected = _main_rows(rows, ("MAIN_A",))
    if not selected:
        return None
    fig, ax = plt.subplots(figsize=(5.6, 3.5))
    for axis, ids, color, marker in (
        (1, ("MAIN_A0_", "MAIN_A1_", "MAIN_A2_"),
         "C0", "o"),
        (2, ("MAIN_A0_", "MAIN_A3_", "MAIN_A4_"),
         "C1", "s"),
    ):
        buckets = {}
        for row in selected:
            if not row["run_id"].startswith(ids):
                continue
            x = abs(fnum(row, f"align_t{axis}_before_deg"))
            y = fnum(row, f"align_t{axis}_improve_deg")
            if np.isnan(x) or np.isnan(y):
                continue
            buckets.setdefault(x, {"good": [], "bad": []})
            buckets[x]["bad" if data_suspect(row) else "good"].append(y)
        errorbar_from_buckets(
            ax, buckets, rf"Offset about $t_{axis}$", color, marker=marker
        )
    ax.axhline(0.0, color="0.45", linewidth=1)
    ax.set_xlabel(INITIAL_MISALIGNMENT_LABEL)
    ax.set_ylabel(ALIGNMENT_IMPROVEMENT_LABEL)
    axis_legend(ax)
    return save(fig, "MAIN_A_angle.pdf")


def fig_main_rotational_stiffness(rows):
    selected = _main_rows(rows, ("MAIN_A2_", "MAIN_A4_", "MAIN_B"))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, key, ylabel in (
        (axes[0], "axis_improvement", ALIGNMENT_IMPROVEMENT_LABEL),
        (axes[1], "alignment_time90_s", ALIGNMENT_TIME_LABEL),
        (axes[2], "force_steady_N", STEADY_ESTIMATED_LOAD_LABEL),
    ):
        for axis, ids, xkey, color, marker in (
            (1, ("MAIN_A2_", "MAIN_B1_"), "setup_KR_t1", "C0", "o"),
            (2, ("MAIN_A4_", "MAIN_B2_"), "setup_KR_t2", "C1", "s"),
        ):
            buckets = {}
            for row in selected:
                if not row["run_id"].startswith(ids):
                    continue
                x = fnum(row, xkey)
                y = fnum(
                    row,
                    f"align_t{axis}_improve_deg"
                    if key == "axis_improvement" else key,
                )
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax,
                buckets,
                rf"Offset about $t_{axis}$ ($K_{{R,t_{axis}}}$)",
                color,
                marker=marker,
            )
        sweep_axis(ax, (5, 15, 50))
        ax.set_xlabel(ROTATIONAL_STIFFNESS_LABEL)
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes)
    return save(fig, "MAIN_B_KR.pdf")


def fig_main_translational_stiffness(rows):
    selected = _main_rows(rows, ("MAIN_A2_", "MAIN_A4_", "MAIN_C"))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, key, ylabel in (
        (axes[0], "axis_improvement", ALIGNMENT_IMPROVEMENT_LABEL),
        (axes[1], "edge_travel_mm", FEATURE_TRAVEL_LABEL),
        (axes[2], "force_steady_N", STEADY_ESTIMATED_LOAD_LABEL),
    ):
        for axis, ids, xkey, color, marker in (
            (1, ("MAIN_A2_", "MAIN_C1_KPt2_"), "setup_Kp_t2", "C0", "o"),
            (2, ("MAIN_A4_", "MAIN_C2_KPt1_"), "setup_Kp_t1", "C1", "s"),
        ):
            buckets = {}
            for row in selected:
                if not row["run_id"].startswith(ids):
                    continue
                x = fnum(row, xkey)
                y = fnum(
                    row,
                    f"align_t{axis}_improve_deg"
                    if key == "axis_improvement" else key,
                )
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            stiffness_axis = 2 if axis == 1 else 1
            errorbar_from_buckets(
                ax,
                buckets,
                rf"Offset about $t_{axis}$ ($K_{{p,t_{stiffness_axis}}}$)",
                color,
                marker=marker,
            )
        sweep_axis(ax, (300.0, 800.0, 2000.0))
        ax.set_xlabel(TRANSLATIONAL_STIFFNESS_LABEL)
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes)
    return save(fig, "MAIN_C_KP.pdf")


def fig_main_interaction(rows):
    selected = _main_rows(
        rows,
        ("MAIN_A2_", "MAIN_A4_", "MAIN_B1_KRt1_50",
         "MAIN_B2_KRt2_50", "MAIN_C1_KPt2_0300",
         "MAIN_C2_KPt1_0300", "MAIN_C1_interaction",
         "MAIN_C2_interaction"),
    )
    if not selected:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(7.5, 3.2), sharey=True)
    for axis, ax in enumerate(axes, start=1):
        kr_key = f"setup_KR_t{axis}"
        kp_key = "setup_Kp_t2" if axis == 1 else "setup_Kp_t1"
        for kp, color, marker in ((300.0, "C0", "s"), (2000.0, "C1", "o")):
            buckets = {}
            for row in selected:
                is_axis = (
                    (axis == 1 and row["run_id"].startswith(
                        ("MAIN_A2_", "MAIN_B1_", "MAIN_C1_")))
                    or
                    (axis == 2 and row["run_id"].startswith(
                        ("MAIN_A4_", "MAIN_B2_", "MAIN_C2_")))
                )
                if not is_axis or abs(fnum(row, kp_key) - kp) > 1e-6:
                    continue
                x = fnum(row, kr_key)
                y = fnum(row, f"align_t{axis}_improve_deg")
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax, buckets, rf"$K_p={kp:.0f}$ N/m", color, marker=marker
            )
        ax.set_xlabel(
            rf"Rotation-axis stiffness $K_{{R,t_{axis}}}$ "
            rf"[$\mathrm{{N\,m/rad}}$]"
        )
        ax.set_title(rf"Offset about $t_{axis}$", fontsize=9)
    axes[0].set_ylabel(ALIGNMENT_IMPROVEMENT_LABEL)
    figure_legend(fig, axes)
    return save(fig, "MAIN_C_interaction.pdf")


def fig_main_compliance_centre(rows):
    selected = _main_rows(rows, ("MAIN_D",))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, key, ylabel in (
        (axes[0], "axis_improvement", ALIGNMENT_IMPROVEMENT_LABEL),
        (axes[1], "alignment_time90_s", ALIGNMENT_TIME_LABEL),
        (axes[2], "force_steady_N", STEADY_ESTIMATED_LOAD_LABEL),
    ):
        for axis, prefix, xkey, color, marker in (
            (1, "MAIN_D1_", "rc_t2_mm", "C0", "o"),
            (2, "MAIN_D2_", "rc_t1_mm", "C1", "s"),
        ):
            buckets = {}
            for row in selected:
                if not row["run_id"].startswith(prefix):
                    continue
                x = fnum(row, xkey)
                y = fnum(
                    row,
                    f"align_t{axis}_improve_deg"
                    if key == "axis_improvement" else key,
                )
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(x, {"good": [], "bad": []})
                buckets[x]["bad" if data_suspect(row) else "good"].append(y)
            lever_axis = 2 if axis == 1 else 1
            errorbar_from_buckets(
                ax,
                buckets,
                rf"Offset about $t_{axis}$ ($r_{{c,t_{lever_axis}}}$)",
                color,
                marker=marker,
            )
        # D3 has no perpendicular lever to sit on the x axis: its pole is
        # 20 mm along the tool axis, off the tangent plane the sweep lives in.
        # Drawn as a level rather than a point, so the comparison against the
        # r_c = 0 runs is readable without inventing an x position for it.
        for axis, prefix, color in ((1, "MAIN_D3_t1_", "C0"),
                                    (2, "MAIN_D3_t2_", "C1")):
            vals = [
                fnum(row, f"align_t{axis}_improve_deg"
                     if key == "axis_improvement" else key)
                for row in selected
                if row["run_id"].startswith(prefix) and not data_suspect(row)
            ]
            vals = [v for v in vals if not np.isnan(v)]
            if vals:
                ax.axhline(
                    float(np.mean(vals)), color=color, linewidth=1.1,
                    linestyle=":",
                    label=rf"Offset about $t_{axis}$, face centre",
                )
        # Zero means "no correction" for the improvement panel only. On the
        # load and time panels it forces the axis down to zero and squashes the
        # data into a corner.
        if key == "axis_improvement":
            ax.axhline(0.0, color="0.45", linewidth=1)
        sweep_axis(ax, (-60, 0, 60))
        ax.set_xlabel(COMPLIANCE_LEVER_LABEL)
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes, ncol=5)
    return save(fig, "MAIN_D_CoC.pdf")


def _lever_buckets(rows, prefix, xkey, ykey, transform=abs):
    """Runs under one prefix, bucketed by their commanded lever."""
    buckets = {}
    for row in rows:
        if not row["run_id"].startswith(prefix):
            continue
        x = fnum(row, xkey)
        y = fnum(row, ykey)
        if np.isnan(x) or np.isnan(y):
            continue
        x = transform(x) if transform is not None else x
        buckets.setdefault(x, {"good": [], "bad": []})
        buckets[x]["bad" if data_suspect(row) else "good"].append(y)
    return buckets


def fig_main_sign_symmetry(rows):
    """Case F against Case D: does the lever reverse with the tilt?

    One panel per surface axis, both tilt signs on the same lever axis. The
    moment rule makes an exact prediction here -- negating the tilt negates the
    lever that corrects it and changes nothing else -- so the two curves should
    be reflections of each other in the vertical axis. That reflection is drawn
    explicitly, as the positive-tilt curve mirrored, because the claim being
    tested is a shape and not a single number: if the mirrored curve lands on
    the measured negative-tilt one, the pole is a function of the measured
    tilt; if the negative-tilt curve instead tracks the positive one unmirrored,
    the pole is a fixed property of the fixture and the sign rule is wrong.
    """
    selected = _main_rows(rows, ("MAIN_F",))
    if not selected:
        return None
    paired = _main_rows(rows, ("MAIN_D1_", "MAIN_D2_"))
    fig, axes = plt.subplots(1, 2, figsize=(7.4, 3.3))
    for ax, axis in zip(axes, (1, 2)):
        xkey = "rc_t2_mm" if axis == 1 else "rc_t1_mm"
        ykey = f"align_t{axis}_improve_deg"
        pos = _lever_buckets(paired, f"MAIN_D{axis}_t{axis}_rc_",
                             xkey, ykey, transform=None)
        neg = _lever_buckets(selected, f"MAIN_F{axis}_t{axis}neg_rc_",
                             xkey, ykey, transform=None)
        # The prediction first, so the measurements are drawn over it.
        mirrored = {-x: v for x, v in pos.items()}
        if mirrored:
            xs = sorted(mirrored)
            means = [np.mean(mirrored[x]["good"]) for x in xs
                     if mirrored[x]["good"]]
            xs = [x for x in xs if mirrored[x]["good"]]
            if xs:
                # Wide and pale, and drawn first, so that agreement reads as
                # the measured curve sitting inside the predicted band rather
                # than as a line disappearing underneath it.
                ax.plot(xs, means, color="0.72", linewidth=3.2,
                        solid_capstyle="round", zorder=1,
                        label=r"$+10^\circ$ Mirrored")
        errorbar_from_buckets(ax, pos, r"$+10^\circ$ (Case D)",
                              SERIES_BLACK, marker="o")
        errorbar_from_buckets(ax, neg, r"$-10^\circ$ (Case F)",
                              SERIES_RED, marker="s")
        ax.axhline(0.0, color="0.45", linewidth=1)
        sweep_axis(ax, (-60, 0, 60))
        ax.set_xlabel(
            rf"Compliance-centre lever "
            rf"$r_{{c,t_{2 if axis == 1 else 1}}}$ [mm]"
        )
        ax.set_ylabel(ALIGNMENT_IMPROVEMENT_LABEL)
    figure_legend(fig, axes, ncol=3)
    return save(fig, "MAIN_F_sign_symmetry.pdf")


def fig_main_lever_magnitude(rows):
    """Case K: how much lever each initial tilt needs.

    The sign is not a variable here -- Cases D and J settle it -- so every run
    uses the assisting sign for its axis and the x axis is the magnitude alone.
    The 10 deg / 60 mm point of each axis was measured in Case D at the same
    gains, so those runs are read in rather than repeated.

    Three panels, because the most correction is not on its own the best
    setting. The middle panel keeps the sign of the residual: a lever long
    enough to carry the tool past flat shows up there as a crossing below zero,
    which the removed-angle panel cannot distinguish from a good correction.
    """
    selected = _main_rows(rows, ("MAIN_G",))
    if not selected:
        return None
    reused = _main_rows(rows, ("MAIN_D1_t1_rc_t2_m060",
                               "MAIN_D2_t2_rc_t1_p060"))
    series = (
        (1, 5.0, "MAIN_G1_t1_05deg_", SERIES_BLACK, "o"),
        (1, 10.0, "MAIN_G1_t1_10deg_", SERIES_RED, "s"),
        (2, 5.0, "MAIN_G2_t2_05deg_", SERIES_BLUE, "^"),
        (2, 10.0, "MAIN_G2_t2_10deg_", SERIES_YELLOW, "D"),
    )
    fig, axes = plt.subplots(1, 3, figsize=(9.5, 3.1))
    for ax, ykey_template, ylabel, zero_line in (
        (axes[0], "align_t{axis}_improve_deg",
         ALIGNMENT_IMPROVEMENT_LABEL, True),
        (axes[1], "align_t{axis}_after_deg",
         FINAL_MISALIGNMENT_LABEL, True),
        (axes[2], "force_steady_N", STEADY_ESTIMATED_LOAD_LABEL, False),
    ):
        for axis, tilt_deg, prefix, color, marker in series:
            xkey = "rc_t2_mm" if axis == 1 else "rc_t1_mm"
            ykey = ykey_template.format(axis=axis)
            buckets = _lever_buckets(selected, prefix, xkey, ykey)
            if tilt_deg == 10.0:
                for x, v in _lever_buckets(
                    reused, f"MAIN_D{axis}_t{axis}_rc_", xkey, ykey
                ).items():
                    dest = buckets.setdefault(x, {"good": [], "bad": []})
                    dest["good"].extend(v["good"])
                    dest["bad"].extend(v["bad"])
            errorbar_from_buckets(
                ax, buckets,
                rf"$t_{axis}$, ${tilt_deg:.0f}^\circ$",
                color, marker=marker,
            )
        if zero_line:
            ax.axhline(0.0, color="0.45", linewidth=1)
        sweep_axis(ax, (20, 40, 60, 80))
        ax.set_xlabel(LEVER_MAGNITUDE_LABEL)
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes, ncol=5)
    return save(fig, "MAIN_G_lever_magnitude.pdf")


def fig_main_reversed_magnitude(rows):
    """Case H: the reversed tilts at two lever magnitudes.

    Two panels rather than one, because the improvement alone cannot say how
    close the tool ended up: a condition starting further out can remove more
    angle and still finish further from the surface. The residual panel carries
    that, and the matched positive-tilt runs are drawn as reference levels so
    the two signs are read against each other rather than in isolation.
    """
    selected = _main_rows(rows, ("MAIN_H1_", "MAIN_H2_"))
    if not selected:
        return None
    prior = _main_rows(rows, ("MAIN_F1_t1neg_rc_t2_p060",
                              "MAIN_F2_t2neg_rc_t1_m060"))
    positive = _main_rows(rows, ("MAIN_D1_t1_rc_t2_m060",
                                 "MAIN_D2_t2_rc_t1_p060"))
    series = (
        (1, "MAIN_F1_t1neg_rc_t2_p060", "MAIN_H1_", "MAIN_D1_t1_rc_t2_m060",
         SERIES_BLACK, "o"),
        (2, "MAIN_F2_t2neg_rc_t1_m060", "MAIN_H2_", "MAIN_D2_t2_rc_t1_p060",
         SERIES_RED, "s"),
    )
    fig, axes = plt.subplots(1, 2, figsize=(7.4, 3.3))
    for ax, key, ylabel in (
        (axes[0], "align_gain_deg", ALIGNMENT_IMPROVEMENT_LABEL),
        (axes[1], "align_after_deg", FINAL_MISALIGNMENT_LABEL),
    ):
        for axis, short_id, long_prefix, pos_id, colour, marker in series:
            buckets = {}
            for row in selected + prior:
                rid = row["run_id"]
                if rid != short_id and not rid.startswith(long_prefix):
                    continue
                x = fnum(row, "rc_t2_mm" if axis == 1 else "rc_t1_mm")
                y = fnum(row, key)
                if np.isnan(x) or np.isnan(y):
                    continue
                buckets.setdefault(abs(x), {"good": [], "bad": []})
                buckets[abs(x)]["bad" if data_suspect(row) else "good"].append(y)
            errorbar_from_buckets(
                ax, buckets, rf"$t_{axis}$, $-10^\circ$", colour, marker=marker)
            vals = [fnum(r, key) for r in positive
                    if r["run_id"] == pos_id and not data_suspect(r)]
            vals = [v for v in vals if not np.isnan(v)]
            if vals:
                ax.axhline(float(np.mean(vals)), color=colour, linewidth=1.0,
                           linestyle=":",
                           label=rf"$t_{axis}$, $+10^\circ$ at 60 mm")
        if key == "align_after_deg":
            ax.axhline(0.0, color="0.45", linewidth=1)
        sweep_axis(ax, (60, 80, 90))
        ax.set_xlabel(LEVER_MAGNITUDE_LABEL)
        ax.set_ylabel(ylabel)
    figure_legend(fig, axes, ncol=4)
    return save(fig, "MAIN_H_reversed_magnitude.pdf")


# Categorical slots 1 and 2 of the validated default palette. Two series only:
# the all-pairs floors hold for the first three slots, and marker shape carries
# identity as well as hue so the figure survives greyscale printing.
SERIES_B2 = SERIES_BLACK
SERIES_B3 = SERIES_RED
SERIES_B4 = SERIES_BLUE
INK = "#0b0b0b"
INK_MUTED = "#52514e"


PROVENANCE_FLAGS = {"dirty-tree"}


def data_suspect(row):
    """True if the run's numbers are untrustworthy, not merely its provenance."""
    flags = {f.split("(")[0] for f in row.get("flags", "").split(";") if f}
    return bool(flags - PROVENANCE_FLAGS)


def fig_main_tool_axis_tilt(rows):
    """Does the t1/t2 asymmetry belong to the plane or to the tool face?

    A2 and A4 tilt about the surface tangents; E1 tilts the same 10 deg about
    the tool's own axes, which the commanded twist puts 25 deg away from them.
    If the difference tracks the tool axes the asymmetry is the 40 x 120 mm
    face; if it tracks the surface axes it is the plane.
    """
    groups = [
        (r"About $t_1$", ("MAIN_A2_",), 1, "C0"),
        (r"About $t_2$", ("MAIN_A4_",), 2, "C1"),
        (r"About $Y_{EE}$ (120 mm edge)", ("MAIN_E1_tilt_about_y_long",), None, "C2"),
        (r"About $X_{EE}$ (40 mm edge)", ("MAIN_E1_tilt_about_x_short",), None, "C3"),
    ]
    labels, means, errs, colors, markers = [], [], [], [], []
    for label, prefixes, axis, color in groups:
        vals = []
        for row in rows:
            if not any(row["run_id"].startswith(p) for p in prefixes):
                continue
            if data_suspect(row):
                continue
            # A tool-axis tilt lands on both surface axes at once, so the
            # scalar gain is the only measure common to all four groups.
            v = fnum(row, "align_gain_deg")
            if not np.isnan(v):
                vals.append(v)
        if not vals:
            continue
        labels.append(label)
        means.append(float(np.mean(vals)))
        errs.append(float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0)
        colors.append(color)
        markers.append(("o", "s", "^", "D")[len(markers)])
    if not means:
        return None

    fig, ax = plt.subplots(figsize=(5.6, 3.4))
    xs = np.arange(len(means))
    for x, m, e, c, marker, lb in zip(
        xs, means, errs, colors, markers, labels
    ):
        ax.errorbar(
            x, m, yerr=e, capsize=3, color=c, marker=marker,
            markersize=6, markerfacecolor="white", markeredgecolor=c,
            markeredgewidth=1.1, elinewidth=1.0, capthick=1.0,
            linestyle="none", label=lb,
        )
    ax.set_xticks(xs)
    ax.set_xticklabels(["Surface\n$t_1$", "Surface\n$t_2$",
                        "Tool\n$Y_{EE}$", "Tool\n$X_{EE}$"][:len(means)])
    ax.set_xlabel(r"Signed commanded rotation axis $u_{\mathrm{rot}}$ [-]")
    ax.set_ylabel(ALIGNMENT_IMPROVEMENT_LABEL)
    ax.axhline(0.0, color="0.45", linewidth=1)
    axis_legend(ax, ncol=2, fontsize=7)
    return save(fig, "MAIN_E_tool_axis.pdf")


def fig_plane_validation(rows):
    """Matched horizontal-primary and tilted-validation baseline conditions."""
    definitions = (
        ("0 deg", "MAIN_A0_", "VALID_T0_", None),
        (r"$10^\circ$ about $t_1$", "MAIN_A2_", "VALID_T1_", 1),
        (r"$10^\circ$ about $t_2$", "MAIN_A4_", "VALID_T2_", 2),
    )
    selected = [
        row for row in rows
        if any(
            row["run_id"].startswith((horizontal, tilted))
            for _, horizontal, tilted, _ in definitions
        )
    ]
    if not any(row["run_id"].startswith("VALID_T") for row in selected):
        return None

    fig, axes = plt.subplots(1, 2, figsize=(7.8, 3.3), sharex=True)
    x_base = np.arange(len(definitions), dtype=float)
    for profile, prefix_index, offset, color, marker in (
        ("horizontal", 1, -0.08, "C0", "o"),
        ("tilted", 2, +0.08, "C1", "s"),
    ):
        initial_means, initial_errs = [], []
        residual_means, residual_errs = [], []
        for _, horizontal_prefix, tilted_prefix, axis in definitions:
            prefix = (horizontal_prefix, tilted_prefix)[prefix_index - 1]
            matching = [
                row for row in selected
                if row["run_id"].startswith(prefix) and not data_suspect(row)
            ]
            if axis is None:
                initial = [fnum(row, "align_before_deg") for row in matching]
                residual = [fnum(row, "align_after_deg") for row in matching]
            else:
                initial = [
                    abs(fnum(row, f"align_t{axis}_before_deg"))
                    for row in matching
                ]
                residual = [
                    abs(fnum(row, f"align_t{axis}_after_deg"))
                    for row in matching
                ]
            initial = [value for value in initial if not np.isnan(value)]
            residual = [value for value in residual if not np.isnan(value)]
            initial_means.append(np.mean(initial) if initial else np.nan)
            residual_means.append(np.mean(residual) if residual else np.nan)
            initial_errs.append(
                np.std(initial, ddof=1) if len(initial) > 1 else 0.0
            )
            residual_errs.append(
                np.std(residual, ddof=1) if len(residual) > 1 else 0.0
            )
        for ax, means, errors in (
            (axes[0], initial_means, initial_errs),
            (axes[1], residual_means, residual_errs),
        ):
            ax.errorbar(
                x_base + offset, means, yerr=errors, label=profile,
                color=color, marker=marker, linewidth=1.25, capsize=3,
                elinewidth=1.0, capthick=1.0, markersize=5.5,
                markerfacecolor="white", markeredgecolor=color,
                markeredgewidth=1.1,
            )

    axes[0].set_ylabel("measured initial error [deg]")
    axes[1].set_ylabel("residual after set-up [deg]")
    for ax in axes:
        ax.set_xticks(x_base)
        ax.set_xticklabels([definition[0] for definition in definitions])
    axis_legend(axes[0])
    fig.suptitle("Horizontal primary and tilted validation", fontsize=10)
    return save(fig, "PLANE_validation.pdf")


def _pole_points(rows, prefix, xkey):
    """(x, improvement) for every run of a series that commanded a pole."""
    xs, ys = [], []
    for r in rows:
        if not r["run_id"].startswith(prefix) or data_suspect(r):
            continue
        x, y = fnum(r, xkey), fnum(r, "align_improve_real_deg")
        if np.isnan(x) or np.isnan(y):
            continue
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)


def fig_b_pole_axis(rows):
    """Which component of the pole actually governs alignment.

    Plotted against the improvement toward the MEASURED plane, not the
    configured one -- see sgc_log.alignment_improvement_deg. The two panels are
    the same runs against the two pole components, which is why this is small
    multiples and not a second y-axis.
    """
    series = (("B2_pole_normal", "B2: swept along normal n", SERIES_B2, "o"),
              ("B3_pole_tangent_", "B3: swept along tangent $t_1$", SERIES_B3, "^"),
              ("B4_pole_tangent2", "B4: swept along tangent $t_2$", SERIES_B4, "s"))
    if not any(_pole_points(rows, p, "pole_cmd_x_mm")[0].size for p, *_ in series):
        return None

    fig, axes = plt.subplots(1, 3, figsize=(11.0, 3.4), sharey=True)
    for ax, xkey, xlabel in (
            (axes[0], "pole_cmd_x_mm", "pole offset along $t_1$ [mm]"),
            (axes[1], "pole_cmd_y_mm", "pole offset along $t_2$ [mm]"),
            (axes[2], "pole_cmd_z_mm", "pole offset along normal [mm]")):
        allx, ally = [], []
        for prefix, label, color, marker in series:
            x, y = _pole_points(rows, prefix, xkey)
            if not x.size:
                continue
            ax.plot(x, y, marker, color=color, ms=6, mew=1.0, mec=color,
                    mfc="white",
                    linestyle="none", label=label)
            allx.append(x)
            ally.append(y)
        if allx:
            X = np.concatenate(allx)
            Y = np.concatenate(ally)
            # The two in-plane axes turn over inside the tested range, so a
            # straight line through them is not just imprecise, it points the
            # wrong way past the optimum. The normal axis stays linear.
            deg = 1 if xkey == "pole_cmd_z_mm" else 2
            coef = np.polyfit(X, Y, deg)
            pred = np.polyval(coef, X)
            ss = 1.0 - ((Y - pred) ** 2).sum() / ((Y - Y.mean()) ** 2).sum()
            grid = np.linspace(X.min(), X.max(), 200)
            ax.plot(grid, np.polyval(coef, grid), "-", color=INK_MUTED,
                    linewidth=1.5, zorder=0)
            if deg == 2:
                peak = -coef[1] / (2.0 * coef[0])
                note = f"$R^2$ = {ss:.3f}\noptimum {peak:+.0f} mm"
            else:
                note = f"$R^2$ = {ss:.3f}\n{coef[0]:+.3f} deg/mm"
            ax.annotate(note, xy=(0.04, 0.94), xycoords="axes fraction",
                        va="top", fontsize=8, color=INK)
        ax.axhline(0.0, color="0.55", linewidth=1, zorder=0)
        ax.set_xlabel(xlabel, color=INK_MUTED)
    axes[0].set_ylabel("alignment gained toward the real plane [deg]",
                       color=INK_MUTED)
    axis_legend(axes[0], fontsize=8, loc="lower right", frameon=False)
    fig.suptitle("B: both in-plane pole components govern alignment; "
                 "the normal component does not", fontsize=10, color=INK)
    return save(fig, "B_pole_component.pdf")


def fig_b_pole_surface(rows):
    """The contribution in one plot: alignment as a surface over the pole plane.

    Diverging scale, because the quantity crosses zero and the sign is the
    point -- above zero the coupling drives the tool onto the surface, below it
    drives the tool away. Neutral gray at zero, blue/red poles.
    """
    pts = []
    for r in rows:
        if not r["run_id"].startswith(("B1_", "B2_", "B3_", "B4_")):
            continue
        if data_suspect(r):
            continue
        x, y = fnum(r, "pole_cmd_x_mm"), fnum(r, "pole_cmd_y_mm")
        v = fnum(r, "align_improve_real_deg")
        if np.isnan(x) or np.isnan(y) or np.isnan(v):
            continue
        pts.append((x, y, v))
    if len(pts) < 12:
        return None
    P = np.array(pts)
    X, Y, V = P[:, 0], P[:, 1], P[:, 2]

    A = np.column_stack([X, X ** 2, Y, Y ** 2, np.ones(len(P))])
    coef, res, *_ = np.linalg.lstsq(A, V, rcond=None)
    r2 = 1.0 - res[0] / ((V - V.mean()) ** 2).sum()
    x_opt = -coef[0] / (2.0 * coef[1])
    y_opt = -coef[2] / (2.0 * coef[3])

    gx = np.linspace(X.min() - 10, X.max() + 10, 240)
    gy = np.linspace(Y.min() - 10, Y.max() + 10, 240)
    GX, GY = np.meshgrid(gx, gy)
    GZ = (coef[0] * GX + coef[1] * GX ** 2
          + coef[2] * GY + coef[3] * GY ** 2 + coef[4])

    # The runs form a cross, not a grid: t1 was swept at t2 ~ 0 and t2 at
    # t1 ~ 15 mm. Drawing the fitted surface across the unsampled corners would
    # show confident contours over regions no run visited, and would let the
    # extrapolated minimum set the colour scale. Mask anything far from a
    # measured point, and scale the colours by the measured range.
    reach = 45.0
    d2 = np.min((GX[..., None] - X) ** 2 + (GY[..., None] - Y) ** 2, axis=-1)
    GZ = np.ma.masked_where(d2 > reach ** 2, GZ)

    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    lim = np.abs(V).max()
    cf = ax.contourf(GX, GY, GZ, levels=np.linspace(-lim, lim, 15),
                     cmap="RdBu_r", extend="both")
    ax.contour(GX, GY, GZ, levels=[0.0], colors="0.25", linewidths=1.2)
    ax.plot(X, Y, "o", ms=5, mfc="none", mec=INK, mew=0.9,
            linestyle="none", label="measured runs")
    ax.plot([x_opt], [y_opt], "*", ms=15, color=INK, linestyle="none",
            label=f"optimum ({x_opt:+.0f}, {y_opt:+.0f}) mm")
    cb = fig.colorbar(cf, ax=ax)
    cb.set_label("alignment gained toward the real plane [deg]",
                 color=INK_MUTED)
    ax.set_xlabel("pole offset along $t_1$ [mm]", color=INK_MUTED)
    ax.set_ylabel("pole offset along $t_2$ [mm]", color=INK_MUTED)
    ax.set_title(f"Fitted pole surface, $R^2$ = {r2:.3f} over {len(P)} runs; "
                 "shaded only where runs support it", fontsize=9, color=INK)
    axis_legend(ax, fontsize=7, loc="lower right", framealpha=0.9)
    ax.grid(False)
    return save(fig, "B_pole_surface.pdf")


# Case H directions, named in the tool frame and placed on a real axis: the
# angle each tilt axis makes with Y_EE. A fixed pole is optimal at 0 by
# construction, so the x axis is also "how far the tilt has turned away from
# the pole", which is what the fixed-pole series is measuring.
H_DIRECTION_DEG = {
    "yEE": 0.0,
    "diag_m45": -45.0,
    "xEE": -90.0,
    "diag_p45": 45.0,
}
H_NO_POLE = {
    "MAIN_E1_tilt_about_y_long": 0.0,
    "MAIN_E1_tilt_about_x_short": -90.0,
}


def _h_direction(run_id, prefix):
    name = run_id[len(prefix):]
    return H_DIRECTION_DEG.get(name, float("nan"))


def fig_main_general_pole(rows):
    """Is there one pole for every tilt direction, and which side of the plane?

    The press force is normal, so m = f x r_c leaves only the tangential lever
    turning the tool, perpendicular to itself: a tilt about u needs the lever
    rho (sin a, -cos a), which rotates with the tilt. The left panel puts that
    prediction against a pole held fixed. The right panel moves the same lever
    along the normal, where the rule says it makes no moment at all and can
    only add K_p,t r_n^2 of rotational stiffness against the correction.
    """
    selected = _main_rows(rows, ("MAIN_I",))
    if not selected:
        return None
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.3))

    # Left: the lever that follows the tilt against the one that does not.
    for prefix, label, color, marker in (
        ("MAIN_I1_rot_", "Direction-selected lever", SERIES_B2, "o"),
        ("MAIN_I2_fix_", "Fixed lever", SERIES_B3, "s"),
    ):
        buckets = {}
        for row in selected:
            if not row["run_id"].startswith(prefix):
                continue
            x = _h_direction(row["run_id"], prefix)
            y = fnum(row, "align_improve_real_deg")
            if np.isnan(x) or np.isnan(y):
                continue
            buckets.setdefault(x, {"good": [], "bad": []})
            buckets[x]["bad" if data_suspect(row) else "good"].append(y)
        # The fixed lever is the rotating one at 0 deg, so it is drawn there
        # too rather than leaving its own optimum off its curve.
        if prefix == "MAIN_I2_fix_" and buckets:
            shared = [
                fnum(row, "align_improve_real_deg") for row in selected
                if row["run_id"] == "MAIN_I1_rot_yEE" and not data_suspect(row)
            ]
            shared = [v for v in shared if not np.isnan(v)]
            if shared:
                buckets[0.0] = {"good": shared, "bad": []}
        errorbar_from_buckets(axes[0], buckets, label, color, marker=marker)

    # The matched no-pole runs: Case E commanded these same two tilts with the
    # decoupled spring, which is the floor any pole has to beat.
    no_pole = {}
    for row in _main_rows(rows, ("MAIN_E1_",)):
        x = H_NO_POLE.get(row["run_id"], float("nan"))
        y = fnum(row, "align_improve_real_deg")
        if np.isnan(x) or np.isnan(y):
            continue
        no_pole.setdefault(x, {"good": [], "bad": []})
        no_pole[x]["bad" if data_suspect(row) else "good"].append(y)
    errorbar_from_buckets(axes[0], no_pole, "No pole (Case E)", SERIES_BLUE,
                          marker="^")

    axes[0].axhline(0.0, color="0.45", linewidth=1)
    sweep_axis(axes[0], (-90, -45, 0, 45))
    axes[0].set_xlabel(
        r"Commanded rotation-axis angle $\alpha$ from $y_{\mathrm{EE}}$ "
        r"[$^\circ$]"
    )
    axes[0].set_ylabel(ALIGNMENT_IMPROVEMENT_LABEL)

    # Right: the same lever, moved along the normal.
    buckets = {}
    for row in selected:
        if not row["run_id"].startswith(("MAIN_I3_", "MAIN_I1_rot_yEE")):
            continue
        x = fnum(row, "rc_n_mm")
        y = fnum(row, "align_improve_real_deg")
        if np.isnan(x) or np.isnan(y):
            continue
        buckets.setdefault(x, {"good": [], "bad": []})
        buckets[x]["bad" if data_suspect(row) else "good"].append(y)
    errorbar_from_buckets(axes[1], buckets, "Lever along the normal",
                          SERIES_YELLOW, marker="D")
    # r_c = p_TCP - p_c, and the TCP stands about 20 mm off the plane at
    # contact: positive is under the plane, and +20 is in it.
    axes[1].axvline(20.0, color="0.45", linewidth=1, linestyle="--")
    axes[1].annotate("Pole in the plane", xy=(20.0, 0.02), xycoords=("data", "axes fraction"),
                     rotation=90, fontsize=7, color=INK_MUTED,
                     ha="right", va="bottom")
    axes[1].axhline(0.0, color="0.45", linewidth=1)
    sweep_axis(axes[1], (-60, 0, 20, 60, 120))
    axes[1].set_xlabel(
        r"Normal compliance-centre lever $r_{c,n}$ [mm]"
    )
    axes[1].set_ylabel(ALIGNMENT_IMPROVEMENT_LABEL)

    figure_legend(fig, axes, ncol=3)
    return save(fig, "MAIN_H_general_pole.pdf")


def fig_c2_nullspace(rows):
    """Null-space modes: sigma recovery and the task-invariance proof."""
    sub = [r for r in rows if r["run_id"].startswith("C2_hold_mode")]
    if not sub:
        return None

    order = {"0": 0, "1": 1, "2": 2, "3": 3}
    names = {0: "off", 1: "damping", 2: "sigma", 3: "both"}

    fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.2))
    for ax, key, ylabel in (
        (axes[0], "sigma_gain", r"recovered $\Delta\sigma_{\min}$"),
        (axes[1], "task_pos_error_drift_mm", "task position drift [mm]"),
    ):
        buckets = {}
        for r in sub:
            mode = r["run_id"].split("_")[2].replace("mode", "")
            if mode not in order:
                continue
            y = fnum(r, key)
            if np.isnan(y):
                continue
            x = order[mode]
            buckets.setdefault(x, {"good": [], "bad": []})
            buckets[x]["good" if not data_suspect(r) else "bad"].append(y)
        errorbar_from_buckets(ax, buckets, "measured", "C0")
        ax.set_xticks(sorted(buckets))
        ax.set_xticklabels([names[int(k)] for k in sorted(buckets)])
        ax.set_xlabel("null-space mode")
        ax.set_ylabel(ylabel)
    axes[1].axhline(0.0, color="0.3", linewidth=1)
    axes[1].set_title("must stay near zero", fontsize=8)
    figure_legend(fig, axes)
    fig.suptitle("C2/C3: null-space modes and task invariance", fontsize=10)
    return save(fig, "C2_nullspace_modes.pdf")


def fig_g2_convergence(rows):
    """Did the set-up phase actually reach equilibrium?"""
    sub = [r for r in rows if r["run_id"].startswith("G2_equilibrium")]
    if not sub:
        return None

    fig, ax = plt.subplots(figsize=(5.2, 3.4))
    buckets = {}
    for r in sub:
        x = fnum(r, "setup_duration_s")
        y = fnum(r, "tip_final_deg")
        if np.isnan(x) or np.isnan(y):
            continue
        buckets.setdefault(round(x), {"good": [], "bad": []})
        buckets[round(x)]["good" if not data_suspect(r) else "bad"].append(y)
    errorbar_from_buckets(ax, buckets, "final tip", "C0")
    ax.set_xlabel("set-up phase duration [s]")
    ax.set_ylabel("final tip angle [deg]")
    axis_legend(ax)
    ax.set_title("G2: is 4 s long enough to reach equilibrium?", fontsize=10)
    return save(fig, "G2_equilibrium.pdf")


def main():
    if not os.path.exists(METRICS):
        sys.exit(f"no metrics file: {METRICS}\n"
                 f"Run extract_metrics.py first.")
    rows = load_metrics(METRICS)
    print(f"{len(rows)} runs in metrics.csv")

    made = []
    for fn in (
        fig_g2_convergence,
        fig_a2_stiffness,
        fig_d_axis_stiffness,
        fig_d_initial_angle,
        fig_main_initial_angle,
        fig_main_rotational_stiffness,
        fig_main_translational_stiffness,
        fig_main_interaction,
        fig_main_compliance_centre,
        fig_main_sign_symmetry,
        fig_main_lever_magnitude,
        fig_main_reversed_magnitude,
        fig_main_tool_axis_tilt,
        fig_main_general_pole,
        fig_plane_validation,
        fig_b_pole_axis,
        fig_b_pole_surface,
        fig_c2_nullspace,
    ):
        try:
            p = fn(rows)
            if p:
                made.append(p)
            else:
                print(f"  (skipped {fn.__name__}: no data yet)")
        except Exception as exc:  # noqa: BLE001
            print(f"  ERROR in {fn.__name__}: {type(exc).__name__}: {exc}")

    print(f"\n{len(made)} figure(s) written to {FIGURES}")


if __name__ == "__main__":
    main()
