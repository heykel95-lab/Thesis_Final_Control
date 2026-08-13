#!/usr/bin/env python3
"""Plot one set-up trial as a time series against the thesis figure style.

  python3 analysis/plot_setup_trial.py <log.csv> [--out-dir DIR] [--axis t1|t2|n]

Writes two vector PDFs per log:

  <name>_setup_trial.pdf      three panels sharing the time axis -- the
                              misalignment angle, the press force, and the
                              moment about the tilt axis split into its two
                              contributions.
  <name>_setup_coupling.pdf   the regulated moment M_TCP alone, resolved
                              into all three surface axes.

The moment panel shows components rather than norms on purpose. M_contact is
the sum of M_TCP and the lever term, and that only reads as a sum along one
axis: norms do not add.
"""

import argparse
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sgc_log  # noqa: E402

# Text is matched to the thesis rather than to matplotlib's defaults, the same
# way make_figures.py does it. usetex stays off: it needs dvipng for the Agg
# backend, and it would tie the figure to a preamble kept somewhere else.
FONT_STYLE = "latex"

# The categorical palette begins with the agreed black, red, blue and yellow.
SERIES_BLACK = "#000000"
SERIES_RED = "#c00000"
SERIES_BLUE = "#0057b8"
SERIES_YELLOW = "#e0ad00"
SERIES_COLOURS = (SERIES_BLACK, SERIES_RED, SERIES_BLUE, SERIES_YELLOW)

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
    "legend.frameon": False,
    "legend.fontsize": 8,
    "legend.handlelength": 1.6,
    "legend.handletextpad": 0.5,
    "legend.labelspacing": 0.3,
    "legend.columnspacing": 1.2,
    "legend.borderaxespad": 0.4,
})
plt.rcParams.update(_FONT_STYLES[FONT_STYLE])

AXIS_NAMES = ("t1", "t2", "n")
AXIS_LABELS = {
    "t1": r"$t_1$",
    "t2": r"$t_2$",
    "n": r"$n_s$",
}

MISALIGNMENT_LABEL = r"Misalignment $\theta$ [$^\circ$]"
PRESS_FORCE_LABEL = r"Press force $f_n$ [N]"

COLUMNS = (
    "time", "phase",
    "p_EE_x", "p_EE_y", "p_EE_z",
    "tool_contact_x", "tool_contact_y", "tool_contact_z",
    "angular_deviation_deg",
    "external_force_x", "external_force_y", "external_force_z",
    "external_moment_x", "external_moment_y", "external_moment_z",
    "contact_force_bias_x", "contact_force_bias_y", "contact_force_bias_z",
    "contact_moment_bias_x", "contact_moment_bias_y", "contact_moment_bias_z",
)


def vec3(d, prefix):
    return np.column_stack((d[f"{prefix}_x"], d[f"{prefix}_y"], d[f"{prefix}_z"]))


def surface_frame(a_deg, b_deg):
    """Build [t1, t2, n] in base axes from the configured surface tilts."""
    normal = sgc_log.normal_from_tilt(a_deg, b_deg)
    # The controller projects the base-x hint into the plane; repeat that here.
    hint = np.array([1.0, 0.0, 0.0])
    t1 = hint - normal * np.dot(normal, hint)
    if np.linalg.norm(t1) <= 1e-9:
        t1 = np.array([0.0, 1.0, 0.0])
        t1 = t1 - normal * np.dot(normal, t1)
    t1 /= np.linalg.norm(t1)
    t2 = np.cross(normal, t1)
    t2 /= np.linalg.norm(t2)
    return np.column_stack((t1, t2, normal))


def setup_slice(d):
    """Restrict every array to the set-up phase, with time from its start."""
    mask = sgc_log.phase_mask(d["phase"], sgc_log.PHASE_SET_UP)
    if not mask.any():
        raise SystemExit("log contains no set-up phase")
    out = {k: v[mask] for k, v in d.items()}
    out["time"] = out["time"] - out["time"][0]
    return out


def moment_terms(d, R_bs):
    """Return press force and the three moments, resolved in surface axes."""
    force = vec3(d, "external_force") - vec3(d, "contact_force_bias")
    moment_tcp = vec3(d, "external_moment") - vec3(d, "contact_moment_bias")
    # r_contact points from the contact point back to the TCP, so the transfer
    # is M_contact = M_TCP + r_contact x f.
    r_contact = vec3(d, "p_EE") - vec3(d, "tool_contact")
    lever = np.cross(r_contact, force)
    moment_contact = moment_tcp + lever

    # Resolving into surface axes: rows are samples, so project with R_bs.
    to_surface = lambda v: v @ R_bs
    press = to_surface(force)[:, 2]
    return press, to_surface(moment_tcp), to_surface(lever), to_surface(moment_contact)


def tilt_axis_index(name):
    return AXIS_NAMES.index(name)


def plot_trial(d, press, m_tcp, m_lever, m_contact, axis, out_path):
    i = tilt_axis_index(axis)
    fig, axes = plt.subplots(3, 1, figsize=(5.4, 6.0), sharex=True)

    axes[0].plot(d["time"], d["angular_deviation_deg"], color=SERIES_BLACK)
    axes[0].set_ylabel(MISALIGNMENT_LABEL)

    axes[1].plot(d["time"], press, color=SERIES_BLACK)
    axes[1].set_ylabel(PRESS_FORCE_LABEL)

    axes[2].plot(d["time"], m_tcp[:, i], color=SERIES_BLACK,
                 label=r"$M_{\mathrm{TCP}}$")
    axes[2].plot(d["time"], m_lever[:, i], color=SERIES_RED,
                 label=r"$r_{\mathrm{contact}}\times f$")
    axes[2].plot(d["time"], m_contact[:, i], color=SERIES_BLUE,
                 label=r"$M_{\mathrm{contact}}$")
    axes[2].set_ylabel(
        f"Moment about {AXIS_LABELS[axis]} " r"[$\mathrm{N\,m}$]")
    axes[2].set_xlabel(r"Set-up time $t$ [s]")

    handles, labels = axes[2].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3,
               bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out_path)
    plt.close(fig)


def plot_coupling(d, m_tcp, out_path):
    """Resolve the regulated moment into surface axes.

    M_TCP is the moment the impedance works against, so it is the quantity
    that carries the commanded coupling from the compliance centre. Splitting
    it by axis shows whether a correction about the commanded axis leaks into
    the orthogonal one.
    """
    fig, ax = plt.subplots(figsize=(5.4, 2.8))
    for i, (name, colour) in enumerate(
            zip(AXIS_NAMES, (SERIES_BLACK, SERIES_RED, SERIES_BLUE))):
        ax.plot(d["time"], m_tcp[:, i], color=colour,
                label=AXIS_LABELS[name])
    ax.set_xlabel(r"Set-up time $t$ [s]")
    ax.set_ylabel(r"$M_{\mathrm{TCP}}$ [$\mathrm{N\,m}$]")

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3,
               bbox_to_anchor=(0.5, 0.0))
    fig.tight_layout(rect=(0, 0.12, 1, 1))
    fig.savefig(out_path)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("log", help="controller CSV log")
    p.add_argument("--out-dir", default=None,
                   help="output directory (default: alongside the log)")
    p.add_argument("--axis", default="t1", choices=AXIS_NAMES,
                   help="surface axis the tool was tilted about (default t1)")
    p.add_argument("--tilt-x", type=float, default=-1.585191335,
                   help="surface tilt about base x [deg]")
    p.add_argument("--tilt-y", type=float, default=0.988473281,
                   help="surface tilt about base y [deg]")
    args = p.parse_args()

    d, _ = sgc_log.read_csv(args.log, columns=COLUMNS)
    d = setup_slice(d)

    R_bs = surface_frame(args.tilt_x, args.tilt_y)
    press, m_tcp, m_lever, m_contact = moment_terms(d, R_bs)

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.log))
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.log))[0]

    trial_pdf = os.path.join(out_dir, f"{stem}_setup_trial.pdf")
    coupling_pdf = os.path.join(out_dir, f"{stem}_setup_coupling.pdf")
    plot_trial(d, press, m_tcp, m_lever, m_contact, args.axis, trial_pdf)
    plot_coupling(d, m_tcp, coupling_pdf)
    print(f"wrote {trial_pdf}")
    print(f"wrote {coupling_pdf}")


if __name__ == "__main__":
    main()
