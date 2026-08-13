#!/usr/bin/env python3
"""Draw what one set-up press does: rotation, press force and both moments.

  python3 analysis/plot_setup_diagnostics.py [TRIAL=LABEL ...] [--out-dir DIR]

One column per trial, four rows sharing the same time axis, so the rotation can
be read against the load that produced it.

  rotation   the turn since first contact, resolved along the surface axes. It
             comes from joint angles alone, so no tool axis enters it.
  force      the press, as the change in the estimated external force since
             contact.
  M_TCP      the estimated external moment referenced to the TCP, less its
             value at contact. O_F_ext_hat_K reports the moment about the base
             origin, so the lever of the TCP position is removed first;
             leaving it in buries a 1 N m tool moment under 42 N m of reach.
  M_contact  the same moment carried to the contact point,
             M_contact = M_TCP + r_contact x df, with r_contact = p_TCP - p_contact.

The moments are resolved along the surface axes rather than the base axes, so a
moment about a tangent sits in the same row as the rotation about it.
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from extract_metrics import surface_frame, read_params  # noqa: E402

RESULTS = os.path.join(HERE, "..", "experiments", "results")

AXIS_STYLE = (
    (r"$t_1$", "#000000", "--"),
    (r"$t_2$", "#c00000", "-"),
    (r"$n$", "#0057b8", ":"),
)

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Latin Modern Roman", "CMU Serif", "cmr10", "DejaVu Serif"],
    "mathtext.fontset": "cm",
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.size": 9,
    "axes.edgecolor": "#1a1a1a",
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "axes.grid.axis": "y",
    "grid.alpha": 0.3,
    "grid.linewidth": 0.6,
    "lines.linewidth": 1.2,
    "legend.frameon": False,
    "legend.fontsize": 8,
})

SETUP_PHASE = 2  # ControlPhase::kSetup

DEFAULT_TRIALS = [
    ("S1_none_t1_10deg/r01", "no lever"),
    ("S5_normal_p090/r01", "90 mm along the tool axis"),
]


def vec(row, prefix):
    return [float(row[f"{prefix}_{a}"]) for a in "xyz"]


def load(trial):
    """Return time, rotation, press force, and both moments in surface axes."""
    directory = os.path.join(RESULTS, trial)
    logs = glob.glob(os.path.join(directory, "logs", "*.csv"))
    if not logs:
        raise SystemExit(f"no log csv under {trial}")
    params = read_params(os.path.join(directory, "params_effective"))
    frame = surface_frame(float(params["surface_tilt_x_deg"]),
                          float(params["surface_tilt_y_deg"]))

    time, rotation, force, moment_tcp, moment_contact = [], [], [], [], []
    with open(logs[0]) as f:
        for row in csv.DictReader(f):
            if float(row["phase"]) != SETUP_PHASE:
                continue
            time.append(float(row["time"]))
            rotation.append(vec(row, "e_R"))
            # Referencing both wrench parts to their value at first contact.
            df = np.array(vec(row, "external_force")) - vec(row, "contact_force_bias")
            p_ee = np.array(vec(row, "p_EE"))
            # Removing the base-origin lever, so the moment describes the tool.
            m_tcp = (np.array(vec(row, "external_moment"))
                     - vec(row, "contact_moment_bias") - np.cross(p_ee, df))
            r_contact = p_ee - np.array(vec(row, "tool_contact"))
            force.append(df)
            moment_tcp.append(m_tcp)
            moment_contact.append(m_tcp + np.cross(r_contact, df))

    t = np.array(time)
    return (t - t[0],
            np.degrees(np.array(rotation)) @ frame,
            np.linalg.norm(np.array(force), axis=1),
            np.array(moment_tcp) @ frame,
            np.array(moment_contact) @ frame)


def draw_axes(ax, t, values, ylabel, legend=False):
    for i, (label, colour, style) in enumerate(AXIS_STYLE):
        ax.plot(t, values[:, i], color=colour, linestyle=style, label=label)
    ax.axhline(0.0, color="#888888", linewidth=0.8, zorder=0)
    if ylabel:
        ax.set_ylabel(ylabel)
    if legend:
        ax.legend(loc="best", ncol=3)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trials", nargs="*", metavar="TRIAL=LABEL")
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    selected = ([tuple(a.split("=", 1)) for a in args.trials]
                if args.trials else DEFAULT_TRIALS)

    fig, axes = plt.subplots(4, len(selected), figsize=(6.9, 7.4),
                             sharex=True, squeeze=False)

    for column, (trial, label) in enumerate(selected):
        t, rotation, force, m_tcp, m_contact = load(trial)
        first = column == 0
        draw_axes(axes[0][column], t, rotation,
                  r"Rotation since contact [$^\circ$]" if first else "",
                  legend=first)
        axes[1][column].plot(t, force, color="#000000")
        if first:
            axes[1][column].set_ylabel("Press force [N]")
        draw_axes(axes[2][column], t, m_tcp,
                  r"$M_{\mathrm{TCP}}$ [N m]" if first else "")
        draw_axes(axes[3][column], t, m_contact,
                  r"$M_{\mathrm{contact}}$ [N m]" if first else "")
        axes[0][column].set_title(label, fontsize=9)
        axes[3][column].set_xlabel("Time from first contact [s]")

        print(f"{trial:24s} rotation t1 {rotation[-1, 0]:+6.2f} deg | "
              f"force {force[-1]:5.1f} N | "
              f"M_TCP t1 {m_tcp[-1, 0]:+6.2f} | "
              f"M_contact t1 {m_contact[-1, 0]:+6.2f} N m")

    fig.tight_layout()
    out = os.path.join(args.out_dir, "SETUP_diagnostics.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
