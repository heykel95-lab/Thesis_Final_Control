#!/usr/bin/env python3
"""Draw one centre-of-compliance case in the fixed three-panel order.

  python3 analysis/plot_coc_case.py TRIAL=LABEL [...] --axis t1 --out NAME

Every case is read the same way, top to bottom:

  1  alignment error   e_tilt, the angle between the tool normal and the
                       surface normal. Its zero is the flat tool, so a curve
                       can be read as how flat the tool ended.
  2  normal force      the press along n_s, both as the controller commanded
                       it and as the robot model estimated it from outside.
  3  alignment moment  the moment about the commanded tilt axis, again
                       commanded against estimated.

Both moments are referred to the TCP, which is the only point at which the
commanded and the estimated one can be compared without a contact model in
between. The commanded moment is the rotational half of the impedance wrench,
which the coupled law already forms at the TCP; the estimated one is the
external moment with the base-origin lever removed.

Referring them to the centre of compliance instead was tried and dropped. The
moment there is nearly constant, which is what the point shift is built to do,
so it says little about what distinguishes the conditions. Forming it as
r_eff x df_ext, with r_eff running from the centre to the contact point, is
worse than uninformative: it treats the pressed face as a point force at the
geometric extreme of the face and returns values the measured external moment
contradicts.

The sign is kept rather than reduced to a magnitude. With r_eff running from
the centre to the contact, the moment that flattens the tool carries the same
sign as the rotation that flattens it: a commanded tilt of +10 degrees about
t1 is corrected by a positive rotation about t1 and shows a positive moment.

The normal force is negative while the tool presses. n_s points out of the
plate, so the press runs along -n_s, and the commanded and estimated curves
carry that sign alike.

Commanded and estimated are drawn in panels of their own rather than over each
other, because they differ by an order of magnitude on the moment axis and the
smaller of the two is unreadable when they share a scale.

Each panel carries its own legend in its upper right corner. The deviation
panel is annotated with the attitude the tool arrived with, so the figure can
be read without the commanded offset, which the tool does not reach exactly.
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
from extract_metrics import surface_frame, read_params  # noqa: E402
from figure_style import (apply_style, reference_line, shared_legend,  # noqa: E402
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


def configured_lever_ee(directory):
    """Return the configured compliance-centre offset in EE coordinates [m]."""
    params = read_params(os.path.join(directory, "params_effective"))
    if params.get("use_virtual_compliance_center", "0").strip() in ("0", ""):
        return np.zeros(3)
    return np.array([
        float(params.get(f"compliance_center_offset_ee_{a}", 0.0))
        for a in "xyz"])


def load(trial, axis):
    """Return time, tilt error, both normal forces and both alignment moments."""
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

    time, tilt, fn_cmd, fn_ext, m_cmd, m_ext = [], [], [], [], [], []
    for row in rows:
        time.append(float(row["time"]))
        tilt.append(float(row["angular_deviation_deg"]))
        df = vec(row, "external_force") - vec(row, "contact_force_bias")
        fn_cmd.append(float(normal @ vec(row, "f")))
        fn_ext.append(float(normal @ df))
        m_cmd.append(float(tilt_axis @ vec(row, "m")))
        # Removing the base-origin lever, so the estimate describes the tool.
        p_ee = vec(row, "p_EE")
        m_tcp = (vec(row, "external_moment") - vec(row, "contact_moment_bias")
                 - np.cross(p_ee, df))
        m_ext.append(float(tilt_axis @ m_tcp))

    t = np.array(time)
    return (t - t[0], np.array(tilt), np.array(fn_cmd), np.array(fn_ext),
            np.array(m_cmd), np.array(m_ext))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trials", nargs="+", metavar="TRIAL=LABEL")
    p.add_argument("--axis", default="t1", choices=sorted(AXIS_COLUMN))
    p.add_argument("--out", default="COC_case")
    p.add_argument("--out-dir", default=os.path.join(HERE, "..", "figures"))
    args = p.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    selected = [tuple(a.split("=", 1)) for a in args.trials]
    fig, axes = plt.subplots(5, 1, figsize=(5.8, 9.6), sharex=True)

    for index, ((trial, label), colour) in enumerate(zip(selected,
                                                         SERIES_COLOURS)):
        t, tilt, fn_cmd, fn_ext, m_cmd, m_ext = thin(*load(trial, args.axis))
        for ax, series in zip(axes, (tilt, fn_cmd, fn_ext, m_cmd, m_ext)):
            ax.plot(t, series, color=colour, label=label)
        # Naming the attitude the tool arrived with, at the left edge of the
        # deviation panel. The conditions start within a tenth of a degree of
        # each other, so the labels are stacked rather than left to overprint.
        axes[0].annotate(f"{tilt[0]:.2f}", xy=(t[0], tilt[0]),
                         xytext=(4, -11 * index - 4),
                         textcoords="offset points",
                         color=colour, fontsize=8)
        print(f"{trial:26s} e_tilt {tilt[0]:6.2f} -> {tilt[-1]:6.2f} deg | "
              f"Fn_ext {fn_ext[-1]:7.1f} N | M_ext {m_ext[-1]:+6.2f} N m")

    sub = AXIS_SUBSCRIPT[args.axis]
    labels = [r"$e_{\mathrm{tilt}}$ [$^\circ$]",
              r"$F_{n,\mathrm{cmd}}$ [N]",
              r"$F_{n,\mathrm{ext}}$ [N]",
              rf"$M_{{{sub},\mathrm{{cmd}}}}$ [N m]",
              rf"$M_{{{sub},\mathrm{{ext}}}}$ [N m]"]
    for ax, text in zip(axes, labels):
        ax.set_ylabel(text)
        # Zero separates a flat tool from a tilted one, and a restoring moment
        # from a driving one. The press panels are left without a line.
        if "F_" not in text:
            reference_line(ax)
        # Headroom so the corner legend sits above the data rather than on it.
        ax.margins(y=0.28)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Time from first contact [s]")
    fig.tight_layout()
    out = os.path.join(args.out_dir, f"{args.out}.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
