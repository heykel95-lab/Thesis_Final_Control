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

The estimated moment is taken about the centre of compliance,

  M_CoC = r_eff x df_ext,   r_eff = p_contact - p_CoC,

which is the lever the contact force actually turns the tool through. It is
not the M_contact of the results chapter, which is referred to the contact
point instead. A run with no lever leaves the centre on the TCP, so the same
expression covers both and no case has to be separated.

The sign is kept rather than reduced to a magnitude. With r_eff running from
the centre to the contact, the moment that flattens the tool carries the same
sign as the rotation that flattens it: a commanded tilt of +10 degrees about
t1 is corrected by a positive rotation about t1 and shows a positive moment.

The normal force is negative while the tool presses. n_s points out of the
plate, so the press runs along -n_s, and the commanded and estimated curves
carry that sign alike.

Commanded and estimated share one colour per condition and are told apart by
the markers on the commanded curve. Neither is dashed, and neither is grey,
which the figure style reserves for reference lines.
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
        reader = csv.DictReader(f)
        has_r_eff = "r_eff_x" in (reader.fieldnames or [])
        if not has_r_eff and np.linalg.norm(configured_lever_ee(directory)) > 1e-9:
            raise SystemExit(
                f"{trial} was archived without r_eff and carries a shifted "
                "compliance centre, so the moment about that centre cannot be "
                "recovered. Rerun it with the current controller.")
        rows = [r for r in reader if float(r["phase"]) == SETUP_PHASE]

    time, tilt, fn_cmd, fn_ext, m_cmd, m_ext = [], [], [], [], [], []
    for row in rows:
        time.append(float(row["time"]))
        tilt.append(float(row["angular_deviation_deg"]))
        df = vec(row, "external_force") - vec(row, "contact_force_bias")
        fn_cmd.append(float(normal @ vec(row, "f")))
        fn_ext.append(float(normal @ df))
        m_cmd.append(float(tilt_axis @ vec(row, "m")))
        # With no r_eff column the centre sat on the TCP, checked above.
        r_eff = (vec(row, "r_eff") if has_r_eff
                 else vec(row, "tool_contact") - vec(row, "p_EE"))
        m_ext.append(float(tilt_axis @ np.cross(r_eff, df)))

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
    fig, axes = plt.subplots(3, 1, figsize=(5.8, 7.0), sharex=True)

    for (trial, label), colour in zip(selected, SERIES_COLOURS):
        t, tilt, fn_cmd, fn_ext, m_cmd, m_ext = thin(*load(trial, args.axis))
        marked = dict(color=colour, marker="o", markevery=90,
                      markerfacecolor="white", markeredgewidth=1.1,
                      markersize=4.5)
        axes[0].plot(t, tilt, color=colour, label=label)
        axes[1].plot(t, fn_ext, color=colour, label=f"estimated, {label}")
        axes[1].plot(t, fn_cmd, label=f"commanded, {label}", **marked)
        axes[2].plot(t, m_ext, color=colour, label=f"estimated, {label}")
        axes[2].plot(t, m_cmd, label=f"commanded, {label}", **marked)
        print(f"{trial:26s} e_tilt {tilt[0]:6.2f} -> {tilt[-1]:6.2f} deg | "
              f"Fn_ext {fn_ext[-1]:7.1f} N | M_ext {m_ext[-1]:+6.2f} N m")

    axis_name = AXIS_LABEL[args.axis]
    axes[0].set_ylabel(r"$e_{\mathrm{tilt}}$ [$^\circ$]")
    axes[1].set_ylabel(r"$F_n$ [N]")
    axes[2].set_ylabel(rf"$M_{{\mathrm{{align}}}}$ about {axis_name} [N m]")
    axes[2].set_xlabel("Time from first contact [s]")
    # Zero separates a flat tool from a tilted one, and a restoring moment from
    # a driving one. The force panel is left without a line.
    reference_line(axes[0])
    reference_line(axes[2])

    shared_legend(fig, [axes[0], axes[2]], ncol=2, bottom=0.12)
    out = os.path.join(args.out_dir, f"{args.out}.pdf")
    fig.savefig(out)
    fig.savefig(out.replace(".pdf", ".png"), dpi=160)
    plt.close(fig)
    print(f"wrote {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
