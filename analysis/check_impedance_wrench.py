#!/usr/bin/env python3
"""Static check of the impedance law against the estimated external wrench.

Run mode t (Cartesian pose hold with the setup impedance), displace the tool
by hand, hold it still, and let go. This script reads the resulting log and
answers two questions per hold:

  1. Does the commanded wrench match K times the measured displacement?
     (a self-consistency check of the law as implemented)
  2. Does the model-estimated external wrench mirror the commanded one?
     (the physical check: the robot really delivers what it commands)

Everything is reported in surface axes [tangent1, tangent2, normal], so
F_n and M_t1 are read directly.

Two conventions matter and are applied here:

  * O_F_ext_hat_K reports the moment about the BASE origin, so the TCP moment
    is  M_tcp = dM_ext - p_EE x dF_ext.  Skipping this leaves tens of N m of
    lever term on top of the ~1 N m that is actually of interest. The same
    correction is made in control_loop.cpp when set-up evaluates the contact.
  * The estimated external wrench is what the environment applies to the
    robot, so in equilibrium it opposes the commanded wrench:  W_ext = -W_cmd.
    The report prints both residuals so the sign is confirmed, not assumed.

Depends only on numpy and matplotlib, like the rest of this directory.
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sgc_log  # noqa: E402

PHASE_HOLD = 4

AXES = ("t1", "t2", "n")

COLUMNS = (
    "time", "phase",
    "p_EE_x", "p_EE_y", "p_EE_z",
    "e_p_x", "e_p_y", "e_p_z",
    "e_R_x", "e_R_y", "e_R_z",
    "pdot_x", "pdot_y", "pdot_z",
    "omega_x", "omega_y", "omega_z",
    "f_x", "f_y", "f_z",
    "m_x", "m_y", "m_z",
    "external_force_x", "external_force_y", "external_force_z",
    "external_moment_x", "external_moment_y", "external_moment_z",
)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def read_conf(directory):
    """Read every params/*.conf into one {key: float} table."""
    values = {}
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".conf"):
            continue
        with open(os.path.join(directory, name)) as handle:
            for line in handle:
                line = line.split("#", 1)[0].strip()
                if "=" not in line:
                    continue
                key, _, value = line.partition("=")
                try:
                    values[key.strip()] = float(value.strip())
                except ValueError:
                    pass
    return values


def surface_frame(conf):
    """R_base_surface = [t1 t2 n], matching makeSurfaceFrameFromNormalTangent."""
    normal = sgc_log.normal_from_tilt(conf["surface_tilt_x_deg"],
                                      conf["surface_tilt_y_deg"])
    normal = normal / np.linalg.norm(normal)
    hint = np.array([conf["surface_tangent1_hint_base_x"],
                     conf["surface_tangent1_hint_base_y"],
                     conf["surface_tangent1_hint_base_z"]])
    tangent1 = hint - normal * (normal @ hint)
    if np.linalg.norm(tangent1) <= 1e-9:
        fallback = np.array([0.0, 1.0, 0.0])
        if abs(normal @ fallback) > 0.95:
            fallback = np.array([0.0, 0.0, 1.0])
        tangent1 = fallback - normal * (normal @ fallback)
    tangent1 /= np.linalg.norm(tangent1)
    tangent2 = np.cross(normal, tangent1)
    tangent2 /= np.linalg.norm(tangent2)
    return np.column_stack((tangent1, tangent2, normal))


def configured_gains(conf, which="setup"):
    """Stiffness diagonals of the selected phase, in surface axes [t1, t2, n].

    Mode t holds with the set-up gains; mode h holds with hold.conf, whose
    gains are defined in base axes and are only diagonal in surface axes
    because they happen to be isotropic.
    """
    if which == "hold":
        Kp = [conf["hold_Kp_x"], conf["hold_Kp_y"], conf["hold_Kp_z"]]
        KR = [conf["hold_KR_x"], conf["hold_KR_y"], conf["hold_KR_z"]]
        return np.array(Kp), np.array(KR), np.zeros(3)
    if conf.get("setup_translation_surface_frame", 1.0) >= 0.5:
        Kp = [conf["setup_Kp_surface_tangent1"],
              conf["setup_Kp_surface_tangent2"],
              conf["setup_Kp_surface_normal"]]
    else:
        # Base-frame gains are not diagonal in surface axes; reported for
        # information only, and the fitted slope is then the thing to read.
        Kp = [conf["setup_Kp_x"], conf["setup_Kp_y"], conf["setup_Kp_z"]]
    KR = [conf["setup_KR_tangent1"],
          conf["setup_KR_tangent2"],
          conf["setup_KR_normal"]]
    lever = [conf.get("compliance_lever_surface_tangent1", 0.0),
             conf.get("compliance_lever_surface_tangent2", 0.0),
             conf.get("compliance_lever_surface_normal", 0.0)]
    return np.array(Kp), np.array(KR), np.array(lever)


# ---------------------------------------------------------------------------
# Log processing
# ---------------------------------------------------------------------------

def stack(d, prefix, keys=("x", "y", "z")):
    return np.column_stack([d[f"{prefix}_{k}"] for k in keys])


def to_surface(R, vectors):
    """Express base-frame row vectors in surface axes."""
    return vectors @ R


def segments(mask, time, min_duration, level=None, tolerance=None):
    """Contiguous True runs of at least min_duration seconds.

    A hand never stops completely between two displacements, so a run is also
    cut wherever `level` drifts further than `tolerance` from the mean of the
    run so far. Without that a slow drift from 2 mm to 10 mm reads as one hold.
    """
    out = []
    start = None

    def close(first, last):
        if last - first >= 1 and time[last] - time[first] >= min_duration:
            out.append((first, last))

    for i, flag in enumerate(mask):
        if not flag:
            if start is not None:
                close(start, i - 1)
                start = None
            continue
        if start is None:
            start = i
            continue
        if level is not None and tolerance is not None:
            reference = level[start:i].mean()
            if abs(level[i] - reference) > tolerance:
                close(start, i - 1)
                start = i
    if start is not None:
        close(start, len(mask) - 1)
    return out


def fit_slope(x, y, min_range):
    """Least-squares slope of y = a x + b over a sufficiently excited x."""
    if x.size < 2 or float(np.ptp(x)) < min_range:
        return float("nan")
    a, _ = np.polyfit(x, y, 1)
    return float(a)


def fmt(value, decimals):
    """Print a slope, or say the axis was not displaced enough to fit one."""
    if not math.isfinite(value):
        return f"{'not excited':>8}"
    return f"{value:8.{decimals}f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="surface_grinding_controller_log.csv from a mode-t run")
    parser.add_argument("--params", default=None,
                        help="params directory of the run (default: ../surface_grinding_controller/params)")
    parser.add_argument("--bias", nargs=2, type=float, metavar=("T0", "T1"),
                        default=(0.5, 2.5),
                        help="hands-off window [s] averaged as the external-wrench bias")
    parser.add_argument("--speed-limit", type=float, default=0.002,
                        help="quasi-static translation limit [m/s]")
    parser.add_argument("--rate-limit", type=float, default=2.0,
                        help="quasi-static rotation limit [deg/s]")
    parser.add_argument("--min-hold", type=float, default=0.5,
                        help="shortest accepted hold [s]")
    parser.add_argument("--hold-tolerance", type=float, default=0.0005,
                        help="displacement drift that ends one hold and starts the next [m]")
    parser.add_argument("--min-displacement", type=float, default=0.0005,
                        help="displacement below which a hold is only the rest pose [m]")
    parser.add_argument("--gains", choices=("setup", "hold"), default="setup",
                        help="which configured gains to compare against: "
                             "setup for mode t, hold for mode h")
    parser.add_argument("--plot", default=None, help="write a PDF here")
    args = parser.parse_args()

    params_dir = args.params or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), os.pardir,
        "surface_grinding_controller", "params")
    conf = read_conf(params_dir)
    R = surface_frame(conf)
    Kp, KR, lever = configured_gains(conf, args.gains)

    d, _ = sgc_log.read_csv(args.log, COLUMNS)
    hold = d["phase"] == PHASE_HOLD
    if not hold.any():
        raise SystemExit("no pose-hold samples in this log")
    d = {k: v[hold] for k, v in d.items()}

    time = d["time"] - d["time"][0]
    p_EE = stack(d, "p_EE")
    f_ext = stack(d, "external_force")
    m_ext = stack(d, "external_moment")

    # Bias: the estimator carries a payload-model offset, so a hands-off
    # sample is subtracted before anything is compared.
    window = (time >= args.bias[0]) & (time <= args.bias[1])
    if window.sum() < 10:
        raise SystemExit(f"bias window {args.bias} holds too few samples")
    f_bias = f_ext[window].mean(axis=0)
    m_bias = m_ext[window].mean(axis=0)

    df_ext = f_ext - f_bias
    # Referencing the estimated moment to the TCP before comparing it with the
    # commanded moment, which acts at the TCP.
    dm_ext_tcp = (m_ext - m_bias) - np.cross(p_EE, df_ext)

    # Surface-axis quantities.
    e_p_s = to_surface(R, stack(d, "e_p"))
    e_R_s = to_surface(R, stack(d, "e_R"))
    f_cmd_s = to_surface(R, stack(d, "f"))
    m_cmd_s = to_surface(R, stack(d, "m"))
    f_ext_s = to_surface(R, df_ext)
    m_ext_s = to_surface(R, dm_ext_tcp)

    speed = np.linalg.norm(stack(d, "pdot"), axis=1)
    rate = np.degrees(np.linalg.norm(stack(d, "omega"), axis=1))
    still = (speed <= args.speed_limit) & (rate <= args.rate_limit)

    print(f"log            {args.log}")
    print(f"params         {os.path.normpath(params_dir)}")
    print(f"hold samples   {time.size}  ({time[-1]:.1f} s)")
    print(f"surface axes   t1 {np.array2string(R[:, 0], precision=4)}")
    print(f"               t2 {np.array2string(R[:, 1], precision=4)}")
    print(f"               n  {np.array2string(R[:, 2], precision=4)}")
    print(f"gains          {args.gains}")
    print(f"Kp [t1,t2,n]   {Kp} N/m")
    print(f"KR [t1,t2,n]   {KR} N m/rad")
    print(f"r_c            {1000.0 * lever} mm in surface axes"
          + ("" if np.allclose(lever, 0.0)
             else "   <-- non-zero: force and moment are coupled, see note below"))
    print(f"bias window    {args.bias[0]:.1f}-{args.bias[1]:.1f} s   "
          f"F {np.array2string(f_bias, precision=2)} N   "
          f"M {np.array2string(m_bias, precision=2)} N m")
    print(f"quasi-static   {still.sum()} of {still.size} samples "
          f"(<= {1000.0 * args.speed_limit:.1f} mm/s, <= {args.rate_limit:.1f} deg/s)")

    holds = [(a, b) for (a, b) in segments(still, time, args.min_hold,
                                          level=np.linalg.norm(e_p_s, axis=1),
                                          tolerance=args.hold_tolerance)
             if (np.linalg.norm(e_p_s[a:b + 1], axis=1).mean() >= args.min_displacement
                 or np.degrees(np.linalg.norm(e_R_s[a:b + 1], axis=1)).mean() >= 0.5)]

    print()
    print("Each row averages one quasi-static hold. dF and dM are the")
    print("commanded value plus the estimated external value; they should be")
    print("near zero if the external estimate opposes the command.")
    print()
    header = (f"{'t [s]':>12}  {'dur':>5}  {'e_n [mm]':>9}  {'Fn cmd':>8}  "
              f"{'Fn ext':>8}  {'sum':>7}  {'th_t1 [deg]':>11}  {'Mt1 cmd':>8}  "
              f"{'Mt1 ext':>8}  {'sum':>7}")
    print(header)
    print("-" * len(header))
    for a, b in holds:
        sl = slice(a, b + 1)
        e_n = 1000.0 * e_p_s[sl, 2].mean()
        th_t1 = math.degrees(e_R_s[sl, 0].mean())
        fn_c = f_cmd_s[sl, 2].mean()
        fn_e = f_ext_s[sl, 2].mean()
        mt_c = m_cmd_s[sl, 0].mean()
        mt_e = m_ext_s[sl, 0].mean()
        print(f"{time[a]:12.2f}  {time[b] - time[a]:5.2f}  {e_n:9.3f}  "
              f"{fn_c:8.2f}  {fn_e:8.2f}  {fn_c + fn_e:7.2f}  "
              f"{th_t1:11.3f}  {mt_c:8.3f}  {mt_e:8.3f}  {mt_c + mt_e:7.3f}")
    if not holds:
        print("  (no hold long enough; lower --min-hold or --min-displacement)")

    # Stiffness identified over every quasi-static sample.
    # Stiffness identified over every quasi-static sample. An axis that was
    # not displaced is reported as unexcited rather than fitted to noise.
    print()
    for i, axis in enumerate(AXES):
        k_cmd = fit_slope(e_p_s[still, i], f_cmd_s[still, i], 0.0002)
        k_ext = fit_slope(e_p_s[still, i], -f_ext_s[still, i], 0.0002)
        print(f"K_{axis:<2} configured {Kp[i]:8.1f}   from command {fmt(k_cmd, 1)}   "
              f"from external estimate {fmt(k_ext, 1)}  N/m")
    for i, axis in enumerate(AXES):
        k_cmd = fit_slope(e_R_s[still, i], m_cmd_s[still, i], math.radians(0.2))
        k_ext = fit_slope(e_R_s[still, i], -m_ext_s[still, i], math.radians(0.2))
        print(f"KR_{axis:<2} configured {KR[i]:8.2f}   from command {fmt(k_cmd, 2)}   "
              f"from external estimate {fmt(k_ext, 2)}  N m/rad")

    if args.plot:
        write_plot(args.plot, time, e_p_s, e_R_s, f_cmd_s, f_ext_s,
                   m_cmd_s, m_ext_s, still, Kp, KR)
        print(f"\nwrote {args.plot}")


def write_plot(path, time, e_p_s, e_R_s, f_cmd_s, f_ext_s, m_cmd_s, m_ext_s,
               still, Kp, KR):
    import figure_style
    figure_style.apply_style()
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(7.0, 5.0))

    axes[0, 0].plot(time, f_cmd_s[:, 2], color=figure_style.SERIES_BLACK,
                    label="commanded")
    axes[0, 0].plot(time, -f_ext_s[:, 2], color=figure_style.SERIES_RED,
                    label="negated external estimate")
    axes[0, 0].set_xlabel("time [s]")
    axes[0, 0].set_ylabel(r"$F_n$ [N]")

    axes[0, 1].plot(time, m_cmd_s[:, 0], color=figure_style.SERIES_BLACK)
    axes[0, 1].plot(time, -m_ext_s[:, 0], color=figure_style.SERIES_RED)
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel(r"$M_{t1}$ [N m]")

    x = 1000.0 * e_p_s[still, 2]
    axes[1, 0].plot(x, f_cmd_s[still, 2], ".", ms=2,
                    color=figure_style.SERIES_BLACK)
    axes[1, 0].plot(x, -f_ext_s[still, 2], ".", ms=2,
                    color=figure_style.SERIES_RED)
    span = np.array([x.min(), x.max()]) if x.size else np.array([0.0, 1.0])
    axes[1, 0].plot(span, Kp[2] * span / 1000.0,
                    color=figure_style.REFERENCE_GREY)
    axes[1, 0].set_xlabel(r"$e_n$ [mm]")
    axes[1, 0].set_ylabel(r"$F_n$ [N]")

    x = np.degrees(e_R_s[still, 0])
    axes[1, 1].plot(x, m_cmd_s[still, 0], ".", ms=2,
                    color=figure_style.SERIES_BLACK)
    axes[1, 1].plot(x, -m_ext_s[still, 0], ".", ms=2,
                    color=figure_style.SERIES_RED)
    span = np.array([x.min(), x.max()]) if x.size else np.array([0.0, 1.0])
    axes[1, 1].plot(span, KR[0] * np.radians(span),
                    color=figure_style.REFERENCE_GREY)
    axes[1, 1].set_xlabel(r"$\theta_{t1}$ [deg]")
    axes[1, 1].set_ylabel(r"$M_{t1}$ [N m]")

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, frameon=False)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)


if __name__ == "__main__":
    main()
