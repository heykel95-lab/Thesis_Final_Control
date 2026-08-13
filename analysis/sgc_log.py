#!/usr/bin/env python3
"""Loader and metric extraction for surface_grinding_controller logs.

Deliberately depends only on numpy: pandas and scipy are not installed on the
robot PC and pip is unavailable, so anything heavier could not be run where the
data actually lives.

Handles old logs without alignment data, scalar-alignment logs, and the current
schema with signed surface-frame alignment components.
"""

import math
import os

import numpy as np

# ControlPhase enum order in controller.h.
PHASE_APPROACH_ORIENT = 0
PHASE_APPROACH_DESCEND = 1
PHASE_SET_UP = 2
PHASE_GRIND = 3
PHASE_HOLD = 4
PHASE_MANUAL_GUIDE = 5

PHASE_NAMES = {
    0: "approach_orient",
    1: "approach_descend",
    2: "set_up",
    3: "grind",
    4: "hold",
    5: "manual_guide",
}

# ---------------------------------------------------------------------------
# Legacy surface geometry. The old A/B campaign scored alignment against a
# deliberately offset configured normal, so it must be rescored here against
# the separately measured plane. The calibrated D series logs the physical
# plane metric directly and does not use this legacy correction.
# ---------------------------------------------------------------------------

# alignment_target_tilt_angle_deg / _y_deg, i.e. (a about base x, b about y).
CONFIGURED_PLANE = (0.0, 5.0)

# tools/measure_plane readings, three hand-seatings on 2026-07-28. The spread
# is partly seating technique and partly the workpiece moving between
# attempts. The absolute residual depends on which of these you pick; the
# improvement does not, moving under 0.1 deg across all three.
MEASURED_PLANE = [(-1.01, 12.98), (-0.40, 16.79), (-0.76, 15.19)]


def measured_plane_mean():
    return (sum(p[0] for p in MEASURED_PLANE) / len(MEASURED_PLANE),
            sum(p[1] for p in MEASURED_PLANE) / len(MEASURED_PLANE))


def normal_from_tilt(a_deg, b_deg):
    """n = R_y(b) * R_x(a) * [0,0,1], matching config.cpp."""
    a, b = math.radians(a_deg), math.radians(b_deg)
    return np.array([math.sin(b) * math.cos(a),
                     -math.sin(a),
                     math.cos(b) * math.cos(a)])


def rotate_vector(v, axis, angle):
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return v
    k = axis / n
    return (v * math.cos(angle)
            + np.cross(k, v) * math.sin(angle)
            + k * (k @ v) * (1.0 - math.cos(angle)))


def angle_between_deg(u, v):
    return math.degrees(math.acos(float(np.clip(u @ v, -1.0, 1.0))))


def alignment_improvement_deg(csv_path, plane=None):
    """How much closer to the REAL plane the tool got during set_up [deg].

    Positive means it rotated toward the surface. Returns None if the run has
    no set-up phase.

    The tool's rotation is -e_R, not +e_R: orientationError(R_current,
    R_desired) returns the rotation taking current -> desired, and during
    set_up the desired is the orientation frozen at contact, so the logged e_R
    points back to the start. Using +e_R inverts the conclusion.

    The starting tool axis is taken to be the configured normal. The runs put
    align_before at 0.41-0.49 deg, so that is good to about half a degree.
    """
    a_m, b_m = plane if plane is not None else measured_plane_mean()
    n_cfg = normal_from_tilt(*CONFIGURED_PLANE)
    n_real = normal_from_tilt(a_m, b_m)

    d, _ = read_csv(csv_path, ["phase", "e_R_x", "e_R_y", "e_R_z"])
    idx = np.where(phase_mask(d["phase"], PHASE_SET_UP))[0]
    if idx.size == 0:
        return None
    i = idx[-1]
    e_R = np.array([d["e_R_x"][i], d["e_R_y"][i], d["e_R_z"][i]])
    final = rotate_vector(n_cfg.copy(), -e_R, float(np.linalg.norm(e_R)))
    return angle_between_deg(n_cfg, n_real) - angle_between_deg(final, n_real)


def read_csv(path, columns=None):
    """Read a controller CSV into {name: 1-D array}.

    columns=None loads everything. Passing the subset you need is much faster
    on the 120k-row general log.
    """
    with open(path) as f:
        header = f.readline().strip().split(",")
    header = [h.strip() for h in header]

    if columns is None:
        use = list(range(len(header)))
        names = header
    else:
        missing = [c for c in columns if c not in header]
        if missing:
            raise KeyError(f"{os.path.basename(path)} lacks columns {missing}")
        use = [header.index(c) for c in columns]
        names = list(columns)

    data = np.loadtxt(path, delimiter=",", skiprows=1, usecols=use, ndmin=2)
    return {name: data[:, i] for i, name in enumerate(names)}, header


def has_alignment_metric(header):
    return "alignment_angle_deg" in header


def has_alignment_components(header):
    return all(
        name in header
        for name in (
            "alignment_error_t1_deg",
            "alignment_error_t2_deg",
            "alignment_error_normal_deg",
        )
    )


def phase_mask(phase, wanted):
    return phase == wanted


def _norm3(d, prefix):
    return np.sqrt(d[f"{prefix}_x"] ** 2
                   + d[f"{prefix}_y"] ** 2
                   + d[f"{prefix}_z"] ** 2)


def setup_metrics(path):
    """Extract the per-run set-up metrics used by the thesis tables.

    Returns a dict of scalars. Values that cannot be computed from the
    available schema are NaN, never invented.
    """
    wanted = [
        "time", "phase",
        "e_R_x", "e_R_y", "e_R_z",
        "p_EE_x", "p_EE_y", "p_EE_z",
        "tool_contact_x", "tool_contact_y", "tool_contact_z",
        "first_contact_x", "first_contact_y", "first_contact_z",
        "external_force_x", "external_force_y", "external_force_z",
        "contact_force_bias_x", "contact_force_bias_y", "contact_force_bias_z",
        "tau_cmd_1", "tau_cmd_2", "tau_cmd_3", "tau_cmd_4",
        "tau_cmd_5", "tau_cmd_6", "tau_cmd_7",
    ]
    with open(path) as f:
        header = [h.strip() for h in f.readline().strip().split(",")]
    if has_alignment_metric(header):
        wanted.append("alignment_angle_deg")
    if has_alignment_components(header):
        wanted.extend(
            [
                "alignment_error_t1_deg",
                "alignment_error_t2_deg",
                "alignment_error_normal_deg",
            ]
        )

    d, header = read_csv(path, wanted)

    out = {
        "n_rows": len(d["time"]),
        "duration_s": float(d["time"][-1] - d["time"][0]) if len(d["time"]) else np.nan,
        "has_alignment_metric": has_alignment_metric(header),
        "has_alignment_components": has_alignment_components(header),
    }

    setup = phase_mask(d["phase"], PHASE_SET_UP)
    out["setup_samples"] = int(setup.sum())
    if out["setup_samples"] < 2:
        out["setup_present"] = False
        return out
    out["setup_present"] = True

    t = d["time"][setup]
    out["setup_duration_s"] = float(t[-1] - t[0])

    # Angular deviation: rotation away from the orientation frozen at the
    # clearance transition. This is how far the tool TURNED, with no reference
    # to the plane, so no calibration enters it.
    e_r = np.sqrt(d["e_R_x"][setup] ** 2
                  + d["e_R_y"][setup] ** 2
                  + d["e_R_z"][setup] ** 2)
    out["angular_deviation_final_deg"] = float(np.degrees(e_r[-1]))
    out["angular_deviation_max_deg"] = float(np.degrees(e_r.max()))

    # Alignment: residual angle to the configured surface. This is how FLAT it
    # ended up, which is the quantity the thesis calls e_R before/after.
    if out["has_alignment_metric"]:
        a = d["alignment_angle_deg"][setup]
        out["align_before_deg"] = float(a[0])
        out["align_after_deg"] = float(a[-1])
        out["align_gain_deg"] = float(a[0] - a[-1])
        total_change = float(a[0] - a[-1])
        if total_change > 0.0:
            target = float(a[0] - 0.9 * total_change)
            reached = np.where(a <= target)[0]
            out["alignment_time90_s"] = (
                float(t[reached[0]] - t[0]) if reached.size else np.nan
            )
        else:
            out["alignment_time90_s"] = np.nan
    else:
        out["align_before_deg"] = np.nan
        out["align_after_deg"] = np.nan
        out["align_gain_deg"] = np.nan
        out["alignment_time90_s"] = np.nan

    if out["has_alignment_components"]:
        for axis in ("t1", "t2"):
            values = d[f"alignment_error_{axis}_deg"][setup]
            out[f"align_{axis}_before_deg"] = float(values[0])
            out[f"align_{axis}_after_deg"] = float(values[-1])
            out[f"align_{axis}_improve_deg"] = float(
                abs(values[0]) - abs(values[-1])
            )
    else:
        for axis in ("t1", "t2"):
            out[f"align_{axis}_before_deg"] = np.nan
            out[f"align_{axis}_after_deg"] = np.nan
            out[f"align_{axis}_improve_deg"] = np.nan

    # Contact force relative to the bias captured at the clearance transition.
    fx = d["external_force_x"][setup] - d["contact_force_bias_x"][setup]
    fy = d["external_force_y"][setup] - d["contact_force_bias_y"][setup]
    fz = d["external_force_z"][setup] - d["contact_force_bias_z"][setup]
    f = np.sqrt(fx ** 2 + fy ** 2 + fz ** 2)
    out["force_final_N"] = float(f[-1])
    out["force_max_N"] = float(f.max())
    # Steady value: median of the last quarter, robust to the contact spike.
    tail = max(1, len(f) // 4)
    out["force_steady_N"] = float(np.median(f[-tail:]))

    # Edge travel: how far the pressed contact feature slid.
    ex = d["tool_contact_x"][setup] - d["first_contact_x"][setup]
    ey = d["tool_contact_y"][setup] - d["first_contact_y"][setup]
    ez = d["tool_contact_z"][setup] - d["first_contact_z"][setup]
    out["edge_travel_mm"] = float(1000.0 * np.sqrt(ex[-1] ** 2 + ey[-1] ** 2 + ez[-1] ** 2))

    tau = np.vstack([d[f"tau_cmd_{i}"][setup] for i in range(1, 8)])
    out["tau_max_Nm"] = float(np.abs(tau).max())
    out["tau_norm_max_Nm"] = float(np.linalg.norm(tau, axis=0).max())

    # Equilibrium check: how much the tool still moved over the last 20% of the
    # phase. Large values mean the reported number is a transient, not an
    # equilibrium, which invalidates the quasi-static reading.
    last = max(2, len(e_r) // 5)
    out["tip_drift_last20pct_deg"] = float(
        np.degrees(abs(e_r[-1] - e_r[-last])))

    return out


def hold_metrics(path):
    """Extract null-space metrics from a general log covering a hold test."""
    wanted = [
        "time", "phase", "nullspace_mode",
        "sigma_current", "sigma_difference", "sigma_direction_valid",
        "tau_sigma_norm", "nullspace_speed", "sigma_speed_toward_better",
        "sigma_Jn_norm", "tau_nullspace_norm",
        "e_p_x", "e_p_y", "e_p_z",
        "e_R_x", "e_R_y", "e_R_z",
    ]
    d, _ = read_csv(path, wanted)

    hold = phase_mask(d["phase"], PHASE_HOLD)
    out = {"hold_samples": int(hold.sum())}
    if out["hold_samples"] < 2:
        out["hold_present"] = False
        return out
    out["hold_present"] = True

    out["nullspace_mode"] = int(np.median(d["nullspace_mode"][hold]))
    s = d["sigma_current"][hold]
    out["sigma_start"] = float(s[0])
    out["sigma_end"] = float(s[-1])
    out["sigma_min"] = float(s.min())
    out["sigma_max"] = float(s.max())
    out["sigma_gain"] = float(s[-1] - s[0])

    out["tau_sigma_norm_mean"] = float(d["tau_sigma_norm"][hold].mean())
    out["tau_sigma_norm_max"] = float(d["tau_sigma_norm"][hold].max())
    out["nullspace_speed_mean"] = float(d["nullspace_speed"][hold].mean())
    out["speed_toward_better_mean"] = float(
        d["sigma_speed_toward_better"][hold].mean())
    out["direction_valid_fraction"] = float(
        d["sigma_direction_valid"][hold].mean())

    # Task invariance: null-space torque must not disturb the Cartesian task.
    # This is the empirical proof that the projector is correct.
    ep = _norm3({k: d[k][hold] for k in ("e_p_x", "e_p_y", "e_p_z")}, "e_p")
    er = _norm3({k: d[k][hold] for k in ("e_R_x", "e_R_y", "e_R_z")}, "e_R")
    out["task_pos_error_max_mm"] = float(1000.0 * ep.max())
    out["task_pos_error_drift_mm"] = float(1000.0 * abs(ep[-1] - ep[0]))
    out["task_rot_error_max_deg"] = float(np.degrees(er.max()))
    out["task_rot_error_drift_deg"] = float(np.degrees(abs(er[-1] - er[0])))
    out["jacobian_null_residual_max"] = float(d["sigma_Jn_norm"][hold].max())

    return out


def parse_setup_report(terminal_log_path):
    """Pull the controller's own set-up report out of the saved transcript.

    Gives an independent cross-check of the CSV-derived metrics: if the two
    disagree, one of them is wrong and the run should not be used.
    """
    out = {}
    if not os.path.exists(terminal_log_path):
        return out
    # The pole the controller actually commanded, relative to the contact
    # edge. This is NOT the coupled_pole_from_edge parameter: the printed value
    # resolves the tool geometry as well. Its x component turns out to be the
    # variable that governs alignment, so it has to come from the transcript
    # rather than from params_effective.
    want_pole = False
    with open(terminal_log_path, errors="replace") as f:
        for line in f:
            line = line.strip()
            if "manual pole" in line and "COMMANDED" in line:
                want_pole = True
                continue
            if want_pole and line.startswith("pole_from_edge"):
                vec = line.split("[")[1].split("]")[0]
                v = [float(x) for x in vec.split(",")]
                out["pole_cmd_x_mm"], out["pole_cmd_y_mm"], out["pole_cmd_z_mm"] = v
                want_pole = False
                continue
            if line.startswith("stop:"):
                for part in line.split("|"):
                    part = part.strip()
                    # Archives predating the renames carry "tip=" or "defl=".
                    if part.startswith(("dev=", "defl=", "tip=")):
                        out["report_angular_deviation_deg"] = float(
                            part.split("=", 1)[1].split()[0])
                    elif part.startswith("F="):
                        out["report_force_N"] = float(part[2:].split()[0])
                    elif part.startswith("M="):
                        out["report_moment_Nm"] = float(part[2:].split()[0])
                    elif part.startswith("t="):
                        out["report_phase_time_s"] = float(part[2:].split()[0])
            elif line.startswith("alignment:"):
                for part in line.replace("alignment:", "").split("|"):
                    part = part.strip()
                    if part.startswith("before="):
                        out["report_align_before_deg"] = float(part[7:].split()[0])
                    elif part.startswith("after="):
                        out["report_align_after_deg"] = float(part[6:].split()[0])
                    elif part.startswith("gain="):
                        out["report_align_gain_deg"] = float(part[5:].split()[0])
            elif line.startswith("r_c [t1,t2,n]"):
                vec = line.split("[", 2)[2].split("]")[0]
                values = [float(value) for value in vec.split(",")]
                (out["report_rc_t1_mm"],
                 out["report_rc_t2_mm"],
                 out["report_rc_n_mm"]) = values
            # Where on the tool the pivot actually was. A pole commanded in
            # the surface frame moves in the tool as the tilt changes, so this
            # is the column a claim about the tool has to be made from.
            elif line.startswith("p_c [EE]"):
                vec = line.split("[", 2)[2].split("]")[0]
                (out["report_pc_ee_x_mm"],
                 out["report_pc_ee_y_mm"],
                 out["report_pc_ee_z_mm"]) = [
                    float(value) for value in vec.split(",")]
    return out


def read_overlay(path):
    """Read a setup overlay into {key: value} for labelling plots."""
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    return out
