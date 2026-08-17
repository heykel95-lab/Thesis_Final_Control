#!/usr/bin/env python3
"""Collect one row per archived trial into experiments/derived/metrics.csv.

  python3 analysis/extract_metrics.py [--results DIR] [--out FILE]

Reads what each trial recorded rather than recomputing it: the set-up report in
terminal.log carries the alignment, the wrench and the stop condition, and
params_effective/ carries the settings the trial actually ran with. Both were
written by the trial itself, so a row here cannot disagree with its archive.

A trial that stopped before its report is written out with empty metrics and a
note, rather than dropped. A campaign's failures are part of its record.
"""

import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np

PHASE_SET_UP = 2  # ControlPhase::kSetup

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
RESULTS = os.path.join(REPO, "experiments", "results")
OUT = os.path.join(REPO, "experiments", "derived", "metrics.csv")

# Settings worth carrying next to the metrics, so a row is readable on its own.
PARAM_KEYS = (
    "tool_target_offset_tangent1_deg",
    "tool_target_offset_tangent2_deg",
    "tool_target_offset_normal_deg",
    "compliance_center_in_tool_frame",
    "compliance_lever_in_surface_frame",
    "compliance_center_offset_ee_x",
    "compliance_center_offset_ee_y",
    "compliance_center_offset_ee_z",
    "r_tcp_from_compliance_center_surface_tangent1",
    "r_tcp_from_compliance_center_surface_tangent2",
    "r_tcp_from_compliance_center_surface_normal",
    "setup_push_speed",
    "setup_push_end",
    "setup_timeout",
    "setup_translation_surface_frame",
)

FIELDS = (
    ("stop_reason", re.compile(r"^\s*stop:\s*(\w+)", re.M)),
    ("phase_time_s", re.compile(r"stop:.*\|\s*t=([-\d.]+)\s*s")),
    # Archives predating the renames carry tip=, defl= or dev=.
    ("end_effector_deviation_deg",
     re.compile(r"stop:.*\|\s*(?:ee|grip|dev|defl|tip)=([-\d.]+)\s*deg")),
    ("force_norm_n", re.compile(r"stop:.*\|\s*F=([-\d.]+)\s*N")),
    # Archives predating the rename carry a bare M=.
    ("moment_norm_nm", re.compile(r"stop:.*\|\s*M(?:_TCP)?=([-\d.]+)\s*Nm")),
    # Absent from archives written before the criterion was observed, and
    # printed as "not reached" when the run never settled inside it.
    ("t_align_s", re.compile(r"^\s*t_align:\s*([\d.]+)\s*s", re.M)),
    # The relative crossing, and the closest approach with its time. The last
    # two are written by every run, including those that never align.
    ("t_align_fraction_s",
     re.compile(r"^\s*t_align_fraction:\s*([\d.]+)\s*s", re.M)),
    ("deviation_min_deg",
     re.compile(r"^\s*deviation_min:\s*([-\d.]+)\s*deg", re.M)),
    ("t_deviation_min_s",
     re.compile(r"^\s*deviation_min:.*at\s*t=([-\d.]+)\s*s", re.M)),
    # aligned or not_aligned, so the runs that never met the tolerance can be
    # sorted out without being thrown away.
    ("align_status", re.compile(r"^\s*align_status:\s*(\w+)", re.M)),
    # Archives written before the rename carry "alignment:".
    ("deviation_before_deg",
     re.compile(r"(?:deviation|alignment):\s*before=([-\d.]+)")),
    ("deviation_after_deg",
     re.compile(r"(?:deviation|alignment):.*after=([-\d.]+)")),
    ("deviation_gain_deg",
     re.compile(r"(?:deviation|alignment):.*gain=([-+\d.]+)")),
)

# Surface-frame rows of the breakdown block, each three signed numbers.
VECTOR_ROWS = (
    ("force", re.compile(r"^\s*force\s*\[N\]\s*=\s*\[(.*)\]", re.M)),
    ("moment_contact", re.compile(r"^\s*M_contact\s*\[Nm\]\s*=\s*\[(.*)\]", re.M)),
    ("tcp_disp_mm", re.compile(r"^\s*tcp_disp\s*\[mm\]\s*=\s*\[(.*)\]", re.M)),
    ("contact_disp_mm", re.compile(r"^\s*contact_disp\s*\[mm\]\s*=\s*\[(.*)\]", re.M)),
)

AXES = ("t1", "t2", "n")


def read_params(directory):
    """Read every key from an archived parameter directory."""
    values = {}
    if not os.path.isdir(directory):
        return values
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".conf"):
            continue
        with open(os.path.join(directory, name)) as f:
            for raw in f:
                line = raw.split("#")[0].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()
    return values


def parse_report(path):
    """Pull the set-up report out of a trial transcript.

    The first surface-frame block is the alignment-target frame; a second
    M_contact row follows in tool-face axes and is deliberately not taken here.
    """
    row = {}
    if not os.path.isfile(path):
        return row
    with open(path, errors="replace") as f:
        text = f.read()

    for name, pattern in FIELDS:
        match = pattern.search(text)
        if match:
            row[name] = match.group(1)

    for name, pattern in VECTOR_ROWS:
        match = pattern.search(text)
        if not match:
            continue
        parts = [p.strip() for p in match.group(1).split(",")]
        if len(parts) != 3:
            continue
        for axis, value in zip(AXES, parts):
            row[f"{name}_{axis}"] = value
    return row


def surface_frame(tilt_x_deg, tilt_y_deg):
    """Build [tangent1, tangent2, normal] in the base frame from the tilts."""
    a = math.radians(tilt_x_deg)
    b = math.radians(tilt_y_deg)
    r_x = np.array([[1.0, 0.0, 0.0],
                    [0.0, math.cos(a), -math.sin(a)],
                    [0.0, math.sin(a), math.cos(a)]])
    r_y = np.array([[math.cos(b), 0.0, math.sin(b)],
                    [0.0, 1.0, 0.0],
                    [-math.sin(b), 0.0, math.cos(b)]])
    normal = r_y @ r_x @ np.array([0.0, 0.0, 1.0])
    normal /= np.linalg.norm(normal)
    tangent1 = np.array([1.0, 0.0, 0.0]) - normal * normal.dot([1.0, 0.0, 0.0])
    tangent1 /= np.linalg.norm(tangent1)
    return np.column_stack([tangent1, np.cross(normal, tangent1), normal])


def contact_rotation(trial, params):
    """Return the set-up rotation since first contact in surface axes [deg].

    The set-up phase holds the orientation reached at first contact as its
    reference, so the logged orientation error is the rotation away from it.
    That comes from joint angles alone: no tool axis and no plane zero enter
    it, which matters because the tool axis is only known to a degree or two
    and drifts as the tool settles in the gripper. The plane enters solely as
    the direction of the axes the rotation is resolved along.
    """
    logs = glob.glob(os.path.join(trial, "logs", "*.csv"))
    if not logs:
        return {}
    try:
        frame = surface_frame(float(params["surface_tilt_x_deg"]),
                              float(params["surface_tilt_y_deg"]))
    except (KeyError, ValueError):
        return {}

    rotation = []
    with open(logs[0]) as f:
        for record in csv.DictReader(f):
            if float(record["phase"]) != PHASE_SET_UP:
                continue
            rotation.append([float(record["e_R_x"]),
                             float(record["e_R_y"]),
                             float(record["e_R_z"])])
    if len(rotation) < 2:
        return {}

    final = np.degrees(np.array(rotation[-1])) @ frame
    peak = np.abs(np.degrees(np.array(rotation)) @ frame).max(axis=0)
    return {
        "contact_rotation_t1_deg": final[0],
        "contact_rotation_t2_deg": final[1],
        "contact_rotation_normal_deg": final[2],
        "contact_rotation_t1_max_deg": peak[0],
        "contact_rotation_t2_max_deg": peak[1],
    }


def read_provenance(path):
    row = {}
    if not os.path.isfile(path):
        return row
    with open(path) as f:
        for raw in f:
            if ":" not in raw:
                continue
            key, value = raw.split(":", 1)
            row[key.strip()] = value.strip()
    return row


def collect(results_dir):
    rows = []
    for run_id in sorted(os.listdir(results_dir)):
        run_dir = os.path.join(results_dir, run_id)
        if not os.path.isdir(run_dir):
            continue
        for repeat in sorted(os.listdir(run_dir)):
            trial = os.path.join(run_dir, repeat)
            if not os.path.isdir(trial):
                continue

            # The directory name is the authority on which condition a trial
            # belongs to. provenance.txt records the identifier the trial was
            # recorded under, which differs wherever an archive was moved after
            # the fact, so it is kept under its own key rather than merged.
            provenance = read_provenance(os.path.join(trial, "provenance.txt"))
            row = {"run_id": run_id, "repeat": repeat}
            row["series"] = run_id.split("_")[0]
            row["recorded_as"] = provenance.pop("run_id", "")
            provenance.pop("repeat", None)
            row.update(provenance)

            params = read_params(os.path.join(trial, "params_effective"))
            for key in PARAM_KEYS:
                row[key] = params.get(key, "")

            report = parse_report(os.path.join(trial, "terminal.log"))
            row.update(report)
            row.update(contact_rotation(trial, params))
            if "deviation_gain_deg" not in report:
                row["note"] = "no set-up report in transcript"
            rows.append(row)
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--results", default=RESULTS)
    p.add_argument("--out", default=OUT)
    args = p.parse_args()

    if not os.path.isdir(args.results):
        sys.exit(f"no results directory at {args.results}")

    rows = collect(args.results)
    if not rows:
        sys.exit("no trials found")

    # Union of the keys, in first-seen order, so a partial trial still fits.
    columns = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns, restval="")
        writer.writeheader()
        writer.writerows(rows)

    incomplete = sum(1 for row in rows if row.get("note"))
    print(f"wrote {len(rows)} rows to {args.out}"
          + (f" ({incomplete} without a set-up report)" if incomplete else ""))


if __name__ == "__main__":
    main()
