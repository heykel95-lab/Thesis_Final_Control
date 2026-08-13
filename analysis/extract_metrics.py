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
import os
import re
import sys

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
    ("stop_reason", re.compile(r"^\s*stop:\s*(\w+)")),
    ("phase_time_s", re.compile(r"stop:.*\|\s*t=([-\d.]+)\s*s")),
    # Archives predating the renames carry "tip=" or "defl=".
    ("angular_deviation_deg",
     re.compile(r"stop:.*\|\s*(?:dev|defl|tip)=([-\d.]+)\s*deg")),
    ("force_norm_n", re.compile(r"stop:.*\|\s*F=([-\d.]+)\s*N")),
    ("moment_norm_nm", re.compile(r"stop:.*\|\s*M=([-\d.]+)\s*Nm")),
    ("align_before_deg", re.compile(r"alignment:\s*before=([-\d.]+)")),
    ("align_after_deg", re.compile(r"alignment:.*after=([-\d.]+)")),
    ("align_gain_deg", re.compile(r"alignment:.*gain=([-+\d.]+)")),
)

# Surface-frame rows of the breakdown block, each three signed numbers.
VECTOR_ROWS = (
    ("force", re.compile(r"^\s*force\s*\[N\]\s*=\s*\[(.*)\]")),
    ("moment_contact", re.compile(r"^\s*M_contact\s*\[Nm\]\s*=\s*\[(.*)\]")),
    ("tcp_disp_mm", re.compile(r"^\s*tcp_disp\s*\[mm\]\s*=\s*\[(.*)\]")),
    ("contact_disp_mm", re.compile(r"^\s*contact_disp\s*\[mm\]\s*=\s*\[(.*)\]")),
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
            if "align_gain_deg" not in report:
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
