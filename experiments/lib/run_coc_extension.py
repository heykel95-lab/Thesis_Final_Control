#!/usr/bin/env python3
"""Inspect or collect the prepared CoC extension, stopping on any failed trial.

Without --run this only reports progress. Existing successful trials are
validated and skipped. Failed archives are preserved for inspection.
"""

import argparse
import csv
import hashlib
import json
import re
import subprocess
import sys
from pathlib import Path

from apply_overlay import read_overlay

ROOT = Path(__file__).resolve().parents[2]
EXPERIMENTS = ROOT / "experiments"


def check_trial(setting, repeat):
    trial = EXPERIMENTS / "results" / setting["run_id"] / repeat
    if not trial.exists():
        return None
    provenance = dict(line.split(":", 1) for line in
                      (trial / "provenance.txt").read_text().splitlines() if ":" in line)
    if provenance.get("exit_status", "").strip() != "0":
        raise ValueError(f"Failed archive requires inspection: {trial}")
    report = (trial / "terminal.log").read_text(errors="replace")
    if "stop: time | t=5.0 s" not in report:
        raise ValueError(f"No complete five-second contact report: {trial}")
    match = re.search(r"deviation components.*?before=\[(.*?)\].*?after=\[(.*?)\]", report)
    if not match:
        raise ValueError(f"Missing angular-error endpoints: {trial}")
    saved = dict(pair for path in (trial / "params_effective").glob("*.conf")
                 for pair in read_overlay(path))
    overlay = EXPERIMENTS / "setups" / setting["run_id"] / "overlay.txt"
    for key, value in read_overlay(overlay):
        if float(saved[key]) != float(value):
            raise ValueError(f"Archived setting differs: {trial}: {key}")
    logs = list((trial / "logs").glob("*.csv"))
    if len(logs) != 1:
        raise ValueError(f"Expected one raw log: {trial}")
    with logs[0].open() as handle:
        rows = [row for row in csv.DictReader(handle)
                if row.get("state", row.get("phase")) == "2"]
    duration = float(rows[-1]["time"]) - float(rows[0]["time"]) if rows else 0
    if len(rows) < 4000 or not 4.95 <= duration <= 5.05:
        raise ValueError(f"Incomplete contact samples: {trial}: {len(rows)}, {duration}s")
    return dict(run_id=setting["run_id"], repeat=repeat,
                contact_samples=len(rows), contact_duration_s=duration,
                final_t1_deg=-float(match[2].split(",")[0]),
                raw_log_sha256=hashlib.sha256(logs[0].read_bytes()).hexdigest())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="Start robot trials")
    parser.add_argument("--limit", type=int, help="Maximum new trials in this invocation")
    args = parser.parse_args()
    manifest = json.loads((EXPERIMENTS / "coc_extension" / "manifest.json").read_text())
    completed, pending = [], []
    for setting in manifest:
        overlay = EXPERIMENTS / "setups" / setting["run_id"] / "overlay.txt"
        if hashlib.sha256(overlay.read_bytes()).hexdigest() != setting["overlay_sha256"]:
            raise ValueError(f"Overlay changed since preparation: {overlay}")
        for index in range(1, setting["repeats"] + 1):
            repeat = f"r{index:02d}"
            result = check_trial(setting, repeat)
            if result is None:
                pending.append((setting, index))
            else:
                completed.append(result)
    print(f"{len(completed)}/24 trials complete; {len(pending)} pending.", flush=True)
    if not args.run:
        return
    if args.limit is not None and args.limit < 1:
        parser.error("--limit must be positive")
    for setting, index in pending[:args.limit]:
        run_id = setting["run_id"]
        print(f"START {run_id} r{index:02d}", flush=True)
        console = EXPERIMENTS / "coc_extension" / "acquisition.log"
        with console.open("ab") as output:
            result = subprocess.run(
                [sys.executable, str(EXPERIMENTS / "lib" / "auto_drive.py"), run_id, str(index)],
                cwd=ROOT, stdout=output, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError(f"Trial stopped with exit {result.returncode}: {run_id} r{index:02d}; see {console}")
        checked = check_trial(setting, f"r{index:02d}")
        if checked is None:
            raise RuntimeError(f"Trial produced no archive: {run_id} r{index:02d}")
        completed.append(checked)
        (EXPERIMENTS / "coc_extension" / "completed_trials.json").write_text(
            json.dumps(completed, indent=2) + "\n")
        print(f"PASS {len(completed)}/24: {run_id} r{index:02d}, "
              f"angular error {checked['final_t1_deg']:+.2f} deg", flush=True)


if __name__ == "__main__":
    main()
