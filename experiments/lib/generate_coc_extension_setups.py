#!/usr/bin/env python3
"""Prepare the unmeasured +/-90 and +/-100 mm extension of thesis Case D.

Writes eight standalone setups and a separate manifest; never starts the robot
or adds the settings to the general campaign queue.
"""

import hashlib
import json
from pathlib import Path

from apply_overlay import read_overlay

EXPERIMENTS = Path(__file__).resolve().parents[1]
PARAMS = EXPERIMENTS.parent / "surface_grinding_controller" / "params"
DESTINATION = EXPERIMENTS / "coc_extension"
POSITIONS_MM = (-90, 90, -100, 100)
REPEATS = 3
REFERENCE_FILES = (
    "setup.conf", "approach.conf", "phase_gates.conf",
    "auto_damping.conf", "nullspace.conf", "tool_orientation.conf",
)
RENAMES = {
    "enable_orientation_phase": "enable_orientation_state",
    "pause_before_setup": "enable_pre_contact_hold",
    "pause_before_grind": "enable_pre_grinding_hold",
}


def current_parameter(key, value):
    """Translate archived names while preserving their physical meaning."""
    if key == "setup_moment_threshold":
        # The old 150 Nm threshold never ended these trials; both versions
        # complete contact establishment at the configured five-second timeout.
        return None
    for old, new in (("setup_", "contact_establishment_"),
                     ("pause_hold_", "operator_hold_")):
        if key.startswith(old):
            return new + key[len(old):], value
    old_lever = "r_tcp_from_compliance_center_surface_"
    if key.startswith(old_lever):
        return ("compliance_lever_surface_" + key[len(old_lever):],
                f"{-float(value):.6f}")
    return RENAMES.get(key, key), value


def main():
    available = {key for path in PARAMS.glob("*.conf")
                 for key, _ in read_overlay(path)}
    prepared = []
    for position in POSITIONS_MM:
        suffix = f"{'m' if position < 0 else 'p'}{abs(position):03d}"
        reference_suffix = "m080" if position < 0 else "p080"
        for direction in ("pos", "neg"):
            run_id = f"P2_t1_{direction}_{suffix}"
            reference_id = f"P2_t1_{direction}_{reference_suffix}"
            reference = EXPERIMENTS / "results" / reference_id / "r01"
            values = {}
            hashes = {}
            for filename in REFERENCE_FILES:
                path = reference / "params_effective" / filename
                hashes[filename] = hashlib.sha256(path.read_bytes()).hexdigest()
                for key, value in read_overlay(path):
                    pair = current_parameter(key, value)
                    if pair is not None:
                        values[pair[0]] = pair[1]
            unknown = values.keys() - available
            if unknown:
                raise ValueError(f"Unrecognised current parameters: {sorted(unknown)}")
            assert values["compliance_center_in_tool_frame"] == "1"
            assert values["compliance_lever_in_surface_frame"] == "0"
            assert float(values["compliance_center_offset_ee_y"]) == (
                0.08 if position < 0 else -0.08)
            values["compliance_center_offset_ee_y"] = f"{-position / 1000:.6f}"
            directory = EXPERIMENTS / "setups" / run_id
            directory.mkdir(parents=True, exist_ok=True)
            overlay = directory / "overlay.txt"
            overlay.write_text(
                f"# {run_id}: unmeasured thesis/presentation CoC extension.\n"
                f"# Reference: experiments/results/{reference_id}/r01.\n"
                "# Archived contact, approach, hold, damping and tilt parameters\n"
                "# translated to current keys. Only the tool-frame y offset changes.\n"
                "# Plot position [mm] = -1000 * compliance_center_offset_ee_y [m].\n"
                + "".join(f"{key} = {value}\n" for key, value in values.items()))
            (directory / "startup_mode.txt").write_text("s\n")
            (directory / "about.txt").write_text(
                f"run_id: {run_id}\nrepeats: {REPEATS}\n"
                f"reference: {reference_id}/r01\n\n"
                f"Commanded t1 tilt: {values['tool_target_offset_tangent1_deg']} deg.\n"
                f"Position on the existing Case-D plot: {position:+d} mm.\n"
                f"Tool-frame centre y: {-position / 1000:+.6f} m.\n\n"
                "Measurement pending. Record three complete contact-establishment\n"
                "trials before adding a mean and sample SD to either figure.\n")
            prepared.append(dict(
                run_id=run_id, plot_position_mm=position,
                commanded_tilt_t1_deg=float(values["tool_target_offset_tangent1_deg"]),
                compliance_center_offset_ee_y_m=-position / 1000,
                reference=f"experiments/results/{reference_id}/r01",
                reference_parameter_sha256=hashes,
                overlay_sha256=hashlib.sha256(overlay.read_bytes()).hexdigest(),
                repeats=REPEATS, status_at_preparation="measurement_pending"))
    DESTINATION.mkdir(parents=True, exist_ok=True)
    (DESTINATION / "manifest.json").write_text(json.dumps(prepared, indent=2) + "\n")
    (DESTINATION / "run_ids.txt").write_text(
        "# New t1 settings only, 90 mm before 100 mm; three repeats each.\n"
        + "".join(f"{row['run_id']}\n" for row in prepared))
    print(f"Prepared {len(prepared)} settings, {len(prepared) * REPEATS} pending trials.")
    print(f"Manifest: {DESTINATION / 'manifest.json'}")


if __name__ == "__main__":
    main()
