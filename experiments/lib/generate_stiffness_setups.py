#!/usr/bin/env python3
"""Regenerate the stiffness and direction cases under the current press.

Run:  python3 experiments/lib/generate_stiffness_setups.py

Cases A to C were measured with an earlier press: a normal translational
stiffness of 800 N/m against a commanded penetration of 60 mm, where the
compliance-centre cases used 350 N/m against 240 mm. The steady load differs
by nearly a factor of two, and one condition common to both groups differs by
a factor of nine. These setups repeat the three cases under the settings the
compliance-centre cases carry, so the whole results chapter can rest on one
press, one controller version and one log format.

  A  the rotational stiffness about the commanded tangent
  B  the translational stiffness across it, and one combined setting
  C  the commanded rotation direction, at four directions in the tangent
     plane, with the offset selected for each and with one held fixed

The 5 N m/rad and 2000 N/m references are the trials of Case~D and are not
repeated here.

The assisting offset direction is not derived for the diagonals; it is
interpolated between the two principal directions the campaign measured. At
the tangents the interpolation returns exactly those measured settings.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from generate_setups import (COMMON, REPEATS, SETUPS, LEVER_OFFSET_EE,  # noqa: E402
                             offset_keys, tool_frame_lever, write)

TILT_DEG = 10.0

# The offset the direction cases carry [m], the outer position of the sweep.
LEVER_M = 0.040

# Rotational stiffness about the commanded tangent [N m/rad]. 5 is the value
# every other case runs at and is measured by Case D.
ROTATIONAL = [15.0, 50.0]

# Translational stiffness across the commanded tangent [N/m]. 2000 is the
# value every other case runs at.
TRANSLATIONAL = [300.0, 800.0]

# The four commanded rotation directions, as an angle in the tangent plane
# measured from t1 [deg].
DIRECTIONS = [0.0, 45.0, 90.0, -45.0]


def rotational_keys(axis, value):
    """Set the rotational entry about one tangent, leaving the other alone."""
    key = "setup_KR_tangent1" if axis == "t1" else "setup_KR_tangent2"
    return [(key, f"{value:.1f}")]


def translational_keys(axis, value):
    """Set the translational entry across the commanded tangent.

    A commanded rotation about t1 turns the tool over t2, so the entry that
    resists the accompanying translation is the one along t2.
    """
    key = ("setup_Kp_surface_tangent2" if axis == "t1"
           else "setup_Kp_surface_tangent1")
    return [(key, f"{value:.1f}")]


def direction_offset(angle_deg, distance):
    """Interpolate the assisting offset between the two measured directions.

    LEVER_OFFSET_EE holds the end-effector offset that assists a positive
    commanded offset about each tangent, both established by measurement. A
    direction between them takes the same combination its own components take,
    so the two principal directions return their measured settings exactly.
    """
    a = math.radians(angle_deg)
    t1_part = [c * math.cos(a) for c in LEVER_OFFSET_EE["t1"]]
    t2_part = [c * math.sin(a) for c in LEVER_OFFSET_EE["t2"]]
    return [distance * (u + v) for u, v in zip(t1_part, t2_part)]


def direction_tag(angle_deg):
    if angle_deg == 0.0:
        return "t1"
    if angle_deg == 90.0:
        return "t2"
    return f"{'m' if angle_deg < 0 else 'p'}{abs(int(angle_deg)):02d}"


def build():
    setups = {}

    # A -- the rotational entry about the commanded tangent.
    for axis in ("t1", "t2"):
        for value in ROTATIONAL:
            setups[f"A_rot_{axis}_{int(value):02d}"] = (
                offset_keys(TILT_DEG if axis == "t1" else 0.0,
                            TILT_DEG if axis == "t2" else 0.0)
                + rotational_keys(axis, value),
                f"Rotational stiffness {value:.0f} N m/rad about {axis}, "
                f"commanded offset +{TILT_DEG:.0f} deg about {axis}.",
                "A stiffer rotational entry is expected to resist the "
                "alignment motion and reduce the change.",
            )

    # B -- the translational entry across it, and the combined setting.
    for axis in ("t1", "t2"):
        for value in TRANSLATIONAL:
            setups[f"B_trans_{axis}_{int(value):04d}"] = (
                offset_keys(TILT_DEG if axis == "t1" else 0.0,
                            TILT_DEG if axis == "t2" else 0.0)
                + translational_keys(axis, value),
                f"Translational stiffness {value:.0f} N/m across {axis}, "
                f"commanded offset +{TILT_DEG:.0f} deg about {axis}.",
                "Separates the translational response coupled to the "
                "alignment motion from the rotational one.",
            )
        setups[f"B_combined_{axis}"] = (
            offset_keys(TILT_DEG if axis == "t1" else 0.0,
                        TILT_DEG if axis == "t2" else 0.0)
            + translational_keys(axis, TRANSLATIONAL[0])
            + rotational_keys(axis, ROTATIONAL[-1]),
            f"Lowest translational and highest rotational stiffness together, "
            f"commanded offset +{TILT_DEG:.0f} deg about {axis}.",
            "Whether the two act independently at the tested pair.",
        )

    # C -- the commanded rotation direction, with the offset selected for it.
    # The two principal directions are the 40 mm trials of Case E and are not
    # repeated; only the diagonals are new.
    for angle in DIRECTIONS[1::2]:
        a = math.radians(angle)
        setups[f"C_dir_{direction_tag(angle)}"] = (
            offset_keys(TILT_DEG * math.cos(a), TILT_DEG * math.sin(a))
            + tool_frame_lever(direction_offset(angle, LEVER_M)),
            f"Commanded offset +{TILT_DEG:.0f} deg at {angle:+.0f} deg from "
            f"t1 in the tangent plane, with the offset selected for it.",
            "The selection rule is expected to assist at every direction, "
            "not only at the two tangents.",
        )

    # C -- and the same directions with one offset held fixed, which is the
    # one that assists a commanded offset about t1.
    fixed = direction_offset(0.0, LEVER_M)
    for angle in DIRECTIONS[1:]:
        a = math.radians(angle)
        setups[f"C_fixed_{direction_tag(angle)}"] = (
            offset_keys(TILT_DEG * math.cos(a), TILT_DEG * math.sin(a))
            + tool_frame_lever(fixed),
            f"Commanded offset +{TILT_DEG:.0f} deg at {angle:+.0f} deg from "
            f"t1, with the offset held at the one selected for t1.",
            "Paired with the selected trial at the same direction, so the "
            "cost of not turning the offset with the command is measured.",
        )

    return setups


def main():
    setups = build()
    for run_id, (pairs, purpose, criterion) in sorted(setups.items()):
        write(run_id, pairs, purpose, criterion)

    index = os.path.join(SETUPS, "INDEX.txt")
    known = set()
    if os.path.exists(index):
        with open(index) as f:
            known = {l.strip() for l in f if l.strip()
                     and not l.startswith("#")}
    added = [r for r in sorted(setups) if r not in known]
    if added:
        with open(index, "a") as f:
            f.write("\n# Generated by lib/generate_stiffness_setups.py.\n")
            for run_id in added:
                f.write(f"{run_id}\n")

    print(f"wrote {len(setups)} setups, appended {len(added)} run ids")
    print(f"{len(setups)} settings x {REPEATS} repeats = "
          f"{len(setups) * REPEATS} trials")
    for run_id in sorted(setups):
        print(f"  {run_id}")


if __name__ == "__main__":
    main()
