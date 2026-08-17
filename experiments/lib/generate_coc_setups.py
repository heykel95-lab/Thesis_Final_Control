#!/usr/bin/env python3
"""Generate the centre-of-compliance campaign in three phases.

Run:  python3 experiments/lib/generate_coc_setups.py

Written next to lib/generate_setups.py rather than inside it, so the reported
campaign keeps its own spec and stays reproducible from it. The setups land in
the same directory and are appended to INDEX.txt, which is what
run_campaign.sh walks, so the new trials queue behind the archived ones.

Every trial runs the full set-up duration. Nothing ends it early: the moment
threshold is left where it sits, far above anything the contact reaches, and
the alignment criterion is observed rather than enforced. What a run cost in
time is therefore the same for all of them, and the alignment time comes out
of the log afterwards.

  Phase 1  the two tilt axes at one commanded magnitude, centre on the TCP.
           Nothing is varied but the axis, so the asymmetry between them is
           the whole content.

  Phase 2  both tilt signs on both axes, each at three centre positions along
           the assisting tangent. This is what settles whether the assisting
           direction follows the sign of the commanded tilt.

  Phase 3  one reproducible case, the centre stepped along the tool axis.
           120 mm is left out: it loads joint 6 past its limit and the motion
           aborts, so the sweep stops where the robot allows.

A phase is a way of reading the trials, not a separate set of them. Three of
the configurations belong to two phases at once: the zero-position entries of
phase 2 are what phase 1 compares, and the t1 one is also where phase 3 starts.
Each configuration is generated once and PHASES below records which phase
reads which, so no trial is driven twice for the sake of the grouping.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from generate_setups import (COMMON, REPEATS, SETUPS, TOOL_AXIS_EE,  # noqa: E402
                             LEVER_OFFSET_EE, SURFACE_LEVER, offset_keys,
                             scaled, surface_frame_lever, tool_frame_lever,
                             no_lever, write)

# Commanded tilt magnitude carried by every phase [deg].
TILT_DEG = 10.0

# Centre positions along the assisting tangent, swept by phase 2 [m].
#
# The sign is measured along the assisting lever direction of the given axis,
# so p080 places the centre where the press makes a correcting moment and m080
# places it on the opposite side. That is not the convention of the S4 sweep,
# whose tags carry the raw offset along the face long axis; there p050 is the
# opposing side. The overlay file records the end-effector offset either way,
# and it is what the analysis reads.
PHASE2_POSITIONS = [-0.080, 0.0, 0.080]

# Centre positions along the tool axis, swept by phase 3 [m]. Zero is absent
# because phase 2 already carries it as P2_t1_pos_p000. The sweep is symmetric
# so the trend is read over its whole range rather than from one side, and it
# stops at 90 mm because 120 mm loads joint 6 past its limit.
PHASE3_POSITIONS = [-0.090, -0.060, -0.040, 0.040, 0.060, 0.090]

# The lever magnitude the frame comparison and the magnitude study carry [m],
# matching the outer position of phase 2 so the two are read against it.
LEVER_M = 0.080

# The smaller commanded tilt, for telling the response to the offset apart
# from the response to the centre position [deg].
SMALL_TILT_DEG = 5.0

# Which trials each phase is read from. The overlap is deliberate.
PHASES = {
    "phase 1, the two tilt axes with the centre on the TCP":
        ["P2_t1_pos_p000", "P2_t2_pos_p000"],
    "phase 2, direction and sign of the centre offset":
        None,  # every P2 trial
    "phase 3, distance along the tool axis":
        ["P2_t1_pos_p000", "P3_axis_p040", "P3_axis_p060", "P3_axis_p090"],
}


def tilt_keys(axis, sign, magnitude_deg=TILT_DEG):
    """Commanded offset of the given sign about the given surface tangent."""
    magnitude = sign * magnitude_deg
    return offset_keys(magnitude if axis == "t1" else 0.0,
                       magnitude if axis == "t2" else 0.0)


def position_tag(position):
    return f"{'m' if position < 0 else 'p'}{abs(1000 * position):03.0f}"


def sign_tag(sign):
    return "pos" if sign > 0 else "neg"


def build():
    """Return {run_id: (overlay pairs, purpose, pass criterion)}."""
    setups = {}

    # Phase 2 -- direction and sign. The centre moves along the tangent that
    # assists the given axis, in both directions and at zero.
    for axis in ("t1", "t2"):
        for sign in (+1, -1):
            for position in PHASE2_POSITIONS:
                run_id = (f"P2_{axis}_{sign_tag(sign)}_"
                          f"{position_tag(position)}")
                offset_ee = scaled(LEVER_OFFSET_EE[axis], position)
                pairs = (tilt_keys(axis, sign)
                         + (no_lever() if position == 0.0
                            else tool_frame_lever(offset_ee)))
                setups[run_id] = (
                    pairs,
                    f"Commanded offset {sign * TILT_DEG:+.0f} deg about "
                    f"{axis}, centre {1000 * position:+.0f} mm along the "
                    f"assisting tangent.",
                    "The assisting direction is expected to reverse with the "
                    "sign of the commanded offset. A position that helps at "
                    "one sign should hinder at the other.",
                )

    # Phase 3 -- the distance along the tool axis, at the case phase 2 leaves
    # best established.
    for position in PHASE3_POSITIONS:
        run_id = f"P3_axis_{position_tag(position)}"
        pairs = (tilt_keys("t1", +1)
                 + (no_lever() if position == 0.0
                    else tool_frame_lever(scaled(TOOL_AXIS_EE, position))))
        setups[run_id] = (
            pairs,
            f"Commanded offset +{TILT_DEG:.0f} deg about t1, centre "
            f"{1000 * position:+.0f} mm along the tool axis.",
            "Separates how far the centre sits from which direction it sits "
            "in. The alignment time is read from the log alongside the "
            "residual, so a faster correction and a smaller one are told "
            "apart.",
        )

    # The definition frame. The same physical lever is named in surface
    # coordinates instead of tool coordinates. At zero offset the two describe
    # one lever and must agree; at 10 deg they separate as the tool turns, and
    # that separation is the frame and nothing else.
    for tag, sign, magnitude in (("00deg", +1, 0.0), ("t1_10deg", +1, TILT_DEG)):
        setups[f"P4_frame_{tag}"] = (
            tilt_keys("t1", sign, magnitude)
            + surface_frame_lever(*scaled(SURFACE_LEVER["t1"], LEVER_M)),
            f"Surface-frame centre at {1000 * LEVER_M:.0f} mm, commanded "
            f"offset {tag.replace('_', ' ')}.",
            "Should agree with the tool-frame trial at zero offset and part "
            "from it at 10 deg. The zero pair is the consistency check.",
        )

    # The commanded magnitude. Both tangents at half the tilt, with and
    # without the assisting centre, so the response to the offset is told
    # apart from the response to the centre position.
    for axis in ("t1", "t2"):
        for position in (0.0, LEVER_M):
            run_id = f"P5_mag_{axis}_{position_tag(position)}"
            setups[run_id] = (
                tilt_keys(axis, +1, SMALL_TILT_DEG)
                + (no_lever() if position == 0.0
                   else tool_frame_lever(
                       scaled(LEVER_OFFSET_EE[axis], position))),
                f"Commanded offset +{SMALL_TILT_DEG:.0f} deg about {axis}, "
                f"centre {1000 * position:+.0f} mm along the assisting "
                f"tangent.",
                "Pairs with the 10 deg trials at the same centre positions, "
                "so whether the correction scales with the commanded offset "
                "can be read directly.",
            )

    return setups


def main():
    setups = build()
    for run_id, (pairs, purpose, criterion) in sorted(setups.items()):
        write(run_id, pairs, purpose, criterion)

    # Appending rather than rewriting: the archived campaign keeps its entries
    # and its order, and the new trials queue after them.
    index = os.path.join(SETUPS, "INDEX.txt")
    existing = []
    if os.path.exists(index):
        with open(index) as f:
            existing = [line.rstrip("\n") for line in f]
    known = {line.strip() for line in existing if line.strip()
             and not line.startswith("#")}

    added = [run_id for run_id in sorted(setups) if run_id not in known]
    if added:
        with open(index, "a") as f:
            f.write("\n# Generated by lib/generate_coc_setups.py.\n")
            for run_id in added:
                f.write(f"{run_id}\n")

    print(f"wrote {len(setups)} setups to {SETUPS}")
    print(f"appended {len(added)} run ids to INDEX.txt")
    print(f"{len(setups)} settings x {REPEATS} repeats = "
          f"{len(setups) * REPEATS} trials")
    for title, members in PHASES.items():
        listed = sorted(setups) if members is None else members
        print(f"\n{title}:")
        for run_id in listed:
            if members is None and not run_id.startswith("P2_"):
                continue
            print(f"  {run_id}")


if __name__ == "__main__":
    main()
