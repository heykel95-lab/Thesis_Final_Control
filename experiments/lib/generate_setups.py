#!/usr/bin/env python3
"""Generate the campaign setup overlays from one declarative spec.

Each setup is a directory under experiments/setups/<run_id>/ holding:
  overlay.txt        the parameter keys that differ from params/
  about.txt          what the trial varies and what result would count as a pass
  startup_mode.txt   the startup menu key the trial is driven with

Generating them instead of hand-writing the matrix keeps it reproducible: the
spec below is the single source of truth, and the tool-frame lever offsets are
computed from the tool geometry rather than typed in.

Run:  python3 experiments/lib/generate_setups.py

The campaign answers one question in five series. The compliance centre must be
defined in the tool frame for the correction to hold at every commanded tool
orientation offset; a surface-frame definition moves the point on the tool as
the tool turns, which changes two things at once.

  S1  no lever, every offset          the baseline: without coupling there is
                                      no corrective moment to measure
  S2  tool-frame lever, every offset  the claim
  S3  surface-frame lever, two offsets  isolates the definition frame
  S4  tangential lever position       where the point belongs in the plane
  S5  normal lever position           and how far above or below it

Every trial runs the translational stiffness in the surface frame. No
base-frame arm is included: the campaign reports one configuration rather than
a comparison between two.
"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
SETUPS = os.path.normpath(os.path.join(HERE, "..", "setups"))

# The tool axis in end-effector coordinates, from params/tool_geometry.conf.
# The compliance centre is displaced along it, so a tool-frame lever stays on
# the same physical point of the tool at every commanded orientation offset.
TOOL_AXIS_EE = (0.0, 0.0, 1.0)

# The face long axis, used for the in-plane lever positions of S4.
FACE_LONG_EE = (0.0, 1.0, 0.0)

# Repeats per setting, matching the protocol used for the reported campaign.
REPEATS = 3

# Commanded tool orientation offsets [deg] about t1 and t2.
OFFSETS = [
    ("00deg", 0.0, 0.0),
    ("t1_05deg", 5.0, 0.0),
    ("t1_10deg", 10.0, 0.0),
    ("t2_05deg", 0.0, 5.0),
    ("t2_10deg", 0.0, 10.0),
]

# The lever magnitude carried by S2 and S3 [m].
LEVER_M = 0.060

# Tangential lever positions swept by S4 [m], along the face long axis.
S4_POSITIONS = [-0.050, -0.020, 0.0, 0.020, 0.050]

# Normal lever positions swept by S5 [m], along the tool axis. Positive values
# place the centre further from the tool along its own axis.
S5_POSITIONS = [-0.060, 0.0, 0.040, 0.060, 0.120]

# The commanded offset S4 and S5 hold fixed while they sweep the lever.
SWEEP_OFFSET = ("t1_10deg", 10.0, 0.0)

# Applied to every trial: surface-frame translational stiffness, the gates
# driven by lib/auto_drive.py, and the compliance-centre coupling active.
COMMON = [
    ("setup_translation_surface_frame", "1"),
    ("pause_hold_translation_surface_frame", "1"),
    ("pause_hold_rotation_surface_frame", "1"),
    ("pause_before_setup", "1"),
    ("pause_before_grind", "1"),
    ("use_virtual_compliance_center", "1"),
]


def offset_keys(t1_deg, t2_deg):
    return [
        ("tool_target_offset_tangent1_deg", f"{t1_deg:.1f}"),
        ("tool_target_offset_tangent2_deg", f"{t2_deg:.1f}"),
    ]


def scaled(direction, distance):
    return [component * distance for component in direction]


def tool_frame_lever(offset_ee):
    """Select the tool-frame definition and place p_c at the given offset.

    The configuration stores p_c - p_TCP in end-effector coordinates, so the
    lever r_c = p_TCP - p_c is its negation. Writing the offset here keeps the
    spec in the same terms as the parameter file.
    """
    return [
        ("compliance_center_in_tool_frame", "1"),
        ("compliance_lever_in_surface_frame", "0"),
        ("compliance_center_offset_ee_x", f"{offset_ee[0]:.6f}"),
        ("compliance_center_offset_ee_y", f"{offset_ee[1]:.6f}"),
        ("compliance_center_offset_ee_z", f"{offset_ee[2]:.6f}"),
    ]


def surface_frame_lever(t1_m, t2_m, n_m):
    """Select the surface-frame definition and command r_c directly."""
    return [
        ("compliance_center_in_tool_frame", "0"),
        ("compliance_lever_in_surface_frame", "1"),
        ("r_tcp_from_compliance_center_surface_tangent1", f"{t1_m:.6f}"),
        ("r_tcp_from_compliance_center_surface_tangent2", f"{t2_m:.6f}"),
        ("r_tcp_from_compliance_center_surface_normal", f"{n_m:.6f}"),
    ]


def no_lever():
    """Place the centre of compliance at the TCP, leaving the law decoupled."""
    return [
        ("compliance_center_in_tool_frame", "0"),
        ("compliance_lever_in_surface_frame", "1"),
        ("r_tcp_from_compliance_center_surface_tangent1", "0.0"),
        ("r_tcp_from_compliance_center_surface_tangent2", "0.0"),
        ("r_tcp_from_compliance_center_surface_normal", "0.0"),
    ]


def build():
    """Return {run_id: (overlay pairs, purpose, pass criterion)}."""
    setups = {}

    # S1 -- the baseline. With the centre at the TCP the 6x6 gain is block
    # diagonal, so pressing produces force and no moment.
    for tag, t1, t2 in OFFSETS:
        setups[f"S1_none_{tag}"] = (
            offset_keys(t1, t2) + no_lever(),
            f"Baseline without compliance-centre coupling, "
            f"commanded offset {tag.replace('_', ' ')}.",
            "Alignment change should stay near zero at every commanded "
            "offset. This is the reference the other series are read against.",
        )

    # S2 -- the claim. The lever is fixed to the tool, so the same physical
    # point carries it at every commanded offset.
    lever_ee = scaled(TOOL_AXIS_EE, LEVER_M)
    for tag, t1, t2 in OFFSETS:
        setups[f"S2_tool_{tag}"] = (
            offset_keys(t1, t2) + tool_frame_lever(lever_ee),
            f"Tool-frame compliance centre at {1000 * LEVER_M:.0f} mm, "
            f"commanded offset {tag.replace('_', ' ')}.",
            "Alignment improvement should be present at every commanded "
            "offset. A correction that holds across the offsets supports the "
            "tool-frame definition.",
        )

    # S3 -- the definition frame. Two offsets are enough: at zero the two
    # definitions coincide, and any difference at 10 deg is the frame.
    for tag, t1, t2 in (OFFSETS[0], OFFSETS[2]):
        setups[f"S3_surface_{tag}"] = (
            offset_keys(t1, t2) + surface_frame_lever(0.0, 0.0, LEVER_M),
            f"Surface-frame compliance centre at {1000 * LEVER_M:.0f} mm, "
            f"commanded offset {tag.replace('_', ' ')}.",
            "Should match S2 at zero offset and differ from it at 10 deg. "
            "The zero-offset pair is the consistency check.",
        )

    # S4 -- where the point belongs in the plane, at one commanded offset.
    tag, t1, t2 = SWEEP_OFFSET
    for position in S4_POSITIONS:
        name = f"{'m' if position < 0 else 'p'}{abs(1000 * position):03.0f}"
        setups[f"S4_tangential_{name}"] = (
            offset_keys(t1, t2)
            + tool_frame_lever(scaled(FACE_LONG_EE, position)),
            f"Tool-frame compliance centre {1000 * position:+.0f} mm along the "
            f"face long axis.",
            "Locates the tangential lever position giving the largest "
            "improvement at a 10 deg commanded offset about t1.",
        )

    # S5 -- and how far along the tool axis. The normal lever makes no moment
    # against a purely normal press, so a measured dependence indicates a
    # tangential component in the press.
    for position in S5_POSITIONS:
        name = f"{'m' if position < 0 else 'p'}{abs(1000 * position):03.0f}"
        setups[f"S5_normal_{name}"] = (
            offset_keys(t1, t2)
            + tool_frame_lever(scaled(TOOL_AXIS_EE, position)),
            f"Tool-frame compliance centre {1000 * position:+.0f} mm along the "
            f"tool axis.",
            "A response that is not symmetric about zero indicates a "
            "tangential component in the press.",
        )

    return setups


def write(run_id, pairs, purpose, criterion):
    directory = os.path.join(SETUPS, run_id)
    os.makedirs(directory, exist_ok=True)

    with open(os.path.join(directory, "overlay.txt"), "w") as f:
        f.write(f"# {run_id}\n")
        f.write("# Applied on top of surface_grinding_controller/params/.\n")
        f.write("# Only keys listed here differ from the nominal set.\n")
        for key, value in COMMON + pairs:
            f.write(f"{key} = {value}\n")

    with open(os.path.join(directory, "about.txt"), "w") as f:
        f.write(f"run_id:   {run_id}\n")
        f.write(f"repeats:  {REPEATS}\n\n")
        f.write(f"purpose:\n  {purpose}\n\n")
        f.write(f"pass criterion:\n  {criterion}\n")

    with open(os.path.join(directory, "startup_mode.txt"), "w") as f:
        f.write("s\n")


def main():
    setups = build()
    for run_id, (pairs, purpose, criterion) in sorted(setups.items()):
        write(run_id, pairs, purpose, criterion)

    index = os.path.join(SETUPS, "INDEX.txt")
    with open(index, "w") as f:
        f.write("# Generated by lib/generate_setups.py. Do not edit by hand.\n")
        for run_id in sorted(setups):
            f.write(f"{run_id}\n")

    trials = len(setups) * REPEATS
    print(f"wrote {len(setups)} setups ({trials} trials at {REPEATS} repeats)")


if __name__ == "__main__":
    main()
