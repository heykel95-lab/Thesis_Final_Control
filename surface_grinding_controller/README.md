# Surface Grinding Controller

This program implements a real-time Cartesian impedance controller for a
seven-joint Franka robot. The phase sequence orients the tool, approaches the
calibrated surface, applies the setup preload, and executes the grinding motion.
The program also provides Cartesian pose holding, setup-impedance holding, and
manual guidance.

## Program structure

| Path | Purpose |
|---|---|
| `main.cpp` | Connects the robot and coordinates one controller session. |
| `include/` | Defines configuration, controller data, common types, and module interfaces. |
| `src/config/` | Loads the parameter files. |
| `src/control/` | Implements Cartesian impedance, nullspace control, damping, geometry, and robot support. |
| `src/runtime/` | Executes the phase state machine and the 1 kHz torque-control loop. |
| `src/interface/` | Provides the startup menu, gripper actions, and manual guidance. |
| `src/report/` | Writes the control log and prints the run report. |
| `params/` | Stores robot, geometry, controller, safety, and experiment settings. |
| `tools/measure_plane.cpp` | Calibrates one point and the orientation of the workpiece plane. |
| `tools/measure_tool_axis.cpp` | Calibrates the grinding-tool axis in the end-effector frame. |
| `tools/measure_tool_axis_auto.cpp` | Calibrates the same axis without hand guidance. |

The main execution sequence is:

```text
main.cpp
  -> readControllerConfig()
  -> startup menu and robot preparation
  -> buildPhaseImpedanceGains()
  -> runControlLoop() at 1 kHz
  -> writeRunLogs()
```

## Build and start

The Makefile uses the following default paths:

```text
libfranka: /home/hm-panda/libfranka
Eigen:     /usr/include/eigen3
```

### Build first, then start

```bash
make
./surface_grinding_controller
```

### Build and start with one command

```bash
make run
```

At startup, the program performs automatic robot error recovery. A message is
printed only when recovery fails. The parameter directory is resolved relative
to the executable and printed once at startup.

Remove generated files with:

```bash
make clean
```

## Surface plane calibration

`measure_plane` is a read-only calibration tool for the workpiece plane. It
uses the current tool orientation to determine the surface-normal direction and
uses the configured contact-face center to calculate one point on the plane.
The tool does not command robot motion.

Before running the tool:

1. Verify the contact-face center in `params/tool_geometry.conf`.
2. Unlock the robot and guide the tool to the workpiece.
3. Seat the complete tool face flat on the plane.
4. Keep the tool stationary during the measurement.

Build the tool and start it separately:

```bash
make measure_plane
./tools/measure_plane
```

Alternatively, build and start it with one command:

```bash
make run_measure_plane
```

The program transforms the configured face-center offset from the
end-effector frame to the robot base frame:

```text
p_surface = p_EE + R_EE * r_face,EE
```

It prints the complete plane definition in the format used by
`params/surface.conf`:

```text
surface_point_x = ...
surface_point_y = ...
surface_point_z = ...
surface_tilt_x_deg = ...
surface_tilt_y_deg = ...
```

Copy these values to `params/surface.conf` and set
`use_start_as_surface_point = 0`. Run the calibration again while the tool face
remains seated. The normal mismatch and configured-plane offset should both be
close to zero.

## Grinding-tool axis calibration

`measure_tool_axis` is a guided calibration tool for the grinding-tool axis.
It uses four end-effector orientations recorded while the complete tool face
remains flat on the calibrated workpiece plane. Samples T1--T3 determine the axis in the
end-effector frame that remains invariant when the tool is rotated about the
surface normal. Sample T4 provides an independent validation.

Before running the tool:

1. Calibrate the workpiece plane and store it in `params/surface.conf`.
2. Switch the grinder off and remove any commanded surface preload.
3. Hand-guide the robot only while the tool reports that active guidance is on.
4. Keep the robot stationary and press Enter to record each sample.

Build the tool and start it separately:

```bash
make measure_tool_axis
./tools/measure_tool_axis
```

Alternatively, build and start it with one command:

```bash
make run_measure_tool_axis
```

The tool uses gravity-compensated hand guidance for all calibration samples.
T1 is used to seat the complete tool face flat on the surface. For T2, T3, and
T4, Cartesian damping limits translation and tilt while allowing rotation about
the configured surface normal. Joint damping remains active during guidance.

Keep the complete tool face in contact with the surface and vary only the yaw
angle. Use clearly separated orientations, for example 0 deg, +30 deg, -30 deg,
and +50 deg. Samples separated by less than 15 deg are rejected to avoid
unreliable calibration from insufficient yaw variation.

After T4, the tool prints the calibrated axis in the format used by
`params/tool_orientation.conf`:

```text
tool_axis_ee_x = ...
tool_axis_ee_y = ...
tool_axis_ee_z = ...
```

Copy these values to `params/tool_orientation.conf`. The reported T1--T3 axis
spread and T4 validation error indicate the repeatability of the calibration.
The surface-consistency error compares the calibrated tool direction with the
normal stored in `params/surface.conf`.

## Automatic grinding-tool axis calibration

`measure_tool_axis_auto` records the same kind of seatings without hand
guidance. The robot retracts the face, yaws about the calibrated surface
normal, lowers until contact, and presses the face onto the plane. Once contact
is detected, the rotational stiffness about both surface tangents is set to
zero and only rotational damping remains. The resting attitude of each seating
therefore follows from the contact geometry, not from the configured tool axis,
so the measurement does not assume the value it determines. Stiffness about the
surface normal stays active to hold the commanded yaw. No virtual compliance
center is used, so no lever enters the commanded wrench.

Six seatings are recorded, spread over 90 deg of yaw. The first five solve the
axis and the last one validates it.

Before running the tool:

1. Calibrate the workpiece plane and store it in `params/surface.conf`.
2. Switch the grinder off and keep the payload configured, since the released
   tilt axes rely on gravity compensation.
3. Position the tool face on the plane, or up to 150 mm above it, and
   roughly parallel to it.
4. Keep the working area clear. The robot moves without further confirmation.

Build the tool and start it separately:

```bash
make measure_tool_axis_auto
./tools/measure_tool_axis_auto
```

Alternatively, build and start it with one command:

```bash
make run_measure_tool_axis_auto
```

Pressing Enter stops the sequence at any time.

### Why the contact is unloaded before it is measured

Static friction at the contact scales with the normal force, exactly as the
restoring moment does. Pressing harder therefore leaves the same fraction of
the misalignment locked in, and measurements confirm this: raising the seating
force from 40 N to 80 N grew the axis spread from 0.14 to 0.79 deg and the
validation error from 0.49 to 1.39 deg. The larger tilt released at 80 N is the
face turning further before it jams, not seating better.

The seating stage therefore unloads the contact four times before the face is
measured. Each cycle returns to the full force, so the stage begins and ends
pressed and only the friction lock is interrupted.

### Reading the result

The tool reports one line per seating with the seating force, the settling
time, and the tilt released after the stiffness was removed. The released tilt
measures how far the face was from the plane when contact began, so it is
largest on the first seating of a fresh start and small afterwards.

The axis solution, its validation, and the printed `tool_orientation.conf`
entries match the guided tool, which shares the same estimator.

The spread and validation error describe one run. They do not describe how far
apart two runs land: seating scatter from an uneven face and from play in the
gripper dominates, and repeated runs have differed by around 0.7 to 1.0 deg
while agreeing to 0.2 to 0.3 deg within a run. Average several runs when the
axis direction matters at that level.

## Startup menu

| Key | Action |
|---|---|
| `s` | Move to the configured initial posture and run the complete phase sequence. |
| `h` | Move to the configured initial posture and hold the Cartesian pose. |
| `t` | Move to the configured initial posture and hold with the setup impedance. |
| `g` | Start manual guidance directly from the current robot pose. |
| `i` | Move to the configured initial joint posture. |
| `o` | Open the gripper. |
| `c` | Grasp the tool. |
| `r` | Recalibrate the gripper with empty fingers. |
| `f` | Pick up the tool from the configured holder posture. |
| `b` | Return the tool to the configured holder posture. |
| `e` | Exit the program. |

After selecting `g`, move the robot by hand and select an action:

| Key | Manual-guidance action |
|---|---|
| `s` | Start the phase sequence from the current guided pose. |
| `h` | Start Cartesian pose hold from the current guided pose. |
| `t` | Start setup-impedance hold from the current guided pose. |
| `q` | Save the current joint posture in `params/initial_pose.conf`. |
| `m` | Return to the startup menu. |
| `e` | Stop and print the current joint posture. |

During active control, `e` stops the run, `m` returns to the startup menu, and
`g` enters manual guidance from the current pose.

## Parameter files

Selector comments use one consistent form:

```text
# Selector: 0 = first behaviour; 1 = second behaviour.
```

Multi-value selectors list every accepted entry and its effect directly above
the parameter. For the virtual center of compliance, enable
`use_virtual_compliance_center` and select exactly one center definition in
`setup.conf`: a tool-frame center or a surface-frame lever.

| File | Contents |
|---|---|
| `run_settings.conf` | Robot address, run duration, logging, and terminal period. |
| `safety.conf` | Collision and reflex thresholds. |
| `gripper.conf` | Gripper dimensions, force, speed, and tool-transfer posture. |
| `surface.conf` | Surface point and surface-frame orientation. |
| `tool_orientation.conf` | Tool axis, target offsets, and rotational constraints. |
| `tool_geometry.conf` | Tool-face dimensions and contact-feature selection. |
| `initial_pose.conf` | Initial joint configurations. |
| `approach.conf` | Tool orientation and surface-approach settings. |
| `phase_gates.conf` | Surface clearance and phase-transition holds. |
| `setup.conf` | Setup preload, impedance, and virtual compliance center. |
| `grinding.conf` | Grinding direction, amplitude, and frequency. |
| `nullspace.conf` | Nullspace damping and singular-value conditioning. |
| `disturbance.conf` | Disturbance-test timing and force settings. |
| `hold.conf` | Cartesian pose-hold impedance. |
| `auto_damping.conf` | Automatic-damping limits. |
| `guidance.conf` | Manual-guidance damping. |

The program reads the files from the `params/` directory located beside the
executable. Parameter files are loaded when the program starts and before each
new session selected from the startup menu. Editing a `.conf` file does not
require recompilation. Missing parameter files stop the program with the exact
file paths instead of applying compiled defaults.

Distances are specified in metres, linear velocities in metres per second,
forces in newtons, torques in newton metres, joint angles in radians, and
orientation offsets in degrees where stated.
