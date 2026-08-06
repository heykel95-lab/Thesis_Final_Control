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
