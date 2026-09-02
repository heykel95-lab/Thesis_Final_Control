// ============================================================================
// Core controller types
// ============================================================================
// Defines the common mathematical aliases, libfranka aliases, and controller
// modes used by the configuration, runtime-data, and module-interface headers.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "examples_common.h"

// ----------------------------------------------------------------------------
// Fixed-size mathematical types
// ----------------------------------------------------------------------------
// These aliases define the dimensions used by the 6D Cartesian task and the
// 7-DOF Franka robot. Units depend on the represented physical quantity.

using Array6 = std::array<double, 6>;             // Six scalar values [-].
using Array7 = std::array<double, 7>;             // Seven joint values [-].
using Vec3 = Eigen::Vector3d;                     // Three-axis vector.
using Vec6 = Eigen::Matrix<double, 6, 1>;         // Cartesian vector [3+3].
using Vec7 = Eigen::Matrix<double, 7, 1>;         // Joint-space vector.
using Mat3 = Eigen::Matrix3d;                     // Three-axis matrix.
using Mat4x4 = Eigen::Matrix<double, 4, 4>;       // Homogeneous transform.
using Mat6x6 = Eigen::Matrix<double, 6, 6>;       // Cartesian gain matrix.
using Mat6x7 = Eigen::Matrix<double, 6, 7>;       // Geometric Jacobian.
using Mat7x6 = Eigen::Matrix<double, 7, 6>;       // Transposed task mapping.
using Mat7x7 = Eigen::Matrix<double, 7, 7>;       // Joint-space matrix.

/// Maps existing contiguous memory to an Eigen matrix without copying it.
template <typename MatrixType>
using Map = Eigen::Map<MatrixType>;

// libfranka aliases used throughout the controller interfaces.
using Robot = franka::Robot;
using Gripper = franka::Gripper;
using RobotState = franka::RobotState;
using Model = franka::Model;
using Torques = franka::Torques;
using Duration = franka::Duration;
using Frame = franka::Frame;
using franka::MotionFinished;

// ----------------------------------------------------------------------------
// Controller modes
// ----------------------------------------------------------------------------

/// Selects the active state or standalone controller mode.
enum class ControlState {
  kToolOrientation = 0,  // Rotates the tool toward the configured target attitude.
  kSurfaceApproach = 1,  // Moves the selected tool feature toward the surface [m].
  kPreContactHold = 6,   // Holds at clearance until the operator continues.
  kContactEstablishment = 2, // Applies contact penetration and alignment impedance.
  kPreGrindingHold = 7,  // Holds established contact until the operator continues.
  kGrinding = 3,         // Maintains contact and executes the grinding trajectory.
  kPoseHold = 4,         // Holds the captured Cartesian pose with hold gains.
  kManualGuidance = 5    // Applies joint damping for hand-guided positioning.
};

/// Selects the nullspace torque contributions.
enum class NullspaceMode {
  kOff = 0,              // Disables the nullspace torque.
  kDampingOnly = 1,      // Applies projected joint-velocity damping.
  kSigmaOnly = 2,        // Applies singular-value conditioning torque.
  kDampingAndSigma = 3   // Applies damping and conditioning together.
};
