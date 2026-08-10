// ============================================================================
// Grinding-tool axis calibration tool
// ============================================================================
// Defines the guided grinding-tool-axis calibration procedure.
#include "controller_api.h"

#include <cerrno>
#include <poll.h>
#include <unistd.h>

namespace {

// Defining gentle Cartesian damping used to discourage translation and tilt.
constexpr double kCalibrationLinearDamping = 20.0;   // [N s/m]
constexpr double kCalibrationTiltDamping = 3.0;      // [N m s/rad]

// Converting the angle between two unit directions to degrees [deg].
double angleBetweenUnitVectorsDeg(const Vec3& first, const Vec3& second) {
  const double dot = std::max(-1.0, std::min(1.0, first.dot(second)));
  return (180.0 / M_PI) * std::acos(dot);
}

// Calculating the magnitude of the relative orientation between samples [deg].
double orientationSeparationDeg(const Mat3& first, const Mat3& second) {
  const Eigen::AngleAxisd relative(first.transpose() * second);
  return (180.0 / M_PI) * std::abs(relative.angle());
}

// Running active hand guidance until Enter and returning the captured orientation.
Mat3 guideAndCaptureOrientation(Robot& robot,
                                const Model& model,
                                double joint_damping,
                                const Vec3& surface_normal_base,
                                bool damp_non_yaw_motion,
                                const char* label,
                                const char* instruction) {
  // Explaining the requested hand-guided movement before torque control starts.
  printf("\n%s\n", label);
  printf("%s\n", instruction);
  printf("Active gravity-compensated guidance is starting.\n");
  printf("Move the arm by hand; when stationary, press Enter to record.\n");
  fflush(stdout);

  // Polling terminal input outside the real-time control callback.
  std::atomic<bool> capture_requested(false);
  std::atomic<bool> cancel_input(false);
  std::atomic<bool> input_failed(false);
  std::thread input_thread([&]() {
    while (!cancel_input.load()) {
      pollfd terminal{};
      terminal.fd = STDIN_FILENO;
      terminal.events = POLLIN | POLLHUP;
      const int poll_result = poll(&terminal, 1, 50);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        input_failed.store(true);
        capture_requested.store(true);
        return;
      }
      if (poll_result > 0 &&
          (terminal.revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL))) {
        std::string input;
        if (!std::getline(std::cin, input)) {
          input_failed.store(true);
        }
        capture_requested.store(true);
        return;
      }
    }
  });

  // Initializing the captured state and joint-limit abort indicator.
  RobotState captured_state = robot.readOnce();
  std::atomic<int> joint_limit_abort(0);
  // Defining the permitted yaw axis from the calibrated surface normal [-].
  const Vec3 yaw_axis = surface_normal_base.normalized();

  try {
    robot.control([&](const RobotState& state, Duration /*period*/) -> Torques {
      const Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());
      Map<const Vec7> dq(state.dq.data());
      // Calculating the commanded joint torque from Coriolis compensation
      // and viscous joint damping [N m].
      Vec7 commanded_torque = coriolis - joint_damping * dq;

      if (damp_non_yaw_motion) {
        // Damping Cartesian translation and rotation orthogonal to the surface normal.
        const std::array<double, 42> jacobian_array =
            model.zeroJacobian(Frame::kEndEffector, state);
        Map<const Mat6x7> jacobian(jacobian_array.data());
        const Vec6 velocity = jacobian * dq;
        const Vec3 angular_velocity = velocity.tail<3>();
        const Vec3 non_yaw_angular_velocity =
            angular_velocity - yaw_axis * yaw_axis.dot(angular_velocity);

        Vec6 damping_wrench = Vec6::Zero();
        damping_wrench.head<3>() =
            -kCalibrationLinearDamping * velocity.head<3>();
        damping_wrench.tail<3>() =
            -kCalibrationTiltDamping * non_yaw_angular_velocity;
        // Transforming the Cartesian damping wrench to joint torque [N m].
        commanded_torque += jacobian.transpose() * damping_wrench;
      }

      // Ending guidance before a configured joint-limit margin is crossed.
      int joint_out = 0;
      if (!withinJointLimits(state.q, joint_out)) {
        joint_limit_abort.store(joint_out);
        captured_state = state;
        return MotionFinished(Torques(vec7ToArray(commanded_torque)));
      }

      if (capture_requested.load()) {
        captured_state = state;
        return MotionFinished(Torques(vec7ToArray(commanded_torque)));
      }
      return Torques(vec7ToArray(commanded_torque));
    });
  } catch (...) {
    cancel_input.store(true);
    input_thread.join();
    throw;
  }

  // Stopping and joining the terminal poller after control has ended.
  cancel_input.store(true);
  input_thread.join();
  if (input_failed.load()) {
    throw std::runtime_error(
        "Calibration input ended before all samples were recorded.");
  }
  if (joint_limit_abort.load() != 0) {
    throw std::runtime_error(
        "Guidance stopped near the limit of joint " +
        std::to_string(joint_limit_abort.load()) + ". Reposition the arm first.");
  }

  // Returning the orientation captured in the final control callback.
  Map<const Mat4x4> T_EE(captured_state.O_T_EE.data());

  // Returning the EE orientation expressed in the robot base frame [-].
  return T_EE.block<3, 3>(0, 0);
}

// Estimating the EE-frame axis that has the same base direction in T1--T3.
Vec3 estimateInvariantAxis(const std::array<Mat3, 3>& samples,
                           const Vec3& nominal_axis_ee) {
  // Building the least-squares matrix for pairwise orientation differences [-].
  Mat3 normal_matrix = Mat3::Zero();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    for (std::size_t j = i + 1; j < samples.size(); ++j) {
      const Mat3 difference = samples[i] - samples[j];
      normal_matrix += difference.transpose() * difference;
    }
  }

  // Selecting the unit eigenvector with the smallest residual [-].
  Eigen::SelfAdjointEigenSolver<Mat3> solver(normal_matrix);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Tool-axis eigenvalue decomposition failed.");
  }
  Vec3 axis_ee = solver.eigenvectors().col(0).normalized();

  // Selecting the sign that remains closest to the configured nominal axis [-].
  Vec3 reference_axis = nominal_axis_ee;
  if (reference_axis.norm() < 1e-9) {
    reference_axis = Vec3(0.0, 0.0, 1.0);
  }
  reference_axis.normalize();
  if (axis_ee.dot(reference_axis) < 0.0) {
    axis_ee = -axis_ee;
  }

  return axis_ee;
}

}  // namespace

int main() {
  try {
    // Loading the robot address, calibrated plane, and nominal tool axis.
    const ControllerConfig config =
        readControllerConfig(parameterFiles());

    // Normalizing the calibrated surface normal in the robot base frame [-].
    Vec3 surface_normal_base = config.surface_normal_base;
    surface_normal_base.normalize();

    // Selecting the expected tool-axis direction while the face is flat [-].
    const double target_sign =
        config.tool_axis_target_sign >= 0.0 ? 1.0 : -1.0;
    const Vec3 expected_axis_base = target_sign * surface_normal_base;

    // Printing the calibration procedure and the active calibrated plane.
    printf("TOOL AXIS CALIBRATION\n");
    printf("No automatic robot motion or calibration preload is commanded.\n");
    printf("Keep the grinder off and hand-guide only while prompted.\n");
    printf("Use the calibrated workpiece plane from params/surface.conf.\n\n");
    printf("surface point [m] = [%+.9f, %+.9f, %+.9f]\n",
           config.surface_point(0),
           config.surface_point(1),
           config.surface_point(2));
    printf("surface normal    = [%+.9f, %+.9f, %+.9f]\n",
           surface_normal_base(0),
           surface_normal_base(1),
           surface_normal_base(2));
    printf("Connecting to robot: %s\n", config.robot_ip.c_str());

    // Connecting once and preparing the same active guidance used by mode g.
    Robot robot(config.robot_ip);
    robot.automaticErrorRecovery();
    configureCollisionBehavior(robot, config);
    Model model = robot.loadModel();

    // Capturing T1 with free gravity-compensated manual guidance.
    std::array<Mat3, 3> calibration_samples;
    calibration_samples[0] = guideAndCaptureOrientation(
        robot, model, config.manual_guidance_damping, surface_normal_base,
        false, "T1 - calibration sample 1",
        "Seat the complete tool face lightly and "
        "flat on the plane.");

    printf("\nYaw-assisted guidance enabled for T2, T3, and T4.\n");
    printf("Translation and tilt are damped; yaw about the surface normal is free.\n");
    printf("Do not force the robot if motion is blocked.\n");

    // Capturing two well-separated yaw orientations for axis estimation.
    calibration_samples[1] = guideAndCaptureOrientation(
        robot, model, config.manual_guidance_damping, surface_normal_base,
        true, "T2 - calibration sample 2",
        "Rotate about +30 deg in yaw "
        "while keeping the face flat.");
    calibration_samples[2] = guideAndCaptureOrientation(
        robot, model, config.manual_guidance_damping, surface_normal_base,
        true, "T3 - calibration sample 3",
        "Rotate to about -30 deg from T1 so this sample is well separated "
        "from T2.");

    // Rejecting repeated or weakly separated samples before solving the axis.
    const double separation_t1_t2 = orientationSeparationDeg(
        calibration_samples[0], calibration_samples[1]);
    const double separation_t1_t3 = orientationSeparationDeg(
        calibration_samples[0], calibration_samples[2]);
    const double separation_t2_t3 = orientationSeparationDeg(
        calibration_samples[1], calibration_samples[2]);
    printf("\nRecorded orientation separation:\n");
    printf("T1-T2 %.2f deg | T1-T3 %.2f deg | T2-T3 %.2f deg\n",
           separation_t1_t2, separation_t1_t3, separation_t2_t3);
    if (std::min(separation_t1_t2,
                 std::min(separation_t1_t3, separation_t2_t3)) < 15.0) {
      throw std::runtime_error(
          "T1-T3 yaw samples must be separated by at least 15 deg.");
    }

    // Estimating the physical tool axis in end-effector coordinates [-].
    const Vec3 calibrated_axis_ee =
        estimateInvariantAxis(calibration_samples, config.tool_axis_ee);

    // Transforming T1--T3 estimates to the base frame and averaging them [-].
    Vec3 mean_axis_base = Vec3::Zero();
    std::array<Vec3, 3> calibrated_axes_base;
    for (std::size_t i = 0; i < calibration_samples.size(); ++i) {
      calibrated_axes_base[i] =
          (calibration_samples[i] * calibrated_axis_ee).normalized();
      mean_axis_base += calibrated_axes_base[i];
    }
    if (mean_axis_base.norm() < 1e-9) {
      throw std::runtime_error("Calibration samples do not define a stable tool axis.");
    }
    mean_axis_base.normalize();

    // Calculating the largest T1--T3 deviation from the estimated axis [deg].
    double calibration_spread_deg = 0.0;
    for (const Vec3& axis_base : calibrated_axes_base) {
      calibration_spread_deg = std::max(
          calibration_spread_deg,
          angleBetweenUnitVectorsDeg(axis_base, mean_axis_base));
    }

    // Capturing an independent fourth orientation for validation.
    const Mat3 validation_orientation = guideAndCaptureOrientation(
        robot, model, config.manual_guidance_damping, surface_normal_base,
        true, "T4 - validation sample",
        "Choose another distinct yaw angle, such as +50 deg from T1.");

    // Requiring T4 to validate the solution at a genuinely new orientation.
    const double validation_separation = std::min(
        orientationSeparationDeg(validation_orientation, calibration_samples[0]),
        std::min(
            orientationSeparationDeg(validation_orientation,
                                     calibration_samples[1]),
            orientationSeparationDeg(validation_orientation,
                                     calibration_samples[2])));
    printf("T4 nearest-sample separation = %.2f deg\n",
           validation_separation);
    if (validation_separation < 15.0) {
      throw std::runtime_error(
          "T4 must be at least 15 deg from every calibration sample.");
    }

    // Transforming the calibrated axis through T4 for validation [-].
    const Vec3 validation_axis_base =
        (validation_orientation * calibrated_axis_ee).normalized();

    // Calculating repeatability and plane-consistency errors [deg].
    const double validation_error_deg =
        angleBetweenUnitVectorsDeg(validation_axis_base, mean_axis_base);
    const double plane_consistency_error_deg =
        angleBetweenUnitVectorsDeg(mean_axis_base, expected_axis_base);

    // Reporting the calibrated EE-frame axis and validation results.
    printf("\nCALIBRATION RESULT\n");
    printf("tool axis in EE frame = [%+.9f, %+.9f, %+.9f]\n",
           calibrated_axis_ee(0),
           calibrated_axis_ee(1),
           calibrated_axis_ee(2));
    printf("T1-T3 axis spread     = %.4f deg\n", calibration_spread_deg);
    printf("T4 validation error   = %.4f deg\n", validation_error_deg);
    printf("surface consistency   = %.4f deg\n", plane_consistency_error_deg);

    // Printing values ready for params/tool_orientation.conf.
    printf("\nCopy the calibrated values to params/tool_orientation.conf:\n");
    printf("tool_axis_ee_x = %.9f\n", calibrated_axis_ee(0));
    printf("tool_axis_ee_y = %.9f\n", calibrated_axis_ee(1));
    printf("tool_axis_ee_z = %.9f\n", calibrated_axis_ee(2));

    printf("\nA small T1-T3 spread and T4 error indicate repeatable axis "
           "calibration.\n");
    printf("The surface-consistency error checks the calibrated axis against "
           "the plane normal.\n");

    return 0;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
