// ============================================================================
// Automatic grinding-tool axis calibration tool
// ============================================================================
// Defines the unattended grinding-tool-axis calibration procedure. The robot
// seats the tool face on the calibrated plane at four yaw orientations. Tilt
// stiffness is released once contact is established, so each resting attitude
// follows from contact alone and not from the configured tool axis. Solving the
// end-effector direction that the four seatings share then gives the axis.
#include "controller_api.h"

#include <cerrno>
#include <poll.h>
#include <unistd.h>

namespace {

// Defining the number of seatings and the subset used for the axis solution.
constexpr std::size_t kSampleCount = 4;
constexpr std::size_t kEstimationSampleCount = 3;

// Defining the commanded yaw about the surface normal between seatings [deg].
constexpr double kInitialYawDeg = -45.0;
constexpr double kSampleYawStepDeg = 30.0;
constexpr double kYawRateDeg = 15.0;              // Commanded yaw rate [deg/s].
constexpr double kYawSettledRad = 0.010;          // Reached-yaw threshold [rad].

// Defining the transport impedance used while the tool is clear of the plane.
constexpr double kTransportStiffness = 1200.0;    // [N/m]
constexpr double kTransportTiltStiffness = 60.0;  // [N m/rad]

// Defining the seating impedance. Tangential stiffness stays low so the face
// can slide the small distance that pivoting onto the plane requires.
constexpr double kSeatNormalStiffness = 2500.0;      // [N/m]
constexpr double kSeatTangentialStiffness = 300.0;   // [N/m]
constexpr double kYawStiffness = 25.0;               // [N m/rad]

// Defining damping. Rotational damping stays active on every axis, so the
// released face settles onto the plane instead of oscillating.
constexpr double kNormalDamping = 120.0;   // [N s/m]
constexpr double kTangentialDamping = 40.0;  // [N s/m]
constexpr double kTiltDamping = 4.0;       // [N m s/rad]

// Defining the transport geometry and speeds.
constexpr double kLiftClearance = 0.025;      // Face height before yawing [m].
constexpr double kLiftSpeed = 0.020;          // Retraction speed [m/s].
constexpr double kLiftMaxTravel = 0.080;      // Retraction travel limit [m].
constexpr double kDescendSpeed = 0.004;       // Approach speed [m/s].
constexpr double kDescendMaxTravel = 0.060;   // Approach travel limit [m].

// Defining the seating press and the rest condition evaluated afterwards.
constexpr double kContactForce = 5.0;         // Contact-detection threshold [N].
constexpr double kSeatingForce = 40.0;        // Commanded seating force [N].
constexpr double kSeatSpeed = 0.004;          // Virtual penetration rate [m/s].
constexpr double kSeatMaxPush = 0.030;        // Virtual penetration limit [m].
constexpr double kSettleQuietRate = 0.004;    // Rest angular rate [rad/s].
constexpr double kSettleQuietTime = 0.75;     // Required quiet duration [s].
constexpr double kSettleTimeout = 6.0;        // Settling timeout [s].

// Defining the height band accepted for the tool face at program start [m].
constexpr double kStartHeightMin = 0.002;
constexpr double kStartHeightMax = 0.150;

/// Selects the active stage of one seating cycle.
enum class SeatingStage {
  kLift,     // Retracting the face along the surface normal.
  kYaw,      // Rotating the commanded orientation about the surface normal.
  kDescend,  // Lowering the face until the contact force is detected.
  kSeat,     // Releasing the tilt stiffness and ramping the seating force.
  kSettle,   // Waiting for the released face to come to rest.
  kDone      // All seatings recorded.
};

/// Stores what one seating measured, for the per-sample result table.
struct SeatingRecord {
  Mat3 R_EE = Mat3::Identity();      // Seated end-effector orientation [-].
  double seating_force = 0.0;        // Contact-force change at capture [N].
  double tilt_moment = 0.0;          // Contact moment across the plane [N m].
  double released_tilt_deg = 0.0;    // Face rotation after releasing tilt [deg].
  double settle_time = 0.0;          // Time from release to rest [s].
  bool settled = false;              // Indicates the rest condition was met.
};

// Returning the terminal label for one seating stage.
const char* stageName(SeatingStage stage) {
  switch (stage) {
    case SeatingStage::kLift: return "lift";
    case SeatingStage::kYaw: return "yaw";
    case SeatingStage::kDescend: return "descend";
    case SeatingStage::kSeat: return "seat";
    case SeatingStage::kSettle: return "settle";
    case SeatingStage::kDone: return "done";
  }
  return "unknown";
}

// Building a base-frame gain from its diagonal in the surface frame.
Mat3 surfaceGain(const Mat3& R_base_surface, const Vec3& diagonal) {
  return R_base_surface * diagonal.asDiagonal() * R_base_surface.transpose();
}

// Starting the terminal poller that requests an abort on any keypress.
void startAbortThread(std::atomic<bool>& abort_requested,
                      std::atomic<bool>& cancel_input,
                      std::thread& input_thread) {
  input_thread = std::thread([&abort_requested, &cancel_input]() {
    while (!cancel_input.load()) {
      pollfd terminal{};
      terminal.fd = STDIN_FILENO;
      terminal.events = POLLIN | POLLHUP;
      const int poll_result = poll(&terminal, 1, 50);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        return;
      }
      if (poll_result > 0 &&
          (terminal.revents & (POLLIN | POLLHUP | POLLERR | POLLNVAL))) {
        std::string input;
        std::getline(std::cin, input);
        abort_requested.store(true);
        return;
      }
    }
  });
}

}  // namespace

int main() {
  try {
    // Loading the robot address, calibrated plane, and nominal tool axis.
    const ControllerConfig config = readControllerConfig(parameterFiles());

    // Building the surface frame [tangent1, tangent2, normal] in base [-].
    const Mat3 R_base_surface = makeSurfaceFrame(config);
    const Vec3 surface_normal = R_base_surface.col(2);

    // Selecting the expected tool-axis direction while the face is flat [-].
    const double target_sign =
        config.tool_axis_target_sign >= 0.0 ? 1.0 : -1.0;
    const Vec3 expected_axis_base = target_sign * surface_normal;

    // Printing the calibration procedure and the active calibrated plane.
    printf("AUTOMATIC TOOL AXIS CALIBRATION\n");
    printf("The robot moves on its own. Keep the working area clear.\n");
    printf("Keep the grinder off and keep the payload configured.\n");
    printf("Start with the tool face above the plane and roughly parallel.\n");
    printf("Press Enter at any time to stop the procedure.\n");
    printf("Use the calibrated workpiece plane from params/surface.conf.\n\n");
    printf("surface point [m] = [%+.9f, %+.9f, %+.9f]\n",
           config.surface_point(0),
           config.surface_point(1),
           config.surface_point(2));
    printf("surface normal    = [%+.9f, %+.9f, %+.9f]\n",
           surface_normal(0), surface_normal(1), surface_normal(2));
    printf("Connecting to robot: %s\n", config.robot_ip.c_str());

    // Connecting once and preparing the same protection used by the controller.
    Robot robot(config.robot_ip);
    robot.automaticErrorRecovery();
    configureCollisionBehavior(robot, config);
    Model model = robot.loadModel();

    // Reading the start state and mapping the initial pose.
    const RobotState start_state = robot.readOnce();
    Map<const Mat4x4> T_start(start_state.O_T_EE.data());
    const Mat3 R_start = T_start.block<3, 3>(0, 0);
    const Vec3 p_start = T_start.block<3, 1>(0, 3);

    // Calculating the start height of the contact-face center above the plane [m].
    const Vec3 face_center_start =
        p_start + R_start * config.tool_contact_face_center_ee;
    const double start_height =
        surface_normal.dot(face_center_start - config.surface_point);
    printf("face height at start = %.1f mm\n\n", 1000.0 * start_height);
    if (start_height < kStartHeightMin || start_height > kStartHeightMax) {
      throw std::runtime_error(
          "Position the tool face between " +
          std::to_string(static_cast<int>(1000.0 * kStartHeightMin)) + " and " +
          std::to_string(static_cast<int>(1000.0 * kStartHeightMax)) +
          " mm above the plane before starting.");
    }

    // Building the transport and seating impedance in base coordinates.
    const Mat3 Kp_transport = kTransportStiffness * Mat3::Identity();
    const Mat3 Kp_seat = surfaceGain(
        R_base_surface, Vec3(kSeatTangentialStiffness,
                             kSeatTangentialStiffness, kSeatNormalStiffness));
    const Mat3 Dp = surfaceGain(
        R_base_surface,
        Vec3(kTangentialDamping, kTangentialDamping, kNormalDamping));
    const Mat3 KR_transport = kTransportTiltStiffness * Mat3::Identity();
    // Constraining rotation about the surface normal only, so tilt stays free.
    const Mat3 KR_seat =
        kYawStiffness * surface_normal * surface_normal.transpose();
    const Mat3 DR = kTiltDamping * Mat3::Identity();

    // Starting the terminal poller that requests an abort.
    std::atomic<bool> abort_requested(false);
    std::atomic<bool> cancel_input(false);
    std::thread input_thread;
    startAbortThread(abort_requested, cancel_input, input_thread);

    // Initializing the seating sequence and its commanded reference pose.
    std::vector<SeatingRecord> records;
    records.reserve(kSampleCount);
    SeatingStage stage = SeatingStage::kLift;
    std::size_t sample_index = 0;
    Vec3 p_d = p_start;
    Mat3 R_d = R_start;

    // Initializing the per-stage references captured at each stage entry.
    double time = 0.0;
    double stage_start_time = 0.0;
    Vec3 stage_start_p = p_start;
    Mat3 stage_start_R = R_start;
    Vec3 seat_contact_p = p_start;
    Mat3 seat_release_R = R_start;
    double commanded_yaw = 0.0;
    double seat_push = 0.0;
    double quiet_time = 0.0;

    // Initializing the free-space wrench baseline captured before each descent.
    Vec3 force_bias = Vec3::Zero();
    Vec3 moment_bias = Vec3::Zero();

    // Initializing the abort reasons evaluated after the control loop ends.
    std::atomic<int> joint_limit_abort(0);
    bool lift_failed = false;
    bool descend_failed = false;

    printf("Starting the seating sequence with %zu orientations.\n",
           kSampleCount);
    fflush(stdout);

    robot.control([&](const RobotState& state, Duration period) -> Torques {
      time += period.toSec();

      // Mapping the measured pose, velocity, and estimated external wrench.
      Map<const Mat4x4> T_EE(state.O_T_EE.data());
      const Mat3 R_EE = T_EE.block<3, 3>(0, 0);
      const Vec3 p_EE = T_EE.block<3, 1>(0, 3);
      const std::array<double, 42> jacobian_array =
          model.zeroJacobian(Frame::kEndEffector, state);
      Map<const Mat6x7> J(jacobian_array.data());
      Map<const Vec7> dq(state.dq.data());
      const Vec6 velocity = J * dq;
      const Vec3 pdot = velocity.head<3>();
      const Vec3 omega = velocity.tail<3>();
      Map<const Vec6> external_wrench(state.O_F_ext_hat_K.data());
      const Vec3 external_force = external_wrench.head<3>();
      const Vec3 external_moment = external_wrench.tail<3>();

      // Calculating the contact-face center [m] and its height above the plane [m].
      const Vec3 face_center =
          p_EE + R_EE * config.tool_contact_face_center_ee;
      const double face_height =
          surface_normal.dot(face_center - config.surface_point);

      // Calculating the contact-force change [N] used to detect and rate contact.
      const Vec3 contact_force = external_force - force_bias;
      const double contact_force_norm = contact_force.norm();

      // Selecting the impedance of the active stage.
      const bool seating =
          stage == SeatingStage::kSeat || stage == SeatingStage::kSettle;
      const Mat3& Kp = seating ? Kp_seat : Kp_transport;
      const Mat3& KR = seating ? KR_seat : KR_transport;

      // Initializing the commanded velocity of the active stage [m/s].
      Vec3 pdot_d = Vec3::Zero();

      switch (stage) {
        // -----------------------------------------------------------
        // Retracting the face to the clearance height. The reference
        // orientation is reset to the measured attitude, so restoring
        // the tilt stiffness after a seating causes no torque step.
        // -----------------------------------------------------------
        case SeatingStage::kLift: {
          const double stage_time = time - stage_start_time;
          const double travel =
              std::min(kLiftSpeed * stage_time, kLiftMaxTravel);
          p_d = stage_start_p + travel * surface_normal;
          pdot_d = kLiftSpeed * surface_normal;
          R_d = stage_start_R;

          if (face_height >= kLiftClearance) {
            stage = SeatingStage::kYaw;
            stage_start_time = time;
            stage_start_p = p_d;
            stage_start_R = R_EE;
            commanded_yaw = (sample_index == 0)
                                ? (M_PI / 180.0) * kInitialYawDeg
                                : (M_PI / 180.0) * kSampleYawStepDeg;
          } else if (travel >= kLiftMaxTravel) {
            lift_failed = true;
          }
          break;
        }

        // -----------------------------------------------------------
        // Rotating the reference about the surface normal at a bounded
        // rate, then waiting until the measured yaw has followed.
        // -----------------------------------------------------------
        case SeatingStage::kYaw: {
          const double stage_time = time - stage_start_time;
          const double rate = (M_PI / 180.0) * kYawRateDeg;
          const double travelled =
              std::min(rate * stage_time, std::abs(commanded_yaw));
          const double yaw =
              commanded_yaw >= 0.0 ? travelled : -travelled;
          R_d = Eigen::AngleAxisd(yaw, surface_normal).toRotationMatrix() *
                stage_start_R;
          p_d = stage_start_p;

          const bool reached = travelled >= std::abs(commanded_yaw) - 1e-12;
          if (reached &&
              orientationError(R_EE, R_d).norm() <= kYawSettledRad) {
            // Capturing the free-space wrench baseline before descending.
            force_bias = external_force;
            moment_bias = external_moment;
            stage = SeatingStage::kDescend;
            stage_start_time = time;
            stage_start_p = p_d;
          }
          break;
        }

        // -----------------------------------------------------------
        // Lowering the face along the surface normal until the
        // estimated contact force rises above the detection threshold.
        // -----------------------------------------------------------
        case SeatingStage::kDescend: {
          const double stage_time = time - stage_start_time;
          const double travel =
              std::min(kDescendSpeed * stage_time, kDescendMaxTravel);
          p_d = stage_start_p - travel * surface_normal;
          pdot_d = -kDescendSpeed * surface_normal;

          if (contact_force_norm >= kContactForce) {
            stage = SeatingStage::kSeat;
            stage_start_time = time;
            seat_contact_p = p_d;
            seat_release_R = R_EE;
            seat_push = 0.0;
            printf("\nT%zu contact at %.1f mm face height | %.1f N\n",
                   sample_index + 1, 1000.0 * face_height, contact_force_norm);
          } else if (travel >= kDescendMaxTravel) {
            descend_failed = true;
          }
          break;
        }

        // -----------------------------------------------------------
        // Pressing the face onto the plane with the tilt stiffness
        // released. The ramp stops at the commanded seating force, so
        // contact alone decides the resting attitude of the face.
        // -----------------------------------------------------------
        case SeatingStage::kSeat: {
          const double stage_time = time - stage_start_time;
          if (contact_force_norm < kSeatingForce) {
            seat_push = std::min(kSeatSpeed * stage_time, kSeatMaxPush);
            pdot_d = -kSeatSpeed * surface_normal;
          }
          p_d = seat_contact_p - seat_push * surface_normal;

          if (contact_force_norm >= kSeatingForce ||
              seat_push >= kSeatMaxPush) {
            stage = SeatingStage::kSettle;
            stage_start_time = time;
            quiet_time = 0.0;
          }
          break;
        }

        // -----------------------------------------------------------
        // Holding the seating force until the face has come to rest,
        // then recording the orientation that contact has selected.
        // -----------------------------------------------------------
        case SeatingStage::kSettle: {
          const double stage_time = time - stage_start_time;
          p_d = seat_contact_p - seat_push * surface_normal;

          // Accumulating the time the angular rate stays below the threshold.
          if (omega.norm() <= kSettleQuietRate) {
            quiet_time += period.toSec();
          } else {
            quiet_time = 0.0;
          }

          const bool settled = quiet_time >= kSettleQuietTime;
          if (settled || stage_time >= kSettleTimeout) {
            // Transferring the moment from the TCP to the contact face [N m]:
            // M_contact = M_TCP + r_contact x f, r_contact = p_EE - p_contact.
            const Vec3 r_contact = p_EE - face_center;
            const Vec3 contact_moment = (external_moment - moment_bias) +
                                        r_contact.cross(contact_force);
            // Removing the component about the normal, which the yaw
            // stiffness carries and which says nothing about seating.
            const Vec3 tilt_moment =
                contact_moment -
                surface_normal * surface_normal.dot(contact_moment);

            SeatingRecord record;
            record.R_EE = R_EE;
            record.seating_force = contact_force_norm;
            record.tilt_moment = tilt_moment.norm();
            record.released_tilt_deg =
                orientationSeparationDeg(seat_release_R, R_EE);
            record.settle_time = stage_time;
            record.settled = settled;
            records.push_back(record);

            printf("T%zu seated | %5.1f N | tilt moment %5.2f N m | "
                   "released tilt %5.2f deg | %.2f s%s\n",
                   sample_index + 1, record.seating_force, record.tilt_moment,
                   record.released_tilt_deg, record.settle_time,
                   settled ? "" : " (timeout)");

            ++sample_index;
            if (sample_index >= kSampleCount) {
              stage = SeatingStage::kDone;
            } else {
              stage = SeatingStage::kLift;
              stage_start_time = time;
              stage_start_p = p_d;
              stage_start_R = R_EE;
            }
          }
          break;
        }

        case SeatingStage::kDone:
          p_d = p_EE;
          R_d = R_EE;
          break;
      }

      // Calculating the pose error [m, rad] and velocity error [m/s, rad/s].
      Vec6 dx;
      dx.head<3>() = p_d - p_EE;
      dx.tail<3>() = orientationError(R_EE, R_d);
      Vec6 dv;
      dv.head<3>() = pdot_d - pdot;
      dv.tail<3>() = -omega;

      // Calculating the decoupled Cartesian wrench at the TCP [N, N m]. No
      // compliance center is used, so no lever enters the commanded wrench.
      Vec6 wrench;
      wrench.head<3>() = Kp * dx.head<3>() + Dp * dv.head<3>();
      wrench.tail<3>() = KR * dx.tail<3>() + DR * dv.tail<3>();

      // Summing task, nullspace, and Coriolis torque [N m].
      const Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());
      const Vec7 tau_nullspace =
          computeNullspaceTorque(config, model, state, J, dq);
      const Vec7 commanded_torque =
          J.transpose() * wrench + tau_nullspace + coriolis;

      // Ending the procedure before a configured joint-limit margin is crossed.
      int joint_out = 0;
      if (!withinJointLimits(state.q, joint_out)) {
        joint_limit_abort.store(joint_out);
        return MotionFinished(Torques(vec7ToArray(commanded_torque)));
      }

      if (stage == SeatingStage::kDone || abort_requested.load() ||
          lift_failed || descend_failed) {
        return MotionFinished(Torques(vec7ToArray(commanded_torque)));
      }
      return Torques(vec7ToArray(commanded_torque));
    });

    // Stopping and joining the terminal poller after control has ended.
    cancel_input.store(true);
    if (input_thread.joinable()) {
      input_thread.join();
    }

    // Reporting the condition that ended the sequence before the solution.
    if (joint_limit_abort.load() != 0) {
      throw std::runtime_error(
          "Motion stopped near the limit of joint " +
          std::to_string(joint_limit_abort.load()) +
          ". Reposition the arm first.");
    }
    if (lift_failed) {
      throw std::runtime_error(
          "Retraction reached its travel limit without clearing the plane.");
    }
    if (descend_failed) {
      throw std::runtime_error(
          "Approach reached its travel limit without detecting contact.");
    }
    if (records.size() < kSampleCount) {
      throw std::runtime_error(
          "Stopped in the " + std::string(stageName(stage)) + " stage after " +
          std::to_string(records.size()) + " of " +
          std::to_string(kSampleCount) + " seatings.");
    }

    // Collecting the seated orientations used for the axis solution.
    std::vector<Mat3> calibration_samples;
    for (std::size_t i = 0; i < kEstimationSampleCount; ++i) {
      calibration_samples.push_back(records[i].R_EE);
    }
    const Mat3& validation_orientation = records[kSampleCount - 1].R_EE;

    // Rejecting weakly separated seatings before solving the axis.
    double minimum_separation = 360.0;
    printf("\nRecorded orientation separation:\n");
    for (std::size_t i = 0; i < records.size(); ++i) {
      for (std::size_t j = i + 1; j < records.size(); ++j) {
        const double separation =
            orientationSeparationDeg(records[i].R_EE, records[j].R_EE);
        minimum_separation = std::min(minimum_separation, separation);
        printf("T%zu-T%zu %6.2f deg\n", i + 1, j + 1, separation);
      }
    }
    if (minimum_separation < 15.0) {
      throw std::runtime_error(
          "Seated orientations must be separated by at least 15 deg.");
    }

    // Estimating the physical tool axis in end-effector coordinates [-].
    const Vec3 calibrated_axis_ee =
        estimateInvariantAxis(calibration_samples, config.tool_axis_ee);

    // Transforming T1--T3 estimates to the base frame and averaging them [-].
    Vec3 mean_axis_base = Vec3::Zero();
    std::vector<Vec3> calibrated_axes_base;
    for (const Mat3& sample : calibration_samples) {
      calibrated_axes_base.push_back((sample * calibrated_axis_ee).normalized());
      mean_axis_base += calibrated_axes_base.back();
    }
    if (mean_axis_base.norm() < 1e-9) {
      throw std::runtime_error(
          "Seated orientations do not define a stable tool axis.");
    }
    mean_axis_base.normalize();

    // Calculating the largest T1--T3 deviation from the estimated axis [deg].
    double calibration_spread_deg = 0.0;
    for (const Vec3& axis_base : calibrated_axes_base) {
      calibration_spread_deg = std::max(
          calibration_spread_deg,
          angleBetweenUnitVectorsDeg(axis_base, mean_axis_base));
    }

    // Transforming the calibrated axis through T4 for validation [-].
    const Vec3 validation_axis_base =
        (validation_orientation * calibrated_axis_ee).normalized();

    // Calculating repeatability and plane-consistency errors [deg].
    const double validation_error_deg =
        angleBetweenUnitVectorsDeg(validation_axis_base, mean_axis_base);
    const double plane_consistency_error_deg =
        angleBetweenUnitVectorsDeg(mean_axis_base, expected_axis_base);

    // Calculating the correction relative to the configured tool axis [deg].
    Vec3 configured_axis_ee = config.tool_axis_ee;
    if (configured_axis_ee.norm() < 1e-9) {
      configured_axis_ee = Vec3(0.0, 0.0, 1.0);
    }
    configured_axis_ee.normalize();
    const double configured_correction_deg =
        angleBetweenUnitVectorsDeg(calibrated_axis_ee, configured_axis_ee);

    // Reporting the seating quality that supports the solution.
    printf("\nSEATING QUALITY\n");
    double worst_tilt_moment = 0.0;
    for (const SeatingRecord& record : records) {
      worst_tilt_moment = std::max(worst_tilt_moment, record.tilt_moment);
    }
    printf("largest tilt moment   = %.3f N m\n", worst_tilt_moment);
    printf("A small tilt moment indicates the face reached the plane.\n");

    // Reporting the calibrated EE-frame axis and validation results.
    printf("\nCALIBRATION RESULT\n");
    printf("tool axis in EE frame = [%+.9f, %+.9f, %+.9f]\n",
           calibrated_axis_ee(0),
           calibrated_axis_ee(1),
           calibrated_axis_ee(2));
    printf("T1-T3 axis spread     = %.4f deg\n", calibration_spread_deg);
    printf("T4 validation error   = %.4f deg\n", validation_error_deg);
    printf("surface consistency   = %.4f deg\n", plane_consistency_error_deg);
    printf("change from configured axis = %.4f deg\n",
           configured_correction_deg);

    // Printing values ready for params/tool_orientation.conf.
    printf("\nCopy the calibrated values to params/tool_orientation.conf:\n");
    printf("tool_axis_ee_x = %.9f\n", calibrated_axis_ee(0));
    printf("tool_axis_ee_y = %.9f\n", calibrated_axis_ee(1));
    printf("tool_axis_ee_z = %.9f\n", calibrated_axis_ee(2));

    printf("\nA small T1-T3 spread and T4 error indicate a repeatable axis.\n");
    printf("The surface-consistency error checks the calibrated axis against\n"
           "the plane normal. Repeat the procedure after editing the file; the\n"
           "reported change from the configured axis then approaches zero.\n");

    return 0;
  } catch (const franka::Exception& e) {
    // Reporting robot communication errors.
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    // Reporting configuration or numerical errors.
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
