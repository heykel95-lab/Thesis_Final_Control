// ============================================================================
// Robot support and nullspace control
// ============================================================================
// Defining collision settings, input handling, nullspace torque, joint-limit
// checks, and the configured disturbance model.
#include "controller_api.h"

// ====================================================================
// Robot interaction
// ====================================================================

// Defining Panda joint limits with a safety margin [rad].
bool withinJointLimits(const Array7& q, int& joint_out) {
  // Assigning lower and upper joint limits [rad].
  static const std::array<double, 7> lower = {
      {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
  static const std::array<double, 7> upper = {
      {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}};
  // Defining the joint-limit safety margin [rad].
  const double margin = 0.05;
  for (int i = 0; i < 7; ++i) {
    if (q[i] < lower[i] + margin || q[i] > upper[i] - margin) {
      joint_out = i + 1;
      return false;
    }
  }
  joint_out = 0;
  return true;
}

bool solveStandoffPosture(const Model& model,
                          const RobotState& state,
                          const Array7& q_target,
                          double standoff,
                          Array7& q_standoff) {
  // Calculating the target end-effector pose from q_target [rad].
  const std::array<double, 16> pose_target =
      model.pose(Frame::kEndEffector, q_target, state.F_T_EE, state.EE_T_K);
  Map<const Mat4x4> T_target(pose_target.data());
  const Mat3 R_target = T_target.block<3, 3>(0, 0);
  // Assigning the Cartesian standoff target along -Z_EE [m].
  const Vec3 p_goal =
      T_target.block<3, 1>(0, 3) - standoff * (R_target * Vec3(0.0, 0.0, 1.0));

  // Initializing the iterative joint solution [rad].
  Array7 q = q_target;
  for (int iteration = 0; iteration < 200; ++iteration) {
    const std::array<double, 16> pose =
        model.pose(Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
    Map<const Mat4x4> T(pose.data());
    // Calculating position error [m] and orientation error [rad].
    Vec6 dx;
    dx.head<3>() = p_goal - T.block<3, 1>(0, 3);
    dx.tail<3>() = orientationError(T.block<3, 3>(0, 0), R_target);
    if (dx.head<3>().norm() < 1e-6 && dx.tail<3>().norm() < 1e-6) {
      q_standoff = q;
      return true;
    }

    // Loading the 6x7 end-effector Jacobian for the current iterate.
    const std::array<double, 42> jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, q, state.F_T_EE, state.EE_T_K);
    Map<const Mat6x7> J(jacobian_array.data());
    // Forming the damped least-squares inverse kinematics system.
    Mat6x6 damped = J * J.transpose();
    damped.diagonal().array() += 1e-6;
    const Vec7 dq = J.transpose() * damped.ldlt().solve(dx);
    if (!dq.allFinite()) {
      return false;
    }
    // Limiting each inverse-kinematics update to 0.05 rad in joint norm.
    const double scale = std::min(1.0, 0.05 / std::max(1e-9, dq.norm()));
    for (int i = 0; i < 7; ++i) {
      q[i] += scale * dq(i);
    }
  }
  return false;
}

void startKeyboardStopThread(const ControllerConfig& /*params*/,
                             KeyboardSignals& signals) {
  std::atomic<bool>& stop_requested = signals.stop_requested;
  std::atomic<bool>& proceed_requested = signals.proceed_requested;
  std::atomic<bool>& guide_requested = signals.guide_requested;
  std::atomic<char>& guidance_menu_key = signals.guidance_menu_key;
  std::atomic<bool>& guided_hold_selector_pending =
      signals.guided_hold_selector_pending;
  std::atomic<bool>& gate_continue = signals.gate_continue;
  std::atomic<bool>& menu_requested = signals.menu_requested;

  // Starting the detached thread that parses commands during control.
  std::thread keyboard_thread([&signals, &stop_requested, &proceed_requested,
                               &guide_requested, &guidance_menu_key,
                               &guided_hold_selector_pending,
                               &gate_continue, &menu_requested]() {
    std::string line;
    while (true) {
      // Suspending runtime commands while the startup menu owns standard input.
      while (menu_requested.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      if (!std::getline(std::cin, line)) {
        break;
      }
      // Parsing tool-axis tilt commands t1 and t2 [deg].
      {
        int index = 0;
        double value = 0.0;
        if (std::sscanf(line.c_str(), "t%d %lf", &index, &value) == 2) {
          if ((index == 1 || index == 2) && std::isfinite(value)) {
            signals.setup_tilt_deg_request[index - 1].store(value);
          } else {
            printf("Tilt index must be 1 or 2 and the value finite.\n");
          }
          continue;
        }
      }

      // Parsing setup stiffness and compliance-center commands.
      {
        int index = 0;
        double value = 0.0;
        std::array<std::atomic<double>, 3>* target = nullptr;
        if (std::sscanf(line.c_str(), "kp%d %lf", &index, &value) == 2) {
          target = &signals.setup_kp_request;
        } else if (std::sscanf(line.c_str(), "kr%d %lf", &index, &value) == 2) {
          target = &signals.setup_kr_request;
        } else if (std::sscanf(line.c_str(), "pc%d %lf", &index, &value) == 2) {
          target = &signals.setup_compliance_center_mm_request;
        } else if (std::sscanf(line.c_str(), "r%d %lf", &index, &value) == 2) {
          target = &signals.setup_rc_mm_request;
        }
        if (target != nullptr) {
          if (index >= 1 && index <= 3 && std::isfinite(value)) {
            (*target)[index - 1].store(value);
          } else {
            printf("Index must be 1, 2 or 3 and the value finite.\n");
          }
          continue;
        }
      }

      // Parsing live nullspace gains and the probe step.
      if (line.size() > 1 && (line[0] == 'd' || line[0] == 'D' ||
                              line[0] == 'k' || line[0] == 'K' ||
                              line[0] == 'a' || line[0] == 'A')) {
        double value = 0.0;
        if (std::sscanf(line.c_str() + 1, "%lf", &value) == 1 &&
            std::isfinite(value) && value >= 0.0) {
          if (line[0] == 'd' || line[0] == 'D') {
            signals.nullspace_damping_request.store(value);
          } else if (line[0] == 'k' || line[0] == 'K') {
            signals.nullspace_sigma_gain_request.store(value);
          } else {
            // Storing the probe step [deg] for conversion by the control loop.
            signals.nullspace_probe_step_deg_request.store(value);
          }
          continue;
        }
        printf("Expected 'd <Nms/rad>', 'k <Nm>' or 'a <deg>', value >= 0.\n");
        continue;
      }
      if (line.size() == 1 && line[0] >= '0' && line[0] <= '3') {
        // Selecting the live nullspace mode.
        signals.nullspace_mode_request.store(line[0] - '0');
        continue;
      }
      if (line.empty()) {
        // Continuing the active phase gate.
        gate_continue.store(true);
      } else if (line == "e" || line == "E") {
        stop_requested.store(true);
        break;
      } else if (line == "m" || line == "M") {
        // Ending the current run and transferring input to the startup menu.
        menu_requested.store(true);
        stop_requested.store(true);
      } else if (line == "p" || line == "P") {
        proceed_requested.store(true);
      } else if (line == "g" || line == "G") {
        guide_requested.store(true);
      } else if (line == "q" || line == "Q") {
        if (guided_hold_selector_pending.load()) {
          guidance_menu_key.store('q');
        }
      } else if (line == "s" || line == "S") {
        if (guided_hold_selector_pending.load()) {
          guidance_menu_key.store('s');
        } else {
          signals.run_mode_request.store('s');
        }
      } else if (line == "t" || line == "T") {
        if (guided_hold_selector_pending.load()) {
          guidance_menu_key.store('t');
        } else {
          signals.run_mode_request.store('t');
        }
      } else if (line == "h" || line == "H") {
        guidance_menu_key.store('h');
        // Transferring standard-input ownership to the hold-mode selector.
        while (guided_hold_selector_pending.load() &&
               !stop_requested.load()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (stop_requested.load()) {
          break;
        }
      }
    }
  });
  keyboard_thread.detach();
}

void configureCollisionBehavior(Robot& robot, const ControllerConfig& params) {
  setDefaultBehavior(robot);

  // Selecting configured collision thresholds or the Franka defaults.
  if (!params.use_custom_collision_behavior) {
    printf("Collision: Franka default thresholds (safety.conf is off).\n");
    return;
  }

  // Assigning joint-torque thresholds [N m] and Cartesian thresholds [N, N m].
  const Array7 collision_torque_acc = filledArray7(params.collision_torque_acc);
  const Array7 collision_torque_nom = filledArray7(params.collision_torque_nom);
  const Array6 collision_force_acc = filledArray6(params.collision_force_acc);
  const Array6 collision_force_nom = filledArray6(params.collision_force_nom);

  robot.setCollisionBehavior(
      collision_torque_acc,
      collision_torque_acc,
      collision_torque_nom,
      collision_torque_nom,
      collision_force_acc,
      collision_force_acc,
      collision_force_nom,
      collision_force_nom);
}

Vec7 computeNullspaceTorque(
    const ControllerConfig& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq) {
  // Mapping current joint position [rad].
  Map<const Vec7> q_current(state.q.data());

  // Calculating the SVD of the current 6x7 Jacobian.
  Eigen::JacobiSVD<Mat6x7> svd_current(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix<double, 6, 1> singular_values =
      svd_current.singularValues();
  // Defining the relative singular-value cutoff [-].
  const double sigma_max = singular_values.maxCoeff();
  const double svd_cutoff =
      std::max(0.0, params.nullspace_svd_relative_tolerance) * sigma_max;

  // Constructing the Moore-Penrose pseudoinverse J^+ [rad/m].
  Mat7x6 J_pinv = Mat7x6::Zero();
  for (int i = 0; i < 6; ++i) {
    if (singular_values(i) > svd_cutoff) {
      J_pinv +=
          (1.0 / singular_values(i)) *
          svd_current.matrixV().col(i) *
          svd_current.matrixU().col(i).transpose();
    }
  }

  const Mat7x7 I7 = Mat7x7::Identity();
  // Constructing the torque projector N_tau = (I - J^+ J)^T.
  Mat7x7 N_tau = I7 - J.transpose() * J_pinv.transpose();
  N_tau = 0.5 * (N_tau + N_tau.transpose());
  // Projecting measured joint velocity into the nullspace [rad/s].
  const Vec7 dq_nullspace = N_tau * dq;

  if (params.nullspace_mode == NullspaceMode::kOff) {
    return Vec7::Zero();
  }

  // Calculating nullspace damping torque [N m].
  const bool damping_enabled =
      params.nullspace_mode == NullspaceMode::kDampingOnly ||
      params.nullspace_mode == NullspaceMode::kDampingAndSigma;
  Vec7 tau_damping = Vec7::Zero();
  if (damping_enabled) {
    tau_damping.noalias() =
        -params.nullspace_damping * dq_nullspace;
  }
  if (params.nullspace_mode == NullspaceMode::kDampingOnly) {
    return tau_damping;
  }

  const bool sigma_only =
      params.nullspace_mode == NullspaceMode::kSigmaOnly;
  const Vec7 sigma_fallback =
      sigma_only ? Vec7::Zero() : tau_damping;

  // Selecting and normalizing the one-dimensional nullspace direction.
  Vec7 n = svd_current.matrixV().col(6);
  if (n.norm() <= 1e-9) {
    return sigma_fallback;
  }
  n.normalize();

  // Defining the two-sided joint-space probe step [rad].
  const double alpha = std::abs(params.nullspace_probe_step_rad);
  if (alpha <= 1e-12) {
    return sigma_fallback;
  }

  // Assigning the positive and negative sampled postures [rad].
  const Array7 q_plus_array = vec7ToArray(Vec7(q_current + alpha * n));
  const Array7 q_minus_array = vec7ToArray(Vec7(q_current - alpha * n));

  const std::array<double, 42> J_plus_array =
      model.zeroJacobian(Frame::kEndEffector, q_plus_array,
                         state.F_T_EE, state.EE_T_K);
  const std::array<double, 42> J_minus_array =
      model.zeroJacobian(Frame::kEndEffector, q_minus_array,
                         state.F_T_EE, state.EE_T_K);

  Map<const Mat6x7> J_plus(J_plus_array.data());
  Map<const Mat6x7> J_minus(J_minus_array.data());

  // Evaluating the smallest singular value at both sampled postures [-].
  const double sigma_plus = smallestSingularValue(J_plus);
  const double sigma_minus = smallestSingularValue(J_minus);
  if (!std::isfinite(sigma_plus) || !std::isfinite(sigma_minus)) {
    return sigma_fallback;
  }

  // Comparing the sampled singular values against the configured deadband [-].
  const double sigma_difference = sigma_plus - sigma_minus;
  const double deadband = std::max(0.0, params.nullspace_sigma_deadband);
  if (std::abs(sigma_difference) <= deadband) {
    return sigma_fallback;
  }

  const double sigma_direction = (sigma_difference > 0.0) ? 1.0 : -1.0;
  const Vec7 best_direction = sigma_direction * n;

  // Applying conditioning torque along the better sampled direction [N m].
  const Vec7 tau_sigma =
      params.nullspace_sigma_gain * N_tau * best_direction;

  return sigma_only ? tau_sigma : tau_damping + tau_sigma;
}

namespace {

Frame disturbanceFrame(int link) {
  switch (link) {
    case 1: return Frame::kJoint1;
    case 2: return Frame::kJoint2;
    case 3: return Frame::kJoint3;
    case 4: return Frame::kJoint4;
    case 5: return Frame::kJoint5;
    case 6: return Frame::kJoint6;
    case 7: return Frame::kJoint7;
    default: return Frame::kJoint4;
  }
}

}  // namespace

bool validateAutomaticDisturbance(const ControllerConfig& params,
                                  std::string& error) {
  error.clear();
  if (!params.disturbance_auto_enabled) {
    return true;
  }
  if (params.disturbance_link < 1 || params.disturbance_link > 7) {
    error = "disturbance_link must be an integer from 1 to 7";
  } else if (!params.disturbance_point_link.allFinite()) {
    error = "disturbance_point_link must be finite";
  } else if (!std::isfinite(params.disturbance_force) ||
             params.disturbance_force <= 0.0 ||
             params.disturbance_force > 40.0) {
    error = "disturbance_force must be in (0, 40] N";
  } else if (!std::isfinite(params.disturbance_direction_sign) ||
             std::abs(params.disturbance_direction_sign) < 1e-12) {
    error = "disturbance_direction_sign must be non-zero and finite";
  } else if (!std::isfinite(params.disturbance_push_time) ||
             !std::isfinite(params.disturbance_hold_time) ||
             !std::isfinite(params.disturbance_release_time) ||
             params.disturbance_push_time < 0.0 ||
             params.disturbance_hold_time <= params.disturbance_push_time ||
             params.disturbance_release_time < params.disturbance_hold_time) {
    error = "disturbance times must satisfy 0 <= push < hold <= release";
  } else if (!std::isfinite(params.disturbance_release_ramp_time) ||
             params.disturbance_release_ramp_time <= 0.0) {
    error = "disturbance_release_ramp_time must be positive";
  } else if (!std::isfinite(params.disturbance_max_tau_norm) ||
             params.disturbance_max_tau_norm <= 0.0 ||
             params.disturbance_max_tau_norm > 5.0) {
    error = "disturbance_max_tau_norm must be in (0, 5] Nm";
  } else if (!std::isfinite(params.experiment_duration) ||
             params.experiment_duration <=
                 params.disturbance_release_time +
                     params.disturbance_release_ramp_time) {
    error = "experiment_duration must extend past the release ramp";
  }
  return error.empty();
}

double automaticDisturbanceScale(const ControllerConfig& params,
                                 double hold_time) {
  if (!params.disturbance_auto_enabled ||
      hold_time < params.disturbance_push_time) {
    return 0.0;
  }
  if (hold_time < params.disturbance_hold_time) {
    const double u =
        (hold_time - params.disturbance_push_time) /
        (params.disturbance_hold_time - params.disturbance_push_time);
    return 0.5 - 0.5 * std::cos(M_PI * u);
  }
  if (hold_time < params.disturbance_release_time) {
    return 1.0;
  }
  const double release_end =
      params.disturbance_release_time +
      params.disturbance_release_ramp_time;
  if (hold_time >= release_end) {
    return 0.0;
  }
  const double u =
      (hold_time - params.disturbance_release_time) /
      params.disturbance_release_ramp_time;
  return 0.5 + 0.5 * std::cos(M_PI * u);
}

Eigen::Matrix<double, 3, 7> pointJacobian(
    const Mat6x7& link_jacobian,
    const Mat3& R_link,
    const Vec3& point_link) {
  // Transforming the link-frame force point to a base-frame lever [m].
  const Vec3 lever_base = R_link * point_link;
  return link_jacobian.topRows<3>() -
         skewMatrix(lever_base) * link_jacobian.bottomRows<3>();
}

Vec3 automaticDisturbanceDirection(const ControllerConfig& params,
                                   const Model& model,
                                   const RobotState& state,
                                   const Mat6x7& ee_jacobian) {
  // Selecting the current one-dimensional end-effector nullspace direction.
  Eigen::JacobiSVD<Mat6x7> svd(
      ee_jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec7 null_direction = svd.matrixV().col(6);
  Eigen::Index dominant = 0;
  null_direction.cwiseAbs().maxCoeff(&dominant);
  if (null_direction(dominant) < 0.0) {
    null_direction = -null_direction;
  }

  // Selecting the configured robot link and loading its Jacobian and pose.
  const Frame frame = disturbanceFrame(params.disturbance_link);
  const std::array<double, 42> jacobian_array =
      model.zeroJacobian(frame, state);
  const std::array<double, 16> pose_array = model.pose(frame, state);
  Map<const Mat6x7> J_link(jacobian_array.data());
  Map<const Mat4x4> T_link(pose_array.data());
  const Eigen::Matrix<double, 3, 7> J_point =
      pointJacobian(J_link, T_link.block<3, 3>(0, 0),
                    params.disturbance_point_link);
  // Calculating the selected point motion caused by nullspace motion [m/s].
  const Vec3 point_velocity = J_point * null_direction;
  if (!point_velocity.allFinite() || point_velocity.norm() <= 1e-9) {
    return Vec3::Zero();
  }
  const double sign = params.disturbance_direction_sign > 0.0 ? 1.0 : -1.0;
  return sign * point_velocity.normalized();
}

AutomaticDisturbance computeAutomaticDisturbance(
    const ControllerConfig& params,
    const Model& model,
    const RobotState& state,
    const Vec3& force_direction_base,
    double hold_time) {
  AutomaticDisturbance command;
  // Calculating the time-dependent disturbance amplitude [-].
  command.scale = automaticDisturbanceScale(params, hold_time);
  if (command.scale <= 0.0) {
    return command;
  }

  // Selecting the configured robot link and loading its Jacobian and pose.
  const Frame frame = disturbanceFrame(params.disturbance_link);
  const std::array<double, 42> jacobian_array =
      model.zeroJacobian(frame, state);
  const std::array<double, 16> pose_array = model.pose(frame, state);
  Map<const Mat6x7> J_link(jacobian_array.data());
  Map<const Mat4x4> T_link(pose_array.data());
  const Mat3 R_link = T_link.block<3, 3>(0, 0);
  const Eigen::Matrix<double, 3, 7> J_point =
      pointJacobian(J_link, R_link, params.disturbance_point_link);

  // Assigning force point [m], force [N], and equivalent joint torque [N m].
  command.point_base =
      T_link.block<3, 1>(0, 3) + R_link * params.disturbance_point_link;
  command.force_base = command.scale * params.disturbance_force *
                       force_direction_base.normalized();
  command.tau.noalias() = J_point.transpose() * command.force_base;

  // Limiting the disturbance joint-torque norm [N m].
  const double tau_norm = command.tau.norm();
  if (tau_norm > params.disturbance_max_tau_norm) {
    command.torque_scale = params.disturbance_max_tau_norm / tau_norm;
    command.force_base *= command.torque_scale;
    command.tau *= command.torque_scale;
  }
  return command;
}
