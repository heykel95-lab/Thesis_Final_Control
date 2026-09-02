// ============================================================================
// Real-time control loop
// ============================================================================
// Executing the controller state machine and the 1 kHz torque-control callback,
// then returning the recorded run state to the reporting modules.
#include "controller_api.h"

// ====================================================================
// 2 + 3. One run: task frames, run state, and the 1 kHz control loop
// ====================================================================
RunResult runControlLoop(ControllerConfig& params,
                         Robot& robot,
                         const Model& model,
                         StateImpedanceGains gains,  // by value: retuning rebuilds it
                         KeyboardSignals& signals) {
  RunResult result;
  // Assigning local references to state stiffness and damping matrices.
  const Mat3& R_base_surface = gains.R_base_surface;
  const Mat3& Kp_approach = gains.Kp_approach;
  const Mat3& Dp_approach = gains.Dp_approach;
  const Mat3& KR_approach = gains.KR_approach;
  const Mat3& DR_approach = gains.DR_approach;
  const Mat3& Kp_contact_establishment = gains.Kp_contact_establishment;
  const Mat3& Dp_contact_establishment = gains.Dp_contact_establishment;
  const Mat3& KR_contact_establishment = gains.KR_contact_establishment;
  const Mat3& DR_contact_establishment = gains.DR_contact_establishment;
  const Mat3& Kp_hold = gains.Kp_hold;
  const Mat3& Dp_hold = gains.Dp_hold;
  const Mat3& KR_hold = gains.KR_hold;
  const Mat3& DR_hold = gains.DR_hold;
  const Mat3& Kp_operator_hold = gains.Kp_operator_hold;
  const Mat3& Dp_operator_hold = gains.Dp_operator_hold;
  const Mat3& KR_operator_hold = gains.KR_operator_hold;
  const Mat3& DR_operator_hold = gains.DR_operator_hold;

  // ================================================================
  // 2. Task frames, gains and run state
  // ================================================================
  // Reading the initial robot state and mapping the start pose.
  RobotState initial_robot_state = robot.readOnce();
  Map<const Mat4x4> T_initial(initial_robot_robot_state.O_T_EE.data());
  // Initializing TCP position [m], desired orientation [-], and joint posture [rad].
  Vec3 p_start = T_initial.block<3, 1>(0, 3);
  Mat3 R_d = T_initial.block<3, 3>(0, 0);
  Vec7 q_start = Map<const Vec7>(initial_robot_robot_state.q.data());

  // Initializing the reference pose for repeated contact-impedance hold trials.
  Vec3 hold_return_p = p_start;
  Mat3 hold_return_R = R_d;
  bool hold_returning = false;

  // Initializing force [N] and moment [N m] baselines for contact evaluation.
  Map<const Vec6> initial_external_wrench(
      initial_robot_robot_state.O_F_ext_hat_K.data());
  Vec3 contact_force_bias = initial_external_wrench.head<3>();
  Vec3 contact_moment_bias = initial_external_wrench.tail<3>();

  // Defining the desired tool orientation [-] and active surface point [m].
  Mat3 R_base_tool_target =
      makeToolTargetOrientation(params, R_base_surface, R_d);
  Vec3 surface_point_runtime =
      params.use_start_as_surface_point ? p_start : params.surface_point;

  // Selecting the descent direction opposite to the surface normal [-].
  const Vec3 descend_direction = -R_base_surface.col(2);

  // Defining the four tool-face corners in the end-effector frame [m].
  const Vec3& face_center = params.tool_contact_face_center_ee;
  const Vec3& half_width = params.tool_contact_half_width_ee;
  const Vec3& half_length = params.tool_contact_half_length_ee;
  const std::array<Vec3, 4> tool_face_corners_ee = {{
      face_center + half_width + half_length,
      face_center + half_width - half_length,
      face_center - half_width + half_length,
      face_center - half_width - half_length,
  }};
  const auto selectLeadingToolContact =
      [&](const Mat3& R_EE, double& leading_projection_out) -> Vec3 {
    std::array<double, 4> projection;
    double leading_projection =
        (R_EE * tool_face_corners_ee[0]).dot(descend_direction);
    projection[0] = leading_projection;
    for (std::size_t i = 1; i < tool_face_corners_ee.size(); ++i) {
      projection[i] = (R_EE * tool_face_corners_ee[i]).dot(descend_direction);
      leading_projection = std::max(leading_projection, projection[i]);
    }
    leading_projection_out = leading_projection;

    // Averaging tied vertices to obtain the center of the leading edge [m].
    Vec3 selected = Vec3::Zero();
    int selected_count = 0;
    for (std::size_t i = 0; i < tool_face_corners_ee.size(); ++i) {
      if (leading_projection - projection[i] <=
          params.tool_contact_feature_tie_tolerance) {
        selected += tool_face_corners_ee[i];
        ++selected_count;
      }
    }
    return selected / static_cast<double>(selected_count);
  };

  // Selecting the first and initial control states.
  const ControlState sequence_first_state =
      params.enable_orientation_state ? ControlState::kToolOrientation
                                      : ControlState::kSurfaceApproach;
  const ControlState initial_state =
      params.run_state_sequence ? sequence_first_state : ControlState::kPoseHold;
  Vec3 disturbance_force_direction_base = Vec3::Zero();
  if (params.disturbance_auto_enabled) {
    std::string error;
    if (!validateAutomaticDisturbance(params, error)) {
      throw std::runtime_error("automatic disturbance: " + error);
    }
    if (initial_state != ControlState::kPoseHold ||
        params.use_contact_impedance_hold) {
      throw std::runtime_error(
          "automatic disturbance requires the plain h hold mode");
    }
    const std::array<double, 42> initial_jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, initial_robot_state);
    Map<const Mat6x7> initial_jacobian(initial_jacobian_array.data());
    disturbance_force_direction_base = automaticDisturbanceDirection(
        params, model, initial_robot_state, initial_jacobian);
    if (disturbance_force_direction_base.norm() <= 1e-9) {
      throw std::runtime_error(
          "automatic disturbance point cannot excite the redundant axis");
    }
  }
  ControlState state = initial_state;
  // Initializing the state resumed after manual guidance.
  ControlState restart_state = initial_state;
  // Clearing pending run-mode input before entering torque control.
  signals.run_mode_request.store(0);
  // Initializing state and terminal clocks [s].
  double state_start_time = 0.0;
  double next_debug_time = 0.0;
  bool descend_failed = false;

  // Initializing the contact TCP, contact point [m], and contact orientation.
  Vec3 first_contact_tcp = p_start;
  Vec3 first_contact_point = p_start;
  Mat3 R_contact_start = R_base_tool_target;
  Vec3 active_tool_contact_offset_ee = params.tool_contact_face_center_ee;

  // Initializing the bounded desired-orientation command.
  Mat3 R_orient_start = R_d;
  Mat3 R_orient_command = R_base_tool_target;

  // Initializing the operator-controlled hold reference [m].
  Vec3 pre_contact_hold_p_d = p_start;
  bool contact_establishment_reported = false;
  bool disturb_push_cued = false;       // scripted disturbance cues, once each
  bool disturb_hold_cued = false;
  bool disturb_release_cued = false;

  // Initializing contact establishment and grinding virtual penetration [m].
  double contact_establishment_push_start = -params.descend_surface_clearance;
  double grind_push = 0.0;

  // Tracking the offline alignment criterion. The state clock at which the
  // deviation entered the tolerance is kept, and only once it has stayed
  // inside for the configured hold does that entry time become the reported
  // alignment time. Neither value is read by the control law.
  double align_entered_at = -1.0;
  double t_align = -1.0;
  // The relative criterion and the closest approach. The deviation at first
  // contact is captured on the first contact establishment sample, so both are available
  // while the state runs and neither depends on how the state ends.
  double deviation_at_contact_deg = -1.0;
  double t_align_fraction = -1.0;
  double deviation_min_deg = 0.0;
  double t_deviation_min = 0.0;

  // Initializing state-dependent damping matrices.
  StateDampingCache damping = manualStateDampingCache(gains);

  printSection("start pose");
  printVec7Deg("q_start", q_start);
  printVec3Mm("p_start", p_start);

  // Allocating the bounded real-time log buffer before torque control.
  const int log_every_n_cycles = std::max(1, params.log_every_n_cycles);
  const std::size_t max_log_rows = static_cast<std::size_t>(std::max(0, params.max_log_rows));
  std::vector<LogData> log_data(max_log_rows);
  std::size_t control_cycle_count = 0;
  std::size_t log_write_index = 0;
  std::size_t log_rows_written = 0;
  bool log_buffer_wrapped = false;


  // Initializing run time [s] and final-state storage.
  double time = 0.0;
  Vec3 final_p_EE = Vec3::Zero();
  Vec3 final_p_d = Vec3::Zero();
  Vec3 final_e_p = Vec3::Zero();
  Vec3 final_e_R = Vec3::Zero();
  Vec7 final_q = q_start;

  printBanner("RUN");
  // Initializing the terminal state for state-dependent information.
  bool contact_establishment_law_printed = false;
  // Assigning the initial terminal-state tracker.
  ControlState intro_printed_for = ControlState::kManualGuidance;
  if (state == ControlState::kPoseHold && !params.use_contact_impedance_hold) {
    printNullspaceLaw(params);
    printAutomaticDisturbance(params, disturbance_force_direction_base);
  }

  // ================================================================
  // 3. Control loop: libfranka calls this back at ~1 kHz with the current
  //    state and expects the 7 commanded joint torques in return.
  // ================================================================
  robot.control([&](const RobotState& robot_state, Duration period) -> Torques {
    // Integrating elapsed controller time [s].
    time += period.toSec();

    // Mapping measured joint velocity [rad/s] and joint position [rad].
    Map<const Vec7> dq(robot_state.dq.data());
    Map<const Vec7> q_current(robot_state.q.data());

    // Loading the 6x7 end-effector Jacobian with the configured tool offset.
    std::array<double, 42> jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, robot_state);
    Map<const Mat6x7> J(jacobian_array.data());
    // Calculating Cartesian linear velocity [m/s] and angular velocity [rad/s].
    const Vec6 xdot = J * dq;
    const Vec3 pdot = xdot.head<3>();
    const Vec3 omega = xdot.tail<3>();

    // Mapping TCP position [m] and orientation [-] in the robot base frame.
    Map<const Mat4x4> T_EE(robot_state.O_T_EE.data());
    const Vec3 p_EE = T_EE.block<3, 1>(0, 3);
    const Mat3 R_EE = T_EE.block<3, 3>(0, 0);

    // Mapping both libfranka wrench representations. O_F_ext_hat_K is kept
    // unchanged for compatibility with the campaign logs. K_F_ext_hat_K is
    // rotated into the base axes so its moment remains explicitly referenced
    // to K. The relative K-to-TCP offset then suffices for wrench transport.
    Map<const Vec6> external_wrench(robot_state.O_F_ext_hat_K.data());
    const Vec3 external_force = external_wrench.head<3>();
    const Vec3 external_moment = external_wrench.tail<3>();
    Map<const Vec6> external_wrench_K(robot_state.K_F_ext_hat_K.data());
    Map<const Mat4x4> T_EE_K(robot_state.EE_T_K.data());
    const Mat3 R_base_K = R_EE * T_EE_K.block<3, 3>(0, 0);
    const Vec3 r_K_TCP_base =
        R_EE * T_EE_K.block<3, 1>(0, 3);
    const Vec3 external_force_K_base =
        R_base_K * external_wrench_K.head<3>();
    const Vec3 external_moment_K_base =
        R_base_K * external_wrench_K.tail<3>();

    // Reinitializing controller references from the measured pose after a mode change.
    const auto restartFromPoseReached = [&](bool reanchor_surface_point) {
      p_start = p_EE;
      R_d = R_EE;
      q_start = q_current;
      R_base_tool_target =
          makeToolTargetOrientation(params, R_base_surface, R_d);
      // Restarting the orientation slew from the measured pose.
      R_orient_start = R_d;
      R_orient_command = R_d;
      // Re-anchoring the surface point only after manual pose placement.
      if (reanchor_surface_point && params.use_start_as_surface_point) {
        surface_point_runtime = p_start;
      }
      contact_force_bias = external_force;
      contact_moment_bias = external_moment;
      first_contact_tcp = p_start;
      first_contact_point = p_start;
      R_contact_start = R_base_tool_target;
      active_tool_contact_offset_ee = params.tool_contact_face_center_ee;

      state_start_time = time;
      next_debug_time = time;
      pre_contact_hold_p_d = p_start;
      contact_establishment_reported = false;
      contact_establishment_push_start = -params.descend_surface_clearance;
      grind_push = 0.0;
      damping.hold_computed = false;
      damping.Dp_hold = Dp_hold;
      damping.DR_hold = DR_hold;
      damping.operator_hold_computed = false;
      damping.Dp_operator_hold = Dp_operator_hold;
      damping.DR_operator_hold = DR_operator_hold;

    };

    // ---------------------------------------------------------------
    // Selecting the run mode
    // ---------------------------------------------------------------
    // Selecting the state sequence or contact-impedance hold during a run.
    const char run_mode_request = signals.run_mode_request.exchange(0);
    if (run_mode_request == 's') {
      if (state == ControlState::kManualGuidance) {
        printf("Pose recapture is active; complete it with p + Enter.\n");
      } else if (state != ControlState::kPoseHold) {
        printf("The state sequence is active; use t + Enter for contact-impedance hold.\n");
      } else if (hold_returning) {
        // Completing the return trajectory before restarting the sequence.
        printf("Return motion to the hold reference is active.\n");
      } else {
        params.run_state_sequence = true;
        params.use_contact_impedance_hold = false;
        restartFromPoseReached(false);
        restart_state = sequence_first_state;
        state = sequence_first_state;
        intro_printed_for = ControlState::kManualGuidance;
        contact_establishment_law_printed = false;
        printSection("s: sequence from the pose held");
        printVec3Mm("p_start", p_start);
        printf("  %-16s   the one commanded now\n", "impedance");
        printf("  %-16s   t1 %.2f deg | t2 %.2f deg\n", "tilt",
               params.tool_target_offset_tangent1_deg,
               params.tool_target_offset_tangent2_deg);
      }
    } else if (run_mode_request == 't') {
      if (state == ControlState::kManualGuidance) {
        printf("Pose recapture is active; complete it with p + Enter.\n");
      } else if (state == ControlState::kPoseHold && params.use_contact_impedance_hold) {
        printf(hold_returning
                   ? "Return motion to the hold reference is active.\n"
                   : "Contact-impedance hold is active.\n");
      } else {
        params.run_state_sequence = false;
        params.use_contact_impedance_hold = true;
        restartFromPoseReached(false);
        restart_state = ControlState::kPoseHold;
        state = ControlState::kPoseHold;
        hold_returning = true;
        // Refreshing contact establishment-hold information after the mode transition.
        intro_printed_for = ControlState::kManualGuidance;
        contact_establishment_law_printed = false;
        printSection("t: contact establishment impedance hold, returning to its pose");
        printVec3Mm("p_start", hold_return_p);
        printf("  %-16s   %.3f m/s, turning at %.1f deg/s\n", "returning at",
               params.descend_speed, params.approach_orient_max_rate_deg);
      }
    }

    // ---------------------------------------------------------------
    // Entering manual guidance
    // ---------------------------------------------------------------
    // Entering gravity-compensated manual guidance from active control.
    if (state != ControlState::kManualGuidance && signals.guide_requested.load()) {
      signals.guide_requested.store(false);
      signals.proceed_requested.store(false);


      state = ControlState::kManualGuidance;
      state_start_time = time;
      // Refreshing state information after guidance.
      intro_printed_for = ControlState::kManualGuidance;
      printStateHeader(ControlState::kManualGuidance);
      printf("  %-16s   move the tool by hand\n", "motion");
      // Selecting the state resumed after pose recapture.
      printf("  %-16s   p re-capture and restart %s | e stop\n", "keys",
             stateName(restart_state));
    }
    if (state == ControlState::kManualGuidance) {
      // Loading Coriolis torque [N m] and calculating task torque J^T W [N m].
      Array7 coriolis_array = model.coriolis(robot_state);
      Map<const Vec7> coriolis(coriolis_array.data());
      const Array7 tau_array =
          vec7ToArray(Vec7(coriolis - params.manual_guidance_damping * dq));
      if (signals.stop_requested.load()) {
        printf("\nStop requested with e + Enter. Finishing control loop...\n");
        return MotionFinished(Torques(tau_array));
      }
      if (signals.proceed_requested.load()) {
        signals.proceed_requested.store(false);
        // Assigning the guided pose as the new sequence and hold reference.
        restartFromPoseReached(true);
        state = restart_state;
        hold_return_p = p_start;
        hold_return_R = R_d;
        hold_returning = false;

        printSection("resuming from the re-guided pose");
        printVec7Deg("q_start", q_start);
        printVec3Mm("p_start", p_start);
      }
      return Torques(tau_array);
    }

    // ---------------------------------------------------------------
    // Applying the hold-disturbance sequence
    // ---------------------------------------------------------------
    // Defining hold-state time [s] for repeatable disturbance timing.
    if ((params.disturbance_cues_enabled ||
         params.disturbance_auto_enabled) &&
        state == ControlState::kPoseHold) {
      const auto cue = [&](const char* text) {
        printf("\n>>> %s  (t = %.1f s)\n", text, time);
        fflush(stdout);
      };
      const double hold_time = time - state_start_time;
      if (!disturb_push_cued && hold_time >= params.disturbance_push_time) {
        disturb_push_cued = true;
        cue(params.disturbance_auto_enabled
                ? "AUTOMATIC PUSH START"
                : "PUSH THE ARM NOW");
      }
      // Marking the transition from driven motion to static loading.
      if (!disturb_hold_cued && hold_time >= params.disturbance_hold_time) {
        disturb_hold_cued = true;
        cue(params.disturbance_auto_enabled
                ? "AUTOMATIC PUSH AT FULL FORCE"
                : "STOP MOVING - hold it still");
      }
      if (!disturb_release_cued &&
          hold_time >= params.disturbance_release_time) {
        disturb_release_cued = true;
        cue(params.disturbance_auto_enabled
                ? "AUTOMATIC RELEASE START"
                : "RELEASE - do not touch until the run ends");
      }
    }

    // Selecting the pre-transition state for contact-feature locking.
    const bool edge_locked =
        (state == ControlState::kContactEstablishment ||
         state == ControlState::kPreGrindingHold ||
         state == ControlState::kGrinding);

    // ---------------------------------------------------------------
    // Calculating the active tool contact point [m].
    // ---------------------------------------------------------------
    Vec3 tool_contact_offset_ee = Vec3::Zero();
    double leading_contact_projection = 0.0;
    if (params.use_tool_contact_point_control) {
      if (edge_locked) {
        tool_contact_offset_ee = active_tool_contact_offset_ee;
        leading_contact_projection =
            (R_EE * tool_contact_offset_ee).dot(descend_direction);
      } else if (params.auto_select_tool_contact_edge) {
        tool_contact_offset_ee =
            selectLeadingToolContact(R_EE, leading_contact_projection);
      } else {
        tool_contact_offset_ee = params.tool_contact_face_center_ee;
        leading_contact_projection =
            (R_EE * tool_contact_offset_ee).dot(descend_direction);
      }
    }
    const Vec3 tool_contact_point = p_EE + R_EE * tool_contact_offset_ee;

    DesiredTranslationCommand desired{p_start, Vec3::Zero()};
    Vec3 edge_target_log = first_contact_point;
    double push_log = 0.0;
    // The compliance centre sits on the TCP until a lever moves it.
    Vec3 p_CoC_log = p_EE;
    Vec3 r_eff_log = tool_contact_point - p_EE;
    // Tracking the explicit operator-hold state for damping selection.
    bool operator_hold_active = state == ControlState::kPreContactHold;

    switch (state) {
      // -------------------------------------------------------------
      // Orienting the tool before surface descent.
      // -------------------------------------------------------------
      case ControlState::kToolOrientation: {
        // Defining current and desired tool-axis directions in base
        // coordinates [-].
        const Vec3 tool_axis_current =
            currentToolAxisInBase(params, R_EE).normalized();
        const Vec3 tool_axis_target =
            desiredToolAxisInBase(params, R_base_surface).normalized();
        // Calculating the angular separation between both tool axes [rad].
        const double tool_axis_error = std::acos(
            std::max(-1.0,
                     std::min(1.0, tool_axis_current.dot(tool_axis_target))));
        // Calculating elapsed state time [s].
        const double state_time = time - state_start_time;
        // Resolving tool tilt and twist in the surface frame [rad].
        const Vec3 e_R_orient =
            R_base_surface.transpose() *
            applyRotationalAxisMask(
                params, orientationError(R_EE, R_base_tool_target),
                R_base_surface);
        const double spin_error = std::abs(e_R_orient(2));

        // Limiting the desired-orientation slew rate [rad/s].
        {
          const Eigen::AngleAxisd to_target(R_base_tool_target *
                                            R_orient_start.transpose());
          const double reachable =
              (M_PI / 180.0) * params.approach_orient_max_rate_deg * state_time;
          const double commanded =
              std::min(std::abs(to_target.angle()), std::max(0.0, reachable));
          R_orient_command =
              Mat3(Eigen::AngleAxisd(commanded, to_target.axis())) *
              R_orient_start;
        }

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            intro_printed_for == state) {
          printApproachOrientDebug(state_time,
                                   (180.0 / M_PI) * tool_axis_error,
                                   (180.0 / M_PI) * spin_error);
          next_debug_time = time + params.debug_period;
        }

        // Evaluating convergence for each commanded rotational component [rad].
        const bool orientation_reached =
            tool_axis_error <= params.approach_orient_error_threshold &&
            (!params.command_tool_twist ||
             spin_error <= params.approach_orient_spin_error_threshold);
        // Ending the state after convergence or the configured timeout [s].
        const bool orient_timed_out =
            params.approach_orient_timeout > 0.0 &&
            state_time >= params.approach_orient_timeout;
        if (state_time >= params.approach_orient_min_time &&
            (orientation_reached || orient_timed_out)) {
          state = ControlState::kSurfaceApproach;
          state_start_time = time;
          next_debug_time = time;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          if (orientation_reached) {
            printf("\nOrientation reached: axis_err=%.1f deg | spin_err=%.1f deg\n",
                   (180.0 / M_PI) * tool_axis_error,
                   (180.0 / M_PI) * spin_error);
          } else {
            printf("\nOrientation settled short of the %.1f deg threshold after "
                   "%.1f s: axis_err=%.1f deg | spin_err=%.1f deg\n",
                   (180.0 / M_PI) * params.approach_orient_error_threshold,
                   params.approach_orient_timeout,
                   (180.0 / M_PI) * tool_axis_error,
                   (180.0 / M_PI) * spin_error);
          }
        }
        // Maintaining the initial TCP position while rotating the tool.
        break;
      }

      // -------------------------------------------------------------
      // Descending to the configured surface clearance [m].
      // -------------------------------------------------------------
      case ControlState::kSurfaceApproach: {
        // Calculating elapsed approach time [s] and bounded descent [m].
        const double state_time = time - state_start_time;
        const double distance =
            std::min(params.descend_speed * state_time,
                     params.descend_max_distance);
        desired.p_d = p_start + distance * descend_direction;
        desired.pdot_d = params.descend_speed * descend_direction;

        // Defining the surface normal [-] and tool-face clearance [m].
        const Vec3 surface_normal = (-descend_direction).normalized();
        const double height_above_surface =
            surface_normal.dot(p_EE - surface_point_runtime) -
            leading_contact_projection;
        const double controlled_point_height =
            surface_normal.dot(tool_contact_point - surface_point_runtime);
        const Vec3 projected_surface_point =
            tool_contact_point - controlled_point_height * surface_normal;
        const bool clearance_reached =
            height_above_surface <= params.descend_surface_clearance;
        // Projecting the measured contact-force change onto the descent axis [N].
        const double force_along_descend =
            (external_force - contact_force_bias).dot(descend_direction);

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            intro_printed_for == state) {
          printApproachDescendDebug(state_time,
                                    1000.0 * distance,
                                    1000.0 * height_above_surface,
                                    1000.0 * params.descend_surface_clearance,
                                    force_along_descend);
          next_debug_time = time + params.debug_period;
        }

        if (clearance_reached) {
          active_tool_contact_offset_ee = tool_contact_offset_ee;
          first_contact_tcp = p_EE;
          first_contact_point = projected_surface_point;
          // Using the live signed surface coordinate [m] for a continuous transition.
          contact_establishment_push_start = -controlled_point_height;
          R_contact_start = R_EE;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          if (params.enable_pre_contact_hold) {
            pre_contact_hold_p_d = p_EE;
            desired.p_d = pre_contact_hold_p_d;
            desired.pdot_d.setZero();
            signals.operator_hold_continue.store(false);
            state = ControlState::kPreContactHold;
            operator_hold_active = true;
            printf("[STATE] Pre-contact hold active. Press Enter to start "
                   "contact establishment (e+Enter stops).\n");
          } else {
            state = ControlState::kContactEstablishment;
          }
          state_start_time = time;
          next_debug_time = time;
          printf("\nClearance reached: distance=%.1f mm | height=%.1f mm | target=%.1f mm | force=%.1f N\n",
                 1000.0 * distance,
                 1000.0 * height_above_surface,
                 1000.0 * params.descend_surface_clearance,
                 force_along_descend);
          printContactEdgeDebug(active_tool_contact_offset_ee, first_contact_tcp,
                                first_contact_point);
        } else if (distance >= params.descend_max_distance) {
          descend_failed = true;
          signals.stop_requested.store(true);
          desired.p_d = p_EE;
          desired.pdot_d.setZero();
        }
        break;
      }

      // -------------------------------------------------------------
      // Holding the measured clearance pose until operator confirmation.
      // -------------------------------------------------------------
      case ControlState::kPreContactHold: {
        operator_hold_active = true;
        desired.p_d = pre_contact_hold_p_d;
        desired.pdot_d.setZero();

        if (signals.operator_hold_continue.exchange(false)) {
          const Vec3 surface_normal = (-descend_direction).normalized();
          const double controlled_point_height =
              surface_normal.dot(tool_contact_point - surface_point_runtime);
          first_contact_tcp = p_EE;
          first_contact_point =
              tool_contact_point - controlled_point_height * surface_normal;
          contact_establishment_push_start = -controlled_point_height;
          R_contact_start = R_EE;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          state = ControlState::kContactEstablishment;
          state_start_time = time;
          next_debug_time = time;
          operator_hold_active = false;
          printf("[STATE] Continuing to contact establishment.\n");
        }
        break;
      }

      // -------------------------------------------------------------
      // Pressing the contact edge while holding the contact orientation
      // as a soft rotational target.
      // -------------------------------------------------------------
      case ControlState::kContactEstablishment: {
        // Calculating elapsed contact establishment time [s] and initializing push speed [m/s].
        const double state_time = time - state_start_time;
        double contact_establishment_push_velocity = 0.0;
        // Calculating the bounded virtual penetration [m] from the live
        // contact-establishment state clock.
        const double push =
            contactEstablishmentPush(params, state_time, contact_establishment_push_start,
                      contact_establishment_push_velocity);
        // Assigning the desired pressed-edge position in the base frame [m].
        const Vec3 edge_target = first_contact_point + push * descend_direction;
        desired.p_d = edge_target - R_contact_start * tool_contact_offset_ee;
        desired.pdot_d = contact_establishment_push_velocity * descend_direction;
        edge_target_log = edge_target;
        push_log = push;

        // Calculating the contact-force change since first contact [N].
        const Vec3 df_ext = external_force - contact_force_bias;
        const double df_ext_norm = df_ext.norm();

        // Referencing the moment to the TCP [N m]. O_F_ext_hat_K reports the
        // moment about the base origin, so the lever of the TCP position has
        // to come out before the moment describes anything at the tool. It
        // dominates otherwise: 42 N m of base moment against 1 N m at the TCP.
        const Vec3 m_tcp =
            (external_moment - contact_moment_bias) -
            p_EE.cross(df_ext);
        const double m_tcp_norm = m_tcp.norm();

        // Transforming moment from the TCP to the contact point [N m]:
        // M_contact = M_TCP + r_contact x f with r_contact = p_EE - p_contact [m].
        const Vec3 r_contact = p_EE - tool_contact_point;
        const Vec3 m_contact =
            m_tcp + r_contact.cross(df_ext);

        // Locating the centre of compliance and the lever the contact force
        // acts through [m]. A zero lever leaves the centre on the TCP, so the
        // same two lines cover both the referenced and the shifted case.
        const Vec3 r_c_log =
            complianceLeverBase(params, R_EE, R_base_surface);
        p_CoC_log = p_EE + r_c_log;
        r_eff_log = tool_contact_point - p_CoC_log;

        // Observing the alignment criterion. It only reads the deviation and
        // the clock, and never feeds the state exit or the command.
        const double deviation_deg =
            (180.0 / M_PI) *
            toolSurfaceMisalignmentAngle(params, R_EE, R_base_surface);
        if (deviation_deg < params.contact_establishment_align_tolerance_deg) {
          if (align_entered_at < 0.0) {
            align_entered_at = state_time;
          } else if (t_align < 0.0 &&
                     state_time - align_entered_at >=
                         params.contact_establishment_align_hold_time) {
            t_align = align_entered_at;
          }
        } else {
          align_entered_at = -1.0;
        }

        // Capturing the deviation the state started from, then the closest
        // approach to flat and the crossing of the relative threshold.
        if (deviation_at_contact_deg < 0.0) {
          deviation_at_contact_deg = deviation_deg;
          deviation_min_deg = deviation_deg;
          t_deviation_min = state_time;
        }
        if (deviation_deg < deviation_min_deg) {
          deviation_min_deg = deviation_deg;
          t_deviation_min = state_time;
        }
        if (t_align_fraction < 0.0 &&
            deviation_deg <
                params.contact_establishment_align_fraction * deviation_at_contact_deg) {
          t_align_fraction = state_time;
        }

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            intro_printed_for == state) {
          // Measuring passive rotation from the first-contact orientation [rad].
          printContactEstablishmentDebug(state_time,
                          (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm(),
                          df_ext_norm,
                          m_tcp_norm,
                          1000.0 * (tool_contact_point - first_contact_point).norm());
          next_debug_time = time + params.debug_period;
        }

        // Ending contact establishment at its timeout [s].
        if (state_time < params.contact_establishment_timeout) {
          break;
        }

        // Capturing the contact-establishment result before the next state.
        if (!contact_establishment_reported) {
          contact_establishment_reported = true;
          ContactEstablishmentReport report;
          report.state_time = state_time;
          report.df_ext_norm = df_ext_norm;
          report.m_tcp_norm = m_tcp_norm;
          report.t_align = t_align;
          report.t_align_fraction = t_align_fraction;
          report.deviation_min_deg = deviation_min_deg;
          report.t_deviation_min = t_deviation_min;
          report.p_EE = p_EE;
          report.R_EE = R_EE;
          report.tool_contact_point = tool_contact_point;
          report.external_force = external_force;
          report.m_contact = m_contact;
          report.first_contact_tcp = first_contact_tcp;
          report.first_contact_point = first_contact_point;
          report.R_contact_start = R_contact_start;
          report.contact_force_bias = contact_force_bias;
          report.Kp = Kp_contact_establishment;
          report.Dp = params.contact_establishment_auto_damping ? damping.Dp_contact_establishment : Dp_contact_establishment;
          report.KR = KR_contact_establishment;
          report.DR = params.contact_establishment_auto_damping ? damping.DR_contact_establishment : DR_contact_establishment;
          reportContactEstablishmentResult(params, R_base_surface, report);
        }

        // Freezing the established penetration for the following hold or grind [m].
        grind_push = push;
        if (params.enable_pre_grinding_hold) {
          signals.operator_hold_continue.store(false);
          state = ControlState::kPreGrindingHold;
          printf("\n[STATE] Pre-grinding hold active. Press Enter to start "
                 "grinding (e+Enter stops).\n");
        } else {
          state = ControlState::kGrinding;
        }
        state_start_time = time;
        next_debug_time = time;
        break;
      }

      // -------------------------------------------------------------
      // Holding the established contact command until operator confirmation.
      // -------------------------------------------------------------
      case ControlState::kPreGrindingHold: {
        const Vec3 edge_target =
            first_contact_point + grind_push * descend_direction;
        desired.p_d =
            edge_target - R_contact_start * active_tool_contact_offset_ee;
        desired.pdot_d.setZero();
        edge_target_log = edge_target;
        push_log = grind_push;

        if (signals.operator_hold_continue.exchange(false)) {
          state = ControlState::kGrinding;
          state_start_time = time;
          next_debug_time = time;
          printf("[STATE] Continuing to grinding.\n");
        }
        break;
      }

      // -------------------------------------------------------------
      // Maintaining the frozen preload with an optional tangential sweep.
      // -------------------------------------------------------------
      case ControlState::kGrinding: {
        const Vec3 n = descend_direction;  // unit, into the surface
        Vec3 edge_target;

        if (params.grind_sweep_enabled) {
          const Vec3 grind_tangent = (params.grind_tangent_axis == 2)
                                         ? Vec3(R_base_surface.col(1))
                                         : Vec3(R_base_surface.col(0));
          double sweep_s = 0.0;
          double sweep_s_dot = 0.0;
          grindSweep(time - state_start_time, params.grind_amplitude_m,
                     grindStrokeDuration(params), sweep_s, sweep_s_dot);
          edge_target = first_contact_point + grind_push * n + sweep_s * grind_tangent;
          desired.pdot_d = sweep_s_dot * grind_tangent;
        } else {
          const double edge_penetration = n.dot(tool_contact_point - first_contact_point);
          edge_target = tool_contact_point + (grind_push - edge_penetration) * n;
        }

        desired.p_d = edge_target - R_contact_start * tool_contact_offset_ee;
        edge_target_log = edge_target;
        push_log = grind_push;
        break;
      }

      // -------------------------------------------------------------
      // Holding the captured Cartesian pose or returning smoothly to it.
      // -------------------------------------------------------------
      case ControlState::kPoseHold: {
        if (!hold_returning) {
          break;
        }
        // Defining the remaining translation [m] and bounded cycle step [m].
        const Vec3 to_home = hold_return_p - p_start;
        const double distance = to_home.norm();
        const double step = params.descend_speed * period.toSec();
        const bool position_home = distance <= step;
        if (position_home) {
          p_start = hold_return_p;
        } else {
          p_start += (step / distance) * to_home;
          desired.pdot_d = (params.descend_speed / distance) * to_home;
        }

        // Defining the remaining rotation and bounded cycle step [rad].
        const Eigen::AngleAxisd to_home_R(hold_return_R * R_d.transpose());
        const double step_R =
            (M_PI / 180.0) * params.approach_orient_max_rate_deg * period.toSec();
        const bool orientation_home = std::abs(to_home_R.angle()) <= step_R;
        if (orientation_home) {
          R_d = hold_return_R;
        } else {
          R_d = Mat3(Eigen::AngleAxisd(std::copysign(step_R, to_home_R.angle()),
                                       to_home_R.axis())) *
                R_d;
        }

        desired.p_d = p_start;
        if (position_home && orientation_home) {
          hold_returning = false;
          desired.pdot_d.setZero();
          printf("Back at the pose the hold started from.\n");
        }
        break;
      }

      case ControlState::kManualGuidance:
        break;  // hold the captured start position
    }

    // Selecting the post-transition state for control and logging.
    const bool after_contact =
        (state == ControlState::kContactEstablishment ||
         state == ControlState::kPreGrindingHold ||
         state == ControlState::kGrinding);

    // ---------------------------------------------------------------
    // Cartesian errors
    // ---------------------------------------------------------------
    const Vec3 e_p = desired.p_d - p_EE;
    const Mat3& R_d_used =
        after_contact ? R_contact_start
        : (state == ControlState::kPoseHold) ? R_d
        : (state == ControlState::kToolOrientation) ? R_orient_command
                                                   : R_base_tool_target;
    const Vec3 e_R =
        applyRotationalAxisMask(params, orientationError(R_EE, R_d_used), R_base_surface);

    if (state == ControlState::kPoseHold && params.print_hold_debug &&
        params.debug_period > 0.0 && time >= next_debug_time &&
        intro_printed_for == state) {
      printHoldDebug(time,
                     (external_force - contact_force_bias).norm(),
                     1000.0 * e_p.norm(),
                     (180.0 / M_PI) * e_R.norm());
      next_debug_time = time + params.debug_period;
    }

    updateAutoDamping(params, gains, model, robot_state, J, state, after_contact,
                      operator_hold_active, damping);

    const Mat3& Dp_approach_eff =
        params.approach_auto_damping ? damping.Dp_approach : Dp_approach;
    const Mat3& DR_approach_eff =
        params.approach_auto_damping ? damping.DR_approach : DR_approach;
    const Mat3& Dp_contact_establishment_eff = params.contact_establishment_auto_damping ? damping.Dp_contact_establishment : Dp_contact_establishment;
    const Mat3& DR_contact_establishment_eff = params.contact_establishment_auto_damping ? damping.DR_contact_establishment : DR_contact_establishment;
    const Mat3& Dp_hold_eff = params.hold_auto_damping ? damping.Dp_hold : Dp_hold;
    const Mat3& DR_hold_eff = params.hold_auto_damping ? damping.DR_hold : DR_hold;
    const Mat3& Dp_operator_hold_eff =
        params.operator_hold_auto_damping ? damping.Dp_operator_hold : Dp_operator_hold;
    const Mat3& DR_operator_hold_eff =
        params.operator_hold_auto_damping ? damping.DR_operator_hold : DR_operator_hold;

    // ---------------------------------------------------------------
    // Selecting the impedance gains associated with the active state.
    // ---------------------------------------------------------------
    // Selecting hold gains or contact establishment gains when mode t is active.
    const Mat3* Kp_state = params.use_contact_impedance_hold ? &Kp_contact_establishment : &Kp_hold;
    const Mat3* Dp_state =
        params.use_contact_impedance_hold ? &Dp_contact_establishment_eff : &Dp_hold_eff;
    const Mat3* KR_state = params.use_contact_impedance_hold ? &KR_contact_establishment : &KR_hold;
    const Mat3* DR_state =
        params.use_contact_impedance_hold ? &DR_contact_establishment_eff : &DR_hold_eff;
    switch (state) {
      case ControlState::kToolOrientation:
      case ControlState::kSurfaceApproach:
        Kp_state = &Kp_approach;
        Dp_state = &Dp_approach_eff;
        KR_state = &KR_approach;
        DR_state = &DR_approach_eff;
        break;
      case ControlState::kPreContactHold:
        Kp_state = &Kp_operator_hold;
        Dp_state = &Dp_operator_hold_eff;
        KR_state = &KR_operator_hold;
        DR_state = &DR_operator_hold_eff;
        break;
      case ControlState::kContactEstablishment:
      case ControlState::kPreGrindingHold:
      case ControlState::kGrinding:
        Kp_state = &Kp_contact_establishment;
        Dp_state = &Dp_contact_establishment_eff;
        KR_state = &KR_contact_establishment;
        DR_state = &DR_contact_establishment_eff;
        break;
      case ControlState::kPoseHold:
      case ControlState::kManualGuidance:
        break;
    }
    const Mat3& Kp_used = *Kp_state;
    const Mat3& Dp_used = *Dp_state;
    const Mat3& KR_used = *KR_state;
    const Mat3& DR_used = *DR_state;

    // ---------------------------------------------------------------
    // Constructing Cartesian displacement [m, rad] and velocity errors
    // [m/s, rad/s] for the impedance law.
    // ---------------------------------------------------------------
    Vec6 dx;
    dx.head<3>() = e_p;
    dx.tail<3>() = e_R;
    Vec6 dv;
    dv.head<3>() = desired.pdot_d - pdot;
    dv.tail<3>() = -omega;

    const Vec6 wrench =
        computeCartesianImpedanceWrench(
            params, state, Kp_used, Dp_used, KR_used, DR_used,
            R_base_surface, dx, dv, R_EE);
    const Vec3 f = wrench.head<3>();
    const Vec3 m = wrench.tail<3>();

    if (state == ControlState::kGrinding && params.grind_sweep_enabled &&
        params.print_grind_debug && params.debug_period > 0.0 &&
        time >= next_debug_time && intro_printed_for == state) {
      // Reporting sweep displacement [m], error [m], and press force [N].
      const Vec3 grind_tangent = (params.grind_tangent_axis == 2)
                                     ? Vec3(R_base_surface.col(1))
                                     : Vec3(R_base_surface.col(0));
      double sweep_s = 0.0;
      double sweep_s_dot = 0.0;
      grindSweep(time - state_start_time, params.grind_amplitude_m,
                 grindStrokeDuration(params), sweep_s, sweep_s_dot);
      printGrindDebug(time - state_start_time,
                      1000.0 * sweep_s,
                      1000.0 * e_p.dot(grind_tangent),
                      f.dot(descend_direction));
      next_debug_time = time + params.debug_period;
    }

    // Reading live nullspace damping [N m s/rad], sigma torque [N m],
    // and probe step [deg].
    const double damping_request =
        signals.nullspace_damping_request.exchange(
            std::numeric_limits<double>::quiet_NaN());
    const double k_sigma_request =
        signals.nullspace_sigma_gain_request.exchange(
            std::numeric_limits<double>::quiet_NaN());
    const double alpha_deg_request =
        signals.nullspace_probe_step_deg_request.exchange(
            std::numeric_limits<double>::quiet_NaN());
    const int mode_request = signals.nullspace_mode_request.exchange(-1);
    if (std::isfinite(damping_request) || std::isfinite(k_sigma_request) ||
        std::isfinite(alpha_deg_request) || mode_request >= 0) {
      if (state == ControlState::kPoseHold && !params.use_contact_impedance_hold) {
        if (std::isfinite(damping_request)) {
          params.nullspace_damping = damping_request;
          printf("nullspace damping -> %.3f Nms/rad\n", damping_request);
        }
        if (std::isfinite(k_sigma_request)) {
          params.nullspace_sigma_gain = k_sigma_request;
          printf("nullspace k_sigma -> %.3f Nm\n", k_sigma_request);
        }
        if (std::isfinite(alpha_deg_request)) {
          // Converting the probe step from degrees to radians [rad].
          params.nullspace_probe_step_rad = alpha_deg_request * M_PI / 180.0;
          printf("nullspace alpha -> %.3f deg = %.6f rad\n",
                 alpha_deg_request, params.nullspace_probe_step_rad);
        }
        if (mode_request >= 0) {
          params.nullspace_mode = static_cast<NullspaceMode>(mode_request);
        }
        printNullspaceLaw(params);
      } else if (state == ControlState::kPoseHold) {
        printf("Nullspace tuning is available in Cartesian pose hold.\n");
      } else {
        printf("Nullspace tuning requires an active hold mode.\n");
      }
    }

    // Assigning orientation offsets [deg] used by the next state sequence.
    for (int i = 0; i < 2; ++i) {
      const double tilt_deg = signals.contact_establishment_tilt_deg_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(tilt_deg)) {
        continue;
      }
      if (state != ControlState::kPoseHold || !params.use_contact_impedance_hold) {
        printf("Tool-tilt tuning is available in contact-impedance hold.\n");
        continue;
      }
      if (i == 0) {
        params.tool_target_offset_tangent1_deg = tilt_deg;
      } else {
        params.tool_target_offset_tangent2_deg = tilt_deg;
      }
      printf("commanded tilt for s -> t1 %.2f deg | t2 %.2f deg\n",
             params.tool_target_offset_tangent1_deg,
             params.tool_target_offset_tangent2_deg);
    }

    // Rebuilding contact establishment gains and damping after a live tuning update.
    bool contact_establishment_impedance_changed = false;
    for (int i = 0; i < 3; ++i) {
      const double kp = signals.contact_establishment_kp_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double kr = signals.contact_establishment_kr_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double center_mm = signals.contact_establishment_compliance_center_mm_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double rc_mm = signals.contact_establishment_rc_mm_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(kp) && !std::isfinite(kr) && !std::isfinite(center_mm) &&
          !std::isfinite(rc_mm)) {
        continue;
      }
      if (state != ControlState::kPoseHold || !params.use_contact_impedance_hold) {
        printf("Contact-impedance tuning is available in mode t.\n");
        continue;
      }
      if (std::isfinite(kp) && kp > 0.0) {
        if (params.contact_establishment_translation_surface_frame) {
          params.contact_establishment_Kp_surface_diag(i) = kp;
        } else {
          params.contact_establishment_Kp_diag(i) = kp;
        }
        contact_establishment_impedance_changed = true;
      }
      if (std::isfinite(kr) && kr > 0.0) {
        params.contact_establishment_KR_diag(i) = kr;
        contact_establishment_impedance_changed = true;
      }
      // Converting live compliance-center values [mm] to the configured frame.
      if (std::isfinite(center_mm) || std::isfinite(rc_mm)) {
        if (!params.use_virtual_compliance_center) {
          printf("Compliance-center tuning requires the virtual center of "
                 "compliance.\n");
        } else {
          // Resolving r_c = p_C - p_TCP in the robot base frame [m].
          Vec3 r_c_base = Vec3::Zero();
          if (params.compliance_center_in_tool_frame) {
            r_c_base = R_EE * params.compliance_center_offset_ee;
          } else if (params.compliance_lever_in_surface_frame) {
            r_c_base = R_base_surface * params.compliance_lever_surface;
          }

          // Assigning one updated component in the selected command frame [m].
          if (std::isfinite(center_mm)) {
            Vec3 center_ee = R_EE.transpose() * r_c_base;
            center_ee(i) = 0.001 * center_mm;
            r_c_base = R_EE * center_ee;
          }
          if (std::isfinite(rc_mm)) {
            Vec3 rc_surface = R_base_surface.transpose() * r_c_base;
            rc_surface(i) = 0.001 * rc_mm;
            r_c_base = R_base_surface * rc_surface;
          }

          // Storing the updated center in the configured representation [m].
          if (params.compliance_center_in_tool_frame) {
            params.compliance_center_offset_ee = R_EE.transpose() * r_c_base;
          } else if (params.compliance_lever_in_surface_frame) {
            params.compliance_lever_surface =
                R_base_surface.transpose() * r_c_base;
          }
          contact_establishment_impedance_changed = true;
        }
      }
    }
    if (contact_establishment_impedance_changed) {
      gains = buildStateImpedanceGains(params);
      damping = manualStateDampingCache(gains);
      contact_establishment_law_printed = false;  // reprinted with the new gains
    }
    // Printing each state block after active damping matrices are available.
    if (intro_printed_for != state) {
      printStateHeader(state);
      printStateIntro(params, damping, state);
      intro_printed_for = state;
    }

    const bool wants_contact_establishment_block =
        state == ControlState::kContactEstablishment ||
        state == ControlState::kPreGrindingHold ||
        (state == ControlState::kPoseHold && params.use_contact_impedance_hold);
    if (wants_contact_establishment_block && !contact_establishment_law_printed) {
      printContactEstablishmentImpedanceLaw(params, damping,
                             state == ControlState::kPoseHold,
                             R_base_surface, R_EE);
      contact_establishment_law_printed = true;
    }
    if (!wants_contact_establishment_block) {
      // Resetting the report flag before the next contact establishment entry.
      contact_establishment_law_printed = false;
    }
    // Calculating the nullspace torque selected by nullspace_mode [N m].
    const Vec7 tau_nullspace =
        computeNullspaceTorque(params, model, robot_state, J, dq);

    // Initializing the optional joint-torque disturbance [N m].
    AutomaticDisturbance disturbance;
    if (state == ControlState::kPoseHold && params.disturbance_auto_enabled) {
      disturbance = computeAutomaticDisturbance(
          params, model, robot_state, disturbance_force_direction_base,
          time - state_start_time);
    }

    // Loading Coriolis torque [N m] and calculating task torque J^T W [N m].
    Array7 coriolis_array = model.coriolis(robot_state);
    Map<const Vec7> coriolis(coriolis_array.data());
    const Vec7 tau_task = J.transpose() * wrench;
    // Summing task, nullspace, disturbance, and Coriolis torques [N m].
    const Vec7 tau_cmd =
        tau_task + tau_nullspace + disturbance.tau + coriolis;

    // ---------------------------------------------------------------
    // Assigning the sampled state to the preallocated ring buffer.
    // ---------------------------------------------------------------
    ++control_cycle_count;
    if (max_log_rows > 0 &&
        (control_cycle_count % static_cast<std::size_t>(log_every_n_cycles)) == 0) {
      LogData& row = log_data[log_write_index];
      row.time = time;
      row.state = static_cast<int>(state);
      row.nullspace_mode = static_cast<int>(params.nullspace_mode);
      row.p_EE = p_EE;
      row.p_d = desired.p_d;
      row.tool_contact_point = tool_contact_point;
      row.first_contact_tcp = first_contact_tcp;
      row.first_contact_point = first_contact_point;
      row.edge_target = edge_target_log;
      row.tool_contact_offset_ee = tool_contact_offset_ee;
      row.p_CoC = p_CoC_log;
      row.r_eff = r_eff_log;
      row.e_p = e_p;
      row.e_R = e_R;
      row.angular_deviation_surface =
          R_base_surface.transpose() *
          toolSurfaceAlignmentErrorBase(params, R_EE, R_base_surface);
      row.angular_deviation = row.angular_deviation_surface.norm();
      row.t_align = t_align;
      row.pdot = pdot;
      row.pdot_d = desired.pdot_d;
      row.omega = omega;
      row.f = f;
      row.m = m;
      row.external_force = external_force;
      row.external_moment = external_moment;
      row.external_force_K_base = external_force_K_base;
      row.external_moment_K_base = external_moment_K_base;
      row.r_K_TCP_base = r_K_TCP_base;
      row.setup_Dp_used = damping.setup_damping_valid
                              ? damping.setup_Dp_used
                              : gains.setup_Dp_active_diag;
      row.setup_DR_used = damping.setup_damping_valid
                              ? damping.setup_DR_used
                              : params.setup_DR_diag;
      row.contact_force_bias = contact_force_bias;
      row.contact_moment_bias = contact_moment_bias;
      row.push = push_log;
      row.disturbance = disturbance;
      row.tau_nullspace_norm = tau_nullspace.norm();
      row.nullspace_damping = params.nullspace_damping;
      row.tau_cmd = tau_cmd;

      log_write_index = (log_write_index + 1) % max_log_rows;
      if (log_rows_written < max_log_rows) {
        ++log_rows_written;
      } else {
        log_buffer_wrapped = true;
      }
    }

    final_p_EE = p_EE;
    final_p_d = desired.p_d;
    final_e_p = e_p;
    final_e_R = e_R;
    final_q = q_current;

    const Array7 tau_array = vec7ToArray(tau_cmd);
    if ((params.experiment_duration > 0.0 && time >= params.experiment_duration) ||
        signals.stop_requested.load()) {
      if (signals.stop_requested.load()) {
        printf("\nStop requested with e + Enter. Finishing control loop...\n");
      }
      return MotionFinished(Torques(tau_array));
    }
    return Torques(tau_array);
    });
  // Converting the ring buffer to chronological sample order.
  std::vector<LogData> ordered_log_data;
  ordered_log_data.reserve(log_rows_written);
  if (log_buffer_wrapped) {
    ordered_log_data.insert(ordered_log_data.end(),
                            log_data.begin() + static_cast<std::ptrdiff_t>(log_write_index),
                            log_data.end());
    ordered_log_data.insert(ordered_log_data.end(),
                            log_data.begin(),
                            log_data.begin() + static_cast<std::ptrdiff_t>(log_write_index));
    printf("Log buffer wrapped: kept latest %zu rows, sampled every %d control cycles.\n",
           log_rows_written, log_every_n_cycles);
  } else {
    ordered_log_data.insert(ordered_log_data.end(),
                            log_data.begin(),
                            log_data.begin() + static_cast<std::ptrdiff_t>(log_rows_written));
  }
  result.log = std::move(ordered_log_data);

  // Assigning the final state and recorded samples to the run result.
  result.descend_failed = descend_failed;
  result.q_start = q_start;
  result.final_q = final_q;
  result.final_p_d = final_p_d;
  result.final_p_EE = final_p_EE;
  result.final_e_p = final_e_p;
  result.final_e_R = final_e_R;
  return result;
}
