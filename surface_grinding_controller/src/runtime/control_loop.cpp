// ============================================================================
// Real-time control loop
// ============================================================================
// Executing the phase state machine and the 1 kHz torque-control callback,
// then returning the recorded run state to the reporting modules.
#include "controller_api.h"

// ====================================================================
// 2 + 3. One run: task frames, run state, and the 1 kHz control loop
// ====================================================================
RunResult runControlLoop(ControllerConfig& params,
                         Robot& robot,
                         const Model& model,
                         PhaseImpedanceGains gains,  // by value: retuning rebuilds it
                         KeyboardSignals& signals) {
  RunResult result;
  // Assigning local references to phase stiffness and damping matrices.
  const Mat3& R_base_surface = gains.R_base_surface;
  const Mat3& Kp_approach = gains.Kp_approach;
  const Mat3& Dp_approach = gains.Dp_approach;
  const Mat3& KR_approach = gains.KR_approach;
  const Mat3& DR_approach = gains.DR_approach;
  const Mat3& Kp_setup = gains.Kp_setup;
  const Mat3& Dp_setup = gains.Dp_setup;
  const Mat3& KR_setup = gains.KR_setup;
  const Mat3& DR_setup = gains.DR_setup;
  const Mat3& Kp_hold = gains.Kp_hold;
  const Mat3& Dp_hold = gains.Dp_hold;
  const Mat3& KR_hold = gains.KR_hold;
  const Mat3& DR_hold = gains.DR_hold;
  const Mat3& Kp_pause = gains.Kp_pause;
  const Mat3& Dp_pause = gains.Dp_pause;
  const Mat3& KR_pause = gains.KR_pause;
  const Mat3& DR_pause = gains.DR_pause;

  // ================================================================
  // 2. Task frames, gains and run state
  // ================================================================
  // Reading the initial robot state and mapping the start pose.
  RobotState initial_state = robot.readOnce();
  Map<const Mat4x4> T_initial(initial_state.O_T_EE.data());
  // Initializing TCP position [m], desired orientation [-], and joint posture [rad].
  Vec3 p_start = T_initial.block<3, 1>(0, 3);
  Mat3 R_d = T_initial.block<3, 3>(0, 0);
  Vec7 q_start = Map<const Vec7>(initial_state.q.data());

  // Initializing the reference pose for repeated setup-impedance hold trials.
  Vec3 hold_return_p = p_start;
  Mat3 hold_return_R = R_d;
  bool hold_returning = false;

  // Initializing force [N] and moment [N m] baselines for contact evaluation.
  Map<const Vec6> initial_external_wrench(initial_state.O_F_ext_hat_K.data());
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

  // Selecting the first and initial control phases.
  const ControlPhase sequence_first_phase =
      params.enable_orientation_phase ? ControlPhase::kToolOrientation
                                      : ControlPhase::kSurfaceApproach;
  const ControlPhase initial_phase =
      params.run_phase_sequence ? sequence_first_phase : ControlPhase::kPoseHold;
  Vec3 disturbance_force_direction_base = Vec3::Zero();
  if (params.disturbance_auto_enabled) {
    std::string error;
    if (!validateAutomaticDisturbance(params, error)) {
      throw std::runtime_error("automatic disturbance: " + error);
    }
    if (initial_phase != ControlPhase::kPoseHold ||
        params.use_setup_impedance_hold) {
      throw std::runtime_error(
          "automatic disturbance requires the plain h hold mode");
    }
    const std::array<double, 42> initial_jacobian_array =
        model.zeroJacobian(Frame::kEndEffector, initial_state);
    Map<const Mat6x7> initial_jacobian(initial_jacobian_array.data());
    disturbance_force_direction_base = automaticDisturbanceDirection(
        params, model, initial_state, initial_jacobian);
    if (disturbance_force_direction_base.norm() <= 1e-9) {
      throw std::runtime_error(
          "automatic disturbance point cannot excite the redundant axis");
    }
  }
  ControlPhase phase = initial_phase;
  // Initializing the phase resumed after manual guidance.
  ControlPhase restart_phase = initial_phase;
  // Clearing pending run-mode input before entering torque control.
  signals.run_mode_request.store(0);
  // Initializing phase and terminal clocks [s].
  double phase_start_time = 0.0;
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

  // Initializing phase-gate states and paused clocks [s].
  bool gate_setup_armed = false;
  bool gate_setup_passed = false;
  Vec3 gate_setup_hold_pd = Vec3::Zero();
  double gate_paused_time = 0.0;  // frozen descend clock while gated
  bool gate_grind_armed = false;
  bool gate_grind_passed = false;
  bool setup_reported = false;  // the result prints once, at phase end
  bool disturb_push_cued = false;       // scripted disturbance cues, once each
  bool disturb_hold_cued = false;
  bool disturb_release_cued = false;

  // Initializing setup and grinding virtual penetration [m].
  double setup_push_start = -params.descend_surface_clearance;
  double grind_push = 0.0;

  // Initializing phase-dependent damping matrices.
  PhaseDampingCache damping = manualPhaseDampingCache(gains);

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
  // Initializing the terminal state for phase-dependent information.
  bool setup_law_printed = false;
  // Assigning the initial terminal phase-state tracker.
  ControlPhase intro_printed_for = ControlPhase::kManualGuidance;
  bool gate_block_printed = false;
  if (phase == ControlPhase::kPoseHold && !params.use_setup_impedance_hold) {
    printNullspaceLaw(params);
    printAutomaticDisturbance(params, disturbance_force_direction_base);
  }

  // ================================================================
  // 3. Control loop: libfranka calls this back at ~1 kHz with the current
  //    state and expects the 7 commanded joint torques in return.
  // ================================================================
  robot.control([&](const RobotState& state, Duration period) -> Torques {
    // Integrating elapsed controller time [s].
    time += period.toSec();

    // Mapping measured joint velocity [rad/s] and joint position [rad].
    Map<const Vec7> dq(state.dq.data());
    Map<const Vec7> q_current(state.q.data());

    // Loading the 6x7 end-effector Jacobian with the configured tool offset.
    std::array<double, 42> jacobian_array = model.zeroJacobian(Frame::kEndEffector, state);
    Map<const Mat6x7> J(jacobian_array.data());
    // Calculating Cartesian linear velocity [m/s] and angular velocity [rad/s].
    const Vec6 xdot = J * dq;
    const Vec3 pdot = xdot.head<3>();
    const Vec3 omega = xdot.tail<3>();

    // Mapping TCP position [m] and orientation [-] in the robot base frame.
    Map<const Mat4x4> T_EE(state.O_T_EE.data());
    const Vec3 p_EE = T_EE.block<3, 1>(0, 3);
    const Mat3 R_EE = T_EE.block<3, 3>(0, 0);

    // Mapping estimated external force [N] and moment [N m].
    Map<const Vec6> external_wrench(state.O_F_ext_hat_K.data());
    const Vec3 external_force = external_wrench.head<3>();
    const Vec3 external_moment = external_wrench.tail<3>();

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

      phase_start_time = time;
      next_debug_time = time;
      gate_setup_armed = false;
      gate_setup_passed = false;
      gate_paused_time = 0.0;
      gate_grind_armed = false;
      gate_grind_passed = false;
      setup_reported = false;
      setup_push_start = -params.descend_surface_clearance;
      grind_push = 0.0;
      damping.hold_computed = false;
      damping.Dp_hold = Dp_hold;
      damping.DR_hold = DR_hold;
      damping.pause_computed = false;
      damping.Dp_pause = Dp_pause;
      damping.DR_pause = DR_pause;

    };

    // ---------------------------------------------------------------
    // Selecting the run mode
    // ---------------------------------------------------------------
    // Selecting the phase sequence or setup-impedance hold during a run.
    const char run_mode_request = signals.run_mode_request.exchange(0);
    if (run_mode_request == 's') {
      if (phase == ControlPhase::kManualGuidance) {
        printf("Pose recapture is active; complete it with p + Enter.\n");
      } else if (phase != ControlPhase::kPoseHold) {
        printf("The phase sequence is active; use t + Enter for setup hold.\n");
      } else if (hold_returning) {
        // Completing the return trajectory before restarting the sequence.
        printf("Return motion to the hold reference is active.\n");
      } else {
        params.run_phase_sequence = true;
        params.use_setup_impedance_hold = false;
        restartFromPoseReached(false);
        restart_phase = sequence_first_phase;
        phase = sequence_first_phase;
        intro_printed_for = ControlPhase::kManualGuidance;
        setup_law_printed = false;
        printSection("s: sequence from the pose held");
        printVec3Mm("p_start", p_start);
        printf("  %-16s   the one commanded now\n", "impedance");
        printf("  %-16s   t1 %.2f deg | t2 %.2f deg\n", "tilt",
               params.tool_target_offset_tangent1_deg,
               params.tool_target_offset_tangent2_deg);
      }
    } else if (run_mode_request == 't') {
      if (phase == ControlPhase::kManualGuidance) {
        printf("Pose recapture is active; complete it with p + Enter.\n");
      } else if (phase == ControlPhase::kPoseHold && params.use_setup_impedance_hold) {
        printf(hold_returning
                   ? "Return motion to the hold reference is active.\n"
                   : "Setup-impedance hold is active.\n");
      } else {
        params.run_phase_sequence = false;
        params.use_setup_impedance_hold = true;
        restartFromPoseReached(false);
        restart_phase = ControlPhase::kPoseHold;
        phase = ControlPhase::kPoseHold;
        hold_returning = true;
        // Refreshing setup-hold information after the mode transition.
        intro_printed_for = ControlPhase::kManualGuidance;
        setup_law_printed = false;
        printSection("t: setup impedance hold, returning to its pose");
        printVec3Mm("p_start", hold_return_p);
        printf("  %-16s   %.3f m/s, turning at %.1f deg/s\n", "returning at",
               params.descend_speed, params.approach_orient_max_rate_deg);
      }
    }

    // ---------------------------------------------------------------
    // Entering manual guidance
    // ---------------------------------------------------------------
    // Entering gravity-compensated manual guidance from active control.
    if (phase != ControlPhase::kManualGuidance && signals.guide_requested.load()) {
      signals.guide_requested.store(false);
      signals.proceed_requested.store(false);


      phase = ControlPhase::kManualGuidance;
      phase_start_time = time;
      // Refreshing phase information after guidance.
      intro_printed_for = ControlPhase::kManualGuidance;
      printPhaseHeader(ControlPhase::kManualGuidance);
      printf("  %-16s   move the tool by hand\n", "motion");
      // Selecting the phase resumed after pose recapture.
      printf("  %-16s   p re-capture and restart %s | e stop\n", "keys",
             phaseName(restart_phase));
    }
    if (phase == ControlPhase::kManualGuidance) {
      // Loading Coriolis torque [N m] and calculating task torque J^T W [N m].
    Array7 coriolis_array = model.coriolis(state);
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
        phase = restart_phase;
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
    // Defining hold-phase time [s] for repeatable disturbance timing.
    if ((params.disturbance_cues_enabled ||
         params.disturbance_auto_enabled) &&
        phase == ControlPhase::kPoseHold) {
      const auto cue = [&](const char* text) {
        printf("\n>>> %s  (t = %.1f s)\n", text, time);
        fflush(stdout);
      };
      const double hold_time = time - phase_start_time;
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

    // Selecting the pre-transition phase for contact-feature locking.
    const bool edge_locked =
        (phase == ControlPhase::kSetup || phase == ControlPhase::kGrinding);

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
    // Selecting stiff Cartesian hold only while a phase gate blocks.
    bool pause_hold_active = false;

    switch (phase) {
      // -------------------------------------------------------------
      // Orienting the tool before surface descent.
      // -------------------------------------------------------------
      case ControlPhase::kToolOrientation: {
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
        // Calculating elapsed phase time [s].
        const double phase_time = time - phase_start_time;
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
              (M_PI / 180.0) * params.approach_orient_max_rate_deg * phase_time;
          const double commanded =
              std::min(std::abs(to_target.angle()), std::max(0.0, reachable));
          R_orient_command =
              Mat3(Eigen::AngleAxisd(commanded, to_target.axis())) *
              R_orient_start;
        }

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            intro_printed_for == phase) {
          printApproachOrientDebug(phase_time,
                                   (180.0 / M_PI) * tool_axis_error,
                                   (180.0 / M_PI) * spin_error);
          next_debug_time = time + params.debug_period;
        }

        // Evaluating convergence for each commanded rotational component [rad].
        const bool orientation_reached =
            tool_axis_error <= params.approach_orient_error_threshold &&
            (!params.command_tool_twist ||
             spin_error <= params.approach_orient_spin_error_threshold);
        // Ending the phase after convergence or the configured timeout [s].
        const bool orient_timed_out =
            params.approach_orient_timeout > 0.0 &&
            phase_time >= params.approach_orient_timeout;
        if (phase_time >= params.approach_orient_min_time &&
            (orientation_reached || orient_timed_out)) {
          phase = ControlPhase::kSurfaceApproach;
          phase_start_time = time;
          next_debug_time = time;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          if (orientation_reached) {
            printf("\nOrientation reached: axis_err=%.1f deg | spin_err=%.1f deg\n",
                   (180.0 / M_PI) * tool_axis_error,
                   (180.0 / M_PI) * spin_error);
          } else {
            printf("\nOrientation settled short of the %.1f deg gate after "
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
      case ControlPhase::kSurfaceApproach: {
        // Calculating elapsed approach time [s] and bounded descent [m].
        const double phase_time = time - phase_start_time;
        const double distance =
            std::min(params.descend_speed * (phase_time - gate_paused_time),
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

        // Holding the measured clearance pose until operator confirmation.
        bool gate_blocking = false;
        if (params.pause_before_setup && !gate_setup_passed &&
            (gate_setup_armed || clearance_reached)) {
          if (!gate_setup_armed) {
            gate_setup_armed = true;
            // Assigning the measured pose reached at the gate.
            gate_setup_hold_pd = p_EE;
            signals.gate_continue.store(false);
            printf("\n[GATE] Reached %.0f mm above the plane. Press Enter to start "
                   "the setup press (e+Enter stops).\n",
                   1000.0 * params.descend_surface_clearance);
          }
          if (signals.gate_continue.load()) {
            gate_setup_passed = true;
            signals.gate_continue.store(false);
            printf("[GATE] Continuing to the setup press.\n");
          } else {
            gate_blocking = true;
            pause_hold_active = true;
            desired.p_d = gate_setup_hold_pd;
            desired.pdot_d.setZero();
            // Freezing the approach clock during operator confirmation.
            gate_paused_time += period.toSec();
          }
        }

        if (params.debug_period > 0.0 && time >= next_debug_time &&
            !gate_blocking && intro_printed_for == phase) {
          printApproachDescendDebug(phase_time,
                                    1000.0 * distance,
                                    1000.0 * height_above_surface,
                                    1000.0 * params.descend_surface_clearance,
                                    force_along_descend);
          next_debug_time = time + params.debug_period;
        }

        if (clearance_reached && !gate_blocking) {
          active_tool_contact_offset_ee = tool_contact_offset_ee;
          first_contact_tcp = p_EE;
          first_contact_point = projected_surface_point;
          // Using the live signed surface coordinate [m] for a continuous transition.
          setup_push_start = -controlled_point_height;
          R_contact_start = R_EE;
          contact_force_bias = external_force;
          contact_moment_bias = external_moment;
          phase = ControlPhase::kSetup;
          phase_start_time = time;
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
      // Pressing the contact edge while holding the contact orientation
      // as a soft rotational target.
      // -------------------------------------------------------------
      case ControlPhase::kSetup: {
        // Calculating elapsed setup time [s] and initializing push speed [m/s].
        const double phase_time = time - phase_start_time;
        double setup_push_velocity = 0.0;
        // Calculating the bounded virtual penetration [m]. The ramp runs on the
        // live phase clock, so waiting at the grinding gate keeps pressing
        // deeper until the configured final penetration is reached.
        const double push =
            setupPush(params, phase_time, setup_push_start,
                      setup_push_velocity);
        // Assigning the desired pressed-edge position in the base frame [m].
        const Vec3 edge_target = first_contact_point + push * descend_direction;
        desired.p_d = edge_target - R_contact_start * tool_contact_offset_ee;
        desired.pdot_d = setup_push_velocity * descend_direction;
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

        const bool waiting_at_gate = gate_grind_armed && !gate_grind_passed;
        if (params.debug_period > 0.0 && time >= next_debug_time &&
            !waiting_at_gate && intro_printed_for == phase) {
          // Measuring passive rotation from the first-contact orientation [rad].
          printSetupDebug(phase_time,
                          (180.0 / M_PI) * orientationError(R_EE, R_contact_start).norm(),
                          df_ext_norm,
                          m_tcp_norm,
                          params.setup_moment_threshold,
                          1000.0 * (tool_contact_point - first_contact_point).norm());
          next_debug_time = time + params.debug_period;
        }

        // Ending setup at the moment threshold [N m] or timeout [s].
        const bool stopped_on_moment =
            phase_time >= params.setup_min_time &&
            m_tcp_norm >= params.setup_moment_threshold;
        if (!gate_grind_armed && !stopped_on_moment &&
            phase_time < params.setup_timeout) {
          break;
        }

        // Capturing the setup result before entering the grinding gate.
        if (!setup_reported) {
          setup_reported = true;
          SetupReport report;
          report.stopped_on_moment = stopped_on_moment;
          report.phase_time = phase_time;
          report.df_ext_norm = df_ext_norm;
          report.m_tcp_norm = m_tcp_norm;
          report.p_EE = p_EE;
          report.R_EE = R_EE;
          report.tool_contact_point = tool_contact_point;
          report.external_force = external_force;
          report.m_contact = m_contact;
          report.first_contact_tcp = first_contact_tcp;
          report.first_contact_point = first_contact_point;
          report.R_contact_start = R_contact_start;
          report.contact_force_bias = contact_force_bias;
          report.Kp = Kp_setup;
          report.Dp = params.setup_auto_damping ? damping.Dp_setup : Dp_setup;
          report.KR = KR_setup;
          report.DR = params.setup_auto_damping ? damping.DR_setup : DR_setup;
          reportSetupResult(params, R_base_surface, report);
        }

        if (params.pause_before_grind && !gate_grind_passed) {
          if (!gate_grind_armed) {
            gate_grind_armed = true;
            signals.gate_continue.store(false);
            printf("\n[GATE] Set up finished (still pressing). Press "
                   "Enter to start the grind (e+Enter stops).\n");
          }
          if (!signals.gate_continue.load()) {
            break;  // keep ramping the preload while waiting
          }
          gate_grind_passed = true;
          signals.gate_continue.store(false);
          printf("[GATE] Continuing to grind.\n");
        }

        // Assigning the final setup penetration to the grinding phase [m].
        grind_push = push;
        phase = ControlPhase::kGrinding;
        phase_start_time = time;
        next_debug_time = time;
        break;
      }

      // -------------------------------------------------------------
      // Maintaining the frozen preload with an optional tangential sweep.
      // -------------------------------------------------------------
      case ControlPhase::kGrinding: {
        const Vec3 n = descend_direction;  // unit, into the surface
        Vec3 edge_target;

        if (params.grind_sweep_enabled) {
          const Vec3 grind_tangent = (params.grind_tangent_axis == 2)
                                         ? Vec3(R_base_surface.col(1))
                                         : Vec3(R_base_surface.col(0));
          double sweep_s = 0.0;
          double sweep_s_dot = 0.0;
          grindSweep(time - phase_start_time, params.grind_amplitude_m,
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
      case ControlPhase::kPoseHold: {
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

      case ControlPhase::kManualGuidance:
        break;  // hold the captured start position
    }

    // Selecting the post-transition phase for control and logging.
    const bool after_contact =
        (phase == ControlPhase::kSetup || phase == ControlPhase::kGrinding);

    // ---------------------------------------------------------------
    // Cartesian errors
    // ---------------------------------------------------------------
    const Vec3 e_p = desired.p_d - p_EE;
    const Mat3& R_d_used =
        after_contact ? R_contact_start
        : (phase == ControlPhase::kPoseHold) ? R_d
        : (phase == ControlPhase::kToolOrientation) ? R_orient_command
                                                   : R_base_tool_target;
    const Vec3 e_R =
        applyRotationalAxisMask(params, orientationError(R_EE, R_d_used), R_base_surface);

    if (phase == ControlPhase::kPoseHold && params.print_hold_debug &&
        params.debug_period > 0.0 && time >= next_debug_time &&
        intro_printed_for == phase) {
      printHoldDebug(time,
                     (external_force - contact_force_bias).norm(),
                     1000.0 * e_p.norm(),
                     (180.0 / M_PI) * e_R.norm());
      next_debug_time = time + params.debug_period;
    }

    updateAutoDamping(params, gains, model, state, J, phase, after_contact,
                      pause_hold_active, damping);

    const Mat3& Dp_approach_eff =
        params.approach_auto_damping ? damping.Dp_approach : Dp_approach;
    const Mat3& DR_approach_eff =
        params.approach_auto_damping ? damping.DR_approach : DR_approach;
    const Mat3& Dp_setup_eff = params.setup_auto_damping ? damping.Dp_setup : Dp_setup;
    const Mat3& DR_setup_eff = params.setup_auto_damping ? damping.DR_setup : DR_setup;
    const Mat3& Dp_hold_eff = params.hold_auto_damping ? damping.Dp_hold : Dp_hold;
    const Mat3& DR_hold_eff = params.hold_auto_damping ? damping.DR_hold : DR_hold;
    const Mat3& Dp_pause_eff =
        params.pause_hold_auto_damping ? damping.Dp_pause : Dp_pause;
    const Mat3& DR_pause_eff =
        params.pause_hold_auto_damping ? damping.DR_pause : DR_pause;

    // ---------------------------------------------------------------
    // Selecting phase gains; a gate hold overrides the position gains.
    // ---------------------------------------------------------------
    // Selecting hold gains or setup gains when mode t is active.
    const Mat3* Kp_phase = params.use_setup_impedance_hold ? &Kp_setup : &Kp_hold;
    const Mat3* Dp_phase =
        params.use_setup_impedance_hold ? &Dp_setup_eff : &Dp_hold_eff;
    const Mat3* KR_phase = params.use_setup_impedance_hold ? &KR_setup : &KR_hold;
    const Mat3* DR_phase =
        params.use_setup_impedance_hold ? &DR_setup_eff : &DR_hold_eff;
    switch (phase) {
      case ControlPhase::kToolOrientation:
      case ControlPhase::kSurfaceApproach:
        Kp_phase = &Kp_approach;
        Dp_phase = &Dp_approach_eff;
        KR_phase = &KR_approach;
        DR_phase = &DR_approach_eff;
        break;
      case ControlPhase::kSetup:
      case ControlPhase::kGrinding:
        Kp_phase = &Kp_setup;
        Dp_phase = &Dp_setup_eff;
        KR_phase = &KR_setup;
        DR_phase = &DR_setup_eff;
        break;
      case ControlPhase::kPoseHold:
      case ControlPhase::kManualGuidance:
        break;
    }
    const Mat3& Kp_used = pause_hold_active ? Kp_pause : *Kp_phase;
    const Mat3& Dp_used = pause_hold_active ? Dp_pause_eff : *Dp_phase;
    const Mat3& KR_used = pause_hold_active ? KR_pause : *KR_phase;
    const Mat3& DR_used = pause_hold_active ? DR_pause_eff : *DR_phase;

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
            params, phase, Kp_used, Dp_used, KR_used, DR_used,
            R_base_surface, dx, dv, R_EE);
    const Vec3 f = wrench.head<3>();
    const Vec3 m = wrench.tail<3>();

    if (phase == ControlPhase::kGrinding && params.grind_sweep_enabled &&
        params.print_grind_debug && params.debug_period > 0.0 &&
        time >= next_debug_time && intro_printed_for == phase) {
      // Reporting sweep displacement [m], error [m], and press force [N].
      const Vec3 grind_tangent = (params.grind_tangent_axis == 2)
                                     ? Vec3(R_base_surface.col(1))
                                     : Vec3(R_base_surface.col(0));
      double sweep_s = 0.0;
      double sweep_s_dot = 0.0;
      grindSweep(time - phase_start_time, params.grind_amplitude_m,
                 grindStrokeDuration(params), sweep_s, sweep_s_dot);
      printGrindDebug(time - phase_start_time,
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
      if (phase == ControlPhase::kPoseHold && !params.use_setup_impedance_hold) {
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
      } else if (phase == ControlPhase::kPoseHold) {
        printf("Nullspace tuning is available in Cartesian pose hold.\n");
      } else {
        printf("Nullspace tuning requires an active hold mode.\n");
      }
    }

    // Assigning orientation offsets [deg] used by the next phase sequence.
    for (int i = 0; i < 2; ++i) {
      const double tilt_deg = signals.setup_tilt_deg_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(tilt_deg)) {
        continue;
      }
      if (phase != ControlPhase::kPoseHold || !params.use_setup_impedance_hold) {
        printf("Tool-tilt tuning is available in setup-impedance hold.\n");
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

    // Rebuilding setup gains and damping after a live tuning update.
    bool setup_impedance_changed = false;
    for (int i = 0; i < 3; ++i) {
      const double kp = signals.setup_kp_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double kr = signals.setup_kr_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double center_mm = signals.setup_compliance_center_mm_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      const double rc_mm = signals.setup_rc_mm_request[i].exchange(
          std::numeric_limits<double>::quiet_NaN());
      if (!std::isfinite(kp) && !std::isfinite(kr) && !std::isfinite(center_mm) &&
          !std::isfinite(rc_mm)) {
        continue;
      }
      if (phase != ControlPhase::kPoseHold || !params.use_setup_impedance_hold) {
        printf("Setup-impedance tuning is available in mode t.\n");
        continue;
      }
      if (std::isfinite(kp) && kp > 0.0) {
        if (params.setup_translation_surface_frame) {
          params.setup_Kp_surface_diag(i) = kp;
        } else {
          params.setup_Kp_diag(i) = kp;
        }
        setup_impedance_changed = true;
      }
      if (std::isfinite(kr) && kr > 0.0) {
        params.setup_KR_diag(i) = kr;
        setup_impedance_changed = true;
      }
      // Converting live compliance-center values [mm] to the configured frame.
      if (std::isfinite(center_mm) || std::isfinite(rc_mm)) {
        if (!params.use_virtual_compliance_center) {
          printf("Compliance-center tuning requires the virtual center of "
                 "compliance.\n");
        } else {
          // Resolving r_c = p_TCP - p_C in the robot base frame [m].
          Vec3 r_c_base = Vec3::Zero();
          if (params.compliance_center_in_tool_frame) {
            r_c_base = -(R_EE * params.compliance_center_offset_ee);
          } else if (params.compliance_lever_in_surface_frame) {
            r_c_base =
                R_base_surface * params.r_tcp_from_compliance_center_surface;
          }

          // Assigning one updated component in the selected command frame [m].
          if (std::isfinite(center_mm)) {
            Vec3 center_ee = -(R_EE.transpose() * r_c_base);
            center_ee(i) = 0.001 * center_mm;
            r_c_base = -(R_EE * center_ee);
          }
          if (std::isfinite(rc_mm)) {
            Vec3 rc_surface = R_base_surface.transpose() * r_c_base;
            rc_surface(i) = 0.001 * rc_mm;
            r_c_base = R_base_surface * rc_surface;
          }

          // Storing the updated center in the configured representation [m].
          if (params.compliance_center_in_tool_frame) {
            params.compliance_center_offset_ee = -(R_EE.transpose() * r_c_base);
          } else if (params.compliance_lever_in_surface_frame) {
            params.r_tcp_from_compliance_center_surface =
                R_base_surface.transpose() * r_c_base;
          }
          setup_impedance_changed = true;
        }
      }
    }
    if (setup_impedance_changed) {
      gains = buildPhaseImpedanceGains(params);
      damping = manualPhaseDampingCache(gains);
      setup_law_printed = false;  // reprinted with the new gains
    }
    // Reporting phase-gate impedance on its first active cycle.
    if (pause_hold_active && !gate_block_printed) {
      printGateHold(params, damping);
      gate_block_printed = true;
    }
    if (!pause_hold_active) {
      gate_block_printed = false;
    }

    // Printing each phase block after active damping matrices are available.
    if (intro_printed_for != phase) {
      printPhaseHeader(phase);
      printPhaseIntro(params, damping, phase);
      intro_printed_for = phase;
    }

    const bool wants_setup_block =
        phase == ControlPhase::kSetup ||
        (phase == ControlPhase::kPoseHold && params.use_setup_impedance_hold);
    if (wants_setup_block && !setup_law_printed) {
      printSetupImpedanceLaw(params, damping,
                             phase == ControlPhase::kPoseHold,
                             R_base_surface, R_EE);
      setup_law_printed = true;
    }
    if (!wants_setup_block) {
      // Resetting the report flag before the next setup entry.
      setup_law_printed = false;
    }
    // Calculating the nullspace torque selected by nullspace_mode [N m].
    const Vec7 tau_nullspace =
        computeNullspaceTorque(params, model, state, J, dq);

    // Initializing the optional joint-torque disturbance [N m].
    AutomaticDisturbance disturbance;
    if (phase == ControlPhase::kPoseHold && params.disturbance_auto_enabled) {
      disturbance = computeAutomaticDisturbance(
          params, model, state, disturbance_force_direction_base,
          time - phase_start_time);
    }

    // Loading Coriolis torque [N m] and calculating task torque J^T W [N m].
    Array7 coriolis_array = model.coriolis(state);
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
      row.phase = static_cast<int>(phase);
      row.nullspace_mode = static_cast<int>(params.nullspace_mode);
      row.p_EE = p_EE;
      row.p_d = desired.p_d;
      row.tool_contact_point = tool_contact_point;
      row.first_contact_tcp = first_contact_tcp;
      row.first_contact_point = first_contact_point;
      row.edge_target = edge_target_log;
      row.tool_contact_offset_ee = tool_contact_offset_ee;
      row.e_p = e_p;
      row.e_R = e_R;
      row.angular_deviation_surface =
          R_base_surface.transpose() *
          toolSurfaceAlignmentErrorBase(params, R_EE, R_base_surface);
      row.angular_deviation = row.angular_deviation_surface.norm();
      row.pdot = pdot;
      row.pdot_d = desired.pdot_d;
      row.omega = omega;
      row.f = f;
      row.m = m;
      row.external_force = external_force;
      row.external_moment = external_moment;
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
