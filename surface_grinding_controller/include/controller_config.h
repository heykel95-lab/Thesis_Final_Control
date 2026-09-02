// ============================================================================
// Controller configuration
// ============================================================================
// Defines the complete configuration loaded from params/*.conf. Each field
// states its physical unit and coordinate frame where these are relevant.
#pragma once

#include "controller_types.h"

/// Stores controller parameters and the run mode selected by the operator.
/// Parameter files are loaded before each session and assigned to this object.
struct ControllerConfig {
  // --------------------------------------------------------------------------
  // Robot connection, logging, and terminal output
  // --------------------------------------------------------------------------
  std::string robot_ip = "172.16.0.2";  // Franka Control Interface address.
  double experiment_duration = 0.0;     // Run duration [s]; 0 selects operator stop.
  std::string csv_file_name = "surface_grinding_controller_log.csv";
  int log_every_n_cycles = 5;           // Logging interval [control cycles].
  int max_log_rows = 120000;            // Maximum stored samples [-].
  double debug_period = 0.20;            // Terminal update period [s].
  bool print_hold_debug = true;          // Selector: false = silent; true = print hold state.
  bool print_grind_debug = true;         // Selector: false = silent; true = print grinding state.
  bool print_compliance_diagnostics = true;  // Selector for compliance-center output.

  // --------------------------------------------------------------------------
  // Gripper startup action
  // --------------------------------------------------------------------------
  bool perform_gripper_action_before_run = true;  // Selector for the configured action.
  bool abort_on_gripper_action_failure = true;    // Selector for failure handling.
  bool gripper_startup_grasp_tool = false;        // false = open; true = grasp tool.
  double gripper_open_width = 0.08;                // Commanded finger width [m].
  double gripper_open_speed = 0.05;                // Finger speed [m/s].
  double gripper_grasp_width = 0.02;               // Commanded tool width [m].
  double gripper_grasp_speed = 0.05;               // Finger speed [m/s].
  double gripper_grasp_force = 40.0;               // Grasping force [N].
  double gripper_grasp_epsilon_inner = 0.005;      // Inner width tolerance [m].
  double gripper_grasp_epsilon_outer = 0.010;      // Outer width tolerance [m].
  bool startup_gripper_action_completed = false;   // Set after the selected action succeeds.

  // --------------------------------------------------------------------------
  // Run selection and manual guidance
  // --------------------------------------------------------------------------
  bool run_state_sequence = true;          // Selects the complete state sequence.
  bool use_contact_impedance_hold = false;   // Selects contact impedance at the captured pose.
  bool enable_orientation_state = true;    // Selects the initial tool-orientation state.
  bool start_with_manual_guidance = false; // Selects manual guidance at session start.
  double manual_guidance_damping = 0.5;    // Joint damping [N m s/rad].

  // --------------------------------------------------------------------------
  // Pose-hold impedance
  // --------------------------------------------------------------------------
  // Translation: Kp [N/m], Dp [N s/m], base axes [x, y, z].
  // Rotation: KR [N m/rad], DR [N m s/rad], base axes [x, y, z].
  Vec3 hold_Kp_diag = Vec3::Constant(300.0);
  Vec3 hold_Dp_diag = Vec3::Constant(50.0);
  Vec3 hold_KR_diag = Vec3::Constant(40.0);
  Vec3 hold_DR_diag = Vec3::Constant(8.0);
  bool hold_auto_damping = true;               // Selector for inertia-based damping.
  bool hold_auto_match_manual_damping = true;  // Selector for the manual lower bound.
  double hold_auto_damping_factor = 1.0;        // Damping ratio [-].

  // --------------------------------------------------------------------------
  // Surface frame and tool geometry
  // --------------------------------------------------------------------------
  bool use_start_as_surface_point = true;  // false = configured point; true = start TCP.
  Vec3 surface_point = Vec3(0.0, 0.0, 0.0);       // Surface point in base frame [m].
  Vec3 surface_normal_base = Vec3(0.0, 0.0, 1.0); // Surface-normal direction in base [-].
  double surface_tilt_x_deg = 0.0;                 // Surface rotation about base x [deg].
  double surface_tilt_y_deg = 0.0;                 // Surface rotation about base y [deg].
  Vec3 surface_tangent1_hint_base = Vec3(0.0, 1.0, 0.0); // Tangent hint in base [-].

  // Tool-orientation offsets resolved in surface axes [deg].
  double tool_target_offset_tangent1_deg = 0.0;
  double tool_target_offset_tangent2_deg = 0.0;
  double tool_target_offset_normal_deg = 0.0;
  bool command_tool_twist = false;  // false = preserve start twist; true = command normal offset.
  Vec3 tool_axis_ee = Vec3(0.0, 0.0, 1.0);  // Tool-axis direction in EE frame [-].
  double tool_axis_target_sign = -1.0;       // Target direction sign relative to surface normal [-].

  bool use_tool_contact_point_control = true;  // Selects TCP or tool-feature position control.
  bool auto_select_tool_contact_edge = true;   // Selects the leading feature from geometry.
  Vec3 tool_contact_face_center_ee = Vec3(0.0, 0.0, 0.0); // Face center in EE [m].
  Vec3 tool_contact_half_width_ee = Vec3(0.0, 0.0, 0.0);  // Half-width vector in EE [m].
  Vec3 tool_contact_half_length_ee = Vec3(0.0, 0.0, 0.0); // Half-length vector in EE [m].
  double tool_contact_feature_tie_tolerance = 0.0001;      // Feature tie tolerance [m].

  // Rotational spring selectors in surface axes [tangent1, tangent2, normal].
  bool constrain_rotation_about_alignment_normal = true;
  bool constrain_rotation_about_alignment_tangent1 = true;
  bool constrain_rotation_about_alignment_tangent2 = true;

  // --------------------------------------------------------------------------
  // Controller disturbance test
  // --------------------------------------------------------------------------
  bool disturbance_cues_enabled = false;  // Selector for operator timing cues.
  double disturbance_push_time = 5.0;     // Push cue time [s].
  double disturbance_hold_time = 7.0;     // Hold cue time [s].
  double disturbance_release_time = 8.0;  // Release cue time [s].
  bool disturbance_auto_enabled = false;  // Selector for automatic point-force torque.
  int disturbance_link = 4;               // libfranka link index [-].
  Vec3 disturbance_point_link = Vec3::Zero(); // Force point in selected link frame [m].
  double disturbance_force = 0.0;             // Force magnitude [N].
  double disturbance_direction_sign = 1.0;    // Direction multiplier [-].
  double disturbance_release_ramp_time = 1.0; // Release ramp duration [s].
  double disturbance_max_tau_norm = 0.0;      // Joint-torque norm limit [N m].

  // --------------------------------------------------------------------------
  // Tool orientation and surface approach states
  // --------------------------------------------------------------------------
  double approach_orient_min_time = 0.5;                // Minimum orientation time [s].
  double approach_orient_error_threshold = 0.03;        // Tool-axis threshold [rad].
  double approach_orient_spin_error_threshold = 0.009;  // Tool-spin threshold [rad].
  double approach_orient_timeout = 5.0;                 // Orientation timeout [s].
  double approach_orient_max_rate_deg = 20.0;           // Desired rotation-rate limit [deg/s].

  // Surface-frame gains [tangent1, tangent2, normal].
  // Kp [N/m], Dp [N s/m], KR [N m/rad], DR [N m s/rad].
  Vec3 approach_Kp_diag = Vec3(150.0, 150.0, 150.0);
  Vec3 approach_KR_diag = Vec3(90.0, 90.0, 8.0);
  Vec3 approach_Dp_diag = Vec3(20.0, 20.0, 20.0);
  Vec3 approach_DR_diag = Vec3(12.0, 12.0, 12.0);
  bool approach_auto_damping = false;           // Selector for inertia-based damping.
  double approach_auto_damping_factor = 1.0;    // Damping ratio [-].
  double descend_speed = 0.005;                  // Surface-approach speed [m/s].
  double descend_max_distance = 0.02;            // Maximum approach travel [m].
  double descend_surface_clearance = 0.020;      // Tool-feature clearance threshold [m].

  // --------------------------------------------------------------------------
  // Contact-establishment state
  // --------------------------------------------------------------------------
  double contact_establishment_min_time = 0.3;           // Minimum contact establishment duration [s].
  double contact_establishment_timeout = 15.0;           // State timeout [s].
  double contact_establishment_moment_threshold = 60.0;  // Moment-change threshold [N m].
  double contact_establishment_push_speed = 0.0;         // Virtual penetration rate [m/s].
  double contact_establishment_push_end = 0.0;           // Final virtual penetration [m].
  // Offline alignment criterion. It is observed and logged only: reaching it
  // does not end the state, so the run still lasts its configured duration.
  double contact_establishment_align_tolerance_deg = 2.0; // Deviation counted as aligned [deg].
  double contact_establishment_align_hold_time = 0.3;     // Time it must stay inside it [s].
  // A second, relative criterion. An absolute tolerance is only reached by the
  // conditions that align well, so it says nothing about the ones that do not.
  // This one is measured against the deviation the trial started from, which
  // is known as soon as contact is made, and is therefore comparable across
  // conditions that end far apart.
  double contact_establishment_align_fraction = 0.5;      // Fraction of the initial deviation.

  // Base-frame translational gains [x, y, z].
  Vec3 contact_establishment_Kp_diag = Vec3(40.0, 40.0, 5500.0); // [N/m].
  Vec3 contact_establishment_Dp_diag = Vec3(10.0, 10.0, 175.0);  // [N s/m].
  bool contact_establishment_translation_surface_frame = false;   // false = base gains; true = surface gains.

  // Surface-frame translational gains [tangent1, tangent2, normal].
  Vec3 contact_establishment_Kp_surface_diag = Vec3(2000.0, 2000.0, 360.0); // [N/m].
  Vec3 contact_establishment_Dp_surface_diag = Vec3(50.0, 50.0, 25.0);      // [N s/m].

  // Surface-frame rotational gains [tangent1, tangent2, normal].
  Vec3 contact_establishment_KR_diag = Vec3(0.0, 0.0, 8.0);   // [N m/rad].
  Vec3 contact_establishment_DR_diag = Vec3(0.01, 0.01, 4.0); // [N m s/rad].
  bool contact_establishment_auto_damping = false;            // Selector for inertia-based damping.
  double contact_establishment_auto_damping_factor = 1.0;      // Damping ratio [-].

  // --------------------------------------------------------------------------
  // Grinding state
  // --------------------------------------------------------------------------
  bool grind_sweep_enabled = false; // false = stationary contact; true = tangential sweep.
  int grind_tangent_axis = 1;       // Selector: 1 = tangent1; 2 = tangent2.
  double grind_amplitude_m = 0.03;  // Sweep half-amplitude [m].
  double grind_frequency_hz = 0.2;  // Sweep frequency [Hz].

  // --------------------------------------------------------------------------
  // Operator-controlled hold states
  // --------------------------------------------------------------------------
  bool enable_pre_contact_hold = false;  // Enables the hold before contact establishment.
  bool enable_pre_grinding_hold = false; // Enables the hold before grinding.

  // Base-frame translational gains [x, y, z].
  Vec3 operator_hold_Kp_diag = Vec3::Constant(5000.0); // [N/m].
  Vec3 operator_hold_Dp_diag = Vec3::Constant(200.0);  // [N s/m].
  bool operator_hold_translation_surface_frame = false; // false = base gains; true = surface gains.

  // Surface-frame translational gains [tangent1, tangent2, normal].
  Vec3 operator_hold_Kp_surface_diag = Vec3::Constant(5000.0); // [N/m].
  Vec3 operator_hold_Dp_surface_diag = Vec3::Constant(200.0);  // [N s/m].

  // Base-frame rotational gains [x, y, z].
  Vec3 operator_hold_KR_diag = Vec3::Constant(90.0); // [N m/rad].
  Vec3 operator_hold_DR_diag = Vec3::Constant(12.0); // [N m s/rad].
  bool operator_hold_rotation_surface_frame = true;  // false = base gains; true = surface gains.

  // Surface-frame rotational gains [tangent1, tangent2, normal].
  Vec3 operator_hold_KR_surface_diag = Vec3::Constant(90.0); // [N m/rad].
  Vec3 operator_hold_DR_surface_diag = Vec3::Constant(12.0); // [N m s/rad].
  bool operator_hold_auto_damping = true; // Selector for inertia-based damping.

  // --------------------------------------------------------------------------
  // Virtual center of compliance
  // --------------------------------------------------------------------------
  bool use_virtual_compliance_center = false; // Selector for gain shifting to the TCP.

  // Tool-frame definition: p_C = p_TCP + R_EE * offset [m].
  bool compliance_center_in_tool_frame = false;
  Vec3 compliance_center_offset_ee = Vec3::Zero(); // Center offset in EE frame [m].

  // Surface-frame definition: stores r_c = p_C - p_TCP [m].
  bool compliance_lever_in_surface_frame = false;
  Vec3 compliance_lever_surface = Vec3::Zero(); // [t1, t2, n], [m].

  // --------------------------------------------------------------------------
  // Nullspace control
  // --------------------------------------------------------------------------
  NullspaceMode nullspace_mode = NullspaceMode::kDampingAndSigma; // Active contributions.
  double nullspace_damping = 1.0;                  // Damping coefficient [N m s/rad].
  double nullspace_sigma_gain = 1.0;               // Conditioning torque gain [N m].
  double nullspace_probe_step_rad = 0.03;          // Two-sided posture probe [rad].
  double nullspace_sigma_deadband = 1e-6;          // Singular-value difference [-].
  double nullspace_svd_relative_tolerance = 1e-4;  // SVD cutoff relative to sigma_max [-].

  // --------------------------------------------------------------------------
  // Automatic damping
  // --------------------------------------------------------------------------
  double auto_damping_max = 8000.0;        // Maximum diagonal damping coefficient.
  bool auto_damping_min_from_manual = false; // Selector for manual damping lower bounds.

  // --------------------------------------------------------------------------
  // Initial and tool-pickup postures
  // --------------------------------------------------------------------------
  Array7 q_init = {{
      0.0,
      -M_PI_4,
      0.0,
      -3.0 * M_PI_4,
      0.0,
      M_PI_2,
      0.0
  }};  // Initial joint angles [rad].
  std::string q_init_case = "horizontal_tool"; // Descriptive posture label.

  bool use_tool_pickup = false; // Selector for the pickup sequence.
  Array7 q_pickup = {{0.0, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, 0.0}}; // [rad].
  double pickup_standoff = 0.05;             // Tool-axis retreat distance [m].
  double pickup_descend_speed_factor = 0.15; // MotionGenerator speed factor [-].

  // --------------------------------------------------------------------------
  // Collision thresholds
  // --------------------------------------------------------------------------
  bool use_custom_collision_behavior = false; // Selector for configured thresholds.
  double collision_torque_acc = 80.0;  // Acceleration-state joint threshold [N m].
  double collision_torque_nom = 80.0;  // Nominal joint threshold [N m].
  double collision_force_acc = 80.0;   // Acceleration Cartesian threshold [N or N m].
  double collision_force_nom = 80.0;   // Nominal Cartesian threshold [N or N m].
};
