// ============================================================================
// Operator interface
// ============================================================================
// Defining the startup menu, gripper actions, tool transfer, and
// gravity-compensated manual guidance.
#include "controller_api.h"


namespace {

std::string readChoice() {
  // Reading one complete menu line from standard input.
  std::string line;
  if (!std::getline(std::cin, line)) {
    return "";
  }
  // Removing leading and trailing whitespace from the selection.
  line.erase(line.begin(),
             std::find_if(line.begin(), line.end(), [](unsigned char c) {
               return !std::isspace(c);
             }));
  line.erase(std::find_if(line.rbegin(), line.rend(), [](unsigned char c) {
               return !std::isspace(c);
             }).base(),
             line.end());
  // Converting the selection to lowercase for uniform comparison.
  std::transform(line.begin(), line.end(), line.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return line;
}

bool matches(const std::string& choice, std::initializer_list<const char*> words) {
  // Comparing the normalized selection with accepted aliases.
  for (const char* word : words) {
    if (choice == word) {
      return true;
    }
  }
  return false;
}

NullspaceMode askHoldNullspaceMode(NullspaceMode configured,
                                   bool* stop_selected = nullptr) {
  if (stop_selected != nullptr) {
    *stop_selected = false;
  }
  const NullspaceMode fallback = configured;
  printSection("NULLSPACE CONTROL");
  printf("  %-5s  %s\n", "0", "Disabled");
  printf("  %-5s  %s\n", "1", "Nullspace damping");
  printf("  %-5s  %s\n", "2", "Singular-value conditioning");
  printf("  %-5s  %s\n", "3", "Combined damping and conditioning");
  if (stop_selected != nullptr) {
    printf("  %-5s  %s\n", "e", "Stop controller");
  }
  printRule();
  printf("Selection [0/1/2/3, Enter = %s]: ",
         fallback == NullspaceMode::kOff ? "0"
             : fallback == NullspaceMode::kSigmaOnly ? "2"
             : fallback == NullspaceMode::kDampingAndSigma ? "3" : "1");

  const std::string choice = readChoice();
  if (matches(choice, {"0", "off", "none"})) {
    return NullspaceMode::kOff;
  }
  if (matches(choice, {"1", "tau_nullspace", "nullspace", "damping",
                       "posture"})) {
    return NullspaceMode::kDampingOnly;
  }
  if (matches(choice, {"2", "tau_sigma", "sigma"})) {
    return NullspaceMode::kSigmaOnly;
  }
  if (matches(choice, {"3", "both", "combined", "sigma+damping",
                       "damping+sigma"})) {
    return NullspaceMode::kDampingAndSigma;
  }
  if (stop_selected != nullptr && matches(choice, {"e", "stop", "exit"})) {
    *stop_selected = true;
  }
  return fallback;
}

bool selectHoldNullspaceMode(ControllerConfig& params,
                             bool* stop_selected = nullptr) {
  const NullspaceMode selected =
      askHoldNullspaceMode(params.nullspace_mode, stop_selected);
  if (stop_selected != nullptr && *stop_selected) {
    return false;
  }
  params.nullspace_mode = selected;
  return true;
}

}  // namespace

bool gripperWidthCalibrated(const franka::GripperState& state,
                            const ControllerConfig& params) {
  // Defining the accepted width verification tolerance [m].
  const double width_tolerance = 0.002;
  const double commanded =
      std::max(params.gripper_open_width, params.gripper_grasp_width);
  return state.max_width + width_tolerance >= commanded;
}

void reportGripperCalibration(const franka::GripperState& state,
                              const ControllerConfig& params) {
  if (gripperWidthCalibrated(state, params)) {
    return;
  }
  const double commanded =
      std::max(params.gripper_open_width, params.gripper_grasp_width);
  fprintf(stderr,
          "\nHand width calibration is invalid: stroke %.1f mm, but this "
          "program commands up to %.1f mm.\n",
          1000.0 * state.max_width,
          1000.0 * commanded);
  fprintf(stderr,
          "The last homing measured only the travel left beside the tool, so "
          "move() and grasp() are clamped to that stroke and the fingers "
          "barely move.\n");
  fprintf(stderr,
          "Select r in the startup menu with empty fingers. "
          "Perform homing only with an empty gripper.\n\n");
}

bool openGripper(const ControllerConfig& params, Gripper& gripper) {
  try {
    // Reading the initial gripper width and calibrated stroke [m].
    const franka::GripperState before = gripper.readOnce();
    reportGripperCalibration(before, params);
    printSection("gripper: open");
    printf("  %-16s   %.1f mm\n", "target", 1000.0 * params.gripper_open_width);
    // Commanding opening width [m] at the configured speed [m/s].
    const bool opened = gripper.move(params.gripper_open_width, params.gripper_open_speed);
    // Reading the resulting gripper state for command verification.
    const franka::GripperState after = gripper.readOnce();
    // Defining the accepted width verification tolerance [m].
    const double width_tolerance = 0.002;
    const bool calibration_supports_target =
        after.max_width + width_tolerance >= params.gripper_open_width;
    const bool target_reached =
        after.width + width_tolerance >= params.gripper_open_width;
    const bool verified =
        opened && calibration_supports_target && target_reached;

    printf("  %-16s   %.1f -> %.1f mm, reported max %.1f mm\n", "width",
           1000.0 * before.width,
           1000.0 * after.width,
           1000.0 * after.max_width);
    if (verified) {
      printf("  %-16s   open, width verified\n", "result");
    } else if (!calibration_supports_target) {
      reportGripperCalibration(after, params);
    } else {
      fprintf(stderr,
              "Gripper did not reach the requested open width. Support/remove "
              "the tool and select r to recalibrate the hand.\n");
    }
    return verified;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper open failed: %s\n", e.what());
    return false;
  }
}

bool graspTool(const ControllerConfig& params, Gripper& gripper) {
  try {
    // Reading the initial gripper state before applying grasp force.
    const franka::GripperState before = gripper.readOnce();
    reportGripperCalibration(before, params);
    printSection("gripper: grasp the tool");
    printf("  %-16s   %.1f mm at %.1f N\n", "target",
           1000.0 * params.gripper_grasp_width, params.gripper_grasp_force);
    // Commanding grasp width [m], speed [m/s], force [N], and tolerances [m].
    const bool grasped = gripper.grasp(
        params.gripper_grasp_width, params.gripper_grasp_speed,
        params.gripper_grasp_force, params.gripper_grasp_epsilon_inner,
        params.gripper_grasp_epsilon_outer);
    // Reading the resulting gripper state for command verification.
    const franka::GripperState after = gripper.readOnce();
    const bool width_in_band =
        after.width >=
            params.gripper_grasp_width - params.gripper_grasp_epsilon_inner &&
        after.width <=
            params.gripper_grasp_width + params.gripper_grasp_epsilon_outer;
    const bool verified = grasped && after.is_grasped && width_in_band;

    printf("  %-16s   %.1f -> %.1f mm, grasped flag %s\n", "width",
           1000.0 * before.width,
           1000.0 * after.width,
           after.is_grasped ? "yes" : "no");
    if (verified) {
      printf("  %-16s   closed on the tool, grasp verified\n", "result");
    } else {
      fprintf(stderr,
              "Tool grasp was not verified. Support the tool, check its "
              "placement, and select r if the fingers did not travel.\n");
    }
    return verified;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper grasp failed: %s\n", e.what());
    return false;
  }
}

bool saveGuidedPoseAsQInit(const Vec7& q) {
  // Selecting the initial-pose file updated by guidance mode q.
  const std::string path =
      defaultParameterDirectory() + "/initial_pose.conf";
  std::ifstream input(path);
  if (!input) {
    fprintf(stderr, "Could not open %s to save the pose.\n", path.c_str());
    return false;
  }

  // Reading the complete file while preserving comments and ordering.
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }
  input.close();

  // Assigning the seven guided joint angles [rad] and selecting saved_qinit.
  int replaced = 0;
  bool case_set = false;
  for (std::string& text : lines) {
    const std::string trimmed = trim(text);
    if (trimmed.rfind("q_init_case", 0) == 0) {
      text = "q_init_case = saved_qinit";
      case_set = true;
      continue;
    }
    for (int i = 0; i < 7; ++i) {
      const std::string key = "q_init_saved_" + std::to_string(i + 1);
      if (trimmed.rfind(key + " ", 0) == 0 || trimmed.rfind(key + "=", 0) == 0) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%s = %.6f", key.c_str(), q(i));
        text = buffer;
        ++replaced;
        break;
      }
    }
  }
  if (replaced != 7 || !case_set) {
    fprintf(stderr,
            "%s is missing q_init_case or a q_init_saved_* line "
            "(found %d of 7). Nothing was written.\n",
            path.c_str(), replaced);
    return false;
  }

  std::ofstream output(path);
  if (!output) {
    fprintf(stderr, "Could not write %s.\n", path.c_str());
    return false;
  }
  for (const std::string& text : lines) {
    output << text << "\n";
  }
  printf("Saved this pose as q_init_case = saved_qinit in %s.\n", path.c_str());
  printVec7Deg("saved q1..q7", q);
  return true;
}

bool recalibrateGripper(const ControllerConfig& params, Gripper& gripper) {
  try {
    const franka::GripperState before = gripper.readOnce();
    printBanner("FRANKA HAND WIDTH RECALIBRATION");
    printf("  %-16s   %.1f mm, reported max %.1f mm, grasped %s\n", "now",
           1000.0 * before.width,
           1000.0 * before.max_width,
           before.is_grasped ? "yes" : "no");
    reportGripperCalibration(before, params);
    printRule();
    printf("  This opens the fingers completely. A held tool WILL FALL.\n");
    printf("  Support the tool by hand or remove it before continuing.\n");
    printRule();

    // Confirming gripper recalibration only with an empty input.
    printf("Press Enter to recalibrate, anything else to abort: ");
    fflush(stdout);
    const std::string answer = readChoice();
    if (!answer.empty()) {
      printf("Read \"%s\", not a bare Enter. Aborted; nothing was moved.\n",
             answer.c_str());
      return false;
    }

    const bool homed = gripper.homing();
    // Reading the resulting gripper state for command verification.
    const franka::GripperState after = gripper.readOnce();
    // Defining the accepted width verification tolerance [m].
    const double width_tolerance = 0.002;
    const bool calibration_supports_target =
        after.max_width + width_tolerance >= params.gripper_open_width;
    const bool opened =
        after.width + width_tolerance >= params.gripper_open_width;
    const bool verified = homed && calibration_supports_target && opened;

    printf("  %-16s   %.1f mm, reported max %.1f mm\n", "after",
           1000.0 * after.width,
           1000.0 * after.max_width);
    if (verified) {
      printf("  %-16s   open; place the tool and select c\n", "result");
    } else {
      fprintf(stderr,
              "Recalibration was not verified. Do not start the robot "
              "experiment; inspect the Franka Hand state.\n");
    }
    return verified;
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper recalibration failed: %s\n", e.what());
    return false;
  }
}

namespace {

// Transferring the tool through q_init and the configured pickup posture.
bool runToolPickup(const ControllerConfig& params,
                   Robot& robot,
                   const Model& model,
                   bool grasp,
                   const std::function<void()>& ensure_q_init) {
  if (!params.use_tool_pickup) {
    printf("Tool transfer requires a configured pickup posture. Guide the "
           "robot to the holder, save q_pickup_* in params/gripper.conf, "
           "and set use_tool_pickup = 1.\n");
    return false;
  }

  Gripper gripper(params.robot_ip);
  const franka::GripperState state = gripper.readOnce();
  if (grasp && state.is_grasped) {
    printf("The hand is already holding something (width %.1f mm).\n"
           "Press b to put it back, or o to release it, before fetching.\n",
           1000.0 * state.width);
    return false;
  }
  if (!grasp && !state.is_grasped) {
    printf("The hand does not report a grasp; going to the pickup posture "
           "and opening anyway.\n");
  }

  // Calculating the pickup standoff along -Z_EE [m].
  Array7 q_above{};
  const RobotState state_now = robot.readOnce();
  if (!solveStandoffPosture(model, state_now, params.q_pickup,
                            params.pickup_standoff, q_above)) {
    printf("Could not solve a stand-off posture %.0f mm back along -Z_EE.\n"
           "Reduce pickup_standoff or re-measure q_pickup_*.\n",
           1000.0 * params.pickup_standoff);
    return false;
  }
  int bad_joint = 0;
  if (!withinJointLimits(q_above, bad_joint)) {
    printf("The stand-off posture puts joint %d outside its limit. "
           "Reduce pickup_standoff.\n", bad_joint);
    return false;
  }

  printf("\nStand-off %.0f mm back along -Z_EE from the pickup pose:\n",
         1000.0 * params.pickup_standoff);
  printf("  q [deg] = [");
  for (int i = 0; i < 7; ++i) {
    printf("%s%.1f", i ? ", " : "", 180.0 / M_PI * q_above[i]);
  }
  printf("]\n");
  printf("Path: q_init -> stand-off -> down onto the tool -> %s -> lift.\n",
         grasp ? "grasp" : "release");

  // Opening the gripper before approaching the tool for pickup.
  if (grasp && !openGripper(params, gripper)) {
    printf("Not moving: the hand did not reach the open width.\n");
    return false;
  }

  ensure_q_init();
  printf("Moving above the tool...\n");
  MotionGenerator to_above(0.4, q_above);
  robot.control(to_above);

  printf("Descending onto the tool...\n");
  MotionGenerator descend(std::max(0.05, params.pickup_descend_speed_factor),
                          params.q_pickup);
  robot.control(descend);

  const bool ok = grasp ? graspTool(params, gripper)
                        : openGripper(params, gripper);

  // Returning directly to the standoff posture after the gripper action.
  printf("Lifting clear of the holder...\n");
  MotionGenerator lift(std::max(0.05, params.pickup_descend_speed_factor),
                       q_above);
  robot.control(lift);

  if (ok) {
    printf(grasp ? "Tool fetched. The arm waits above the holder.\n"
                 : "Tool released. The arm waits above the holder.\n");
  }
  return ok;
}

}  // namespace

bool askStartupRunMode(ControllerConfig& params, Robot& robot,
                       const Model& model) {
  bool q_init_reached = false;

  const auto move_to_q_init = [&]() {
    const Vec7 q_init = Map<const Vec7>(params.q_init.data());
    printSection("moving to q_init");
    printf("  %-16s   %s\n", "case", params.q_init_case.c_str());
    printVec7Deg("q1..q7 [deg]", q_init);
    // Printing the selected joint posture [rad] for configuration checks.
    printf("  %s = [", params.q_init_case.c_str());
    for (int i = 0; i < 7; ++i) {
      printf("%s%.6f", i ? ", " : "", q_init(i));
    }
    printf("] rad\n");
    MotionGenerator motion_generator(0.4, params.q_init);
    robot.control(motion_generator);
    printf("  q_init reached.\n");
    q_init_reached = true;
  };

  const auto ensure_q_init = [&]() {
    if (!q_init_reached) {
      move_to_q_init();
    }
  };

  // Presenting run modes and robot-preparation actions in separate groups.
  while (true) {
    printBanner("SURFACE GRINDING CONTROLLER - STARTUP MENU");

    printSection("RUN MODES");
    printf("  %-5s  %s\n", "s",
           "Run orientation, approach, contact establishment, and grinding");
    printf("  %-5s  %s\n", "h", "Cartesian pose hold at the initial posture");
    printf("  %-5s  %s\n", "t", "Cartesian pose hold with the contact establishment impedance");
    printf("  %-5s  %s\n", "g",
           "Start manual guidance from the current robot pose");

    printSection("ROBOT PREPARATION");
    printf("  %-5s  %s\n", "i", "Move to the configured initial joint posture");

    printSection("GRIPPER AND TOOL");
    printf("  %-5s  %s\n", "o", "Open the gripper");
    printf("  %-5s  %s\n", "c", "Grasp the tool");
    printf("  %-5s  %s\n", "r", "Recalibrate the gripper with empty fingers");
    printf("  %-5s  %s\n", "f", "Pick up the tool from the holder");
    printf("  %-5s  %s\n", "b", "Return the tool to the holder");

    printSection("PROGRAM");
    printf("  %-5s  %s\n", "e", "Exit");
    printRule();
    printf("  During control: e = stop | m = startup menu | g = guidance\n");
    printRule();
    printf("Selection [s/h/t/g/i/o/c/r/f/b/e]: ");

    const std::string choice = readChoice();
    if (matches(choice, {"s", "sequence"})) {
      printSection("SELECTED MODE: STATE SEQUENCE");
      ensure_q_init();
      params.start_with_manual_guidance = false;
      params.run_state_sequence = true;
      params.use_contact_impedance_hold = false;
      break;
    }
    if (matches(choice, {"h", "hold"})) {
      ensure_q_init();
      params.start_with_manual_guidance = false;
      params.run_state_sequence = false;
      params.use_contact_impedance_hold = false;
      break;
    }
    if (matches(choice, {"t", "test", "contact establishment"})) {
      ensure_q_init();
      params.start_with_manual_guidance = false;
      params.run_state_sequence = false;
      params.use_contact_impedance_hold = true;
      break;
    }
    if (matches(choice, {"i", "init", "qinit"})) {
      move_to_q_init();
      continue;
    }
    if (matches(choice, {"o", "open"})) {
      try {
        Gripper gripper(params.robot_ip);
        openGripper(params, gripper);
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper connection failed: %s\n", e.what());
      }
      // Preserving the gripper state selected by the operator.
      params.startup_gripper_action_completed = true;
      continue;
    }
    if (matches(choice, {"c", "close", "grasp"})) {
      try {
        Gripper gripper(params.robot_ip);
        graspTool(params, gripper);
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper connection failed: %s\n", e.what());
      }
      params.startup_gripper_action_completed = true;
      continue;
    }
    if (matches(choice, {"r", "recal", "recalibrate"})) {
      try {
        Gripper gripper(params.robot_ip);
        recalibrateGripper(params, gripper);
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Gripper connection failed: %s\n", e.what());
      }
      // Preserving the gripper state selected by the operator.
      params.startup_gripper_action_completed = true;
      continue;
    }
    if (matches(choice, {"f", "fetch", "tool"}) ||
        matches(choice, {"b", "back", "putback"})) {
      const bool grasp = matches(choice, {"f", "fetch", "tool"});
      try {
        if (runToolPickup(params, robot, model, grasp, ensure_q_init)) {
          // Preserving the deliberate hand command and invalidating the q_init state.
          params.startup_gripper_action_completed = true;
        }
      } catch (const franka::Exception& e) {
        fprintf(stderr, "Tool pickup failed: %s\n", e.what());
      }
      q_init_reached = false;
      continue;
    }
    if (matches(choice, {"e", "stop", "quit", "exit"})) {
      printf("Stopping without starting a run.\n");
      return false;
    }
    if (matches(choice, {"g", "guide", "guiding"})) {
      printSection("SELECTED MODE: MANUAL GUIDANCE");
      printf("  Starting from the current robot pose.\n");
      params.start_with_manual_guidance = true;
      break;
    }
    if (choice.empty()) {
      printf("Choose s, h, t, g, i, o, c, r, f, b, or e explicitly.\n");
    } else {
      printf("Unknown startup choice '%s'; choose s, h, t, g, i, o, c, r, f, "
             "b, or e.\n", choice.c_str());
    }
  }

  // Selecting the subsequent run mode from the guided pose.
  if (params.start_with_manual_guidance) {
    printf("  Move the robot to the required pose, then select s, h, or t.\n");
    return true;
  }

  // Assigning the configured nullspace mode to the state sequence.
  if (params.run_state_sequence) {
    return true;
  }

  // Assigning the configured nullspace mode to contact-impedance hold.
  if (params.use_contact_impedance_hold) {
    printSection("SELECTED MODE: CONTACT-IMPEDANCE HOLD");
    printf("  %-16s   %s\n", "spring",
           params.use_virtual_compliance_center ? "coupled contact establishment impedance"
                                        : "decoupled contact establishment impedance");
    printf("  %-16s   %s, from nullspace.conf\n", "nullspace",
           nullspaceModeName(params.nullspace_mode));
    return true;
  }

  // Selecting the nullspace mode for Cartesian pose hold.
  (void)selectHoldNullspaceMode(params);
  printSection("SELECTED MODE: CARTESIAN POSE HOLD");
  printf("  %-16s   %s\n", "nullspace",
         nullspaceModeName(params.nullspace_mode));
  return true;
}

bool performStartupGripperAction(const ControllerConfig& params) {
  // Returning when no automatic startup action is requested.
  if (!params.perform_gripper_action_before_run || params.start_with_manual_guidance ||
      params.startup_gripper_action_completed) {
    return true;
  }

  bool gripper_ok = false;
  try {
    Gripper gripper(params.robot_ip);
    if (params.gripper_startup_grasp_tool) {
      // Retaining a verified grasp before issuing another gripper command.
      const franka::GripperState state = gripper.readOnce();
      const bool already_holding =
          state.is_grasped &&
          state.width >= params.gripper_grasp_width - params.gripper_grasp_epsilon_inner &&
          state.width <= params.gripper_grasp_width + params.gripper_grasp_epsilon_outer;
      if (already_holding) {
        printf("Gripper already holding the tool (width %.1f mm); keeping grasp.\n",
               1000.0 * state.width);
        gripper_ok = true;
      } else {
        gripper_ok = graspTool(params, gripper);
      }
    } else {
      gripper_ok = openGripper(params, gripper);
    }
  } catch (const franka::Exception& e) {
    fprintf(stderr, "Gripper action failed: %s\n", e.what());
    gripper_ok = false;
  }

  if (!gripper_ok && params.abort_on_gripper_action_failure) {
    fprintf(stderr, "Stopping because abort_on_gripper_action_failure = 1.\n");
    return false;
  }
  return true;
}

bool runManualGuidanceStart(ControllerConfig& params,
                            Robot& robot,
                            const Model& model,
                            KeyboardSignals& signals) {
  std::atomic<bool>& stop_requested = signals.stop_requested;
  std::atomic<char>& guidance_menu_key = signals.guidance_menu_key;
  std::atomic<bool>& guided_hold_selector_pending =
      signals.guided_hold_selector_pending;

  struct PendingFlagReset {
    std::atomic<bool>& flag;
    ~PendingFlagReset() {
      flag.store(false);
    }
  };

  // Reserving standard input for the manual-guidance selector.
  guided_hold_selector_pending.store(true);
  PendingFlagReset pending_flag_reset{guided_hold_selector_pending};

  // Initializing the joint posture printed on stop [rad].
  Vec7 stop_q = Vec7::Zero();
  const auto print_stop_pose = [&stop_q]() {
    printBanner("MANUAL GUIDANCE STOP POSE");
    printf("  paste into a q_init_* case:\n");
    for (int i = 0; i < 7; ++i) {
      printf("    q_init_%d = %.6f\n", i + 1, stop_q(i));
    }
    printVec7Deg("q1..q7 [deg]", stop_q);
    printRule();
  };

  while (true) {
    printSection("MANUAL GUIDANCE");
    printf("  Move the robot by hand and select an action.\n");
    printf("  %-5s  %s\n", "s", "Start the state sequence from the guided pose");
    printf("  %-5s  %s\n", "h", "Start Cartesian pose hold from the guided pose");
    printf("  %-5s  %s\n", "t",
           "Start contact-impedance hold from the guided pose");
    printf("  %-5s  %s\n", "q",
           "Save the guided pose in params/initial_pose.conf");
    printf("  %-5s  %s\n", "m", "Return to the startup menu");
    printf("  %-5s  %s\n", "e", "Stop and print the guided joint posture");
    printRule();

    // Applying Coriolis compensation and joint damping [N m] during guidance.
    robot.control([&](const RobotState& state, Duration /*period*/) -> Torques {
      // Mapping measured joint velocity [rad/s].
      Map<const Vec7> dq(state.dq.data());
      // Loading model Coriolis torque [N m].
      Array7 coriolis_array = model.coriolis(state);
      Map<const Vec7> coriolis(coriolis_array.data());
      // Calculating the guidance torque command [N m].
      const Array7 tau_array =
          vec7ToArray(Vec7(coriolis - params.manual_guidance_damping * dq));
      if (stop_requested.load()) {
        stop_q = Map<const Vec7>(state.q.data());
        return MotionFinished(Torques(tau_array));
      }
      if (guidance_menu_key.load() != 0) {
        return MotionFinished(Torques(tau_array));
      }
      return Torques(tau_array);
    });

    if (stop_requested.load()) {
      print_stop_pose();
      return false;
    }

    // Selecting the next mode from the pose reached during guidance.
    const char key = guidance_menu_key.exchange(0);
    if (key == 'q') {
      const RobotState saved_state = robot.readOnce();
      (void)saveGuidedPoseAsQInit(Map<const Vec7>(saved_state.q.data()));
    } else if (key == 's') {
      params.run_state_sequence = true;
      params.use_contact_impedance_hold = false;
      printf("Selected: state sequence from the guided pose.\n");
      return true;
    } else if (key == 't') {
      params.run_state_sequence = false;
      params.use_contact_impedance_hold = true;
      printf("Selected: contact-impedance hold from the guided pose.\n");
      return true;
    } else if (key == 'h') {
      params.run_state_sequence = false;
      params.use_contact_impedance_hold = false;
      bool stop_selected = false;
      if (!selectHoldNullspaceMode(params, &stop_selected)) {
        stop_requested.store(true);
        const RobotState stop_state = robot.readOnce();
        stop_q = Map<const Vec7>(stop_state.q.data());
        print_stop_pose();
        return false;
      }
      printf("Selected: hold at the guided pose with %s.\n",
             nullspaceModeName(params.nullspace_mode));
      return true;
    }
  }
}
