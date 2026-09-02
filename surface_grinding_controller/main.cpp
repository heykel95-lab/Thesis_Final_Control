// ============================================================================
// Program entry point
// ============================================================================
// Loading the controller configuration, preparing the robot, selecting the run
// mode, executing the control loop, and writing the run report.
#include "controller_api.h"

int main() {
  try {
    // ------------------------------------------------------------------------
    // 1. Robot and configuration initialization
    // ------------------------------------------------------------------------
    // Resolving the parameter directory relative to the executable location.
    const std::string parameter_directory = defaultParameterDirectory();

    // Defining the ordered configuration files read for every controller run.
    const std::vector<std::string> parameter_files =
        parameterFiles(parameter_directory);

    // Loading the startup configuration and connecting to the robot FCI.
    const ControllerConfig startup_config =
        readControllerConfig(parameter_files);
    Robot robot(startup_config.robot_ip);

    // Printing the active robot address and parameter directory.
    printBanner("SURFACE GRINDING CONTROLLER");
    printf("  %-16s   %s\n", "robot", startup_config.robot_ip.c_str());
    printf("  %-16s   %s\n", "parameters", parameter_directory.c_str());
    printRule();

    // Recovering the robot automatically before configuring the controller.
    try {
      robot.automaticErrorRecovery();
    } catch (const franka::Exception& e) {
      fprintf(stderr, "Automatic error recovery failed: %s\n", e.what());
      fprintf(stderr,
              "Recover and unlock the robot in Franka Desk, then restart.\n");
      return -1;
    }

    // Assigning collision thresholds and loading the dynamic robot model.
    configureCollisionBehavior(robot, startup_config);
    Model model = robot.loadModel();

    // ------------------------------------------------------------------------
    // 2. Operator input initialization
    // ------------------------------------------------------------------------
    // Initializing shared keyboard signals used by menus and the control loop.
    KeyboardSignals signals;
    startKeyboardStopThread(startup_config, signals);

    // ------------------------------------------------------------------------
    // 3. Session selection, execution, and reporting
    // ------------------------------------------------------------------------
    for (int session = 1;; ++session) {
      // Reloading all parameter files before each session.
      ControllerConfig config = readControllerConfig(parameter_files);

      // Assigning a separate CSV filename to repeated sessions.
      if (session > 1) {
        config.csv_file_name =
            sessionFileName(config.csv_file_name, session);
      }

      // Resetting operator requests before opening the startup menu.
      signals.stop_requested.store(false);
      signals.proceed_requested.store(false);
      signals.guide_requested.store(false);
      signals.guidance_menu_key.store(0);
      signals.operator_hold_continue.store(false);

      // Selecting the requested run mode and preparing its initial state.
      if (!askStartupRunMode(config, robot, model)) {
        break;
      }

      // Assigning the guidance selector used when mode g starts from this pose.
      signals.guided_hold_selector_pending.store(
          config.start_with_manual_guidance);
      signals.menu_requested.store(false);

      // Executing the configured automatic gripper action before torque control.
      if (!performStartupGripperAction(config)) {
        return -1;
      }

      // Starting manual guidance and selecting the next mode from the guided pose.
      if (config.start_with_manual_guidance &&
          !runManualGuidanceStart(config, robot, model, signals)) {
        if (signals.menu_requested.load()) {
          printSection("returning to the startup menu");
          continue;
        }
        return 0;
      }

      // Building state-dependent impedance matrices from the active parameters.
      const StateImpedanceGains gains = buildStateImpedanceGains(config);

      // Executing the 1 kHz torque controller and storing the completed run data.
      const RunResult result =
          runControlLoop(config, robot, model, gains, signals);

      // Writing the primary CSV log and the terminal run summary.
      writeRunLogs(config, result);

      // Returning to the menu only when requested during the completed run.
      if (!signals.menu_requested.load()) {
        break;
      }
      printSection("returning to the startup menu");
    }

  } catch (const franka::Exception& e) {
    // Reporting robot communication or control exceptions.
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    fprintf(stderr,
            "Recover the robot in Franka Desk before the next run.\n");
    return -1;
  } catch (const std::exception& e) {
    // Reporting configuration, file, or numerical exceptions.
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }

  return 0;
}
