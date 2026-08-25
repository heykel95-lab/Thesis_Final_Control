// ============================================================================
// Parameter input
// ============================================================================
// Resolving parameter files, parsing scalar values and pi expressions, and
// assigning the resulting values to one ControllerConfig object.
#include "controller_api.h"

#include <cstdlib>
#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>

std::string trim(const std::string& input) {
  // Defining characters removed from both ends of a parameter token.
  const std::string whitespace = " \t\r\n";

  // Selecting the first and last non-whitespace characters.
  const auto begin = input.find_first_not_of(whitespace);
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(begin, end - begin + 1);
}

std::string removeSpaces(std::string value) {
  // Removing internal whitespace before parsing numerical expressions.
  value.erase(
      std::remove_if(value.begin(), value.end(), [](unsigned char c) {
        return std::isspace(c);
      }),
      value.end());
  return value;
}

// Parsing plain numbers and pi expressions such as pi/2 or -3*pi/4 [rad].
double parseDoubleValue(const std::string& input) {
  // Normalizing the expression before identifying its components.
  std::string value = removeSpaces(input);

  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  // Returning plain numerical values without angular conversion.
  const std::string pi = "pi";
  const std::size_t pi_pos = value.find(pi);
  if (pi_pos == std::string::npos) {
    return std::stod(value);
  }

  // Extracting the algebraic sign of the pi expression.
  double sign = 1.0;
  if (!value.empty() && value[0] == '-') {
    sign = -1.0;
    value = value.substr(1);
  } else if (!value.empty() && value[0] == '+') {
    value = value.substr(1);
  }

  // Extracting the multiplier preceding pi.
  double numerator = 1.0;
  if (value.find("*pi") != std::string::npos) {
    numerator = std::stod(value.substr(0, value.find("*pi")));
  } else if (value.find(pi) != 0) {
    numerator = std::stod(value.substr(0, value.find(pi)));
  }

  // Extracting the denominator following the division operator.
  double denominator = 1.0;
  const std::size_t slash_pos = value.find('/');
  if (slash_pos != std::string::npos) {
    denominator = std::stod(value.substr(slash_pos + 1));
  }

  return sign * numerator * M_PI / denominator;
}

NullspaceMode parseNullspaceMode(const std::string& input, NullspaceMode fallback) {
  // Normalizing numerical and textual selector entries.
  std::string value = removeSpaces(input);
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (value == "0" || value == "off" || value == "none" || value == "no") {
    return NullspaceMode::kOff;
  }
  if (value == "1" || value == "posture" || value == "tau_nullspace" ||
      value == "nullspace" || value == "damping") {
    return NullspaceMode::kDampingOnly;
  }
  if (value == "2" || value == "sigma" || value == "tau_sigma") {
    return NullspaceMode::kSigmaOnly;
  }
  if (value == "3" || value == "both" || value == "combined") {
    return NullspaceMode::kDampingAndSigma;
  }

  return fallback;
}

std::string executableDirectory() {
  // Resolving the active executable through the Linux process interface.
  char path[PATH_MAX];
  const ssize_t length = readlink("/proc/self/exe", path, sizeof(path) - 1);
  if (length > 0) {
    path[length] = '\0';
    const std::string full_path(path);
    const std::size_t slash = full_path.find_last_of('/');
    if (slash != std::string::npos) {
      return full_path.substr(0, slash);
    }
  }

  // Selecting the current working directory when no executable path is available.
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    return std::string(cwd);
  }
  throw std::runtime_error("Could not determine the executable directory.");
}

namespace {

bool isDirectory(const std::string& path) {
  // Testing whether the supplied filesystem path identifies a directory.
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

std::string canonicalDirectory(const std::string& path) {
  // Resolving symbolic links and relative path components.
  char resolved[PATH_MAX];
  if (realpath(path.c_str(), resolved) != nullptr) {
    return std::string(resolved);
  }
  return path;
}

}  // namespace

std::string defaultParameterDirectory() {
  // Defining parameter-directory candidates for the controller and tools.
  const std::string executable_dir = executableDirectory();
  const std::vector<std::string> candidates = {
      executable_dir + "/params",
      executable_dir + "/../params",
  };

  // Selecting the first valid directory beside the executable.
  for (const std::string& candidate : candidates) {
    if (isDirectory(candidate)) {
      return canonicalDirectory(candidate);
    }
  }

  // Selecting params/ from the current project directory as a final location.
  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    const std::string candidate = std::string(cwd) + "/params";
    if (isDirectory(candidate)) {
      return canonicalDirectory(candidate);
    }
  }

  throw std::runtime_error(
      "Could not locate the params directory beside the executable or in the "
      "current project directory.");
}

std::vector<std::string> parameterFiles(const std::string& dir) {
  // Selecting the supplied directory or resolving the default location.
  const std::string selected_dir = dir.empty() ? defaultParameterDirectory() : dir;

  // Defining the fixed loading order for all configuration files.
  const std::string prefix =
      selected_dir.back() == '/' ? selected_dir : selected_dir + "/";
  return {
      prefix + "run_settings.conf",
      prefix + "safety.conf",
      prefix + "gripper.conf",
      prefix + "auto_damping.conf",
      prefix + "surface.conf",
      prefix + "tool_orientation.conf",
      prefix + "tool_geometry.conf",
      prefix + "initial_pose.conf",
      prefix + "approach.conf",
      prefix + "phase_gates.conf",
      prefix + "setup.conf",
      prefix + "grinding.conf",
      prefix + "nullspace.conf",
      prefix + "disturbance.conf",
      prefix + "hold.conf",
      prefix + "guidance.conf",
  };
}

ControllerConfig readControllerConfig(const std::vector<std::string>& filenames) {
  // Initializing the configuration with validated compiled defaults.
  ControllerConfig p;

  // Collecting unique key-value pairs from all parameter files.
  std::map<std::string, std::string> values;

  // Reading every required file and reporting all missing files together.
  std::vector<std::string> missing_files;
  for (const std::string& filename : filenames) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      missing_files.push_back(filename);
      continue;
    }

    // Removing comments and selecting key-value assignment lines.
    std::string line;
    while (std::getline(file, line)) {
      const auto comment_pos = line.find('#');
      if (comment_pos != std::string::npos) {
        line = line.substr(0, comment_pos);
      }

      const auto eq_pos = line.find('=');
      if (eq_pos == std::string::npos) {
        continue;
      }

      // Separating and trimming the parameter name and value.
      const std::string key = trim(line.substr(0, eq_pos));
      const std::string value = trim(line.substr(eq_pos + 1));

      if (!key.empty() && !value.empty()) {
        // Rejecting duplicate keys to prevent silent parameter replacement.
        if (values.count(key) != 0) {
          throw std::runtime_error(
              "Duplicate parameter key '" + key + "' in " + filename + ".");
        }
        values[key] = value;
      }
    }
  }

  // Stopping configuration loading when any required file is unavailable.
  if (!missing_files.empty()) {
    std::string message = "Required parameter files could not be opened:";
    for (const std::string& filename : missing_files) {
      message += "\n  " + filename;
    }
    throw std::runtime_error(message);
  }

  // Defining typed accessors with the compiled value as a fallback.
  auto getString = [&](const std::string& key, const std::string& def) {
    return values.count(key) ? values[key] : def;
  };
  auto getDouble = [&](const std::string& key, double def) {
    return values.count(key) ? parseDoubleValue(values[key]) : def;
  };
  auto getBool = [&](const std::string& key, bool def) {
    return values.count(key) ? (std::stoi(values[key]) != 0) : def;
  };
  // Assigning <key>_x/_y/_z to a Cartesian vector.
  auto getVec3Xyz = [&](const std::string& key, const Vec3& def) {
    return Vec3(getDouble(key + "_x", def(0)),
                getDouble(key + "_y", def(1)),
                getDouble(key + "_z", def(2)));
  };
  // Assigning tangent1, tangent2, and normal entries to a surface-frame vector.
  auto getVec3Task = [&](const std::string& key, const Vec3& def) {
    return Vec3(getDouble(key + "_tangent1", def(0)),
                getDouble(key + "_tangent2", def(1)),
                getDouble(key + "_normal", def(2)));
  };
  // Assigning robot connection, run timing [s], logging, and terminal timing [s].
  p.robot_ip = getString("robot_ip", p.robot_ip);
  p.experiment_duration = getDouble("experiment_duration", p.experiment_duration);
  p.csv_file_name = getString("csv_file_name", p.csv_file_name);
  p.log_every_n_cycles =
      std::max(1, static_cast<int>(getDouble("log_every_n_cycles", p.log_every_n_cycles)));
  p.max_log_rows =
      std::max(0, static_cast<int>(getDouble("max_log_rows", p.max_log_rows)));
  p.debug_period = getDouble("debug_period", p.debug_period);
  p.print_hold_debug = getBool("print_hold_debug", p.print_hold_debug);
  p.print_grind_debug = getBool("print_grind_debug", p.print_grind_debug);
  p.print_compliance_diagnostics =
      getBool("print_compliance_diagnostics", p.print_compliance_diagnostics);

  // Assigning gripper widths [m], speeds [m/s], force [N], and tolerances [m].
  p.perform_gripper_action_before_run = getBool("perform_gripper_action_before_run", p.perform_gripper_action_before_run);
  p.abort_on_gripper_action_failure = getBool("abort_on_gripper_action_failure", p.abort_on_gripper_action_failure);
  p.gripper_startup_grasp_tool = getBool("gripper_startup_grasp_tool", p.gripper_startup_grasp_tool);
  p.gripper_open_width = getDouble("gripper_open_width", p.gripper_open_width);
  p.gripper_open_speed = getDouble("gripper_open_speed", p.gripper_open_speed);
  p.gripper_grasp_width = getDouble("gripper_grasp_width", p.gripper_grasp_width);
  p.gripper_grasp_speed = getDouble("gripper_grasp_speed", p.gripper_grasp_speed);
  p.gripper_grasp_force = getDouble("gripper_grasp_force", p.gripper_grasp_force);
  p.gripper_grasp_epsilon_inner =
      getDouble("gripper_grasp_epsilon_inner", p.gripper_grasp_epsilon_inner);
  p.gripper_grasp_epsilon_outer =
      getDouble("gripper_grasp_epsilon_outer", p.gripper_grasp_epsilon_outer);

  // Assigning orientation-phase and manual-guidance selectors.
  p.enable_orientation_phase = getBool("enable_orientation_phase", p.enable_orientation_phase);
  p.manual_guidance_damping =
      getDouble("manual_guidance_damping", p.manual_guidance_damping);

  // Assigning pose-hold stiffness and damping matrices.
  p.hold_Kp_diag = getVec3Xyz("hold_Kp", p.hold_Kp_diag);
  p.hold_Dp_diag = getVec3Xyz("hold_Dp", p.hold_Dp_diag);
  p.hold_KR_diag = getVec3Xyz("hold_KR", p.hold_KR_diag);
  p.hold_DR_diag = getVec3Xyz("hold_DR", p.hold_DR_diag);
  p.hold_auto_damping = getBool("hold_auto_damping", p.hold_auto_damping);
  p.hold_auto_match_manual_damping =
      getBool("hold_auto_match_manual_damping",
              p.hold_auto_match_manual_damping);
  p.hold_auto_damping_factor =
      getDouble("hold_auto_damping_factor", p.hold_auto_damping_factor);

  // Assigning surface geometry, tool orientation, and contact-face dimensions.
  p.use_start_as_surface_point =
      getBool("use_start_as_surface_point", p.use_start_as_surface_point);
  p.surface_point = getVec3Xyz("surface_point", p.surface_point);
  p.surface_tilt_x_deg =
      getDouble("surface_tilt_x_deg", p.surface_tilt_x_deg);
  p.surface_tilt_y_deg =
      getDouble("surface_tilt_y_deg", p.surface_tilt_y_deg);
  const double ax = p.surface_tilt_x_deg * M_PI / 180.0;
  const double ay = p.surface_tilt_y_deg * M_PI / 180.0;
  p.surface_normal_base =
      Vec3(std::sin(ay) * std::cos(ax), -std::sin(ax), std::cos(ay) * std::cos(ax));
  p.surface_normal_base.normalize();
  p.surface_tangent1_hint_base =
      getVec3Xyz("surface_tangent1_hint_base", p.surface_tangent1_hint_base);
  p.tool_target_offset_tangent1_deg =
      getDouble("tool_target_offset_tangent1_deg",
                p.tool_target_offset_tangent1_deg);
  p.tool_target_offset_tangent2_deg =
      getDouble("tool_target_offset_tangent2_deg",
                p.tool_target_offset_tangent2_deg);
  p.tool_axis_ee = getVec3Xyz("tool_axis_ee", p.tool_axis_ee);
  p.tool_axis_target_sign = getDouble("tool_axis_target_sign", p.tool_axis_target_sign);
  p.command_tool_twist = getBool("command_tool_twist", p.command_tool_twist);
  p.tool_target_offset_normal_deg =
      getDouble("tool_target_offset_normal_deg", p.tool_target_offset_normal_deg);
  p.use_tool_contact_point_control =
      getBool("use_tool_contact_point_control", p.use_tool_contact_point_control);
  p.auto_select_tool_contact_edge =
      getBool("auto_select_tool_contact_edge", p.auto_select_tool_contact_edge);
  p.tool_contact_face_center_ee =
      getVec3Xyz("tool_contact_face_center_ee", p.tool_contact_face_center_ee);
  p.tool_contact_half_width_ee =
      getVec3Xyz("tool_contact_half_width_ee", p.tool_contact_half_width_ee);
  p.tool_contact_half_length_ee =
      getVec3Xyz("tool_contact_half_length_ee", p.tool_contact_half_length_ee);
  p.tool_contact_feature_tie_tolerance =
      getDouble("tool_contact_feature_tie_tolerance",
                p.tool_contact_feature_tie_tolerance);
  p.constrain_rotation_about_alignment_normal =
      getBool("constrain_rotation_about_alignment_normal",
              p.constrain_rotation_about_alignment_normal);
  p.constrain_rotation_about_alignment_tangent1 =
      getBool("constrain_rotation_about_alignment_tangent1",
              p.constrain_rotation_about_alignment_tangent1);
  p.constrain_rotation_about_alignment_tangent2 =
      getBool("constrain_rotation_about_alignment_tangent2",
              p.constrain_rotation_about_alignment_tangent2);

  // Assigning disturbance timing [s], force [N], point [m], and torque limit [N m].
  p.disturbance_cues_enabled =
      getBool("disturbance_cues_enabled", p.disturbance_cues_enabled);
  p.disturbance_push_time =
      getDouble("disturbance_push_time", p.disturbance_push_time);
  p.disturbance_hold_time =
      getDouble("disturbance_hold_time", p.disturbance_hold_time);
  p.disturbance_release_time =
      getDouble("disturbance_release_time", p.disturbance_release_time);
  p.disturbance_auto_enabled =
      getBool("disturbance_auto_enabled", p.disturbance_auto_enabled);
  p.disturbance_link = static_cast<int>(
      getDouble("disturbance_link", p.disturbance_link));
  p.disturbance_point_link =
      getVec3Xyz("disturbance_point_link", p.disturbance_point_link);
  p.disturbance_force =
      getDouble("disturbance_force", p.disturbance_force);
  p.disturbance_direction_sign =
      getDouble("disturbance_direction_sign",
                p.disturbance_direction_sign);
  p.disturbance_release_ramp_time =
      getDouble("disturbance_release_ramp_time",
                p.disturbance_release_ramp_time);
  p.disturbance_max_tau_norm =
      getDouble("disturbance_max_tau_norm",
                p.disturbance_max_tau_norm);

  // Assigning tool-orientation and surface-approach parameters.
  p.approach_orient_min_time =
      getDouble("approach_orient_min_time", p.approach_orient_min_time);
  p.approach_orient_error_threshold =
      getDouble("approach_orient_error_threshold", p.approach_orient_error_threshold);
  p.approach_orient_spin_error_threshold =
      getDouble("approach_orient_spin_error_threshold",
                p.approach_orient_spin_error_threshold);
  p.approach_orient_timeout =
      getDouble("approach_orient_timeout", p.approach_orient_timeout);
  p.approach_orient_max_rate_deg =
      getDouble("approach_orient_max_rate_deg", p.approach_orient_max_rate_deg);
  p.approach_Kp_diag = getVec3Task("approach_Kp", p.approach_Kp_diag);
  p.approach_KR_diag = getVec3Task("approach_KR", p.approach_KR_diag);
  p.approach_Dp_diag = getVec3Task("approach_Dp", p.approach_Dp_diag);
  p.approach_DR_diag = getVec3Task("approach_DR", p.approach_DR_diag);
  p.approach_auto_damping = getBool("approach_auto_damping", p.approach_auto_damping);
  p.approach_auto_damping_factor =
      getDouble("approach_auto_damping_factor", p.approach_auto_damping_factor);
  p.descend_speed = getDouble("descend_speed", p.descend_speed);
  p.descend_max_distance = getDouble("descend_max_distance", p.descend_max_distance);
  p.descend_surface_clearance =
      getDouble("descend_surface_clearance", p.descend_surface_clearance);

  // Assigning setup timing [s], preload motion [m], and Cartesian impedance.
  p.setup_min_time = getDouble("setup_min_time", p.setup_min_time);
  p.setup_timeout = getDouble("setup_timeout", p.setup_timeout);
  p.setup_moment_threshold = getDouble("setup_moment_threshold", p.setup_moment_threshold);
  p.setup_push_speed = getDouble("setup_push_speed", p.setup_push_speed);
  p.setup_push_end = getDouble("setup_push_end", p.setup_push_end);
  p.setup_align_tolerance_deg =
      getDouble("setup_align_tolerance_deg", p.setup_align_tolerance_deg);
  p.setup_align_hold_time =
      getDouble("setup_align_hold_time", p.setup_align_hold_time);
  p.setup_align_fraction =
      getDouble("setup_align_fraction", p.setup_align_fraction);
  p.setup_Kp_diag = getVec3Xyz("setup_Kp", p.setup_Kp_diag);
  p.setup_Dp_diag = getVec3Xyz("setup_Dp", p.setup_Dp_diag);
  p.setup_translation_surface_frame =
      getBool("setup_translation_surface_frame", p.setup_translation_surface_frame);
  p.setup_Kp_surface_diag =
      getVec3Task("setup_Kp_surface", p.setup_Kp_surface_diag);
  p.setup_Dp_surface_diag =
      getVec3Task("setup_Dp_surface", p.setup_Dp_surface_diag);
  p.setup_KR_diag = getVec3Task("setup_KR", p.setup_KR_diag);
  p.setup_DR_diag = getVec3Task("setup_DR", p.setup_DR_diag);
  p.setup_auto_damping = getBool("setup_auto_damping", p.setup_auto_damping);
  p.setup_auto_damping_factor =
      getDouble("setup_auto_damping_factor", p.setup_auto_damping_factor);

  // Assigning grinding sweep direction, half-amplitude [m], and frequency [Hz].
  p.grind_sweep_enabled = getBool("grind_sweep_enabled", p.grind_sweep_enabled);
  p.grind_tangent_axis = static_cast<int>(getDouble("grind_tangent_axis", p.grind_tangent_axis));
  p.grind_amplitude_m = getDouble("grind_amplitude_m", p.grind_amplitude_m);
  p.grind_frequency_hz = getDouble("grind_frequency_hz", p.grind_frequency_hz);

  // Assigning phase-gate selectors and Cartesian hold impedance.
  p.pause_before_setup = getBool("pause_before_setup", p.pause_before_setup);
  p.pause_before_grind = getBool("pause_before_grind", p.pause_before_grind);
  p.pause_hold_Kp_diag = getVec3Xyz("pause_hold_Kp", p.pause_hold_Kp_diag);
  p.pause_hold_Dp_diag = getVec3Xyz("pause_hold_Dp", p.pause_hold_Dp_diag);
  p.pause_hold_translation_surface_frame =
      getBool("pause_hold_translation_surface_frame",
              p.pause_hold_translation_surface_frame);
  p.pause_hold_Kp_surface_diag =
      getVec3Task("pause_hold_Kp_surface", p.pause_hold_Kp_surface_diag);
  p.pause_hold_Dp_surface_diag =
      getVec3Task("pause_hold_Dp_surface", p.pause_hold_Dp_surface_diag);
  p.pause_hold_KR_diag = getVec3Xyz("pause_hold_KR", p.pause_hold_KR_diag);
  p.pause_hold_DR_diag = getVec3Xyz("pause_hold_DR", p.pause_hold_DR_diag);
  p.pause_hold_rotation_surface_frame =
      getBool("pause_hold_rotation_surface_frame",
              p.pause_hold_rotation_surface_frame);
  p.pause_hold_KR_surface_diag =
      getVec3Task("pause_hold_KR_surface", p.pause_hold_KR_surface_diag);
  p.pause_hold_DR_surface_diag =
      getVec3Task("pause_hold_DR_surface", p.pause_hold_DR_surface_diag);
  p.pause_hold_auto_damping =
      getBool("pause_hold_auto_damping", p.pause_hold_auto_damping);

  // Assigning the compliance-center selector and lever definitions [m].
  p.use_virtual_compliance_center = getBool("use_virtual_compliance_center", p.use_virtual_compliance_center);
  p.compliance_center_in_tool_frame = getBool("compliance_center_in_tool_frame", p.compliance_center_in_tool_frame);
  p.compliance_center_offset_ee = getVec3Xyz("compliance_center_offset_ee", p.compliance_center_offset_ee);
  p.compliance_lever_in_surface_frame =
      getBool("compliance_lever_in_surface_frame", p.compliance_lever_in_surface_frame);
  p.compliance_lever_surface = getVec3Task("compliance_lever_surface", p.compliance_lever_surface);

  // Assigning nullspace mode, damping [N m s/rad], and sigma torque [N m].
  if (values.count("nullspace_mode")) {
    p.nullspace_mode =
        parseNullspaceMode(values["nullspace_mode"], p.nullspace_mode);
  }
  p.nullspace_damping = getDouble("nullspace_damping", p.nullspace_damping);
  p.nullspace_sigma_gain = getDouble("nullspace_sigma_gain", p.nullspace_sigma_gain);
  p.nullspace_probe_step_rad = getDouble("nullspace_probe_step_rad", p.nullspace_probe_step_rad);
  p.nullspace_sigma_deadband =
      getDouble("nullspace_sigma_deadband", p.nullspace_sigma_deadband);
  p.nullspace_svd_relative_tolerance =
      getDouble("nullspace_svd_relative_tolerance",
                p.nullspace_svd_relative_tolerance);

  p.auto_damping_max = getDouble("auto_damping_max", p.auto_damping_max);
  p.auto_damping_min_from_manual =
      getBool("auto_damping_min_from_manual", p.auto_damping_min_from_manual);

  // Selecting and assigning the configured initial joint posture [rad].
  p.q_init_case = getString("q_init_case", p.q_init_case);
  const char* q_init_prefix = "q_init_horizontal";
  if (p.q_init_case == "horizontal_table_search") {
    q_init_prefix = "q_init_table";
  } else if (p.q_init_case == "tilted_tool") {
    q_init_prefix = "q_init_tilted";
  } else if (p.q_init_case == "tilted_close") {
    q_init_prefix = "q_init_tilted_close";
  } else if (p.q_init_case == "saved_qinit") {
    q_init_prefix = "q_init_saved";
  }
  char q_key[64];
  for (int i = 0; i < 7; ++i) {
    snprintf(q_key, sizeof(q_key), "%s_%d", q_init_prefix, i + 1);
    p.q_init[i] = getDouble(q_key, p.q_init[i]);
  }
  // Assigning optional direct joint entries after the selected posture [rad].
  for (int i = 0; i < 7; ++i) {
    snprintf(q_key, sizeof(q_key), "q_init_%d", i + 1);
    p.q_init[i] = getDouble(q_key, p.q_init[i]);
  }

  // Assigning tool-pickup motion parameters and joint posture [rad].
  p.use_tool_pickup = getBool("use_tool_pickup", p.use_tool_pickup);
  p.pickup_standoff = getDouble("pickup_standoff", p.pickup_standoff);
  p.pickup_descend_speed_factor =
      getDouble("pickup_descend_speed_factor", p.pickup_descend_speed_factor);
  for (int i = 0; i < 7; ++i) {
    snprintf(q_key, sizeof(q_key), "q_pickup_%d", i + 1);
    p.q_pickup[i] = getDouble(q_key, p.q_pickup[i]);
  }

  // Assigning joint-torque [N m] and Cartesian force/moment thresholds.
  p.use_custom_collision_behavior =
      getBool("use_custom_collision_behavior", p.use_custom_collision_behavior);
  p.collision_torque_acc = getDouble("collision_torque_acc", p.collision_torque_acc);
  p.collision_torque_nom = getDouble("collision_torque_nom", p.collision_torque_nom);
  p.collision_force_acc = getDouble("collision_force_acc", p.collision_force_acc);
  p.collision_force_nom = getDouble("collision_force_nom", p.collision_force_nom);

  // Verifying that exactly one compliance-center definition is active.
  if (p.use_virtual_compliance_center &&
      p.compliance_center_in_tool_frame ==
          p.compliance_lever_in_surface_frame) {
    throw std::runtime_error(
        "Virtual compliance-center control requires exactly one definition: "
        "compliance_center_in_tool_frame or "
        "compliance_lever_in_surface_frame.");
  }

  return p;
}
