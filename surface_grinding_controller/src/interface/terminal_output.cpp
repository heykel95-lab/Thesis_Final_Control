// ============================================================================
// Terminal output
// ============================================================================
// Formatting controller settings, phase information, and run summaries for
// the startup, runtime, and reporting modules.
#include "controller_api.h"

#include <cstring>

// ====================================================================
// Terminal printing
// ====================================================================

const char* phaseName(ControlPhase phase) {
  // Selecting the terminal label associated with the active control phase.
  switch (phase) {
    case ControlPhase::kToolOrientation:
      return "approach_orient";
    case ControlPhase::kSurfaceApproach:
      return "approach_descend";
    case ControlPhase::kSetup:
      return "setup";
    case ControlPhase::kGrinding:
      return "grind";
    case ControlPhase::kPoseHold:
      return "hold";
    case ControlPhase::kManualGuidance:
      return "manual_guide";
  }
  return "unknown";
}

const char* nullspaceModeName(NullspaceMode mode) {
  // Selecting the terminal label for the active nullspace mode.
  switch (mode) {
    case NullspaceMode::kOff:
      return "off";
    case NullspaceMode::kDampingOnly:
      return "nullspace_damping_only";
    case NullspaceMode::kSigmaOnly:
      return "sigma_optimization_only";
    case NullspaceMode::kDampingAndSigma:
      return "nullspace_damping_plus_sigma";
  }
  return "unknown";
}

// ====================================================================
// Value formatting
// ====================================================================

// Defining the shared terminal width for aligned banners and tables.
namespace {

constexpr int kRuleWidth = 78;

void printRuleOf(char c, int width) {
  // Limiting the requested rule width to the fixed terminal buffer.
  char rule[kRuleWidth + 1];
  const int used = width < kRuleWidth ? width : kRuleWidth;
  for (int i = 0; i < used; ++i) {
    rule[i] = c;
  }
  rule[used] = '\0';
  printf("%s\n", rule);
}

}  // namespace

void printRule() {
  printRuleOf('-', kRuleWidth);
}

void printBanner(const char* title) {
  printf("\n");
  printRuleOf('=', kRuleWidth);
  printf("  %s\n", title);
  printRuleOf('=', kRuleWidth);
}

void printSection(const char* title) {
  // Printing a section title within the remaining horizontal rule.
  const int used = 4 + static_cast<int>(strlen(title));
  printf("\n-- %s ", title);
  printRuleOf('-', kRuleWidth - used);
}

// Printing one aligned three-axis row with units and an optional note.
void printRow(const char* label, const Vec3& v, const char* unit,
              const char* note) {
  if (note == nullptr || note[0] == '\0') {
    printf("  %-16s = [%9.1f, %9.1f, %9.1f] %s\n",
           label, v(0), v(1), v(2), unit);
    return;
  }
  printf("  %-16s = [%9.1f, %9.1f, %9.1f] %-8s %s\n",
         label, v(0), v(1), v(2), unit, note);
}

void printVec3Mm(const char* label, const Vec3& v) {
  // Converting Cartesian values from metres to millimetres [mm].
  printf("  %-16s = [%9.1f, %9.1f, %9.1f] mm\n",
         label, 1000.0 * v(0), 1000.0 * v(1), 1000.0 * v(2));
}

void printVec3Deg(const char* label, const Vec3& v) {
  // Converting angular values from radians to degrees [deg].
  const double rad_to_deg = 180.0 / M_PI;
  printf("  %-16s = [%9.2f, %9.2f, %9.2f] deg\n",
         label, rad_to_deg * v(0), rad_to_deg * v(1), rad_to_deg * v(2));
}

void printGainVec(const char* label, const Vec3& v) {
  printf("  %-16s = [%9.4g, %9.4g, %9.4g]\n", label, v(0), v(1), v(2));
}

void printVec7Deg(const char* label, const Vec7& v) {
  // Converting seven joint angles from radians to degrees [deg].
  const double rad_to_deg = 180.0 / M_PI;
  printf("  %-16s = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg\n",
         label,
         rad_to_deg * v(0), rad_to_deg * v(1), rad_to_deg * v(2),
         rad_to_deg * v(3), rad_to_deg * v(4), rad_to_deg * v(5),
         rad_to_deg * v(6));
}

void printSpatialGain6(const char* label, const Mat6x6& M) {
  static const char* kRowNames[6] = {"fx", "fy", "fz", "mx", "my", "mz"};
  printf("%s:\n", label);
  printf("           tx        ty        tz    |    rx        ry        rz\n");
  for (int i = 0; i < 6; ++i) {
    printf("  %s [%8.4g %8.4g %8.4g | %8.4g %8.4g %8.4g ]\n",
           kRowNames[i],
           M(i, 0), M(i, 1), M(i, 2), M(i, 3), M(i, 4), M(i, 5));
    if (i == 2) {
      printf("     ---------------------------+---------------------------\n");
    }
  }
}

void printSpatialGainEigenvalues(const char* label, const Mat6x6& M) {
  // Symmetrizing the spatial gain before evaluating its eigenvalues.
  const Mat6x6 symmetric = 0.5 * (M + M.transpose());
  Eigen::SelfAdjointEigenSolver<Mat6x6> solver(symmetric);
  const Eigen::Matrix<double, 6, 1> eig = solver.eigenvalues();
  const double tol = 1e-6 * std::max(1.0, eig.cwiseAbs().maxCoeff());
  // Classifying the matrix as positive semidefinite within numerical tolerance.
  const bool psd = eig.minCoeff() >= -tol;
  printf("%s eigenvalues = [%.4g, %.4g, %.4g, %.4g, %.4g, %.4g] -> %s (min=%.4g)\n",
         label, eig(0), eig(1), eig(2), eig(3), eig(4), eig(5),
         psd ? "PSD ok (>=0)" : "NOT PSD (<0!)", eig(0));
}

void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final) {
  const double rad_to_deg = 180.0 / M_PI;
  printSection("joint motion [deg]");
  printf("  %-8s%9s%9s%9s\n", "joint", "start", "final", "delta");
  for (int i = 0; i < 7; ++i) {
    printf("  q%-7d%9.1f%9.1f%+9.1f\n",
           i + 1,
           rad_to_deg * q_start(i),
           rad_to_deg * q_final(i),
           rad_to_deg * (q_final(i) - q_start(i)));
  }
}

void printSetupImpedanceLaw(const ControllerConfig& params,
                            const PhaseDampingCache& damping,
                            bool tunable,
                            const Mat3& R_base_surface,
                            const Mat3& R_EE) {
  const bool surface = params.setup_translation_surface_frame;
  // Selecting the translational parameter frame used by setup impedance.
  const Vec3& kp = surface ? params.setup_Kp_surface_diag : params.setup_Kp_diag;
  const Vec3& dp = surface ? params.setup_Dp_surface_diag : params.setup_Dp_diag;
  const char* frame = surface ? "[t1,t2,n]" : "[x,y,z]";

  // Defining a consistent row format for the impedance summary.
  const auto row = [](const std::string& label, const Vec3& v,
                      const char* unit, const char* note) {
    printRow(label.c_str(), v, unit, note);
  };
  const auto labelled = [frame](const char* name) {
    return std::string(name) + " " + frame;
  };

  printSection("setup impedance");
  if (params.use_virtual_compliance_center) {
    // Printing the coupled spatial impedance law and shifted gains.
    printf("  %-16s   wrench = K_TCP*dx + D_TCP*dv\n", "law");
    printf("  %-16s   K_TCP  = Ad(r_c)^T * blkdiag(Kp,KR) * Ad(r_c)\n", "");
  } else {
    printf("  %-16s   f = Kp*e_p + Dp*de,  m = KR*e_R + DR*dw "
           "(decoupled)\n", "law");
  }
  row(labelled("Kp"), kp, "N/m", "");
  if (!params.setup_auto_damping) {
    row(labelled("Dp"), dp, "Ns/m", "");
  } else if (damping.setup_damping_valid) {
    row(labelled("Dp"), damping.setup_Dp_used, "Ns/m", "auto");
  } else {
    printf("  %-16s = auto, refit at the first cycle\n",
           labelled("Dp").c_str());
  }
  row("KR [t1,t2,n]", params.setup_KR_diag, "Nm/rad", "");
  if (!params.setup_auto_damping) {
    row("DR [t1,t2,n]", params.setup_DR_diag, "Nms/rad", "");
  } else if (damping.setup_damping_valid) {
    row("DR [t1,t2,n]", damping.setup_DR_used, "Nms/rad", "auto");
  } else {
    printf("  %-16s = auto, refit at the first cycle\n", "DR [t1,t2,n]");
  }
  // Resolving the virtual compliance center in the active command frame [m].
  if (params.use_virtual_compliance_center) {
    Vec3 r_c_base = Vec3::Zero();
    if (params.compliance_center_in_tool_frame) {
      r_c_base = -(R_EE * params.compliance_center_offset_ee);
    } else if (params.compliance_lever_in_surface_frame) {
      r_c_base =
          R_base_surface * params.r_tcp_from_compliance_center_surface;
    }
    const bool tool_frame_center = params.compliance_center_in_tool_frame;
    if (!tunable) {
      printRow("p_c [EE]", Vec3(-1000.0 * (R_EE.transpose() * r_c_base)),
               "mm", tool_frame_center ? "commanded in the tool frame"
                                         : "resolved in the tool frame");
    }
    printRow("r_c [t1,t2,n]",
             Vec3(1000.0 * (R_base_surface.transpose() * r_c_base)), "mm",
             tool_frame_center ? "p_TCP - p_c, in the plane"
                            : "commanded: p_TCP - p_c, in the plane");
  }
  if (tunable) {
    // Displaying orientation offsets applied by the next sequence [deg].
    printRow("tilt for s", Vec3(params.tool_target_offset_tangent1_deg,
                                params.tool_target_offset_tangent2_deg,
                                params.tool_target_offset_normal_deg),
             "deg", "about [t1,t2,n], commanded by the next sequence");
    printf("  %-16s   kp1..kp3 <N/m> | kr1..kr3 <Nm/rad> | t1,t2 <deg>\n",
           "keys");
    if (params.use_virtual_compliance_center) {
      // Displaying coordinate conventions accepted by live center tuning [mm].
      printf("  %-16s   r1..r3 <mm> lever in surface frame | "
             "pc1..pc3 <mm> center in tool frame\n", "");
    }
    printf("  %-16s   s runs the sequence with them | t comes back here\n", "");
  }
}

void printNullspaceLaw(const ControllerConfig& params) {
  char title[64];
  snprintf(title, sizeof(title), "nullspace: %s",
           nullspaceModeName(params.nullspace_mode));
  printSection(title);
  switch (params.nullspace_mode) {
    case NullspaceMode::kOff:
      printf("  %-16s   tau = 0\n", "law");
      break;
    case NullspaceMode::kDampingOnly:
      printf("  %-16s   tau = -d_null * N_tau * dq\n", "law");
      printf("  %-16s   %.3f Nms/rad\n", "d_null",
             params.nullspace_damping);
      printf("  %-16s   d <Nms/rad>\n", "keys");
      break;
    case NullspaceMode::kSigmaOnly:
      printf("  %-16s   tau = +k_sigma * N_tau * n_best\n", "law");
      printf("  %-16s   %.3f Nm\n", "k_sigma",
             params.nullspace_sigma_gain);
      printf("  %-16s   k <Nm> | a <deg>\n", "keys");
      break;
    case NullspaceMode::kDampingAndSigma:
      printf("  %-16s   tau = -d_null * N_tau * dq\n", "law");
      printf("  %-16s       + k_sigma * N_tau * n_best\n", "");
      printf("  %-16s   %.3f Nms/rad\n", "d_null",
             params.nullspace_damping);
      printf("  %-16s   %.3f Nm\n", "k_sigma", params.nullspace_sigma_gain);
      printf("  %-16s   d <Nms/rad> | k <Nm> | a <deg>\n", "keys");
      break;
  }
  // Displaying the probe step for singular-value conditioning modes [deg, rad].
  if (params.nullspace_mode == NullspaceMode::kSigmaOnly ||
      params.nullspace_mode == NullspaceMode::kDampingAndSigma) {
    printf("  %-16s   %.3f deg = %.6f rad\n",
           "probe alpha", 180.0 / M_PI * params.nullspace_probe_step_rad,
           params.nullspace_probe_step_rad);
  }
  printf("  %-16s   0/1/2/3 switch mode\n", "");
}

void printAutomaticDisturbance(const ControllerConfig& params,
                               const Vec3& force_direction_base) {
  if (!params.disturbance_auto_enabled) {
    return;
  }
  printSection("automatic link-point push");
  printf("  %-16s   joint %d frame\n", "link", params.disturbance_link);
  printf("  %-16s   [%+.1f, %+.1f, %+.1f] mm\n", "point [link]",
         1000.0 * params.disturbance_point_link(0),
         1000.0 * params.disturbance_point_link(1),
         1000.0 * params.disturbance_point_link(2));
  printf("  %-16s   [%+.3f, %+.3f, %+.3f]\n", "direction [base]",
         force_direction_base(0), force_direction_base(1),
         force_direction_base(2));
  printf("  %-16s   %.2f N | tau norm <= %.2f Nm\n", "force",
         params.disturbance_force, params.disturbance_max_tau_norm);
  printf("  %-16s   ramp %.1f--%.1f | hold to %.1f | release %.1f s\n",
         "time [s]", params.disturbance_push_time,
         params.disturbance_hold_time, params.disturbance_release_time,
         params.disturbance_release_ramp_time);
}

void printPhaseHeader(ControlPhase phase) {
  char title[48];
  snprintf(title, sizeof(title), "phase: %s", phaseName(phase));
  printSection(title);
}

void printPhaseIntro(const ControllerConfig& params,
                     const PhaseDampingCache& damping,
                     ControlPhase phase) {
  // Selecting a common row layout for all phase impedance summaries.
  const auto row = printRow;

  switch (phase) {
    case ControlPhase::kToolOrientation:
    case ControlPhase::kSurfaceApproach: {
      char auto_note[48] = "";
      if (params.approach_auto_damping) {
        snprintf(auto_note, sizeof(auto_note), "auto (factor %.2f)",
                 params.approach_auto_damping_factor);
      }
      row("Kp [t1,t2,n]", params.approach_Kp_diag, "N/m", "");
      if (params.approach_auto_damping && damping.approach_damping_valid) {
        row("Dp [t1,t2,n]", damping.approach_Dp_used, "Ns/m", auto_note);
      } else if (!params.approach_auto_damping) {
        row("Dp [t1,t2,n]", params.approach_Dp_diag, "Ns/m", "");
      } else {
        printf("  %-16s = %s, fitted at the first cycle\n", "Dp [t1,t2,n]",
               auto_note);
      }
      row("KR [t1,t2,n]", params.approach_KR_diag, "Nm/rad", "");
      if (params.approach_auto_damping && damping.approach_damping_valid) {
        row("DR [t1,t2,n]", damping.approach_DR_used, "Nms/rad", auto_note);
      } else if (!params.approach_auto_damping) {
        row("DR [t1,t2,n]", params.approach_DR_diag, "Nms/rad", "");
      } else {
        printf("  %-16s = %s, fitted at the first cycle\n", "DR [t1,t2,n]",
               auto_note);
      }
      if (phase == ControlPhase::kSurfaceApproach) {
        printf("  %-16s   %.0f mm clearance at %.3f m/s\n", "descend to",
               1000.0 * params.descend_surface_clearance, params.descend_speed);
      }
      break;
    }
    case ControlPhase::kGrinding:
      if (params.grind_sweep_enabled) {
        printf("  %-16s   sweep along tangent%d, %.0f mm at %.2f Hz\n", "motion",
               params.grind_tangent_axis, 1000.0 * params.grind_amplitude_m,
               params.grind_frequency_hz);
      } else {
        printf("  %-16s   free-slide press hold\n", "motion");
      }
      printf("  %-16s   as set up\n", "impedance");
      break;
    case ControlPhase::kPoseHold:
      if (params.use_setup_impedance_hold) {
        break;  // the setup impedance block covers it
      }
      row("Kp [x,y,z]", params.hold_Kp_diag, "N/m", "");
      if (params.hold_auto_damping && damping.hold_damping_valid) {
        row("Dp [x,y,z]", damping.hold_Dp_used, "Ns/m", "auto");
      } else if (!params.hold_auto_damping) {
        row("Dp [x,y,z]", params.hold_Dp_diag, "Ns/m", "");
      }
      row("KR [x,y,z]", params.hold_KR_diag, "Nm/rad", "");
      if (params.hold_auto_damping && damping.hold_damping_valid) {
        row("DR [x,y,z]", damping.hold_DR_used, "Nms/rad", "auto");
      } else if (!params.hold_auto_damping) {
        row("DR [x,y,z]", params.hold_DR_diag, "Nms/rad", "");
      }
      break;
    case ControlPhase::kSetup:
    case ControlPhase::kManualGuidance:
      break;  // these print their own block
  }
}

void printGateHold(const ControllerConfig& params, const PhaseDampingCache& damping) {
  const bool translation_surface = params.pause_hold_translation_surface_frame;
  const bool rotation_surface = params.pause_hold_rotation_surface_frame;
  // Selecting the parameter frames used by the gate hold.
  const Vec3& kp = translation_surface ? params.pause_hold_Kp_surface_diag
                                       : params.pause_hold_Kp_diag;
  const Vec3& dp = translation_surface ? params.pause_hold_Dp_surface_diag
                                       : params.pause_hold_Dp_diag;
  const Vec3& kr = rotation_surface ? params.pause_hold_KR_surface_diag
                                    : params.pause_hold_KR_diag;
  const Vec3& dr = rotation_surface ? params.pause_hold_DR_surface_diag
                                    : params.pause_hold_DR_diag;
  const auto labelled = [](const char* name, bool surface) {
    return std::string(name) + " " + (surface ? "[t1,t2,n]" : "[x,y,z]");
  };

  printSection("gate hold");
  printRow(labelled("Kp", translation_surface).c_str(), kp, "N/m", "");
  if (params.pause_hold_auto_damping && damping.pause_damping_valid) {
    printRow(labelled("Dp", translation_surface).c_str(), damping.pause_Dp_used,
             "Ns/m", "auto");
  } else if (!params.pause_hold_auto_damping) {
    printRow(labelled("Dp", translation_surface).c_str(), dp, "Ns/m", "");
  }
  printRow(labelled("KR", rotation_surface).c_str(), kr, "Nm/rad", "");
  if (params.pause_hold_auto_damping && damping.pause_damping_valid) {
    printRow(labelled("DR", rotation_surface).c_str(), damping.pause_DR_used,
             "Nms/rad", "auto");
  } else if (!params.pause_hold_auto_damping) {
    printRow(labelled("DR", rotation_surface).c_str(), dr, "Nms/rad", "");
  }
}

void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point) {
  printVec3Mm("offset_ee", offset_ee);
  printVec3Mm("p_EE_at_contact", p_EE_at_contact);
  printVec3Mm("contact_point", contact_point);
  printVec3Mm("edge_offset", contact_point - p_EE_at_contact);
}

// ====================================================================
// Per-phase debug lines
// ====================================================================

void printApproachOrientDebug(double phase_time,
                              double axis_error_deg,
                              double spin_error_deg) {
  printf("orient:     t=%5.1f s | axis_err=%5.1f deg | spin_err=%5.1f deg\n",
         phase_time, axis_error_deg, spin_error_deg);
}

void printApproachDescendDebug(double phase_time,
                               double distance_mm,
                               double height_mm,
                               double target_height_mm,
                               double force_n) {
  printf("descend:    t=%5.1f s | distance=%6.1f mm | height=%+6.1f mm (target %.1f) | force=%5.1f N\n",
         phase_time, distance_mm, height_mm, target_height_mm, force_n);
}

void printSetupDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double contact_mm) {
  printf("setup:     t=%5.1f s | tip=%5.1f deg | F=%5.1f N | M=%5.1f Nm (limit %.1f) | contact=%5.1f mm\n",
         phase_time, tip_deg, force_n, moment_nm, moment_limit_nm, contact_mm);
}

void printGrindDebug(double phase_time,
                     double sweep_mm,
                     double track_error_mm,
                     double press_n) {
  printf("grind:      t=%5.1f s | sweep=%+6.1f mm | track_err=%+5.1f mm | press=%5.1f N\n",
         phase_time, sweep_mm, track_error_mm, press_n);
}

void printHoldDebug(double phase_time,
                    double force_n,
                    double pos_error_mm,
                    double rot_error_deg) {
  printf("hold:       t=%5.1f s | force=%5.1f N | pos_err=%5.1f mm | rot_err=%5.1f deg\n",
         phase_time, force_n, pos_error_mm, rot_error_deg);
}

void printFinalSummary(const Vec3& final_p_d,
                       const Vec3& final_p_EE,
                       const Vec3& final_e_p,
                       const Vec3& final_e_R,
                       const std::string& csv_file_name) {
  printBanner("FINAL RESULT");
  printVec3Mm("p_d", final_p_d);
  printVec3Mm("p_EE", final_p_EE);
  printVec3Mm("e_p", final_e_p);
  printVec3Deg("e_R", final_e_R);
  printRule();
  printf("  %-16s   %.2f mm\n", "position error", 1000.0 * final_e_p.norm());
  printf("  %-16s   %.2f deg\n", "rotation error",
         (180.0 / M_PI) * final_e_R.norm());
  printf("  %-16s   %s\n", "csv", csv_file_name.c_str());
  printRule();
}

