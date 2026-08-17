// ============================================================================
// Setup evaluation
// ============================================================================
// Evaluating the final setup state, contact wrench, alignment, and compliance
// after a completed setup phase.
#include "controller_api.h"

namespace {

// Formatting one measured force-displacement or moment-angle ratio.
void formatRatio(char* out, size_t n, double num, double den, double den_floor) {
  if (std::abs(den) < den_floor) {
    snprintf(out, n, "%9s", "n/a");
  } else {
    snprintf(out, n, "%9.1f", num / den);
  }
}

// Constructing the tool-face frame [length, width, tool axis] in base axes.
Mat3 makeToolFaceFrame(const ControllerConfig& params, const Mat3& R_EE) {
  // Expressing the tool axis and the face long axis in base coordinates [-].
  const Vec3 axis_base =
      R_EE * normalizedOrFallback(params.tool_axis_ee, Vec3(0.0, 0.0, 1.0));
  const Vec3 length_base =
      R_EE * normalizedOrFallback(params.tool_contact_half_length_ee,
                                  Vec3(0.0, 1.0, 0.0));
  // Reusing the orthonormalization that builds the surface frame.
  return makeSurfaceFrameFromNormalTangent(axis_base, length_base);
}

// Transforming measured wrench, displacement, and rotation to surface axes.
void printSurfaceFrameBreakdown(const ControllerConfig& params,
                                const Mat3& R_base_surface,
                                const SetupReport& r) {
  const Vec3 force_surf =
      R_base_surface.transpose() * (r.external_force - r.contact_force_bias);
  const Vec3 moment_surf = R_base_surface.transpose() * r.m_contact;
  const Vec3 tcp_disp_surf =
      R_base_surface.transpose() * (r.p_EE - r.first_contact_tcp);
  const Vec3 contact_disp_surf =
      R_base_surface.transpose() * (r.tool_contact_point - r.first_contact_point);
  const Vec3 ee_surf =
      R_base_surface.transpose() * orientationError(r.R_EE, r.R_contact_start);

  // Calculating effective translational [N/m] and rotational [N m/rad] ratios.
  char kp[3][16];
  char kr[3][16];
  for (int i = 0; i < 3; ++i) {
    formatRatio(kp[i], sizeof(kp[i]), force_surf(i), tcp_disp_surf(i), 5e-5);
    formatRatio(kr[i], sizeof(kr[i]), moment_surf(i), ee_surf(i), 5e-4);
  }

  printSection("setup surface-frame breakdown");
  printf("  %-16s   alignment-target [tangent1, tangent2, normal]\n",
         "frame");
  printf("  force        [N]      = [%+9.2f, %+9.2f, %+9.2f]\n",
         force_surf(0), force_surf(1), force_surf(2));
  printf("  tcp_disp     [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         1000.0 * tcp_disp_surf(0), 1000.0 * tcp_disp_surf(1), 1000.0 * tcp_disp_surf(2));
  printf("  contact_disp [mm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         1000.0 * contact_disp_surf(0), 1000.0 * contact_disp_surf(1),
         1000.0 * contact_disp_surf(2));
  printf("  Kp_eff=F/tcp [N/m]    = [%s, %s, %s]\n", kp[0], kp[1], kp[2]);
  printf("  M_contact    [Nm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         moment_surf(0), moment_surf(1), moment_surf(2));
  printf("  ee_angle     [deg]    = [%+9.2f, %+9.2f, %+9.2f]\n",
         (180.0 / M_PI) * ee_surf(0), (180.0 / M_PI) * ee_surf(1),
         (180.0 / M_PI) * ee_surf(2));
  printf("  KR_eff=M/ang [Nm/rad] = [%s, %s, %s]\n", kr[0], kr[1], kr[2]);

  // Resolving the contact moment in tool-face axes to identify the tipping edge.
  const Mat3 R_base_face = makeToolFaceFrame(params, r.R_EE);
  const Vec3 moment_face = R_base_face.transpose() * r.m_contact;
  printf("  %-16s   tool face [length, width, tool axis]\n", "frame");
  printf("  M_contact    [Nm]     = [%+9.2f, %+9.2f, %+9.2f]\n",
         moment_face(0), moment_face(1), moment_face(2));
}

}  // namespace

void reportSetupResult(const ControllerConfig& params,
                       const Mat3& R_base_surface,
                       const SetupReport& r) {
  // Calculating the final end-effector deviation [deg] and displacements [mm].
  const double end_effector_deviation_deg =
      (180.0 / M_PI) * orientationError(r.R_EE, r.R_contact_start).norm();
  const Vec3 contact_from_start_mm =
      1000.0 * (r.tool_contact_point - r.first_contact_point);
  const Vec3 tcp_from_contact_mm = 1000.0 * (r.p_EE - r.first_contact_point);

  printBanner("SETUP RESULT");
  printf("  stop: %s | t=%.1f s | ee=%.1f deg | F=%.1f N | M_TCP=%.1f Nm\n",
         r.stopped_on_moment ? "moment" : "time",
         r.phase_time, end_effector_deviation_deg, r.df_ext_norm, r.m_tcp_norm);
  printf("  contact_from_start = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
         contact_from_start_mm(0), contact_from_start_mm(1),
         contact_from_start_mm(2), contact_from_start_mm.norm());
  printf("  tcp_from_contact  = [%+.1f, %+.1f, %+.1f] mm | norm=%.1f mm\n",
         tcp_from_contact_mm(0), tcp_from_contact_mm(1), tcp_from_contact_mm(2),
         tcp_from_contact_mm.norm());

  // Evaluating the tool-to-plane deviation before and after set-up [deg].
  const double align_before_deg =
      (180.0 / M_PI) *
      toolSurfaceMisalignmentAngle(params, r.R_contact_start, R_base_surface);
  const double align_after_deg =
      (180.0 / M_PI) *
      toolSurfaceMisalignmentAngle(params, r.R_EE, R_base_surface);
  printf("  deviation: before=%.2f deg | after=%.2f deg | gain=%+.2f deg\n",
         align_before_deg, align_after_deg, align_before_deg - align_after_deg);
  // The alignment time is observed, not enforced, so a run that never settled
  // inside the tolerance reports that rather than a number.
  if (r.t_align >= 0.0) {
    printf("  t_align: %.2f s | tolerance=%.2f deg | hold=%.2f s\n",
           r.t_align, params.setup_align_tolerance_deg,
           params.setup_align_hold_time);
  } else {
    printf("  t_align: not reached | tolerance=%.2f deg | hold=%.2f s\n",
           params.setup_align_tolerance_deg, params.setup_align_hold_time);
  }
  // The relative crossing and the closest approach. The second pair is always
  // defined, so a condition that never aligns still reports how close it came.
  if (r.t_align_fraction >= 0.0) {
    printf("  t_align_fraction: %.2f s | fraction=%.2f of the deviation at "
           "contact\n", r.t_align_fraction, params.setup_align_fraction);
  } else {
    printf("  t_align_fraction: not reached | fraction=%.2f of the deviation "
           "at contact\n", params.setup_align_fraction);
  }
  printf("  deviation_min: %.2f deg at t=%.2f s\n",
         r.deviation_min_deg, r.t_deviation_min);
  // One word the analysis can sort on. A run that never met the tolerance is
  // still a measurement, and it is kept with the rest rather than dropped;
  // whether it improved or worsened is read from the gain above.
  printf("  align_status: %s\n", r.t_align >= 0.0 ? "aligned" : "not_aligned");
  const Vec3 align_before_surface_deg =
      (180.0 / M_PI) * R_base_surface.transpose() *
      toolSurfaceAlignmentErrorBase(
          params, r.R_contact_start, R_base_surface);
  const Vec3 align_after_surface_deg =
      (180.0 / M_PI) * R_base_surface.transpose() *
      toolSurfaceAlignmentErrorBase(params, r.R_EE, R_base_surface);
  printf("  deviation components [t1,t2,n] deg: before=[%+.2f,%+.2f,%+.2f] | "
         "after=[%+.2f,%+.2f,%+.2f]\n",
         align_before_surface_deg(0), align_before_surface_deg(1),
         align_before_surface_deg(2), align_after_surface_deg(0),
         align_after_surface_deg(1), align_after_surface_deg(2));

  printSurfaceFrameBreakdown(params, R_base_surface, r);

  // Evaluating the coupled setup impedance at the final state.
  if (!params.print_compliance_diagnostics ||
      !params.use_virtual_compliance_center) {
    return;
  }

  // Resolving the commanded center of compliance at the final setup pose [m].
  const Mat6x6 K_center = blockDiagonal(r.Kp, r.KR);
  const Mat6x6 D_center = blockDiagonal(r.Dp, r.DR);
  const Vec3 tcp_ref = r.p_EE;
  Vec3 r_c = Vec3::Zero();
  if (params.compliance_center_in_tool_frame) {
    // Transforming the tool-fixed lever at the final setup orientation [m].
    r_c = -(r.R_EE * params.compliance_center_offset_ee);
  } else if (params.compliance_lever_in_surface_frame) {
    r_c = R_base_surface * params.r_tcp_from_compliance_center_surface;
  }
  const Mat6x6 K_tcp = shiftGainToTcp(K_center, r_c);
  const Mat6x6 D_tcp = shiftGainToTcp(D_center, r_c);

  // Transforming the commanded center to surface and end-effector coordinates [m].
  const Mat3& R_center_ref = r.R_EE;
  const Vec3 r_c_surface = R_base_surface.transpose() * r_c;
  const Vec3 p_c_ee = -(R_center_ref.transpose() * r_c);
  printSection("center of compliance");
  printf("  %-16s   r_c = p_TCP - p_c\n", "definition");
  printVec3Mm("p_TCP [x,y,z]", tcp_ref);
  printVec3Mm("p_c [EE]", p_c_ee);
  printVec3Mm("r_c [t1,t2,n]", r_c_surface);
  printf("  %-16s   p_c resolved at end of setup\n", "");

  // Extracting effective TCP stiffness and damping after the center shift.
  const Vec3 k_trans = K_tcp.block<3, 3>(0, 0).diagonal();
  const Vec3 k_rot = K_tcp.block<3, 3>(3, 3).diagonal();
  const Vec3 d_trans = D_tcp.block<3, 3>(0, 0).diagonal();
  const Vec3 d_rot = D_tcp.block<3, 3>(3, 3).diagonal();
  printSection("effective TCP impedance");
  printRow("K_TCP translation", k_trans, "N/m", "");
  printRow("K_TCP rotation", k_rot, "Nm/rad", "");
  printRow("commanded KR was", Vec3(r.KR.diagonal()), "Nm/rad",
           "including lever contribution");
  printRow("D_TCP translation", d_trans, "Ns/m", "");
  printRow("D_TCP rotation", d_rot, "Nms/rad", "");

  // Reporting the maximum translation-rotation coupling term.
  const double k_coupling = K_tcp.block<3, 3>(3, 0).cwiseAbs().maxCoeff();
  const double d_coupling = D_tcp.block<3, 3>(3, 0).cwiseAbs().maxCoeff();
  printf("  %-16s   max |moment per unit translation| = %.1f Nm/m (K), "
         "%.1f Nms/m (D)\n", "coupling", k_coupling, d_coupling);

  const Eigen::SelfAdjointEigenSolver<Mat6x6> eig_k(K_tcp);
  const Eigen::SelfAdjointEigenSolver<Mat6x6> eig_d(D_tcp);
  printf("  %-16s   K_TCP %.2f .. %.0f | D_TCP %.2f .. %.0f%s\n",
         "eigenvalues",
         eig_k.eigenvalues().minCoeff(), eig_k.eigenvalues().maxCoeff(),
         eig_d.eigenvalues().minCoeff(), eig_d.eigenvalues().maxCoeff(),
         (eig_k.eigenvalues().minCoeff() > 0.0 &&
          eig_d.eigenvalues().minCoeff() >= -1e-8)
             ? "  (both positive: valid springs)"
             : "  *** NOT POSITIVE DEFINITE ***");

  // Printing the complete spatial stiffness and damping matrices.
  printf("\n");
  printSpatialGain6("  K_TCP [N/m, Nm/rad | rows fx..mz, cols tx..rz]", K_tcp);
  printf("\n");
  printSpatialGain6("  D_TCP [Ns/m, Nms/rad]", D_tcp);
}
