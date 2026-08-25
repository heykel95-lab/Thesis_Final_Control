// ============================================================================
// Cartesian impedance control
// ============================================================================
// Defining surface geometry, orientation errors, gain transformations,
// automatic damping quantities, and the Cartesian impedance wrench.
#include "controller_api.h"

// ====================================================================
// Spatial (6x6) gains
// ====================================================================

Mat6x6 blockDiagonal(const Mat3& translational, const Mat3& rotational) {
  // Initializing a spatial matrix with translation in the upper block and
  // rotation in the lower block.
  Mat6x6 gain = Mat6x6::Zero();
  gain.block<3, 3>(0, 0) = translational;
  gain.block<3, 3>(3, 3) = rotational;
  return gain;
}

Mat6x6 complianceShiftMatrix(const Vec3& r_c) {
  // Defining the spatial shift for r_c = p_C - p_TCP [m].
  Mat6x6 adjoint = Mat6x6::Zero();
  adjoint.block<3, 3>(0, 0) = Mat3::Identity();
  adjoint.block<3, 3>(0, 3) = -skewMatrix(r_c);
  adjoint.block<3, 3>(3, 3) = Mat3::Identity();
  return adjoint;
}

Mat6x6 shiftGainToTcp(const Mat6x6& gain_at_center, const Vec3& r_c) {
  // Transforming a center-defined spatial gain to the TCP.
  const Mat6x6 adjoint = complianceShiftMatrix(r_c);
  return adjoint.transpose() * gain_at_center * adjoint;
}

Mat6x6 blockDiagonalRotation(const Mat3& R) {
  // Applying the same rotation to translational and rotational components.
  Mat6x6 T = Mat6x6::Zero();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 3>(3, 3) = R;
  return T;
}

Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_task_frame, const Mat3& R_task) {
  // Transforming a diagonal task-frame gain to the robot base frame.
  return R_task * diagonal_in_task_frame.asDiagonal() * R_task.transpose();
}

// ====================================================================
// Task-space inertia and auto-damping
// ====================================================================

CartesianInertiaEstimate computeCartesianInertiaEstimate(
    const Mat7x7& joint_mass,
    const Mat6x7& J,
    const Mat3& R_task) {
  CartesianInertiaEstimate result;

  // Factorizing the joint-space mass matrix M(q) [kg, kg m^2].
  Eigen::LDLT<Mat7x7> mass_ldlt(joint_mass);
  if (mass_ldlt.info() != Eigen::Success) {
    return result;
  }

  // Solving M^-1 J^T without explicitly inverting the mass matrix.
  const Mat7x6 Minv_Jt = mass_ldlt.solve(J.transpose());
  if (!Minv_Jt.allFinite()) {
    return result;
  }

  // Constructing the inverse operational-space inertia Lambda^-1.
  Mat6x6 lambda_inv = J * Minv_Jt;
  lambda_inv = 0.5 * (lambda_inv + lambda_inv.transpose());
  if (!lambda_inv.allFinite()) {
    return result;
  }

  // Regularizing the diagonal before solving for Lambda.
  Mat6x6 lambda_inv_damped = lambda_inv;
  lambda_inv_damped.diagonal().array() += 1e-9;
  Eigen::LDLT<Mat6x6> lambda_ldlt(lambda_inv_damped);
  if (lambda_ldlt.info() != Eigen::Success) {
    return result;
  }

  Mat6x6 Lambda_base = lambda_ldlt.solve(Mat6x6::Identity());
  if (!Lambda_base.allFinite()) {
    return result;
  }
  Lambda_base = 0.5 * (Lambda_base + Lambda_base.transpose());

  // Transforming operational-space inertia from base axes to task axes.
  const Mat6x6 T_task = blockDiagonalRotation(R_task);
  Mat6x6 Lambda_task = T_task.transpose() * Lambda_base * T_task;
  Lambda_task = 0.5 * (Lambda_task + Lambda_task.transpose());
  if (!Lambda_task.allFinite()) {
    return result;
  }

  // Assigning translational masses [kg] and rotational inertias [kg m^2].
  for (int i = 0; i < 3; ++i) {
    const double m = Lambda_task(i, i);
    const double I = Lambda_task(i + 3, i + 3);
    if (m <= 0.0 || I <= 0.0) {
      return result;
    }
    result.translational(i) = m;
    result.rotational(i) = I;
  }
  result.valid = true;
  return result;
}

Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                  const Vec3& stiffness,
                                  double damping_ratio,
                                  const Vec3& min_damping,
                                  double max_damping) {
  Vec3 damping = Vec3::Zero();
  // Limiting the requested damping ratio to non-negative values [-].
  const double zeta = std::max(0.0, damping_ratio);

  // Calculating D_i = 2 zeta sqrt(M_i K_i) for each task axis.
  for (int i = 0; i < 3; ++i) {
    // Selecting effective inertia [kg or kg m^2] and stiffness [N/m or N m/rad].
    const double m = std::max(0.0, inertia(i));
    const double k = std::max(0.0, stiffness(i));
    const double critical = 2.0 * zeta * std::sqrt(m * k);
    damping(i) = std::min(max_damping, std::max(std::max(0.0, min_damping(i)), critical));
  }
  return damping;
}

// ====================================================================
// Task frames and orientation
// ====================================================================

Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired) {
  // Defining the relative rotation from the current to the desired frame.
  Mat3 R_error = R_current.transpose() * R_desired;

  // Converting the relative rotation to an axis-angle error [rad].
  Eigen::AngleAxisd angle_axis(R_error);

  if (std::abs(angle_axis.angle()) < 1e-9) {
    return Vec3::Zero();
  }
  // Transforming the local rotation vector to the robot base frame [rad].
  return R_current * angle_axis.axis() * angle_axis.angle();
}

Mat3 makeSurfaceFrameFromNormalTangent(const Vec3& normal_input, const Vec3& tangent1_input) {
  // Normalizing the supplied surface normal [-].
  const Vec3 normal = normalizedOrFallback(normal_input, Vec3(1.0, 0.0, 0.0));

  // Projecting the tangent hint into the surface plane [-].
  Vec3 tangent1 = tangent1_input - normal * normal.dot(tangent1_input);

  // Selecting a stable fallback when the hint is parallel to the normal.
  if (tangent1.norm() <= 1e-9) {
    Vec3 fallback(0.0, 1.0, 0.0);
    if (std::abs(normal.dot(fallback)) > 0.95) {
      fallback = Vec3(0.0, 0.0, 1.0);
    }
    tangent1 = fallback - normal * normal.dot(fallback);
  }

  // Constructing the right-handed frame [tangent1, tangent2, normal].
  tangent1.normalize();
  Vec3 tangent2 = normal.cross(tangent1);
  tangent2.normalize();

  Mat3 R_base_surface;
  R_base_surface.col(0) = tangent1;
  R_base_surface.col(1) = tangent2;
  R_base_surface.col(2) = normal;
  return R_base_surface;
}

Mat3 makeSurfaceFrame(const ControllerConfig& params) {
  return makeSurfaceFrameFromNormalTangent(params.surface_normal_base,
                                           params.surface_tangent1_hint_base);
}

Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit) {
  // Limiting the dot product before evaluating inverse trigonometric functions.
  const double dot = std::max(-1.0, std::min(1.0, from_unit.dot(to_unit)));

  // Returning no rotation for coincident unit vectors.
  if (dot > 1.0 - 1e-9) {
    return Mat3::Identity();
  }

  // Selecting a perpendicular axis for opposite unit vectors.
  if (dot < -1.0 + 1e-9) {
    Vec3 axis = from_unit.cross(Vec3::UnitX());
    if (axis.norm() <= 1e-9) {
      axis = from_unit.cross(Vec3::UnitY());
    }
    axis.normalize();
    return Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
  }

  // Constructing the shortest rotation for the general case.
  Vec3 axis = from_unit.cross(to_unit);
  axis.normalize();
  return Eigen::AngleAxisd(std::acos(dot), axis).toRotationMatrix();
}

Vec3 surfaceToolAxisInBase(const ControllerConfig& params, const Mat3& R_base_surface) {
  // Selecting alignment with or opposite to the surface normal [-].
  const double sign = (params.tool_axis_target_sign >= 0.0) ? 1.0 : -1.0;
  return sign * R_base_surface.col(2);  // normal = 3rd column
}

Vec3 desiredToolAxisInBase(const ControllerConfig& params, const Mat3& R_base_surface) {
  // Initializing the flat tool-axis target in the base frame [-].
  const Vec3 flat_axis = surfaceToolAxisInBase(params, R_base_surface);

  // Defining the commanded tangent-axis offsets [rad].
  const double deg_to_rad = M_PI / 180.0;
  const Vec3 rotation_vector =
      deg_to_rad *
      (params.tool_target_offset_tangent1_deg * R_base_surface.col(0) +
       params.tool_target_offset_tangent2_deg * R_base_surface.col(1));
  // Calculating and applying the combined tangent-axis offset [rad].
  const double angle = rotation_vector.norm();
  if (angle <= 1e-12) {
    return flat_axis;
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle) * flat_axis;
}

Vec3 currentToolAxisInBase(const ControllerConfig& params, const Mat3& R_EE) {
  // Transforming the configured EE-frame tool axis to the base frame [-].
  return R_EE * normalizedOrFallback(params.tool_axis_ee, Vec3(0.0, 0.0, 1.0));
}

Vec3 toolSurfaceAlignmentErrorBase(
    const ControllerConfig& params,
    const Mat3& R_EE,
    const Mat3& R_base_surface) {
  // Defining current and flat target axes in the robot base frame [-].
  const Vec3 tool_axis = currentToolAxisInBase(params, R_EE).normalized();
  const Vec3 flat_axis =
      surfaceToolAxisInBase(params, R_base_surface).normalized();
  // Converting the shortest alignment rotation to a base-frame vector [rad].
  const Eigen::AngleAxisd error(
      rotationBetweenUnitVectors(tool_axis, flat_axis));
  if (std::abs(error.angle()) < 1e-12) {
    return Vec3::Zero();
  }
  return error.axis() * error.angle();
}

Vec3 complianceLeverBase(
    const ControllerConfig& params,
    const Mat3& R_EE,
    const Mat3& R_base_surface) {
  if (!params.use_virtual_compliance_center) {
    return Vec3::Zero();
  }
  if (params.compliance_center_in_tool_frame) {
    // Transforming the tool-frame center offset into the base-frame lever [m].
    return R_EE * params.compliance_center_offset_ee;
  }
  // Transforming the surface-frame lever into the robot base frame [m].
  return R_base_surface * params.compliance_lever_surface;
}

double toolSurfaceMisalignmentAngle(
    const ControllerConfig& params,
    const Mat3& R_EE,
    const Mat3& R_base_surface) {
  // Returning the residual tool-to-surface alignment angle [rad].
  return toolSurfaceAlignmentErrorBase(params, R_EE, R_base_surface).norm();
}

Mat3 makeToolTargetOrientation(
    const ControllerConfig& params,
    const Mat3& R_base_surface,
    const Mat3& R_start) {
  const Vec3 tool_axis_start =
      currentToolAxisInBase(params, R_start).normalized();
  const Vec3 tool_axis_target =
      desiredToolAxisInBase(params, R_base_surface).normalized();

  // Aligning the tool axis while preserving the initial twist.
  const Mat3 R_axis =
      rotationBetweenUnitVectors(tool_axis_start, tool_axis_target) * R_start;
  if (!params.command_tool_twist) {
    return R_axis;
  }

  // Selecting the tool-face long axis used to define in-plane twist [-].
  const Vec3 face_long_axis_ee =
      normalizedOrFallback(params.tool_contact_half_length_ee,
                           Vec3(0.0, 1.0, 0.0));
  // Projecting reference directions onto the plane normal to the tool axis.
  const auto perpendicular = [&tool_axis_target](const Vec3& v) -> Vec3 {
    return v - v.dot(tool_axis_target) * tool_axis_target;
  };
  const Vec3 current = perpendicular(R_axis * face_long_axis_ee);
  const Vec3 zero_reference = perpendicular(R_base_surface.col(0));
  // Retaining the axis-aligned orientation for a degenerate reference.
  if (current.norm() <= 1e-9 || zero_reference.norm() <= 1e-9) {
    return R_axis;
  }

  // Rotating the surface tangent by the commanded normal-axis offset [deg].
  const Vec3 target = Eigen::AngleAxisd(
                          (M_PI / 180.0) * params.tool_target_offset_normal_deg,
                          tool_axis_target) *
                      zero_reference.normalized();
  // Calculating the signed twist about the desired tool axis [rad].
  const Vec3 current_unit = current.normalized();
  double twist = std::atan2(
      tool_axis_target.dot(current_unit.cross(target)), current_unit.dot(target));

  // Selecting the equivalent half-turn nearest the initial orientation.
  const Vec3 axis_ee =
      normalizedOrFallback(params.tool_axis_ee, Vec3(0.0, 0.0, 1.0));
  const Vec3 face_center_off_axis =
      params.tool_contact_face_center_ee -
      params.tool_contact_face_center_ee.dot(axis_ee) * axis_ee;
  // Applying half-turn symmetry for a centered rectangular contact face.
  const double face_scale = std::min(params.tool_contact_half_width_ee.norm(),
                                     params.tool_contact_half_length_ee.norm());
  if (face_center_off_axis.norm() <= 0.05 * face_scale) {
    twist -= M_PI * std::round(twist / M_PI);
  }
  return Eigen::AngleAxisd(twist, tool_axis_target) * R_axis;
}

Vec3 applyRotationalAxisMask(const ControllerConfig& params, Vec3 e_R, const Mat3& R_base_surface) {
  // Transforming the base-frame orientation error to surface axes [rad].
  Vec3 e_R_task = R_base_surface.transpose() * e_R;

  // Selecting the rotational spring axes retained by the controller.
  e_R_task(0) =
      params.constrain_rotation_about_alignment_tangent1 ? e_R_task(0) : 0.0;
  e_R_task(1) =
      params.constrain_rotation_about_alignment_tangent2 ? e_R_task(1) : 0.0;
  e_R_task(2) =
      params.constrain_rotation_about_alignment_normal ? e_R_task(2) : 0.0;

  // Transforming the masked orientation error back to the base frame [rad].
  return R_base_surface * e_R_task;
}

// ====================================================================
// Control law
// ====================================================================

Vec6 computeCartesianImpedanceWrench(const ControllerConfig& params,
                         ControlPhase phase,
                         const Mat3& Kp,
                         const Mat3& Dp,
                         const Mat3& KR,
                         const Mat3& DR,
                         const Mat3& R_base_surface,
                         const Vec6& dx,
                         const Vec6& dv,
                         const Mat3& R_EE) {
  // Selecting compliance-center coupling during setup and setup hold.
  const bool coupled =
      params.use_virtual_compliance_center &&
      (phase == ControlPhase::kSetup ||
       (phase == ControlPhase::kPoseHold && params.use_setup_impedance_hold));
  if (!coupled) {
    // Calculating decoupled force [N] and moment [N m] at the TCP.
    Vec6 wrench;
    wrench.head<3>() = Kp * dx.head<3>() + Dp * dv.head<3>();
    wrench.tail<3>() = KR * dx.tail<3>() + DR * dv.tail<3>();
    return wrench;
  }

  // Resolving r_c = p_C - p_TCP in the robot base frame [m].
  Vec3 r_c = Vec3::Zero();
  if (params.compliance_center_in_tool_frame) {
    // Transforming the tool-frame center offset into the base-frame lever [m].
    r_c = R_EE * params.compliance_center_offset_ee;
  } else {
    // Transforming the surface-frame lever into the robot base frame [m].
    r_c = R_base_surface * params.compliance_lever_surface;
  }

  // Shifting the center-defined stiffness and damping to the TCP.
  const Mat6x6 K_tcp = shiftGainToTcp(blockDiagonal(Kp, KR), r_c);
  const Mat6x6 D_tcp = shiftGainToTcp(blockDiagonal(Dp, DR), r_c);

  // Calculating the coupled TCP wrench [N, N m].
  return K_tcp * dx + D_tcp * dv;
}

