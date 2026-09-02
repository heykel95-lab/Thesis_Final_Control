// ============================================================================
// State impedance gains
// ============================================================================
// Building state-dependent stiffness and damping matrices and calculating
// automatic damping from the operational-space inertia.
#include "controller_api.h"

// ====================================================================
// Gain table
// ====================================================================
// Building state stiffness and damping matrices in the robot base frame.
StateImpedanceGains buildStateImpedanceGains(const ControllerConfig& params) {
  // Initializing the surface frame [tangent1, tangent2, normal].
  StateImpedanceGains gains;
  gains.R_base_surface = makeSurfaceFrame(params);

  // Transforming diagonal surface-frame gains to the robot base frame.
  auto taskGain = [&](const Vec3& diagonal) -> Mat3 {
    return makeSpatialGainMatrix(diagonal, gains.R_base_surface);
  };

  // Assigning approach stiffness [N/m, N m/rad] and damping [N s/m, N m s/rad].
  gains.Kp_approach = taskGain(params.approach_Kp_diag);
  gains.Dp_approach = taskGain(params.approach_Dp_diag);
  gains.KR_approach = taskGain(params.approach_KR_diag);
  gains.DR_approach = taskGain(params.approach_DR_diag);
  // Selecting active translational contact establishment gains from base or surface axes.
  gains.contact_establishment_Kp_active_diag =
      params.contact_establishment_translation_surface_frame
          ? params.contact_establishment_Kp_surface_diag
          : params.contact_establishment_Kp_diag;
  gains.contact_establishment_Dp_active_diag =
      params.contact_establishment_translation_surface_frame
          ? params.contact_establishment_Dp_surface_diag
          : params.contact_establishment_Dp_diag;
  // Transforming the selected translational contact establishment gains to base axes.
  gains.Kp_contact_establishment =
      params.contact_establishment_translation_surface_frame
          ? taskGain(params.contact_establishment_Kp_surface_diag)
          : params.contact_establishment_Kp_diag.asDiagonal();
  gains.Dp_contact_establishment =
      params.contact_establishment_translation_surface_frame
          ? taskGain(params.contact_establishment_Dp_surface_diag)
          : params.contact_establishment_Dp_diag.asDiagonal();
  // Assigning rotational contact establishment gains in surface axes and expressing them in base axes.
  gains.KR_contact_establishment = taskGain(params.contact_establishment_KR_diag);
  gains.DR_contact_establishment = taskGain(params.contact_establishment_DR_diag);
  // Assigning Cartesian pose-hold gains in robot base axes.
  gains.Kp_hold = params.hold_Kp_diag.asDiagonal();
  gains.Dp_hold = params.hold_Dp_diag.asDiagonal();
  gains.KR_hold = params.hold_KR_diag.asDiagonal();
  gains.DR_hold = params.hold_DR_diag.asDiagonal();
  // Selecting active pre-contact-hold gains from base or surface axes.
  gains.operator_hold_Kp_active_diag =
      params.operator_hold_translation_surface_frame
          ? params.operator_hold_Kp_surface_diag
          : params.operator_hold_Kp_diag;
  gains.operator_hold_Dp_active_diag =
      params.operator_hold_translation_surface_frame
          ? params.operator_hold_Dp_surface_diag
          : params.operator_hold_Dp_diag;
  // Transforming the selected translational hold gains to base axes.
  gains.Kp_operator_hold =
      params.operator_hold_translation_surface_frame
          ? taskGain(params.operator_hold_Kp_surface_diag)
          : params.operator_hold_Kp_diag.asDiagonal();
  gains.Dp_operator_hold =
      params.operator_hold_translation_surface_frame
          ? taskGain(params.operator_hold_Dp_surface_diag)
          : params.operator_hold_Dp_diag.asDiagonal();
  // Selecting active rotational hold gains from base or surface axes.
  gains.operator_hold_KR_active_diag =
      params.operator_hold_rotation_surface_frame
          ? params.operator_hold_KR_surface_diag
          : params.operator_hold_KR_diag;
  gains.operator_hold_DR_active_diag =
      params.operator_hold_rotation_surface_frame
          ? params.operator_hold_DR_surface_diag
          : params.operator_hold_DR_diag;
  // Transforming the selected rotational hold gains to base axes.
  gains.KR_operator_hold =
      params.operator_hold_rotation_surface_frame
          ? taskGain(params.operator_hold_KR_surface_diag)
          : params.operator_hold_KR_diag.asDiagonal();
  gains.DR_operator_hold =
      params.operator_hold_rotation_surface_frame
          ? taskGain(params.operator_hold_DR_surface_diag)
          : params.operator_hold_DR_diag.asDiagonal();

  return gains;
}

// ====================================================================
// Auto damping
// ====================================================================

StateDampingCache manualStateDampingCache(const StateImpedanceGains& gains) {
  // Initializing each damping branch with the configured manual matrices.
  StateDampingCache damping;
  damping.Dp_approach = gains.Dp_approach;
  damping.DR_approach = gains.DR_approach;
  damping.Dp_contact_establishment = gains.Dp_contact_establishment;
  damping.DR_contact_establishment = gains.DR_contact_establishment;
  damping.Dp_hold = gains.Dp_hold;
  damping.DR_hold = gains.DR_hold;
  damping.Dp_operator_hold = gains.Dp_operator_hold;
  damping.DR_operator_hold = gains.DR_operator_hold;
  return damping;
}

void updateAutoDamping(const ControllerConfig& params,
                       const StateImpedanceGains& gains,
                       const Model& model,
                       const RobotState& robot_state,
                       const Mat6x7& J,
                       ControlState state,
                       bool after_contact,
                       bool operator_hold_active,
                       StateDampingCache& damping) {
  // Selecting contact damping for contact-impedance pose hold.
  const bool hold_as_contact_establishment =
      state == ControlState::kPoseHold && params.use_contact_impedance_hold;
  const bool use_contact_establishment_branch = after_contact || hold_as_contact_establishment;
  // Selecting the active state group and invalidating inactive cached values.
  const bool in_approach = (state == ControlState::kToolOrientation ||
                            state == ControlState::kSurfaceApproach);
  if (!in_approach) {
    damping.approach_computed = false;
  }
  if (!use_contact_establishment_branch) {
    damping.contact_establishment_computed = false;
  }
  if (state != ControlState::kPoseHold || hold_as_contact_establishment) {
    damping.hold_computed = false;
  }
  if (!operator_hold_active) {
    damping.operator_hold_computed = false;
  }

  // Determining whether the current state requires a new inertia estimate.
  const bool need_damping_update =
      (operator_hold_active && params.operator_hold_auto_damping &&
       !damping.operator_hold_computed) ||
      (in_approach && params.approach_auto_damping && !damping.approach_computed) ||
      (use_contact_establishment_branch && params.contact_establishment_auto_damping &&
       !damping.contact_establishment_computed) ||
      (state == ControlState::kPoseHold && !hold_as_contact_establishment &&
       params.hold_auto_damping && !damping.hold_computed);
  if (need_damping_update) {
    // Loading the current joint-space mass matrix M(q).
    std::array<double, 49> mass_array = model.mass(robot_state);
    Map<const Mat7x7> joint_mass(mass_array.data());

    // Selecting zero or manual damping as the lower bound.
    auto dampingFloor = [&](const Vec3& manual) {
      return params.auto_damping_min_from_manual ? manual : Vec3::Zero();
    };

    if (operator_hold_active) {
      // Selecting base or surface axes for translation and for rotation.
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      const CartesianInertiaEstimate inertia_task =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_base_surface);
      const CartesianInertiaEstimate& inertia_translation =
          params.operator_hold_translation_surface_frame ? inertia_task : inertia_base;
      const CartesianInertiaEstimate& inertia_rotation =
          params.operator_hold_rotation_surface_frame ? inertia_task : inertia_base;
      if (inertia_translation.valid && inertia_rotation.valid) {
        // Calculating unit-ratio critical damping for gain matching.
        const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
            inertia_translation.translational,
            gains.operator_hold_Kp_active_diag,
            1.0, Vec3::Zero(), params.auto_damping_max);
        const Vec3 unit_critical_DR = criticalDampingFromStiffness(
            inertia_rotation.rotational,
            gains.operator_hold_KR_active_diag,
            1.0, Vec3::Zero(), params.auto_damping_max);

        // Fitting damping ratios to the configured hold-state damping.
        const Vec3& target_Dp = gains.operator_hold_Dp_active_diag;
        const Vec3& target_DR = gains.operator_hold_DR_active_diag;
        const auto fittedFactor = [](const Vec3& unit_critical,
                                     const Vec3& target) {
          const double denominator = unit_critical.squaredNorm();
          return (denominator > 1e-12)
                     ? unit_critical.dot(target) / denominator
                     : 1.0;
        };
        const double Dp_factor = fittedFactor(unit_critical_Dp, target_Dp);
        const double DR_factor = fittedFactor(unit_critical_DR, target_DR);

        // Assigning the fitted damping matrices and report values.
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_translation.translational,
            gains.operator_hold_Kp_active_diag,
            Dp_factor, dampingFloor(target_Dp), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_rotation.rotational,
            gains.operator_hold_KR_active_diag,
            DR_factor, dampingFloor(target_DR), params.auto_damping_max);
        // Assigning hold-state damping in the corresponding stiffness axes.
        damping.Dp_operator_hold =
            params.operator_hold_translation_surface_frame
                ? makeSpatialGainMatrix(Dp_diag, gains.R_base_surface)
                : Dp_diag.asDiagonal();
        damping.DR_operator_hold =
            params.operator_hold_rotation_surface_frame
                ? makeSpatialGainMatrix(DR_diag, gains.R_base_surface)
                : DR_diag.asDiagonal();
        // Storing damping values reported in the pre-contact hold state.
        damping.operator_hold_Dp_used = Dp_diag;
        damping.operator_hold_DR_used = DR_diag;
        damping.operator_hold_damping_valid = true;
      } else {
        damping.Dp_operator_hold = gains.Dp_operator_hold;
        damping.DR_operator_hold = gains.DR_operator_hold;
        printf("operator hold damping: inertia estimate unavailable, using manual "
               "Dp=[%.1f, %.1f, %.1f] Ns/m and DR=[%.1f, %.1f, %.1f] "
               "Nms/rad\n",
               gains.operator_hold_Dp_active_diag(0), gains.operator_hold_Dp_active_diag(1),
               gains.operator_hold_Dp_active_diag(2), gains.operator_hold_DR_active_diag(0),
               gains.operator_hold_DR_active_diag(1), gains.operator_hold_DR_active_diag(2));
      }
      damping.operator_hold_computed = true;
    } else if (in_approach) {
      // Calculating approach inertia and damping in surface axes.
      const CartesianInertiaEstimate inertia =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_base_surface);
      if (inertia.valid) {
        const double zeta = params.approach_auto_damping_factor;
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia.translational, params.approach_Kp_diag, zeta,
            dampingFloor(params.approach_Dp_diag), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia.rotational, params.approach_KR_diag, zeta,
            dampingFloor(params.approach_DR_diag), params.auto_damping_max);
        damping.Dp_approach = makeSpatialGainMatrix(Dp_diag, gains.R_base_surface);
        damping.DR_approach = makeSpatialGainMatrix(DR_diag, gains.R_base_surface);
        // Storing damping values reported at state entry.
        damping.approach_Dp_used = Dp_diag;
        damping.approach_DR_used = DR_diag;
        damping.approach_damping_valid = true;
        damping.approach_computed = true;
      }
    } else if (use_contact_establishment_branch) {
      // Selecting base or surface axes for translation and surface axes for rotation.
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      const CartesianInertiaEstimate inertia_task =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_base_surface);
      const CartesianInertiaEstimate& inertia_translation =
          params.contact_establishment_translation_surface_frame ? inertia_task : inertia_base;
      if (inertia_translation.valid && inertia_task.valid) {
        const double zeta = params.contact_establishment_auto_damping_factor;
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_translation.translational, gains.contact_establishment_Kp_active_diag, zeta,
            dampingFloor(gains.contact_establishment_Dp_active_diag), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_task.rotational, params.contact_establishment_KR_diag, zeta,
            dampingFloor(params.contact_establishment_DR_diag), params.auto_damping_max);
        // Assigning contact establishment damping in the same axes as the corresponding stiffness.
        damping.Dp_contact_establishment =
            params.contact_establishment_translation_surface_frame
                ? makeSpatialGainMatrix(Dp_diag, gains.R_base_surface)
                : Dp_diag.asDiagonal();
        damping.DR_contact_establishment = makeSpatialGainMatrix(DR_diag, gains.R_base_surface);
        damping.contact_establishment_Dp_used = Dp_diag;
        damping.contact_establishment_DR_used = DR_diag;
        damping.contact_establishment_damping_valid = true;
        // Storing damping values reported for contact establishment impedance.
        damping.contact_establishment_computed = true;
      }
    } else {
      // Calculating pose-hold inertia and damping in robot base axes.
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      if (inertia_base.valid) {
        const Vec3& manual_hold_Dp = params.hold_Dp_diag;
        const Vec3& manual_hold_DR = params.hold_DR_diag;
        double hold_factor = params.hold_auto_damping_factor;
        if (params.hold_auto_match_manual_damping) {
          const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
              inertia_base.translational,
              params.hold_Kp_diag,
              1.0, Vec3::Zero(), params.auto_damping_max);
          const double denominator = unit_critical_Dp.squaredNorm();
          if (denominator > 1e-12) {
            // Fitting one damping factor to the configured translational damping.
            hold_factor =
                unit_critical_Dp.dot(manual_hold_Dp) / denominator;
          }
          printf("hold auto damping: fitted factor=%.3f toward "
                 "Dp=[%.1f, %.1f, %.1f] Ns/m\n",
                 hold_factor, manual_hold_Dp(0), manual_hold_Dp(1),
                 manual_hold_Dp(2));
        }
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_base.translational,
            params.hold_Kp_diag,
            hold_factor,
            dampingFloor(manual_hold_Dp),
            params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_base.rotational,
            params.hold_KR_diag,
            hold_factor,
            dampingFloor(manual_hold_DR),
            params.auto_damping_max);
        damping.Dp_hold = Dp_diag.asDiagonal();
        damping.DR_hold = DR_diag.asDiagonal();
        damping.hold_Dp_used = Dp_diag;
        damping.hold_DR_used = DR_diag;
        damping.hold_damping_valid = true;
      } else {
        damping.Dp_hold = gains.Dp_hold;
        damping.DR_hold = gains.DR_hold;
        printf("hold damping: inertia estimate unavailable, using manual "
               "hold_Dp=[%.1f, %.1f, %.1f] Ns/m and "
               "hold_DR=[%.1f, %.1f, %.1f] Nms/rad\n",
               params.hold_Dp_diag(0), params.hold_Dp_diag(1),
               params.hold_Dp_diag(2), params.hold_DR_diag(0),
               params.hold_DR_diag(1), params.hold_DR_diag(2));
      }
      damping.hold_computed = true;
    }
  }
}
