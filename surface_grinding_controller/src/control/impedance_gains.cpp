// ============================================================================
// Phase impedance gains
// ============================================================================
// Building phase-dependent stiffness and damping matrices and calculating
// automatic damping from the operational-space inertia.
#include "controller_api.h"

// ====================================================================
// Gain table
// ====================================================================
// Building phase stiffness and damping matrices in the robot base frame.
PhaseImpedanceGains buildPhaseImpedanceGains(const ControllerConfig& params) {
  // Initializing the surface frame [tangent1, tangent2, normal].
  PhaseImpedanceGains gains;
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
  // Selecting active translational setup gains from base or surface axes.
  gains.setup_Kp_active_diag =
      params.setup_translation_surface_frame
          ? params.setup_Kp_surface_diag
          : params.setup_Kp_diag;
  gains.setup_Dp_active_diag =
      params.setup_translation_surface_frame
          ? params.setup_Dp_surface_diag
          : params.setup_Dp_diag;
  // Transforming the selected translational setup gains to base axes.
  gains.Kp_setup =
      params.setup_translation_surface_frame
          ? taskGain(params.setup_Kp_surface_diag)
          : params.setup_Kp_diag.asDiagonal();
  gains.Dp_setup =
      params.setup_translation_surface_frame
          ? taskGain(params.setup_Dp_surface_diag)
          : params.setup_Dp_diag.asDiagonal();
  // Assigning rotational setup gains in surface axes and expressing them in base axes.
  gains.KR_setup = taskGain(params.setup_KR_diag);
  gains.DR_setup = taskGain(params.setup_DR_diag);
  // Assigning Cartesian pose-hold gains in robot base axes.
  gains.Kp_hold = params.hold_Kp_diag.asDiagonal();
  gains.Dp_hold = params.hold_Dp_diag.asDiagonal();
  gains.KR_hold = params.hold_KR_diag.asDiagonal();
  gains.DR_hold = params.hold_DR_diag.asDiagonal();
  // Assigning phase-gate hold gains in base and surface axes.
  gains.Kp_pause = params.pause_hold_Kp_diag.asDiagonal();
  gains.Dp_pause = params.pause_hold_Dp_diag.asDiagonal();
  gains.KR_pause = taskGain(params.pause_hold_KR_diag);
  gains.DR_pause = taskGain(params.pause_hold_DR_diag);

  return gains;
}

// ====================================================================
// Auto damping
// ====================================================================

PhaseDampingCache manualPhaseDampingCache(const PhaseImpedanceGains& gains) {
  // Initializing each damping branch with the configured manual matrices.
  PhaseDampingCache damping;
  damping.Dp_approach = gains.Dp_approach;
  damping.DR_approach = gains.DR_approach;
  damping.Dp_setup = gains.Dp_setup;
  damping.DR_setup = gains.DR_setup;
  damping.Dp_hold = gains.Dp_hold;
  damping.DR_hold = gains.DR_hold;
  damping.Dp_pause = gains.Dp_pause;
  damping.DR_pause = gains.DR_pause;
  return damping;
}

void updateAutoDamping(const ControllerConfig& params,
                       const PhaseImpedanceGains& gains,
                       const Model& model,
                       const RobotState& state,
                       const Mat6x7& J,
                       ControlPhase phase,
                       bool after_contact,
                       bool pause_hold_active,
                       PhaseDampingCache& damping) {
  // Selecting setup damping for setup-impedance hold.
  const bool hold_as_setup =
      phase == ControlPhase::kPoseHold && params.use_setup_impedance_hold;
  const bool use_setup_branch = after_contact || hold_as_setup;
  // Selecting the active phase group and invalidating inactive cached values.
  const bool in_approach = (phase == ControlPhase::kToolOrientation ||
                            phase == ControlPhase::kSurfaceApproach);
  if (!in_approach) {
    damping.approach_computed = false;
  }
  if (!use_setup_branch) {
    damping.setup_computed = false;
  }
  if (phase != ControlPhase::kPoseHold || hold_as_setup) {
    damping.hold_computed = false;
  }
  if (!pause_hold_active) {
    damping.pause_computed = false;
  }

  // Determining whether the current phase requires a new inertia estimate.
  const bool need_damping_update =
      (pause_hold_active && params.pause_hold_auto_damping &&
       !damping.pause_computed) ||
      (in_approach && params.approach_auto_damping && !damping.approach_computed) ||
      (use_setup_branch && params.setup_auto_damping &&
       !damping.setup_computed) ||
      (phase == ControlPhase::kPoseHold && !hold_as_setup &&
       params.hold_auto_damping && !damping.hold_computed);
  if (need_damping_update) {
    // Loading the current joint-space mass matrix M(q).
    std::array<double, 49> mass_array = model.mass(state);
    Map<const Mat7x7> joint_mass(mass_array.data());

    // Selecting zero or manual damping as the lower bound.
    auto dampingFloor = [&](const Vec3& manual) {
      return params.auto_damping_min_from_manual ? manual : Vec3::Zero();
    };

    if (pause_hold_active) {
      // Calculating translational base-frame and rotational surface-frame inertia.
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      const CartesianInertiaEstimate inertia_task =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_base_surface);
      if (inertia_base.valid && inertia_task.valid) {
        // Calculating unit-ratio critical damping for gain matching.
        const Vec3 unit_critical_Dp = criticalDampingFromStiffness(
            inertia_base.translational,
            params.pause_hold_Kp_diag,
            1.0, Vec3::Zero(), params.auto_damping_max);
        const Vec3 unit_critical_DR = criticalDampingFromStiffness(
            inertia_task.rotational,
            params.pause_hold_KR_diag,
            1.0, Vec3::Zero(), params.auto_damping_max);

        // Fitting damping ratios to the configured gate-hold damping.
        const Vec3& target_Dp = params.pause_hold_Dp_diag;
        const Vec3& target_DR = params.pause_hold_DR_diag;
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
            inertia_base.translational,
            params.pause_hold_Kp_diag,
            Dp_factor, dampingFloor(target_Dp), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_task.rotational,
            params.pause_hold_KR_diag,
            DR_factor, dampingFloor(target_DR), params.auto_damping_max);
        damping.Dp_pause = Dp_diag.asDiagonal();
        damping.DR_pause =
            makeSpatialGainMatrix(DR_diag, gains.R_base_surface);
        // Storing damping values reported at the phase gate.
        damping.pause_Dp_used = Dp_diag;
        damping.pause_DR_used = DR_diag;
        damping.pause_damping_valid = true;
      } else {
        damping.Dp_pause = gains.Dp_pause;
        damping.DR_pause = gains.DR_pause;
        printf("pause damping: inertia estimate unavailable, using manual "
               "Dp=[%.1f, %.1f, %.1f] Ns/m and DR=[%.1f, %.1f, %.1f] "
               "Nms/rad\n",
               params.pause_hold_Dp_diag(0), params.pause_hold_Dp_diag(1),
               params.pause_hold_Dp_diag(2), params.pause_hold_DR_diag(0),
               params.pause_hold_DR_diag(1), params.pause_hold_DR_diag(2));
      }
      damping.pause_computed = true;
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
        // Storing damping values reported at phase entry.
        damping.approach_Dp_used = Dp_diag;
        damping.approach_DR_used = DR_diag;
        damping.approach_damping_valid = true;
        damping.approach_computed = true;
      }
    } else if (use_setup_branch) {
      // Selecting base or surface axes for translation and surface axes for rotation.
      const CartesianInertiaEstimate inertia_base =
          computeCartesianInertiaEstimate(joint_mass, J, Mat3::Identity());
      const CartesianInertiaEstimate inertia_task =
          computeCartesianInertiaEstimate(joint_mass, J, gains.R_base_surface);
      const CartesianInertiaEstimate& inertia_translation =
          params.setup_translation_surface_frame ? inertia_task : inertia_base;
      if (inertia_translation.valid && inertia_task.valid) {
        const double zeta = params.setup_auto_damping_factor;
        const Vec3 Dp_diag = criticalDampingFromStiffness(
            inertia_translation.translational, gains.setup_Kp_active_diag, zeta,
            dampingFloor(gains.setup_Dp_active_diag), params.auto_damping_max);
        const Vec3 DR_diag = criticalDampingFromStiffness(
            inertia_task.rotational, params.setup_KR_diag, zeta,
            dampingFloor(params.setup_DR_diag), params.auto_damping_max);
        // Assigning setup damping in the same axes as the corresponding stiffness.
        damping.Dp_setup =
            params.setup_translation_surface_frame
                ? makeSpatialGainMatrix(Dp_diag, gains.R_base_surface)
                : Dp_diag.asDiagonal();
        damping.DR_setup = makeSpatialGainMatrix(DR_diag, gains.R_base_surface);
        damping.setup_Dp_used = Dp_diag;
        damping.setup_DR_used = DR_diag;
        damping.setup_damping_valid = true;
        // Storing damping values reported for setup impedance.
        damping.setup_computed = true;
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
