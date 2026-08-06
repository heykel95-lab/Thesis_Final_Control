// ============================================================================
// Numerical utilities
// ============================================================================
// Defining fixed-size conversions, matrix utilities, and smooth trajectory
// functions used by the runtime and control modules.
#include "controller_api.h"

// ====================================================================
// Small numeric helpers
// ====================================================================

Array7 vec7ToArray(const Vec7& v) {
  // Converting an Eigen joint vector to the libfranka array representation.
  Array7 array{};
  for (int i = 0; i < 7; ++i) {
    array[i] = v(i);
  }
  return array;
}

Array7 filledArray7(double value) {
  // Initializing a seven-element array with one scalar value.
  Array7 array{};
  array.fill(value);
  return array;
}

Array6 filledArray6(double value) {
  // Initializing a six-element array with one scalar value.
  Array6 array{};
  array.fill(value);
  return array;
}

double smallestSingularValue(const Mat6x7& J) {
  // Calculating sigma_min(J) for the 6x7 geometric Jacobian [-].
  Eigen::JacobiSVD<Mat6x7> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.singularValues().minCoeff();
}

Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback) {
  // Selecting a normalized input or a normalized fallback direction [-].
  if (v.norm() > 1e-9) {
    return v.normalized();
  }
  return fallback.normalized();
}

Mat3 skewMatrix(const Vec3& v) {
  // Defining S(v) such that S(v) x = v cross x.
  Mat3 s;
  s << 0.0, -v(2), v(1),
       v(2), 0.0, -v(0),
       -v(1), v(0), 0.0;
  return s;
}

// ====================================================================
// Trajectory primitives
// ====================================================================

double smoothStep(double r) {
  // Limiting normalized trajectory time to r in [0, 1].
  r = std::max(0.0, std::min(1.0, r));

  // Evaluating the fifth-order position interpolation [-].
  return 10.0 * std::pow(r, 3) - 15.0 * std::pow(r, 4) + 6.0 * std::pow(r, 5);
}

double smoothStepDerivative(double r, double T) {
  // Limiting normalized trajectory time and returning zero at both endpoints.
  r = std::max(0.0, std::min(1.0, r));
  if (r <= 0.0 || r >= 1.0) {
    return 0.0;
  }

  // Calculating ds/dt from ds/dr and the motion duration T [s].
  const double ds_dr =
      30.0 * std::pow(r, 2)
    - 60.0 * std::pow(r, 3)
    + 30.0 * std::pow(r, 4);
  return ds_dr / T;
}

void grindSweep(double t, double amplitude, double stroke_duration,
                double& s, double& s_dot) {
  // Returning zero displacement [m] and velocity [m/s] for an invalid duration.
  if (stroke_duration <= 1e-9) {
    s = 0.0;
    s_dot = 0.0;
    return;
  }
  // Selecting the active half-stroke from elapsed time t [s].
  const double tau = std::max(0.0, t) / stroke_duration;
  const int k = static_cast<int>(std::floor(tau));
  const double r = tau - std::floor(tau);
  // Assigning alternating endpoints 0, +1, -1, +1, ... .
  auto endpoint = [](int i) -> double {
    if (i <= 0) return 0.0;
    return (i % 2 == 1) ? 1.0 : -1.0;
  };
  // Calculating smooth sweep displacement [m] and velocity [m/s].
  const double a0 = endpoint(k);
  const double a1 = endpoint(k + 1);
  s = amplitude * (a0 + (a1 - a0) * smoothStep(r));
  s_dot = amplitude * (a1 - a0) * smoothStepDerivative(r, stroke_duration);
}

double grindStrokeDuration(const ControllerConfig& params) {
  // Converting sweep frequency [Hz] to one half-stroke duration [s].
  return (params.grind_frequency_hz > 1e-9) ? (0.5 / params.grind_frequency_hz) : 0.0;
}

double setupPush(const ControllerConfig& params,
                 double phase_time,
                 double start_push,
                 double& push_speed) {
  // Defining the remaining virtual penetration distance [m].
  const double delta = params.setup_push_end - start_push;

  // Selecting the configured penetration speed magnitude [m/s].
  const double speed_magnitude = std::abs(params.setup_push_speed);
  if (std::abs(delta) <= 1e-12 || speed_magnitude <= 1e-12) {
    push_speed = 0.0;
    return (std::abs(delta) <= 1e-12) ? params.setup_push_end : start_push;
  }

  // Assigning penetration direction and signed speed [m/s].
  const double direction = (delta >= 0.0) ? 1.0 : -1.0;
  const double signed_speed = direction * speed_magnitude;
  // Integrating the unclamped virtual penetration [m].
  const double unclamped = start_push + signed_speed * std::max(0.0, phase_time);
  const bool end_reached = (direction > 0.0)
                               ? (unclamped >= params.setup_push_end)
                               : (unclamped <= params.setup_push_end);
  // Clamping the motion at the configured final penetration [m].
  push_speed = end_reached ? 0.0 : signed_speed;
  return end_reached ? params.setup_push_end : unclamped;
}

