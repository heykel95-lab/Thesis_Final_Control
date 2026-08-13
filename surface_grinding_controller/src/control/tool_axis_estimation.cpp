// ============================================================================
// Tool-axis estimation
// ============================================================================
// Defines the orientation comparisons and the least-squares axis solution used
// by the guided and the automatic grinding-tool-axis calibration utilities.
#include "controller_api.h"

double angleBetweenUnitVectorsDeg(const Vec3& first, const Vec3& second) {
  const double dot = std::max(-1.0, std::min(1.0, first.dot(second)));
  return (180.0 / M_PI) * std::acos(dot);
}

double orientationSeparationDeg(const Mat3& first, const Mat3& second) {
  const Eigen::AngleAxisd relative(first.transpose() * second);
  return (180.0 / M_PI) * std::abs(relative.angle());
}

Vec3 estimateInvariantAxis(const std::vector<Mat3>& samples,
                           const Vec3& nominal_axis_ee) {
  // Requiring at least two orientations before an axis is defined.
  if (samples.size() < 2) {
    throw std::runtime_error(
        "Tool-axis estimation requires at least two orientation samples.");
  }

  // Building the least-squares matrix for pairwise orientation differences [-].
  Mat3 normal_matrix = Mat3::Zero();
  for (std::size_t i = 0; i < samples.size(); ++i) {
    for (std::size_t j = i + 1; j < samples.size(); ++j) {
      const Mat3 difference = samples[i] - samples[j];
      normal_matrix += difference.transpose() * difference;
    }
  }

  // Selecting the unit eigenvector with the smallest residual [-].
  Eigen::SelfAdjointEigenSolver<Mat3> solver(normal_matrix);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Tool-axis eigenvalue decomposition failed.");
  }
  Vec3 axis_ee = solver.eigenvectors().col(0).normalized();

  // Selecting the sign that remains closest to the configured nominal axis [-].
  Vec3 reference_axis = nominal_axis_ee;
  if (reference_axis.norm() < 1e-9) {
    reference_axis = Vec3(0.0, 0.0, 1.0);
  }
  reference_axis.normalize();
  if (axis_ee.dot(reference_axis) < 0.0) {
    axis_ee = -axis_ee;
  }

  return axis_ee;
}
