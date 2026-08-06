// ============================================================================
// Surface-plane calibration tool
// ============================================================================
// Reading one robot state with the tool face seated on the plane, calculating
// one point on the plane and its normal direction, and reporting the values
// used in surface.conf. Guiding the operator through one read-only measurement.
#include "controller_api.h"

namespace {

// Transforming a unit normal into the configured x- and y-axis tilts [deg].
// The inverse relation is n = R_y(b) R_x(a) [0,0,1]^T.
void tiltAnglesFromNormal(const Vec3& normal,
                          double& tilt_x_deg,
                          double& tilt_y_deg) {
  // Limiting the inverse-sine argument to the numerical domain [-1, 1].
  const double tilt_x_rad =
      std::asin(std::max(-1.0, std::min(1.0, -normal(1))));

  // Calculating the y-axis tilt from the x and z normal components [rad].
  const double tilt_y_rad = std::atan2(normal(0), normal(2));

  // Converting the measured tilts from radians to degrees [deg].
  tilt_x_deg = (180.0 / M_PI) * tilt_x_rad;
  tilt_y_deg = (180.0 / M_PI) * tilt_y_rad;
}

}  // namespace

int main() {
  try {
    // Loading the robot address, tool geometry, and surface definition.
    const ControllerConfig config =
        readControllerConfig(parameterFiles());

    // Printing the calibration procedure before connecting to the robot.
    printf("SURFACE PLANE CALIBRATION\n");
    printf("Read-only measurement. Keep the tool stationary.\n");
    printf("Seat the tool face flat on the plane before continuing.\n");
    printf("Connecting to robot: %s\n\n", config.robot_ip.c_str());

    // Connecting to the robot and reading one measured state.
    Robot robot(config.robot_ip);
    const RobotState state = robot.readOnce();

    // Mapping the measured end-effector transformation O_T_EE [-].
    Map<const Mat4x4> T_EE(state.O_T_EE.data());

    // Extracting end-effector orientation [-] and TCP position [m].
    const Mat3 R_EE = T_EE.block<3, 3>(0, 0);
    const Vec3 p_EE = T_EE.block<3, 1>(0, 3);

    // Initializing and normalizing the controlled tool axis in the EE frame [-].
    Vec3 tool_axis_ee = config.tool_axis_ee;
    if (tool_axis_ee.norm() < 1e-9) {
      tool_axis_ee = Vec3(0.0, 0.0, 1.0);
    }
    tool_axis_ee.normalize();

    // Transforming the tool axis from the EE frame to the robot base frame [-].
    Vec3 measured_normal_base = R_EE * tool_axis_ee;

    // Selecting the outward normal convention with a positive base z component.
    if (measured_normal_base(2) < 0.0) {
      measured_normal_base = -measured_normal_base;
    }
    measured_normal_base.normalize();

    // Transforming the contact-face center from the EE frame to the base frame [m].
    const Vec3 measured_surface_point_base =
        p_EE + R_EE * config.tool_contact_face_center_ee;

    // Calculating the measured surface tilt angles [deg].
    double measured_tilt_x_deg = 0.0;
    double measured_tilt_y_deg = 0.0;
    tiltAnglesFromNormal(measured_normal_base,
                         measured_tilt_x_deg,
                         measured_tilt_y_deg);

    // Initializing the configured surface normal in the robot base frame [-].
    Vec3 configured_normal_base = config.surface_normal_base;
    if (configured_normal_base(2) < 0.0) {
      configured_normal_base = -configured_normal_base;
    }
    configured_normal_base.normalize();

    // Calculating the angular mismatch between both unit normals [deg].
    const double normal_dot = std::max(
        -1.0,
        std::min(1.0, measured_normal_base.dot(configured_normal_base)));
    const double mismatch_deg =
        (180.0 / M_PI) * std::acos(normal_dot);

    // Calculating the configured-plane offset at the measured face center [m].
    const double configured_plane_offset = configured_normal_base.dot(
        measured_surface_point_base - config.surface_point);

    // Reporting the measured TCP position [mm] and tool-axis definition [-].
    printf("TCP position [mm]      = [%+8.1f, %+8.1f, %+8.1f]\n",
           1000.0 * p_EE(0), 1000.0 * p_EE(1), 1000.0 * p_EE(2));
    printf("tool_axis_ee           = [%+7.3f, %+7.3f, %+7.3f]\n",
           tool_axis_ee(0), tool_axis_ee(1), tool_axis_ee(2));
    printf("face center in EE [mm] = [%+8.1f, %+8.1f, %+8.1f]\n\n",
           1000.0 * config.tool_contact_face_center_ee(0),
           1000.0 * config.tool_contact_face_center_ee(1),
           1000.0 * config.tool_contact_face_center_ee(2));

    // Reporting the measured and configured points on the plane [mm].
    printf("measured surface point = [%+8.1f, %+8.1f, %+8.1f] mm\n",
           1000.0 * measured_surface_point_base(0),
           1000.0 * measured_surface_point_base(1),
           1000.0 * measured_surface_point_base(2));
    printf("configured point       = [%+8.1f, %+8.1f, %+8.1f] mm\n",
           1000.0 * config.surface_point(0),
           1000.0 * config.surface_point(1),
           1000.0 * config.surface_point(2));
    printf("configured-plane offset = %+8.2f mm\n\n",
           1000.0 * configured_plane_offset);

    // Reporting measured and configured plane normals in the base frame [-].
    printf("measured plane normal  = [%+7.4f, %+7.4f, %+7.4f]\n",
           measured_normal_base(0),
           measured_normal_base(1),
           measured_normal_base(2));
    printf("configured normal      = [%+7.4f, %+7.4f, %+7.4f]\n\n",
           configured_normal_base(0),
           configured_normal_base(1),
           configured_normal_base(2));

    // Reporting measured, configured, and residual tilt angles [deg].
    printf("MEASURED tilt: a(x) = %+6.2f deg | b(y) = %+6.2f deg\n",
           measured_tilt_x_deg, measured_tilt_y_deg);
    printf("CONFIGURED   : a(x) = %+6.2f deg | b(y) = %+6.2f deg\n",
           config.surface_tilt_x_deg,
           config.surface_tilt_y_deg);
    printf("difference   : a(x) = %+6.2f deg | b(y) = %+6.2f deg\n\n",
           measured_tilt_x_deg - config.surface_tilt_x_deg,
           measured_tilt_y_deg - config.surface_tilt_y_deg);

    // Printing the total orientation mismatch [deg].
    printf("total mismatch between measured and configured normal: %.2f deg\n",
           mismatch_deg);

    // Printing values that can be copied directly into params/surface.conf.
    printf("\nCopy the measured values to params/surface.conf:\n");
    printf("surface_point_x = %.6f\n", measured_surface_point_base(0));
    printf("surface_point_y = %.6f\n", measured_surface_point_base(1));
    printf("surface_point_z = %.6f\n", measured_surface_point_base(2));
    printf("surface_tilt_x_deg = %.6f\n", measured_tilt_x_deg);
    printf("surface_tilt_y_deg = %.6f\n", measured_tilt_y_deg);
    if (config.use_start_as_surface_point) {
      printf("\nSet use_start_as_surface_point = 0 to use the calibrated point.\n");
    }
    printf("\nRun the calibration again after editing the file. A correctly\n"
           "calibrated plane gives a small normal mismatch and a configured-\n"
           "plane offset close to zero while the tool face remains seated.\n");

    return 0;
  } catch (const franka::Exception& e) {
    // Reporting robot communication errors.
    fprintf(stderr, "libfranka exception: %s\n", e.what());
    return -1;
  } catch (const std::exception& e) {
    // Reporting configuration or numerical errors.
    fprintf(stderr, "Exception: %s\n", e.what());
    return -1;
  }
}
