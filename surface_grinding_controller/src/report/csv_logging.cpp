// ============================================================================
// CSV logging
// ============================================================================
// Writing recorded control samples after the real-time callback has ended.
#include "controller_api.h"

#include <cerrno>
#include <cstring>
#include <sys/stat.h>

namespace {

void writeVec3(std::ofstream& out, const Vec3& v) {
  // Writing three values in their stored SI units.
  out << v(0) << "," << v(1) << "," << v(2) << ",";
}

void writeVec7(std::ofstream& out, const Vec7& v) {
  // Writing seven joint-space values in their stored SI units.
  for (int i = 0; i < 7; ++i) {
    out << v(i) << ",";
  }
}

void ensureParentDirectory(const std::string& path) {
  // Selecting and creating the parent directory of the CSV path.
  const std::size_t slash = path.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return;
  }
  const std::string dir = path.substr(0, slash);
  if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
    fprintf(stderr, "Could not create %s: %s\n", dir.c_str(), strerror(errno));
  }
}

}  // namespace

void writeLogToCsv(
    const std::vector<LogData>& log_data,
    const std::string& csv_file_name) {
  // Creating the output directory and opening the CSV file.
  ensureParentDirectory(csv_file_name);
  std::ofstream log_file(csv_file_name);

  // Writing the fixed column order used by the analysis workflow.
  log_file << "time,state,"
           << "p_EE_x,p_EE_y,p_EE_z,"
           << "p_d_x,p_d_y,p_d_z,"
           << "tool_contact_x,tool_contact_y,tool_contact_z,"
           << "first_contact_tcp_x,first_contact_tcp_y,first_contact_tcp_z,"
           << "first_contact_x,first_contact_y,first_contact_z,"
           << "edge_target_x,edge_target_y,edge_target_z,"
           << "tool_contact_offset_ee_x,tool_contact_offset_ee_y,tool_contact_offset_ee_z,"
           << "p_CoC_x,p_CoC_y,p_CoC_z,"
           << "r_eff_x,r_eff_y,r_eff_z,"
           << "e_p_x,e_p_y,e_p_z,"
           << "e_R_x,e_R_y,e_R_z,"
           << "angular_deviation_t1_deg,angular_deviation_t2_deg,"
           << "angular_deviation_normal_deg,"
           << "angular_deviation_deg,"
           << "t_align,"
           << "pdot_x,pdot_y,pdot_z,"
           << "pdot_d_x,pdot_d_y,pdot_d_z,"
           << "omega_x,omega_y,omega_z,"
           << "f_x,f_y,f_z,"
           << "m_x,m_y,m_z,"
           << "external_force_x,external_force_y,external_force_z,"
           << "external_moment_x,external_moment_y,external_moment_z,"
           << "contact_force_bias_x,contact_force_bias_y,contact_force_bias_z,"
           << "contact_moment_bias_x,contact_moment_bias_y,contact_moment_bias_z,"
           << "force_after_contact_x,force_after_contact_y,force_after_contact_z,"
           << "moment_after_contact_x,moment_after_contact_y,moment_after_contact_z,"
           << "push,nullspace_mode,tau_nullspace_norm,nullspace_damping,"
           << "disturbance_scale,disturbance_torque_scale,"
           << "disturbance_point_base_x,disturbance_point_base_y,"
           << "disturbance_point_base_z,"
           << "disturbance_force_base_x,disturbance_force_base_y,"
           << "disturbance_force_base_z,"
           << "tau_disturbance_1,tau_disturbance_2,tau_disturbance_3,"
           << "tau_disturbance_4,tau_disturbance_5,tau_disturbance_6,"
           << "tau_disturbance_7,"
           << "tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,tau_cmd_6,tau_cmd_7"
           << "\n";

  // Writing each recorded sample in chronological order.
  log_file << std::fixed << std::setprecision(9);
  for (const auto& row : log_data) {
    log_file << row.time << "," << row.state << ",";
    writeVec3(log_file, row.p_EE);
    writeVec3(log_file, row.p_d);
    writeVec3(log_file, row.tool_contact_point);
    writeVec3(log_file, row.first_contact_tcp);
    writeVec3(log_file, row.first_contact_point);
    writeVec3(log_file, row.edge_target);
    writeVec3(log_file, row.tool_contact_offset_ee);
    writeVec3(log_file, row.p_CoC);
    writeVec3(log_file, row.r_eff);
    writeVec3(log_file, row.e_p);
    writeVec3(log_file, row.e_R);
    writeVec3(log_file, (180.0 / M_PI) * row.angular_deviation_surface);
    log_file << (180.0 / M_PI) * row.angular_deviation << ",";
    log_file << row.t_align << ",";
    writeVec3(log_file, row.pdot);
    writeVec3(log_file, row.pdot_d);
    writeVec3(log_file, row.omega);
    writeVec3(log_file, row.f);
    writeVec3(log_file, row.m);
    writeVec3(log_file, row.external_force);
    writeVec3(log_file, row.external_moment);
    writeVec3(log_file, row.contact_force_bias);
    writeVec3(log_file, row.contact_moment_bias);
    writeVec3(log_file, Vec3(row.external_force - row.contact_force_bias));
    writeVec3(log_file, Vec3(row.external_moment - row.contact_moment_bias));
    log_file << row.push << ","
             << row.nullspace_mode << ","
             << row.tau_nullspace_norm << ","
             << row.nullspace_damping << ","
             << row.disturbance.scale << ","
             << row.disturbance.torque_scale << ",";
    writeVec3(log_file, row.disturbance.point_base);
    writeVec3(log_file, row.disturbance.force_base);
    writeVec7(log_file, row.disturbance.tau);
    log_file << row.tau_cmd(0) << "," << row.tau_cmd(1) << ","
             << row.tau_cmd(2) << "," << row.tau_cmd(3) << ","
             << row.tau_cmd(4) << "," << row.tau_cmd(5) << ","
             << row.tau_cmd(6) << "\n";
  }
}
