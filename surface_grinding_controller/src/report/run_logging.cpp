// ============================================================================
// Run reporting
// ============================================================================
// Coordinating terminal summaries and CSV output after each controller run,
// and creating numbered filenames for repeated sessions.
#include "controller_api.h"

// Adding a session suffix to repeated-run filenames.
std::string sessionFileName(const std::string& name, int session) {
  // Inserting the session number before the filename extension.
  const std::size_t dot = name.find_last_of('.');
  const std::string suffix = "_s" + std::to_string(session);
  return (dot == std::string::npos) ? name + suffix
                                    : name.substr(0, dot) + suffix +
                                          name.substr(dot);
}

// ====================================================================
// 4. Post-run report
// ====================================================================
void writeRunLogs(const ControllerConfig& params, const RunResult& result) {
  // Reporting an approach-distance stop before the standard run summary.
  if (result.descend_failed) {
    printSection("descend stopped");
    printf("  maximum distance reached before the clearance height.\n");
  }
  // Printing joint motion, writing CSV samples, and printing final errors.
  printJointStartEndTableDeg(result.q_start, result.final_q);
  writeLogToCsv(result.log, params.csv_file_name);
  printFinalSummary(result.final_p_d, result.final_p_EE, result.final_e_p,
                    result.final_e_R, params.csv_file_name);
}
