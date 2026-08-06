// ============================================================================
// Controller module interface
// ============================================================================
// Declares the configuration, geometry, control, robot-support, reporting, and
// logging functions implemented in src/. main.cpp uses this interface to
// assemble the complete controller workflow.
#pragma once

#include "controller_data.h"

// ---------------------------------------------------------------------------
// Configuration input
// ---------------------------------------------------------------------------

/// Removes leading and trailing whitespace from one text value.
std::string trim(const std::string& input);

/// Loads the listed parameter files and returns the validated configuration.
/// @param filenames Parameter files in increasing load order.
ControllerConfig readControllerConfig(const std::vector<std::string>& filenames);

/// Returns the absolute directory containing the running executable.
std::string executableDirectory();

/// Returns the resolved params/ directory for the controller or utility.
std::string defaultParameterDirectory();

/// Returns all parameter files in the required load order.
/// @param dir Parameter directory; an empty value uses the resolved default.
std::vector<std::string> parameterFiles(const std::string& dir = "");

// ---------------------------------------------------------------------------
// Numerical and trajectory utilities
// ---------------------------------------------------------------------------

/// Copies a seven-axis Eigen vector into a libfranka-compatible array.
Array7 vec7ToArray(const Vec7& v);

/// Creates a seven-element array with one repeated value.
Array7 filledArray7(double value);

/// Creates a six-element array with one repeated value.
Array6 filledArray6(double value);

/// Returns the smallest singular value of a 6x7 Jacobian [-].
double smallestSingularValue(const Mat6x7& J);

/// Returns a normalized vector or a normalized fallback for a small input.
Vec3 normalizedOrFallback(const Vec3& v, const Vec3& fallback);

/// Builds S(v) such that S(v)x = v cross x.
Mat3 skewMatrix(const Vec3& v);

/// Evaluates a cubic smooth-step profile for r in [0,1] [-].
double smoothStep(double r);

/// Evaluates the time derivative of the smooth-step profile [1/s].
double smoothStepDerivative(double r, double T);

/// Calculates the periodic grinding displacement and velocity.
/// @param t Phase time [s].
/// @param amplitude Sweep half-amplitude [m].
/// @param stroke_duration Duration of one smooth stroke [s].
/// @param s Returned tangential displacement [m].
/// @param s_dot Returned tangential velocity [m/s].
void grindSweep(double t, double amplitude, double stroke_duration,
                double& s, double& s_dot);

/// Returns the configured duration of one grinding stroke [s].
double grindStrokeDuration(const ControllerConfig& params);

/// Calculates the setup penetration profile and its velocity.
/// @param phase_time Time from setup entry [s].
/// @param start_push Penetration at setup entry [m].
/// @param push_speed Returned penetration velocity [m/s].
/// @return Current virtual penetration [m].
double setupPush(const ControllerConfig& params,
                 double phase_time,
                 double start_push,
                 double& push_speed);

// ---------------------------------------------------------------------------
// Spatial gain transformations
// ---------------------------------------------------------------------------

/// Combines translational and rotational 3x3 matrices into a 6x6 matrix.
Mat6x6 blockDiagonal(const Mat3& translational, const Mat3& rotational);

/// Builds the 6D shift matrix for r_c = p_TCP - p_C [m].
Mat6x6 complianceShiftMatrix(const Vec3& r_c);

/// Shifts stiffness or damping from the compliance center to the TCP.
/// @param gain_at_center Cartesian gain defined at the virtual center.
/// @param r_c Vector from the compliance center to the TCP [m].
Mat6x6 shiftGainToTcp(const Mat6x6& gain_at_center, const Vec3& r_c);

// ---------------------------------------------------------------------------
// Cartesian impedance and surface geometry
// ---------------------------------------------------------------------------

/// Builds diag(R,R) for transforming a 6D Cartesian quantity.
Mat6x6 blockDiagonalRotation(const Mat3& R);

/// Transforms a diagonal task-frame gain into base coordinates.
Mat3 makeSpatialGainMatrix(const Vec3& diagonal_in_task_frame,
                           const Mat3& R_task);

/// Estimates diagonal Cartesian inertia in the selected task frame.
/// @param joint_mass Joint-space inertia matrix [kg, kg m^2].
/// @param J Geometric Jacobian in the base frame.
/// @param R_task Task frame expressed in the base frame.
CartesianInertiaEstimate computeCartesianInertiaEstimate(
    const Mat7x7& joint_mass,
    const Mat6x7& J,
    const Mat3& R_task);

/// Calculates diagonal critical damping from inertia and stiffness.
/// Translational entries return [N s/m]; rotational entries return [N m s/rad].
Vec3 criticalDampingFromStiffness(const Vec3& inertia,
                                  const Vec3& stiffness,
                                  double damping_ratio,
                                  const Vec3& min_damping,
                                  double max_damping);

/// Returns the current-to-desired orientation error in the base frame [rad].
Vec3 orientationError(const Mat3& R_current, const Mat3& R_desired);

/// Constructs an orthonormal surface frame [tangent1,tangent2,normal].
Mat3 makeSurfaceFrameFromNormalTangent(
    const Vec3& normal_input, const Vec3& tangent1_input);

/// Constructs the configured surface frame expressed in the base frame.
Mat3 makeSurfaceFrame(const ControllerConfig& params);

/// Returns the shortest rotation that maps one unit vector to another.
Mat3 rotationBetweenUnitVectors(const Vec3& from_unit, const Vec3& to_unit);

/// Returns the nominal surface-facing tool axis in the base frame [-].
Vec3 surfaceToolAxisInBase(
    const ControllerConfig& params, const Mat3& R_base_surface);

/// Returns the tilted target tool axis in the base frame [-].
Vec3 desiredToolAxisInBase(
    const ControllerConfig& params, const Mat3& R_base_surface);

/// Transforms the configured EE-frame tool axis into the base frame [-].
Vec3 currentToolAxisInBase(
    const ControllerConfig& params, const Mat3& R_EE);

/// Returns the tool-axis alignment error in the base frame [rad].
Vec3 toolSurfaceAlignmentErrorBase(
    const ControllerConfig& params,
    const Mat3& R_EE,
    const Mat3& R_base_surface);

/// Returns the residual angle between the tool axis and its target [rad].
double toolSurfaceMisalignmentAngle(
    const ControllerConfig& params,
    const Mat3& R_EE,
    const Mat3& R_base_surface);

/// Constructs the desired EE orientation from surface alignment and twist.
Mat3 makeToolTargetOrientation(
    const ControllerConfig& params,
    const Mat3& R_base_surface,
    const Mat3& R_start);

/// Removes disabled rotational-error components in the surface frame [rad].
Vec3 applyRotationalAxisMask(
    const ControllerConfig& params, Vec3 e_R, const Mat3& R_base_surface);

// ---------------------------------------------------------------------------
// Terminal formatting
// ---------------------------------------------------------------------------

/// Prints a full-width program banner.
void printBanner(const char* title);

/// Prints a labelled terminal section.
void printSection(const char* title);

/// Prints one horizontal separator.
void printRule();

/// Prints one labelled three-axis value with units and an optional note.
void printRow(const char* label, const Vec3& v, const char* unit,
              const char* note);

// ---------------------------------------------------------------------------
// Robot support and control laws
// ---------------------------------------------------------------------------

/// Starts the non-real-time keyboard thread that writes operator requests.
void startKeyboardStopThread(const ControllerConfig& params,
                             KeyboardSignals& signals);

/// Assigns the configured collision thresholds to the robot.
void configureCollisionBehavior(Robot& robot, const ControllerConfig& params);

/// Calculates the Cartesian impedance wrench [F;M].
/// @param dx Pose error [position in m; orientation in rad].
/// @param dv Velocity error [linear in m/s; angular in rad/s].
/// @return Force [N] and moment [N m], expressed in the base frame.
Vec6 computeCartesianImpedanceWrench(const ControllerConfig& params,
                                    ControlPhase phase,
                                    const Mat3& Kp,
                                    const Mat3& Dp,
                                    const Mat3& KR,
                                    const Mat3& DR,
                                    const Mat3& R_base_surface,
                                    const Vec6& dx,
                                    const Vec6& dv,
                                    const Mat3& R_EE);

/// Calculates projected nullspace damping and singular-value conditioning.
/// @param J Geometric Jacobian in the base frame.
/// @param dq Measured joint velocity [rad/s].
/// @return Nullspace joint torque [N m].
Vec7 computeNullspaceTorque(
    const ControllerConfig& params,
    const Model& model,
    const RobotState& state,
    const Mat6x7& J,
    const Vec7& dq);

/// Validates the automatic point-force disturbance configuration.
bool validateAutomaticDisturbance(const ControllerConfig& params,
                                  std::string& error);

/// Returns the smooth disturbance scale at the hold-phase time [-].
double automaticDisturbanceScale(const ControllerConfig& params,
                                 double hold_time);

/// Calculates the linear-velocity Jacobian of a link-fixed point.
/// @param point_link Point coordinates in the selected link frame [m].
/// @return Point Jacobian expressed in the base frame [m/rad].
Eigen::Matrix<double, 3, 7> pointJacobian(
    const Mat6x7& link_jacobian,
    const Mat3& R_link,
    const Vec3& point_link);

/// Selects the base-frame force direction that excites the redundant axis.
Vec3 automaticDisturbanceDirection(const ControllerConfig& params,
                                   const Model& model,
                                   const RobotState& state,
                                   const Mat6x7& ee_jacobian);

/// Calculates the configured point force and equivalent joint torque.
AutomaticDisturbance computeAutomaticDisturbance(
    const ControllerConfig& params,
    const Model& model,
    const RobotState& state,
    const Vec3& force_direction_base,
    double hold_time);

/// Checks whether the calibrated gripper stroke covers the commanded widths.
bool gripperWidthCalibrated(const franka::GripperState& state,
                            const ControllerConfig& params);

/// Prints the measured gripper stroke and required calibration action.
void reportGripperCalibration(const franka::GripperState& state,
                              const ControllerConfig& params);

/// Opens the gripper to the configured width [m] and speed [m/s].
bool openGripper(const ControllerConfig& params, Gripper& gripper);

/// Grasps the tool with the configured width [m] and force [N].
bool graspTool(const ControllerConfig& params, Gripper& gripper);

/// Performs Franka Hand homing after operator confirmation.
bool recalibrateGripper(const ControllerConfig& params, Gripper& gripper);

/// Writes the guided joint posture [rad] to params/initial_pose.conf.
bool saveGuidedPoseAsQInit(const Vec7& q);

/// Displays the startup menu and assigns the selected run mode.
bool askStartupRunMode(ControllerConfig& params, Robot& robot,
                       const Model& model);

/// Executes the gripper action selected in the startup menu.
bool performStartupGripperAction(const ControllerConfig& params);

// ---------------------------------------------------------------------------
// Phase gains and real-time execution
// ---------------------------------------------------------------------------

/// Builds the base-frame impedance matrices for every controller phase.
PhaseImpedanceGains buildPhaseImpedanceGains(const ControllerConfig& params);

/// Initializes the phase-damping cache from the configured damping matrices.
PhaseDampingCache manualPhaseDampingCache(const PhaseImpedanceGains& gains);

/// Updates inertia-based damping when a phase or contact state becomes active.
void updateAutoDamping(const ControllerConfig& params,
                       const PhaseImpedanceGains& gains,
                       const Model& model,
                       const RobotState& state,
                       const Mat6x7& J,
                       ControlPhase phase,
                       bool after_contact,
                       bool pause_hold_active,
                       PhaseDampingCache& damping);

/// Executes one complete 1 kHz torque-control run and returns its samples.
RunResult runControlLoop(ControllerConfig& params,
                         Robot& robot,
                         const Model& model,
                         PhaseImpedanceGains gains,
                         KeyboardSignals& signals);

/// Writes the post-run summary and chronological CSV log.
void writeRunLogs(const ControllerConfig& params, const RunResult& result);

/// Adds a numbered session suffix before the filename extension.
std::string sessionFileName(const std::string& name, int session);

/// Solves a joint posture at the specified tool-axis standoff [m].
/// @param q_target Target joint posture [rad].
/// @param q_standoff Returned joint posture [rad].
bool solveStandoffPosture(const Model& model,
                          const RobotState& state,
                          const Array7& q_target,
                          double standoff,
                          Array7& q_standoff);

/// Checks joint limits and returns the one-based violating joint index.
bool withinJointLimits(const Array7& q, int& joint_out);

/// Runs manual guidance from the current robot pose and routes the selected exit.
bool runManualGuidanceStart(ControllerConfig& params,
                            Robot& robot,
                            const Model& model,
                            KeyboardSignals& signals);

// ---------------------------------------------------------------------------
// Operator output and data logging
// ---------------------------------------------------------------------------

/// Returns the terminal label for one controller phase.
const char* phaseName(ControlPhase phase);

/// Returns the terminal label for one nullspace mode.
const char* nullspaceModeName(NullspaceMode mode);

/// Prints a three-axis position or displacement in millimetres [mm].
void printVec3Mm(const char* label, const Vec3& v);

/// Prints a three-axis angular value in degrees [deg].
void printVec3Deg(const char* label, const Vec3& v);

/// Prints a three-axis gain vector using its caller-defined label.
void printGainVec(const char* label, const Vec3& v);

/// Prints a seven-joint posture in degrees [deg].
void printVec7Deg(const char* label, const Vec7& v);

/// Prints a complete 6x6 Cartesian gain matrix.
void printSpatialGain6(const char* label, const Mat6x6& M);

/// Prints the eigenvalues of a symmetric 6x6 Cartesian gain matrix.
void printSpatialGainEigenvalues(const char* label, const Mat6x6& M);

/// Prints the start and final joint postures in degrees [deg].
void printJointStartEndTableDeg(const Vec7& q_start, const Vec7& q_final);

/// Prints the active phase name and section heading.
void printPhaseHeader(ControlPhase phase);

/// Prints the impedance and damping selected for the active phase.
void printPhaseIntro(const ControllerConfig& params,
                     const PhaseDampingCache& damping,
                     ControlPhase phase);

/// Prints the impedance applied during a phase-transition hold.
void printGateHold(const ControllerConfig& params,
                   const PhaseDampingCache& damping);

/// Prints setup gains and the selected virtual compliance-center definition.
void printSetupImpedanceLaw(const ControllerConfig& params,
                            const PhaseDampingCache& damping,
                            bool tunable,
                            const Mat3& R_base_surface,
                            const Mat3& R_EE);

/// Prints the active nullspace law and available live commands.
void printNullspaceLaw(const ControllerConfig& params);

/// Prints the automatic disturbance force direction and timing.
void printAutomaticDisturbance(const ControllerConfig& params,
                               const Vec3& force_direction_base);

/// Prints the selected contact-feature geometry in the base and EE frames [m].
void printContactEdgeDebug(const Vec3& offset_ee,
                           const Vec3& p_EE_at_contact,
                           const Vec3& contact_point);

/// Prints tool-orientation progress and angular errors [deg].
void printApproachOrientDebug(double phase_time,
                              double axis_error_deg,
                              double spin_error_deg);

/// Prints surface-approach distance, clearance, and force [mm, N].
void printApproachDescendDebug(double phase_time,
                               double distance_mm,
                               double height_mm,
                               double target_height_mm,
                               double force_n);

/// Prints setup tilt, force, moment, and tool-feature displacement.
void printSetupDebug(double phase_time,
                     double tip_deg,
                     double force_n,
                     double moment_nm,
                     double moment_limit_nm,
                     double edge_mm);

/// Prints grinding sweep, tracking error, and normal force [mm, N].
void printGrindDebug(double phase_time,
                     double sweep_mm,
                     double track_error_mm,
                     double press_n);

/// Prints hold force, position error, and orientation error [N, mm, deg].
void printHoldDebug(double phase_time,
                    double force_n,
                    double pos_error_mm,
                    double rot_error_deg);

/// Prints the final desired pose, measured pose, errors, and log filename.
void printFinalSummary(const Vec3& final_p_d,
                       const Vec3& final_p_EE,
                       const Vec3& final_e_p,
                       const Vec3& final_e_R,
                       const std::string& csv_file_name);

/// Writes all chronological control samples to the selected CSV file.
void writeLogToCsv(const std::vector<LogData>& log_data,
                   const std::string& csv_file_name);

/// Prints the setup termination condition, final pose, wrench, and gains.
void reportSetupResult(const ControllerConfig& params,
                       const Mat3& R_base_surface,
                       const SetupReport& report);
