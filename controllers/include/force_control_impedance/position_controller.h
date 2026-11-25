/**
 * POSITION CONTROLLER
 *
 * This file is part of the package:
 * https://github.com/mikelasa/force_control_impedance
 *
 * Cartesian position (pose-tracking) controller. Unlike ImpedanceController,
 * which outputs joint torques and realises the spring itself, this controller
 * outputs a *pose command* and delegates tracking to the robot's own internal
 * controller (on a Franka: ControllerMode::kCartesianImpedance +
 * MotionGeneratorMode::kCartesianPosition). In that sense it is the structural
 * twin of AdmittanceController, which also outputs a pose.
 *
 * It exists because a vision-only policy (e.g. vanilla Diffusion Policy) emits
 * an absolute pose command meant to be *tracked*, not a spring equilibrium to
 * be hung off. Under ImpedanceController such a command settles with a
 * steady-state error of f/K, which is 25 mm at K=800 N/m and 20 N of contact.
 *
 * What this controller does on top of a straight pass-through is bound the
 * command in three ways, all of which matter when the reference comes from a
 * policy that replans a whole horizon at a time:
 *
 *   1. rate limit     - caps the per-cycle change of the command, so a
 *                       replanning discontinuity becomes a ramp.
 *   2. tracking leash - keeps the command within a fixed distance of the
 *                       measured pose. Since the robot's internal controller
 *                       is itself a spring of stiffness K_int, this puts a
 *                       ceiling on the force it can develop:
 *                       |f| <= K_int * max_pose_error_m.
 *                       At K_int = 1000 N/m and a 20 mm leash that is ~20 N.
 *                       It also stops the command winding away from the robot
 *                       when motion is blocked.
 *   3. low-pass       - optional first-order smoothing of the reference. The
 *                       Franka interface already low-passes in setCartesian(),
 *                       so this defaults to off.
 *
 * Poses are [x, y, z, qw, qx, qy, qz] throughout, matching the rest of the
 * package.
 */

#pragma once
#ifndef _POSITION_CONTROLLER_H_
#define _POSITION_CONTROLLER_H_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <fstream>

class PositionController {
 public:
  /**
   * Configuration structure for PositionController initialization
   */
  struct PositionControllerConfig {
    double dt{0.001};               // Control period (s)
    bool log_to_file{false};        // Enable data logging to file
    std::string log_file_path{""};  // Path for log file output
    int log_max_samples{600000};    // Pre-allocated log buffer (600000 = 10 min at 1kHz)
    bool alert_overrun{false};      // Print warning when step() exceeds dt

    /**
     * Limits applied to the outgoing pose command. Every one of them is
     * disabled by setting it to 0, which turns this controller into a plain
     * pass-through of the reference.
     */
    struct TrackingLimits {
      double max_translation_step_m{0.0};  // Max |Δposition| per step (m)
      double max_rotation_step_rad{0.0};   // Max rotation per step (rad)
      double max_pose_error_m{0.0};        // Leash radius on position (m)
      double max_pose_error_rad{0.0};      // Leash radius on orientation (rad)
      double lowpass_cutoff_hz{0.0};       // First-order filter on the reference (Hz)
    };
    TrackingLimits tracking{};
  };

  // ========== Constructor/Destructor ==========

  PositionController();
  ~PositionController();
  PositionController(PositionController&&);

  // ========== Initialization ==========

  /**
   * Initialize the position controller
   *
   * @param time0         Start time reference point for controller timing
   * @param config        Configuration structure
   * @param pose_current  Current robot pose, used as the initial reference and
   *                      the initial command so the first step() is a no-op
   * @return true if initialization successful
   */
  bool init(const RUT::TimePoint& time0,
            const PositionControllerConfig& config,
            const RUT::Vector7d& pose_current);

  // ========== Robot State Management ==========

  /**
   * Update robot status with current measurements
   *
   * @param pose_WT   Current tool frame pose in world frame
   * @param wrench_T  Tool wrench feedback in tool frame. Not used by the
   *                  control law -- kept for interface parity with the other
   *                  controllers, and logged.
   */
  void setRobotStatus(const RUT::Vector7d& pose_WT,
                      const RUT::Vector6d& wrench_T);

  /**
   * Set the pose reference command
   *
   * @param pose_WT     Target tool pose in world frame
   * @param wrench_WTr  Target wrench. Ignored: this controller does no force
   *                    control. Kept for interface parity.
   */
  void setRobotReference(const RUT::Vector7d& pose_WT,
                         const RUT::Vector6d& wrench_WTr);

  // ========== Control Mode Configuration ==========

  /**
   * Present for interface parity with Impedance/AdmittanceController so the
   * ManipServer construction path can stay uniform.
   *
   * @note No-op. Every axis is position-controlled by definition here; there
   *       is no force-controlled subspace to select. Calling it with n_af > 0
   *       prints a warning rather than silently doing nothing.
   */
  void setForceControlledAxis(const RUT::Matrix6d& Tr, int n_af);

  // ========== Control Execution ==========

  /**
   * Execute one control step and compute the pose command to send
   *
   * @param pose  Output pose command [x, y, z, qw, qx, qy, qz]
   * @return 0 on success, 1 if the result was non-finite (command held)
   */
  int step(RUT::Vector7d& pose);

  // ========== State Management ==========

  /**
   * Reset internal state. The command is re-seeded from the last known
   * feedback pose, so the next step() starts from where the robot actually is.
   * Call this when switching from one trajectory source to another.
   */
  void reset();

  // ========== Debugging and Monitoring ==========

  /** Flush the in-memory log buffer to disk. Safe to call more than once. */
  void flushLog();

  /** Print current controller state to console. */
  void displayStates();

 private:
  // Pimpl idiom: Private implementation pointer
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif  // _POSITION_CONTROLLER_H_
