/**
 * POSITION CONTROLLER
 *
 * Purpose:
 *   Cartesian position (pose-tracking) controller. Outputs a bounded pose
 *   command and leaves the tracking to the robot's own internal controller.
 *   See position_controller.h for why the bounds are the substance of this
 *   class and what each one buys.
 *
 * This file is part of the package:
 *   https://github.com/mikelasa/force_control_impedance
 */

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control_impedance/position_controller.h>

#include <array>
#include <cmath>
#include <iostream>
#include <string>

using RUT::Matrix6d;
using RUT::Quaterniond;
using RUT::Vector3d;
using RUT::Vector6d;
using RUT::Vector7d;

// MATLAB-style formatting for Eigen matrices
static const Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n",
                                       "", "", "[", "]");

namespace {

/** Pull a quaternion onto the same hemisphere as a reference, so that
 *  interpolating between them takes the short way round. */
inline Quaterniond align_hemisphere(const Quaterniond& q,
                                    const Quaterniond& reference) {
  if (reference.dot(q) < 0.0) {
    return Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
  }
  return q;
}

/** Rotation angle, in radians, of the relative rotation from a to b. */
inline double angle_between(const Quaterniond& a, const Quaterniond& b) {
  const double dot = std::fabs(a.normalized().dot(b.normalized()));
  // guard acos against dot slipping just outside [-1, 1] through rounding
  return 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
}

/** Read a Vector7d pose into translation + unit quaternion. */
inline void split_pose(const Vector7d& pose, Vector3d& trans, Quaterniond& rot) {
  trans = pose.head(3);
  rot = Quaterniond(pose[3], pose[4], pose[5], pose[6]);  // (w, x, y, z)
  rot.normalize();
}

/** Write translation + quaternion back into a Vector7d pose. */
inline void join_pose(const Vector3d& trans, const Quaterniond& rot,
                      Vector7d& pose) {
  const Quaterniond q = rot.normalized();
  pose[0] = trans[0];
  pose[1] = trans[1];
  pose[2] = trans[2];
  pose[3] = q.w();
  pose[4] = q.x();
  pose[5] = q.y();
  pose[6] = q.z();
}

}  // namespace

/**
 * Private implementation structure for PositionController
 *
 * Thread Safety:
 *   Not internally synchronized; callers must serialise access (in this
 *   package, through the ManipServer controller mutex).
 */
struct PositionController::Implementation {
  Implementation();
  ~Implementation();

  bool initialize(const RUT::TimePoint& time0,
                  const PositionControllerConfig& config);

  void setRobotStatus(const Vector7d& pose_WT, const Vector6d& wrench_T);
  void setRobotReference(const Vector7d& pose_WT, const Vector6d& wrench_WTr);
  void setForceControlledAxis(const Matrix6d& Tr, int n_af);

  int step(Vector7d& pose_to_send);

  void reset();
  void logStates();
  void flushLog();
  void displayStates();

  // ========== Configuration ==========
  PositionControllerConfig config{};

  // ========== Internal Controller States ==========

  // Feedback
  Vector7d pose_fb{Vector7d::Zero()};
  Vector3d pose_fb_trans{Vector3d::Zero()};
  Quaterniond pose_fb_rot{Quaterniond::Identity()};
  Vector6d wrench_fb{Vector6d::Zero()};

  // Reference (what the trajectory source asked for)
  Vector7d pose_ref{Vector7d::Zero()};
  Vector3d pose_ref_trans{Vector3d::Zero()};
  Quaterniond pose_ref_rot{Quaterniond::Identity()};

  // Command (what we actually send, after the limits)
  Vector7d pose_cmd{Vector7d::Zero()};
  Vector3d pose_cmd_trans{Vector3d::Zero()};
  Quaterniond pose_cmd_rot{Quaterniond::Identity()};

  bool has_feedback{false};  // false until the first setRobotStatus()

  // Which limits engaged on the last step, for logging and displayStates()
  bool limit_hit_lowpass{false};
  bool limit_hit_rate{false};
  bool limit_hit_leash{false};

  // Timing. `timer` is tic'ed once, in initialize(), from the server's time0,
  // so toc_ms() is absolute time on the same clock as the pose/wrench/rgb
  // buffers -- which is what makes the log correlatable with them. Never tic()
  // it again. `step_timer` is the separate stopwatch for per-step duration.
  RUT::Timer timer{};
  RUT::Timer step_timer{};
  double time_now_ms{0.0};

  // ========== Logging ==========
  std::ofstream log_file{};

  // Pre-allocated to avoid allocations in the RT loop
  struct LogEntry {
    double timestamp_ms;
    std::array<double, 7> pose_ref;
    std::array<double, 7> pose_cmd;
    std::array<double, 7> pose_fb;
    double pos_error_m;
    double rot_error_rad;
    unsigned char flags;  // bit0 lowpass, bit1 rate, bit2 leash
  };
  std::vector<LogEntry> log_buffer{};
};

// ========== Implementation Constructor/Destructor ==========

PositionController::Implementation::Implementation() {}

PositionController::Implementation::~Implementation() {
  flushLog();
}

void PositionController::Implementation::flushLog() {
  if (!config.log_to_file || !log_file.is_open() || log_buffer.empty()) return;
  const size_t n = log_buffer.size();
  for (const auto& entry : log_buffer) {
    log_file << entry.timestamp_ms;
    for (double v : entry.pose_ref) log_file << " " << v;
    for (double v : entry.pose_cmd) log_file << " " << v;
    for (double v : entry.pose_fb) log_file << " " << v;
    log_file << " " << entry.pos_error_m;
    log_file << " " << entry.rot_error_rad;
    log_file << " " << static_cast<int>(entry.flags);
    log_file << "\n";
  }
  log_file.flush();
  log_file.close();
  log_buffer.clear();
  std::cout << "[PositionController] log flushed (" << n << " samples)"
            << std::endl;
}

// ========== Implementation Initialization ==========

bool PositionController::Implementation::initialize(
    const RUT::TimePoint& time_initial_0,
    const PositionControllerConfig& position_controller_config) {
  std::cout << "[PositionController] Begin initialization.\n";

  // ========================================================================
  // Step 1: Load configuration and start synchronized timer
  // ========================================================================
  config = position_controller_config;
  timer.tic(time_initial_0);

  // ========================================================================
  // Step 2: Reset all internal states to default values
  // ========================================================================
  reset();

  // ========================================================================
  // Step 3: Initialize data logging (optional)
  // ========================================================================
  if (config.log_to_file) {
    log_file.open(config.log_file_path);
    if (log_file.is_open()) {
      log_buffer.reserve(config.log_max_samples);
      std::cout << "[PositionController] log file opened successfully at "
                << config.log_file_path << " (buffer: " << config.log_max_samples
                << " samples)" << std::endl;
    } else {
      std::cerr << "[PositionController] Failed to open log file at "
                << config.log_file_path << std::endl;
    }
  }

  // ========================================================================
  // Step 4: Announce which limits are active, since a zero silently
  //         disables one and that is easy to miss in a yaml
  // ========================================================================
  const auto& t = config.tracking;
  std::cout << "[PositionController] limits:"
            << " translation_step="
            << (t.max_translation_step_m > 0.0
                    ? std::to_string(t.max_translation_step_m) + " m"
                    : "off")
            << ", rotation_step="
            << (t.max_rotation_step_rad > 0.0
                    ? std::to_string(t.max_rotation_step_rad) + " rad"
                    : "off")
            << ", leash_pos="
            << (t.max_pose_error_m > 0.0
                    ? std::to_string(t.max_pose_error_m) + " m"
                    : "off")
            << ", leash_rot="
            << (t.max_pose_error_rad > 0.0
                    ? std::to_string(t.max_pose_error_rad) + " rad"
                    : "off")
            << ", lowpass="
            << (t.lowpass_cutoff_hz > 0.0
                    ? std::to_string(t.lowpass_cutoff_hz) + " Hz"
                    : "off")
            << std::endl;

  return true;
}

// ========== Robot State Management Implementation ==========

void PositionController::Implementation::setRobotStatus(
    const Vector7d& pose_WT, const Vector6d& wrench_T) {
  pose_fb = pose_WT;
  wrench_fb = wrench_T;
  split_pose(pose_fb, pose_fb_trans, pose_fb_rot);

  // The very first feedback also seeds the command, so that step() cannot
  // send a stale or zero pose before the robot's true pose is known.
  if (!has_feedback) {
    has_feedback = true;
    pose_cmd = pose_fb;
    split_pose(pose_cmd, pose_cmd_trans, pose_cmd_rot);
    pose_ref = pose_fb;
    split_pose(pose_ref, pose_ref_trans, pose_ref_rot);
  }
}

void PositionController::Implementation::setRobotReference(
    const Vector7d& pose_WT, const Vector6d& wrench_WTr) {
  (void)wrench_WTr;  // no force control here
  pose_ref = pose_WT;
  split_pose(pose_ref, pose_ref_trans, pose_ref_rot);
}

// ========== Control Mode Configuration Implementation ==========

void PositionController::Implementation::setForceControlledAxis(
    const Matrix6d& Tr, int n_af) {
  (void)Tr;
  if (n_af != 0) {
    std::cerr << "[PositionController] setForceControlledAxis(n_af=" << n_af
              << ") ignored: this controller position-controls every axis. "
                 "Use ImpedanceController or AdmittanceController if you need "
                 "a force-controlled subspace."
              << std::endl;
  }
}

// ========== Control Execution Implementation ==========

/*
Control step.

The reference is passed through three bounds, in this order:

  1. low-pass toward the reference   (optional, smooths the input)
  2. rate limit vs the last command  (turns a replan jump into a ramp)
  3. leash vs the measured pose      (bounds the force the robot can develop,
                                      and stops command windup when blocked)

The leash is applied last so it is a hard guarantee. Because the next step's
rate limit is measured from the *clamped* command rather than from the
reference, a blocked robot cannot accumulate an ever-growing command: the pair
of limits is self-consistent, with no integral term to wind up.
*/
int PositionController::Implementation::step(Vector7d& pose_to_send) {
  time_now_ms = timer.toc_ms();
  if (config.alert_overrun) {
    step_timer.tic();
  }

  limit_hit_lowpass = false;
  limit_hit_rate = false;
  limit_hit_leash = false;

  // Until the first feedback arrives we have nothing safe to send; hold.
  if (!has_feedback) {
    pose_to_send = pose_cmd;
    return 0;
  }

  const auto& lim = config.tracking;

  Vector3d target_trans = pose_ref_trans;
  Quaterniond target_rot = align_hemisphere(pose_ref_rot, pose_cmd_rot);

  // ------------------------------------------------------------------------
  // Phase 1: First-order low-pass of the reference
  // ------------------------------------------------------------------------
  // alpha = dt / (tau + dt), tau = 1 / (2*pi*fc)
  if (lim.lowpass_cutoff_hz > 0.0 && config.dt > 0.0) {
    const double tau = 1.0 / (2.0 * M_PI * lim.lowpass_cutoff_hz);
    const double alpha = config.dt / (tau + config.dt);
    target_trans = pose_cmd_trans + alpha * (target_trans - pose_cmd_trans);
    target_rot = pose_cmd_rot.slerp(alpha, target_rot);
    limit_hit_lowpass = true;
  }

  // ------------------------------------------------------------------------
  // Phase 2: Rate limit relative to the previous command
  // ------------------------------------------------------------------------
  if (lim.max_translation_step_m > 0.0) {
    const Vector3d delta = target_trans - pose_cmd_trans;
    const double dist = delta.norm();
    if (dist > lim.max_translation_step_m) {
      target_trans = pose_cmd_trans + delta * (lim.max_translation_step_m / dist);
      limit_hit_rate = true;
    }
  }
  if (lim.max_rotation_step_rad > 0.0) {
    const double angle = angle_between(pose_cmd_rot, target_rot);
    if (angle > lim.max_rotation_step_rad) {
      target_rot =
          pose_cmd_rot.slerp(lim.max_rotation_step_rad / angle, target_rot);
      limit_hit_rate = true;
    }
  }

  // ------------------------------------------------------------------------
  // Phase 3: Leash relative to the measured pose
  // ------------------------------------------------------------------------
  // With the robot's internal controller acting as a spring of stiffness
  // K_int, this caps the developed force at K_int * max_pose_error_m.
  if (lim.max_pose_error_m > 0.0) {
    const Vector3d error = target_trans - pose_fb_trans;
    const double dist = error.norm();
    if (dist > lim.max_pose_error_m) {
      target_trans = pose_fb_trans + error * (lim.max_pose_error_m / dist);
      limit_hit_leash = true;
    }
  }
  if (lim.max_pose_error_rad > 0.0) {
    const Quaterniond fb_aligned = align_hemisphere(pose_fb_rot, target_rot);
    const double angle = angle_between(fb_aligned, target_rot);
    if (angle > lim.max_pose_error_rad) {
      target_rot = fb_aligned.slerp(lim.max_pose_error_rad / angle, target_rot);
      limit_hit_leash = true;
    }
  }

  // ------------------------------------------------------------------------
  // Phase 4: Commit, guarding against non-finite values
  // ------------------------------------------------------------------------
  Vector7d candidate{Vector7d::Zero()};
  join_pose(target_trans, target_rot, candidate);

  if (!candidate.allFinite()) {
    std::cerr << "[PositionController] non-finite pose command computed; "
                 "holding the previous command.\n"
              << "  pose_ref: " << pose_ref.transpose() << "\n"
              << "  pose_fb:  " << pose_fb.transpose() << std::endl;
    pose_to_send = pose_cmd;
    return 1;
  }

  pose_cmd = candidate;
  split_pose(pose_cmd, pose_cmd_trans, pose_cmd_rot);
  pose_to_send = pose_cmd;

  logStates();

  if (config.alert_overrun) {
    const double elapsed_ms = step_timer.toc_ms();
    if (elapsed_ms > config.dt * 1000.0) {
      std::cerr << "[PositionController] step took too long: " << elapsed_ms
                << "ms" << std::endl;
    }
  }

  return 0;
}

// ========== State Management Implementation ==========

void PositionController::Implementation::reset() {
  // Re-seed the command from the last known feedback, so the next step()
  // starts from where the robot actually is rather than from a stale target.
  if (has_feedback) {
    pose_cmd = pose_fb;
    pose_ref = pose_fb;
  } else {
    pose_cmd = Vector7d::Zero();
    pose_cmd[3] = 1.0;  // identity quaternion
    pose_ref = pose_cmd;
  }
  split_pose(pose_cmd, pose_cmd_trans, pose_cmd_rot);
  split_pose(pose_ref, pose_ref_trans, pose_ref_rot);

  limit_hit_lowpass = false;
  limit_hit_rate = false;
  limit_hit_leash = false;
}

// ========== Debugging and Monitoring Implementation ==========

void PositionController::Implementation::logStates() {
  if (!config.log_to_file || !log_file.is_open()) return;
  if (log_buffer.size() >= static_cast<size_t>(config.log_max_samples)) return;

  LogEntry entry{};
  entry.timestamp_ms = time_now_ms;
  for (int i = 0; i < 7; ++i) {
    entry.pose_ref[i] = pose_ref[i];
    entry.pose_cmd[i] = pose_cmd[i];
    entry.pose_fb[i] = pose_fb[i];
  }
  entry.pos_error_m = (pose_cmd_trans - pose_fb_trans).norm();
  entry.rot_error_rad = angle_between(pose_fb_rot, pose_cmd_rot);
  entry.flags = static_cast<unsigned char>((limit_hit_lowpass ? 1 : 0) |
                                           (limit_hit_rate ? 2 : 0) |
                                           (limit_hit_leash ? 4 : 0));
  log_buffer.push_back(entry);
}

void PositionController::Implementation::displayStates() {
  std::cout << "[PositionController] states:" << std::endl;
  std::cout << "  pose_ref: " << pose_ref.transpose() << std::endl;
  std::cout << "  pose_cmd: " << pose_cmd.transpose() << std::endl;
  std::cout << "  pose_fb:  " << pose_fb.transpose() << std::endl;
  std::cout << "  position error: " << (pose_cmd_trans - pose_fb_trans).norm()
            << " m" << std::endl;
  std::cout << "  rotation error: " << angle_between(pose_fb_rot, pose_cmd_rot)
            << " rad" << std::endl;
  std::cout << "  limits engaged: lowpass=" << limit_hit_lowpass
            << " rate=" << limit_hit_rate << " leash=" << limit_hit_leash
            << std::endl;
}

// ========== PositionController Class Implementation ==========

PositionController::PositionController()
    : m_impl{std::make_unique<Implementation>()} {}

PositionController::~PositionController() = default;

PositionController::PositionController(PositionController&&) = default;

bool PositionController::init(const RUT::TimePoint& time0,
                              const PositionControllerConfig& config,
                              const RUT::Vector7d& pose_current) {
  m_impl->initialize(time0, config);

  // Seed both feedback and reference from the current pose, so the first
  // step() is a no-op that re-sends where the robot already is.
  setRobotStatus(pose_current, RUT::Vector6d::Zero());
  setRobotReference(pose_current, RUT::Vector6d::Zero());

  RUT::Vector7d pose_out;
  step(pose_out);

  std::cout << "[PositionController] initialization is done." << std::endl;
  return true;
}

void PositionController::setRobotStatus(const RUT::Vector7d& pose_WT,
                                         const RUT::Vector6d& wrench_T) {
  m_impl->setRobotStatus(pose_WT, wrench_T);
}

void PositionController::setRobotReference(const RUT::Vector7d& pose_WT,
                                            const RUT::Vector6d& wrench_WTr) {
  m_impl->setRobotReference(pose_WT, wrench_WTr);
}

void PositionController::setForceControlledAxis(const Matrix6d& Tr, int n_af) {
  m_impl->setForceControlledAxis(Tr, n_af);
}

int PositionController::step(RUT::Vector7d& pose_to_send) {
  return m_impl->step(pose_to_send);
}

void PositionController::reset() {
  m_impl->reset();
}

void PositionController::flushLog() {
  m_impl->flushLog();
}

void PositionController::displayStates() {
  m_impl->displayStates();
}
