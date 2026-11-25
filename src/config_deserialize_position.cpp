#include "force_control_impedance/config_deserialize_position.h"
#include <RobotUtilities/spatial_utilities.h>

#include <yaml-cpp/yaml.h>

#include "force_control_impedance/position_controller.h"

namespace {

/** Read an optional scalar, falling back to the struct's default. A missing
 *  limit means "disabled", which is a legitimate choice, so it must not be a
 *  parse error. */
double optional_double(const YAML::Node& node, const std::string& key,
                       double fallback) {
  if (!node || !node[key]) return fallback;
  return node[key].as<double>();
}

}  // namespace

template <>
bool deserialize(const YAML::Node& node,
                 PositionController::PositionControllerConfig& config) {
  try {
    config.dt = node["dt"].as<double>();
    config.log_to_file = node["log_to_file"].as<bool>();
    config.log_file_path = node["log_file_path"].as<std::string>();
    config.alert_overrun = node["alert_overrun"].as<bool>();

    // Every tracking limit is optional; 0 disables that limit.
    const YAML::Node& tracking = node["tracking"];
    config.tracking.max_translation_step_m = optional_double(
        tracking, "max_translation_step_m",
        config.tracking.max_translation_step_m);
    config.tracking.max_rotation_step_rad = optional_double(
        tracking, "max_rotation_step_rad", config.tracking.max_rotation_step_rad);
    config.tracking.max_pose_error_m = optional_double(
        tracking, "max_pose_error_m", config.tracking.max_pose_error_m);
    config.tracking.max_pose_error_rad = optional_double(
        tracking, "max_pose_error_rad", config.tracking.max_pose_error_rad);
    config.tracking.lowpass_cutoff_hz = optional_double(
        tracking, "lowpass_cutoff_hz", config.tracking.lowpass_cutoff_hz);

  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  // validity check
  if (config.dt <= 0) {
    std::cerr << "Invalid dt: " << config.dt << std::endl;
    return false;
  }

  // All limits must be non-negative; 0 means disabled.
  const auto& t = config.tracking;
  const std::pair<const char*, double> limits[] = {
      {"tracking.max_translation_step_m", t.max_translation_step_m},
      {"tracking.max_rotation_step_rad", t.max_rotation_step_rad},
      {"tracking.max_pose_error_m", t.max_pose_error_m},
      {"tracking.max_pose_error_rad", t.max_pose_error_rad},
      {"tracking.lowpass_cutoff_hz", t.lowpass_cutoff_hz},
  };
  for (const auto& limit : limits) {
    if (limit.second < 0) {
      std::cerr << "Invalid " << limit.first << ": " << limit.second
                << ". Must be non-negative (0 disables the limit)." << std::endl;
      return false;
    }
  }

  // A rate limit larger than the leash can never bind, which almost always
  // means a unit mix-up (per-step metres vs total metres).
  if (t.max_pose_error_m > 0 && t.max_translation_step_m > t.max_pose_error_m) {
    std::cerr << "Invalid parameters. tracking.max_translation_step_m ("
              << t.max_translation_step_m
              << ") exceeds tracking.max_pose_error_m (" << t.max_pose_error_m
              << "). The step limit is per control cycle, so it should be far "
                 "the smaller of the two."
              << std::endl;
    return false;
  }

  // Running with no leash is allowed but worth saying out loud: the command
  // can then walk away from the robot and the internal controller's force is
  // bounded only by the robot's own collision thresholds.
  if (t.max_pose_error_m <= 0) {
    std::cerr << "[PositionController config] warning: "
                 "tracking.max_pose_error_m is 0, so the position leash is "
                 "disabled and contact force is unbounded by this controller."
              << std::endl;
  }

  return true;
}
