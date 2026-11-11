#include "bw_serial/pelvis_jog_controller.hpp"

#include <algorithm>
#include <cmath>

namespace bw_serial
{

PelvisJogController::PelvisJogController(const PelvisJogConfig & config)
: config_(config),
  target_height_mm_(clamp(config.default_height_mm, config.min_height_mm, config.max_height_mm)),
  startup_init_active_(config.startup_init_enabled),
  startup_init_elapsed_sec_(0.0)
{
}

PelvisJogOutput PelvisJogController::step(float raw_input, double dt_sec)
{
  PelvisJogOutput output;
  output.normalized_input = apply_deadzone(raw_input, config_.input_deadzone);

  if (dt_sec <= 0.0) {
    output.target_height_mm = target_height_mm_;
    return output;
  }

  const float velocity_mm_s = output.normalized_input * config_.max_speed_mm_s;
  target_height_mm_ += velocity_mm_s * static_cast<float>(dt_sec);
  target_height_mm_ = clamp(target_height_mm_, config_.min_height_mm, config_.max_height_mm);

  output.target_height_mm = target_height_mm_;
  output.max_velocity_mm_s = std::fabs(velocity_mm_s);
  return output;
}

PelvisJogOutput PelvisJogController::startup_initialization_step(
  bool has_feedback,
  float feedback_height_mm,
  double dt_sec)
{
  PelvisJogOutput output;
  output.target_height_mm = target_height_mm_;

  if (!startup_init_active_) {
    return output;
  }

  if (dt_sec > 0.0) {
    startup_init_elapsed_sec_ += dt_sec;
  }

  const float safe_tolerance = std::max(config_.startup_init_tolerance_mm, 0.0F);
  const bool reached_target =
    has_feedback && std::fabs(feedback_height_mm - target_height_mm_) <= safe_tolerance;
  const bool timed_out =
    config_.startup_init_timeout_sec > 0.0 &&
    startup_init_elapsed_sec_ >= config_.startup_init_timeout_sec;

  if (reached_target || timed_out) {
    startup_init_active_ = false;
    output.startup_initialization_completed = true;
    return output;
  }

  output.startup_initializing = true;
  output.max_velocity_mm_s = config_.max_speed_mm_s;
  return output;
}

void PelvisJogController::reset()
{
  target_height_mm_ = clamp(config_.default_height_mm, config_.min_height_mm, config_.max_height_mm);
  startup_init_active_ = config_.startup_init_enabled;
  startup_init_elapsed_sec_ = 0.0;
}

void PelvisJogController::stop_startup_initialization()
{
  startup_init_active_ = false;
}

void PelvisJogController::sync_target_height_mm(float feedback_height_mm)
{
  target_height_mm_ = clamp(feedback_height_mm, config_.min_height_mm, config_.max_height_mm);
}

float PelvisJogController::target_height_mm() const
{
  return target_height_mm_;
}

bool PelvisJogController::startup_initialization_active() const
{
  return startup_init_active_;
}

float PelvisJogController::apply_deadzone(float raw_input, float deadzone)
{
  const float clamped_input = clamp(raw_input, -1.0F, 1.0F);
  const float abs_input = std::fabs(clamped_input);
  const float safe_deadzone = clamp(deadzone, 0.0F, 0.999F);

  if (abs_input <= safe_deadzone) {
    return 0.0F;
  }

  const float scaled = (abs_input - safe_deadzone) / (1.0F - safe_deadzone);
  return std::copysign(scaled, clamped_input);
}

float PelvisJogController::clamp(float value, float min_value, float max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

namespace
{

std::string normalize_robot_version(const std::string & robot_version)
{
  if (robot_version == "1.0" || robot_version == "1") {
    return "1.0";
  }
  if (robot_version == "3.0" || robot_version == "3") {
    return "3.0";
  }
  if (robot_version == "2.0" || robot_version == "2") {
    return "2.0";
  }
  if (robot_version == "standard" || robot_version == "std") {
    return "standard";
  }
  return "2.0";
}

}  // namespace

PelvisJogConfig default_pelvis_jog_config_for_robot_version(const std::string & robot_version)
{
  PelvisJogConfig config;
  const std::string normalized = normalize_robot_version(robot_version);

  if (normalized == "3.0") {
    config.default_height_mm = 900.0F;
    config.min_height_mm = 600.0F;
    config.max_height_mm = 1000.0F;
    return config;
  }

  if (normalized == "standard") {
    config.default_height_mm = -100.0F;
    config.min_height_mm = -500.0F;
    config.max_height_mm = 0.0F;
    return config;
  }

  // 1.0 研发预留：当前与 2.0 一致。
  config.default_height_mm = 1500.0F;
  config.min_height_mm = 900.0F;
  config.max_height_mm = 1500.0F;
  return config;
}

PelvisJogConfig default_pelvis_jog_config_for_protocol(const std::string & protocol_version)
{
  if (protocol_version == "v3" || protocol_version == "V3") {
    return default_pelvis_jog_config_for_robot_version("3.0");
  }
  if (protocol_version == "v2" || protocol_version == "V2") {
    return default_pelvis_jog_config_for_robot_version("2.0");
  }
  return default_pelvis_jog_config_for_robot_version("2.0");
}

}  // namespace bw_serial
