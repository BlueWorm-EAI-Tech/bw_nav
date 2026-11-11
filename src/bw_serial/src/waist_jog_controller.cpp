#include "bw_serial/waist_jog_controller.hpp"

#include <algorithm>
#include <cmath>

namespace bw_serial
{

WaistJogController::WaistJogController(const WaistJogConfig & config)
: config_(config),
  target_position_deg_(clamp(config.default_position_deg, config.min_position_deg, config.max_position_deg))
{
}

WaistJogOutput WaistJogController::step(float raw_input, double dt_sec)
{
  WaistJogOutput output;
  output.normalized_input = apply_deadzone(raw_input, config_.input_deadzone);

  if (dt_sec <= 0.0) {
    output.target_position_deg = target_position_deg_;
    return output;
  }

  const float velocity_deg_s = output.normalized_input * config_.max_speed_deg_s;
  target_position_deg_ += velocity_deg_s * static_cast<float>(dt_sec);
  target_position_deg_ = clamp(target_position_deg_, config_.min_position_deg, config_.max_position_deg);

  output.target_position_deg = target_position_deg_;
  output.max_velocity_deg_s = std::fabs(velocity_deg_s);
  return output;
}

void WaistJogController::reset()
{
  target_position_deg_ = clamp(config_.default_position_deg, config_.min_position_deg, config_.max_position_deg);
}

float WaistJogController::target_position_deg() const
{
  return target_position_deg_;
}

float WaistJogController::apply_deadzone(float raw_input, float deadzone)
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

float WaistJogController::clamp(float value, float min_value, float max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace bw_serial
