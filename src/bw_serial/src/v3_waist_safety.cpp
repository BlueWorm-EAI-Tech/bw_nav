#include "bw_serial/v3_waist_safety.hpp"

namespace bw_serial
{

void apply_v3_waist_safety_defaults(
  MantisCommandV3_t & command,
  const V3WaistSafetyDefaults & defaults)
{
  command.waist_pos = defaults.waist_pos_deg;
  command.waist_max_vel = defaults.waist_max_velocity_deg_s;
}

}  // namespace bw_serial
