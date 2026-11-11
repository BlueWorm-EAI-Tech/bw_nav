#include <gtest/gtest.h>

#include "bw_serial/v3_waist_safety.hpp"

namespace bw_serial
{

TEST(V3WaistSafetyDefaultsTest, AppliesConfiguredSafeValues)
{
  MantisCommandV3_t command{};
  command.pelvis_height = 1234.0F;
  command.pelvis_velocity = 88.0F;
  command.waist_pos = -42.0F;
  command.waist_max_vel = 9.0F;

  V3WaistSafetyDefaults defaults;
  defaults.waist_pos_deg = 0.0F;
  defaults.waist_max_velocity_deg_s = 0.0F;

  apply_v3_waist_safety_defaults(command, defaults);

  EXPECT_FLOAT_EQ(command.pelvis_height, 1234.0F);
  EXPECT_FLOAT_EQ(command.pelvis_velocity, 88.0F);
  EXPECT_FLOAT_EQ(command.waist_pos, defaults.waist_pos_deg);
  EXPECT_FLOAT_EQ(command.waist_max_vel, defaults.waist_max_velocity_deg_s);
}

}  // namespace bw_serial
