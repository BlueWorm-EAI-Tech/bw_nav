#include <gtest/gtest.h>

#include "bw_serial/waist_jog_controller.hpp"

namespace bw_serial
{

TEST(WaistJogControllerTest, StartsAtConfiguredDefaultPosition)
{
  WaistJogConfig config;
  config.default_position_deg = 5.0F;
  config.min_position_deg = -30.0F;
  config.max_position_deg = 30.0F;
  config.max_speed_deg_s = 20.0F;
  config.input_deadzone = 0.1F;

  WaistJogController controller(config);

  EXPECT_FLOAT_EQ(controller.target_position_deg(), 5.0F);
}

TEST(WaistJogControllerTest, PositiveInputIntegratesPositionAndVelocity)
{
  WaistJogConfig config;
  config.default_position_deg = 0.0F;
  config.min_position_deg = -30.0F;
  config.max_position_deg = 30.0F;
  config.max_speed_deg_s = 20.0F;
  config.input_deadzone = 0.0F;

  WaistJogController controller(config);
  const WaistJogOutput output = controller.step(1.0F, 0.5);

  EXPECT_FLOAT_EQ(output.target_position_deg, 10.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_deg_s, 20.0F);
}

TEST(WaistJogControllerTest, DeadzoneInputStopsMotion)
{
  WaistJogConfig config;
  config.default_position_deg = 3.0F;
  config.min_position_deg = -30.0F;
  config.max_position_deg = 30.0F;
  config.max_speed_deg_s = 20.0F;
  config.input_deadzone = 0.1F;

  WaistJogController controller(config);
  const WaistJogOutput output = controller.step(0.05F, 0.5);

  EXPECT_FLOAT_EQ(output.target_position_deg, 3.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_deg_s, 0.0F);
}

TEST(WaistJogControllerTest, ClampStopsAtConfiguredLimit)
{
  WaistJogConfig config;
  config.default_position_deg = 25.0F;
  config.min_position_deg = -30.0F;
  config.max_position_deg = 30.0F;
  config.max_speed_deg_s = 20.0F;
  config.input_deadzone = 0.0F;

  WaistJogController controller(config);
  const WaistJogOutput output = controller.step(1.0F, 1.0);

  EXPECT_FLOAT_EQ(output.target_position_deg, 30.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_deg_s, 20.0F);
}

}  // namespace bw_serial
