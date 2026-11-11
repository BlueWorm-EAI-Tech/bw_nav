#include <gtest/gtest.h>

#include "bw_serial/pelvis_jog_controller.hpp"

namespace bw_serial
{

TEST(PelvisJogControllerTest, StartsAtConfiguredDefaultHeight)
{
  PelvisJogConfig config;
  config.default_height_mm = 1000.0F;
  config.min_height_mm = 500.0F;
  config.max_height_mm = 1500.0F;
  config.max_speed_mm_s = 200.0F;
  config.input_deadzone = 0.1F;

  PelvisJogController controller(config);

  EXPECT_FLOAT_EQ(controller.target_height_mm(), 1000.0F);
}

TEST(PelvisJogControllerTest, PositiveInputIntegratesHeightAndVelocityLimit)
{
  PelvisJogConfig config;
  config.default_height_mm = 1000.0F;
  config.min_height_mm = 500.0F;
  config.max_height_mm = 1500.0F;
  config.max_speed_mm_s = 200.0F;
  config.input_deadzone = 0.0F;

  PelvisJogController controller(config);
  const PelvisJogOutput output = controller.step(0.5F, 0.1);

  EXPECT_FLOAT_EQ(output.target_height_mm, 1010.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_mm_s, 100.0F);
  EXPECT_FLOAT_EQ(controller.target_height_mm(), 1010.0F);
}

TEST(PelvisJogControllerTest, NegativeInputClampsAtMinimumHeight)
{
  PelvisJogConfig config;
  config.default_height_mm = 510.0F;
  config.min_height_mm = 500.0F;
  config.max_height_mm = 1500.0F;
  config.max_speed_mm_s = 200.0F;
  config.input_deadzone = 0.0F;

  PelvisJogController controller(config);
  const PelvisJogOutput output = controller.step(-1.0F, 0.2);

  EXPECT_FLOAT_EQ(output.target_height_mm, 500.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_mm_s, 200.0F);
  EXPECT_FLOAT_EQ(controller.target_height_mm(), 500.0F);
}

TEST(PelvisJogControllerTest, DeadzoneInputStopsMotionAndKeepsTarget)
{
  PelvisJogConfig config;
  config.default_height_mm = 1000.0F;
  config.min_height_mm = 500.0F;
  config.max_height_mm = 1500.0F;
  config.max_speed_mm_s = 200.0F;
  config.input_deadzone = 0.1F;

  PelvisJogController controller(config);
  const PelvisJogOutput output = controller.step(0.05F, 0.1);

  EXPECT_FLOAT_EQ(output.target_height_mm, 1000.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_mm_s, 0.0F);
  EXPECT_FLOAT_EQ(controller.target_height_mm(), 1000.0F);
}

TEST(PelvisJogControllerTest, StartupInitializationCommandsDefaultHeightUntilFeedbackReachesTarget)
{
  PelvisJogConfig config;
  config.default_height_mm = 900.0F;
  config.min_height_mm = 600.0F;
  config.max_height_mm = 1000.0F;
  config.max_speed_mm_s = 200.0F;
  config.startup_init_enabled = true;
  config.startup_init_tolerance_mm = 5.0F;
  config.startup_init_timeout_sec = 10.0;

  PelvisJogController controller(config);

  const PelvisJogOutput moving_output = controller.startup_initialization_step(true, 1200.0F, 0.01);
  EXPECT_TRUE(moving_output.startup_initializing);
  EXPECT_FLOAT_EQ(moving_output.target_height_mm, 900.0F);
  EXPECT_FLOAT_EQ(moving_output.max_velocity_mm_s, 200.0F);
  EXPECT_TRUE(controller.startup_initialization_active());

  const PelvisJogOutput reached_output = controller.startup_initialization_step(true, 903.0F, 0.01);
  EXPECT_TRUE(reached_output.startup_initialization_completed);
  EXPECT_FALSE(reached_output.startup_initializing);
  EXPECT_FLOAT_EQ(reached_output.target_height_mm, 900.0F);
  EXPECT_FLOAT_EQ(reached_output.max_velocity_mm_s, 0.0F);
  EXPECT_FALSE(controller.startup_initialization_active());
}

TEST(PelvisJogControllerTest, SyncTargetHeightUsesFeedbackBeforeJogIntegration)
{
  PelvisJogConfig config;
  config.default_height_mm = 1350.0F;
  config.min_height_mm = 900.0F;
  config.max_height_mm = 1500.0F;
  config.max_speed_mm_s = 200.0F;
  config.input_deadzone = 0.0F;

  PelvisJogController controller(config);
  controller.sync_target_height_mm(1200.0F);

  const PelvisJogOutput output = controller.step(0.5F, 0.1);

  EXPECT_FLOAT_EQ(output.target_height_mm, 1210.0F);
  EXPECT_FLOAT_EQ(output.max_velocity_mm_s, 100.0F);
  EXPECT_FLOAT_EQ(controller.target_height_mm(), 1210.0F);
}

TEST(PelvisJogControllerTest, SyncTargetHeightClampsFeedbackToConfiguredRange)
{
  PelvisJogConfig config;
  config.default_height_mm = 1350.0F;
  config.min_height_mm = 900.0F;
  config.max_height_mm = 1500.0F;

  PelvisJogController controller(config);
  controller.sync_target_height_mm(2000.0F);

  EXPECT_FLOAT_EQ(controller.target_height_mm(), 1500.0F);
}

TEST(PelvisJogControllerTest, StandardUsesLowerControllerRelativeTravel)
{
  const PelvisJogConfig config = default_pelvis_jog_config_for_robot_version("standard");

  EXPECT_FLOAT_EQ(config.default_height_mm, -100.0F);
  EXPECT_FLOAT_EQ(config.min_height_mm, -500.0F);
  EXPECT_FLOAT_EQ(config.max_height_mm, 0.0F);

  PelvisJogController controller(config);
  controller.sync_target_height_mm(-100.0F);

  EXPECT_FLOAT_EQ(controller.step(1.0F, 1.0).target_height_mm, 0.0F);
  EXPECT_FLOAT_EQ(controller.step(-1.0F, 3.0).target_height_mm, -500.0F);
}

}  // namespace bw_serial
