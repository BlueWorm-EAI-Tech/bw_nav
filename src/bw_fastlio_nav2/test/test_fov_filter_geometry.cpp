#include "bw_fastlio_nav2/fov_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace bw_fastlio_nav2 {
namespace {

FovFilterConfig default_config() {
  FovFilterConfig config;
  validate_fov_config(config);
  return config;
}

Point make_point(double x, double y, double z, float intensity = 7.0F) {
  Point point;
  point.x = static_cast<float>(x);
  point.y = static_cast<float>(y);
  point.z = static_cast<float>(z);
  point.intensity = intensity;
  return point;
}

TEST(FovFilter, DefaultWindowKeepsForwardSidesAndBelowOnly) {
  const auto config = default_config();
  EXPECT_TRUE(point_inside_fov(make_point(2.0, 0.0, 0.0), config));
  EXPECT_TRUE(point_inside_fov(make_point(0.0, 2.0, -0.01), config));
  EXPECT_TRUE(point_inside_fov(make_point(0.0, -2.0, -0.01), config));
  // Rear quarter and anything at/above the LiDAR plane are rejected.
  EXPECT_FALSE(point_inside_fov(make_point(-2.0, 0.0, -0.01), config));
  EXPECT_FALSE(point_inside_fov(make_point(1.0, 1.0, 0.2), config));
  EXPECT_FALSE(point_inside_fov(make_point(1.0, 1.0, 0.001), config));
  EXPECT_TRUE(point_inside_fov(make_point(1.0, 1.0, 0.0), config));
}

TEST(FovFilter, AzimuthLimitsAreInclusive) {
  const auto config = default_config();
  const double radius = 2.0;
  for (const double azimuth_deg : {-135.0, -134.9, 0.0, 134.9, 135.0}) {
    const double azimuth = azimuth_deg * kFovDegToRad;
    EXPECT_TRUE(point_inside_fov(
        make_point(radius * std::cos(azimuth), radius * std::sin(azimuth),
                   -0.01),
        config))
        << azimuth_deg;
  }
  for (const double azimuth_deg : {-180.0, -135.1, 135.1, 180.0}) {
    const double azimuth = azimuth_deg * kFovDegToRad;
    EXPECT_FALSE(point_inside_fov(
        make_point(radius * std::cos(azimuth), radius * std::sin(azimuth),
                   -0.01),
        config))
        << azimuth_deg;
  }
}

TEST(FovFilter, HorizontalRangeLimitsAreInclusive) {
  const auto config = default_config();
  EXPECT_FALSE(point_inside_fov(make_point(0.04, 0.0, 0.0), config));
  EXPECT_TRUE(point_inside_fov(make_point(0.05, 0.0, 0.0), config));
  EXPECT_TRUE(point_inside_fov(make_point(15.0, 0.0, -0.01), config));
  EXPECT_FALSE(point_inside_fov(make_point(15.01, 0.0, -0.01), config));
}

TEST(FovFilter, MountYawRotatesTheLimitingFrame) {
  FovFilterConfig config;
  config.mount_yaw_deg = 90.0;
  validate_fov_config(config);
  // With the limiting frame yawed by +90 deg, body -y becomes forward.
  EXPECT_TRUE(point_inside_fov(make_point(-2.0, 0.0, -0.01), config));
  EXPECT_TRUE(point_inside_fov(make_point(2.0, 0.0, -0.01), config));
  EXPECT_FALSE(point_inside_fov(make_point(0.0, 2.0, -0.01), config));
}

TEST(FovFilter, MountPitchRotatesTheLimitingFrame) {
  FovFilterConfig config;
  config.mount_pitch_deg = 90.0;
  validate_fov_config(config);
  // +x is rotated down, -x up: the same points now land on opposite sides of
  // the elevation limit.
  EXPECT_TRUE(point_inside_fov(make_point(1.0, 2.0, 0.0), config));
  EXPECT_FALSE(point_inside_fov(make_point(-1.0, 2.0, 0.0), config));
}

TEST(FovFilter, MountRollRotatesTheLimitingFrame) {
  FovFilterConfig config;
  config.mount_roll_deg = 90.0;
  validate_fov_config(config);
  // Roll about +x turns +y into +z (rejected) and -y into -z (kept).
  EXPECT_FALSE(point_inside_fov(make_point(2.0, 1.0, 0.0), config));
  EXPECT_TRUE(point_inside_fov(make_point(2.0, -1.0, 0.0), config));
}

TEST(FovFilter, ValidationRejectsInvalidBounds) {
  FovFilterConfig config;
  config.min_azimuth_deg = 135.0;
  config.max_azimuth_deg = 135.0;
  EXPECT_THROW(validate_fov_config(config), std::runtime_error);

  config = FovFilterConfig();
  config.min_azimuth_deg = -181.0;
  EXPECT_THROW(validate_fov_config(config), std::runtime_error);

  config = FovFilterConfig();
  config.max_elevation_deg = 91.0;
  EXPECT_THROW(validate_fov_config(config), std::runtime_error);

  config = FovFilterConfig();
  config.min_elevation_deg = 0.0;
  EXPECT_THROW(validate_fov_config(config), std::runtime_error);

  config = FovFilterConfig();
  config.max_range = 0.01;
  EXPECT_THROW(validate_fov_config(config), std::runtime_error);

  config = FovFilterConfig();
  config.mount_yaw_deg = std::nan("");
  EXPECT_THROW(validate_fov_config(config), std::runtime_error);
}

TEST(FovFilter, FilterCloudCountsRemovedAndPreservesContent) {
  const auto config = default_config();
  Cloud input;
  input.push_back(make_point(2.0, 0.0, 0.0, 1.0F));
  input.push_back(make_point(0.0, 2.0, -0.01, 2.0F));
  input.push_back(make_point(-2.0, 0.0, -0.01, 3.0F));
  input.push_back(make_point(1.0, 1.0, 0.5, 4.0F));
  input.push_back(make_point(0.02, 0.0, 0.0, 5.0F));
  input.width = input.size();
  input.height = 1;

  std::size_t removed = 0;
  const Cloud::Ptr output = filter_cloud_by_fov(input, config, removed);
  EXPECT_EQ(removed, 3U);
  ASSERT_EQ(output->size(), 2U);
  EXPECT_EQ(output->width, 2U);
  EXPECT_EQ(output->height, 1U);
  EXPECT_FLOAT_EQ((*output)[0].intensity, 1.0F);
  EXPECT_FLOAT_EQ((*output)[1].intensity, 2.0F);
}

}  // namespace
}  // namespace bw_fastlio_nav2
