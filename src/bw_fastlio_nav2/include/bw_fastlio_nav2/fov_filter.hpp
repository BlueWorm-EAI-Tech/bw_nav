#pragma once

#include "bw_fastlio_nav2/ground_segmentation.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>

namespace bw_fastlio_nav2 {

constexpr double kFovDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kFovRadToDeg = 180.0 / 3.14159265358979323846;

// Field-of-view gate applied on top of the ground/self filtering. The point is
// first rotated into the limiting frame by the mount roll/pitch/yaw, then
// gated in spherical coordinates: azimuth around +x, elevation off the x/y
// plane and horizontal range. The navigation default keeps every return at or
// below the LiDAR plane and inside +-135 deg of the forward axis, so the two
// costmaps never see the rear quarter or anything above the sensor.
struct FovFilterConfig {
  bool enabled{true};
  double min_azimuth_deg{-135.0};
  double max_azimuth_deg{135.0};
  double min_elevation_deg{-90.0};
  double max_elevation_deg{0.0};
  double min_range{0.05};
  double max_range{15.0};
  double mount_roll_deg{0.0};
  double mount_pitch_deg{0.0};
  double mount_yaw_deg{0.0};
  Eigen::Matrix3f rotation{Eigen::Matrix3f::Identity()};
};

inline void validate_fov_config(FovFilterConfig &config) {
  if (!std::isfinite(config.min_azimuth_deg) ||
      !std::isfinite(config.max_azimuth_deg) ||
      !std::isfinite(config.min_elevation_deg) ||
      !std::isfinite(config.max_elevation_deg) ||
      !std::isfinite(config.min_range) || !std::isfinite(config.max_range) ||
      !std::isfinite(config.mount_roll_deg) ||
      !std::isfinite(config.mount_pitch_deg) ||
      !std::isfinite(config.mount_yaw_deg)) {
    throw std::runtime_error("FOV filter parameters must be finite");
  }
  if (config.min_azimuth_deg < -180.0 || config.max_azimuth_deg > 180.0 ||
      config.min_azimuth_deg >= config.max_azimuth_deg) {
    throw std::runtime_error(
        "FOV azimuth bounds must satisfy -180 <= min < max <= 180 deg");
  }
  if (config.min_elevation_deg < -90.0 || config.max_elevation_deg > 90.0 ||
      config.min_elevation_deg >= config.max_elevation_deg) {
    throw std::runtime_error(
        "FOV elevation bounds must satisfy -90 <= min < max <= 90 deg");
  }
  if (config.min_range < 0.0 || config.max_range <= config.min_range) {
    throw std::runtime_error("FOV range bounds must satisfy 0 <= min < max");
  }
  // Intrinsic yaw-pitch-roll: yaw about +z, then pitch about +y, then roll
  // about +x, applied to the incoming point cloud.
  config.rotation =
      Eigen::AngleAxisf(
          static_cast<float>(config.mount_yaw_deg * kFovDegToRad),
          Eigen::Vector3f::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisf(
          static_cast<float>(config.mount_pitch_deg * kFovDegToRad),
          Eigen::Vector3f::UnitY())
          .toRotationMatrix() *
      Eigen::AngleAxisf(
          static_cast<float>(config.mount_roll_deg * kFovDegToRad),
          Eigen::Vector3f::UnitX())
          .toRotationMatrix();
}

inline bool point_inside_fov(const Point &point,
                             const FovFilterConfig &config) {
  const Eigen::Vector3f rotated =
      config.rotation * Eigen::Vector3f(point.x, point.y, point.z);
  const double horizontal_range = std::hypot(
      static_cast<double>(rotated.x()), static_cast<double>(rotated.y()));
  if (!std::isfinite(horizontal_range) ||
      horizontal_range < config.min_range ||
      horizontal_range > config.max_range) {
    return false;
  }
  const double elevation_deg =
      std::atan2(static_cast<double>(rotated.z()), horizontal_range) *
      kFovRadToDeg;
  if (elevation_deg < config.min_elevation_deg ||
      elevation_deg > config.max_elevation_deg) {
    return false;
  }
  const double azimuth_deg =
      std::atan2(static_cast<double>(rotated.y()),
                 static_cast<double>(rotated.x())) *
      kFovRadToDeg;
  return azimuth_deg >= config.min_azimuth_deg &&
         azimuth_deg <= config.max_azimuth_deg;
}

inline Cloud::Ptr filter_cloud_by_fov(const Cloud &input,
                                      const FovFilterConfig &config,
                                      std::size_t &removed) {
  Cloud::Ptr output(new Cloud);
  output->reserve(input.size());
  removed = 0;
  for (const auto &point : input) {
    if (point_inside_fov(point, config)) {
      output->push_back(point);
    } else {
      ++removed;
    }
  }
  output->width = output->size();
  output->height = 1;
  output->is_dense = true;
  return output;
}

}  // namespace bw_fastlio_nav2
