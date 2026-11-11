#pragma once

#include "bw_fastlio_nav2/ground_segmentation.hpp"

#include <patchwork/patchworkpp.h>

#include <Eigen/Core>

#include <cmath>
#include <ctime>
#include <memory>
#include <stdexcept>

namespace bw_fastlio_nav2 {

struct PatchworkSegmentationConfig {
  double voxel_leaf_size = 0.05;
  double sensor_height = 0.35;
  int num_iter = 3;
  int num_lpr = 20;
  int num_min_points = 10;
  double seed_threshold = 0.12;
  double distance_threshold = 0.08;
  double vertical_seed_threshold = 0.25;
  double vertical_distance_threshold = 0.10;
  double min_range = 0.05;
  double max_range = 15.0;
  double uprightness_threshold = 0.866;
  bool enable_reflected_noise_removal = false;
  bool enable_vertical_plane_fitting = true;
  bool enable_temporal_ground_revert = true;
  double obstacle_min_height = 0.10;
  double obstacle_max_height = 1.80;
  double outlier_radius = 0.20;
  int outlier_min_neighbors = 2;
};

struct PatchworkSegmentationResult {
  Cloud::Ptr filtered{new Cloud};
  Cloud::Ptr ground{new Cloud};
  Cloud::Ptr obstacles{new Cloud};
  std::size_t range_filtered_points = 0;
  double algorithm_time_ms = 0.0;
};

inline void validate_patchwork_config(
    const PatchworkSegmentationConfig &config) {
  if (config.voxel_leaf_size <= 0.0 || config.sensor_height <= 0.0 ||
      config.num_iter < 1 || config.num_lpr < 1 ||
      config.num_min_points < 1 || config.seed_threshold <= 0.0 ||
      config.distance_threshold <= 0.0 ||
      config.vertical_seed_threshold <= 0.0 ||
      config.vertical_distance_threshold <= 0.0 || config.min_range < 0.0 ||
      config.max_range <= config.min_range ||
      config.uprightness_threshold <= 0.0 ||
      config.uprightness_threshold >= 1.0 || config.obstacle_min_height < 0.0 ||
      config.obstacle_max_height <= config.obstacle_min_height ||
      config.outlier_radius < 0.0 || config.outlier_min_neighbors < 0) {
    throw std::runtime_error("invalid Patchwork++ segmentation configuration");
  }
}

inline patchwork::Params make_patchwork_params(
    const PatchworkSegmentationConfig &config) {
  validate_patchwork_config(config);
  patchwork::Params params;
  params.verbose = false;
  params.sensor_height = config.sensor_height;
  params.num_iter = config.num_iter;
  params.num_lpr = config.num_lpr;
  params.num_min_pts = config.num_min_points;
  params.th_seeds = config.seed_threshold;
  params.th_dist = config.distance_threshold;
  params.th_seeds_v = config.vertical_seed_threshold;
  params.th_dist_v = config.vertical_distance_threshold;
  params.min_range = config.min_range;
  params.max_range = config.max_range;
  params.uprightness_thr = config.uprightness_threshold;
  params.enable_RNR = config.enable_reflected_noise_removal;
  params.enable_RVPF = config.enable_vertical_plane_fitting;
  params.enable_TGR = config.enable_temporal_ground_revert;
  return params;
}

inline PatchworkSegmentationResult segment_ground_patchwork(
    const Cloud &input, patchwork::PatchWorkpp &algorithm,
    const PatchworkSegmentationConfig &config) {
  validate_patchwork_config(config);
  PatchworkSegmentationResult result;
  const auto voxelized = voxel_filter(input, config.voxel_leaf_size);
  result.filtered->reserve(voxelized->size());
  const double min_range_squared = config.min_range * config.min_range;
  const double max_range_squared = config.max_range * config.max_range;
  for (const auto &point : *voxelized) {
    const double range_squared =
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y;
    if (range_squared >= min_range_squared &&
        range_squared <= max_range_squared) {
      result.filtered->push_back(point);
    } else {
      ++result.range_filtered_points;
    }
  }
  result.filtered->width = result.filtered->size();
  result.filtered->height = 1;
  result.filtered->is_dense = true;
  if (result.filtered->size() < 100) {
    throw std::runtime_error("too few points remain inside Patchwork++ range");
  }

  Eigen::MatrixXf matrix(result.filtered->size(), 4);
  for (std::size_t i = 0; i < result.filtered->size(); ++i) {
    const auto &point = (*result.filtered)[i];
    matrix.row(static_cast<Eigen::Index>(i)) <<
        point.x, point.y, point.z, point.intensity;
  }
  algorithm.estimateGround(std::move(matrix));
  result.algorithm_time_ms =
      algorithm.getTimeTaken() * 1000.0 / static_cast<double>(CLOCKS_PER_SEC);

  const auto ground_indices = algorithm.getGroundIndices();
  result.ground->reserve(static_cast<std::size_t>(ground_indices.size()));
  for (Eigen::Index i = 0; i < ground_indices.size(); ++i) {
    const int index = ground_indices[i];
    if (index < 0 || static_cast<std::size_t>(index) >= result.filtered->size()) {
      throw std::runtime_error("Patchwork++ returned an invalid ground index");
    }
    result.ground->push_back((*result.filtered)[static_cast<std::size_t>(index)]);
  }
  result.ground->width = result.ground->size();
  result.ground->height = 1;
  result.ground->is_dense = true;

  const auto obstacle_indices = algorithm.getNongroundIndices();
  result.obstacles->reserve(static_cast<std::size_t>(obstacle_indices.size()));
  for (Eigen::Index i = 0; i < obstacle_indices.size(); ++i) {
    const int index = obstacle_indices[i];
    if (index < 0 || static_cast<std::size_t>(index) >= result.filtered->size()) {
      throw std::runtime_error("Patchwork++ returned an invalid obstacle index");
    }
    const auto &point = (*result.filtered)[static_cast<std::size_t>(index)];
    const double height_above_ground =
        static_cast<double>(point.z) + config.sensor_height;
    if (height_above_ground >= config.obstacle_min_height &&
        height_above_ground <= config.obstacle_max_height) {
      result.obstacles->push_back(point);
    }
  }
  result.obstacles->width = result.obstacles->size();
  result.obstacles->height = 1;
  result.obstacles->is_dense = true;
  result.obstacles = radius_filter(
      result.obstacles, config.outlier_radius, config.outlier_min_neighbors);
  return result;
}

}  // namespace bw_fastlio_nav2
