#pragma once

#include "bw_fastlio_nav2/ground_segmentation.hpp"

#include <CSF.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace bw_fastlio_nav2 {

struct CsfSegmentationConfig {
  double voxel_leaf_size = 0.08;
  double cloth_resolution = 0.20;
  int rigidness = 3;
  int iterations = 500;
  double classification_threshold = 0.12;
  double time_step = 0.65;
  bool slope_smoothing = false;
  double obstacle_min_height = 0.25;
  double obstacle_max_height = 1.80;
  double ground_reference_radius = 0.75;
  int ground_reference_neighbors = 5;
  double outlier_radius = 0.20;
  int outlier_min_neighbors = 2;
};

struct GroundSegmentationResult {
  Cloud::Ptr filtered{new Cloud};
  Cloud::Ptr ground{new Cloud};
  Cloud::Ptr obstacles{new Cloud};
  std::size_t off_ground_points = 0;
  std::size_t no_ground_reference_points = 0;
};

inline void validate_csf_config(const CsfSegmentationConfig &config) {
  if (config.voxel_leaf_size <= 0.0 || config.cloth_resolution <= 0.0 ||
      config.rigidness < 1 || config.rigidness > 3 || config.iterations < 10 ||
      config.classification_threshold <= 0.0 || config.time_step <= 0.0 ||
      config.obstacle_min_height < 0.0 ||
      config.obstacle_max_height <= config.obstacle_min_height ||
      config.ground_reference_radius <= 0.0 ||
      config.ground_reference_neighbors < 1 ||
      config.ground_reference_neighbors > 50 || config.outlier_radius < 0.0 ||
      config.outlier_min_neighbors < 0) {
    throw std::runtime_error("invalid CSF segmentation configuration");
  }
}

inline GroundSegmentationResult segment_ground_csf(
    const Cloud &input, const CsfSegmentationConfig &config) {
  validate_csf_config(config);
  GroundSegmentationResult result;
  result.filtered = voxel_filter(input, config.voxel_leaf_size);

  std::vector<csf::Point> csf_points;
  csf_points.reserve(result.filtered->size());
  for (const auto &point : *result.filtered) {
    csf_points.emplace_back(point.x, point.y, point.z);
  }

  CSF filter;
  filter.params.cloth_resolution = config.cloth_resolution;
  filter.params.rigidness = config.rigidness;
  filter.params.interations = config.iterations;
  filter.params.class_threshold = config.classification_threshold;
  filter.params.time_step = config.time_step;
  filter.params.bSloopSmooth = config.slope_smoothing;
  filter.setPointCloud(csf_points);

  std::vector<int> ground_indices;
  std::vector<int> off_ground_indices;
  filter.do_filtering(ground_indices, off_ground_indices, false);
  if (ground_indices.empty()) {
    throw std::runtime_error("CSF did not classify any ground points");
  }

  result.ground->reserve(ground_indices.size());
  pcl::PointCloud<pcl::PointXY>::Ptr ground_xy(
      new pcl::PointCloud<pcl::PointXY>);
  ground_xy->reserve(ground_indices.size());
  std::vector<float> ground_z;
  ground_z.reserve(ground_indices.size());
  for (const int index : ground_indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= result.filtered->size()) {
      throw std::runtime_error("CSF returned an invalid ground index");
    }
    const auto &point = (*result.filtered)[static_cast<std::size_t>(index)];
    result.ground->push_back(point);
    pcl::PointXY xy;
    xy.x = point.x;
    xy.y = point.y;
    ground_xy->push_back(xy);
    ground_z.push_back(point.z);
  }
  result.ground->width = result.ground->size();
  result.ground->height = 1;
  result.ground->is_dense = true;
  ground_xy->width = ground_xy->size();
  ground_xy->height = 1;
  ground_xy->is_dense = true;

  pcl::KdTreeFLANN<pcl::PointXY> ground_tree;
  ground_tree.setInputCloud(ground_xy);
  std::vector<int> neighbor_index(
      static_cast<std::size_t>(config.ground_reference_neighbors));
  std::vector<float> neighbor_distance(
      static_cast<std::size_t>(config.ground_reference_neighbors));
  result.off_ground_points = off_ground_indices.size();
  result.obstacles->reserve(off_ground_indices.size());
  const double reference_radius_squared =
      config.ground_reference_radius * config.ground_reference_radius;
  for (const int index : off_ground_indices) {
    if (index < 0 || static_cast<std::size_t>(index) >= result.filtered->size()) {
      throw std::runtime_error("CSF returned an invalid off-ground index");
    }
    const auto &point = (*result.filtered)[static_cast<std::size_t>(index)];
    pcl::PointXY query;
    query.x = point.x;
    query.y = point.y;
    const int neighbors_found = ground_tree.nearestKSearch(
        query, config.ground_reference_neighbors, neighbor_index,
        neighbor_distance);
    std::vector<float> local_ground_z;
    local_ground_z.reserve(static_cast<std::size_t>(neighbors_found));
    for (int neighbor = 0; neighbor < neighbors_found; ++neighbor) {
      if (neighbor_distance[static_cast<std::size_t>(neighbor)] <=
          reference_radius_squared) {
        local_ground_z.push_back(
            ground_z[static_cast<std::size_t>(
                neighbor_index[static_cast<std::size_t>(neighbor)])]);
      }
    }
    if (local_ground_z.empty()) {
      ++result.no_ground_reference_points;
      continue;
    }
    const auto middle = local_ground_z.begin() + local_ground_z.size() / 2;
    std::nth_element(local_ground_z.begin(), middle, local_ground_z.end());
    const double local_ground_height = *middle;
    const double height =
        static_cast<double>(point.z) - local_ground_height;
    if (height >= config.obstacle_min_height &&
        height <= config.obstacle_max_height) {
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
