#include "bw_fastlio_nav2/freedom_mask.hpp"

#include "freedom/freedom.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace bw_fastlio_nav2
{
namespace
{
constexpr double kLeaf = 0.05;
constexpr double kBoundaryGuard = 1e-5;
constexpr double kCoordinateLimit = 1e6;
constexpr double kSensorMaxRange = 40.0;
constexpr double kSensorMinZ = -5.0;
constexpr double kSensorMaxZ = 5.0;
// A 5 cm grid at ±1e6 m uses signed indices of magnitude at most 20 million.
// The extra margin covers neighbor cells and the robocentric map/raycast bounds.
static_assert(std::numeric_limits<int>::max() >= 21'000'000,
  "FreeDOM requires signed indices large enough for its 5 cm grid");

using Cell = freedom::Index;
using CellSet = std::unordered_set<Cell, freedom::IndexHash>;
using FirstHits = std::unordered_map<Cell, Cell, freedom::IndexHash>;

freedom::FreeDOM::Config replay_config()
{
  // The validated offline indoor profile; keep the replay serial and deterministic.
  freedom::FreeDOM::Config config{};
  config.sensor_min_range = 0.0;
  config.sensor_max_range = kSensorMaxRange;
  config.sensor_min_z = kSensorMinZ;
  config.sensor_max_z = kSensorMaxZ;
  config.sub_voxel_size = kLeaf;
  config.voxel_depth = 2;
  config.block_depth = 5;
  config.enable_local_map = false;
  config.raycast_max_range = 10.0;
  config.raycast_min_z = -3.0;
  config.raycast_max_z = 3.0;
  config.counts_to_free = 3;
  config.counts_to_revert = 20;
  config.conservative_connectivity = 26;
  config.aggressive_connectivity = 124;
  config.enable_raycast_enhancement = false;
  config.num_threads = 1;
  return config;
}

bool finite(const pcl::PointXYZI &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

bool grid_safe(const Eigen::Vector3d &point)
{
  return point.allFinite() && point.cwiseAbs().maxCoeff() <= kCoordinateLimit;
}

Cell cell_for(const Eigen::Vector3d &point)
{
  // compare_masks.py uses division in double on the stored PCD float coordinates.
  return Cell(
    static_cast<int>(std::floor(point.x() / kLeaf)),
    static_cast<int>(std::floor(point.y() / kLeaf)),
    static_cast<int>(std::floor(point.z() / kLeaf)));
}

bool near_face(const Eigen::Vector3d &point)
{
  for (int axis = 0; axis < 3; ++axis) {
    const double coord = point[axis];
    if (std::abs(coord - std::round(coord / kLeaf) * kLeaf) <= kBoundaryGuard) {
      return true;
    }
  }
  return false;
}

void subtract_static_cells(const freedom::MRMap &map, CellSet &cells)
{
  const auto block_width = map.getVoxel2blockMultiples();
  const auto voxel_width = map.getSubvoxel2voxelMultiples();
  const auto voxels_per_block = map.getVoxel2blockMultiplesCubed();
  const auto cells_per_voxel = map.getSubvoxel2voxelMultiplesCubed();
  const Eigen::Vector3d half_leaf = Eigen::Vector3d::Constant(kLeaf * 0.5);
  for (const auto &[block_key, block] : map.get_static_blocks()) {
    const Eigen::Vector3d block_bias = map.getBlockSize() * block_key.cast<double>();
    Cell voxel_key = Cell::Zero();
    for (unsigned int i = 0; i < voxels_per_block;
      ++i, freedom::incrementIdx(voxel_key, block_width))
    {
      if (!block.is_static_voxel_allocated(i)) {
        continue;
      }
      const auto &voxel = block.getStaticVoxel(i);
      const Eigen::Vector3d voxel_bias = map.getVoxelSize() * voxel_key.cast<double>();
      Cell sub_key = Cell::Zero();
      for (unsigned int j = 0; j < cells_per_voxel;
        ++j, freedom::incrementIdx(sub_key, voxel_width))
      {
        if (voxel.scan_in_subvoxel[j] == freedom::StaticVoxel::NOT_A_SCAN ||
          voxel.dynamic_level[j] > freedom::DynamicLevel::STATIC)
        {
          continue;
        }
        // Match the static voxel centers used by the verified comparison, without PCD I/O.
        const Eigen::Vector3d center =
          block_bias + voxel_bias + kLeaf * sub_key.cast<double>() + half_leaf;
        const Eigen::Vector3f stored_center = center.cast<float>();
        cells.erase(cell_for(stored_center.cast<double>()));
        // At very large offsets the stored float can round into a neighbor. Keep
        // the actual static cell as well so precision loss never deletes that cell.
        cells.erase(cell_for(center));
      }
    }
  }
}
}  // namespace

struct FreedomMask::Impl
{
  explicit Impl(const Eigen::Isometry3d &extrinsic)
  : body_from_lidar(extrinsic), lidar_from_body(extrinsic.inverse()),
    core(std::make_unique<freedom::FreeDOM>())
  {
    core->set_params(replay_config());
  }

  const Eigen::Isometry3d body_from_lidar;
  const Eigen::Isometry3d lidar_from_body;
  std::unique_ptr<freedom::FreeDOM> core;
  FirstHits first_hits;
  CellSet removed_cells;
};

FreedomMask::FreedomMask(const Eigen::Isometry3d &body_from_lidar)
{
  if (!body_from_lidar.matrix().allFinite()) {
    throw std::invalid_argument("FreeDOM extrinsic must be finite");
  }
  impl_ = std::make_unique<Impl>(body_from_lidar);
}

FreedomMask::~FreedomMask() = default;

void FreedomMask::integrate(
  const pcl::PointCloud<pcl::PointXYZI> &cloud,
  const Eigen::Isometry3d &world_from_body)
{
  if (!impl_->core) {
    throw std::logic_error("cannot integrate after FreeDOM finalize");
  }
  if (!world_from_body.matrix().allFinite()) {
    return;
  }
  const Eigen::Isometry3d world_from_lidar = world_from_body * impl_->body_from_lidar;
  const Eigen::Vector3d origin = world_from_lidar.translation();
  if (!world_from_lidar.matrix().allFinite() || !grid_safe(origin)) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> lidar_cloud;
  lidar_cloud.reserve(cloud.size());
  for (const auto &point : cloud) {
    if (!finite(point)) {
      continue;
    }
    const Eigen::Vector3d sensor =
      impl_->lidar_from_body * Eigen::Vector3d(point.x, point.y, point.z);
    if (!sensor.allFinite() ||
      sensor.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max())
    {
      continue;
    }
    // The archived body-frame samples took this float round trip in the baseline runner.
    const Eigen::Vector3f stored_sensor = sensor.cast<float>();
    const Eigen::Vector3d world = world_from_lidar * stored_sensor.cast<double>();
    if (!grid_safe(world)) {
      continue;
    }
    // Let the core apply its own range/absolute-z tests; the verified first-hit
    // baseline separately tests relative z and uses the first sample in scan order.
    lidar_cloud.emplace_back(stored_sensor.x(), stored_sensor.y(), stored_sensor.z());
    const Eigen::Vector3d delta = world - origin;
    if (delta.squaredNorm() > kSensorMaxRange * kSensorMaxRange ||
      delta.z() < kSensorMinZ || delta.z() > kSensorMaxZ)
    {
      continue;
    }
    const Eigen::Vector3d grid = world * (1.0 / kLeaf);
    const Cell first_key(
      static_cast<int>(std::floor(grid.x())), static_cast<int>(std::floor(grid.y())),
      static_cast<int>(std::floor(grid.z())));
    const Eigen::Vector3f stored_world = world.cast<float>();
    impl_->first_hits.try_emplace(first_key, cell_for(stored_world.cast<double>()));
  }
  if (!lidar_cloud.empty()) {
    impl_->core->pointcloud_integrate(lidar_cloud, world_from_lidar);
  }
}

void FreedomMask::finalize()
{
  if (!impl_->core) {
    return;
  }
  CellSet removed_cells;
  removed_cells.reserve(impl_->first_hits.size());
  for (const auto &hit : impl_->first_hits) {
    removed_cells.insert(hit.second);
  }
  subtract_static_cells(impl_->core->static_map(), removed_cells);
  impl_->removed_cells.swap(removed_cells);
  FirstHits().swap(impl_->first_hits);
  impl_->core.reset();
}

bool FreedomMask::removes(const pcl::PointXYZI &point) const
{
  if (!finite(point)) {
    return true;
  }
  const Eigen::Vector3d world(point.x, point.y, point.z);
  return !impl_->core && grid_safe(world) && !near_face(world) &&
         impl_->removed_cells.find(cell_for(world)) != impl_->removed_cells.end();
}
}  // namespace bw_fastlio_nav2
