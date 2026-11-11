// Rebuilds a 3D consistency map from PGO keyframe scans.
//
// Pipeline:
//   pass A  transform every scan into the world frame, clip by height, and
//           accumulate per-coarse-cell point covariance to estimate surface
//           normals (C6 height clip + normal model for C7)
//   pass B  transform the scans again, weight each point by its incidence angle
//           against the local normal, and accumulate weighted voxel centroids
//   merge   drop voxels seen by too few scans, emit the weighted centroid
//
// Both passes are multi-threaded (C4): scans are distributed dynamically and
// each worker owns a private accumulator that is merged afterwards.

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

struct Config {
  std::filesystem::path scans_directory;
  std::filesystem::path poses_path;
  std::filesystem::path output_path;

  double voxel_size = 0.05;
  double max_range = 60.0;
  int min_observations = 2;

  // C6: height clip in the world frame of the PGO poses.
  double z_min = -0.60;
  double z_max = 2.50;

  // C7: normals are estimated on a coarser grid than the output voxels.
  double normal_voxel_size = 0.30;
  int min_normal_points = 8;
  double min_planarity = 0.55;

  // Incidence weighting.  cos(incidence) == 1 means the surface is seen head on.
  double max_incidence_deg = 88.0;
  double full_weight_deg = 60.0;
  double min_weight = 0.10;

  int threads = 0;  // 0 -> hardware concurrency
};

struct VoxelKey {
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

// Accumulated covariance of every point that falls into one coarse cell.
struct NormalCell {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
  std::size_t count = 0;
};

struct VoxelStats {
  Eigen::Vector3d weighted_sum = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;
  double intensity_weighted_sum = 0.0;
  std::size_t point_count = 0;
  int observation_count = 0;
  int last_scan_index = -1;
};

struct Counters {
  std::size_t scans = 0;
  std::size_t points = 0;
  std::size_t dropped_non_finite = 0;
  std::size_t dropped_range = 0;
  std::size_t dropped_height = 0;
  std::size_t dropped_incidence = 0;
  std::size_t no_normal = 0;
};

std::string require_value(int argc, char **argv, int &index) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  return argv[++index];
}

void print_usage() {
  std::cout
      << "Usage: rebuild_pgo_consistent_map --scans-directory Scans "
         "--poses optimized_poses_batch.txt --output map_consistent.pcd\n"
      << "\nVoxel / observation filter:\n"
      << "  [--voxel-size 0.05]         output voxel edge, metres\n"
      << "  [--min-observations 2]      minimum number of keyframes per voxel\n"
      << "  [--max-range 60]            drop points beyond this sensor range\n"
      << "\nHeight clip (world frame of the PGO poses):\n"
      << "  [--z-min -0.60] [--z-max 2.50]\n"
      << "\nIncidence weighting (grazing observations lose weight):\n"
      << "  [--normal-voxel-size 0.30]  cell size for surface normal estimation\n"
      << "  [--min-normal-points 8]     points needed before a normal is trusted\n"
      << "  [--min-planarity 0.55]      (l1-l0)/l2 needed to trust a normal\n"
      << "  [--max-incidence-deg 88]    hard reject beyond this incidence angle\n"
      << "  [--full-weight-deg 60]      full weight at or below this angle\n"
      << "  [--min-weight 0.10]         floor applied to the incidence weight\n"
      << "\nExecution:\n"
      << "  [--threads 0]               0 uses all hardware threads\n";
}

Config parse_args(int argc, char **argv) {
  Config config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--scans-directory") {
      config.scans_directory = require_value(argc, argv, i);
    } else if (arg == "--poses") {
      config.poses_path = require_value(argc, argv, i);
    } else if (arg == "--output") {
      config.output_path = require_value(argc, argv, i);
    } else if (arg == "--voxel-size") {
      config.voxel_size = std::stod(require_value(argc, argv, i));
    } else if (arg == "--max-range") {
      config.max_range = std::stod(require_value(argc, argv, i));
    } else if (arg == "--min-observations") {
      config.min_observations = std::stoi(require_value(argc, argv, i));
    } else if (arg == "--z-min") {
      config.z_min = std::stod(require_value(argc, argv, i));
    } else if (arg == "--z-max") {
      config.z_max = std::stod(require_value(argc, argv, i));
    } else if (arg == "--normal-voxel-size") {
      config.normal_voxel_size = std::stod(require_value(argc, argv, i));
    } else if (arg == "--min-normal-points") {
      config.min_normal_points = std::stoi(require_value(argc, argv, i));
    } else if (arg == "--min-planarity") {
      config.min_planarity = std::stod(require_value(argc, argv, i));
    } else if (arg == "--max-incidence-deg") {
      config.max_incidence_deg = std::stod(require_value(argc, argv, i));
    } else if (arg == "--full-weight-deg") {
      config.full_weight_deg = std::stod(require_value(argc, argv, i));
    } else if (arg == "--min-weight") {
      config.min_weight = std::stod(require_value(argc, argv, i));
    } else if (arg == "--threads") {
      config.threads = std::stoi(require_value(argc, argv, i));
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (config.scans_directory.empty() || config.poses_path.empty() ||
      config.output_path.empty()) {
    throw std::runtime_error(
        "--scans-directory, --poses and --output are required");
  }
  if (config.voxel_size <= 0.0 || config.max_range <= 0.0 ||
      config.min_observations < 1) {
    throw std::runtime_error("invalid filter configuration");
  }
  if (!(config.z_min < config.z_max)) {
    throw std::runtime_error("--z-min must be smaller than --z-max");
  }
  if (config.normal_voxel_size < config.voxel_size) {
    throw std::runtime_error("--normal-voxel-size must be >= --voxel-size");
  }
  if (config.min_normal_points < 3) {
    throw std::runtime_error("--min-normal-points must be >= 3");
  }
  if (!(config.min_planarity > 0.0 && config.min_planarity < 1.0)) {
    throw std::runtime_error("--min-planarity must be in (0, 1)");
  }
  if (!(config.max_incidence_deg > 0.0 && config.max_incidence_deg <= 90.0)) {
    throw std::runtime_error("--max-incidence-deg must be in (0, 90]");
  }
  if (!(config.full_weight_deg >= 0.0 &&
        config.full_weight_deg <= config.max_incidence_deg)) {
    throw std::runtime_error(
        "--full-weight-deg must be in [0, --max-incidence-deg]");
  }
  if (!(config.min_weight >= 0.0 && config.min_weight <= 1.0)) {
    throw std::runtime_error("--min-weight must be in [0, 1]");
  }
  return config;
}

std::vector<Eigen::Matrix4f> load_poses(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open pose file: " + path.string());

  std::vector<Eigen::Matrix4f> poses;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    std::istringstream stream(line);
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 4; ++column) {
        if (!(stream >> pose(row, column))) {
          throw std::runtime_error("invalid 3x4 pose line in " + path.string());
        }
      }
    }
    poses.push_back(pose);
  }
  if (poses.empty()) throw std::runtime_error("pose file is empty: " + path.string());
  return poses;
}

std::filesystem::path scan_path(
    const std::filesystem::path &directory, std::size_t index) {
  std::ostringstream name;
  name.width(6);
  name.fill('0');
  name << index << ".pcd";
  return directory / name.str();
}

VoxelKey make_key(const Eigen::Vector3d &point, double leaf) {
  return VoxelKey{static_cast<int>(std::floor(point.x() / leaf)),
                  static_cast<int>(std::floor(point.y() / leaf)),
                  static_cast<int>(std::floor(point.z() / leaf))};
}

int resolve_threads(const Config &config, std::size_t scan_count) {
  int requested = config.threads;
  if (requested <= 0) {
    requested = static_cast<int>(std::thread::hardware_concurrency());
  }
  if (requested <= 0) requested = 1;
  const int upper = static_cast<int>(std::max<std::size_t>(scan_count, 1));
  return std::min(requested, upper);
}

// Distributes scans over a fixed worker pool.  fn(scan_index, worker_index) is
// called concurrently; the first exception aborts the pool and is rethrown.
template <typename Fn>
void parallel_over_scans(std::size_t scan_count, std::size_t worker_count, Fn &&fn) {
  const std::size_t workers = std::max<std::size_t>(1, worker_count);
  std::atomic<std::size_t> next{0};
  std::atomic<bool> abort{false};
  std::mutex error_mutex;
  std::exception_ptr error;
  std::vector<std::thread> pool;
  pool.reserve(workers);
  for (std::size_t worker = 0; worker < workers; ++worker) {
    pool.emplace_back([&, worker]() {
      try {
        while (!abort.load(std::memory_order_relaxed)) {
          const std::size_t index = next.fetch_add(1);
          if (index >= scan_count) break;
          fn(index, worker);
        }
      } catch (...) {
        abort.store(true);
        std::lock_guard<std::mutex> lock(error_mutex);
        if (!error) error = std::current_exception();
      }
    });
  }
  for (auto &worker : pool) worker.join();
  if (error) std::rethrow_exception(error);
}

enum class Reject { kNone, kNonFinite, kRange, kHeight };

// Translates one scan point into the world frame and applies the range and
// height gates.
Reject project_point(const Point &point, const Eigen::Matrix4f &pose, double z_min,
                     double z_max, double max_range_squared,
                     Eigen::Vector3d &world) {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z)) {
    return Reject::kNonFinite;
  }
  const double range_squared = static_cast<double>(point.x) * point.x +
                               static_cast<double>(point.y) * point.y +
                               static_cast<double>(point.z) * point.z;
  if (range_squared > max_range_squared) return Reject::kRange;
  const Eigen::Vector3f local(point.x, point.y, point.z);
  world = (pose.topLeftCorner<3, 3>() * local + pose.block<3, 1>(0, 3)).cast<double>();
  if (world.z() < z_min || world.z() > z_max) return Reject::kHeight;
  return Reject::kNone;
}

void account_reject(Counters &counters, Reject reject) {
  switch (reject) {
    case Reject::kNonFinite:
      ++counters.dropped_non_finite;
      break;
    case Reject::kRange:
      ++counters.dropped_range;
      break;
    case Reject::kHeight:
      ++counters.dropped_height;
      break;
    case Reject::kNone:
      break;
  }
}

// Estimates one unit normal per coarse cell from the point covariance.  Cells
// that are not clearly planar (vegetation, corners, poles) are discarded, and
// points falling into them keep full incidence weight.
std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> estimate_normals(
    const std::unordered_map<VoxelKey, NormalCell, VoxelKeyHash> &cells,
    const Config &config, std::size_t &planar_cells) {
  std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> normals;
  normals.reserve(cells.size());
  planar_cells = 0;
  for (const auto &[key, cell] : cells) {
    if (cell.count < static_cast<std::size_t>(config.min_normal_points)) continue;
    const double inverse_count = 1.0 / static_cast<double>(cell.count);
    const Eigen::Vector3d mean = cell.sum * inverse_count;
    Eigen::Matrix3d covariance = cell.outer * inverse_count - mean * mean.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) continue;
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    const double l0 = std::max(0.0, eigenvalues(0));
    const double l1 = std::max(0.0, eigenvalues(1));
    const double l2 = std::max(0.0, eigenvalues(2));
    if (l2 <= 0.0) continue;
    const double planarity = (l1 - l0) / l2;
    if (planarity < config.min_planarity) continue;
    normals.emplace(key, solver.eigenvectors().col(0).normalized());
    ++planar_cells;
  }
  return normals;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Config config = parse_args(argc, argv);
    const auto poses = load_poses(config.poses_path);
    const int thread_count = resolve_threads(config, poses.size());
    const double max_range_squared = config.max_range * config.max_range;
    const std::size_t worker_count = static_cast<std::size_t>(thread_count);
    const auto started = std::chrono::steady_clock::now();
    std::mutex warn_mutex;

    // ---- pass A: height clip + surface normals ------------------------------
    std::vector<std::unordered_map<VoxelKey, NormalCell, VoxelKeyHash>> normal_parts(
        worker_count);
    std::vector<Counters> pass_a(worker_count);
    parallel_over_scans(poses.size(), worker_count,
                        [&](std::size_t scan_index, std::size_t worker) {
      const auto path = scan_path(config.scans_directory, scan_index);
      if (!std::filesystem::is_regular_file(path)) {
        std::lock_guard<std::mutex> lock(warn_mutex);
        std::cerr << "Warning: missing scan " << path << '\n';
        return;
      }
      Cloud scan;
      if (pcl::io::loadPCDFile<Point>(path.string(), scan) != 0) {
        throw std::runtime_error("failed to load scan: " + path.string());
      }
      auto &counters = pass_a[worker];
      ++counters.scans;
      counters.points += scan.size();
      auto &grid = normal_parts[worker];
      for (const auto &point : scan) {
        Eigen::Vector3d world;
        const Reject reject =
            project_point(point, poses[scan_index], config.z_min, config.z_max,
                          max_range_squared, world);
        if (reject != Reject::kNone) {
          account_reject(counters, reject);
          continue;
        }
        NormalCell &cell = grid[make_key(world, config.normal_voxel_size)];
        cell.sum += world;
        cell.outer += world * world.transpose();
        ++cell.count;
      }
    });

    std::unordered_map<VoxelKey, NormalCell, VoxelKeyHash> normal_cells;
    Counters pass_a_total;
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
      for (const auto &[key, cell] : normal_parts[worker]) {
        NormalCell &target = normal_cells[key];
        target.sum += cell.sum;
        target.outer += cell.outer;
        target.count += cell.count;
      }
      pass_a_total.scans += pass_a[worker].scans;
      pass_a_total.points += pass_a[worker].points;
      pass_a_total.dropped_non_finite += pass_a[worker].dropped_non_finite;
      pass_a_total.dropped_range += pass_a[worker].dropped_range;
      pass_a_total.dropped_height += pass_a[worker].dropped_height;
    }
    normal_parts.clear();

    std::size_t planar_cells = 0;
    const auto normals = estimate_normals(normal_cells, config, planar_cells);
    const std::size_t normal_cell_count = normal_cells.size();
    normal_cells.clear();

    // ---- pass B: incidence-weighted voxel accumulation ----------------------
    const double cos_max_incidence =
        std::cos(config.max_incidence_deg * kDegToRad);
    const double cos_full_weight = std::cos(config.full_weight_deg * kDegToRad);
    std::vector<std::unordered_map<VoxelKey, VoxelStats, VoxelKeyHash>> voxel_parts(
        worker_count);
    std::vector<Counters> pass_b(worker_count);
    parallel_over_scans(poses.size(), worker_count,
                        [&](std::size_t scan_index, std::size_t worker) {
      const auto path = scan_path(config.scans_directory, scan_index);
      if (!std::filesystem::is_regular_file(path)) return;  // warn in pass A
      Cloud scan;
      if (pcl::io::loadPCDFile<Point>(path.string(), scan) != 0) {
        throw std::runtime_error("failed to load scan: " + path.string());
      }
      auto &counters = pass_b[worker];
      ++counters.scans;
      counters.points += scan.size();
      auto &grid = voxel_parts[worker];
      const Eigen::Matrix4f &pose = poses[scan_index];
      const Eigen::Vector3d origin = pose.block<3, 1>(0, 3).cast<double>();
      const int scan_id = static_cast<int>(scan_index);
      for (const auto &point : scan) {
        Eigen::Vector3d world;
        const Reject reject = project_point(point, pose, config.z_min, config.z_max,
                                            max_range_squared, world);
        if (reject != Reject::kNone) {
          account_reject(counters, reject);
          continue;
        }
        double weight = 1.0;
        const auto normal_it = normals.find(make_key(world, config.normal_voxel_size));
        if (normal_it != normals.end()) {
          const Eigen::Vector3d direction = world - origin;
          const double length = direction.norm();
          if (length > 1e-6) {
            const double cos_incidence =
                std::abs(normal_it->second.dot(direction / length));
            if (cos_incidence < cos_max_incidence) {
              ++counters.dropped_incidence;
              continue;
            }
            const double scaled = cos_incidence / cos_full_weight;
            weight = std::clamp(scaled, config.min_weight, 1.0);
          }
        } else {
          ++counters.no_normal;
        }
        VoxelStats &stats = grid[make_key(world, config.voxel_size)];
        stats.weighted_sum += weight * world;
        stats.weight_sum += weight;
        stats.intensity_weighted_sum += weight * static_cast<double>(point.intensity);
        ++stats.point_count;
        if (stats.last_scan_index != scan_id) {
          stats.last_scan_index = scan_id;
          ++stats.observation_count;
        }
      }
    });

    std::unordered_map<VoxelKey, VoxelStats, VoxelKeyHash> voxels;
    Counters pass_b_total;
    for (std::size_t worker = 0; worker < worker_count; ++worker) {
      for (const auto &[key, stats] : voxel_parts[worker]) {
        VoxelStats &target = voxels[key];
        target.weighted_sum += stats.weighted_sum;
        target.weight_sum += stats.weight_sum;
        target.intensity_weighted_sum += stats.intensity_weighted_sum;
        target.point_count += stats.point_count;
        // Every scan is handled by exactly one worker, so the per-worker scan
        // sets are disjoint and the distinct-scan counts add up.
        target.observation_count += stats.observation_count;
      }
      pass_b_total.scans += pass_b[worker].scans;
      pass_b_total.points += pass_b[worker].points;
      pass_b_total.dropped_non_finite += pass_b[worker].dropped_non_finite;
      pass_b_total.dropped_range += pass_b[worker].dropped_range;
      pass_b_total.dropped_height += pass_b[worker].dropped_height;
      pass_b_total.dropped_incidence += pass_b[worker].dropped_incidence;
      pass_b_total.no_normal += pass_b[worker].no_normal;
    }
    voxel_parts.clear();

    // ---- emit ---------------------------------------------------------------
    Cloud output;
    output.reserve(voxels.size());
    for (const auto &[key, stats] : voxels) {
      (void)key;
      if (stats.observation_count < config.min_observations ||
          stats.point_count == 0 || stats.weight_sum <= 0.0) {
        continue;
      }
      const Eigen::Vector3d centroid = stats.weighted_sum / stats.weight_sum;
      Point point;
      point.x = static_cast<float>(centroid.x());
      point.y = static_cast<float>(centroid.y());
      point.z = static_cast<float>(centroid.z());
      point.intensity =
          static_cast<float>(stats.intensity_weighted_sum / stats.weight_sum);
      output.push_back(point);
    }
    // Sort so the output does not depend on worker scheduling: the merge order
    // of the private accumulators varies, unordered_map iteration order with it.
    std::sort(output.points.begin(), output.points.end(),
              [](const Point &lhs, const Point &rhs) {
                if (lhs.x != rhs.x) return lhs.x < rhs.x;
                if (lhs.y != rhs.y) return lhs.y < rhs.y;
                return lhs.z < rhs.z;
              });
    output.width = output.size();
    output.height = 1;
    output.is_dense = true;
    if (output.empty()) {
      throw std::runtime_error("consistency filter removed every point");
    }
    if (!config.output_path.parent_path().empty()) {
      std::filesystem::create_directories(config.output_path.parent_path());
    }
    if (pcl::io::savePCDFileBinary(config.output_path.string(), output) != 0) {
      throw std::runtime_error("failed to write: " + config.output_path.string());
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started);
    std::cout << "PGO consistency map\n"
              << "  scans            " << pass_b_total.scans << "/" << poses.size()
              << '\n'
              << "  input_points     " << pass_b_total.points << '\n'
              << "  dropped_nonfinite " << pass_b_total.dropped_non_finite << '\n'
              << "  dropped_range    " << pass_b_total.dropped_range << '\n'
              << "  dropped_height   " << pass_b_total.dropped_height << '\n'
              << "  dropped_grazing  " << pass_b_total.dropped_incidence << '\n'
              << "  no_normal        " << pass_b_total.no_normal << '\n'
              << "  normal_cells     " << normal_cell_count << " (planar "
              << planar_cells << ")\n"
              << "  voxels           " << voxels.size() << '\n'
              << "  kept             " << output.size() << " (min_observations="
              << config.min_observations << ")\n"
              << "  voxel            " << config.voxel_size << " m\n"
              << "  height           [" << config.z_min << ", " << config.z_max
              << "] m\n"
              << "  incidence        reject > " << config.max_incidence_deg
              << " deg, full weight <= " << config.full_weight_deg
              << " deg, min_weight " << config.min_weight << '\n'
              << "  threads          " << worker_count << '\n'
              << "  elapsed          " << elapsed.count() << " ms\n"
              << "Wrote: " << config.output_path << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
