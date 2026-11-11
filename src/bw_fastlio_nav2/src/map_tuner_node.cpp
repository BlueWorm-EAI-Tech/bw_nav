// Interactive companion to pcd_to_nav2_map.
//
// Loads one PCD consistency map once, then re-runs the exact same
// CSF ground segmentation + grid projection pipeline whenever a parameter
// changes.  The rendered grid is published as nav_msgs/OccupancyGrid
// (transient local) and a JSON statistics blob is published alongside so the
// rqt_bw_map_tuner panel can draw the result while the sliders move.
//
// The save service writes byte-identical .pgm/.yaml files to what
// pcd_to_nav2_map produces for the same parameters.

#include "bw_fastlio_nav2/csf_ground_segmentation.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bw_fastlio_nav2 {
namespace {

namespace fs = std::filesystem;
using namespace std::chrono_literals;

struct MapTunerConfig {
  std::string input_path;
  std::string output_prefix;
  std::string frame_id{"map"};
  double resolution{0.05};
  double voxel_leaf_size{0.08};
  double cloth_resolution{0.20};
  int rigidness{3};
  int iterations{500};
  double classification_threshold{0.12};
  double time_step{0.65};
  bool slope_smoothing{false};
  double obstacle_min_height{0.25};
  double obstacle_max_height{1.80};
  double ground_reference_radius{0.75};
  int ground_reference_neighbors{5};
  double outlier_radius{0.20};
  int outlier_min_neighbors{2};
  double padding{1.0};
  double occupied_radius{0.08};
  double free_radius{0.25};
  int min_obstacle_neighbors{2};
  double obstacle_cluster_radius{0.0};
  int min_obstacle_component_cells{1};
  // Free-space model and gap closing.  Keep the cheap CLI defaults for the
  // live preview; the production maps enable raycast, islands and sealing.
  std::string free_mode{"geodesic"};
  double free_max_range{40.0};
  double wall_bridge_length{0.0};
  int wall_bridge_directions{4};
  int wall_bridge_min_component_cells{0};
  int min_free_component_cells{0};
  double free_max_offset{0.0};
  std::string seal_mode{"off"};
  std::string free_islands{"off"};
  double free_island_wall_ratio{0.5};
  double free_island_max_area{25.0};
  int unknown_value{127};
  // Trajectory used as the ray-cast seed set.  Only used by free_mode=raycast.
  std::string poses_file;
  std::vector<std::array<double, 2>> free_seeds;
  double occupied_thresh{0.65};
  double free_thresh{0.196};
  bool auto_render{true};
};

struct MapStats {
  std::size_t input_points{0};
  std::size_t filtered_points{0};
  std::size_t ground_points{0};
  std::size_t obstacle_points{0};
  std::size_t obstacle_components{0};
  std::size_t cloud_points{0};
  std::size_t off_ground_points{0};
  std::size_t no_ground_reference_points{0};
  std::size_t occupied_cells{0};
  std::size_t free_cells{0};
  std::size_t unknown_cells{0};
  double render_ms{0.0};
  double load_ms{0.0};
};

CsfSegmentationConfig to_segmentation_config(const MapTunerConfig &config) {
  CsfSegmentationConfig segmentation;
  segmentation.voxel_leaf_size = config.voxel_leaf_size;
  segmentation.cloth_resolution = config.cloth_resolution;
  segmentation.rigidness = config.rigidness;
  segmentation.iterations = config.iterations;
  segmentation.classification_threshold = config.classification_threshold;
  segmentation.time_step = config.time_step;
  segmentation.slope_smoothing = config.slope_smoothing;
  segmentation.obstacle_min_height = config.obstacle_min_height;
  segmentation.obstacle_max_height = config.obstacle_max_height;
  segmentation.ground_reference_radius = config.ground_reference_radius;
  segmentation.ground_reference_neighbors = config.ground_reference_neighbors;
  segmentation.outlier_radius = config.outlier_radius;
  segmentation.outlier_min_neighbors = config.outlier_min_neighbors;
  return segmentation;
}

bool parse_free_mode(const std::string &value, FreeMode &mode) {
  if (value == "euclid") {
    mode = FreeMode::kEuclid;
    return true;
  }
  if (value == "geodesic") {
    mode = FreeMode::kGeodesic;
    return true;
  }
  if (value == "raycast") {
    mode = FreeMode::kRaycast;
    return true;
  }
  return false;
}

bool parse_seal_mode(const std::string &value, SealMode &mode) {
  if (value == "off") {
    mode = SealMode::kOff;
    return true;
  }
  if (value == "free") {
    mode = SealMode::kFree;
    return true;
  }
  return false;
}

// The PGO workflow leaves the trajectory next to map_consistent.pcd.  Keep
// this list in sync with POSE_FILE_CANDIDATES in
// rqt_bw_map_tuner/param_spec.py.
std::string find_pose_file(const std::string &input_path) {
  if (input_path.empty()) {
    return "";
  }
  const fs::path directory = fs::path(input_path).parent_path();
  for (const char *name : {"optimized_poses_batch.txt", "optimized_poses.txt",
                           "odom_poses.txt", "poses.txt",
                           "trajectory.txt"}) {
    std::error_code error;
    const fs::path candidate = directory / name;
    if (fs::is_regular_file(candidate, error)) {
      return candidate.string();
    }
  }
  return "";
}

// An explicit poses_file wins; otherwise look next to the input cloud so the
// GUI and `ros2 run ... map_tuner -p input_path:=...` both work without the
// user having to fill in a second path.
std::string effective_poses_file(const MapTunerConfig &config) {
  if (!config.poses_file.empty()) {
    return config.poses_file;
  }
  return find_pose_file(config.input_path);
}

// One 3x4 row-major matrix per line; the translation column seeds the flood.
bool load_pose_seeds(const std::string &path,
                     std::vector<std::array<double, 2>> &seeds,
                     std::string &error) {
  seeds.clear();
  if (path.empty()) {
    return true;
  }
  std::ifstream file(path);
  if (!file) {
    error = "poses_file: 无法打开 " + path;
    return false;
  }
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    std::istringstream stream(line);
    double values[12] = {0.0};
    bool complete = true;
    for (double &value : values) {
      if (!(stream >> value)) {
        complete = false;
        break;
      }
    }
    if (!complete) {
      error = "poses_file: 第 " + std::to_string(line_number) +
              " 行不是 12 个数字（3x4 行主序）";
      return false;
    }
    seeds.push_back({values[3], values[7]});
  }
  if (seeds.empty()) {
    error = "poses_file: " + path + " 里没有位姿";
    return false;
  }
  return true;
}

GridProjectionConfig to_projection_config(const MapTunerConfig &config) {
  GridProjectionConfig projection;
  projection.resolution = config.resolution;
  projection.padding = config.padding;
  projection.occupied_radius = config.occupied_radius;
  projection.free_radius = config.free_radius;
  projection.min_obstacle_neighbors = config.min_obstacle_neighbors;
  projection.obstacle_cluster_radius = config.obstacle_cluster_radius;
  projection.min_obstacle_component_cells =
      config.min_obstacle_component_cells;
  parse_free_mode(config.free_mode, projection.free_mode);
  projection.free_max_range = config.free_max_range;
  projection.wall_bridge_length = config.wall_bridge_length;
  projection.wall_bridge_directions = config.wall_bridge_directions;
  projection.wall_bridge_min_component_cells =
      config.wall_bridge_min_component_cells;
  projection.min_free_component_cells = config.min_free_component_cells;
  projection.free_max_offset = config.free_max_offset;
  parse_seal_mode(config.seal_mode, projection.seal_mode);
  projection.free_islands_enclosed = (config.free_islands == "enclosed");
  projection.free_island_wall_ratio = config.free_island_wall_ratio;
  projection.free_island_max_area = config.free_island_max_area;
  projection.free_seeds = config.free_seeds;
  return projection;
}

// Returns an empty string when the configuration is usable.  The messages are
// shown verbatim in the rqt panel, so they name the offending parameter.
std::string validate_config(const MapTunerConfig &config) {
  if (config.resolution <= 0.0 || config.resolution > 1.0) {
    return "resolution: 必须在 (0, 1] 之间";
  }
  if (config.voxel_leaf_size <= 0.0) {
    return "voxel_leaf_size: 必须大于 0";
  }
  if (config.cloth_resolution <= 0.0) {
    return "cloth_resolution: 必须大于 0";
  }
  if (config.rigidness < 1 || config.rigidness > 3) {
    return "rigidness: 必须在 1~3 之间";
  }
  if (config.iterations < 10) {
    return "iterations: 不能小于 10";
  }
  if (config.classification_threshold <= 0.0) {
    return "classification_threshold: 必须大于 0";
  }
  if (config.time_step <= 0.0) {
    return "time_step: 必须大于 0";
  }
  if (config.obstacle_min_height < 0.0) {
    return "obstacle_min_height: 不能小于 0";
  }
  if (config.obstacle_max_height <= config.obstacle_min_height) {
    return "obstacle_max_height: 必须大于 obstacle_min_height";
  }
  if (config.ground_reference_radius <= 0.0) {
    return "ground_reference_radius: 必须大于 0";
  }
  if (config.ground_reference_neighbors < 1 ||
      config.ground_reference_neighbors > 50) {
    return "ground_reference_neighbors: 必须在 1~50 之间";
  }
  if (config.outlier_radius < 0.0) {
    return "outlier_radius: 不能小于 0";
  }
  if (config.outlier_min_neighbors < 0) {
    return "outlier_min_neighbors: 不能小于 0";
  }
  if (config.padding < 0.0) {
    return "padding: 不能小于 0";
  }
  if (config.occupied_radius < 0.0) {
    return "occupied_radius: 不能小于 0";
  }
  if (config.free_radius < 0.0) {
    return "free_radius: 不能小于 0";
  }
  if (config.min_obstacle_neighbors < 1) {
    return "min_obstacle_neighbors: 不能小于 1";
  }
  if (config.obstacle_cluster_radius < 0.0) {
    return "obstacle_cluster_radius: 不能小于 0";
  }
  if (config.min_obstacle_component_cells < 1) {
    return "min_obstacle_component_cells: 不能小于 1";
  }
  FreeMode free_mode;
  if (!parse_free_mode(config.free_mode, free_mode)) {
    return "free_mode: 只能是 euclid / geodesic / raycast";
  }
  SealMode seal_mode;
  if (!parse_seal_mode(config.seal_mode, seal_mode)) {
    return "seal_mode: 只能是 off / free";
  }
  if (config.free_islands != "off" && config.free_islands != "enclosed") {
    return "free_islands: 只能是 off / enclosed";
  }
  if (config.free_island_wall_ratio <= 0.0 || config.free_island_wall_ratio > 1.0) {
    return "free_island_wall_ratio: 必须在 (0, 1] 之间";
  }
  if (config.free_island_max_area < 0.0) {
    return "free_island_max_area: 不能小于 0";
  }
  if (config.free_max_range <= 0.0) {
    return "free_max_range: 必须大于 0";
  }
  if (config.wall_bridge_length < 0.0) {
    return "wall_bridge_length: 不能小于 0";
  }
  if (config.wall_bridge_directions != 4 && config.wall_bridge_directions != 8) {
    return "wall_bridge_directions: 只能是 4 或 8";
  }
  if (config.wall_bridge_min_component_cells < 0) {
    return "wall_bridge_min_component_cells: 不能小于 0";
  }
  if (config.min_free_component_cells < 0) {
    return "min_free_component_cells: 不能小于 0";
  }
  if (config.free_max_offset < 0.0) {
    return "free_max_offset: 不能小于 0";
  }
  if (config.unknown_value < 1 || config.unknown_value > 254) {
    return "unknown_value: 必须在 1~254 之间";
  }
  if (free_mode == FreeMode::kRaycast && config.free_seeds.empty()) {
    return "free_mode=raycast 需要 poses_file（PCD 同目录也没有 "
           "optimized_poses_batch.txt 等轨迹文件）";
  }
  if (config.occupied_thresh <= 0.0 || config.occupied_thresh > 1.0) {
    return "occupied_thresh: 必须在 (0, 1] 之间";
  }
  if (config.free_thresh < 0.0 ||
      config.free_thresh >= config.occupied_thresh) {
    return "free_thresh: 必须小于 occupied_thresh";
  }
  return "";
}

bool render_parameters_differ(const MapTunerConfig &lhs,
                              const MapTunerConfig &rhs) {
  return lhs.resolution != rhs.resolution ||
         lhs.voxel_leaf_size != rhs.voxel_leaf_size ||
         lhs.cloth_resolution != rhs.cloth_resolution ||
         lhs.rigidness != rhs.rigidness || lhs.iterations != rhs.iterations ||
         lhs.classification_threshold != rhs.classification_threshold ||
         lhs.time_step != rhs.time_step ||
         lhs.slope_smoothing != rhs.slope_smoothing ||
         lhs.obstacle_min_height != rhs.obstacle_min_height ||
         lhs.obstacle_max_height != rhs.obstacle_max_height ||
         lhs.ground_reference_radius != rhs.ground_reference_radius ||
         lhs.ground_reference_neighbors != rhs.ground_reference_neighbors ||
         lhs.outlier_radius != rhs.outlier_radius ||
         lhs.outlier_min_neighbors != rhs.outlier_min_neighbors ||
         lhs.padding != rhs.padding ||
         lhs.occupied_radius != rhs.occupied_radius ||
         lhs.free_radius != rhs.free_radius ||
         lhs.min_obstacle_neighbors != rhs.min_obstacle_neighbors ||
         lhs.obstacle_cluster_radius != rhs.obstacle_cluster_radius ||
         lhs.min_obstacle_component_cells !=
             rhs.min_obstacle_component_cells ||
         lhs.free_mode != rhs.free_mode ||
         lhs.free_max_range != rhs.free_max_range ||
         lhs.wall_bridge_length != rhs.wall_bridge_length ||
         lhs.wall_bridge_directions != rhs.wall_bridge_directions ||
         lhs.wall_bridge_min_component_cells !=
             rhs.wall_bridge_min_component_cells ||
         lhs.min_free_component_cells != rhs.min_free_component_cells ||
         lhs.free_max_offset != rhs.free_max_offset ||
         lhs.seal_mode != rhs.seal_mode ||
         lhs.free_islands != rhs.free_islands ||
         lhs.free_island_wall_ratio != rhs.free_island_wall_ratio ||
         lhs.free_island_max_area != rhs.free_island_max_area ||
         lhs.unknown_value != rhs.unknown_value ||
         lhs.free_seeds != rhs.free_seeds;
}

std::string json_escape(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size() + 8);
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(character) < 0x20) {
          std::ostringstream code;
          code << "\\u" << std::hex << std::setw(4) << std::setfill('0')
               << static_cast<int>(static_cast<unsigned char>(character));
          escaped += code.str();
        } else {
          escaped += character;
        }
    }
  }
  return escaped;
}

std::string format_fixed(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

// The preview overlay is a debugging aid, so cap how many points travel over
// DDS when a very dense consistency map is loaded.
constexpr std::size_t kPreviewPointLimit = 300000;

struct PointIdentity {
  std::uint32_t x;
  std::uint32_t y;
  std::uint32_t z;

  bool operator==(const PointIdentity &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PointIdentityHash {
  std::size_t operator()(const PointIdentity &identity) const noexcept {
    std::size_t seed = 0;
    for (const std::uint32_t value : {identity.x, identity.y, identity.z}) {
      seed ^= static_cast<std::size_t>(value) + 0x9e3779b9U + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

using PointIdentitySet =
    std::unordered_set<PointIdentity, PointIdentityHash>;

PointIdentity point_identity(const pcl::PointXYZI &point) {
  static_assert(sizeof(point.x) == sizeof(std::uint32_t),
                "float must be 32 bit to classify points by their bits");
  PointIdentity identity{};
  std::memcpy(&identity.x, &point.x, sizeof(point.x));
  std::memcpy(&identity.y, &point.y, sizeof(point.y));
  std::memcpy(&identity.z, &point.z, sizeof(point.z));
  return identity;
}

PointIdentitySet identity_set(const Cloud &cloud) {
  PointIdentitySet identities;
  identities.reserve(cloud.size() * 2);
  for (const auto &point : cloud) {
    identities.insert(point_identity(point));
  }
  return identities;
}

// The ground and obstacle clouds are copies of the filtered cloud, so the
// preview can be assembled from the filtered points alone.  intensity carries
// the class: 0 = ground, 1 = obstacle, 2 = filtered but not used by the grid.
Cloud::Ptr build_preview_cloud(const GroundSegmentationResult &segmented) {
  const PointIdentitySet ground = identity_set(*segmented.ground);
  const PointIdentitySet obstacles = identity_set(*segmented.obstacles);
  const std::size_t total = segmented.filtered->size();
  const std::size_t stride =
      total > kPreviewPointLimit
          ? (total + kPreviewPointLimit - 1) / kPreviewPointLimit
          : 1;

  auto preview = std::make_shared<Cloud>();
  preview->reserve((total + stride - 1) / stride);
  for (std::size_t index = 0; index < total; index += stride) {
    const auto &point = (*segmented.filtered)[index];
    const PointIdentity identity = point_identity(point);
    float intensity = 2.0F;
    if (ground.count(identity) != 0U) {
      intensity = 0.0F;
    } else if (obstacles.count(identity) != 0U) {
      intensity = 1.0F;
    }
    pcl::PointXYZI preview_point;
    preview_point.x = point.x;
    preview_point.y = point.y;
    preview_point.z = point.z;
    preview_point.intensity = intensity;
    preview->push_back(preview_point);
  }
  preview->width = static_cast<std::uint32_t>(preview->size());
  preview->height = 1;
  preview->is_dense = true;
  return preview;
}

struct RenderOutcome {
  GridProjection grid;
  MapStats stats;
  Cloud::Ptr preview_cloud;
};

RenderOutcome render_map(const Cloud &input, const MapTunerConfig &config) {
  const auto start = std::chrono::steady_clock::now();
  const auto segmented =
      segment_ground_csf(input, to_segmentation_config(config));
  RenderOutcome outcome;
  outcome.grid = project_xy(*segmented.ground, *segmented.obstacles,
                            to_projection_config(config));
  outcome.preview_cloud = build_preview_cloud(segmented);
  outcome.stats.input_points = input.size();
  outcome.stats.filtered_points = segmented.filtered->size();
  outcome.stats.ground_points = segmented.ground->size();
  outcome.stats.obstacle_points = segmented.obstacles->size();
  outcome.stats.obstacle_components = outcome.grid.obstacle_components;
  outcome.stats.cloud_points = outcome.preview_cloud->size();
  outcome.stats.off_ground_points = segmented.off_ground_points;
  outcome.stats.no_ground_reference_points =
      segmented.no_ground_reference_points;

  const auto &grid = outcome.grid;
  for (std::size_t index = 0; index < grid.occupied.size(); ++index) {
    if (grid.occupied[index] != 0) {
      ++outcome.stats.occupied_cells;
    } else if (grid.observed[index] != 0) {
      ++outcome.stats.free_cells;
    } else {
      ++outcome.stats.unknown_cells;
    }
  }
  outcome.stats.render_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start)
          .count();
  return outcome;
}

// Writes the same bytes as pcd_to_nav2_map.cpp for an identical grid/config.
void write_map_files(const fs::path &prefix, const GridProjection &grid,
                     const MapTunerConfig &config) {
  if (prefix.has_parent_path()) {
    fs::create_directories(prefix.parent_path());
  }
  const fs::path pgm_path = prefix.string() + ".pgm";
  const fs::path yaml_path = prefix.string() + ".yaml";
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("无法写入: " + pgm_path.string());
  }
  pgm << "P5\n" << grid.width << " " << grid.height << "\n255\n";
  for (std::size_t output_y = 0; output_y < grid.height; ++output_y) {
    const std::size_t grid_y = grid.height - 1 - output_y;
    for (std::size_t x = 0; x < grid.width; ++x) {
      const std::size_t index = grid_y * grid.width + x;
      std::uint8_t value = static_cast<std::uint8_t>(config.unknown_value);
      if (grid.occupied[index] != 0) {
        value = 0;
      } else if (grid.observed[index] != 0) {
        value = 254;
      }
      pgm.write(reinterpret_cast<const char *>(&value), 1);
    }
  }
  pgm.close();
  if (!pgm) {
    throw std::runtime_error("写入失败: " + pgm_path.string());
  }

  std::ofstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("无法写入: " + yaml_path.string());
  }
  yaml << "image: " << pgm_path.filename().string() << "\n"
       << "mode: trinary\n" << std::fixed << std::setprecision(6)
       << "resolution: " << config.resolution << "\n"
       << "origin: [" << grid.origin_x << ", " << grid.origin_y
       << ", 0.0]\n"
       << "negate: 0\n"
       << std::defaultfloat
       << "occupied_thresh: " << config.occupied_thresh << "\n"
       << "free_thresh: " << config.free_thresh << "\n";
  yaml.close();
  if (!yaml) {
    throw std::runtime_error("写入失败: " + yaml_path.string());
  }
}

}  // namespace

class MapTunerNode : public rclcpp::Node {
 public:
  MapTunerNode() : Node("map_tuner") {
    declare_parameter<std::string>("input_path", "");
    declare_parameter<std::string>("output_prefix", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("auto_render", true);

    declare_parameter<double>("resolution", 0.05);
    declare_parameter<double>("padding", 1.0);
    declare_parameter<double>("occupied_radius", 0.08);
    declare_parameter<double>("free_radius", 0.25);
    declare_parameter<int>("min_obstacle_neighbors", 2);
    declare_parameter<double>("obstacle_cluster_radius", 0.0);
    declare_parameter<int>("min_obstacle_component_cells", 1);

    declare_parameter<std::string>("free_mode", "geodesic");
    declare_parameter<double>("free_max_range", 40.0);
    declare_parameter<double>("wall_bridge_length", 0.0);
    declare_parameter<int>("wall_bridge_directions", 4);
    declare_parameter<int>("wall_bridge_min_component_cells", 0);
    declare_parameter<int>("min_free_component_cells", 0);
    declare_parameter<double>("free_max_offset", 0.0);
    declare_parameter<std::string>("seal_mode", "off");
    declare_parameter<std::string>("free_islands", "off");
    declare_parameter<double>("free_island_wall_ratio", 0.5);
    declare_parameter<double>("free_island_max_area", 25.0);
    declare_parameter<int>("unknown_value", 127);
    declare_parameter<std::string>("poses_file", "");

    declare_parameter<double>("voxel_leaf_size", 0.08);
    declare_parameter<double>("cloth_resolution", 0.20);
    declare_parameter<int>("rigidness", 3);
    declare_parameter<int>("iterations", 500);
    declare_parameter<double>("classification_threshold", 0.12);
    declare_parameter<double>("time_step", 0.65);
    declare_parameter<bool>("slope_smoothing", false);

    declare_parameter<double>("obstacle_min_height", 0.25);
    declare_parameter<double>("obstacle_max_height", 1.80);
    declare_parameter<double>("ground_reference_radius", 0.75);
    declare_parameter<int>("ground_reference_neighbors", 5);
    declare_parameter<double>("outlier_radius", 0.20);
    declare_parameter<int>("outlier_min_neighbors", 2);

    declare_parameter<double>("occupied_thresh", 0.65);
    declare_parameter<double>("free_thresh", 0.196);

    config_ = read_config();
    {
      std::string pose_error;
      const std::string poses = effective_poses_file(config_);
      if (!load_pose_seeds(poses, config_.free_seeds, pose_error)) {
        RCLCPP_WARN(get_logger(), "%s", pose_error.c_str());
      } else if (config_.poses_file.empty() && !poses.empty()) {
        RCLCPP_INFO(get_logger(), "poses_file 未设置，自动使用 %s",
                    poses.c_str());
      }
    }

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.transient_local().reliable();
    map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>("~/map", map_qos);
    stats_publisher_ =
        create_publisher<std_msgs::msg::String>("~/stats", map_qos);
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/cloud", map_qos);

    reload_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reload",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          handle_reload(request, response);
        });
    render_service_ = create_service<std_srvs::srv::Trigger>(
        "~/render",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          handle_render(request, response);
        });
    save_service_ = create_service<std_srvs::srv::Trigger>(
        "~/save",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          handle_save(request, response);
        });

    parameter_callback_handle_ =
        add_on_set_parameters_callback([this](const auto &parameters) {
          return on_parameters_set(parameters);
        });

    worker_ = std::thread(&MapTunerNode::worker_loop, this);

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (config_.input_path.empty()) {
      state_ = "idle";
      message_ = "等待设置 input_path";
      publish_stats_locked();
    } else {
      request_load_locked();
    }
  }

  ~MapTunerNode() override {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      shutdown_ = true;
    }
    job_cv_.notify_all();
    render_done_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  struct Job {
    bool load{false};
    std::uint64_t sequence{0};
    MapTunerConfig config;
  };

  MapTunerConfig read_config() const {
    MapTunerConfig config;
    config.input_path = get_parameter("input_path").as_string();
    config.output_prefix = get_parameter("output_prefix").as_string();
    config.frame_id = get_parameter("frame_id").as_string();
    config.auto_render = get_parameter("auto_render").as_bool();
    config.resolution = get_parameter("resolution").as_double();
    config.padding = get_parameter("padding").as_double();
    config.occupied_radius = get_parameter("occupied_radius").as_double();
    config.free_radius = get_parameter("free_radius").as_double();
    config.min_obstacle_neighbors =
        static_cast<int>(get_parameter("min_obstacle_neighbors").as_int());
    config.obstacle_cluster_radius =
        get_parameter("obstacle_cluster_radius").as_double();
    config.min_obstacle_component_cells = static_cast<int>(
        get_parameter("min_obstacle_component_cells").as_int());
    config.free_mode = get_parameter("free_mode").as_string();
    config.free_max_range = get_parameter("free_max_range").as_double();
    config.wall_bridge_length = get_parameter("wall_bridge_length").as_double();
    config.wall_bridge_directions =
        static_cast<int>(get_parameter("wall_bridge_directions").as_int());
    config.wall_bridge_min_component_cells = static_cast<int>(
        get_parameter("wall_bridge_min_component_cells").as_int());
    config.min_free_component_cells =
        static_cast<int>(get_parameter("min_free_component_cells").as_int());
    config.free_max_offset = get_parameter("free_max_offset").as_double();
    config.seal_mode = get_parameter("seal_mode").as_string();
    config.free_islands = get_parameter("free_islands").as_string();
    config.free_island_wall_ratio =
        get_parameter("free_island_wall_ratio").as_double();
    config.free_island_max_area =
        get_parameter("free_island_max_area").as_double();
    config.unknown_value =
        static_cast<int>(get_parameter("unknown_value").as_int());
    config.poses_file = get_parameter("poses_file").as_string();
    config.voxel_leaf_size = get_parameter("voxel_leaf_size").as_double();
    config.cloth_resolution = get_parameter("cloth_resolution").as_double();
    config.rigidness = static_cast<int>(get_parameter("rigidness").as_int());
    config.iterations = static_cast<int>(get_parameter("iterations").as_int());
    config.classification_threshold =
        get_parameter("classification_threshold").as_double();
    config.time_step = get_parameter("time_step").as_double();
    config.slope_smoothing = get_parameter("slope_smoothing").as_bool();
    config.obstacle_min_height =
        get_parameter("obstacle_min_height").as_double();
    config.obstacle_max_height =
        get_parameter("obstacle_max_height").as_double();
    config.ground_reference_radius =
        get_parameter("ground_reference_radius").as_double();
    config.ground_reference_neighbors = static_cast<int>(
        get_parameter("ground_reference_neighbors").as_int());
    config.outlier_radius = get_parameter("outlier_radius").as_double();
    config.outlier_min_neighbors =
        static_cast<int>(get_parameter("outlier_min_neighbors").as_int());
    config.occupied_thresh = get_parameter("occupied_thresh").as_double();
    config.free_thresh = get_parameter("free_thresh").as_double();
    return config;
  }

  static bool apply_parameter(MapTunerConfig &config,
                              const rclcpp::Parameter &parameter,
                              std::string &error) {
    const std::string &name = parameter.get_name();
    const auto type = parameter.get_type();
    const auto as_double = [&](double &target) {
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        target = parameter.as_double();
        return true;
      }
      if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        target = static_cast<double>(parameter.as_int());
        return true;
      }
      error = name + ": 需要数值";
      return false;
    };
    const auto as_int = [&](int &target) {
      if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
        target = static_cast<int>(parameter.as_int());
        return true;
      }
      if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        const double value = parameter.as_double();
        if (value == static_cast<double>(static_cast<long long>(value))) {
          target = static_cast<int>(value);
          return true;
        }
      }
      error = name + ": 需要整数";
      return false;
    };
    const auto as_bool = [&](bool &target) {
      if (type != rclcpp::ParameterType::PARAMETER_BOOL) {
        error = name + ": 需要布尔值";
        return false;
      }
      target = parameter.as_bool();
      return true;
    };
    const auto as_string = [&](std::string &target) {
      if (type != rclcpp::ParameterType::PARAMETER_STRING) {
        error = name + ": 需要字符串";
        return false;
      }
      target = parameter.as_string();
      return true;
    };

    if (name == "input_path") return as_string(config.input_path);
    if (name == "output_prefix") return as_string(config.output_prefix);
    if (name == "frame_id") return as_string(config.frame_id);
    if (name == "auto_render") return as_bool(config.auto_render);
    if (name == "resolution") return as_double(config.resolution);
    if (name == "padding") return as_double(config.padding);
    if (name == "occupied_radius") return as_double(config.occupied_radius);
    if (name == "free_radius") return as_double(config.free_radius);
    if (name == "min_obstacle_neighbors") {
      return as_int(config.min_obstacle_neighbors);
    }
    if (name == "obstacle_cluster_radius") {
      return as_double(config.obstacle_cluster_radius);
    }
    if (name == "min_obstacle_component_cells") {
      return as_int(config.min_obstacle_component_cells);
    }
    if (name == "free_mode") return as_string(config.free_mode);
    if (name == "free_max_range") return as_double(config.free_max_range);
    if (name == "wall_bridge_length") {
      return as_double(config.wall_bridge_length);
    }
    if (name == "wall_bridge_directions") {
      return as_int(config.wall_bridge_directions);
    }
    if (name == "wall_bridge_min_component_cells") {
      return as_int(config.wall_bridge_min_component_cells);
    }
    if (name == "min_free_component_cells") {
      return as_int(config.min_free_component_cells);
    }
    if (name == "free_max_offset") return as_double(config.free_max_offset);
    if (name == "seal_mode") return as_string(config.seal_mode);
    if (name == "free_islands") return as_string(config.free_islands);
    if (name == "free_island_wall_ratio")
      return as_double(config.free_island_wall_ratio);
    if (name == "free_island_max_area")
      return as_double(config.free_island_max_area);
    if (name == "unknown_value") return as_int(config.unknown_value);
    if (name == "poses_file") return as_string(config.poses_file);
    if (name == "voxel_leaf_size") return as_double(config.voxel_leaf_size);
    if (name == "cloth_resolution") return as_double(config.cloth_resolution);
    if (name == "rigidness") return as_int(config.rigidness);
    if (name == "iterations") return as_int(config.iterations);
    if (name == "classification_threshold") {
      return as_double(config.classification_threshold);
    }
    if (name == "time_step") return as_double(config.time_step);
    if (name == "slope_smoothing") return as_bool(config.slope_smoothing);
    if (name == "obstacle_min_height") {
      return as_double(config.obstacle_min_height);
    }
    if (name == "obstacle_max_height") {
      return as_double(config.obstacle_max_height);
    }
    if (name == "ground_reference_radius") {
      return as_double(config.ground_reference_radius);
    }
    if (name == "ground_reference_neighbors") {
      return as_int(config.ground_reference_neighbors);
    }
    if (name == "outlier_radius") return as_double(config.outlier_radius);
    if (name == "outlier_min_neighbors") {
      return as_int(config.outlier_min_neighbors);
    }
    if (name == "occupied_thresh") return as_double(config.occupied_thresh);
    if (name == "free_thresh") return as_double(config.free_thresh);
    error = name + ": 未知参数";
    return false;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(state_mutex_);
    MapTunerConfig next = config_;
    for (const auto &parameter : parameters) {
      std::string error;
      if (!apply_parameter(next, parameter, error)) {
        result.successful = false;
        result.reason = error;
        return result;
      }
    }
    if (next.poses_file != config_.poses_file ||
        next.input_path != config_.input_path) {
      std::string pose_error;
      const std::string poses = effective_poses_file(next);
      if (!load_pose_seeds(poses, next.free_seeds, pose_error)) {
        result.successful = false;
        result.reason = pose_error;
        return result;
      }
      if (next.poses_file.empty() && !poses.empty()) {
        RCLCPP_INFO(get_logger(), "poses_file 未设置，自动使用 %s",
                    poses.c_str());
      }
    }
    const std::string error = validate_config(next);
    if (!error.empty()) {
      result.successful = false;
      result.reason = error;
      return result;
    }

    const bool path_changed = next.input_path != config_.input_path;
    const bool render_changed = render_parameters_differ(config_, next);
    config_ = next;
    if (path_changed && !config_.input_path.empty()) {
      request_load_locked();
    } else if (render_changed && config_.auto_render) {
      request_render_locked();
    }
    publish_stats_locked();
    return result;
  }

  void request_load_locked() {
    ++request_sequence_;
    pending_job_ = Job{true, request_sequence_, config_};
    state_ = "loading";
    message_.clear();
    job_cv_.notify_all();
  }

  void request_render_locked() {
    ++request_sequence_;
    if (pending_job_.has_value() && pending_job_->load) {
      pending_job_->sequence = request_sequence_;
      pending_job_->config = config_;
    } else {
      pending_job_ = Job{false, request_sequence_, config_};
    }
    state_ = "rendering";
    message_.clear();
    job_cv_.notify_all();
  }

  void handle_reload(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (config_.input_path.empty()) {
      response->success = false;
      response->message = "input_path 未设置";
      return;
    }
    request_load_locked();
    response->success = true;
    response->message = "开始加载: " + config_.input_path;
  }

  void handle_render(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!cloud_) {
      if (!config_.input_path.empty()) {
        request_load_locked();
        response->success = true;
        response->message = "点云尚未加载，开始加载: " + config_.input_path;
      } else {
        response->success = false;
        response->message = "还没有可渲染的点云，请先设置 input_path";
      }
      return;
    }
    request_render_locked();
    response->success = true;
    response->message = "已排队渲染 (请求 " +
                        std::to_string(request_sequence_) + ")";
  }

  void handle_save(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::uint64_t target = 0;
    GridProjection grid;
    MapTunerConfig published_config;
    std::string output_prefix;
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      target = request_sequence_;
      if (target == 0 && published_sequence_ == 0) {
        response->success = false;
        response->message = "还没有生成地图，请先加载点云";
        return;
      }
      if (published_sequence_ < target) {
        const bool done = render_done_cv_.wait_for(
            lock, 20s, [this, target] {
              return published_sequence_ >= target ||
                     failed_sequence_ >= target || shutdown_;
            });
        if (!done || published_sequence_ < target) {
          response->success = false;
          response->message =
              message_.empty() ? "等待渲染完成超时" : message_;
          return;
        }
      }
      if (published_sequence_ == 0) {
        response->success = false;
        response->message = "还没有生成地图，请先加载点云";
        return;
      }
      grid = published_grid_;
      published_config = published_config_;
      output_prefix = config_.output_prefix;
    }
    if (output_prefix.empty()) {
      response->success = false;
      response->message = "output_prefix 未设置";
      return;
    }
    try {
      const fs::path prefix(output_prefix);
      write_map_files(prefix, grid, published_config);
      std::ostringstream message;
      message << "已保存 " << fs::absolute(prefix.string() + ".pgm").string()
              << " 和 " << fs::absolute(prefix.string() + ".yaml").string()
              << " (" << grid.width << "x" << grid.height << ")";
      response->success = true;
      response->message = message.str();
      RCLCPP_INFO(get_logger(), "%s", message.str().c_str());
    } catch (const std::exception &error) {
      response->success = false;
      response->message = std::string("保存失败: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  void worker_loop() {
    std::unique_lock<std::mutex> lock(state_mutex_);
    while (true) {
      job_cv_.wait(lock, [this] {
        return shutdown_ || pending_job_.has_value();
      });
      if (shutdown_) {
        break;
      }
      Job job = std::move(*pending_job_);
      pending_job_.reset();

      if (job.load) {
        state_ = "loading";
        message_.clear();
        publish_stats_locked();
        const std::string path = job.config.input_path;
        lock.unlock();

        Cloud::Ptr loaded(new Cloud);
        const auto load_start = std::chrono::steady_clock::now();
        const int load_result = pcl::io::loadPCDFile(path, *loaded);
        const double load_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - load_start)
                .count();

        lock.lock();
        if (load_result != 0 || loaded->empty()) {
          cloud_.reset();
          stats_.input_points = 0;
          stats_.load_ms = load_ms;
          state_ = "error";
          message_ = "无法读取点云: " + path;
          failed_sequence_ = job.sequence;
          publish_stats_locked();
          render_done_cv_.notify_all();
          continue;
        }
        cloud_ = loaded;
        stats_.input_points = loaded->size();
        stats_.load_ms = load_ms;
        if (job.sequence != request_sequence_) {
          // A newer request is queued; it will render the freshly loaded cloud.
          continue;
        }
      }

      if (!cloud_) {
        state_ = "error";
        message_ = "还没有可渲染的点云，请先设置 input_path";
        failed_sequence_ = job.sequence;
        publish_stats_locked();
        render_done_cv_.notify_all();
        continue;
      }

      state_ = "rendering";
      publish_stats_locked();
      Cloud::Ptr cloud = cloud_;
      const MapTunerConfig config = job.config;
      lock.unlock();

      RenderOutcome outcome;
      std::string render_error;
      try {
        outcome = render_map(*cloud, config);
      } catch (const std::exception &error) {
        render_error = error.what();
      }

      lock.lock();
      if (job.sequence != request_sequence_) {
        continue;
      }
      if (!render_error.empty()) {
        state_ = "error";
        message_ = render_error;
        failed_sequence_ = job.sequence;
        publish_stats_locked();
        render_done_cv_.notify_all();
        continue;
      }
      published_grid_ = std::move(outcome.grid);
      published_config_ = config;
      published_cloud_ = std::move(outcome.preview_cloud);
      stats_.filtered_points = outcome.stats.filtered_points;
      stats_.ground_points = outcome.stats.ground_points;
      stats_.obstacle_points = outcome.stats.obstacle_points;
      stats_.obstacle_components = outcome.stats.obstacle_components;
      stats_.cloud_points = outcome.stats.cloud_points;
      stats_.off_ground_points = outcome.stats.off_ground_points;
      stats_.no_ground_reference_points =
          outcome.stats.no_ground_reference_points;
      stats_.occupied_cells = outcome.stats.occupied_cells;
      stats_.free_cells = outcome.stats.free_cells;
      stats_.unknown_cells = outcome.stats.unknown_cells;
      stats_.render_ms = outcome.stats.render_ms;
      published_sequence_ = job.sequence;
      state_ = "ok";
      message_.clear();
      publish_map_locked();
      publish_cloud_locked();
      publish_stats_locked();
      render_done_cv_.notify_all();
      RCLCPP_INFO(get_logger(),
                  "rendered %zux%zu map in %.1f ms (request %lu)",
                  published_grid_.width, published_grid_.height,
                  stats_.render_ms,
                  static_cast<unsigned long>(published_sequence_));
    }
  }

  void publish_map_locked() {
    nav_msgs::msg::OccupancyGrid message;
    message.header.frame_id = published_config_.frame_id;
    message.header.stamp = now();
    message.info.resolution = published_config_.resolution;
    message.info.width = static_cast<std::uint32_t>(published_grid_.width);
    message.info.height = static_cast<std::uint32_t>(published_grid_.height);
    message.info.origin.position.x = published_grid_.origin_x;
    message.info.origin.position.y = published_grid_.origin_y;
    message.info.origin.position.z = 0.0;
    message.info.origin.orientation.w = 1.0;
    message.data.resize(published_grid_.occupied.size());
    for (std::size_t index = 0; index < published_grid_.occupied.size();
         ++index) {
      if (published_grid_.occupied[index] != 0) {
        message.data[index] = 100;
      } else if (published_grid_.observed[index] != 0) {
        message.data[index] = 0;
      } else {
        message.data[index] = -1;
      }
    }
    map_publisher_->publish(message);
  }

  void publish_cloud_locked() {
    if (!published_cloud_ || published_cloud_->empty()) {
      return;
    }
    struct CloudPoint {
      float x;
      float y;
      float z;
      float intensity;
    };
    static_assert(sizeof(CloudPoint) == 16, "unexpected PointCloud2 layout");

    sensor_msgs::msg::PointCloud2 message;
    message.header.frame_id = published_config_.frame_id;
    message.header.stamp = now();
    message.height = 1;
    message.width = static_cast<std::uint32_t>(published_cloud_->size());
    message.fields.resize(4);
    message.fields[0].name = "x";
    message.fields[1].name = "y";
    message.fields[2].name = "z";
    message.fields[3].name = "intensity";
    for (std::size_t index = 0; index < message.fields.size(); ++index) {
      message.fields[index].offset = static_cast<std::uint32_t>(index * 4);
      message.fields[index].datatype =
          sensor_msgs::msg::PointField::FLOAT32;
      message.fields[index].count = 1;
    }
    message.is_bigendian = false;
    message.point_step = sizeof(CloudPoint);
    message.row_step = message.point_step * message.width;
    message.is_dense = true;
    message.data.resize(static_cast<std::size_t>(message.row_step));
    for (std::size_t index = 0; index < published_cloud_->size(); ++index) {
      const auto &point = (*published_cloud_)[index];
      const CloudPoint value{point.x, point.y, point.z, point.intensity};
      std::memcpy(message.data.data() + index * sizeof(CloudPoint), &value,
                  sizeof(value));
    }
    cloud_publisher_->publish(message);
  }

  void publish_stats_locked() {
    std::ostringstream json;
    json << '{';
    json << "\"state\":\"" << json_escape(state_) << "\",";
    json << "\"message\":\"" << json_escape(message_) << "\",";
    json << "\"has_map\":" << (published_sequence_ > 0 ? "true" : "false")
         << ',';
    json << "\"seq_requested\":" << request_sequence_ << ',';
    json << "\"seq_published\":" << published_sequence_ << ',';
    json << "\"dirty\":"
         << (request_sequence_ > published_sequence_ ? "true" : "false")
         << ',';
    json << "\"input_path\":\"" << json_escape(config_.input_path) << "\",";
    json << "\"output_prefix\":\"" << json_escape(config_.output_prefix)
         << "\",";
    json << "\"frame_id\":\"" << json_escape(published_config_.frame_id)
         << "\",";
    json << "\"input_points\":" << stats_.input_points << ',';
    json << "\"filtered_points\":" << stats_.filtered_points << ',';
    json << "\"ground_points\":" << stats_.ground_points << ',';
    json << "\"obstacle_points\":" << stats_.obstacle_points << ',';
    json << "\"obstacle_components\":" << stats_.obstacle_components << ',';
    json << "\"cloud_points\":" << stats_.cloud_points << ',';
    json << "\"off_ground_points\":" << stats_.off_ground_points << ',';
    json << "\"no_ground_reference_points\":"
         << stats_.no_ground_reference_points << ',';
    json << "\"width\":" << published_grid_.width << ',';
    json << "\"height\":" << published_grid_.height << ',';
    json << "\"resolution\":" << format_fixed(published_config_.resolution, 6)
         << ',';
    json << "\"origin_x\":" << format_fixed(published_grid_.origin_x, 6)
         << ',';
    json << "\"origin_y\":" << format_fixed(published_grid_.origin_y, 6)
         << ',';
    json << "\"occupied_cells\":" << stats_.occupied_cells << ',';
    json << "\"free_cells\":" << stats_.free_cells << ',';
    json << "\"unknown_cells\":" << stats_.unknown_cells << ',';
    json << "\"render_ms\":" << format_fixed(stats_.render_ms, 1) << ',';
    json << "\"load_ms\":" << format_fixed(stats_.load_ms, 1) << ',';
    json << "\"auto_render\":"
         << (config_.auto_render ? "true" : "false") << '}';

    std_msgs::msg::String message;
    message.data = json.str();
    stats_publisher_->publish(message);
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cloud_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr render_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;

  mutable std::mutex state_mutex_;
  std::condition_variable job_cv_;
  std::condition_variable render_done_cv_;
  std::thread worker_;

  MapTunerConfig config_;
  Cloud::Ptr cloud_;
  MapStats stats_;
  GridProjection published_grid_;
  MapTunerConfig published_config_;
  Cloud::Ptr published_cloud_;
  std::string state_{"idle"};
  std::string message_;
  std::uint64_t request_sequence_{0};
  std::uint64_t published_sequence_{0};
  std::uint64_t failed_sequence_{0};
  std::optional<Job> pending_job_;
  bool shutdown_{false};
};

}  // namespace bw_fastlio_nav2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bw_fastlio_nav2::MapTunerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
