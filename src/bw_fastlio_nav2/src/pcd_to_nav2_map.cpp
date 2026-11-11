#include "bw_fastlio_nav2/csf_ground_segmentation.hpp"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using bw_fastlio_nav2::Cloud;
using bw_fastlio_nav2::CsfSegmentationConfig;
using bw_fastlio_nav2::GridProjectionConfig;

struct Options {
  std::string input;
  std::string output_prefix;
  double resolution = 0.05;
  double voxel_leaf_size = 0.08;
  double cloth_resolution = 0.20;
  int rigidness = 3;
  int iterations = 500;
  double classification_threshold = 0.12;
  double time_step = 0.65;
  bool slope_smoothing = false;
  double min_height = 0.25;
  double max_height = 1.80;
  double ground_reference_radius = 0.75;
  int ground_reference_neighbors = 5;
  double outlier_radius = 0.20;
  int outlier_min_neighbors = 2;
  double padding = 1.0;
  double occupied_radius = 0.08;
  double free_radius = 0.25;
  int min_obstacle_neighbors = 2;
  int min_obstacle_component_cells = 1;
  double obstacle_cluster_radius = 0.0;
  // gap-closing pipeline
  std::string free_mode = "geodesic";
  double free_max_range = 40.0;
  double free_max_offset = 0.0;
  std::string free_islands = "off";
  double free_island_wall_ratio = 0.5;
  double free_island_max_area = 25.0;
  int min_free_component_cells = 0;
  double wall_bridge_length = 0.0;
  int wall_bridge_directions = 4;
  int wall_bridge_min_component_cells = 0;
  std::string seal_mode = "off";
  int unknown_value = 127;
  // Optional trajectory (one 3x4 row-major matrix per line).  Free space is
  // flooded from these positions; anything unreachable is dropped.
  std::string poses;
  // Debug aid: write the segmented ground and obstacle clouds next to the map.
  std::string dump_dir;
};

void print_usage(const char *program) {
  std::cout
      << "Usage: " << program
      << " --input map.pcd --output-prefix maps/nav2/map\n"
      << "  [--resolution 0.05] [--voxel-size 0.08]\n"
      << "  [--cloth-resolution 0.20] [--rigidness 3] [--iterations 500]\n"
      << "  [--classification-threshold 0.12] [--time-step 0.65]\n"
      << "  [--slope-smoothing true|false] [--min-height 0.25]"
         " [--max-height 1.80] [--ground-reference-radius 0.75]\n"
      << "  [--ground-reference-neighbors 5]\n"
      << "  [--outlier-radius 0.20] [--outlier-min-neighbors 2]\n"
      << "  [--min-obstacle-neighbors 2] [--padding 1.0]\n"
      << "  [--occupied-radius 0.08] [--free-radius 0.25]\n"
      << "  [--min-obstacle-component-cells 1] [--obstacle-cluster-radius 0.0]\n"
      << "  [--free-mode euclid|geodesic|raycast] [--free-max-range 40]\n"
      << "  [--free-max-offset 0.0]\n"
      << "  [--free-islands off|enclosed] [--free-island-wall-ratio 0.5]\n"
      << "  [--free-island-max-area 25]\n"
      << "  [--min-free-component-cells 0]\n"
      << "  [--wall-bridge-length 0.0] [--wall-bridge-directions 4|8]\n"
      << "  [--wall-bridge-min-component-cells 0]\n"
      << "  [--seal-mode off|free] [--unknown-value 127]\n"
      << "  [--poses poses.txt] [--dump-dir DIR]\n"
      << "\n"
      << "Semantics worth knowing:\n"
      << "  --free-radius means two different things depending on --free-mode.\n"
      << "    euclid   : radius of the disk dilated around every floor return\n"
      << "               (the historical behaviour; large values leak through\n"
      << "               walls, 1.38 was used by the old pipeline).\n"
      << "    geodesic : how far free space may creep along unobstructed cells.\n"
      << "    raycast  : margin applied after the line-of-sight flood; it only\n"
      << "               closes the gaps between neighbouring rays, so 0.2~0.5\n"
      << "               is the useful range.\n"
      << "  --obstacle-cluster-radius closes gaps narrower than twice that\n"
      << "    radius, merging returns of the same object cluster into one blob;\n"
      << "    unlike the directional bridging it cannot draw long lines.  Use\n"
      << "    it on the nav map only; BBS wants the raw thin obstacles.\n"
      << "  --wall-bridge-length closes 1-cell wall gaps up to that length; the\n"
      << "    filled cells are always within half that length of a real point.\n"
      << "  --seal-mode free turns every free/unknown neighbour into occupied, so\n"
      << "    the free cell set ends up fully enclosed (free-unknown boundary 0).\n"
      << "    Sealed cells sit on the observed limit of the free space: measure\n"
      << "    them with --dump-dir plus tools/seal_audit.py if you need to know\n"
      << "    how far they are from real obstacles.\n"
      << "  --unknown-value 127 writes p=0.502; 205 writes p=0.196078, which\n"
      << "    stays unknown only while free_thresh is below 0.196.\n";
}

double parse_double(const std::string &name, const std::string &value) {
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result)) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  return result;
}

int parse_int(const std::string &name, const std::string &value) {
  std::size_t consumed = 0;
  const long result = std::stol(value, &consumed);
  if (consumed != value.size() || result < 0 || result > 100000) {
    throw std::runtime_error("Invalid value for " + name + ": " + value);
  }
  return static_cast<int>(result);
}

bool parse_bool(const std::string &name, const std::string &value) {
  if (value == "true" || value == "1" || value == "yes") return true;
  if (value == "false" || value == "0" || value == "no") return false;
  throw std::runtime_error("Invalid value for " + name + ": " + value);
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
    const std::string value = argv[++i];
    if (arg == "--input") options.input = value;
    else if (arg == "--output-prefix") options.output_prefix = value;
    else if (arg == "--resolution") options.resolution = parse_double(arg, value);
    else if (arg == "--voxel-size") options.voxel_leaf_size = parse_double(arg, value);
    else if (arg == "--cloth-resolution") options.cloth_resolution = parse_double(arg, value);
    else if (arg == "--rigidness") options.rigidness = parse_int(arg, value);
    else if (arg == "--iterations") options.iterations = parse_int(arg, value);
    else if (arg == "--classification-threshold") options.classification_threshold = parse_double(arg, value);
    else if (arg == "--time-step") options.time_step = parse_double(arg, value);
    else if (arg == "--slope-smoothing") options.slope_smoothing = parse_bool(arg, value);
    else if (arg == "--min-height") options.min_height = parse_double(arg, value);
    else if (arg == "--max-height") options.max_height = parse_double(arg, value);
    else if (arg == "--ground-reference-radius") options.ground_reference_radius = parse_double(arg, value);
    else if (arg == "--ground-reference-neighbors") options.ground_reference_neighbors = parse_int(arg, value);
    else if (arg == "--outlier-radius") options.outlier_radius = parse_double(arg, value);
    else if (arg == "--outlier-min-neighbors") options.outlier_min_neighbors = parse_int(arg, value);
    else if (arg == "--min-obstacle-neighbors") options.min_obstacle_neighbors = parse_int(arg, value);
    else if (arg == "--padding") options.padding = parse_double(arg, value);
    else if (arg == "--occupied-radius") options.occupied_radius = parse_double(arg, value);
    else if (arg == "--free-radius") options.free_radius = parse_double(arg, value);
    else if (arg == "--min-obstacle-component-cells")
      options.min_obstacle_component_cells = parse_int(arg, value);
    else if (arg == "--obstacle-cluster-radius")
      options.obstacle_cluster_radius = parse_double(arg, value);
    else if (arg == "--free-mode")
      options.free_mode = value;
    else if (arg == "--free-euclidean")
      options.free_mode = parse_bool(arg, value) ? "euclid" : "geodesic";
    else if (arg == "--free-max-range")
      options.free_max_range = parse_double(arg, value);
    else if (arg == "--free-max-offset")
      options.free_max_offset = parse_double(arg, value);
    else if (arg == "--free-islands")
      options.free_islands = value;
    else if (arg == "--free-island-wall-ratio")
      options.free_island_wall_ratio = parse_double(arg, value);
    else if (arg == "--free-island-max-area")
      options.free_island_max_area = parse_double(arg, value);
    else if (arg == "--min-free-component-cells")
      options.min_free_component_cells = parse_int(arg, value);
    else if (arg == "--wall-bridge-min-component-cells")
      options.wall_bridge_min_component_cells = parse_int(arg, value);
    else if (arg == "--wall-bridge-length")
      options.wall_bridge_length = parse_double(arg, value);
    else if (arg == "--wall-bridge-directions")
      options.wall_bridge_directions = parse_int(arg, value);
    else if (arg == "--seal-mode")
      options.seal_mode = value;
    else if (arg == "--unknown-value")
      options.unknown_value = parse_int(arg, value);
    else if (arg == "--poses")
      options.poses = value;
    else if (arg == "--dump-dir")
      options.dump_dir = value;
    else throw std::runtime_error("Unknown argument: " + arg);
  }
  if (options.input.empty() || options.output_prefix.empty()) {
    throw std::runtime_error("--input and --output-prefix are required");
  }
  return options;
}

int main(int argc, char **argv) {
  try {
    const Options options = parse_options(argc, argv);
    Cloud input;
    if (pcl::io::loadPCDFile(options.input, input) != 0) {
      throw std::runtime_error("Failed to load PCD: " + options.input);
    }

    CsfSegmentationConfig segmentation_config;
    segmentation_config.voxel_leaf_size = options.voxel_leaf_size;
    segmentation_config.cloth_resolution = options.cloth_resolution;
    segmentation_config.rigidness = options.rigidness;
    segmentation_config.iterations = options.iterations;
    segmentation_config.classification_threshold =
        options.classification_threshold;
    segmentation_config.time_step = options.time_step;
    segmentation_config.slope_smoothing = options.slope_smoothing;
    segmentation_config.obstacle_min_height = options.min_height;
    segmentation_config.obstacle_max_height = options.max_height;
    segmentation_config.ground_reference_radius =
        options.ground_reference_radius;
    segmentation_config.ground_reference_neighbors =
        options.ground_reference_neighbors;
    segmentation_config.outlier_radius = options.outlier_radius;
    segmentation_config.outlier_min_neighbors =
        options.outlier_min_neighbors;
    const auto segmented =
        bw_fastlio_nav2::segment_ground_csf(input, segmentation_config);

    GridProjectionConfig grid_config;
    grid_config.resolution = options.resolution;
    grid_config.padding = options.padding;
    grid_config.occupied_radius = options.occupied_radius;
    grid_config.free_radius = options.free_radius;
    grid_config.min_obstacle_neighbors = options.min_obstacle_neighbors;
    grid_config.min_obstacle_component_cells =
        options.min_obstacle_component_cells;
    grid_config.obstacle_cluster_radius = options.obstacle_cluster_radius;
    if (options.free_mode == "euclid") {
      grid_config.free_mode = bw_fastlio_nav2::FreeMode::kEuclid;
    } else if (options.free_mode == "geodesic") {
      grid_config.free_mode = bw_fastlio_nav2::FreeMode::kGeodesic;
    } else if (options.free_mode == "raycast") {
      grid_config.free_mode = bw_fastlio_nav2::FreeMode::kRaycast;
    } else {
      throw std::runtime_error("Invalid --free-mode: " + options.free_mode);
    }
    grid_config.free_max_range = options.free_max_range;
    grid_config.free_max_offset = options.free_max_offset;
    grid_config.free_island_wall_ratio = options.free_island_wall_ratio;
    grid_config.free_island_max_area = options.free_island_max_area;
    if (options.free_islands == "enclosed") {
      grid_config.free_islands_enclosed = true;
    } else if (options.free_islands != "off") {
      throw std::runtime_error("Invalid --free-islands: " + options.free_islands);
    }
    grid_config.min_free_component_cells = options.min_free_component_cells;
    grid_config.wall_bridge_length = options.wall_bridge_length;
    grid_config.wall_bridge_directions = options.wall_bridge_directions;
    grid_config.wall_bridge_min_component_cells =
        options.wall_bridge_min_component_cells;
    if (options.seal_mode == "free") {
      grid_config.seal_mode = bw_fastlio_nav2::SealMode::kFree;
    } else if (options.seal_mode == "off") {
      grid_config.seal_mode = bw_fastlio_nav2::SealMode::kOff;
    } else {
      throw std::runtime_error("Invalid --seal-mode: " + options.seal_mode);
    }
    std::vector<std::array<double, 2>> seeds;
    if (!options.poses.empty()) {
      std::ifstream poses(options.poses);
      if (!poses) throw std::runtime_error("Cannot open " + options.poses);
      std::string line;
      while (std::getline(poses, line)) {
        std::istringstream stream(line);
        double values[12];
        bool ok = true;
        for (double &value : values) {
          if (!(stream >> value)) {
            ok = false;
            break;
          }
        }
        if (!ok) continue;
        seeds.push_back({values[3], values[7]});
      }
      if (seeds.empty()) {
        throw std::runtime_error("No poses parsed from " + options.poses);
      }
    }
    grid_config.free_seeds = seeds;
    if (grid_config.free_mode == bw_fastlio_nav2::FreeMode::kRaycast &&
        seeds.empty()) {
      std::cerr << "WARNING: --free-mode raycast without --poses; free space falls "
                   "back to the floor returns and will be patchy.\n";
    }
    const auto grid = bw_fastlio_nav2::project_xy(
        *segmented.ground, *segmented.obstacles, grid_config);

    if (!options.dump_dir.empty()) {
      fs::create_directories(options.dump_dir);
      pcl::io::savePCDFileBinary(options.dump_dir + "/ground.pcd",
                                 *segmented.ground);
      pcl::io::savePCDFileBinary(options.dump_dir + "/obstacles.pcd",
                                 *segmented.obstacles);
    }

    fs::path prefix(options.output_prefix);
    if (prefix.has_parent_path()) fs::create_directories(prefix.parent_path());
    const fs::path pgm_path = prefix.string() + ".pgm";
    const fs::path yaml_path = prefix.string() + ".yaml";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) throw std::runtime_error("Cannot open output: " + pgm_path.string());
    pgm << "P5\n" << grid.width << " " << grid.height << "\n255\n";

    std::size_t occupied_count = 0;
    std::size_t free_count = 0;
    std::size_t unknown_count = 0;
    for (std::size_t output_y = 0; output_y < grid.height; ++output_y) {
      const std::size_t grid_y = grid.height - 1 - output_y;
      for (std::size_t x = 0; x < grid.width; ++x) {
        const std::size_t index = grid_y * grid.width + x;
        std::uint8_t value = static_cast<std::uint8_t>(options.unknown_value);
        if (grid.occupied[index] != 0) {
          value = 0;
          ++occupied_count;
        } else if (grid.observed[index] != 0) {
          value = 254;
          ++free_count;
        } else {
          ++unknown_count;
        }
        pgm.write(reinterpret_cast<const char *>(&value), 1);
      }
    }
    pgm.close();

    std::ofstream yaml(yaml_path);
    if (!yaml) throw std::runtime_error("Cannot open output: " + yaml_path.string());
    yaml << "image: " << pgm_path.filename().string() << "\n"
         << "mode: trinary\n" << std::fixed << std::setprecision(6)
         << "resolution: " << options.resolution << "\n"
         << "origin: [" << grid.origin_x << ", " << grid.origin_y
         << ", 0.0]\n"
         << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    yaml.close();

    std::cout << std::fixed << std::setprecision(3)
              << "Input/filtered points: " << input.size() << "/"
              << segmented.filtered->size() << "\n"
              << "CSF: cloth_resolution=" << options.cloth_resolution
              << " rigidness=" << options.rigidness
              << " iterations=" << options.iterations
              << " classification_threshold="
              << options.classification_threshold
              << " slope_smoothing="
              << (options.slope_smoothing ? "true" : "false") << "\n"
              << "Ground/obstacle points: " << segmented.ground->size() << "/"
              << segmented.obstacles->size() << "\n"
              << "CSF off-ground/no-local-ground-reference: "
              << segmented.off_ground_points << "/"
              << segmented.no_ground_reference_points << "\n"
              << "Grid: " << grid.width << " x " << grid.height << " at "
              << options.resolution << " m/cell\n"
              << "Obstacle components (>= "
              << options.min_obstacle_component_cells << " cells): "
              << grid.obstacle_components << "\n"
              << "Gap closing: raw_obstacle_cells=" << grid.obstacle_cells_raw
              << " bridged=" << grid.wall_bridged_cells
              << " clustered=" << grid.obstacle_clustered_cells
              << " sealed=" << grid.sealed_cells << "\n"
              << "Free space: components=" << grid.free_components
              << " dropped_small=" << grid.dropped_free_cells
              << " unreachable=" << grid.unreachable_free_cells
              << " mode=" << options.free_mode
              << " islands=" << grid.free_islands
              << " island_cells=" << grid.free_island_cells
              << " seeds=" << seeds.size()
              << "\n"
              << "Cells occupied/free/unknown: " << occupied_count << "/"
              << free_count << "/" << unknown_count << "\n"
              << "Wrote: " << pgm_path << " and " << yaml_path << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "pcd_to_nav2_map: " << error.what() << "\n";
    print_usage(argv[0]);
    return 1;
  }
}
