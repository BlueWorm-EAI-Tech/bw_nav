#pragma once

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace bw_fastlio_nav2 {

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

inline Cloud::Ptr finite_cloud(const Cloud &input) {
  Cloud::Ptr output(new Cloud);
  output->reserve(input.size());
  for (const auto &point : input) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z) && std::abs(point.x) < 1000.0F &&
        std::abs(point.y) < 1000.0F && std::abs(point.z) < 1000.0F) {
      output->push_back(point);
    }
  }
  output->width = output->size();
  output->height = 1;
  output->is_dense = true;
  return output;
}

inline Cloud::Ptr voxel_filter(const Cloud &input, double leaf_size) {
  if (leaf_size <= 0.0) {
    throw std::runtime_error("voxel leaf size must be positive");
  }
  const auto finite = finite_cloud(input);
  if (finite->size() < 100) {
    throw std::runtime_error("point cloud has fewer than 100 finite points");
  }

  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(finite);
  voxel.filter(*filtered);

  if (filtered->size() < 100) {
    throw std::runtime_error("too few points remain after voxel filtering");
  }
  return filtered;
}

inline Cloud::Ptr radius_filter(
    const Cloud::Ptr &input, double radius, int min_neighbors) {
  if (radius < 0.0 || min_neighbors < 0) {
    throw std::runtime_error("invalid radius-outlier configuration");
  }
  if (min_neighbors > 0 && radius > 0.0 && !input->empty()) {
    Cloud::Ptr output(new Cloud);
    pcl::RadiusOutlierRemoval<Point> outlier_filter;
    outlier_filter.setInputCloud(input);
    outlier_filter.setRadiusSearch(radius);
    outlier_filter.setMinNeighborsInRadius(min_neighbors);
    outlier_filter.filter(*output);
    output->width = output->size();
    output->height = 1;
    output->is_dense = true;
    return output;
  }
  return input;
}

enum class FreeMode {
  // Euclidean disk around every floor return.  Ignores obstacles; kept for
  // reproducing the old maps.
  kEuclid = 0,
  // Euclidean distance but grown only through non-obstacle cells.
  kGeodesic = 1,
  // Line of sight from the trajectory to every obstacle cell.  This is the only
  // model that produces a room-shaped free region from sparse floor returns,
  // because a single wall return frees the whole ray that reached it.
  kRaycast = 2,
};

enum class SealMode {
  // Do not touch the free boundary; unknown stays unknown.
  kOff = 0,
  // Draw a one-cell wall along the whole boundary of the free region.  This is
  // not "a wall that was observed", it is "the edge of the known world": it
  // guarantees nav2 can never plan out of the observed area.
  kFree = 1,
};

struct GridProjectionConfig {
  double resolution = 0.05;
  double padding = 1.0;
  double occupied_radius = 0.08;
  double free_radius = 0.25;
  int min_obstacle_neighbors = 2;
  // Blobs of connected obstacle area smaller than this many cells are dropped.
  // Point-level filters (min_obstacle_neighbors, outlier radius) also delete
  // isolated returns, but they cannot tell a speck from a thinly sampled wall
  // and therefore shred walls into fragments.  Filtering at the connected
  // component level keeps every wall that is long enough to matter while
  // clearing the single-cell speckle that makes a map look noisy.
  int min_obstacle_component_cells = 1;

  // ---- free space model ---------------------------------------------------
  // Euclidean disk dilation ignores obstacles, so free space leaks straight
  // through a one-cell hole in a wall and a 5 cm hole becomes a `free_radius`
  // wide corridor.  Geodesic dilation only grows through non-obstacle cells,
  // so a wall actually stops free space.
  FreeMode free_mode = FreeMode::kGeodesic;
  // Longest line of sight considered by the ray casting model, metres.
  double free_max_range = 40.0;
  // Connected free blobs smaller than this many cells are dropped.  Isolated
  // ground patches far from the trajectory turn into little free islands that
  // nothing can reach; 0 disables the filter.
  int min_free_component_cells = 0;
  // How far free space may be from the trajectory, measured *through* free
  // cells, metres.  Line of sight alone keeps painting free space through any
  // hole the map has no wall in, so a missing wall turns into a large free fan.
  // Capping the offset keeps the map to the space the robot has been able to
  // reach; 0 disables the cap.
  double free_max_offset = 0.0;
  // A room whose door was never observed gets flooded from outside and then cut
  // by the offset cap, so the room disappears although it is real space.  Keep
  // the part beyond the cap when it is mostly enclosed by real walls, and wall
  // off the connection the flood invented: the room stays free but unreachable.
  bool free_islands_enclosed = false;
  // Fraction of a candidate island's boundary that must be real wall.
  double free_island_wall_ratio = 0.5;
  // Largest island kept, m2.  A room is small; an open hall that the flood
  // spilled into is not, and keeping it would just restore the fan.
  double free_island_max_area = 25.0;

  // ---- obstacle clustering (nav map) --------------------------------------
  // Merges obstacle returns that belong to the same object cluster (chair
  // legs, desk clutter) into one solid blob.  A disk closing only fills gaps
  // narrower than 2*radius, so it cannot draw long synthetic lines between
  // unrelated objects the way directional wall bridging does.  The BBS map
  // keeps this at 0: its distance field wants the raw, thin returns.
  double obstacle_cluster_radius = 0.0;

  // ---- wall completion ----------------------------------------------------
  // Dotted walls are the normal failure mode of sparse LiDAR returns.  A 1D
  // closing along a fixed set of directions bridges the dots along the wall
  // without inflating corners the way a disk closing does.  Length is in
  // metres; 0 disables the step.
  double wall_bridge_length = 0.0;
  // 4 => horizontal/vertical/both diagonals, 8 => also the 2:1 slopes.
  int wall_bridge_directions = 4;
  // Bridges that end up in a component smaller than this many cells are treated
  // as clutter connections and removed.  0 keeps every bridge.
  int wall_bridge_min_component_cells = 0;

  // ---- sealing ------------------------------------------------------------
  SealMode seal_mode = SealMode::kOff;

  // ---- reachability -------------------------------------------------------
  // Free space the robot can actually be in.  Growing it from floor returns
  // also grows a blob around every stray floor return far away, and those blobs
  // are what makes a corridor end look like an open field.  When seeds are
  // given, only free cells connected to a seed survive: the free region is
  // flooded from the trajectory and everything the flood cannot reach is
  // dropped.  World coordinates, metres.
  std::vector<std::array<double, 2>> free_seeds;

  // ---- walked-through clearing -------------------------------------------
  // Removes obstacle cells inside the robot's swept footprint before free
  // space is grown.  A trajectory pose is, by definition, a place the chassis
  // occupied, so a return there cannot be a real obstacle: it is CSF residue
  // on the floor, a dynamic object from another pass, or a self return.  The
  // ordinary ray-cast step cannot remove it -- rays only paint free cells and
  // `occupied` always wins when the PGM is written.
  //
  // Use the footprint's inscribed radius around base_footprint, i.e. exactly
  // the number nav2 calls the robot radius; only cells the chassis covered are
  // removed, so real objects the robot merely drove past stay.  0 disables.
  double trajectory_carve_radius = 0.0;
};

struct GridProjection {
  std::size_t width = 0;
  std::size_t height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::size_t obstacle_components = 0;
  // Diagnostics for the gap-closing pipeline; all are cell counts.
  std::size_t obstacle_cells_raw = 0;
  std::size_t obstacle_clustered_cells = 0;
  std::size_t wall_bridged_cells = 0;
  std::size_t sealed_cells = 0;
  std::size_t free_components = 0;
  std::size_t dropped_free_cells = 0;
  std::size_t unreachable_free_cells = 0;
  std::size_t free_island_cells = 0;
  std::size_t free_islands = 0;
  std::size_t trajectory_carved_cells = 0;
  std::vector<std::uint8_t> occupied;
  std::vector<std::uint8_t> observed;
};

inline std::vector<std::uint8_t> dilate_mask(
    const std::vector<std::uint8_t> &input, std::size_t width,
    std::size_t height, int radius) {
  if (radius <= 0) return input;
  std::vector<std::uint8_t> output(input.size(), 0);
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      if (input[y * width + x] == 0) continue;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (dx * dx + dy * dy > radius * radius) continue;
          const long nx = static_cast<long>(x) + dx;
          const long ny = static_cast<long>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < static_cast<long>(width) &&
              ny < static_cast<long>(height)) {
            output[static_cast<std::size_t>(ny) * width +
                   static_cast<std::size_t>(nx)] = 1;
          }
        }
      }
    }
  }
  return output;
}

// Binary erosion with the same disk footprint as dilate_mask, implemented as
// the complement of a dilation of the complement.
inline std::vector<std::uint8_t> erode_mask(
    const std::vector<std::uint8_t> &input, std::size_t width,
    std::size_t height, int radius) {
  if (radius <= 0) return input;
  std::vector<std::uint8_t> inverted(input.size(), 0);
  for (std::size_t i = 0; i < input.size(); ++i) {
    inverted[i] = input[i] != 0 ? 0 : 1;
  }
  inverted = dilate_mask(inverted, width, height, radius);
  std::vector<std::uint8_t> output(input.size(), 0);
  for (std::size_t i = 0; i < input.size(); ++i) {
    output[i] = inverted[i] != 0 ? 0 : 1;
  }
  return output;
}

// Disk closing: fills gaps narrower than 2*radius and holes smaller than the
// disk, and leaves isolated objects the same size.  Unlike the directional
// line closing it cannot draw long synthetic connections between objects.
inline std::vector<std::uint8_t> close_mask(
    const std::vector<std::uint8_t> &input, std::size_t width,
    std::size_t height, int radius) {
  return erode_mask(dilate_mask(input, width, height, radius), width, height,
                    radius);
}

// Removes every 8-connected blob of occupied cells smaller than
// `min_cells` and reports how many blobs survived.  Speckle from lone
// returns and from thin structures that only clipped a cell is what makes
// a map read as "dotted"; deleting at the component level keeps walls that
// a per-point filter would have shredded.
inline std::size_t drop_small_components(std::vector<std::uint8_t> &mask,
                                         std::size_t width, std::size_t height,
                                         int min_cells) {
  if (mask.empty() || min_cells <= 1) {
    std::size_t alive = 0;
    for (const std::uint8_t cell : mask) {
      alive += cell != 0 ? 1U : 0U;
    }
    return alive;
  }
  const std::size_t threshold = static_cast<std::size_t>(min_cells);
  std::vector<std::int32_t> label(mask.size(), -1);
  std::vector<std::size_t> component_size;
  std::vector<std::size_t> stack;
  for (std::size_t start = 0; start < mask.size(); ++start) {
    if (mask[start] == 0 || label[start] >= 0) continue;
    const std::int32_t id = static_cast<std::int32_t>(component_size.size());
    component_size.push_back(0);
    label[start] = id;
    stack.push_back(start);
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      ++component_size[static_cast<std::size_t>(id)];
      const long x = static_cast<long>(index % width);
      const long y = static_cast<long>(index / width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const long nx = x + dx;
          const long ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<long>(width) ||
              ny >= static_cast<long>(height)) {
            continue;
          }
          const std::size_t neighbor = static_cast<std::size_t>(ny) * width +
                                       static_cast<std::size_t>(nx);
          if (mask[neighbor] == 0 || label[neighbor] >= 0) continue;
          label[neighbor] = id;
          stack.push_back(neighbor);
        }
      }
    }
  }
  std::size_t kept = 0;
  for (const std::size_t size : component_size) {
    kept += size >= threshold ? 1U : 0U;
  }
  for (std::size_t index = 0; index < mask.size(); ++index) {
    if (mask[index] == 0) continue;
    if (component_size[static_cast<std::size_t>(label[index])] < threshold) {
      mask[index] = 0;
    }
  }
  return kept;
}

// ---------------------------------------------------------------------------
// Gap closing helpers
// ---------------------------------------------------------------------------

// 1D dilation along a packed line: every cell within `radius` of a set cell.
inline void line_dilate(std::vector<std::uint8_t> &line, int radius) {
  if (radius <= 0 || line.empty()) return;
  std::vector<std::uint8_t> out(line.size(), 0);
  long last = -static_cast<long>(line.size()) - radius;
  for (std::size_t i = 0; i < line.size(); ++i) {
    if (line[i] != 0) last = static_cast<long>(i);
    if (static_cast<long>(i) - last <= radius) out[i] = 1;
  }
  long next = static_cast<long>(line.size()) * 2 + radius;
  for (long i = static_cast<long>(line.size()) - 1; i >= 0; --i) {
    if (line[static_cast<std::size_t>(i)] != 0) next = i;
    if (next - i <= radius) out[static_cast<std::size_t>(i)] = 1;
  }
  line.swap(out);
}

// 1D erosion = complement of a dilation of the complement.
inline void line_erode(std::vector<std::uint8_t> &line, int radius) {
  if (radius <= 0 || line.empty()) return;
  std::vector<std::uint8_t> inverted(line.size());
  for (std::size_t i = 0; i < line.size(); ++i) {
    inverted[i] = line[i] != 0 ? 0 : 1;
  }
  line_dilate(inverted, radius);
  for (std::size_t i = 0; i < line.size(); ++i) {
    line[i] = inverted[i] != 0 ? 0 : 1;
  }
}

// Closes one straight line of the mask in place (dilate then erode along the
// line only).  Used to bridge collinear dropouts without inflating corners.
inline void close_line(std::vector<std::uint8_t> &mask, std::size_t width,
                       std::size_t height, long x0, long y0, int dx, int dy,
                       int radius) {
  std::vector<std::size_t> index;
  for (long x = x0, y = y0;
       x >= 0 && y >= 0 && x < static_cast<long>(width) &&
       y < static_cast<long>(height);
       x += dx, y += dy) {
    index.push_back(static_cast<std::size_t>(y) * width +
                    static_cast<std::size_t>(x));
  }
  if (index.size() < 2) return;
  std::vector<std::uint8_t> line(index.size());
  for (std::size_t i = 0; i < index.size(); ++i) line[i] = mask[index[i]];
  line_dilate(line, radius);
  line_erode(line, radius);
  for (std::size_t i = 0; i < index.size(); ++i) {
    if (line[i] != 0) mask[index[i]] = 1;
  }
}

// Morphological closing with a 1D structuring element, applied separately along
// several directions and unioned.  This bridges collinear gaps (the usual shape
// of a thinly sampled wall) and leaves corners untouched, unlike a disk
// closing which grows every protrusion.
inline std::size_t bridge_line_gaps(std::vector<std::uint8_t> &mask,
                                    std::size_t width, std::size_t height,
                                    int radius, int directions,
                                    int min_component_cells = 0) {
  if (radius < 1 || mask.empty()) return 0;
  std::vector<std::uint8_t> accumulated = mask;
  std::vector<std::uint8_t> per_direction;
  // Horizontal.
  per_direction = mask;
  for (std::size_t y = 0; y < height; ++y) {
    close_line(per_direction, width, height, 0, static_cast<long>(y), 1, 0,
               radius);
  }
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (per_direction[i] != 0) accumulated[i] = 1;
  }
  // Vertical.
  per_direction = mask;
  for (std::size_t x = 0; x < width; ++x) {
    close_line(per_direction, width, height, static_cast<long>(x), 0, 0, 1,
               radius);
  }
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (per_direction[i] != 0) accumulated[i] = 1;
  }
  // Diagonal down-right.
  per_direction = mask;
  for (std::size_t y = 0; y < height; ++y) {
    close_line(per_direction, width, height, 0, static_cast<long>(y), 1, 1,
               radius);
  }
  for (std::size_t x = 1; x < width; ++x) {
    close_line(per_direction, width, height, static_cast<long>(x), 0, 1, 1,
               radius);
  }
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (per_direction[i] != 0) accumulated[i] = 1;
  }
  // Diagonal up-right.
  per_direction = mask;
  for (std::size_t y = 0; y < height; ++y) {
    close_line(per_direction, width, height, 0, static_cast<long>(y), 1, -1,
               radius);
  }
  for (std::size_t x = 1; x < width; ++x) {
    close_line(per_direction, width, height, static_cast<long>(x),
               static_cast<long>(height) - 1, 1, -1, radius);
  }
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (per_direction[i] != 0) accumulated[i] = 1;
  }
  if (directions >= 8) {
    // Shallow slopes (2:1).  Walls at 26.6 degrees are common in buildings and
    // a 4-direction closing leaves them dotted.
    const int slopes[4][2] = {{2, 1}, {2, -1}, {1, 2}, {-1, 2}};
    for (const auto &slope : slopes) {
      per_direction = mask;
      const long dx = slope[0];
      const long dy = slope[1];
      std::vector<std::pair<long, long>> starts;
      if (std::abs(dx) == 2) {
        for (std::size_t y = 0; y < height; ++y) {
          starts.emplace_back(dx > 0 ? 0 : static_cast<long>(width) - 1,
                              static_cast<long>(y));
        }
        for (std::size_t x = 0; x < width; ++x) {
          starts.emplace_back(static_cast<long>(x),
                              dy > 0 ? 0 : static_cast<long>(height) - 1);
        }
      } else {
        for (std::size_t x = 0; x < width; ++x) {
          starts.emplace_back(static_cast<long>(x),
                              dy > 0 ? 0 : static_cast<long>(height) - 1);
        }
        for (std::size_t y = 0; y < height; ++y) {
          starts.emplace_back(dx > 0 ? 0 : static_cast<long>(width) - 1,
                              static_cast<long>(y));
        }
      }
      for (const auto &start : starts) {
        close_line(per_direction, width, height, start.first, start.second,
                   static_cast<int>(dx), static_cast<int>(dy), radius);
      }
      for (std::size_t i = 0; i < mask.size(); ++i) {
        if (per_direction[i] != 0) accumulated[i] = 1;
      }
    }
  }
  // A closing cannot tell a dotted wall from a row of unrelated clutter dots:
  // both look like `X..X..X` along one direction, so a cluttered room turns into
  // a spider web of bridges.  The two are separable by what they join: a bridge
  // that ends up inside a small component joined nothing, while a bridge inside
  // a long component is part of a wall.  Drop bridge-only cells whose component
  // stays small; never touch a cell that came from the point cloud.
  if (min_component_cells > 0) {
    std::vector<std::int32_t> label(accumulated.size(), -1);
    std::vector<std::size_t> size;
    std::vector<std::size_t> stack;
    for (std::size_t start = 0; start < accumulated.size(); ++start) {
      if (accumulated[start] == 0 || label[start] >= 0) continue;
      const std::int32_t id = static_cast<std::int32_t>(size.size());
      size.push_back(0);
      label[start] = id;
      stack.push_back(start);
      while (!stack.empty()) {
        const std::size_t index = stack.back();
        stack.pop_back();
        ++size[static_cast<std::size_t>(id)];
        const long x = static_cast<long>(index % width);
        const long y = static_cast<long>(index / width);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const long nx = x + dx;
            const long ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<long>(width) ||
                ny >= static_cast<long>(height)) {
              continue;
            }
            const std::size_t neighbor =
                static_cast<std::size_t>(ny) * width +
                static_cast<std::size_t>(nx);
            if (accumulated[neighbor] == 0 || label[neighbor] >= 0) continue;
            label[neighbor] = id;
            stack.push_back(neighbor);
          }
        }
      }
    }
    const std::size_t threshold = static_cast<std::size_t>(min_component_cells);
    for (std::size_t i = 0; i < accumulated.size(); ++i) {
      if (accumulated[i] == 0 || mask[i] != 0) continue;
      if (size[static_cast<std::size_t>(label[i])] < threshold) {
        accumulated[i] = 0;
      }
    }
  }
  std::size_t added = 0;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (accumulated[i] != 0 && mask[i] == 0) ++added;
  }
  mask.swap(accumulated);
  return added;
}

// Geodesic (obstacle-aware) chamfer 3-4 distance to the nearest seed cell.
// Units are thirds of a cell; blocked cells are never entered.
inline std::vector<std::int32_t> geodesic_distance(
    const std::vector<std::uint8_t> &seed,
    const std::vector<std::uint8_t> &blocked, std::size_t width,
    std::size_t height) {
  constexpr std::int32_t kInf = std::numeric_limits<std::int32_t>::max() / 4;
  std::vector<std::int32_t> dist(seed.size(), kInf);
  for (std::size_t i = 0; i < seed.size(); ++i) {
    if (seed[i] != 0 && blocked[i] == 0) dist[i] = 0;
  }
  const auto at = [&](long x, long y) -> std::int32_t {
    if (x < 0 || y < 0 || x >= static_cast<long>(width) ||
        y >= static_cast<long>(height)) {
      return kInf;
    }
    return dist[static_cast<std::size_t>(y) * width +
                static_cast<std::size_t>(x)];
  };
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const std::size_t index = y * width + x;
      if (blocked[index] != 0) continue;
      std::int32_t best = dist[index];
      const long lx = static_cast<long>(x);
      const long ly = static_cast<long>(y);
      best = std::min(best, at(lx - 1, ly) + 3);
      best = std::min(best, at(lx - 1, ly - 1) + 4);
      best = std::min(best, at(lx, ly - 1) + 3);
      best = std::min(best, at(lx + 1, ly - 1) + 4);
      dist[index] = best;
    }
  }
  for (long y = static_cast<long>(height) - 1; y >= 0; --y) {
    for (long x = static_cast<long>(width) - 1; x >= 0; --x) {
      const std::size_t index = static_cast<std::size_t>(y) * width +
                                static_cast<std::size_t>(x);
      if (blocked[index] != 0) continue;
      std::int32_t best = dist[index];
      best = std::min(best, at(x + 1, y) + 3);
      best = std::min(best, at(x + 1, y + 1) + 4);
      best = std::min(best, at(x, y + 1) + 3);
      best = std::min(best, at(x - 1, y + 1) + 4);
      dist[index] = best;
    }
  }
  return dist;
}

// Removes every 8-connected blob smaller than `min_cells`, reporting how many
// survived and how many cells were removed.
inline std::size_t filter_small_components(std::vector<std::uint8_t> &mask,
                                           std::size_t width,
                                           std::size_t height, int min_cells,
                                           std::size_t *dropped_cells = nullptr,
                                           std::size_t *kept_components = nullptr) {
  std::size_t dropped = 0;
  std::size_t kept = 0;
  if (!mask.empty()) {
    const std::size_t threshold =
        min_cells <= 1 ? 1 : static_cast<std::size_t>(min_cells);
    std::vector<std::int32_t> label(mask.size(), -1);
    std::vector<std::size_t> component_size;
    std::vector<std::size_t> stack;
    for (std::size_t start = 0; start < mask.size(); ++start) {
      if (mask[start] == 0 || label[start] >= 0) continue;
      const std::int32_t id = static_cast<std::int32_t>(component_size.size());
      component_size.push_back(0);
      label[start] = id;
      stack.push_back(start);
      while (!stack.empty()) {
        const std::size_t index = stack.back();
        stack.pop_back();
        ++component_size[static_cast<std::size_t>(id)];
        const long x = static_cast<long>(index % width);
        const long y = static_cast<long>(index / width);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const long nx = x + dx;
            const long ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<long>(width) ||
                ny >= static_cast<long>(height)) {
              continue;
            }
            const std::size_t neighbor =
                static_cast<std::size_t>(ny) * width +
                static_cast<std::size_t>(nx);
            if (mask[neighbor] == 0 || label[neighbor] >= 0) continue;
            label[neighbor] = id;
            stack.push_back(neighbor);
          }
        }
      }
    }
    for (const std::size_t size : component_size) {
      if (size >= threshold) {
        ++kept;
      } else {
        dropped += size;
      }
    }
    for (std::size_t index = 0; index < mask.size(); ++index) {
      if (mask[index] == 0) continue;
      if (component_size[static_cast<std::size_t>(label[index])] < threshold) {
        mask[index] = 0;
      }
    }
  }
  if (dropped_cells != nullptr) *dropped_cells = dropped;
  if (kept_components != nullptr) *kept_components = kept;
  return kept;
}

// Free space by line of sight: every cell on a segment from a trajectory pose to
// an obstacle cell is free, and the segment stops at the first obstacle it
// meets, so a ray never carves space behind a wall.  This is the only model that
// turns sparse floor returns into a room-shaped free region: one wall return
// frees the whole ray that reached it.
inline std::vector<std::uint8_t> raycast_free(
    const std::vector<std::uint8_t> &occupied,
    const std::vector<std::uint8_t> &ground_mask,
    const std::vector<std::array<double, 2>> &seeds, std::size_t width,
    std::size_t height, double origin_x, double origin_y, double resolution,
    double max_range) {
  std::vector<std::uint8_t> free(occupied.size(), 0);
  std::vector<std::size_t> targets;
  targets.reserve(occupied.size() / 16);
  for (std::size_t i = 0; i < occupied.size(); ++i) {
    if (occupied[i] != 0) targets.push_back(i);
  }
  const double max_range_cells = max_range / resolution;
  const double max_range_squared = max_range_cells * max_range_cells;
  for (const auto &seed : seeds) {
    const long sx = static_cast<long>(
        std::floor((seed[0] - origin_x) / resolution));
    const long sy = static_cast<long>(
        std::floor((seed[1] - origin_y) / resolution));
    if (sx < 0 || sy < 0 || sx >= static_cast<long>(width) ||
        sy >= static_cast<long>(height)) {
      continue;
    }
    if (occupied[static_cast<std::size_t>(sy) * width +
                 static_cast<std::size_t>(sx)] != 0) {
      continue;
    }
    for (const std::size_t target : targets) {
      const long tx = static_cast<long>(target % width);
      const long ty = static_cast<long>(target / width);
      const double dx = static_cast<double>(tx - sx);
      const double dy = static_cast<double>(ty - sy);
      if (dx * dx + dy * dy > max_range_squared) continue;
      const long steps = static_cast<long>(
          std::max(std::abs(tx - sx), std::abs(ty - sy)));
      if (steps < 1) continue;
      const double step_x = dx / static_cast<double>(steps);
      const double step_y = dy / static_cast<double>(steps);
      bool blocked = false;
      for (long step = 0; step <= steps && !blocked; ++step) {
        const long cx =
            static_cast<long>(std::lround(static_cast<double>(sx) +
                                          step_x * static_cast<double>(step)));
        const long cy =
            static_cast<long>(std::lround(static_cast<double>(sy) +
                                          step_y * static_cast<double>(step)));
        if (cx < 0 || cy < 0 || cx >= static_cast<long>(width) ||
            cy >= static_cast<long>(height)) {
          break;
        }
        const std::size_t index = static_cast<std::size_t>(cy) * width +
                                  static_cast<std::size_t>(cx);
        if (occupied[index] != 0) {
          blocked = true;
          break;
        }
        free[index] = 1;
      }
    }
  }
  if (seeds.empty()) {
    // No trajectory: the floor returns are all we know to be free.
    for (std::size_t i = 0; i < free.size(); ++i) {
      if (ground_mask[i] != 0 && occupied[i] == 0) free[i] = 1;
    }
  }
  return free;
}

// Drops every free cell that is not 8-connected to a seed cell.  This is the
// "flood from the trajectory" test: free space the robot cannot reach is not
// free space, it is a piece of floor the sensor saw through a gap.
inline std::size_t keep_reachable(std::vector<std::uint8_t> &mask,
                                  const std::vector<std::uint8_t> &seeds,
                                  std::size_t width, std::size_t height) {
  std::vector<std::uint8_t> reached(mask.size(), 0);
  std::vector<std::size_t> stack;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (seeds[i] != 0 && mask[i] != 0) {
      reached[i] = 1;
      stack.push_back(i);
    }
  }
  if (stack.empty()) return 0;
  while (!stack.empty()) {
    const std::size_t index = stack.back();
    stack.pop_back();
    const long x = static_cast<long>(index % width);
    const long y = static_cast<long>(index / width);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const long nx = x + dx;
        const long ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<long>(width) ||
            ny >= static_cast<long>(height)) {
          continue;
        }
        const std::size_t neighbor =
            static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
        if (mask[neighbor] == 0 || reached[neighbor] != 0) continue;
        reached[neighbor] = 1;
        stack.push_back(neighbor);
      }
    }
  }
  std::size_t removed = 0;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] != 0 && reached[i] == 0) ++removed;
  }
  mask.swap(reached);
  return removed;
}

// Draws a one-cell wall on every non-free cell that touches the free region.
// Semantics: "the edge of the known world", not "an observed wall".
inline std::size_t seal_free_boundary(std::vector<std::uint8_t> &occupied,
                                      const std::vector<std::uint8_t> &free,
                                      std::size_t width, std::size_t height) {
  std::vector<std::size_t> pending;
  pending.reserve(free.size() / 8);
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const std::size_t index = y * width + x;
      if (free[index] != 0 || occupied[index] != 0) continue;
      bool touches_free = false;
      for (int dy = -1; dy <= 1 && !touches_free; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const long nx = static_cast<long>(x) + dx;
          const long ny = static_cast<long>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<long>(width) ||
              ny >= static_cast<long>(height)) {
            continue;
          }
          if (free[static_cast<std::size_t>(ny) * width +
                   static_cast<std::size_t>(nx)] != 0) {
            touches_free = true;
            break;
          }
        }
      }
      if (touches_free) pending.push_back(index);
    }
  }
  for (const std::size_t index : pending) occupied[index] = 1;
  return pending.size();
}

// Removes obstacle cells that the robot chassis physically covered along the
// trajectory.  Apart from a pose being the strongest free evidence there is,
// this is the only step that deletes obstacle cells: ray casting only fills
// free space, and occupied wins when the PGM is written.
//
// Consecutive poses are joined into capsules because keyframes are often
// ~0.5 m apart -- wider than the disk diameter -- and disk-only clearing would
// leave stripes of fake obstacle between them.  Pairs farther apart than
// link_limit are not joined, so a jump in the pose file cannot carve a long
// straight corridor across the map.
inline std::size_t carve_trajectory(
    std::vector<std::uint8_t> &mask,
    const std::vector<std::array<double, 2>> &seeds, std::size_t width,
    std::size_t height, double origin_x, double origin_y, double resolution,
    double radius) {
  if (mask.empty() || seeds.empty() || !(radius > 0.0) ||
      !(resolution > 0.0)) {
    return 0;
  }
  const double radius_squared = radius * radius;
  const double link_limit = std::max(1.0, 4.0 * radius);
  std::size_t carved = 0;

  const auto carve_capsule = [&](double ax, double ay, double bx, double by) {
    const long min_x = static_cast<long>(std::floor(
        (std::min(ax, bx) - radius - origin_x) / resolution));
    const long max_x = static_cast<long>(std::floor(
        (std::max(ax, bx) + radius - origin_x) / resolution));
    const long min_y = static_cast<long>(std::floor(
        (std::min(ay, by) - radius - origin_y) / resolution));
    const long max_y = static_cast<long>(std::floor(
        (std::max(ay, by) + radius - origin_y) / resolution));
    const double abx = bx - ax;
    const double aby = by - ay;
    const double length_squared = abx * abx + aby * aby;
    for (long y = std::max<long>(0, min_y);
         y <= std::min<long>(static_cast<long>(height) - 1, max_y); ++y) {
      for (long x = std::max<long>(0, min_x);
           x <= std::min<long>(static_cast<long>(width) - 1, max_x); ++x) {
        const double px =
            origin_x + (static_cast<double>(x) + 0.5) * resolution;
        const double py =
            origin_y + (static_cast<double>(y) + 0.5) * resolution;
        double t = 0.0;
        if (length_squared > 0.0) {
          t = ((px - ax) * abx + (py - ay) * aby) / length_squared;
          t = std::clamp(t, 0.0, 1.0);
        }
        const double dx = px - (ax + t * abx);
        const double dy = py - (ay + t * aby);
        if (dx * dx + dy * dy > radius_squared) continue;
        std::uint8_t &cell =
            mask[static_cast<std::size_t>(y) * width +
                 static_cast<std::size_t>(x)];
        if (cell != 0) {
          cell = 0;
          ++carved;
        }
      }
    }
  };

  carve_capsule(seeds[0][0], seeds[0][1], seeds[0][0], seeds[0][1]);
  for (std::size_t i = 1; i < seeds.size(); ++i) {
    const double ax = seeds[i - 1][0];
    const double ay = seeds[i - 1][1];
    const double bx = seeds[i][0];
    const double by = seeds[i][1];
    const double dx = bx - ax;
    const double dy = by - ay;
    if (dx * dx + dy * dy <= link_limit * link_limit) {
      carve_capsule(ax, ay, bx, by);
    } else {
      carve_capsule(bx, by, bx, by);
    }
  }
  return carved;
}

inline GridProjection project_xy(
    const Cloud &ground, const Cloud &obstacles,
    const GridProjectionConfig &config) {
  if (config.resolution <= 0.0 || config.resolution > 1.0 ||
      config.padding < 0.0 || config.occupied_radius < 0.0 ||
      config.free_radius < 0.0 || config.min_obstacle_neighbors < 1 ||
      config.min_obstacle_component_cells < 1 ||
      config.min_free_component_cells < 0 ||
      config.obstacle_cluster_radius < 0.0 ||
      config.trajectory_carve_radius < 0.0 ||
      config.wall_bridge_length < 0.0 ||
      (config.wall_bridge_directions != 4 &&
       config.wall_bridge_directions != 8)) {
    throw std::runtime_error("invalid grid-projection configuration");
  }
  if (ground.empty() && obstacles.empty()) {
    throw std::runtime_error("cannot project two empty point clouds");
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  const auto extend_bounds = [&](const Cloud &cloud) {
    for (const auto &point : cloud) {
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
  };
  extend_bounds(ground);
  extend_bounds(obstacles);

  GridProjection grid;
  grid.origin_x = min_x - config.padding;
  grid.origin_y = min_y - config.padding;
  max_x += config.padding;
  max_y += config.padding;
  grid.width = static_cast<std::size_t>(
                   std::ceil((max_x - grid.origin_x) / config.resolution)) +
               1;
  grid.height = static_cast<std::size_t>(
                    std::ceil((max_y - grid.origin_y) / config.resolution)) +
                1;
  if (grid.width == 0 || grid.height == 0 || grid.width > 20000 ||
      grid.height > 20000 || grid.width > 100000000ULL / grid.height) {
    throw std::runtime_error("projected map bounds are too large");
  }

  std::vector<std::uint8_t> ground_mask(grid.width * grid.height, 0);
  std::vector<std::uint16_t> obstacle_counts(grid.width * grid.height, 0);
  const auto cell_index = [&](const Point &point) -> std::optional<std::size_t> {
    const long x = static_cast<long>(
        std::floor((point.x - grid.origin_x) / config.resolution));
    const long y = static_cast<long>(
        std::floor((point.y - grid.origin_y) / config.resolution));
    if (x < 0 || y < 0 || x >= static_cast<long>(grid.width) ||
        y >= static_cast<long>(grid.height)) {
      return std::nullopt;
    }
    return static_cast<std::size_t>(y) * grid.width +
           static_cast<std::size_t>(x);
  };
  for (const auto &point : ground) {
    if (const auto index = cell_index(point)) ground_mask[*index] = 1;
  }
  for (const auto &point : obstacles) {
    if (const auto index = cell_index(point)) {
      obstacle_counts[*index] = std::min<std::uint16_t>(
          std::numeric_limits<std::uint16_t>::max(),
          static_cast<std::uint16_t>(obstacle_counts[*index] + 1));
    }
  }

  std::vector<std::uint8_t> obstacle_mask(grid.width * grid.height, 0);
  for (std::size_t y = 0; y < grid.height; ++y) {
    for (std::size_t x = 0; x < grid.width; ++x) {
      const std::size_t index = y * grid.width + x;
      if (obstacle_counts[index] == 0) continue;
      int neighbor_count = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const long nx = static_cast<long>(x) + dx;
          const long ny = static_cast<long>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < static_cast<long>(grid.width) &&
              ny < static_cast<long>(grid.height) &&
              obstacle_counts[static_cast<std::size_t>(ny) * grid.width +
                              static_cast<std::size_t>(nx)] > 0) {
            ++neighbor_count;
          }
        }
      }
      if (neighbor_count >= config.min_obstacle_neighbors) {
        obstacle_mask[index] = 1;
      }
    }
  }

  grid.obstacle_cells_raw = 0;
  for (const std::uint8_t cell : obstacle_mask) {
    grid.obstacle_cells_raw += cell != 0 ? 1U : 0U;
  }
  if (config.wall_bridge_length > 0.0) {
    const int radius = static_cast<int>(
        std::ceil(0.5 * config.wall_bridge_length / config.resolution));
    grid.wall_bridged_cells = bridge_line_gaps(
        obstacle_mask, grid.width, grid.height, radius,
        config.wall_bridge_directions,
        config.wall_bridge_min_component_cells);
  }
  if (config.obstacle_cluster_radius > 0.0) {
    // Group nearby returns of the same object into one blob.  Closing adds
    // only cells that are within `radius` of the obstacle, so the result stays
    // local to the cluster.
    const int radius = static_cast<int>(
        std::ceil(config.obstacle_cluster_radius / config.resolution));
    const std::vector<std::uint8_t> before = obstacle_mask;
    obstacle_mask = close_mask(obstacle_mask, grid.width, grid.height, radius);
    for (std::size_t i = 0; i < obstacle_mask.size(); ++i) {
      if (obstacle_mask[i] != 0 && before[i] == 0) {
        ++grid.obstacle_clustered_cells;
      }
    }
  }

  // Clear the walked corridor before dilation so clustering cannot refill it;
  // the component filter then sees the shortened blobs too.  With
  // occupied_radius > 0 the dilation below could still push a surviving
  // obstacle back into the corridor, hence the second pass after the filter.
  grid.trajectory_carved_cells = carve_trajectory(
      obstacle_mask, config.free_seeds, grid.width, grid.height,
      grid.origin_x, grid.origin_y, config.resolution,
      config.trajectory_carve_radius);

  grid.occupied = dilate_mask(
      obstacle_mask, grid.width, grid.height,
      static_cast<int>(std::ceil(config.occupied_radius / config.resolution)));
  grid.obstacle_components =
      drop_small_components(grid.occupied, grid.width, grid.height,
                            config.min_obstacle_component_cells);
  grid.trajectory_carved_cells += carve_trajectory(
      grid.occupied, config.free_seeds, grid.width, grid.height,
      grid.origin_x, grid.origin_y, config.resolution,
      config.trajectory_carve_radius);

  const int free_cells =
      static_cast<int>(std::ceil(config.free_radius / config.resolution));
  if (config.free_mode == FreeMode::kRaycast) {
    grid.observed = raycast_free(grid.occupied, ground_mask, config.free_seeds,
                                 grid.width, grid.height, grid.origin_x,
                                 grid.origin_y, config.resolution,
                                 config.free_max_range);
    // A small margin so the free region is not a one-cell line where the ray
    // skimmed a wall, still bounded by the walls themselves.
    const std::vector<std::int32_t> dist = geodesic_distance(
        grid.observed, grid.occupied, grid.width, grid.height);
    const std::int32_t limit = static_cast<std::int32_t>(free_cells) * 3;
    for (std::size_t i = 0; i < grid.observed.size(); ++i) {
      if (grid.occupied[i] == 0 && dist[i] <= limit) grid.observed[i] = 1;
    }
    if (config.free_max_offset > 0.0 && !config.free_seeds.empty()) {
      // How far the robot could actually get: a geodesic cap around the seeds.
      std::vector<std::uint8_t> seed_cells(grid.observed.size(), 0);
      for (const auto &seed : config.free_seeds) {
        const long sx = static_cast<long>(
            std::floor((seed[0] - grid.origin_x) / config.resolution));
        const long sy = static_cast<long>(
            std::floor((seed[1] - grid.origin_y) / config.resolution));
        if (sx < 0 || sy < 0 || sx >= static_cast<long>(grid.width) ||
            sy >= static_cast<long>(grid.height)) {
          continue;
        }
        const std::size_t index = static_cast<std::size_t>(sy) * grid.width +
                                  static_cast<std::size_t>(sx);
        if (grid.occupied[index] == 0) seed_cells[index] = 1;
      }
      const std::vector<std::int32_t> reach = geodesic_distance(
          seed_cells, grid.occupied, grid.width, grid.height);
      const std::int32_t cap = static_cast<std::int32_t>(
          std::lround(config.free_max_offset / config.resolution * 3.0));
      std::vector<std::uint8_t> keep(grid.observed.size(), 0);
      for (std::size_t i = 0; i < grid.observed.size(); ++i) {
        if (grid.observed[i] != 0 && reach[i] <= cap) keep[i] = 1;
      }
      // A flood that reaches a real room through a door nobody observed makes
      // the room look like a corridor extension.  Regions beyond the cap that
      // are mostly walled in are kept as free islands; the invented doorway
      // between them and the reachable space is then walled off.
      std::vector<std::uint8_t> island(grid.observed.size(), 0);
      if (config.free_islands_enclosed) {
        std::vector<std::uint8_t> seen(grid.observed.size(), 0);
        std::vector<std::size_t> stack;
        std::vector<std::size_t> component;
        const std::size_t min_island_cells = static_cast<std::size_t>(
            std::ceil(1.0 / (config.resolution * config.resolution)));
        for (std::size_t root = 0; root < grid.observed.size(); ++root) {
          if (grid.observed[root] == 0 || keep[root] != 0 || seen[root] != 0) {
            continue;
          }
          component.clear();
          stack.clear();
          stack.push_back(root);
          seen[root] = 1;
          std::size_t wall_touch = 0;
          std::size_t open_touch = 0;
          while (!stack.empty()) {
            const std::size_t cell = stack.back();
            stack.pop_back();
            component.push_back(cell);
            const long cx = static_cast<long>(cell % grid.width);
            const long cy = static_cast<long>(cell / grid.width);
            for (long dy = -1; dy <= 1; ++dy) {
              for (long dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                const long nx = cx + dx;
                const long ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= static_cast<long>(grid.width) ||
                    ny >= static_cast<long>(grid.height)) {
                  ++open_touch;
                  continue;
                }
                const std::size_t nb = static_cast<std::size_t>(ny) * grid.width +
                                       static_cast<std::size_t>(nx);
                if (grid.observed[nb] != 0) {
                  if (keep[nb] == 0 && seen[nb] == 0) {
                    seen[nb] = 1;
                    stack.push_back(nb);
                  }
                  continue;
                }
                if (grid.occupied[nb] != 0) {
                  ++wall_touch;
                } else {
                  ++open_touch;
                }
              }
            }
          }
          const std::size_t total = wall_touch + open_touch;
          if (component.size() < min_island_cells || total == 0) continue;
          if (config.free_island_max_area > 0.0 &&
              static_cast<double>(component.size()) * config.resolution *
                      config.resolution >
                  config.free_island_max_area) {
            continue;
          }
          if (static_cast<double>(wall_touch) / static_cast<double>(total) <
              config.free_island_wall_ratio) {
            continue;
          }
          for (const std::size_t cell : component) island[cell] = 1;
          ++grid.free_islands;
        }
      }
      std::vector<std::uint8_t> doorway(grid.observed.size(), 0);
      for (std::size_t i = 0; i < grid.observed.size(); ++i) {
        if (island[i] == 0) continue;
        ++grid.free_island_cells;
        const long cx = static_cast<long>(i % grid.width);
        const long cy = static_cast<long>(i / grid.width);
        for (long dy = -1; dy <= 1; ++dy) {
          for (long dx = -1; dx <= 1; ++dx) {
            const long nx = cx + dx;
            const long ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<long>(grid.width) ||
                ny >= static_cast<long>(grid.height)) {
              continue;
            }
            const std::size_t nb = static_cast<std::size_t>(ny) * grid.width +
                                   static_cast<std::size_t>(nx);
            if (keep[nb] != 0 && island[nb] == 0) doorway[nb] = 1;
          }
        }
      }
      for (std::size_t i = 0; i < grid.observed.size(); ++i) {
        if (doorway[i] != 0) {
          grid.occupied[i] = 1;
          grid.observed[i] = 0;
        } else if (grid.observed[i] != 0 && keep[i] == 0 && island[i] == 0) {
          grid.observed[i] = 0;
        }
      }
    }
  } else if (config.free_mode == FreeMode::kGeodesic) {
    // Grow free space only through non-obstacle cells so a wall actually stops
    // it.  A Euclidean disk ignores the wall and turns every one-cell hole into
    // a `free_radius` wide passage.
    const std::vector<std::int32_t> dist = geodesic_distance(
        ground_mask, grid.occupied, grid.width, grid.height);
    const std::int32_t limit = static_cast<std::int32_t>(free_cells) * 3;
    grid.observed.assign(grid.width * grid.height, 0);
    for (std::size_t i = 0; i < grid.observed.size(); ++i) {
      if (grid.occupied[i] == 0 && dist[i] <= limit) grid.observed[i] = 1;
    }
  } else {
    grid.observed = dilate_mask(ground_mask, grid.width, grid.height,
                                free_cells);
  }
  filter_small_components(grid.observed, grid.width, grid.height,
                          config.min_free_component_cells,
                          &grid.dropped_free_cells, &grid.free_components);

  if (config.free_mode != FreeMode::kRaycast &&
      !config.free_seeds.empty() && !grid.observed.empty()) {
    std::vector<std::uint8_t> seeds(grid.width * grid.height, 0);
    std::size_t planted = 0;
    const int search_radius =
        static_cast<int>(std::ceil(0.5 / config.resolution));
    for (const auto &seed : config.free_seeds) {
      const long sx = static_cast<long>(
          std::floor((seed[0] - grid.origin_x) / config.resolution));
      const long sy = static_cast<long>(
          std::floor((seed[1] - grid.origin_y) / config.resolution));
      bool found = false;
      for (int radius = 0; radius <= search_radius && !found; ++radius) {
        for (int dy = -radius; dy <= radius && !found; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const long nx = sx + dx;
            const long ny = sy + dy;
            if (nx < 0 || ny < 0 || nx >= static_cast<long>(grid.width) ||
                ny >= static_cast<long>(grid.height)) {
              continue;
            }
            const std::size_t index = static_cast<std::size_t>(ny) *
                                          grid.width +
                                      static_cast<std::size_t>(nx);
            if (grid.observed[index] != 0) {
              seeds[index] = 1;
              found = true;
              ++planted;
              break;
            }
          }
        }
      }
    }
    if (planted > 0) {
      grid.unreachable_free_cells =
          keep_reachable(grid.observed, seeds, grid.width, grid.height);
    }
  }

  if (config.seal_mode == SealMode::kFree) {
    grid.sealed_cells = seal_free_boundary(grid.occupied, grid.observed,
                                           grid.width, grid.height);
  }
  return grid;
}

}  // namespace bw_fastlio_nav2
