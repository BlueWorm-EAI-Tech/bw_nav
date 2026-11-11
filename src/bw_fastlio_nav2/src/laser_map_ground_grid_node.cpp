#include "bw_fastlio_nav2/fov_filter.hpp"
#include "bw_fastlio_nav2/patchwork_ground_segmentation.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

namespace bw_fastlio_nav2 {

class LaserMapGroundGridNode : public rclcpp::Node {
 public:
  LaserMapGroundGridNode() : Node("laser_map_ground_grid") {
    declare_parameter<std::string>("input_topic", "/Laser_map");
    declare_parameter<std::string>("grid_topic", "/obstacle_grid");
    declare_parameter<std::string>("ground_topic", "/ground_points");
    declare_parameter<std::string>("obstacle_topic", "/obstacle_points");
    declare_parameter<double>("voxel_leaf_size", 0.08);
    declare_parameter<double>("sensor_height", 0.35);
    declare_parameter<int>("num_iter", 3);
    declare_parameter<int>("num_lpr", 20);
    declare_parameter<int>("num_min_points", 10);
    declare_parameter<double>("seed_threshold", 0.12);
    declare_parameter<double>("distance_threshold", 0.08);
    declare_parameter<double>("vertical_seed_threshold", 0.25);
    declare_parameter<double>("vertical_distance_threshold", 0.10);
    declare_parameter<double>("min_range", 0.05);
    declare_parameter<double>("max_range", 15.0);
    declare_parameter<double>("uprightness_threshold", 0.866);
    declare_parameter<bool>("enable_reflected_noise_removal", false);
    declare_parameter<bool>("enable_vertical_plane_fitting", true);
    declare_parameter<bool>("enable_temporal_ground_revert", true);
    declare_parameter<double>("obstacle_min_height", 0.15);
    declare_parameter<double>("obstacle_max_height", 1.80);
    declare_parameter<double>("outlier_radius", 0.20);
    declare_parameter<int>("outlier_min_neighbors", 2);
    declare_parameter<double>("resolution", 0.05);
    declare_parameter<double>("padding", 1.0);
    declare_parameter<double>("occupied_radius", 0.08);
    declare_parameter<double>("free_radius", 0.25);
    declare_parameter<int>("min_obstacle_neighbors", 2);
    declare_parameter<double>("min_update_period_sec", 0.5);
    // The upstream cloud carries the LiDAR clock, which is not aligned with the
    // host clock. Consumers such as nav2_collision_monitor compare the header
    // stamp against their own now() and silently drop the source when the
    // offset exceeds source_timeout, so publish with the receive time instead.
    declare_parameter<bool>("restamp_output_to_now", true);
    // The FOV copy is consumed by the two costmaps, whose layers resolve their
    // transforms with tf2's MessageFilter at the *message* stamp. FAST-LIO
    // stamps camera_init->body with lidar_end_time, so the cloud must stay on
    // the LiDAR clock: a host-clock restamp is ~18 h ahead of the TF cache and
    // every message is dropped before it reaches the voxel/obstacle layer.
    // Only /obstacle_points_live (collision_monitor, which compares the stamp
    // against now()) needs host time. This knob exists for a host-clock
    // consumer of the FOV topic; it must stay false for the costmaps.
    declare_parameter<bool>("restamp_fov_output_to_now", false);
    declare_parameter<bool>("log_stamp_delay", true);
    declare_parameter<double>("stamp_delay_log_period_sec", 1.0);
    declare_parameter<bool>("self_filter_enabled", false);
    declare_parameter<double>("self_filter_min_x", -0.46);
    declare_parameter<double>("self_filter_max_x", 0.26);
    declare_parameter<double>("self_filter_min_y", -0.34);
    declare_parameter<double>("self_filter_max_y", 0.34);
    // Second-stage field-of-view gate. It runs on the already ground- and
    // self-filtered obstacle cloud and publishes on its own topic, so the
    // original topics keep their unmodified content.
    declare_parameter<bool>("fov_filter_enabled", true);
    declare_parameter<std::string>("fov_obstacle_topic",
                                   "/obstacle_points_fov");
    declare_parameter<double>("fov_min_azimuth_deg", -135.0);
    declare_parameter<double>("fov_max_azimuth_deg", 135.0);
    declare_parameter<double>("fov_min_elevation_deg", -90.0);
    declare_parameter<double>("fov_max_elevation_deg", 0.0);
    declare_parameter<double>("fov_min_range", 0.05);
    declare_parameter<double>("fov_max_range", 15.0);
    declare_parameter<double>("fov_mount_roll_deg", 0.0);
    declare_parameter<double>("fov_mount_pitch_deg", 0.0);
    declare_parameter<double>("fov_mount_yaw_deg", 0.0);

    config_.voxel_leaf_size = get_parameter("voxel_leaf_size").as_double();
    config_.sensor_height = get_parameter("sensor_height").as_double();
    config_.num_iter = static_cast<int>(get_parameter("num_iter").as_int());
    config_.num_lpr = static_cast<int>(get_parameter("num_lpr").as_int());
    config_.num_min_points =
        static_cast<int>(get_parameter("num_min_points").as_int());
    config_.seed_threshold = get_parameter("seed_threshold").as_double();
    config_.distance_threshold =
        get_parameter("distance_threshold").as_double();
    config_.vertical_seed_threshold =
        get_parameter("vertical_seed_threshold").as_double();
    config_.vertical_distance_threshold =
        get_parameter("vertical_distance_threshold").as_double();
    config_.min_range = get_parameter("min_range").as_double();
    config_.max_range = get_parameter("max_range").as_double();
    config_.uprightness_threshold =
        get_parameter("uprightness_threshold").as_double();
    config_.enable_reflected_noise_removal =
        get_parameter("enable_reflected_noise_removal").as_bool();
    config_.enable_vertical_plane_fitting =
        get_parameter("enable_vertical_plane_fitting").as_bool();
    config_.enable_temporal_ground_revert =
        get_parameter("enable_temporal_ground_revert").as_bool();
    config_.obstacle_min_height =
        get_parameter("obstacle_min_height").as_double();
    config_.obstacle_max_height =
        get_parameter("obstacle_max_height").as_double();
    config_.outlier_radius = get_parameter("outlier_radius").as_double();
    config_.outlier_min_neighbors =
        static_cast<int>(get_parameter("outlier_min_neighbors").as_int());
    grid_config_.resolution = get_parameter("resolution").as_double();
    grid_config_.padding = get_parameter("padding").as_double();
    grid_config_.occupied_radius =
        get_parameter("occupied_radius").as_double();
    grid_config_.free_radius = get_parameter("free_radius").as_double();
    grid_config_.min_obstacle_neighbors =
        static_cast<int>(get_parameter("min_obstacle_neighbors").as_int());
    min_update_period_ = rclcpp::Duration::from_seconds(
        get_parameter("min_update_period_sec").as_double());
    restamp_output_to_now_ = get_parameter("restamp_output_to_now").as_bool();
    restamp_fov_output_to_now_ =
        get_parameter("restamp_fov_output_to_now").as_bool();
    log_stamp_delay_ = get_parameter("log_stamp_delay").as_bool();
    stamp_delay_log_period_sec_ = std::max(
        0.1, get_parameter("stamp_delay_log_period_sec").as_double());
    self_filter_enabled_ = get_parameter("self_filter_enabled").as_bool();
    self_filter_min_x_ = get_parameter("self_filter_min_x").as_double();
    self_filter_max_x_ = get_parameter("self_filter_max_x").as_double();
    self_filter_min_y_ = get_parameter("self_filter_min_y").as_double();
    self_filter_max_y_ = get_parameter("self_filter_max_y").as_double();
    if (self_filter_enabled_ &&
        (self_filter_min_x_ >= self_filter_max_x_ ||
         self_filter_min_y_ >= self_filter_max_y_)) {
      throw std::runtime_error("invalid self-filter bounds");
    }
    fov_config_.enabled = get_parameter("fov_filter_enabled").as_bool();
    fov_config_.min_azimuth_deg =
        get_parameter("fov_min_azimuth_deg").as_double();
    fov_config_.max_azimuth_deg =
        get_parameter("fov_max_azimuth_deg").as_double();
    fov_config_.min_elevation_deg =
        get_parameter("fov_min_elevation_deg").as_double();
    fov_config_.max_elevation_deg =
        get_parameter("fov_max_elevation_deg").as_double();
    fov_config_.min_range = get_parameter("fov_min_range").as_double();
    fov_config_.max_range = get_parameter("fov_max_range").as_double();
    fov_config_.mount_roll_deg =
        get_parameter("fov_mount_roll_deg").as_double();
    fov_config_.mount_pitch_deg =
        get_parameter("fov_mount_pitch_deg").as_double();
    fov_config_.mount_yaw_deg =
        get_parameter("fov_mount_yaw_deg").as_double();
    validate_fov_config(fov_config_);
    validate_patchwork_config(config_);
    patchwork_ = std::make_unique<patchwork::PatchWorkpp>(
        make_patchwork_params(config_));

    const auto grid_qos = rclcpp::QoS(1).reliable().transient_local();
    grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        get_parameter("grid_topic").as_string(), grid_qos);
    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        get_parameter("ground_topic").as_string(), 2);
    obstacle_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        get_parameter("obstacle_topic").as_string(), 2);
    if (fov_config_.enabled) {
      fov_obstacle_publisher_ =
          create_publisher<sensor_msgs::msg::PointCloud2>(
              get_parameter("fov_obstacle_topic").as_string(), 2);
    }
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        get_parameter("input_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&LaserMapGroundGridNode::cloud_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Patchwork++ ground grid ready: %s -> %s (sensor_height=%.3f m, "
        "range=[%.2f, %.2f] m, %.3f m/cell)",
        get_parameter("input_topic").as_string().c_str(),
        get_parameter("grid_topic").as_string().c_str(),
        config_.sensor_height, config_.min_range, config_.max_range,
        grid_config_.resolution);
    if (self_filter_enabled_) {
      RCLCPP_INFO(
          get_logger(),
          "Self-filter enabled in input frame: x=[%.2f, %.2f] y=[%.2f, %.2f] m",
          self_filter_min_x_, self_filter_max_x_, self_filter_min_y_,
          self_filter_max_y_);
    }
    if (fov_config_.enabled) {
      RCLCPP_INFO(
          get_logger(),
          "FOV filter: azimuth=[%.1f, %.1f] deg elevation=[%.1f, %.1f] deg "
          "range=[%.2f, %.2f] m mount rpy=[%.1f, %.1f, %.1f] deg -> %s",
          fov_config_.min_azimuth_deg, fov_config_.max_azimuth_deg,
          fov_config_.min_elevation_deg, fov_config_.max_elevation_deg,
          fov_config_.min_range, fov_config_.max_range,
          fov_config_.mount_roll_deg, fov_config_.mount_pitch_deg,
          fov_config_.mount_yaw_deg,
          get_parameter("fov_obstacle_topic").as_string().c_str());
    } else {
      RCLCPP_WARN(
          get_logger(),
          "FOV filter disabled: %s is not published",
          get_parameter("fov_obstacle_topic").as_string().c_str());
    }
  }

 private:
  void publish_cloud(
      const Cloud &cloud, const std_msgs::msg::Header &header,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
          &publisher) const {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = header;
    publisher->publish(output);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    // Throttle on the receive time: the upstream stamp belongs to the LiDAR
    // clock, and a stamp that jumps backwards must not stall the node forever.
    const rclcpp::Time receive_time = now();
    if (last_processed_time_.nanoseconds() != 0 &&
        receive_time - last_processed_time_ < min_update_period_) {
      return;
    }
    last_processed_time_ = receive_time;

    // A constant offset means the LiDAR clock is simply unaligned with the
    // host clock; a growing offset means the two clocks drift apart.
    const rclcpp::Time cloud_stamp(message->header.stamp);
    const double stamp_delay_sec = (receive_time - cloud_stamp).seconds();
    track_stamp_delay(receive_time, stamp_delay_sec, message->header.frame_id);

    std_msgs::msg::Header output_header = message->header;
    if (restamp_output_to_now_) {
      output_header.stamp = receive_time;
    }
    // The costmap consumer needs the LiDAR stamp; see the parameter comment.
    std_msgs::msg::Header fov_header = message->header;
    if (restamp_fov_output_to_now_) {
      fov_header.stamp = receive_time;
    }

    try {
      Cloud input;
      pcl::fromROSMsg(*message, input);
      Cloud segmentation_input;
      segmentation_input.reserve(input.size());
      std::size_t self_points_removed = 0;
      for (const auto &point : input) {
        if (self_filter_enabled_) {
          const bool inside_robot =
              point.x >= self_filter_min_x_ && point.x <= self_filter_max_x_ &&
              point.y >= self_filter_min_y_ && point.y <= self_filter_max_y_;
          if (inside_robot) {
            ++self_points_removed;
            continue;
          }
        }
        segmentation_input.push_back(point);
      }
      segmentation_input.width = segmentation_input.size();
      segmentation_input.height = 1;
      segmentation_input.is_dense = input.is_dense;
      const auto segmented =
          segment_ground_patchwork(segmentation_input, *patchwork_, config_);
      const auto projected =
          project_xy(*segmented.ground, *segmented.obstacles, grid_config_);

      publish_cloud(*segmented.ground, output_header, ground_publisher_);
      publish_cloud(*segmented.obstacles, output_header, obstacle_publisher_);

      std::size_t fov_points_removed = 0;
      std::size_t fov_points_kept = 0;
      if (fov_config_.enabled) {
        const Cloud::Ptr fov_obstacles = filter_cloud_by_fov(
            *segmented.obstacles, fov_config_, fov_points_removed);
        fov_points_kept = fov_obstacles->size();
        publish_cloud(*fov_obstacles, fov_header, fov_obstacle_publisher_);
      }

      nav_msgs::msg::OccupancyGrid grid;
      grid.header = output_header;
      grid.info.map_load_time = now();
      grid.info.resolution = static_cast<float>(grid_config_.resolution);
      grid.info.width = static_cast<std::uint32_t>(projected.width);
      grid.info.height = static_cast<std::uint32_t>(projected.height);
      grid.info.origin.position.x = projected.origin_x;
      grid.info.origin.position.y = projected.origin_y;
      grid.info.origin.orientation.w = 1.0;
      grid.data.assign(projected.width * projected.height, -1);
      std::size_t occupied_count = 0;
      std::size_t free_count = 0;
      for (std::size_t i = 0; i < grid.data.size(); ++i) {
        if (projected.occupied[i] != 0) {
          grid.data[i] = 100;
          ++occupied_count;
        } else if (projected.observed[i] != 0) {
          grid.data[i] = 0;
          ++free_count;
        }
      }
      grid_publisher_->publish(grid);

      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Patchwork++ %.2f ms: filtered=%zu ground=%zu obstacles=%zu "
          "self_removed=%zu range_removed=%zu fov_kept=%zu fov_removed=%zu "
          "grid=%ux%u occupied/free=%zu/%zu",
          segmented.algorithm_time_ms,
          segmented.filtered->size(), segmented.ground->size(),
          segmented.obstacles->size(), self_points_removed,
          segmented.range_filtered_points, fov_points_kept,
          fov_points_removed, grid.info.width, grid.info.height,
          occupied_count, free_count);
    } catch (const std::exception &error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Skipping Patchwork++ update: %s", error.what());
    }
  }

  // Keeps a one-line summary of how far the upstream stamp is from the wall
  // clock. Deliberately independent of the restamping so the delay can still be
  // observed when the output is republished with now().
  void track_stamp_delay(
      const rclcpp::Time &receive_time, double delay_sec,
      const std::string &frame_id) {
    if (!log_stamp_delay_) {
      return;
    }
    if (stamp_delay_count_ == 0) {
      stamp_delay_min_ = delay_sec;
      stamp_delay_max_ = delay_sec;
    } else {
      stamp_delay_min_ = std::min(stamp_delay_min_, delay_sec);
      stamp_delay_max_ = std::max(stamp_delay_max_, delay_sec);
    }
    stamp_delay_sum_ += delay_sec;
    ++stamp_delay_count_;

    if (has_previous_stamp_delay_ &&
        std::abs(delay_sec - previous_stamp_delay_sec_) > 0.25) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Cloud stamp delay jumped by %+.3f s (%.3f -> %.3f s); the LiDAR "
          "clock moved or the driver restarted",
          delay_sec - previous_stamp_delay_sec_, previous_stamp_delay_sec_,
          delay_sec);
    }
    previous_stamp_delay_sec_ = delay_sec;
    has_previous_stamp_delay_ = true;

    if (last_stamp_delay_log_time_.nanoseconds() == 0 ||
        receive_time - last_stamp_delay_log_time_ >=
            rclcpp::Duration::from_seconds(stamp_delay_log_period_sec_)) {
      const double average =
          stamp_delay_sum_ / static_cast<double>(stamp_delay_count_);
      RCLCPP_INFO(
          get_logger(),
          "Cloud stamp delay: latest=%+.3f s avg=%+.3f s min=%+.3f s "
          "max=%+.3f s over %zu frames (frame=%s, restamp=%s, fov_restamp=%s)",
          delay_sec, average, stamp_delay_min_, stamp_delay_max_,
          stamp_delay_count_, frame_id.c_str(),
          restamp_output_to_now_ ? "on" : "off",
          restamp_fov_output_to_now_ ? "on" : "off");
      last_stamp_delay_log_time_ = receive_time;
      stamp_delay_count_ = 0;
      stamp_delay_sum_ = 0.0;
      stamp_delay_min_ = 0.0;
      stamp_delay_max_ = 0.0;
    }
  }

  PatchworkSegmentationConfig config_;
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
  GridProjectionConfig grid_config_;
  bool self_filter_enabled_{false};
  double self_filter_min_x_{-0.46};
  double self_filter_max_x_{0.26};
  double self_filter_min_y_{-0.34};
  double self_filter_max_y_{0.34};
  FovFilterConfig fov_config_;
  rclcpp::Duration min_update_period_{0, 0};
  bool restamp_output_to_now_{true};
  bool restamp_fov_output_to_now_{false};
  bool log_stamp_delay_{true};
  double stamp_delay_log_period_sec_{1.0};
  rclcpp::Time last_processed_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stamp_delay_log_time_{0, 0, RCL_ROS_TIME};
  double stamp_delay_min_{0.0};
  double stamp_delay_max_{0.0};
  double stamp_delay_sum_{0.0};
  std::size_t stamp_delay_count_{0};
  double previous_stamp_delay_sec_{0.0};
  bool has_previous_stamp_delay_{false};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      fov_obstacle_publisher_;
};

}  // namespace bw_fastlio_nav2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<bw_fastlio_nav2::LaserMapGroundGridNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("laser_map_ground_grid"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
