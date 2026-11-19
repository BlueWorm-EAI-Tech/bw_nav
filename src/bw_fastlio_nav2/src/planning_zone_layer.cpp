#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace bw_fastlio_nav2
{
class PlanningZoneLayer : public nav2_costmap_2d::Layer
{
  using Mask = nav_msgs::msg::OccupancyGrid;

public:
  bool isClearable() override {return false;}

  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node || layered_costmap_->isRolling()) {
      throw std::runtime_error("PlanningZoneLayer requires a non-rolling global costmap");
    }
    enabled_ = true;
    current_ = false;
    declareParameter("mask_topic", rclcpp::ParameterValue("/planning_zones/mask"));
    const auto topic = node->get_parameter(getFullName("mask_topic")).as_string();
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node->create_subscription<Mask>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](Mask::ConstSharedPtr msg) {receive(msg);}, options);
  }

  void reset() override {matchSize();}
  void activate() override {matchSize();}

  void matchSize() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    dirty_ = true;
  }

  void updateBounds(double, double, double, double * x0, double * y0,
    double * x1, double * y1) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & grid = *layered_costmap_->getCostmap();
    if (pending_ && aligned(*pending_, grid)) {
      active_ = std::move(pending_);
      dirty_ = true;
    }
    // The static layer can resize the global map after DDS has delivered the mask.
    if (pending_ && grid.getSizeInCellsX() && grid.getSizeInCellsY()) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "Planning zone mask does not align with costmap");
    }
    const bool aligned_now = active_ && aligned(*active_, grid);
    if (!aligned_now) {
      current_ = false;
    }
    if (dirty_ && aligned_now) {
      *x0 = std::min(*x0, grid.getOriginX());
      *y0 = std::min(*y0, grid.getOriginY());
      *x1 = std::max(*x1, grid.getOriginX() + grid.getSizeInCellsX() * grid.getResolution());
      *y1 = std::max(*y1, grid.getOriginY() + grid.getSizeInCellsY() * grid.getResolution());
      dirty_ = false;
    }
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int x0, int y0, int x1, int y1) override
  {
    // Only updateBounds switches the active snapshot. Both calls occur in the
    // same costmap update cycle; the subscription may only replace pending_.
    if (!active_ || !aligned(*active_, grid)) {
      current_ = false;
      return;
    }
    constexpr unsigned char zone_cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
    auto * costs = grid.getCharMap();
    for (int j = y0; j < y1; ++j) {
      size_t i = grid.getIndex(x0, j);
      for (int x = x0; x < x1; ++x, ++i) {
        if (active_->data[i] == 100 && costs[i] < zone_cost) {
          costs[i] = zone_cost;
        }
      }
    }
    current_ = true;
  }

private:
  void receive(const Mask::ConstSharedPtr & msg)
  {
    const auto & info = msg->info;
    const auto & p = info.origin.position;
    const auto & q = info.origin.orientation;
    if (msg->header.frame_id != layered_costmap_->getGlobalFrameID() ||
      !std::isfinite(info.resolution) || info.resolution <= 0 ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || p.z != 0.0 ||
      q.x != 0.0 || q.y != 0.0 || q.z != 0.0 || std::abs(q.w) != 1.0 ||
      !info.width || !info.height ||
      msg->data.size() != static_cast<uint64_t>(info.width) * info.height ||
      !std::all_of(msg->data.begin(), msg->data.end(), [](int8_t v) {return v == 0 || v == 100;}))
    {
      RCLCPP_WARN(logger_, "Ignoring invalid planning zone mask");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = msg;
  }

  static bool aligned(const Mask & mask, const nav2_costmap_2d::Costmap2D & grid)
  {
    const auto & info = mask.info;
    return info.width == grid.getSizeInCellsX() && info.height == grid.getSizeInCellsY() &&
           info.resolution == static_cast<float>(grid.getResolution()) &&
           std::abs(info.origin.position.x - grid.getOriginX()) < 1e-7 &&
           std::abs(info.origin.position.y - grid.getOriginY()) < 1e-7;
  }

  std::mutex mutex_;
  Mask::ConstSharedPtr pending_;
  Mask::ConstSharedPtr active_;
  bool dirty_{true};
  rclcpp::Subscription<Mask>::SharedPtr subscription_;
};
}  // namespace bw_fastlio_nav2

PLUGINLIB_EXPORT_CLASS(bw_fastlio_nav2::PlanningZoneLayer, nav2_costmap_2d::Layer)
