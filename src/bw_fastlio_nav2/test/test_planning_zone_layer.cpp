#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_loader.hpp"

using namespace std::chrono_literals;
using Mask = nav_msgs::msg::OccupancyGrid;

class EnvironmentLayer : public nav2_costmap_2d::Layer
{
public:
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D & map, int, int, int, int) override
  {
    map.setCost(1, 0, 253);
    map.setCost(2, 0, 254);
    map.setCost(3, 0, 255);
    map.setCost(4, 0, 179);
  }
};

TEST(PlanningZoneLayer, ReplacesMaskWithoutClearingEnvironment)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<nav2_util::LifecycleNode>("planning_zone_layer_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  nav2_costmap_2d::LayeredCostmap map("map", false, false);
  map.resizeMap(6, 2, 0.5, 0, 0);
  map.addPlugin(std::make_shared<EnvironmentLayer>());
  auto layer = loader.createSharedInstance("bw_fastlio_nav2::PlanningZoneLayer");
  map.addFilter(layer);
  layer->initialize(&map, "planning_zone_layer", nullptr, node, nullptr);
  EXPECT_FALSE(layer->isCurrent());

  auto pub = node->create_publisher<Mask>("planning_zones/mask", rclcpp::QoS(1).transient_local());
  pub->on_activate();
  Mask mask;
  mask.header.frame_id = "map";
  mask.info.width = 6;
  mask.info.height = 2;
  mask.info.resolution = 0.5;
  mask.info.origin.orientation.w = 1.0;
  mask.data = {100, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0};
  auto wait_for = [&](unsigned char expected) {
      auto end = std::chrono::steady_clock::now() + 2500ms;
      do {
        executor.spin_some();
        map.updateMap(0.5, 0.5, 0.0);
        if (layer->isCurrent() && map.getCostmap()->getCost(0, 0) == expected) {return true;}
        std::this_thread::sleep_for(5ms);
      } while (std::chrono::steady_clock::now() < end);
      return false;
    };
  pub->publish(mask);
  ASSERT_TRUE(wait_for(252));
  EXPECT_EQ(map.getCostmap()->getCost(1, 0), 253);
  EXPECT_EQ(map.getCostmap()->getCost(2, 0), 254);
  EXPECT_EQ(map.getCostmap()->getCost(3, 0), 255);
  EXPECT_EQ(map.getCostmap()->getCost(4, 0), 252);
  EXPECT_EQ(map.getCostmap()->getCost(5, 0), 0);

  layer->reset();
  ASSERT_TRUE(wait_for(252));
  mask.data.assign(12, 0);
  pub->publish(mask);
  ASSERT_TRUE(wait_for(0));
  EXPECT_EQ(map.getCostmap()->getCost(4, 0), 179);
  EXPECT_EQ(map.getCostmap()->getCost(2, 0), 254);
}
