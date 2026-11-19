#ifndef BW_FASTLIO_NAV2__FREEDOM_MASK_HPP_
#define BW_FASTLIO_NAV2__FREEDOM_MASK_HPP_

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace bw_fastlio_nav2
{

// Offline delete-only mask for the production world-frame map. One instance per replay.
class FreedomMask
{
public:
  explicit FreedomMask(const Eigen::Isometry3d &body_from_lidar);
  ~FreedomMask();

  FreedomMask(const FreedomMask &) = delete;
  FreedomMask &operator=(const FreedomMask &) = delete;

  // Integrate the original body-frame scan in temporal order. Invalid samples/poses
  // do not contribute observations. Calling after finalize() is a logic error.
  void integrate(
    const pcl::PointCloud<pcl::PointXYZI> &cloud,
    const Eigen::Isometry3d &world_from_body);

  // Freeze the mask and release the replay state. Safe to call more than once.
  void finalize();

  // Query a production point in world coordinates. Finite points are kept until
  // finalize() and always kept if the cell was unseen or the point is near a face.
  // Invalid XYZI samples are removed independently of the FreeDOM mask.
  bool removes(const pcl::PointXYZI &point) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bw_fastlio_nav2

#endif  // BW_FASTLIO_NAV2__FREEDOM_MASK_HPP_
