/**
 * @file laserPosegraphOptimization.cpp
 * @brief ROS2 port of yanliang-wang/FAST_LIO_LC's PGO/src/laserPosegraphOptimization.cpp
 *
 * 1. Detect the keyframes
 * 2. Maintain the Gtsam-based pose graph
 * 3. Detect the radius-search-based loop closure, and add them to the pose graph
 *
 * Ported from roscpp/tf to rclcpp/tf2. The GTSAM/PCL/ScanContext logic itself is
 * unchanged from the original - only the ROS I/O layer was translated.
 */
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <map>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <atomic>
#include <iostream>
#include <string>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "scancontext/Scancontext.h"

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
bool isNowKeyFrame = false;

std::queue<nav_msgs::msg::Odometry::SharedPtr> odometryBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> fullResBuf;
std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gpsBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds;
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;
// for loop closure detection
std::map<int, int> loopIndexContainer; // existing loop pairs
pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<pcl::PointXYZ>());
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;
SCManager scManager;
double scDistThres, scMaximumRadius;

pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
// downSizeFilterMapPGO is owned by the map-viz thread (pubMap). The batch
// service runs in the executor thread, so it must not reuse that object: two
// concurrent filter() calls on one VoxelGrid resize/free the same internal
// vectors and corrupt the heap ("double free or corruption"). This holds the
// same leaf size for a private filter instance.
double g_mapVizFilterSize = 0.4;
bool laserCloudMapPGORedraw = true;

bool useGPS = true;
sensor_msgs::msg::NavSatFix::SharedPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;
double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMapAftPGO;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftPGO;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathAftPGO;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocal, pubLoopSubmapLocal;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomRepubVerifier;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srvBatchOptimize;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory;
std::string odomKITTIformat;
// World frame all PGO outputs (map, path, odom, loop markers, TF parent) are
// stamped in. Mirrors FAST-LIO's publish.map_frame parameter: defaults to the
// legacy "camera_init" but is overridden to "odom" so the optimized map lands
// in the same REP-105 tree the lio_map_odom_bridge already publishes.
std::string map_frame = "camera_init";
std::fstream pgTimeSaveStream;

// for front_end
rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pubKeyFramesId;

// for loop closure
double historyKeyframeSearchRadius;
double historyKeyframeSearchTimeDiff;
int historyKeyframeSearchNum;
double loopClosureFrequency;
int graphUpdateTimes;
double graphUpdateFrequency;
double loopNoiseScore;
double vizmapFrequency;
double vizPathFrequency;
double speedFactor;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLoopScanLocalRegisted;
double loopFitnessScoreThreshold;

rclcpp::Node::SharedPtr g_node;
std::shared_ptr<tf2_ros::TransformBroadcaster> g_tf_broadcaster;
std::atomic<uint32_t> g_seq_counter{0};

// -- small ROS1 -> ROS2 time helpers --------------------------------------
inline double stampToSec(const builtin_interfaces::msg::Time & t)
{
    return rclcpp::Time(t).seconds();
}

inline builtin_interfaces::msg::Time secToStamp(double sec)
{
    rclcpp::Time t(static_cast<int64_t>(sec * 1e9));
    return t;
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void saveOdometryVerticesKITTIformat(std::string _filename)
{
    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    for(const auto& _pose6d: keyframePoses) {
        gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}

bool saveOptimizedVerticesKITTIformat(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);
    if (!stream.is_open()) {
        RCLCPP_ERROR(g_node->get_logger(), "failed to open %s for writing", _filename.c_str());
        return false;
    }

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
    stream.flush();
    return stream.good();
}

// PCL's PCD writer throws pcl::IOException when the target file cannot be
// opened (missing directory, permissions, full disk, a directory removed while
// the node is running). Uncaught, that throw terminates the whole mapping node,
// so every PCD write goes through this guard: log once per failing path and
// keep mapping instead of aborting.
bool savePcdSafe(const std::string &path, const pcl::PointCloud<PointType> &cloud)
{
    try {
        std::filesystem::path parent = std::filesystem::path(path).parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        pcl::io::savePCDFileBinary(path, cloud);
        return true;
    } catch (const std::exception &e) {
        static std::string last_failed_path;
        if (last_failed_path != path) {
            last_failed_path = path;
            RCLCPP_ERROR(g_node->get_logger(),
                "failed to write %s: %s (mapping continues)",
                path.c_str(), e.what());
        }
        return false;
    }
}

void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr _laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

void laserCloudFullResHandler(const sensor_msgs::msg::PointCloud2::SharedPtr _laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler

void gpsHandler(const sensor_msgs::msg::NavSatFix::SharedPtr _gps)
{
    if(useGPS) {
        mBuf.lock();
        gpsBuf.push(_gps);
        mBuf.unlock();
    }
} // gpsHandler

void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);

    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0;
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore;
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D getOdom(nav_msgs::msg::Odometry::SharedPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    const geometry_msgs::msg::Quaternion & quat = _odom->pose.pose.orientation;
    tf2::Quaternion tfq(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw, g_seq_counter.fetch_add(1)};
} // getOdom

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw)), 0};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

void pubPath( void )
{
    // pub odom and path
    nav_msgs::msg::Odometry odomAftPGO;
    nav_msgs::msg::Path pathAftPGO;
    pathAftPGO.header.frame_id = map_frame;
    mKF.lock();
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // updated poses

        nav_msgs::msg::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = map_frame;
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = secToStamp(keyframeTimes.at(node_idx));
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;
        tf2::Quaternion q;
        q.setRPY(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGOthis.pose.pose.orientation = tf2::toMsg(q);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::msg::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = map_frame;
        pathAftPGO.poses.push_back(poseStampAftPGO);
    }
    mKF.unlock();
    pubOdomAftPGO->publish(odomAftPGO); // last pose
    pubPathAftPGO->publish(pathAftPGO); // poses

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odomAftPGO.header.stamp;
    transform.header.frame_id = map_frame;
    transform.child_frame_id = "aft_pgo";
    transform.transform.translation.x = odomAftPGO.pose.pose.position.x;
    transform.transform.translation.y = odomAftPGO.pose.pose.position.y;
    transform.transform.translation.z = odomAftPGO.pose.pose.position.z;
    transform.transform.rotation = odomAftPGO.pose.pose.orientation;
    g_tf_broadcaster->sendTransform(transform);
} // pubPath

void updatePoses(void)
{
    mKF.lock();
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    // Store a count, not the last index. All visualization loops use '<', so
    // subtracting one omitted the newest keyframe and published empty odometry
    // for a one-keyframe graph.
    recentIdxUpdated = int(keyframePosesUpdated.size());

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    for(int i = graphUpdateTimes; i > 0; --i){
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
    pubPath();
}

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(),
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );

    int numberOfCores = 8;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
        cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    for (int i = -submap_size; i <= submap_size; ++i) {
        int keyNear = root_idx + i;
        if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
            continue;

        mKF.lock();
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud

void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    nearKeyframes->clear();
    int cloudSize = keyframeLaserClouds.size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        mKF.lock();
        *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[keyNear]);
        mKF.unlock();
    }

    if (nearKeyframes->empty())
        return;

    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

Eigen::Affine3f Pose6dToAffine3f(Pose6D pose)
{
    return pcl::getTransformation(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

gtsam::Pose3 Pose6dTogtsamPose3(Pose6D pose)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(pose.roll), double(pose.pitch), double(pose.yaw)),
                                gtsam::Point3(double(pose.x),    double(pose.y),     double(pose.z)));
}

gtsam::Pose3 doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframes(cureKeyframeCloud, _curr_kf_idx, 0);
    loopFindNearKeyframes(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum);

    // loop verification
    sensor_msgs::msg::PointCloud2 cureKeyframeCloudMsg;
    pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
    cureKeyframeCloudMsg.header.frame_id = map_frame;
    pubLoopScanLocal->publish(cureKeyframeCloudMsg);

    sensor_msgs::msg::PointCloud2 targetKeyframeCloudMsg;
    pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
    targetKeyframeCloudMsg.header.frame_id = map_frame;
    pubLoopSubmapLocal->publish(targetKeyframeCloudMsg);

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align pointclouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(targetKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    sensor_msgs::msg::PointCloud2 cureKeyframeCloudRegMsg;
    pcl::toROSMsg(*unused_result, cureKeyframeCloudRegMsg);
    cureKeyframeCloudRegMsg.header.frame_id = map_frame;
    pubLoopScanLocalRegisted->publish(cureKeyframeCloudRegMsg);

    if (icp.hasConverged() == false || icp.getFitnessScore() > loopFitnessScoreThreshold) {
        std::cout << "[SC loop] ICP fitness test failed (" << icp.getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return gtsam::Pose3::Identity();
    } else {
        std::cout << "[SC loop] ICP fitness test passed (" << icp.getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // keyframePosesUpdated is rewritten by updatePoses() from process_isam;
    // take a copy under the keyframe lock instead of reading it unlocked.
    Pose6D currPoseCopy, loopPoseCopy;
    mKF.lock();
    currPoseCopy = keyframePosesUpdated[_curr_kf_idx];
    loopPoseCopy = keyframePosesUpdated[_loop_kf_idx];
    mKF.unlock();

    Eigen::Affine3f tWrong = Pose6dToAffine3f(currPoseCopy);

    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo =  Pose6dTogtsamPose3(loopPoseCopy);

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{
    while(rclcpp::ok())
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not
            //
			mBuf.lock();
            while (!odometryBuf.empty() && stampToSec(odometryBuf.front()->header.stamp) < stampToSec(fullResBuf.front()->header.stamp))
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            timeLaserOdometry = stampToSec(odometryBuf.front()->header.stamp);
            timeLaser = stampToSec(fullResBuf.front()->header.stamp);

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            // find nearest gps
            double eps = 0.1; // find a gps topic arrived within eps second
            while (!gpsBuf.empty()) {
                auto thisGPS = gpsBuf.front();
                auto thisGPSTime = stampToSec(thisGPS->header.stamp);
                if( abs(thisGPSTime - timeLaserOdometry) < eps ) {
                    currGPS = thisGPS;
                    hasGPSforThisKF = true;
                    break;
                } else {
                    hasGPSforThisKF = false;
                }
                gpsBuf.pop();
            }
            mBuf.unlock();

            //
            // Select against the last accepted keyframe. Accumulating absolute
            // frame-to-frame deltas turns stationary estimator jitter into
            // artificial motion and duplicate keyframes/loop edges.
            //
            if (keyframePoses.empty()) {
                isNowKeyFrame = true;
            } else {
                const Pose6D dtf = diffTransformation(keyframePoses.back(), pose_curr);
                const double delta_translation =
                    std::sqrt(dtf.x * dtf.x + dtf.y * dtf.y + dtf.z * dtf.z);
                const double delta_rotation =
                    std::sqrt(dtf.roll * dtf.roll + dtf.pitch * dtf.pitch +
                              dtf.yaw * dtf.yaw);
                isNowKeyFrame = delta_translation >= keyframeMeterGap ||
                                delta_rotation >= keyframeRadGap;
            }

            if( ! isNowKeyFrame )
                continue;

            if( !gpsOffsetInitialized ) {
                if(hasGPSforThisKF) { // if the very first frame
                    gpsAltitudeInitOffset = currGPS->altitude;
                    gpsOffsetInitialized = true;
                }
            }

            //
            // Save data and Add consecutive node
            //
            pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            downSizeFilterScancontext.filter(*thisKeyFrameDS);

            mKF.lock();
            keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframePoses.push_back(pose_curr);
            {
                // publish keyframe id
                std_msgs::msg::Header keyFrameHeader;
                keyFrameHeader.stamp = g_node->now();
                pubKeyFramesId->publish(keyFrameHeader);
            }
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            scManager.makeAndSaveScancontextAndKeys(*thisKeyFrameDS);

            laserCloudMapPGORedraw = true;
            mKF.unlock();

            const int prev_node_idx = keyframePoses.size() - 2;
            const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts with 0
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0;
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));

                mtxPosegraph.lock();
                {
                    // prior factor
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                }
                mtxPosegraph.unlock();

                gtSAMgraphMade = true;

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ {
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, poseFrom.between(poseTo), odomNoise));

                    // gps factor
                    if(hasGPSforThisKF) {
                        double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                        mtxRecentPose.lock();
                        gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY, curr_altitude_offseted);
                        mtxRecentPose.unlock();
                        gtSAMgraph.add(gtsam::GPSFactor(curr_node_idx, gpsConstraint, robustGPSNoise));
                        cout << "GPS factor added at node " << curr_node_idx << endl;
                    }
                    initialEstimate.insert(curr_node_idx, poseTo);
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 100 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }

            // save utility
            std::string curr_node_idx_str = padZeros(curr_node_idx);
            savePcdSafe(pgScansDirectory + curr_node_idx_str + ".pcd", *thisKeyFrame); // scan
            pgTimeSaveStream << timeLaser << std::endl; // path
        }

        // ps.
        // scan context detector is running in another thread (in constant Hz, e.g., 1 Hz)
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void performSCLoopClosure(void)
{
    if( int(keyframePoses.size()) < scManager.NUM_EXCLUDE_RECENT) // do not try too early
        return;

    auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
    int SCclosestHistoryFrameID = detectResult.first;
    if( SCclosestHistoryFrameID != -1 ) {
        const int prev_node_idx = SCclosestHistoryFrameID;
        const int curr_node_idx = keyframePoses.size() - 1; // because cpp starts 0 and ends n-1
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // adding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

pcl::PointCloud<pcl::PointXYZ>::Ptr vector2pc(const std::vector<Pose6D> vectorPose6d){
    pcl::PointCloud<pcl::PointXYZ>::Ptr res( new pcl::PointCloud<pcl::PointXYZ> ) ;
    for( auto p : vectorPose6d){
        res->points.emplace_back(p.x, p.y, p.z);
    }
    return res;
}

/**
 * Find near keyframes by distance in history keyframes; pick a temporally distant one as loop candidate
*/
bool detectLoopClosureDistance(int *loopKeyCur, int *loopKeyPre)
{
    auto it = loopIndexContainer.find(*loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloudKeyPoses3D = vector2pc(keyframePoses);
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for(int i = 0; i < pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if ( abs( keyframeTimes[id] - keyframeTimes[*loopKeyCur] ) > historyKeyframeSearchTimeDiff )
        {
            *loopKeyPre = id;
            break;
        }
    }

    if (*loopKeyPre == -1 || *loopKeyCur == *loopKeyPre)
        return false;

    return true;
}

void performRSLoopClosure(void)
{
    if( keyframePoses.empty() )
        return;

    int loopKeyCur = keyframePoses.size() - 1;
    int loopKeyPre = -1;
    if ( detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) ){
        cout << "Loop detected! - between " << loopKeyPre << " and " << loopKeyCur << "" << endl;
        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(loopKeyPre, loopKeyCur));
        loopIndexContainer[loopKeyCur] = loopKeyPre ;
        mBuf.unlock();
    } else
        return;
} // performRSLoopClosure

void visualizeLoopClosure()
{
    if (loopIndexContainer.empty())
        return;

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = map_frame;
    markerNode.header.stamp = secToStamp(keyframeTimes.back());
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = map_frame;
    markerEdge.header.stamp = secToStamp(keyframeTimes.back());
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::msg::Point p;
        p.x = keyframePosesUpdated[key_cur].x;
        p.y = keyframePosesUpdated[key_cur].y;
        p.z = keyframePosesUpdated[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = keyframePosesUpdated[key_pre].x;
        p.y = keyframePosesUpdated[key_pre].y;
        p.z = keyframePosesUpdated[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge->publish(markerArray);
}

void process_lcd(void)
{
    rclcpp::Rate rate(loopClosureFrequency);
    while (rclcpp::ok())
    {
        rate.sleep();
        // keyframePoses / keyframeTimes / keyframePosesUpdated are pushed back
        // (and therefore reallocated) by process_pg and rewritten by updatePoses
        // while this thread reads them; without the lock a reallocation here
        // leaves dangling reads of freed heap memory.
        mKF.lock();
        performRSLoopClosure();
        visualizeLoopClosure();
        mKF.unlock();
    }
} // process_lcd

void process_icp(void)
{
    while(rclcpp::ok())
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                RCLCPP_WARN(g_node->get_logger(), "Too many loop closure candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock();
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock();

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if( !relative_pose.equals( gtsam::Pose3::Identity() )) {
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(curr_node_idx, prev_node_idx, relative_pose, robustLoopNoise));
                mtxPosegraph.unlock();
            }
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = vizPathFrequency;
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = graphUpdateFrequency;
    rclcpp::Rate rate(hz);
    while (rclcpp::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            mtxPosegraph.unlock();

            saveOptimizedVerticesKITTIformat(isamCurrentEstimate, pgKITTIformat); // pose
            saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
        }
    }
}

void pubMap(void)
{
    int SKIP_FRAMES = 1; // sparse map visualization to save computations
    int counter = 0;

    laserCloudMapPGO->clear();

    mKF.lock();
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        if(counter % SKIP_FRAMES == 0) {
            *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        }
        counter++;
    }
    mKF.unlock();

    downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::msg::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = map_frame;
    pubMapAftPGO->publish(laserCloudMapPGOMsg);
}

// One-shot batch re-optimization of the FULL accumulated pose graph, called
// via the /pgo_batch_optimize service once a session is done. process_isam()
// only ever runs iSAM2's incremental update (good enough given data seen so
// far); ISAM2 internally retains every factor it has been given, so
// getFactorsUnsafe() hands back the complete NonlinearFactorGraph built over
// the whole run, which a batch Levenberg-Marquardt solve can then optimize
// as a single global problem instead of walking through it causally.
// Writes optimized_poses_batch.txt (does not touch the live
// optimized_poses.txt) and rebuilds+saves a map from the batch poses.
void batchOptimizeHandler(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    mtxPosegraph.lock();
    gtsam::NonlinearFactorGraph fullGraph = isam->getFactorsUnsafe();
    gtsam::Values initial = isam->calculateEstimate();
    mtxPosegraph.unlock();

    if (fullGraph.empty() || initial.empty()) {
        response->success = false;
        response->message = "Pose graph is empty, nothing to optimize.";
        return;
    }

    double initialError = fullGraph.error(initial);

    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    params.setMaxIterations(100);
    gtsam::LevenbergMarquardtOptimizer optimizer(fullGraph, initial, params);
    gtsam::Values batchEstimate = optimizer.optimize();

    double finalError = fullGraph.error(batchEstimate);

    // The session directory can disappear while the node is running (it lives on
    // a removable/remote filesystem or is cleaned up manually); recreate it before
    // the writers below, which throw or silently fail on a missing directory.
    std::error_code dir_ec;
    std::filesystem::create_directories(save_directory, dir_ec);

    bool posesSaved = saveOptimizedVerticesKITTIformat(batchEstimate, save_directory + "optimized_poses_batch.txt");

    pcl::PointCloud<PointType>::Ptr batchMap(new pcl::PointCloud<PointType>());
    mKF.lock();
    int n = std::min(int(batchEstimate.size()), int(keyframeLaserClouds.size()));
    for (int node_idx = 0; node_idx < n; node_idx++) {
        const gtsam::Pose3& pose = batchEstimate.at<gtsam::Pose3>(node_idx);
        Pose6D p6;
        p6.x = pose.translation().x();
        p6.y = pose.translation().y();
        p6.z = pose.translation().z();
        p6.roll = pose.rotation().roll();
        p6.pitch = pose.rotation().pitch();
        p6.yaw = pose.rotation().yaw();
        p6.seq = node_idx;
        *batchMap += *local2global(keyframeLaserClouds[node_idx], p6);
    }
    mKF.unlock();

    pcl::VoxelGrid<PointType> batchFilter; // private: see g_mapVizFilterSize
    batchFilter.setLeafSize(g_mapVizFilterSize, g_mapVizFilterSize, g_mapVizFilterSize);
    batchFilter.setInputCloud(batchMap);
    batchFilter.filter(*batchMap);
    if (!savePcdSafe(save_directory + "map_batch.pcd", *batchMap)) {
        response->success = false;
        response->message = "Batch optimization finished over " + std::to_string(n)
            + " keyframes, but map_batch.pcd could not be written to " + save_directory;
        return;
    }

    std::ostringstream msg;
    msg << "Batch LM re-optimization done over " << n << " keyframes. "
        << "Graph error " << initialError << " -> " << finalError << ". "
        << "Poses: " << save_directory << "optimized_poses_batch.txt, "
        << "map: " << save_directory << "map_batch.pcd";
    if (!posesSaved) {
        msg << " (WARNING: optimized_poses_batch.txt could not be written)";
    }
    response->success = true;
    response->message = msg.str();
    RCLCPP_INFO(g_node->get_logger(), "%s", msg.str().c_str());
}

void process_viz_map(void)
{
    rclcpp::Rate rate(vizmapFrequency);
    while (rclcpp::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	g_node = std::make_shared<rclcpp::Node>("laserPGO");
	g_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

    g_node->declare_parameter<std::string>("save_directory", "/");
    save_directory = g_node->get_parameter("save_directory").as_string();
    std::filesystem::path save_root(save_directory);
    if (save_root.empty() || save_root == save_root.root_path()) {
        RCLCPP_FATAL(g_node->get_logger(),
            "save_directory must name a dedicated session directory, not '%s'",
            save_directory.c_str());
        rclcpp::shutdown();
        return 2;
    }
    save_root = save_root.lexically_normal();
    save_directory = save_root.string();
    while (save_directory.size() > 1 && save_directory.back() == '/') {
        save_directory.pop_back();
    }
    save_directory += "/";

    // World frame for all published outputs (see map_frame declaration above).
    g_node->declare_parameter<std::string>("map_frame", "camera_init");
    map_frame = g_node->get_parameter("map_frame").as_string();
    pgKITTIformat = save_directory + "optimized_poses.txt";
    odomKITTIformat = save_directory + "odom_poses.txt";
    pgScansDirectory = save_directory + "Scans/";
    // Only the per-session scan directory is replaced. Avoid passing the
    // user-provided path through a shell.
    try {
        std::filesystem::create_directories(save_directory);
        std::filesystem::remove_all(pgScansDirectory);
        std::filesystem::create_directories(pgScansDirectory);
        std::filesystem::remove(save_directory + "optimized_poses_batch.txt");
        std::filesystem::remove(save_directory + "map_batch.pcd");
    } catch (const std::exception &e) {
        RCLCPP_FATAL(g_node->get_logger(),
            "cannot prepare save_directory '%s': %s", save_directory.c_str(), e.what());
        rclcpp::shutdown();
        return 3;
    }
    pgTimeSaveStream = std::fstream(save_directory + "times.txt", std::fstream::out);
    if (!pgTimeSaveStream.is_open()) {
        RCLCPP_ERROR(g_node->get_logger(), "failed to open %stimes.txt for writing", save_directory.c_str());
    }
    pgTimeSaveStream.precision(std::numeric_limits<double>::max_digits10);

    g_node->declare_parameter<double>("keyframe_meter_gap", 2.0);
    keyframeMeterGap = g_node->get_parameter("keyframe_meter_gap").as_double();
    g_node->declare_parameter<double>("keyframe_deg_gap", 10.0);
    keyframeDegGap = g_node->get_parameter("keyframe_deg_gap").as_double();
    keyframeRadGap = deg2rad(keyframeDegGap);

    g_node->declare_parameter<double>("sc_dist_thres", 0.2);
    scDistThres = g_node->get_parameter("sc_dist_thres").as_double();
    g_node->declare_parameter<double>("sc_max_radius", 80.0);
    scMaximumRadius = g_node->get_parameter("sc_max_radius").as_double();

    // for loop closure detection
    g_node->declare_parameter<double>("historyKeyframeSearchRadius", 10.0);
    historyKeyframeSearchRadius = g_node->get_parameter("historyKeyframeSearchRadius").as_double();
    g_node->declare_parameter<double>("historyKeyframeSearchTimeDiff", 30.0);
    historyKeyframeSearchTimeDiff = g_node->get_parameter("historyKeyframeSearchTimeDiff").as_double();
    g_node->declare_parameter<int>("historyKeyframeSearchNum", 25);
    historyKeyframeSearchNum = g_node->get_parameter("historyKeyframeSearchNum").as_int();
    g_node->declare_parameter<double>("loopNoiseScore", 0.5);
    loopNoiseScore = g_node->get_parameter("loopNoiseScore").as_double();
    g_node->declare_parameter<int>("graphUpdateTimes", 2);
    graphUpdateTimes = g_node->get_parameter("graphUpdateTimes").as_int();
    g_node->declare_parameter<double>("loopFitnessScoreThreshold", 0.3);
    loopFitnessScoreThreshold = g_node->get_parameter("loopFitnessScoreThreshold").as_double();

    g_node->declare_parameter<double>("speedFactor", 1.0);
    speedFactor = g_node->get_parameter("speedFactor").as_double();
    {
        g_node->declare_parameter<double>("loopClosureFrequency", 2.0);
        loopClosureFrequency = g_node->get_parameter("loopClosureFrequency").as_double();
        loopClosureFrequency *= speedFactor;
        g_node->declare_parameter<double>("graphUpdateFrequency", 1.0);
        graphUpdateFrequency = g_node->get_parameter("graphUpdateFrequency").as_double();
        graphUpdateFrequency *= speedFactor;
        g_node->declare_parameter<double>("vizmapFrequency", 0.1);
        vizmapFrequency = g_node->get_parameter("vizmapFrequency").as_double();
        vizmapFrequency *= speedFactor;
        g_node->declare_parameter<double>("vizPathFrequency", 10.0);
        vizPathFrequency = g_node->get_parameter("vizPathFrequency").as_double();
        vizPathFrequency *= speedFactor;
    }

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    scManager.setSCdistThres(scDistThres);
    scManager.setMaximumRadius(scMaximumRadius);

    float filter_size = 0.4;
    downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    g_node->declare_parameter<double>("mapviz_filter_size", 0.4);
    double mapVizFilterSize = g_node->get_parameter("mapviz_filter_size").as_double();
    downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);
    g_mapVizFilterSize = mapVizFilterSize;

    g_node->declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    g_node->declare_parameter<std::string>("odom_topic", "/Odometry");
    std::string cloud_topic = g_node->get_parameter("cloud_topic").as_string();
    std::string odom_topic = g_node->get_parameter("odom_topic").as_string();

	auto subLaserCloudFullRes = g_node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::QoS(100), laserCloudFullResHandler);
	auto subLaserOdometry = g_node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(100), laserOdometryHandler);
	auto subGPS = g_node->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", rclcpp::QoS(100), gpsHandler);

	pubOdomAftPGO = g_node->create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = g_node->create_publisher<nav_msgs::msg::Odometry>("/repub_odom", 100);

    // for front-end
    pubKeyFramesId = g_node->create_publisher<std_msgs::msg::Header>("/key_frames_ids", 10);

    // for loop closure
    pubLoopConstraintEdge = g_node->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 1);
	pubLoopScanLocalRegisted = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local_registed", 100);

	pubPathAftPGO = g_node->create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 100);

	pubLoopScanLocal = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", 100);
	pubLoopSubmapLocal = g_node->create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", 100);

	srvBatchOptimize = g_node->create_service<std_srvs::srv::Trigger>(
        "/pgo_batch_optimize", batchOptimizeHandler);

	std::thread posegraph_slam {process_pg}; // pose graph construction
	std::thread lc_detection {process_lcd}; // loop closure detection
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp
	std::thread isam_update {process_isam}; // isam2 optimization

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)

	rclcpp::spin(g_node);

    posegraph_slam.join();
    lc_detection.join();
    icp_calculation.join();
    isam_update.join();
    viz_map.join();
    pgTimeSaveStream.close();
    delete isam;

    // The upstream implementation stores these handles globally. Destroy
    // them deterministically instead of relying on static destruction order.
    srvBatchOptimize.reset();
    pubLoopSubmapLocal.reset();
    pubLoopScanLocal.reset();
    pubMapAftPGO.reset();
    pubPathAftPGO.reset();
    pubLoopScanLocalRegisted.reset();
    pubLoopConstraintEdge.reset();
    pubKeyFramesId.reset();
    pubOdomRepubVerifier.reset();
    pubOdomAftPGO.reset();
    g_tf_broadcaster.reset();
    subGPS.reset();
    subLaserOdometry.reset();
    subLaserCloudFullRes.reset();
    g_node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

	return 0;
}
