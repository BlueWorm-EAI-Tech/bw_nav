#include "freedom/freedom.h"

#include <stdexcept>

namespace freedom{
void FreeDOM::set_params(const Config& config)
{
    // set params
    ScanMap::ScanMapConfig scan_config{
        config.sub_voxel_size,
        config.voxel_depth,
        config.block_depth,
        config.enable_local_map,
        config.local_map_range,
        config.local_map_min_z,
        config.local_map_max_z,
        config.num_threads,
        config.sensor_min_range,
        config.sensor_max_range,
        config.sensor_min_z,
        config.sensor_max_z
    };
    scan.set_params(scan_config);

    MRMap::MRMapConfig map_config{
        config.sub_voxel_size,
        config.voxel_depth,
        config.block_depth,
        config.enable_local_map,
        config.local_map_range,
        config.local_map_min_z,
        config.local_map_max_z,
        config.num_threads,
        config.sensor_max_range,
        config.sensor_min_z,
        config.sensor_max_z,
        config.raycast_max_range,
        config.raycast_min_z,
        config.raycast_max_z,
        config.counts_to_free,
        config.counts_to_revert,
        config.conservative_connectivity,
        config.aggressive_connectivity
    };
    map.set_params(map_config);

    enable_raycast_enhancement = config.enable_raycast_enhancement;
    if(enable_raycast_enhancement)
    {
        DepthImage::DepthImageConfig depth_image_config{
            config.lidar_horizon_fov,
            config.lidar_vertical_fov_upper,
            config.lidar_vertical_fov_lower,
            config.depth_image_vertical_lines,
            config.depth_image_min_range,
            config.max_raycast_enhancement_range,
            config.raycast_enhancement_depth_margin,
            config.inpaint_size,
            config.erosion_size,
            config.min_raycast_enhancement_area,
            config.depth_image_top_margin,
            config.learn_fov,
            config.enable_fov_mask,
            config.fov_mask_path,
            config.num_threads
        };
        depth_image.set_params(depth_image_config);
    }

    // init variables
    scan_seq = 0;
}

void FreeDOM::set_scan_removal_callback(std::function<void(const ScanMap&)> callback)
{
    scan_removal_callback = callback;
}

void FreeDOM::set_raycast_enhancement_callback(std::function<void(const DepthImage&)> callback)
{
    raycast_enhancement_callback = callback;
}

void FreeDOM::set_map_removal_callback(std::function<void(const MRMap&)> callback)
{
    map_removal_callback = callback;
}

void FreeDOM::pointcloud_integrate(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform)
{
    if (scan_seq == StaticVoxel::NOT_A_SCAN)
        throw std::overflow_error("FreeDOM scan sequence exhausted");

    // 构建scan map,为了快速随机查找与快速遍历
    scan.build_scan_map(cloud,transform);

    // 遍历scan voxel，查找对应的free voxel并设置scan voxel为CONSERVATIVE_DYNAMIC且加入occ_in_freespace
    // 对occ_in_freespace进行聚类并完善其DynamicLevel
    map.scan_removal(scan);

    // 滤除当前帧动态点云后立即触发回调
    if(scan_removal_callback)
        scan_removal_callback(scan);

    if(enable_raycast_enhancement)
    {
        depth_image.raycast_enhancement(cloud,transform);

        if(raycast_enhancement_callback)
            raycast_enhancement_callback(depth_image);
    }

    Indices freespace_incremental;
    // FreeSpace estimation
    map.freespace_estimation(scan,depth_image,freespace_incremental);

    // map clearing
    map.map_removal(freespace_incremental);

    // map integration
    map.staticspace_integration(scan,scan_seq);

    map.remove_map_out_of_bound();

    // 集成地图后立即触发回调
    if(map_removal_callback)
        map_removal_callback(map);

    scan.reset();
    map.reset();
    ++ scan_seq;
}
}
