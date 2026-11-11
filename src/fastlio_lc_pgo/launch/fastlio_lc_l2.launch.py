import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    pgo_share = get_package_share_directory('fastlio_lc_pgo')
    sensor_tf_share = get_package_share_directory('pepper_slam')
    default_rviz_cfg = os.path.join(pgo_share, 'rviz', 'fastlio_lc.rviz')

    save_directory = LaunchConfiguration('save_directory')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    use_sim_time = LaunchConfiguration('use_sim_time')
    occupancy = LaunchConfiguration('occupancy')

    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory', default_value='/home/yoha/Lidar/run_l2_lc/pgo_output/',
        description='Directory where PGO writes optimized poses, odom poses, times and keyframe scans (its Scans/ subfolder is wiped on startup)'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with both the raw FAST-LIO view and the loop-closure (PGO) view pre-configured '
                     '(/aft_pgo_map, /aft_pgo_path, /loop_closure_constraints).'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the bag clock (ros2 bag play --clock). Keep true for offline runs.'
    )
    declare_occupancy_cmd = DeclareLaunchArgument(
        'occupancy', default_value='true',
        description='Run octomap_server on the per-scan /cloud_registered_body to '
                    'build a ray-traced 2D OccupancyGrid on /projected_map. Save it: '
                    'ros2 run nav2_map_server map_saver_cli -t /projected_map -f <name>'
    )
    # Height band for the 2D projection, in the map_level (gravity-aligned) frame.
    # MEASURED floor plane of this L2-on-Pepper rig sits at z ~= -0.45 m in
    # map_level (near -0.42, dropping to ~-0.53 by 8 m out -- a slight leveling
    # tilt). The lower bound MUST stay above the floor or the traversed floor is
    # marked occupied (a black trail wherever the robot drives). -0.25 clears the
    # floor by ~0.17 m while still catching low obstacles.
    declare_occ_min_z_cmd = DeclareLaunchArgument(
        'occ_min_z', default_value='-0.25',
        description='occupancy_min_z for octomap (map_level coords; must be ABOVE '
                    'the floor plane ~= -0.45 m, else the floor becomes obstacles)'
    )
    declare_occ_max_z_cmd = DeclareLaunchArgument(
        'occ_max_z', default_value='1.0',
        description='occupancy_max_z for octomap (map_level coords; ~1.45 m above floor)'
    )
    declare_self_hit_range_cmd = DeclareLaunchArgument(
        'self_hit_range', default_value='0.8',
        description='Drop scan points closer than this (m) before octomap, to remove '
                    'the low lidar seeing Pepper self-hits (a black trail along the path).'
    )
    declare_max_range_cmd = DeclareLaunchArgument(
        'max_range', default_value='20.0',
        description='octomap sensor_model.max_range (m). Beams are only inserted/cleared '
                    'out to this. Kept at full range for accuracy; clean the thin clearing-'
                    'ray spokes off the FINISHED map with clean_occupancy_map.py instead.'
    )
    # Radius-outlier removal is OFF by default: the per-scan L2 cloud is sparse,
    # so any neighbourly setting also deletes real far-wall returns (tested
    # 3-in-0.25 m dropped ~60% of points), and it does NOT remove the clearing-
    # ray spokes anyway (those come from real see-through observations, not spike
    # endpoints -- use clean_occupancy_map.py for spokes). Enable ONLY if you see
    # isolated occupied dots, and keep it loose (e.g. ror_radius:=0.5).
    declare_ror_neighbors_cmd = DeclareLaunchArgument(
        'ror_min_neighbors', default_value='0',
        description='Radius-outlier removal: drop scan points with fewer than this many '
                    'neighbours within ror_radius. 0 = OFF (default). Only for isolated '
                    'occupied dots; does NOT remove clearing-ray spokes.'
    )
    declare_ror_radius_cmd = DeclareLaunchArgument(
        'ror_radius', default_value='0.5',
        description='Radius (m) for radius-outlier removal neighbour count (keep large '
                    'on the sparse per-scan cloud so real far returns are not deleted).'
    )
    occ_min_z = LaunchConfiguration('occ_min_z')
    occ_max_z = LaunchConfiguration('occ_max_z')
    self_hit_range = LaunchConfiguration('self_hit_range')
    max_range = LaunchConfiguration('max_range')
    ror_min_neighbors = LaunchConfiguration('ror_min_neighbors')
    ror_radius = LaunchConfiguration('ror_radius')

    # Static sensor rig: base_footprint -> l2lidar_frame -> l2lidar_imu (+ cams).
    # This is the piece missing from the bag's own /tf; without it the FAST-LIO
    # bridge cannot close odom -> base_footprint.
    sensor_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensor_tf_share, 'launch', 'pepper_sensor_tf.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # FAST-LIO owns odom -> base_footprint (via lio_map_odom_bridge). Its own
    # leveling (odom_level -> odom) is disabled here because PGO owns map -> odom
    # below -- otherwise odom would get two parents.
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'l2.yaml',
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
            'use_sim_time': use_sim_time,
            'bridge_level_frame': 'false',
        }.items()
    )

    # PGO owns the loop-closure correction map -> odom, completing the REP-105
    # tree:  map -> odom -> base_footprint -> l2lidar_frame -> l2lidar_imu.
    # It also publishes the one-time upright frame map_level (RViz fixed frame).
    pgo_map_odom_bridge = Node(
        package='fast_lio',
        executable='pgo_map_odom_bridge.py',
        name='pgo_map_odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'level_frame': 'map_level',
            'odom_topic': '/Odometry',
            'pgo_odom_topic': '/aft_pgo_odom',
            'publish_level_frame': True,
        }],
    )

    # Ray-traced 2D occupancy grid via octomap_server. It consumes the PER-SCAN
    # cloud /cloud_registered_body (body frame l2lidar_imu) and, using our
    # corrected TF (map -> odom -> base_footprint -> l2lidar_imu), clears free
    # space by casting rays from each scan's sensor origin -- which a top-down
    # projection of the accumulated /aft_pgo_map cannot do. The octree is built
    # in the gravity-aligned map_level frame so the down-projection is level.
    # Publishes nav_msgs/OccupancyGrid on /projected_map.
    # NOTE: octomap inserts scans online and does not retro-correct already-
    # inserted voxels, so a LARGE loop closure leaves a seam; rebuild from
    # corrected keyframes if that matters. For room/corridor drift it is fine.
    # Strip robot self-hits (near-field cluster within ~0.6 m of the low lidar)
    # from the scan before octomap; otherwise they print as a black trail along
    # the path. Only octomap consumes the filtered cloud; SLAM is untouched.
    range_filter_node = Node(
        package='fast_lio',
        executable='cloud_range_filter.py',
        name='cloud_range_filter',
        output='screen',
        condition=IfCondition(occupancy),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/cloud_registered_body',
            'output_topic': '/cloud_registered_body_filtered',
            'min_range': self_hit_range,
            'ror_min_neighbors': ror_min_neighbors,
            'ror_radius': ror_radius,
        }],
    )

    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        condition=IfCondition(occupancy),
        remappings=[('cloud_in', '/cloud_registered_body_filtered')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map_level',
            'base_frame_id': 'base_footprint',
            'resolution': 0.05,
            'sensor_model.max_range': max_range,
            # Drop isolated single occupied voxels (stray beam noise / reflections /
            # points seen through wall gaps) so they don't pepper the map outside walls.
            'filter_speckles': True,
            'occupancy_min_z': occ_min_z,
            'occupancy_max_z': occ_max_z,
            # Segment the floor as ground (free) instead of obstacle. The map has
            # a residual ~1.5 deg tilt (FAST-LIO pitch drift), so a fixed z-band
            # alone can't separate floor from low obstacles across the whole map;
            # plane segmentation classifies the ground locally regardless of tilt.
            # Runs in base_frame_id (base_footprint), which sits ON the floor.
            'filter_ground_plane': True,
            'ground_filter.distance': 0.05,       # inlier band around fitted plane
            'ground_filter.angle': 0.15,          # rad; tolerant of the ~1.5 deg tilt
            'ground_filter.plane_distance': 0.12,  # plane must be within this of base z=0
            'latch': True,
        }],
    )

    # ros-humble-gtsam (4.2.0, /opt/ros/humble/lib) got installed alongside the
    # pre-existing apt libgtsam4 (4.1.1, /usr/lib) this package is written
    # against (uses the 4.1.x-only Pose3::identity() API). /opt/ros/humble/lib
    # is earlier on LD_LIBRARY_PATH, so without this override pgo_node builds
    # against the 4.1.1 headers but loads the ABI-incompatible 4.2.0 .so at
    # runtime -- segfaults inside GTSAM's own NoiseModelFactor1::unwhitenedError
    # on the very first ISAM2 update. Prepending /usr/lib here forces it back
    # onto the library that actually matches what it was compiled against.
    pgo_additional_env = {
        'LD_LIBRARY_PATH':
            '/usr/lib/x86_64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')
    }

    pgo_node = Node(
        package='fastlio_lc_pgo',
        executable='pgo_node',
        name='laserPGO',
        output='screen',
        additional_env=pgo_additional_env,
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_directory': save_directory,
            'cloud_topic': '/cloud_registered_body',
            'odom_topic': '/Odometry',

            # PGO's optimized map/path/odom live in the loop-closure-corrected
            # "map" frame; pgo_map_odom_bridge turns the offset between this and
            # FAST-LIO's odom into the REP-105 map -> odom transform.
            'map_frame': 'map',

            # keyframe selection
            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 10.0,

            # Scan Context (indoor, Unitree L2: 30 m det_range -> same
            # indoor radius as the corridor benchmark run)
            'sc_dist_thres': 0.4,
            'sc_max_radius': 20.0,

            # loop closure
            'historyKeyframeSearchRadius': 1.5,
            'historyKeyframeSearchTimeDiff': 30.0,
            'historyKeyframeSearchNum': 20,
            'speedFactor': 1.0,
            'loopClosureFrequency': 4.0,
            'graphUpdateFrequency': 2.0,
            'graphUpdateTimes': 5,
            'loopNoiseScore': 0.1,
            'vizmapFrequency': 10.0,
            'loopFitnessScoreThreshold': 0.3,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_cfg_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_occupancy_cmd)
    ld.add_action(declare_occ_min_z_cmd)
    ld.add_action(declare_occ_max_z_cmd)
    ld.add_action(declare_self_hit_range_cmd)
    ld.add_action(declare_max_range_cmd)
    ld.add_action(declare_ror_neighbors_cmd)
    ld.add_action(declare_ror_radius_cmd)
    ld.add_action(sensor_tf_launch)
    ld.add_action(fast_lio_launch)
    ld.add_action(pgo_node)
    ld.add_action(pgo_map_odom_bridge)
    ld.add_action(range_filter_node)
    ld.add_action(octomap_node)
    return ld
