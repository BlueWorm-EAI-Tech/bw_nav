"""MID360 3D localization, automatic relocalization, Nav2 and chassis bridge."""

import os
import sys
import math
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from bw_utils import get_robot_sn
from bw_utils.robot_profile import (
    load_robot_profile,
    normalize_robot_version,
    robot_profile_to_pelvis_params,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.descriptions import Parameter
from nav2_common.launch import RewrittenYaml


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _plugin_package(plugin):
    """Package of a pluginlib class name in ``pkg/Class`` or ``pkg::Class`` form."""
    text = str(plugin)
    for separator in ("/", "::"):
        if separator in text:
            return text.split(separator, 1)[0].strip()
    return ""


def _costmap_layer_packages(params_path):
    """Package names behind every costmap layer in a params file."""
    try:
        data = yaml.safe_load(Path(params_path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return set()
    packages = set()
    for node in ("global_costmap", "local_costmap"):
        params = (data.get(node) or {}).get(node, {}).get("ros__parameters", {})
        for layer in params.get("plugins", []):
            packages.add(_plugin_package((params.get(layer) or {}).get("plugin", "")))
    packages.discard("")
    return packages


def _package_available(package):
    try:
        get_package_share_directory(package)
    except Exception:  # PackageNotFoundError and unusable prefixes alike
        return False
    return True


def _require_layer_packages(params_path):
    """Fail before bringup when a costmap layer plugin package is absent.

    pluginlib reports a missing package only while a costmap configures, inside
    a lifecycle transition, and the message names the plugin class rather than
    the ROS package to install; the bringup then leaves half configured nodes
    behind (and, once the layer stack aborts, warns that no inflation layer is
    present). Check the ament index up front so the operator gets the command.
    """
    missing = [
        package for package in sorted(_costmap_layer_packages(params_path))
        if not _package_available(package)
    ]
    if not missing:
        return
    distro = os.environ.get("ROS_DISTRO", "humble")
    commands = "\n".join(
        f"  sudo apt install ros-{distro}-{package.replace('_', '-')}"
        for package in missing)
    raise RuntimeError(
        f"{params_path} selects costmap layers from missing package(s): "
        f"{', '.join(missing)}\n"
        f"Install them in the environment that runs Nav2:\n{commands}")


def _robot_urdf_path(context):
    """Absolute path of the URDF describing the chassis that is running."""
    argument = LaunchConfiguration("robot_urdf").perform(context).strip()
    if argument:
        return Path(argument).expanduser()
    urdf_dir = Path(get_package_share_directory("bw_std_description")) / "urdf"
    # Prefer the navigation variant (ground-aligned base_footprint plus a
    # livox_frame) and fall back to the vendor file when it is absent.
    nav_urdf = urdf_dir / "std_v6_nav.urdf"
    return nav_urdf if nav_urdf.is_file() else urdf_dir / "std_v6.urdf"


_FOOTPRINT_MODULES = {}
_FOOTPRINT_HULLS = {}


def _footprint_module():
    """Import urdf_footprint.py from the source tree or the share directory."""
    cached = _FOOTPRINT_MODULES.get("module")
    if cached is not None:
        return cached
    script_dir = Path(__file__).resolve().parents[1] / "scripts"
    if not (script_dir / "urdf_footprint.py").is_file():
        # Copied (non symlink) install: use the copy in the package share dir.
        try:
            script_dir = (
                Path(get_package_share_directory("bw_fastlio_nav2")) / "scripts"
            )
        except Exception:
            script_dir = None
    if script_dir is None or not (script_dir / "urdf_footprint.py").is_file():
        raise RuntimeError("urdf_footprint.py is not installed")
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))
    import urdf_footprint

    _FOOTPRINT_MODULES["module"] = urdf_footprint
    return urdf_footprint


def _hull_polygon(context):
    """Convex hull of the chassis in base_footprint, before any padding.

    The hull is cached so the costmap footprint and the collision_monitor
    polygon share a single parse of the very same meshes.
    """
    urdf_path = _robot_urdf_path(context)
    key = str(urdf_path)
    if key not in _FOOTPRINT_HULLS:
        _FOOTPRINT_HULLS[key] = _footprint_module().compute_polygon(
            urdf_path, padding=0.0)
    return _FOOTPRINT_MODULES["module"], _FOOTPRINT_HULLS[key]


def _costmap_footprint(context):
    """Derive the Nav2 footprint from the URDF, or "" to keep the YAML value.

    Nav2 accepts only a flat vertex list, so the convex hull of every collision
    mesh is computed here at launch time and injected through RewrittenYaml.
    Any failure is reported and leaves the footprint from the parameter file in
    place, so a broken model never blocks navigation.
    """
    override = LaunchConfiguration("costmap_footprint").perform(context).strip()
    if override:
        return override
    if not _as_bool(context, "costmap_footprint_from_urdf"):
        return ""
    padding = float(LaunchConfiguration("costmap_footprint_padding").perform(context))
    logger = get_logger("mid360_nav2_localization")
    try:
        module, hull = _hull_polygon(context)
        polygon = module.offset_convex(hull["hull"], padding)
    except Exception as exc:  # missing package, unreadable mesh, bad model
        logger.warning(f"Costmap footprint stays as configured ({exc})")
        return ""
    logger.info(
        f"Costmap footprint from {hull['urdf']}: {len(polygon)} vertices, "
        f"{abs(module.signed_area(polygon)):.4f} m^2, padding {padding:.3f} m")
    return module.format_footprint(polygon)


def _collision_monitor_polygon(context):
    """Flat RadarStopZone points list, or None to keep the YAML value.

    collision_monitor declares ``points`` as a double array, which a parameter
    file rewrite cannot express (RewrittenYaml only replaces scalars), so the
    polygon travels as a command line parameter override on the node instead.
    """
    override = LaunchConfiguration("collision_monitor_polygon").perform(context).strip()
    if override:
        text = override.replace("[", " ").replace("]", " ").replace(",", " ")
        return [float(item) for item in text.split()]
    if not _as_bool(context, "collision_monitor_polygon_from_urdf"):
        return None
    padding = float(
        LaunchConfiguration("collision_monitor_polygon_padding").perform(context))
    logger = get_logger("mid360_nav2_localization")
    try:
        module, hull = _hull_polygon(context)
        polygon = module.offset_convex(hull["hull"], padding)
    except Exception as exc:
        logger.warning(f"RadarStopZone polygon stays as configured ({exc})")
        return None
    logger.info(
        f"RadarStopZone polygon from {hull['urdf']}: {len(polygon)} vertices, "
        f"{abs(module.signed_area(polygon)):.4f} m^2, padding {padding:.3f} m")
    # boost::geometry treats a clockwise ring as the positive orientation, so
    # hand collision_monitor the clockwise winding of the (counter clockwise)
    # hull and let bg::correct() be a no-op.
    return [float(value) for point in polygon[::-1] for value in point]


def _launch_setup(context):
    integration_share = get_package_share_directory("bw_fastlio_nav2")
    bt_share = Path(__file__).resolve().parents[1] / "behavior_trees"
    localization_share = get_package_share_directory("lidar_localization_ros2")
    fast_lio_share = get_package_share_directory("fast_lio")
    livox_share = get_package_share_directory("livox_ros_driver2")
    nav2_share = get_package_share_directory("nav2_bringup")

    pcd_map = Path(LaunchConfiguration("pcd_map").perform(context)).expanduser().resolve()
    map_yaml = Path(LaunchConfiguration("map_yaml").perform(context)).expanduser().resolve()
    relocalization_yaml = Path(
        LaunchConfiguration("relocalization_yaml").perform(context)
    ).expanduser().resolve()
    localization_params = Path(
        LaunchConfiguration("localization_params").perform(context)
    ).expanduser().resolve()
    nav2_params = Path(LaunchConfiguration("nav2_params").perform(context)).expanduser().resolve()
    for label, path in (
        ("pcd_map", pcd_map),
        ("map_yaml", map_yaml),
        ("relocalization_yaml", relocalization_yaml),
        ("localization_params", localization_params),
        ("nav2_params", nav2_params),
    ):
        if not path.is_file():
            raise RuntimeError(f"{label} does not exist: {path}")

    # Costmap plugins are loaded inside a lifecycle transition, so a missing
    # package aborts the bringup with an unrelated looking pluginlib error.
    _require_layer_packages(nav2_params)

    rewrites = {
        "default_nav_to_pose_bt_xml": os.path.join(
            str(bt_share),
            "navigate_to_pose_forward_only.xml",
        ),
        "default_nav_through_poses_bt_xml": os.path.join(
            integration_share,
            "behavior_trees",
            "navigate_through_poses_forward_only.xml",
        ),
    }
    # One string feeds both costmaps: RewrittenYaml walks every leaf key, so
    # global_costmap and local_costmap pick up the same outline.
    footprint = _costmap_footprint(context)
    if footprint:
        rewrites["footprint"] = footprint

    navigation_params = RewrittenYaml(
        source_file=str(nav2_params),
        param_rewrites=rewrites,
        convert_types=True,
    )

    # collision_monitor reads RadarStopZone.points as a double array, which the
    # YAML rewrite cannot produce, so the URDF outline is passed as a node
    # parameter override. The same hull feeds both, only the margin differs.
    stop_zone = _collision_monitor_polygon(context)
    collision_parameters = [navigation_params]
    if stop_zone:
        # A dict would be dumped to a temporary YAML file with a Python tuple
        # tag, which rclcpp refuses to parse; Parameter expands to
        # "-p RadarStopZone.points:=..." and wins over the parameter file.
        collision_parameters.append(Parameter("RadarStopZone.points", stop_zone))

    protocol = LaunchConfiguration("protocol_version").perform(context).lower()
    if protocol not in ("v2", "v3"):
        raise RuntimeError(
            "protocol_version must be either v2 or v3"
        )

    # The serial node speaks one more dialect than the Nav2 bridge does:
    # v3_chassis (standalone chassis payloads 0x21/0x22). Keep it on a separate
    # argument so protocol_version can stay v2/v3 for the bridge.
    mantis_protocol = (
        LaunchConfiguration("mantis_protocol_version").perform(context).strip().lower()
    )
    if mantis_protocol not in ("v1", "v2", "v3", "v3_chassis"):
        raise RuntimeError(
            "mantis_protocol_version must be one of v1, v2, v3, v3_chassis"
        )

    robot_version = normalize_robot_version(
        LaunchConfiguration("robot_version").perform(context)
    )
    robot_profile = load_robot_profile(robot_version)
    pelvis_params = robot_profile_to_pelvis_params(robot_profile)
    robot_sn = str(get_robot_sn()).strip().strip("/")
    use_sim_time = LaunchConfiguration("use_sim_time")
    auto_relocalization = LaunchConfiguration("auto_relocalization")

    # MID-360 and MID-360S ship different driver launches (and config blocks).
    # install.sh rewrites this name after detecting the model; 360S stays the
    # default so a workspace that never ran install.sh behaves as before.
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360s_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_lidar")),
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_fastlio")),
        parameters=[
            os.path.join(fast_lio_share, "config", "mid360.yaml"),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                # The mapping profile keeps these enabled to build the world
                # map. Navigation only consumes odometry and the registered
                # body cloud, while the single-threaded FAST-LIO node would
                # otherwise accumulate, serialize, and publish an ever-growing
                # map every second, stalling LiDAR callbacks on Jetson.
                "publish.map_en": False,
                "publish.path_en": False,
                "pcd_save.pcd_save_en": False,
            },
        ],
    )

    # The measured mounting is base_footprint -> body = (0.14, 0, 0.18)
    # on the v6 flat-top chassis, with aligned axes, so publish its inverse.
    body_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_footprint",
        arguments=[
            "--x", LaunchConfiguration("body_to_base_x"),
            "--y", LaunchConfiguration("body_to_base_y"),
            "--z", LaunchConfiguration("body_to_base_z"),
            "--roll", LaunchConfiguration("body_to_base_roll"),
            "--pitch", LaunchConfiguration("body_to_base_pitch"),
            "--yaw", LaunchConfiguration("body_to_base_yaw"),
            "--frame-id", "body",
            "--child-frame-id", "base_footprint",
        ],
    )

    planar_odom = Node(
        package="bw_fastlio_nav2",
        executable="fastlio_planar_odom.py",
        name="fastlio_planar_odom",
        output="screen",
        parameters=[{
            "input_topic": "/Odometry",
            "output_topic": "/nav2_odom",
            "base_frame_id": "base_footprint",
            "body_to_base_x": ParameterValue(
                LaunchConfiguration("body_to_base_x"), value_type=float
            ),
            "body_to_base_y": ParameterValue(
                LaunchConfiguration("body_to_base_y"), value_type=float
            ),
            "body_to_base_z": ParameterValue(
                LaunchConfiguration("body_to_base_z"), value_type=float
            ),
            "body_to_base_yaw": ParameterValue(
                LaunchConfiguration("body_to_base_yaw"), value_type=float
            ),
            "filter_alpha": 0.35,
            "max_pose_step_m": 0.40,
            "max_yaw_step_rad": 0.60,
            "max_linear_speed_mps": 1.0,
            "max_angular_speed_rad_s": 1.2,
        }],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                localization_share, "launch", "mid360_legged_localization.launch.py"
            )
        ),
        launch_arguments={
            "localization_param_dir": str(localization_params),
            "map_path": str(pcd_map),
            "registration_method": LaunchConfiguration("registration_method"),
            "ndt_num_threads": LaunchConfiguration("ndt_num_threads"),
            "global_frame_id": "map",
            "odom_frame_id": "camera_init",
            "base_frame_id": "body",
            "use_sim_time": use_sim_time,
            "enable_map_odom_tf": "true",
            "cloud_topic": "/cloud_registered_body",
            "imu_topic": "/livox/imu",
            "use_imu_preintegration": "false",
            "use_continuous_time_deskew": "false",
            "set_initial_pose": LaunchConfiguration("set_initial_pose"),
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_z": LaunchConfiguration("initial_pose_z"),
            "initial_pose_qx": LaunchConfiguration("initial_pose_qx"),
            "initial_pose_qy": LaunchConfiguration("initial_pose_qy"),
            "initial_pose_qz": LaunchConfiguration("initial_pose_qz"),
            "initial_pose_qw": LaunchConfiguration("initial_pose_qw"),
            "publish_lidar_tf": "false",
            "publish_imu_tf": "false",
        }.items(),
    )

    # Patchwork++ runs on each body-frame scan. Keeping the cloud in body makes
    # the ray-clearing origin follow the lidar; /Laser_map is never used by the
    # online dynamic layer.
    live_obstacles = Node(
        package="bw_fastlio_nav2",
        executable="laser_map_ground_grid",
        name="live_ground_obstacle_segmentation",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "input_topic": "/cloud_registered_body",
            "grid_topic": "/obstacle_grid_live",
            "ground_topic": "/ground_points_live",
            "obstacle_topic": "/obstacle_points_live",
            "voxel_leaf_size": 0.05,
            # ground_relative = z_body + sensor_height (patchwork hpp:144);
            # the livox sits 0.18 m above base_footprint, so use the real value
            # or the whole height band is shifted by 3 cm.
            "sensor_height": 0.18,
            "num_iter": 3,
            "num_lpr": 20,
            "num_min_points": 10,
            "seed_threshold": 0.12,
            "distance_threshold": 0.02,
            "vertical_seed_threshold": 0.25,
            "vertical_distance_threshold": 0.02,
            "min_range": 0.05,
            "max_range": 15.0,
            "uprightness_threshold": 0.866,
            "enable_reflected_noise_removal": False,
            "enable_vertical_plane_fitting": True,
            "enable_temporal_ground_revert": True,
            # Ground-relative height band. Measured from the URDF meshes: the
            # chassis top is 0.0455 + 0.175 = 0.220 m, the lidar sits at 0.18 m
            # and its housing reaches ~0.24 m. So for this base "above the lidar"
            # and "above the body" are the same line, and the band is
            # 0.03..0.30 m: floor to a few cm above the sensor. Anything higher
            # (table tops, ceilings, signage) cannot be hit by this robot and
            # must not create costmap obstacles.
            "obstacle_min_height": 0.03,
            "obstacle_max_height": 0.30,
            # Keep these independently tunable. The defaults were selected
            # from side-by-side tests on the dense MID360 body-frame cloud.
            "outlier_radius": ParameterValue(
                LaunchConfiguration("live_outlier_radius"), value_type=float
            ),
            "outlier_min_neighbors": ParameterValue(
                LaunchConfiguration("live_outlier_min_neighbors"), value_type=int
            ),
            "resolution": 0.05,
            "min_obstacle_neighbors": 3,
            # The input is already 10 Hz. Throttling on receive time at the
            # input period drops every frame that arrives a few ms early:
            # delivery jitter turned the 10 Hz body cloud into ~6 Hz on the
            # Orin (4 Hz under full load), so the obstacle stream must not be
            # rate-limited here. One frame costs ~3 ms.
            "min_update_period_sec": 0.0,
            # Input is in the MID360/body frame; base_footprint is 0.14 m
            # behind it. The URDF collision hull is x=[-0.454, +0.091],
            # y=[-0.313, +0.313] in this frame, and RadarStopZone is that hull
            # grown by 5 cm. The self-filter must cover the whole hull: any
            # surviving body surface point lands inside RadarStopZone, and
            # collision_monitor then holds STOP forever. In STOP it only
            # publishes zero velocity for stop_pub_timeout (0.5 s) and goes
            # silent afterwards, so nav_cmd_relay times out and sends zeros to
            # the chassis - the base never moves.
            "self_filter_enabled": True,
            "self_filter_min_x": -0.50,
            "self_filter_max_x": 0.17,
            "self_filter_min_y": -0.35,
            "self_filter_max_y": 0.35,
            # Second-stage field-of-view gate. /obstacle_points_live keeps the
            # full ground- and self-filtered cloud; only the FOV-limited copy is
            # consumed by the two costmaps. Defaults keep everything at or below
            # the LiDAR plane inside +-135 deg of the forward axis.
            "fov_filter_enabled": ParameterValue(
                LaunchConfiguration("fov_filter_enabled"), value_type=bool
            ),
            # Must stay in sync with the costmap sources in nav2_omni.yaml.
            "fov_obstacle_topic": LaunchConfiguration("fov_obstacle_topic"),
            "fov_min_azimuth_deg": ParameterValue(
                LaunchConfiguration("fov_min_azimuth_deg"), value_type=float
            ),
            "fov_max_azimuth_deg": ParameterValue(
                LaunchConfiguration("fov_max_azimuth_deg"), value_type=float
            ),
            "fov_min_elevation_deg": ParameterValue(
                LaunchConfiguration("fov_min_elevation_deg"), value_type=float
            ),
            "fov_max_elevation_deg": ParameterValue(
                LaunchConfiguration("fov_max_elevation_deg"), value_type=float
            ),
            "fov_min_range": ParameterValue(
                LaunchConfiguration("fov_min_range"), value_type=float
            ),
            "fov_max_range": ParameterValue(
                LaunchConfiguration("fov_max_range"), value_type=float
            ),
            # Must stay false: the costmaps look the transform up at this
            # cloud's stamp, and FAST-LIO's TF is stamped with LiDAR time.
            "restamp_fov_output_to_now": ParameterValue(
                LaunchConfiguration("restamp_fov_output_to_now"),
                value_type=bool,
            ),
            "fov_mount_roll_deg": ParameterValue(
                LaunchConfiguration("fov_mount_roll_deg"), value_type=float
            ),
            "fov_mount_pitch_deg": ParameterValue(
                LaunchConfiguration("fov_mount_pitch_deg"), value_type=float
            ),
            "fov_mount_yaw_deg": ParameterValue(
                LaunchConfiguration("fov_mount_yaw_deg"), value_type=float
            ),
        }],
    )

    global_localization = Node(
        package="lidar_localization_ros2",
        executable="global_localization_node.py",
        name="global_localization_node",
        output="screen",
        condition=IfCondition(auto_relocalization),
        parameters=[{
            "occupancy_yaml": str(relocalization_yaml),
            "cloud_topic": "/cloud_registered_body",
            "global_frame_id": "map",
            "map_path": str(pcd_map),
            "enable_registration_scoring": True,
            "registration_score_gate": 4.0,
            # Publish the NDT-refined winner, but only when its BBS occupancy
            # score does not regress. The guard keeps a bad refinement from
            # replacing a coarse cell that was already the better pose.
            "registration_refine_candidates": True,
            "registration_refine_bbs_guard": True,
            "registration_refine_bbs_margin": 0.01,
            "independent_sampling": True,
            "sampling_seed": 0,
            "registration_seed_z_m": ParameterValue(
                LaunchConfiguration("global_seed_z"), value_type=float
            ),
            "ndt_scan_voxel_leaf_size": 0.5,
            "ndt_target_voxel_leaf_size": 0.2,
            "angular_resolution_deg": 10.0,
            "max_scan_points": 1024,
            "pyramid_depth": 4,
            "max_candidates": 8,
            "nms_radius_m": 2.0,
            "use_cpp_backend": True,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    startup_initialization = Node(
        package="lidar_localization_ros2",
        executable="startup_initialization_node.py",
        name="startup_initialization",
        output="screen",
        condition=IfCondition(auto_relocalization),
        parameters=[{
            "map_path": str(pcd_map),
            "pose_state_path": LaunchConfiguration("pose_state_path"),
            "restore_saved_pose": ParameterValue(
                LaunchConfiguration("restore_saved_pose"), value_type=bool
            ),
            "initial_pose_preconfigured": ParameterValue(
                LaunchConfiguration("set_initial_pose"), value_type=bool
            ),
            # Preserve this deployment's stationary multi-scan startup policy.
            "enable_odom_motion_compensation": False,
            "max_registration_fitness_ratio": 0.0,
            "saved_pose_max_age_sec": 86400.0,
            "saved_pose_score_threshold": 0.6,
            "saved_pose_max_refine_displacement_m": 2.0,
            "enable_global_initialization": True,
            "require_global_registration_scoring": True,
            "require_candidate_fitness": True,
            "max_candidate_registration_fitness": 0.4,
            "min_query_input_points": 500,
            "min_query_scan_points": 150,
            "min_candidate_bbs_score": 0.15,
            "cloud_topic": "/cloud_registered_body",
            "pose_topic": "/localization/pose_with_covariance",
            "alignment_status_topic": "/alignment_status",
            "initialpose_topic": "/initialpose",
            "query_service": "/global_localization_node/query",
            "global_frame_id": "map",
            "default_z_m": ParameterValue(
                LaunchConfiguration("global_seed_z"), value_type=float
            ),
            "min_candidate_score": 0.15,
            # The current PGO map contains similar corridor hypotheses whose
            # registration scores are often only 0.04 apart. Keep a per-scan
            # ambiguity gate, then rely on a much tighter multi-scan pose
            # consensus to reject the occasional alias.
            "min_score_margin": 0.03,
            "query_timeout_sec": 45.0,
            "verification_fitness_threshold": 4.0,
            "verification_samples": 3,
            "global_consensus_samples": 2,
            "global_consensus_translation_m": 0.50,
            "global_consensus_yaw_deg": 15.0,
            "max_global_attempts": 8,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    reinit_supervisor = Node(
        package="lidar_localization_ros2",
        executable="reinitialization_supervisor_node.py",
        name="reinitialization_supervisor_node",
        output="screen",
        condition=IfCondition(auto_relocalization),
        parameters=[{
            "query_service": "/global_localization_node/query",
            "alignment_status_topic": "/alignment_status",
            "initialpose_topic": "/initialpose",
            "global_frame_id": "map",
            "min_candidate_score": 0.15,
            "max_attempts": 4,
            "query_timeout_sec": 45.0,
            "settle_timeout_sec": 20.0,
            "recovery_confirmation_samples": 3,
            "recovery_fitness_threshold": 4.0,
            "confirm_cross_check": True,
            "use_odom_bridge_candidate": True,
            "odom_bridge_max_attempts": 1,
            "reset_default_z_m": ParameterValue(
                LaunchConfiguration("global_seed_z"), value_type=float
            ),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            str(nav2_params),
            {
                "yaml_filename": str(map_yaml),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        # RViz's Navigation panel looks for this standard manager name when
        # displaying its Localization state. The 3D localizer is independently
        # lifecycle-activated by its own launch file; this manager owns the map.
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": navigation_params,
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        emulate_tty=True,
        parameters=collision_parameters,
    )
    collision_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": True,
            "node_names": ["collision_monitor"],
        }],
    )

    navigation_startup_gate = Node(
        package="bw_fastlio_nav2",
        executable="navigation_startup_gate.py",
        name="navigation_startup_gate",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "pose_topic": "/localization/pose_with_covariance",
            "global_frame": "map",
            "odom_frame": "camera_init",
            "base_frame": "base_footprint",
            "stable_samples": 3,
            "stable_hold_sec": 0.5,
            "require_startup_verification": ParameterValue(
                auto_relocalization, value_type=bool
            ),
            "startup_status_topic": "/startup_initialization/status",
            "startup_timeout_sec": 600.0,
        }],
    )

    def start_navigation_after_localization(event, launch_context):
        if launch_context.is_shutdown:
            return []
        if event.returncode != 0:
            get_logger("navigation_startup_gate").error(
                "Navigation startup gate failed; Nav2 remains stopped "
                f"(exit code {event.returncode})"
            )
            return []
        return [navigation, collision_monitor, collision_lifecycle]

    navigation_startup_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_startup_gate,
            on_exit=start_navigation_after_localization,
        )
    )

    serial = Node(
        package="bw_serial",
        executable="mantis_comm_node",
        name="mantis_comm_node",
        namespace=robot_sn,
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_serial")),
        parameters=[{
            "port_name": LaunchConfiguration("port_name"),
            "baud_rate": LaunchConfiguration("baud_rate"),
            "publish_frequency": LaunchConfiguration("serial_frequency"),
            "serial_command_diagnostics": True,
            "chassis_cmd_vel_timeout_sec": 0.25,
            "protocol_version": mantis_protocol,
            "robot_version": robot_profile.robot_version,
            "pelvis_startup_init_enabled": False,
            **pelvis_params,
        }],
    )
    # Minimal relay: forward /cmd_vel verbatim, emit zero velocity when the
    # command stream goes stale, and request the V3 arm/lower-body enable. All
    # of the old speed shaping (V2 calibration tables, 0.427 m/s breakaway
    # boost, lateral suppression, rear-turn latching, goal-rotation shaping,
    # output clamping) used to live here and handed the chassis commands Nav2
    # never asked for.
    # The relay takes the velocity_smoother output directly, so collision_monitor
    # is bypassed: /cmd_vel_safe is still published but nothing consumes it.
    cmd_relay = Node(
        package="bw_fastlio_nav2",
        executable="nav_cmd_relay.py",
        name="nav_cmd_relay",
        namespace=robot_sn,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "enable_motion": _as_bool(context, "enable_motion"),
            "input_topic": "/cmd_vel",
            "output_topic": "Teleop/cmd_vel",
            "power_service": "set_ctrl_src",
            "command_timeout_sec": 0.25,
            "escape_topic": "/escape_cmd_vel",
            "linear_scale": float(
                LaunchConfiguration("linear_scale").perform(context)
            ),
            # Sideways speed gets the same m/s -> normalised conversion.
            "lateral_scale": 3.0,
            # Look-ahead bumper: sweep the direction the command points at and
            # brake before contact. Everything else in the stack is either blind
            # (RPP's use_collision_detection cannot be enabled, the body already
            # sits inside the lethal+inscribed band of a 1 m corridor) or
            # last-moment (RadarStopZone is hull + 5 cm, so the first reaction
            # is the impact). Only lethal cells count, so the 253 band along a
            # corridor does not pin the base in place.
            "bumper_enabled": True,
            "bumper_costmap_topic": "/local_costmap/costmap",
            "bumper_stop_m": 0.28,
            "bumper_slow_m": 0.90,
            "bumper_half_width_m": 0.32,
            "bumper_blocked_cost": 100,
        }],
    )

    # Drives the robot out of a corner when Nav2 asks for motion and the base
    # has not moved for a second and a half. It bypasses collision_monitor,
    # which is the only way out: the monitor holds /cmd_vel_safe at zero for as
    # long as an obstacle point stays inside RadarStopZone, and the point stays
    # there while the robot is stuck. Bounded to a few seconds and one short
    # leg in the freest direction of the local costmap.
    escape_node = Node(
        package="bw_fastlio_nav2",
        executable="escape_node.py",
        name="escape_node",
        namespace=robot_sn,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "costmap_topic": "/local_costmap/costmap",
            "odom_topic": "/nav2_odom",
            "cmd_topic": "/cmd_vel",
            "output_topic": "/escape_cmd_vel",
            "trigger_sec": 1.5,
            # Escape may use the full envelope the base tracks, not the old
            # 0.12 m/s crawl that cleared 0.24 m per attempt. 0.30 m/s is the
            # same command Nav2 sends, and nav_cmd_relay's linear_scale 3.0
            # applies to it identically.
            "drive_speed": 0.30,
            "turn_vel": 1.00,
            "drive_sec": 0.70,
            # Must stay >= drive_sec * drive_speed + abort_free_m (0.21+0.30);
            # the node raises at startup if it does not.
            "min_free_m": 0.55,
            "abort_free_m": 0.30,
            # Omni base: leave a corner sideways instead of rotating in place.
            "use_lateral_motion": True,
            "lateral_speed": 0.20,
            # Back out first: the corridor behind is the one the robot came in
            # through, so it is already known to be clear.
            "prefer_reverse": True,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(integration_share, "rviz", "mid360_nav2.rviz"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Optional robot body for RViz. The URDF only supplies the static link
    # chain (base_footprint -> base_link -> wheels); the live pose still comes
    # from the body -> base_footprint static TF above.
    robot_state_publisher = None
    robot_joint_states = None
    if _as_bool(context, "use_robot_model"):
        robot_description = ""
        urdf_path = None
        try:
            urdf_path = _robot_urdf_path(context)
            robot_description = urdf_path.read_text(encoding="utf-8")
        except Exception as exc:  # missing package or unreadable file
            get_logger("mid360_nav2_localization").warning(
                "Robot model disabled (%s); RViz will show no robot body", exc
            )
        else:
            robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            )
            # Continuous wheel joints need a source of /joint_states before
            # robot_state_publisher will broadcast their transforms.
            robot_joint_states = Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                arguments=[str(urdf_path)],
            )

    # These are not a dependency graph: every node except Nav2 waits for its
    # topics, services or TF on its own.  The original 2/4/6/7/8 s staging kept
    # the one-time map loads and the relocalization verification from
    # overlapping the LiDAR driver's spin-up.  All-at-once is now the default
    # (verified on the NUC); pass startup_all_at_once:=false to restore the
    # staged timers.  Nav2 still starts only after the gate exits either way.
    #
    # The one sequence worth keeping in mind is startup_initialization: it
    # publishes the saved /initialpose once, after its first LiDAR scan.  At
    # t=0 that scan is still ~3.3 s away (the driver's spin-up), so the
    # localizer is already active and subscribed before the message is sent.
    if _as_bool(context, "startup_all_at_once"):
        startup_actions = [
            fast_lio,
            localization,
            live_obstacles,
            global_localization,
            startup_initialization,
            reinit_supervisor,
            map_server,
            # nav2_lifecycle_manager runs its autostart pass as soon as it is
            # up and does not retry a failed configure call.  map_server is
            # first in this list, but give it 0.5 s so its lifecycle services
            # are up even under a cold-start CPU spike.
            TimerAction(period=0.5, actions=[map_lifecycle]),
            navigation_startup_gate,
            rviz,
        ]
    else:
        startup_actions = [
            TimerAction(period=2.0, actions=[fast_lio]),
            TimerAction(period=4.0, actions=[localization, live_obstacles]),
            TimerAction(
                period=6.0,
                actions=[
                    global_localization,
                    startup_initialization,
                    reinit_supervisor,
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[map_server, map_lifecycle, navigation_startup_gate],
            ),
            TimerAction(period=8.0, actions=[rviz]),
        ]

    return [
        livox_driver,
        body_to_base,
        *([robot_state_publisher] if robot_state_publisher else []),
        *([robot_joint_states] if robot_joint_states else []),
        planar_odom,
        serial,
        cmd_relay,
        escape_node,
        navigation_startup_handler,
        *startup_actions,
    ]


def generate_launch_description():
    package_share = get_package_share_directory("bw_fastlio_nav2")
    workspace = Path(package_share).resolve().parents[3]
    return LaunchDescription([
        # Fast DDS shared-memory endpoints can survive an unclean shutdown and
        # silently drop local lifecycle service replies. Keep this all-local
        # navigation graph on UDP so Nav2 startup cannot hang in change_state().
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        DeclareLaunchArgument(
            "pcd_map",
            default_value=str(workspace / "maps/pgo_session/map_consistent.pcd"),
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=str(workspace / "maps/nav2/pgm_v2/mid360_pgo_csf.yaml"),
        ),
        DeclareLaunchArgument(
            "relocalization_yaml",
            default_value=str(workspace / "maps/nav2/pgm_v2/mid360_pgo_bbs.yaml"),
            description=(
                "Occupancy grid used by the global (BBS) relocalizer. Kept "
                "separate from map_yaml: the navigation map seals every gap and "
                "walls off rooms whose door was never observed, which is good "
                "for the planner but adds invented walls to the scan-matching "
                "affinity field."
            ),
        ),
        DeclareLaunchArgument(
            "localization_params",
            default_value=os.path.join(package_share, "config", "mid360_localization.yaml"),
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(package_share, "config", "nav2_omni.yaml"),
        ),
        DeclareLaunchArgument(
            "pose_state_path",
            default_value="~/.ros/bw_fastlio_nav2/mid360_verified_pose.json",
        ),
        DeclareLaunchArgument("registration_method", default_value="NDT_OMP"),
        DeclareLaunchArgument("ndt_num_threads", default_value="4"),
        DeclareLaunchArgument("global_seed_z", default_value="0.0"),
        DeclareLaunchArgument("set_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_z", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_qx", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_qy", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_qz", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_qw", default_value="1.0"),
        DeclareLaunchArgument("auto_relocalization", default_value="true"),
        DeclareLaunchArgument("restore_saved_pose", default_value="false"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument(
            "startup_all_at_once",
            default_value="true",
            description=(
                "Start every non-Nav2 node immediately. This is the default; "
                "pass false to restore the staged 2/4/6/7/8 s timers. Nav2 "
                "still waits for the startup gate either way."
            ),
        ),
        DeclareLaunchArgument("start_serial", default_value="true"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        # The chassis turns linear.x into roughly a third of that in metres per
        # second: 0.30 in -> 0.097 m/s out, 0.50 in -> 0.171 m/s out, the same
        # slope in every bag recorded so far. MPPI plans in m/s, so a 0.30 m/s
        # command has to leave here as ~0.88 or the arc it drew is three times
        # wider than the one the base actually drives.
        DeclareLaunchArgument("linear_scale", default_value="3.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_robot_model", default_value="true"),
        DeclareLaunchArgument("robot_urdf", default_value=""),
        # The costmaps now run on a fixed 0.10 m planning radius from the YAML
        # (robot_radius), matching the reference nav2_params.yaml: the hull's
        # 0.251 m inscribed band made SmacPlanner reject any start pose that
        # drifted a few centimetres towards an obstacle. Physical safety is not
        # Both costmaps load their footprint dynamically: the convex hull of the
        # URDF is computed at launch time and injected through RewrittenYaml, so
        # the outline follows the model instead of a hand-typed polygon.
        # Set costmap_footprint_from_urdf:=false to keep the YAML value.
        DeclareLaunchArgument("costmap_footprint_from_urdf", default_value="true"),
        # Only used when the footprint comes from the URDF; with the fixed
        # radius in the YAML the padding is applied on top of 0.10 m instead.
        DeclareLaunchArgument("costmap_footprint_padding", default_value="0.0"),
        DeclareLaunchArgument("costmap_footprint", default_value=""),
        # The stop polygon has its own margin, so it gets its own arguments.
        DeclareLaunchArgument(
            "collision_monitor_polygon_from_urdf", default_value="true"),
        DeclareLaunchArgument(
            # 0.05, not 0.12. Hull half width is 0.313 m and the narrowest
            # mapped passage leaves 0.447 m from the centreline to a wall, i.e.
            # 0.134 m of margin. A 0.12 ring is 0.433 m half width: it fits a
            # 1.0 m corridor with 14 mm to spare and stops the robot dead in
            # the middle of a passage as soon as it drifts 2 cm off centre,
            # without touching anything. 0.05 leaves 0.084 m of tolerance.
            "collision_monitor_polygon_padding", default_value="0.05"),
        DeclareLaunchArgument("collision_monitor_polygon", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("robot_version", default_value="2.0"),
        DeclareLaunchArgument("protocol_version", default_value="v3"),
        DeclareLaunchArgument("mantis_protocol_version", default_value="v3"),
        DeclareLaunchArgument("port_name", default_value="/dev/ttyACM0"),
        # USB CDC ignores the UART baud, but 4 Mbps is the highest standard
        # termios value that can carry a 254-byte V2 frame within a 1 ms cycle.
        DeclareLaunchArgument("baud_rate", default_value="4000000"),
        DeclareLaunchArgument("serial_frequency", default_value="1000"),
        DeclareLaunchArgument("body_to_base_x", default_value="-0.14"),
        DeclareLaunchArgument("body_to_base_y", default_value="0.0"),
        DeclareLaunchArgument("body_to_base_z", default_value="-0.18"),
        DeclareLaunchArgument("body_to_base_roll", default_value="0.0"),
        DeclareLaunchArgument("body_to_base_pitch", default_value="0.0"),
        DeclareLaunchArgument("body_to_base_yaw", default_value="0.0"),
        DeclareLaunchArgument("live_outlier_radius", default_value="0.20"),
        DeclareLaunchArgument("live_outlier_min_neighbors", default_value="2"),
        # Field-of-view gate applied after the ground and self filtering. Azimuth
        # is measured around body +x (0 = forward, + = left), elevation from the
        # body x/y plane, and the mount roll/pitch/yaw rotate the cloud into the
        # limiting frame first. -90..0 deg elevation drops everything at or above
        # the LiDAR plane; +-135 deg azimuth keeps the forward 270 deg window.
        DeclareLaunchArgument("fov_filter_enabled", default_value="true"),
        DeclareLaunchArgument(
            "fov_obstacle_topic", default_value="/obstacle_points_fov"),
        DeclareLaunchArgument("fov_min_azimuth_deg", default_value="-90.0"),
        DeclareLaunchArgument("fov_max_azimuth_deg", default_value="90.0"),
        DeclareLaunchArgument("fov_min_elevation_deg", default_value="-90.0"),
        # Elevation gate in the sensor frame. What actually keeps ceilings,
        # table tops and signage out of the costmap is the ground-relative
        # band on the segmentation node (obstacle_min/max_height 0.03..0.30 m,
        # the height of this chassis: 0.220 m body, lidar at 0.18 m), not this
        # gate. This one only has to cover the near field, where the band is
        # not reached yet: a 20 deg cap sees 0.18 + tan(20)*d, i.e. 0.36 m at
        # 0.5 m, so it never limits what the band allows beyond half a metre.
        # A cap of 0.0 would admit only 0.18 m and below and make people
        # invisible again; 90.0 removes the gate entirely (the band still
        # applies).
        DeclareLaunchArgument("fov_max_elevation_deg", default_value="20.0"),
        DeclareLaunchArgument("fov_min_range", default_value="0.05"),
        DeclareLaunchArgument("fov_max_range", default_value="15.0"),
        # /obstacle_points_fov feeds the two costmaps, whose layers resolve
        # camera_init->body at the cloud stamp. FAST-LIO stamps that TF with the
        # LiDAR clock, so this copy must NOT be restamped to host time (the
        # offset was +67738 s in the field): tf2's MessageFilter would drop
        # every message and the voxel layer would mark nothing. The full cloud
        # on /obstacle_points_live stays host-stamped for collision_monitor.
        DeclareLaunchArgument(
            "restamp_fov_output_to_now", default_value="false"),
        DeclareLaunchArgument("fov_mount_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("fov_mount_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("fov_mount_yaw_deg", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
