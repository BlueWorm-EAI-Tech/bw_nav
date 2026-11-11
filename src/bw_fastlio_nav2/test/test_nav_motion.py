import ast
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = PACKAGE_DIR / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from fastlio_planar_odom import (  # noqa: E402
    PlanarPose,
    _compose_base_pose,
    _derive_planar_twist,
    _yaw_from_quaternion,
)
from navigation_startup_gate import (  # noqa: E402
    ReadinessWindow,
    _pose_is_valid,
)
from v2_nav_cmd_bridge import (  # noqa: E402
    LocalizationJumpRecovery,
    RearTurnDirectionLatch,
    RelocalizationGate,
    RotationWatchdog,
    _clamp_forward,
    _inverse_speed_map,
    _map_signed_speed,
    _passthrough_command,
    _path_heading_error,
    _requests_motion,
    _shape_goal_rotation,
    _shape_routine_lateral_command,
    _shape_translation,
    _validate_speed_calibration,
)


def test_localization_jump_recovery_requires_stable_stationary_hold():
    recovery = LocalizationJumpRecovery(hold_sec=5.0, required_stable_samples=3)
    recovery.begin()
    recovery.observe_alignment(True)
    recovery.observe_pose(True, 10.0)
    recovery.observe_pose(True, 11.0)
    recovery.observe_pose(True, 12.0)

    assert not recovery.try_release(14.9, True, True, True)
    assert not recovery.try_release(15.0, True, True, False)
    assert recovery.stable_samples == 0

    recovery.observe_pose(True, 16.0)
    recovery.observe_pose(True, 18.0)
    recovery.observe_pose(True, 20.0)
    assert recovery.try_release(21.0, True, True, True)
    assert not recovery.active


def test_localization_jump_recovery_resets_on_unhealthy_or_unstable_pose():
    recovery = LocalizationJumpRecovery(hold_sec=2.0, required_stable_samples=2)
    recovery.begin()
    recovery.observe_alignment(True)
    recovery.observe_pose(True, 1.0)
    recovery.observe_alignment(False)
    assert recovery.stable_samples == 0

    recovery.observe_alignment(True)
    recovery.observe_pose(True, 2.0)
    recovery.observe_pose(False, 2.5)
    recovery.observe_pose(True, 3.0)
    recovery.observe_pose(True, 4.0)
    assert not recovery.try_release(5.0, False, True, True)
    assert recovery.try_release(5.0, True, True, True)


def test_navigation_readiness_requires_stable_consecutive_samples():
    readiness = ReadinessWindow(min_samples=3, hold_sec=0.5)

    assert not readiness.observe(True, 10.0)
    assert not readiness.observe(True, 10.3)
    assert not readiness.observe(False, 10.4)
    assert not readiness.observe(True, 11.0)
    assert not readiness.observe(True, 11.3)
    assert readiness.observe(True, 11.5)


def test_navigation_readiness_rejects_invalid_pose():
    from geometry_msgs.msg import PoseWithCovarianceStamped

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.orientation.w = 1.0
    assert _pose_is_valid(pose, "map")

    pose.header.frame_id = "camera_init"
    assert not _pose_is_valid(pose, "map")
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = math.nan
    assert not _pose_is_valid(pose, "map")


def test_aligned_sensor_extrinsic_composes_base_pose():
    position, orientation = _compose_base_pose(
        (1.0, 2.0, 0.35),
        (0.0, 0.0, 0.0, 1.0),
        (-0.10, 0.0, -0.35),
        0.0,
    )

    assert position == pytest.approx((0.9, 2.0, 0.0))
    assert _yaw_from_quaternion(orientation) == pytest.approx(0.0)


def test_planar_velocity_is_expressed_in_base_frame():
    previous = PlanarPose(1.0, 0.0, 0.0, math.pi / 2.0)
    current = PlanarPose(2.0, 0.0, 0.30, math.pi / 2.0)

    vx, vy, angular, dt = _derive_planar_twist(previous, current)

    assert (vx, vy, angular, dt) == pytest.approx((0.30, 0.0, 0.0, 1.0))


def test_breakaway_boost_only_applies_while_stationary():
    stationary = _shape_translation(0.10, 0.0, 0.30, 0.02, 0.35, 0.35, True)
    moving = _shape_translation(0.10, 0.0, 0.30, 0.02, 0.35, 0.35, False)
    stopped = _shape_translation(0.01, 0.0, 0.30, 0.02, 0.35, 0.35, True)

    assert stationary == pytest.approx((0.30, 0.0, True))
    assert moving == pytest.approx((0.10, 0.0, False))
    assert stopped == pytest.approx((0.0, 0.0, False))


def test_small_lateral_trim_turns_then_becomes_forward_motion():
    turning = _shape_routine_lateral_command(
        0.0, -0.05, 0.10, 0.08, -0.40, 0.35, 0.25
    )
    aligned = _shape_routine_lateral_command(
        0.20, 0.05, 0.02, 0.08, 0.05, 0.35, 0.25
    )

    assert turning == pytest.approx((0.0, 0.0, -0.25, True, True))
    assert aligned == pytest.approx(
        (math.hypot(0.20, 0.05), 0.0, 0.02, True, False)
    )


def test_moderate_heading_error_keeps_forward_motion_for_smooth_arc():
    shaped = _shape_routine_lateral_command(
        0.20, 0.05, 0.12, 0.08, 0.20, 0.35, 0.25
    )

    assert shaped == pytest.approx(
        (math.hypot(0.20, 0.05), 0.0, 0.12, True, False)
    )


def test_decisive_lateral_obstacle_sidestep_is_preserved():
    assert _shape_routine_lateral_command(
        0.10, 0.08, -0.05, 0.08, 0.50, 0.35, 0.25
    ) == pytest.approx((0.10, 0.08, -0.05, False, False))


def test_final_goal_rotation_is_slow_and_uses_shortest_yaw_direction():
    shaped = _shape_goal_rotation(
        0.0,
        0.0,
        0.20,
        (0.0, 0.0),
        0.30,
        (0.10, 0.0),
        -0.10,
        0.30,
        0.02,
        math.radians(30.0),
        0.08,
        0.10,
    )

    assert shaped == pytest.approx((-0.08, True, -0.40))


def test_final_goal_rotation_stops_inside_tolerance_and_ignores_translation():
    within_tolerance = _shape_goal_rotation(
        0.0, 0.0, -0.20, (0.0, 0.0), 0.0, (0.10, 0.0), 0.05,
        0.30, 0.02, math.radians(30.0), 0.08, 0.10,
    )
    still_translating = _shape_goal_rotation(
        0.03, 0.0, -0.20, (0.0, 0.0), 0.0, (0.10, 0.0), -0.40,
        0.30, 0.02, math.radians(30.0), 0.08, 0.10,
    )

    assert within_tolerance == pytest.approx((0.0, True, 0.05))
    assert still_translating == (-0.20, False, None)


def test_large_final_yaw_error_keeps_normal_rotation_speed():
    shaped = _shape_goal_rotation(
        0.0, 0.0, 0.20, (0.0, 0.0), 1.0, (0.10, 0.0), 0.0,
        0.30, 0.02, math.radians(30.0), 0.08, 0.10,
    )

    assert shaped == pytest.approx((-0.20, False, -1.0))


def test_hardware_boundary_rejects_reverse_and_zero_commands():
    assert _clamp_forward(-0.20, 0.30) == pytest.approx(0.0)
    assert _clamp_forward(0.40, 0.30) == pytest.approx(0.30)
    assert not _requests_motion(-0.20, 0.0, 0.0, 0.02)
    assert _requests_motion(-0.20, 0.0, 0.0, 0.02, allow_reverse=True)
    assert not _requests_motion(0.01, 0.0, 0.0, 0.02)
    assert _requests_motion(0.03, 0.0, 0.0, 0.02)
    assert _requests_motion(0.0, 0.0, 0.05, 0.02)


def test_inverse_speed_map_interpolates_and_extrapolates_measured_curve():
    actual = (0.0, 0.223, 0.271)
    commands = (0.0, 0.427, 0.500)

    assert _inverse_speed_map(0.223, actual, commands) == pytest.approx(0.427)
    assert _inverse_speed_map(0.250, actual, commands) == pytest.approx(0.4680625)
    assert _inverse_speed_map(0.300, actual, commands) > 0.500


def test_signed_speed_map_uses_direction_specific_feedback():
    positive = ((0.0, 0.223, 0.271), (0.0, 0.427, 0.500))
    negative = ((0.0, 0.238, 0.293), (0.0, 0.427, 0.500))

    positive_command = _map_signed_speed(0.20, *positive, *negative)
    negative_command = _map_signed_speed(-0.20, *positive, *negative)

    assert positive_command == pytest.approx(0.3829596)
    assert negative_command == pytest.approx(-0.3588235)


def test_speed_calibration_rejects_non_monotonic_points():
    with pytest.raises(ValueError, match="strictly increasing"):
        _validate_speed_calibration((0.0, 0.2, 0.1), (0.0, 0.4, 0.5), "x")


def test_rotation_watchdog_trips_on_accumulated_angle():
    watchdog = RotationWatchdog(0.08, 1.0, 100.0, max_sample_dt=1.0)

    assert not watchdog.update(0.0, 0.15, 0.4)
    assert not watchdog.update(1.0, 0.15, 0.4)
    assert not watchdog.update(2.0, 0.15, 0.4)
    assert watchdog.update(3.0, 0.15, 0.4)
    assert watchdog.measured_angle == pytest.approx(1.2)


def test_rear_path_latches_first_turn_direction_across_pi_wrap():
    error = _path_heading_error(
        0.0,
        0.0,
        0.0,
        ((0.0, 0.0), (-0.6, 0.01)),
        0.5,
    )
    latch = RearTurnDirectionLatch(2.75, 2.20, 0.35, 0.02, 0.04)

    assert error == pytest.approx(math.atan2(0.01, -0.6))
    assert latch.update(error)
    assert latch.direction == 1
    assert latch.alignment_active
    assert not latch.update(-3.12)
    shaped, overridden = latch.shape(0.0, 0.0, -0.10)
    assert shaped == pytest.approx(0.10)
    assert overridden


def test_rear_path_reselects_direction_when_replan_makes_it_shorter():
    latch = RearTurnDirectionLatch(2.75, 2.20, 0.35, 0.02, 0.04, 0.05)
    latch.update(3.0)
    assert latch.direction == 1

    assert latch.update(-2.8)
    assert latch.direction == -1
    shaped, overridden = latch.shape(0.0, 0.0, 0.10)
    assert shaped == pytest.approx(-0.10)
    assert overridden


def test_rear_path_latch_releases_direction_before_translation():
    latch = RearTurnDirectionLatch(2.75, 2.20, 0.35, 0.02, 0.04)
    latch.update(-3.10)

    assert latch.update(-2.0)
    assert latch.direction == 0
    assert latch.alignment_active
    assert latch.shape(0.0, 0.0, 0.10) == (0.10, False)
    assert latch.shape_translation(0.20, -0.10) == (0.0, 0.0, True)

    assert latch.update(-0.34)
    assert not latch.alignment_active
    assert latch.shape_translation(0.20, -0.10) == (0.20, -0.10, False)


def test_rear_path_alignment_forces_rotation_after_nav2_stops_turning():
    latch = RearTurnDirectionLatch(2.75, 2.20, 0.35, 0.02, 0.04)
    latch.update(-3.10)

    shaped, forced = latch.shape_alignment_rotation(0.10, 0.25)
    assert shaped == pytest.approx(-0.25)
    assert forced

    latch.update(-2.0)
    assert latch.direction == 0
    shaped, forced = latch.shape_alignment_rotation(0.0, 0.25)
    assert shaped == pytest.approx(-0.25)
    assert forced

    latch.update(-0.34)
    assert latch.shape_alignment_rotation(0.10, 0.25) == (0.10, False)


def test_rear_path_latch_never_overrides_while_translating():
    latch = RearTurnDirectionLatch(2.75, 2.20, 0.35, 0.02, 0.04)
    latch.update(3.10)

    assert latch.shape(0.05, 0.0, -0.10) == (-0.10, False)


def test_front_path_does_not_suppress_translation():
    latch = RearTurnDirectionLatch(2.75, 2.20, 0.35, 0.02, 0.04)

    assert not latch.update(1.0)
    assert not latch.alignment_active
    assert latch.shape_translation(0.20, 0.10) == (0.20, 0.10, False)


def test_rotation_watchdog_trips_on_duration_and_resets_on_reversal():
    watchdog = RotationWatchdog(0.08, 10.0, 2.0, max_sample_dt=1.0)

    assert not watchdog.update(0.0, 0.15, 0.0)
    assert not watchdog.update(1.0, 0.15, 0.0)
    assert not watchdog.update(1.5, -0.15, 0.0)
    assert watchdog.direction == -1
    assert watchdog.start_time == pytest.approx(1.5)
    assert not watchdog.update(2.5, -0.15, 0.0)
    assert watchdog.update(3.5, -0.15, 0.0)

    assert not watchdog.update(4.0, 0.0, 0.0)
    assert watchdog.direction == 0
    assert watchdog.measured_angle == pytest.approx(0.0)


def test_relocalization_gate_requires_hold_health_and_fresh_data():
    gate = RelocalizationGate(2.0, 3)
    gate.begin(10.0)
    for _ in range(3):
        gate.observe_alignment(True)

    assert not gate.try_release(11.9, True, True)
    assert not gate.try_release(12.0, False, True)
    assert not gate.try_release(12.0, True, False)
    assert gate.try_release(12.0, True, True)
    assert not gate.active


def test_relocalization_gate_resets_consecutive_health_count():
    gate = RelocalizationGate(2.0, 3)
    gate.begin(10.0)
    gate.observe_alignment(True)
    gate.observe_alignment(True)
    gate.observe_alignment(False)
    gate.observe_alignment(True)
    gate.observe_alignment(True)

    assert gate.stable_samples == 2
    assert not gate.try_release(12.0, True, True)
    gate.observe_alignment(True)
    assert gate.try_release(12.0, True, True)


def test_navigation_bridge_never_requests_whole_robot_power_off():
    source = (PACKAGE_DIR / "scripts" / "v2_nav_cmd_bridge.py").read_text()

    assert "_request_power(0)" not in source
    assert "request.value = 0" not in source
    assert "_request_power(1)" in source


def test_velocity_passthrough_forwards_the_command_untouched():
    from geometry_msgs.msg import Twist

    command = Twist()
    command.linear.x = 0.05
    command.linear.y = -0.03
    command.angular.z = 0.72
    output = _passthrough_command(command)

    # Values that the shaping chain would otherwise boost, clamp or calibrate.
    assert output.linear.x == pytest.approx(0.05)
    assert output.linear.y == pytest.approx(-0.03)
    assert output.angular.z == pytest.approx(0.72)


def test_nav_config_uses_mppi_and_realistic_goal_tolerance():
    config = yaml.safe_load((PACKAGE_DIR / "config" / "nav2_omni.yaml").read_text())
    controller = config["controller_server"]["ros__parameters"]
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    follow_path = controller["FollowPath"]
    bt_navigator = config["bt_navigator"]["ros__parameters"]
    behavior_server = config["behavior_server"]["ros__parameters"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert follow_path["plugin"] == "nav2_mppi_controller::MPPIController"
    # MPPI reads controller_frequency from its own namespace and throws at
    # configure time unless one integration step equals that period
    # ("Controller period more then model dt"). Guard both halves here.
    assert follow_path["controller_frequency"] == pytest.approx(
        controller["controller_frequency"]
    )
    assert follow_path["model_dt"] == pytest.approx(
        1.0 / follow_path["controller_frequency"]
    )
    # Keep the prediction horizon around 3 s; 28 x 0.1 s matches the reference.
    assert follow_path["model_dt"] * follow_path["time_steps"] <= 3.0
    assert follow_path["motion_model"] == "Omni"
    assert follow_path["iteration_count"] == 1
    assert follow_path["temperature"] <= 0.30
    assert follow_path["prune_distance"] >= 1.0
    assert planner["plugin"] == "nav2_smac_planner/SmacPlanner2D"
    assert planner["cost_travel_multiplier"] >= 30.0
    # Speed envelope carried over from the DWB tuning.
    assert follow_path["vx_min"] == pytest.approx(0.0)
    assert follow_path["vx_max"] == pytest.approx(0.30)
    assert follow_path["vy_max"] == pytest.approx(0.10)
    assert follow_path["wz_max"] == pytest.approx(0.25)
    assert controller["progress_checker"]["plugin"] == (
        "nav2_controller::PoseProgressChecker"
    )
    assert controller["progress_checker"]["required_movement_angle"] == pytest.approx(
        0.10
    )
    expected_critics = {
        "ConstraintCritic",
        "CostCritic",
        "GoalCritic",
        "PathFollowCritic",
        "PathAlignCritic",
        "ObstaclesCritic",
    }
    assert expected_critics <= set(follow_path["critics"])
    for critic in expected_critics:
        assert follow_path[critic]["enabled"] is True
    # GoalCritic closes the approach, PathFollow/PathAlign keep the plan.
    assert follow_path["GoalCritic"]["cost_weight"] >= 20.0
    assert follow_path["PathFollowCritic"]["cost_weight"] >= 10.0
    assert follow_path["PathAlignCritic"]["cost_weight"] >= 10.0
    assert controller["goal_checker"]["xy_goal_tolerance"] == pytest.approx(0.20)
    assert controller["goal_checker"]["yaw_goal_tolerance"] == pytest.approx(0.10)
    assert bt_navigator["robot_base_frame"] == "base_footprint"
    assert behavior_server["robot_base_frame"] == "base_footprint"
    assert smoother["max_velocity"] == pytest.approx([0.30, 0.10, 0.25])
    assert smoother["min_velocity"][0] == pytest.approx(-0.10)
    # MPPI in Humble exposes no acceleration limits, so the smoother carries
    # the former DWB envelope unchanged.
    assert smoother["max_accel"] == pytest.approx([0.80, 0.80, 0.40])
    assert smoother["max_decel"] == pytest.approx([-0.80, -0.80, -0.40])
    assert smoother["feedback"] == "OPEN_LOOP"


def test_localization_keeps_nav_tf_current_between_ndt_updates():
    config = yaml.safe_load(
        (PACKAGE_DIR / "config" / "mid360_localization.yaml").read_text()
    )["/**"]["ros__parameters"]

    assert config["enable_map_odom_tf"] is True
    assert config["use_odom_tf_prediction"] is True
    assert config["publish_bridge_pose_when_lost"] is True
    assert config["enable_timer_publishing"] is True
    assert config["pose_publish_frequency"] >= 10.0


def _fast_lio_parameter_overrides(launch_path):
    tree = ast.parse(launch_path.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        package = keywords.get("package")
        if not (isinstance(package, ast.Constant) and package.value == "fast_lio"):
            continue

        overrides = {}
        parameters = keywords.get("parameters")
        if isinstance(parameters, ast.List):
            for element in parameters.elts:
                if not isinstance(element, ast.Dict):
                    continue
                for key, value in zip(element.keys, element.values):
                    if isinstance(key, ast.Constant) and isinstance(value, ast.Constant):
                        overrides[key.value] = value.value
        return overrides

    raise AssertionError("localization launch does not define the fast_lio node")


def test_localization_fastlio_stops_the_unbounded_map_and_pcd_save():
    launch = PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    overrides = _fast_lio_parameter_overrides(launch)

    # The mapping profile keeps these enabled for map building.  Navigation
    # consumes odometry and the registered body cloud only, so accumulating
    # and serializing the world map here competes with scan processing.
    assert overrides["publish.map_en"] is False
    assert overrides["publish.path_en"] is False
    assert overrides["pcd_save.pcd_save_en"] is False


def test_localization_bounds_the_published_path_history():
    config = yaml.safe_load(
        (PACKAGE_DIR / "config" / "mid360_localization.yaml").read_text()
    )["/**"]["ros__parameters"]

    # At 10 Hz the timer would otherwise grow /path without limit.  Keep a
    # few minutes of history, and always keep the bound well below the size
    # where publishing the path copy costs more than a scan registration.
    assert 0 < config["max_path_poses"] <= 5000


def test_bridge_watchdogs_cover_measured_odom_jitter_but_stop_stale_commands():
    launch = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()

    assert '"odometry_timeout_sec": 1.0' in launch
    assert '"command_timeout_sec": 0.25' in launch
    assert '"localization_min_sample_dt_sec": 0.05' in launch
    assert '"localization_min_speed_jump_step_m": 0.25' in launch
    assert '"localization_jump_auto_recovery_enabled": True' in launch
    assert '"localization_jump_recovery_hold_sec": 5.0' in launch
    assert '"localization_jump_recovery_stable_samples": 10' in launch
    assert '"localization_jump_recovery_max_attempts": 1' in launch
    assert '"localization_jump_recovery_max_linear_speed_mps": 0.12' in launch
    assert '"localization_jump_recovery_max_angular_speed_rad_s": 0.05' in launch
    assert '"enable_reverse_recovery": True' in launch
    assert '"max_desired_reverse_x_mps": 0.10' in launch
    assert '"enable_obstacle_escape": False' in launch
    assert '"obstacle_escape_block_timeout_sec": 0.3' in launch
    assert '"obstacle_escape_duration_sec": 4.0' in launch
    assert '"obstacle_escape_cooldown_sec": 0.5' in launch
    assert '"obstacle_escape_x_mps": 0.0' in launch
    assert '"obstacle_escape_y_mps": 0.08' in launch
    assert '"obstacle_escape_radius_m": 0.40' in launch
    assert '"obstacle_escape_clearance_m": 0.60' in launch
    assert '"obstacle_escape_data_timeout_sec": 0.5' in launch
    assert '"body_to_base_x_m": 0.14' in launch
    assert '"rotation_watchdog_max_angle_rad": 6.28' in launch
    assert '"rotation_watchdog_max_duration_sec": 30.0' in launch
    assert '"rotation_fault_recovery_timeout_sec": 5.0' in launch
    assert '"min_linear_speed_mps": 0.427' in launch
    assert '"linear_command_scale": 1.0' in launch
    assert 'DeclareLaunchArgument("velocity_passthrough", default_value="true")' in launch
    assert (
        '"velocity_passthrough": _as_bool(context, "velocity_passthrough")'
        in launch
    )

    bridge = (PACKAGE_DIR / "scripts" / "v2_nav_cmd_bridge.py").read_text()
    assert '"velocity_passthrough", False' in bridge
    assert "def _passthrough_command(" in bridge
    assert "output = _passthrough_command(self.last_command)" in bridge
    assert "odometry_fault_latched" not in bridge
    assert "Motion will resume automatically when /nav2_odom recovers" in bridge
    assert "Planar odometry recovered; navigation motion may resume" in bridge
    assert 'localization_fault_reason = "timeout"' not in bridge
    assert "Motion will resume automatically when localization recovers" in bridge
    assert "3D localization recovered; navigation motion may resume" in bridge
    assert '"block_reason": block_reason' in bridge
    assert "competing_publisher" in bridge
    assert "localization_jump_latched" in bridge
    assert "localization_jump_recovery" in bridge
    assert "Localization-jump auto-recovery stabilized" in bridge
    assert "rotation_fault_latched" in bridge
    assert '"localization_min_sample_dt_sec", 0.05' in bridge
    assert "dt >= self.localization_min_sample_dt" in bridge
    assert '"localization_min_speed_jump_step_m", 0.25' in bridge
    assert "step >= self.localization_min_speed_jump_step" in bridge
    assert '"enable_reverse_recovery", False' in bridge
    assert '"max_desired_reverse_x_mps", 0.10' in bridge
    assert "allow_reverse=self.enable_reverse_recovery" in bridge
    assert '"enable_obstacle_escape", False' in bridge
    assert "obstacle_escape_started" in bridge
    assert "self._obstacle_escape_xy" in bridge
    assert '"rotation_watchdog_max_angle_rad", 2.0' in bridge
    assert '"rotation_watchdog_max_duration_sec", 15.0' in bridge
    assert '"rotation_fault_recovery_timeout_sec", 5.0' in bridge
    assert "rotation_fault_recovery_timeout" in bridge
    assert '"obstacle_escape_radius_m", 0.40' in bridge
    assert '"obstacle_escape_clearance_m", 0.60' in bridge
    assert "obstacle_distance > self.obstacle_escape_clearance" in bridge
    assert '"obstacle_escape_data_timeout_sec", 0.5' in bridge
    assert '"body_to_base_x_m", 0.14' in bridge
    assert "latest_left_clearance" in bridge
    assert "latest_right_clearance" in bridge


def test_navigation_launch_uses_v2_hardware_serial_rate():
    launch = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()

    assert 'DeclareLaunchArgument("baud_rate", default_value="4000000")' in launch
    assert 'DeclareLaunchArgument("serial_frequency", default_value="1000")' in launch


def test_navigation_launch_selects_v3_without_changing_v2_speed_calibration():
    launch = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()

    assert 'DeclareLaunchArgument("protocol_version", default_value="v3")' in launch
    assert '"protocol_version": protocol' in launch
    assert '"protocol_version": "v2"' not in launch
    assert '"x_positive_command_mps": [0.0, 0.427, 0.500]' in launch
    assert '"angular_z_command_scale": 0.538' in launch


def test_navigation_processes_start_only_after_localization_gate_succeeds():
    launch = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()

    assert 'SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")' in launch
    assert 'executable="navigation_startup_gate.py"' in launch
    assert '"odom_frame": "camera_init"' in launch
    assert "OnProcessExit(" in launch
    assert "if launch_context.is_shutdown:" in launch
    assert "if event.returncode != 0:" in launch
    assert "return [navigation, collision_monitor, collision_lifecycle]" in launch
    assert (
        "actions=[map_server, map_lifecycle, navigation_startup_gate]" in launch
    )
    assert "actions=[map_server, map_lifecycle, navigation]" not in launch

    gate = (PACKAGE_DIR / "scripts" / "navigation_startup_gate.py").read_text()
    odometry = (PACKAGE_DIR / "scripts" / "fastlio_planar_odom.py").read_text()
    for source in (gate, odometry):
        assert "signal_handler_options=SignalHandlerOptions.NO" in source
        assert "while rclpy.ok() and not shutdown_requested" in source


def _point_in_convex(points, px, py):
    """Containment test for a counter-clockwise convex polygon."""
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        if (x2 - x1) * (py - y1) - (y2 - y1) * (px - x1) < 0.0:
            return False
    return True


def _distance_to_polygon(points, px, py):
    """Shortest distance from a point to the outline of a convex polygon."""
    best = math.inf
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        dx, dy = x2 - x1, y2 - y1
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0.0 else ((px - x1) * dx + (py - y1) * dy) / length_sq
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - (x1 + t * dx), py - (y1 + t * dy)))
    return best


def test_live_obstacles_drive_replanning_and_near_field_safety_stop():
    config = yaml.safe_load((PACKAGE_DIR / "config" / "nav2_omni.yaml").read_text())
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    monitor = config["collision_monitor"]["ros__parameters"]

    assert "stvl_voxel_layer" in global_costmap["plugins"]
    for costmap in (global_costmap, local_costmap):
        layer = costmap["stvl_voxel_layer"]
        source = layer["live_obstacles"]
        # The launch file derives the outline from the URDF and rewrites both
        # costmaps; the YAML keeps an identical fallback for nav2_bringup runs.
        vertices = json.loads(costmap["footprint"])
        assert len(vertices) >= 6
        assert max(x for x, _ in vertices) == pytest.approx(0.2510, abs=1e-4)
        assert min(x for x, _ in vertices) == pytest.approx(-0.3340, abs=1e-4)
        assert max(abs(y) for _, y in vertices) == pytest.approx(0.3335, abs=1e-4)
        assert costmap["footprint_padding"] == pytest.approx(0.0)
        assert source["topic"] == "/obstacle_points_live"
        # The voxel layer keeps the former 2D thresholds: near field at 0.05 m,
        # 8 m marking/clearing reach, and a 0.5 s time-based retention.
        assert source["min_z"] <= 0.05
        assert source["max_z"] == pytest.approx(8.0)
        assert source["obstacle_range"] == pytest.approx(8.0)
        assert source["min_obstacle_height"] == pytest.approx(-0.20)
        assert source["max_obstacle_height"] == pytest.approx(2.00)
        assert layer["voxel_size"] == pytest.approx(0.05)
        assert layer["voxel_decay"] == pytest.approx(0.50)
        assert layer["decay_model"] == 0
        assert source["model_type"] == 1
        assert costmap["inflation_layer"]["inflation_radius"] == pytest.approx(0.50)
        assert costmap["inflation_layer"]["cost_scaling_factor"] <= 0.6

    # MPPI has to treat live obstacles as more than a path-tracking nudge.
    assert controller["ObstaclesCritic"]["repulsion_weight"] >= 1.0
    assert controller["CostCritic"]["cost_weight"] >= 3.0
    assert monitor["cmd_vel_in_topic"] == "/cmd_vel"
    assert monitor["cmd_vel_out_topic"] == "/cmd_vel_safe"
    assert monitor["source_timeout"] == pytest.approx(0.5)
    assert monitor["live_obstacles"]["topic"] == "/obstacle_points_live"
    stop_zone = monitor["RadarStopZone"]
    points = list(zip(stop_zone["points"][::2], stop_zone["points"][1::2]))
    # Outline of the std_v6 chassis in base_footprint (mesh bounds; the three
    # wheel links stay inside it).
    body_outline = [
        (-0.314, -0.080),
        (0.076, -0.313),
        (0.2305, -0.225),
        (0.2305, 0.225),
        (0.076, 0.313),
        (-0.314, 0.080),
    ]
    assert stop_zone["action_type"] == "stop"
    assert len(points) == len(body_outline)
    assert max(x for x, _ in points) == pytest.approx(0.310)
    assert min(x for x, _ in points) == pytest.approx(-0.394)
    assert max(abs(y) for _, y in points) == pytest.approx(0.406)
    # The zone has to enclose the chassis and sit 0.08 m outside it: any
    # closer and the body would touch an obstacle before the stop fires.
    for x, y in body_outline:
        assert _point_in_convex(points, x, y), (x, y)
        assert _distance_to_polygon(points, x, y) == pytest.approx(0.08, abs=2.0e-3)


def test_navigation_launch_relays_the_smoothed_command():
    """The relay takes the velocity_smoother output, not /cmd_vel_safe.

    collision_monitor is still launched and still publishes its own output
    topic, but nothing consumes it: while the monitor holds STOP it goes
    silent after stop_pub_timeout and the relay would send zeros forever.
    """
    source = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()

    assert 'package="nav2_collision_monitor"' in source
    assert '"input_topic": "/cmd_vel"' in source
    assert '"self_filter_enabled": True' in source
    assert '"self_filter_min_x": -0.50' in source
    assert '"self_filter_max_x": 0.17' in source
    assert '"self_filter_min_y": -0.35' in source
    assert '"self_filter_max_y": 0.35' in source


def test_navigation_default_tree_never_moves_the_base():
    launch_source = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()
    assert "navigate_to_pose_forward_only.xml" in launch_source
    # The default tree must not be the legacy one that backs the base up.
    assert "navigate_to_pose_obstacle_recovery.xml" not in launch_source
    assert '"enable_obstacle_escape": False' in launch_source

    active_tree = PACKAGE_DIR / "behavior_trees" / "navigate_to_pose_forward_only.xml"
    active_text = active_tree.read_text()
    tags = {element.tag for element in ET.parse(active_tree).iter()}
    # A blocked path must never turn into a reverse or an in-place rotation.
    assert "BackUp" not in tags
    assert "Spin" not in tags
    assert "Wait" in tags
    assert 'wait_duration="3"' in active_text
    assert "ClearEntireCostmap" in tags
    assert "PipelineSequence" in tags
    assert "RateController" in tags
    assert "ComputePathToPose" in tags
    assert "FollowPath" in tags

    behavior_plugins = yaml.safe_load(
        (PACKAGE_DIR / "config" / "nav2_omni.yaml").read_text()
    )["behavior_server"]["ros__parameters"]["behavior_plugins"]
    assert "backup" in behavior_plugins

    # The legacy motion-recovery tree stays in the package, but unused.
    tree_paths = sorted((PACKAGE_DIR / "behavior_trees").glob("*.xml"))
    assert len(tree_paths) == 3


def test_costmap_footprint_is_derived_at_launch_time():
    launch_source = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()
    # Nav2 only accepts a vertex list, so the launch file must compute it and
    # rewrite both costmaps instead of hard-coding the outline in the YAML.
    assert "costmap_footprint_from_urdf" in launch_source
    assert "costmap_footprint_padding" in launch_source
    assert 'rewrites["footprint"] = footprint' in launch_source
    assert "urdf_footprint" in launch_source


def test_collision_monitor_polygon_is_derived_at_launch_time():
    launch_source = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()
    monitor = yaml.safe_load(
        (PACKAGE_DIR / "config" / "nav2_omni.yaml").read_text()
    )["collision_monitor"]["ros__parameters"]
    # collision_monitor reads ``points`` as a double array, which a YAML
    # rewrite cannot express, so the polygon must travel as a node override.
    assert "collision_monitor_polygon_from_urdf" in launch_source
    assert "collision_monitor_polygon_padding" in launch_source
    assert (
        'collision_parameters.append(Parameter("RadarStopZone.points", stop_zone))'
        in launch_source
    )
    assert "from launch_ros.descriptions import Parameter" in launch_source
    assert "parameters=collision_parameters" in launch_source
    # The stop polygon needs a wider margin than the costmap footprint: the
    # body must not touch an obstacle before the stop fires.
    assert float(monitor["RadarStopZone"]["max_points"]) >= 2
    fallback = monitor["RadarStopZone"]["points"]
    assert len(fallback) % 2 == 0 and len(fallback) >= 6


def test_urdf_footprint_hull_and_padding(tmp_path, monkeypatch):
    import urdf_footprint

    # Keep the test hermetic: never write into the developer's home directory.
    monkeypatch.setenv("BW_URDF_FOOTPRINT_CACHE", str(tmp_path / "cache"))

    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="probe">
  <link name="base_footprint"/>
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1.0 0.6 0.2"/></geometry>
    </collision>
  </link>
  <joint name="base_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_footprint"/>
    <child link="base_link"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    silent = lambda *args, **kwargs: None  # noqa: E731
    bare = urdf_footprint.compute_polygon(urdf, log=silent)
    assert bare["area"] == pytest.approx(0.60, rel=1e-3)
    assert max(x for x, _ in bare["polygon"]) == pytest.approx(0.5, abs=1e-6)
    assert max(abs(y) for _, y in bare["polygon"]) == pytest.approx(0.3, abs=1e-6)

    grown = urdf_footprint.compute_polygon(urdf, padding=0.05, log=silent)
    assert grown["area"] > bare["area"]
    assert max(x for x, _ in grown["polygon"]) == pytest.approx(0.55, abs=1e-6)
    assert max(abs(y) for _, y in grown["polygon"]) == pytest.approx(0.35, abs=1e-6)

    # The string is what Nav2 parses, and must round-trip to the same polygon.
    parsed = json.loads(urdf_footprint.format_footprint(bare["polygon"]))
    assert len(parsed) == len(bare["polygon"])

    # Nav2's robot radius for this hull is the largest origin-centred disc:
    # half the short side of the 1.0 x 0.6 box.  The map builder clears the
    # walked corridor with the same number, so pin it here.
    assert urdf_footprint.inscribed_radius(bare["hull"]) == pytest.approx(0.3)


def test_urdf_footprint_cache_hits_and_invalidates(tmp_path, monkeypatch):
    import os

    import urdf_footprint

    cache_dir = tmp_path / "cache"
    monkeypatch.setenv("BW_URDF_FOOTPRINT_CACHE", str(cache_dir))
    urdf = tmp_path / "robot.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="probe">
  <link name="base_footprint"/>
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="2.0 1.0 0.2"/></geometry>
    </collision>
  </link>
  <joint name="base_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_footprint"/>
    <child link="base_link"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    silent = lambda *args, **kwargs: None  # noqa: E731

    first = urdf_footprint.compute_polygon(urdf, log=silent)
    assert first["cached"] is False
    entries = sorted(cache_dir.glob("*.json"))
    assert len(entries) == 1

    second = urdf_footprint.compute_polygon(urdf, log=silent)
    assert second["cached"] is True
    assert second["area"] == pytest.approx(first["area"], rel=1e-9)
    assert [list(p) for p in second["polygon"]] == [list(p) for p in first["polygon"]]
    assert second["records"] == first["records"]

    # ``use_cache=False`` must ignore the stored outline but keep it intact.
    uncached = urdf_footprint.compute_polygon(urdf, log=silent, use_cache=False)
    assert uncached["cached"] is False
    assert len(sorted(cache_dir.glob("*.json"))) == 1

    # A different padding is a different question and needs its own entry.
    padded = urdf_footprint.compute_polygon(urdf, padding=0.1, log=silent)
    assert padded["cached"] is False
    assert len(sorted(cache_dir.glob("*.json"))) == 2
    assert urdf_footprint.compute_polygon(
        urdf, padding=0.1, log=silent)["cached"] is True

    # Touching the model must invalidate the entry.
    stat = urdf.stat()
    os.utime(urdf, ns=(stat.st_atime_ns, stat.st_mtime_ns + 1_000_000_000))
    assert urdf_footprint.compute_polygon(urdf, log=silent)["cached"] is False

    # A corrupt entry is ignored rather than fatal.
    for entry in cache_dir.glob("*.json"):
        entry.write_text("{not json", encoding="utf-8")
    assert urdf_footprint.compute_polygon(urdf, log=silent)["cached"] is False


def test_waypoint_tree_uses_the_actual_robot_base_frame():
    tree = ET.parse(
        PACKAGE_DIR
        / "behavior_trees"
        / "navigate_through_poses_forward_only.xml"
    )
    remove_passed_goals = next(
        element for element in tree.iter() if element.tag == "RemovePassedGoals"
    )

    assert remove_passed_goals.attrib["global_frame"] == "map"
    assert remove_passed_goals.attrib["robot_base_frame"] == "base_footprint"


def test_rviz_visualizes_transient_local_waypoints():
    config = yaml.safe_load(
        (PACKAGE_DIR / "rviz" / "mid360_nav2.rviz").read_text()
    )
    displays = config["Visualization Manager"]["Displays"]
    nav2_group = next(
        display for display in displays if display["Name"] == "Nav2 costmaps"
    )
    waypoints = next(
        display for display in nav2_group["Displays"] if display["Name"] == "Waypoints"
    )

    assert waypoints["Class"] == "rviz_default_plugins/MarkerArray"
    assert waypoints["Enabled"] is True
    assert waypoints["Topic"]["Value"] == "/waypoints"
    assert waypoints["Topic"]["Durability Policy"] == "Transient Local"


def test_static_and_live_ground_segmentation_use_distinct_algorithms():
    converter = (PACKAGE_DIR / "src" / "pcd_to_nav2_map.cpp").read_text()
    live_node = (
        PACKAGE_DIR / "src" / "laser_map_ground_grid_node.cpp"
    ).read_text()
    localization_launch = (
        PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
    ).read_text()
    readme = (PACKAGE_DIR / "README.md").read_text()

    assert "segment_ground_csf" in converter
    assert "segment_ground_patchwork" in live_node
    assert '"input_topic": "/cloud_registered_body"' in localization_launch
    assert '"sensor_height": 0.18' in localization_launch
    assert '"min_range": 0.05' in localization_launch
    assert "mid360_pgo_csf.yaml" in localization_launch
    assert "RANSAC plane" not in readme
    assert "mid360_pgo_ransac" not in readme


def test_ground_segmentation_upstream_sources_and_safety_height_are_present():
    cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text()
    config = yaml.safe_load((PACKAGE_DIR / "config" / "nav2_omni.yaml").read_text())
    monitor = config["collision_monitor"]["ros__parameters"]

    assert "third_party/csf/src" in cmake
    assert "third_party/patchworkpp" in cmake
    assert (PACKAGE_DIR / "third_party" / "csf" / "LICENSE.txt").is_file()
    assert (PACKAGE_DIR / "third_party" / "patchworkpp" / "LICENSE").is_file()
    assert monitor["live_obstacles"]["min_height"] <= -0.20


def test_costmap_layer_packages_are_installed():
    """Every layer plugin named in the YAML must resolve in this environment.

    pluginlib only reports a missing package while a costmap configures, i.e.
    inside a lifecycle transition during bringup, and it names the plugin class
    rather than the ROS package. Checking the ament index here turns a failed
    robot start into a failed build.
    """
    ament = pytest.importorskip("ament_index_python.packages")
    config = yaml.safe_load((PACKAGE_DIR / "config" / "nav2_omni.yaml").read_text())

    packages = set()
    for node in ("global_costmap", "local_costmap"):
        params = config[node][node]["ros__parameters"]
        for layer in params["plugins"]:
            # pluginlib accepts both "pkg/Class" and "pkg::Class" spellings.
            plugin = str(params[layer]["plugin"])
            for separator in ("/", "::"):
                if separator in plugin:
                    packages.add(plugin.split(separator, 1)[0].strip())
                    break

    missing = []
    for package in sorted(packages):
        try:
            ament.get_package_share_directory(package)
        except ament.PackageNotFoundError:
            missing.append(package)

    assert not missing, (
        "nav2_omni.yaml selects costmap layers from missing packages: "
        + ", ".join(missing)
        + " (install them with rosdep or the matching ros-$ROS_DISTRO-* package)")
