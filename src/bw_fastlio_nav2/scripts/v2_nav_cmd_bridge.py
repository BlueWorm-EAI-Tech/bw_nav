#!/usr/bin/env python3
"""Safety gate between Nav2 velocity output and the Mantis serial driver."""

from __future__ import annotations

from dataclasses import dataclass
import math
import signal
import time

import rclpy
from bw_serial.srv import SetCtrlSrc
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _clamp_forward(value: float, limit: float) -> float:
    """Clamp an x command while making reverse motion impossible."""
    return max(0.0, min(limit, value))


def _requests_motion(
    x: float,
    y: float,
    angular_z: float,
    linear_deadband: float,
    angular_deadband: float = 1.0e-3,
    allow_reverse: bool = False,
) -> bool:
    """Return whether a command can intentionally move the chassis."""
    motion_x = x if allow_reverse else _clamp_forward(x, math.inf)
    return (
        math.hypot(motion_x, y) >= linear_deadband
        or abs(angular_z) >= angular_deadband
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_orientation(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def _yaw_from_pose(msg: PoseWithCovarianceStamped) -> float:
    return _yaw_from_orientation(msg.pose.pose.orientation)


def _validate_speed_calibration(
    actual_speeds: tuple[float, ...],
    commands: tuple[float, ...],
    name: str,
) -> None:
    if len(actual_speeds) != len(commands) or len(actual_speeds) < 2:
        raise ValueError(f"{name} calibration arrays must have equal length >= 2")
    if not all(math.isfinite(value) for value in (*actual_speeds, *commands)):
        raise ValueError(f"{name} calibration must contain only finite values")
    if actual_speeds[0] != 0.0 or commands[0] != 0.0:
        raise ValueError(f"{name} calibration must start at (0, 0)")
    if any(b <= a for a, b in zip(actual_speeds, actual_speeds[1:])):
        raise ValueError(f"{name} actual speeds must be strictly increasing")
    if any(b <= a for a, b in zip(commands, commands[1:])):
        raise ValueError(f"{name} commands must be strictly increasing")


def _inverse_speed_map(
    desired_speed: float,
    actual_speeds: tuple[float, ...],
    commands: tuple[float, ...],
) -> float:
    """Interpolate the measured command curve in the desired-to-command direction."""
    if desired_speed <= 0.0:
        return 0.0

    upper = 1
    while upper < len(actual_speeds) and desired_speed > actual_speeds[upper]:
        upper += 1
    if upper == len(actual_speeds):
        upper -= 1
    lower = upper - 1
    actual_span = actual_speeds[upper] - actual_speeds[lower]
    ratio = (desired_speed - actual_speeds[lower]) / actual_span
    return commands[lower] + ratio * (commands[upper] - commands[lower])


def _map_signed_speed(
    desired: float,
    positive_actual: tuple[float, ...],
    positive_commands: tuple[float, ...],
    negative_actual: tuple[float, ...],
    negative_commands: tuple[float, ...],
) -> float:
    if desired == 0.0:
        return 0.0
    actual = positive_actual if desired > 0.0 else negative_actual
    commands = positive_commands if desired > 0.0 else negative_commands
    return math.copysign(_inverse_speed_map(abs(desired), actual, commands), desired)


def _shape_translation(
    x: float,
    y: float,
    min_speed: float,
    deadband: float,
    max_x: float,
    max_y: float,
    startup_boost: bool = True,
) -> tuple[float, float, bool]:
    """Clamp a planar command and apply the breakaway boost while stationary."""
    x = _clamp(x, max_x)
    y = _clamp(y, max_y)
    speed = math.hypot(x, y)
    if speed < deadband:
        return 0.0, 0.0, False
    if not startup_boost or speed >= min_speed:
        return x, y, False
    scale = min_speed / speed
    return x * scale, y * scale, True


def _shape_routine_lateral_command(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    lateral_threshold: float,
    path_heading_error: float | None,
    turn_in_place_angle: float,
    alignment_speed: float,
) -> tuple[float, float, float, bool, bool]:
    """Convert routine strafe into turn-then-forward chassis motion."""
    if not 0.0 < abs(linear_y) < lateral_threshold:
        return linear_x, linear_y, angular_z, False, False

    forward_x = max(0.0, linear_x)
    speed = math.hypot(forward_x, linear_y)
    if path_heading_error is None or not math.isfinite(path_heading_error):
        heading_error = math.atan2(linear_y, max(forward_x, 1.0e-6))
    else:
        heading_error = _normalize_angle(path_heading_error)

    if abs(heading_error) > turn_in_place_angle:
        angular_z = math.copysign(max(abs(angular_z), alignment_speed), heading_error)
        return 0.0, 0.0, angular_z, True, True
    return speed, 0.0, angular_z, True, False


def _shape_goal_rotation(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    current_xy: tuple[float, float] | None,
    current_yaw: float | None,
    goal_xy: tuple[float, float] | None,
    goal_yaw: float | None,
    activation_distance: float,
    translation_threshold: float,
    fine_tune_angle: float,
    rotation_speed: float,
    yaw_tolerance: float,
) -> tuple[float, bool, float | None]:
    """Use a slow, shortest-direction turn during final pose alignment."""
    if (
        current_xy is None
        or current_yaw is None
        or goal_xy is None
        or goal_yaw is None
        or math.hypot(linear_x, linear_y) > translation_threshold
        or math.hypot(goal_xy[0] - current_xy[0], goal_xy[1] - current_xy[1])
        > activation_distance
    ):
        return angular_z, False, None

    yaw_error = _normalize_angle(goal_yaw - current_yaw)
    if abs(yaw_error) <= yaw_tolerance:
        return 0.0, True, yaw_error
    if abs(angular_z) <= 1.0e-3:
        return angular_z, False, yaw_error
    if abs(yaw_error) > fine_tune_angle:
        return math.copysign(abs(angular_z), yaw_error), False, yaw_error
    return math.copysign(rotation_speed, yaw_error), True, yaw_error


def _passthrough_command(command: Twist) -> Twist:
    """Copy a command verbatim so no shaping, mapping or limit applies."""
    output = Twist()
    output.linear.x = command.linear.x
    output.linear.y = command.linear.y
    output.angular.z = command.angular.z
    return output


def _path_heading_error(
    current_x: float,
    current_y: float,
    current_yaw: float,
    path_xy: tuple[tuple[float, float], ...],
    sample_distance: float,
) -> float | None:
    """Return the heading error to the first useful point on a global path."""
    fallback: tuple[float, float] | None = None
    for x, y in path_xy:
        dx = x - current_x
        dy = y - current_y
        distance = math.hypot(dx, dy)
        if distance <= 1.0e-3:
            continue
        fallback = (dx, dy)
        if distance >= sample_distance:
            return _normalize_angle(math.atan2(dy, dx) - current_yaw)
    if fallback is None:
        return None
    return _normalize_angle(math.atan2(fallback[1], fallback[0]) - current_yaw)


@dataclass
class RearTurnDirectionLatch:
    """Hold turn direction and translation while aligning with a rear path."""

    engage_angle: float
    release_angle: float
    alignment_release_angle: float
    command_threshold: float
    translation_threshold: float
    pi_tie_hysteresis: float = 0.05
    direction: int = 0
    alignment_active: bool = False
    heading_error: float | None = None

    def reset(self) -> None:
        self.direction = 0
        self.alignment_active = False
        self.heading_error = None

    def update(self, heading_error: float | None) -> bool:
        previous = (self.direction, self.alignment_active)
        if heading_error is None or not math.isfinite(heading_error):
            self.reset()
            return previous != (0, False)

        self.heading_error = heading_error
        desired_direction = 1 if heading_error > 0.0 else -1
        if self.direction == 0 and abs(heading_error) >= self.engage_angle:
            self.direction = desired_direction
            self.alignment_active = True
        elif self.direction != 0 and abs(heading_error) <= self.release_angle:
            self.direction = 0
        elif (
            self.direction != 0
            and desired_direction != self.direction
            and math.pi - abs(heading_error) > self.pi_tie_hysteresis
        ):
            # Follow a replanned shorter turn, but retain direction inside the
            # near-pi tie region to avoid noise-driven direction oscillation.
            self.direction = desired_direction
        if (
            self.alignment_active
            and abs(heading_error) <= self.alignment_release_angle
        ):
            self.alignment_active = False
        return (self.direction, self.alignment_active) != previous

    def shape_translation(
        self,
        linear_x: float,
        linear_y: float,
    ) -> tuple[float, float, bool]:
        if self.alignment_active:
            return 0.0, 0.0, True
        return linear_x, linear_y, False

    def shape_alignment_rotation(
        self,
        angular_z: float,
        alignment_speed: float,
    ) -> tuple[float, bool]:
        if not self.alignment_active or self.heading_error is None:
            return angular_z, False
        direction = self.direction
        if direction == 0 and abs(self.heading_error) > 1.0e-6:
            direction = 1 if self.heading_error > 0.0 else -1
        if direction == 0:
            return angular_z, False
        shaped = math.copysign(max(abs(angular_z), alignment_speed), direction)
        return shaped, shaped != angular_z

    def shape(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
    ) -> tuple[float, bool]:
        if (
            self.direction == 0
            or math.hypot(linear_x, linear_y) > self.translation_threshold
            or abs(angular_z) < self.command_threshold
        ):
            return angular_z, False
        shaped = math.copysign(abs(angular_z), self.direction)
        return shaped, shaped != angular_z


@dataclass
class RotationWatchdog:
    command_threshold: float
    max_angle: float
    max_duration: float
    max_sample_dt: float = 0.5
    direction: int = 0
    start_time: float | None = None
    last_time: float | None = None
    measured_angle: float = 0.0

    def reset(self) -> None:
        self.direction = 0
        self.start_time = None
        self.last_time = None
        self.measured_angle = 0.0

    def update(self, now: float, command: float, measured: float) -> bool:
        """Return true after an implausibly long same-direction rotation."""
        if not all(math.isfinite(value) for value in (now, command, measured)):
            self.reset()
            return False

        if abs(command) < self.command_threshold:
            self.reset()
            return False

        direction = 1 if command > 0.0 else -1
        if self.start_time is None or direction != self.direction:
            self.direction = direction
            self.start_time = now
            self.last_time = now
            self.measured_angle = 0.0
            return False

        if self.last_time is None or now < self.last_time:
            self.start_time = now
            self.last_time = now
            self.measured_angle = 0.0
            return False

        dt = min(now - self.last_time, self.max_sample_dt)
        self.last_time = now
        self.measured_angle += abs(measured) * dt
        duration = now - self.start_time
        return self.measured_angle >= self.max_angle or duration >= self.max_duration


@dataclass
class RelocalizationGate:
    hold_sec: float
    required_stable_samples: int
    active: bool = False
    started_at: float = 0.0
    stable_samples: int = 0

    def begin(self, now: float) -> None:
        self.active = True
        self.started_at = now
        self.stable_samples = 0

    def observe_alignment(self, healthy: bool) -> None:
        if not self.active:
            return
        self.stable_samples = self.stable_samples + 1 if healthy else 0

    def try_release(
        self,
        now: float,
        localization_fresh: bool,
        odometry_fresh: bool,
    ) -> bool:
        ready = (
            self.active
            and now - self.started_at >= self.hold_sec
            and self.stable_samples >= self.required_stable_samples
            and localization_fresh
            and odometry_fresh
        )
        if ready:
            self.active = False
        return ready


@dataclass
class LocalizationJumpRecovery:
    hold_sec: float
    required_stable_samples: int
    active: bool = False
    alignment_healthy: bool = False
    stable_since: float | None = None
    stable_samples: int = 0

    def begin(self) -> None:
        self.active = True
        self.alignment_healthy = False
        self.stable_since = None
        self.stable_samples = 0

    def cancel(self) -> None:
        self.active = False
        self.alignment_healthy = False
        self.stable_since = None
        self.stable_samples = 0

    def observe_alignment(self, healthy: bool) -> None:
        if not self.active:
            return
        self.alignment_healthy = healthy
        if not healthy:
            self.stable_since = None
            self.stable_samples = 0

    def observe_pose(self, stable: bool, now: float) -> None:
        if not self.active:
            return
        if not stable or not self.alignment_healthy:
            self.stable_since = None
            self.stable_samples = 0
            return
        if self.stable_since is None or now < self.stable_since:
            self.stable_since = now
            self.stable_samples = 1
        else:
            self.stable_samples += 1

    def try_release(
        self,
        now: float,
        localization_fresh: bool,
        odometry_fresh: bool,
        chassis_stationary: bool,
    ) -> bool:
        if self.active and not chassis_stationary:
            self.stable_since = None
            self.stable_samples = 0
            return False
        ready = (
            self.active
            and self.alignment_healthy
            and self.stable_since is not None
            and now - self.stable_since >= self.hold_sec
            and self.stable_samples >= self.required_stable_samples
            and localization_fresh
            and odometry_fresh
            and chassis_stationary
        )
        if ready:
            self.cancel()
        return ready


class NavCmdBridge(Node):
    def __init__(self) -> None:
        super().__init__("nav_cmd_bridge")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("protocol_version", "v3")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "Teleop/cmd_vel")
        self.declare_parameter("power_service", "set_ctrl_src")
        self.declare_parameter(
            "localization_topic", "/localization/pose_with_covariance"
        )
        self.declare_parameter("alignment_status_topic", "/alignment_status")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("initialpose_frame_id", "map")
        self.declare_parameter("odometry_topic", "/nav2_odom")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("yaw_tracking_topic", "/yaw_tracking_status")
        self.declare_parameter("require_localization", True)
        self.declare_parameter("localization_timeout_sec", 1.0)
        self.declare_parameter("odometry_timeout_sec", 0.50)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("desired_velocity_topic", "/cmd_vel_nav")
        self.declare_parameter("obstacle_topic", "/obstacle_points_live")
        self.declare_parameter("enable_obstacle_escape", False)
        self.declare_parameter("obstacle_escape_block_timeout_sec", 0.3)
        self.declare_parameter("obstacle_escape_duration_sec", 4.0)
        self.declare_parameter("obstacle_escape_cooldown_sec", 0.5)
        self.declare_parameter("obstacle_escape_x_mps", 0.0)
        self.declare_parameter("obstacle_escape_y_mps", 0.08)
        self.declare_parameter("obstacle_escape_radius_m", 0.40)
        self.declare_parameter("obstacle_escape_clearance_m", 0.60)
        self.declare_parameter("obstacle_escape_max_range_m", 1.5)
        self.declare_parameter("obstacle_escape_data_timeout_sec", 0.5)
        self.declare_parameter("body_to_base_x_m", 0.14)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("yaw_diagnostic_period_sec", 1.0)
        self.declare_parameter("plan_timeout_sec", 2.0)
        self.declare_parameter("yaw_goal_tolerance_rad", 0.10)
        self.declare_parameter("goal_rotation_activation_distance_m", 0.30)
        self.declare_parameter("goal_rotation_fine_tune_angle_rad", 0.523599)
        self.declare_parameter("goal_rotation_speed_rad_s", 0.08)
        self.declare_parameter("min_linear_speed_mps", 0.427)
        self.declare_parameter("linear_deadband_mps", 0.02)
        self.declare_parameter("lateral_motion_threshold_mps", 0.08)
        self.declare_parameter("forward_turn_in_place_angle_rad", 0.35)
        self.declare_parameter("max_desired_linear_x_mps", 0.30)
        self.declare_parameter("max_desired_linear_y_mps", 0.10)
        self.declare_parameter("max_desired_angular_z_rad_s", 1.0)
        self.declare_parameter("enable_reverse_recovery", False)
        self.declare_parameter("max_desired_reverse_x_mps", 0.10)
        self.declare_parameter("max_output_linear_x_mps", 0.55)
        self.declare_parameter("max_output_linear_y_mps", 0.50)
        self.declare_parameter("max_output_angular_z_rad_s", 1.0)
        self.declare_parameter("linear_command_scale", 1.0)
        # Debug switch: forward /cmd_vel verbatim and skip every shaping,
        # mapping and limit stage (see _passthrough_command).
        self.declare_parameter("velocity_passthrough", False)
        # Reuse the V2 wheel calibration until a V3 command sweep is available.
        self.declare_parameter("x_positive_actual_mps", [0.0, 0.223, 0.271])
        self.declare_parameter("x_positive_command_mps", [0.0, 0.427, 0.500])
        self.declare_parameter("x_negative_actual_mps", [0.0, 0.238, 0.293])
        self.declare_parameter("x_negative_command_mps", [0.0, 0.427, 0.500])
        self.declare_parameter("y_positive_actual_mps", [0.0, 0.230, 0.303])
        self.declare_parameter("y_positive_command_mps", [0.0, 0.427, 0.500])
        self.declare_parameter("y_negative_actual_mps", [0.0, 0.248, 0.317])
        self.declare_parameter("y_negative_command_mps", [0.0, 0.427, 0.500])
        self.declare_parameter("motion_detect_on_mps", 0.08)
        self.declare_parameter("motion_detect_off_mps", 0.04)
        self.declare_parameter("max_localization_step_m", 0.50)
        self.declare_parameter("max_localization_speed_mps", 1.0)
        self.declare_parameter("max_localization_yaw_step_rad", 0.60)
        self.declare_parameter("localization_min_sample_dt_sec", 0.05)
        self.declare_parameter("localization_min_speed_jump_step_m", 0.25)
        self.declare_parameter("initialpose_hold_sec", 2.0)
        self.declare_parameter("initialpose_stable_samples", 3)
        self.declare_parameter("localization_jump_auto_recovery_enabled", True)
        self.declare_parameter("localization_jump_recovery_hold_sec", 5.0)
        self.declare_parameter("localization_jump_recovery_stable_samples", 10)
        self.declare_parameter("localization_jump_recovery_max_attempts", 1)
        self.declare_parameter(
            "localization_jump_recovery_max_linear_speed_mps", 0.12
        )
        self.declare_parameter(
            "localization_jump_recovery_max_angular_speed_rad_s", 0.05
        )
        self.declare_parameter("rotation_watchdog_command_rad_s", 0.08)
        self.declare_parameter("rotation_watchdog_max_angle_rad", 2.0)
        self.declare_parameter("rotation_watchdog_max_duration_sec", 15.0)
        self.declare_parameter("rotation_fault_recovery_timeout_sec", 5.0)
        self.declare_parameter("rear_turn_sample_distance_m", 0.50)
        self.declare_parameter("rear_turn_engage_angle_rad", 2.75)
        self.declare_parameter("rear_turn_release_angle_rad", 2.20)
        self.declare_parameter("rear_turn_alignment_release_angle_rad", 0.35)
        self.declare_parameter("rear_turn_alignment_speed_rad_s", 0.25)
        self.declare_parameter("rear_turn_command_threshold_rad_s", 0.02)
        self.declare_parameter("rear_turn_translation_threshold_mps", 0.04)
        self.declare_parameter("rear_turn_pi_tie_hysteresis_rad", 0.05)

        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.protocol_version = str(
            self.get_parameter("protocol_version").value
        ).lower()
        if self.protocol_version not in ("v2", "v3"):
            raise ValueError("protocol_version must be either v2 or v3")
        self.require_localization = bool(
            self.get_parameter("require_localization").value
        )
        self.localization_timeout = float(
            self.get_parameter("localization_timeout_sec").value
        )
        self.odometry_timeout = float(
            self.get_parameter("odometry_timeout_sec").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.enable_obstacle_escape = bool(
            self.get_parameter("enable_obstacle_escape").value
        )
        self.obstacle_escape_block_timeout = float(
            self.get_parameter("obstacle_escape_block_timeout_sec").value
        )
        self.obstacle_escape_duration = float(
            self.get_parameter("obstacle_escape_duration_sec").value
        )
        self.obstacle_escape_cooldown = float(
            self.get_parameter("obstacle_escape_cooldown_sec").value
        )
        self.obstacle_escape_x = float(
            self.get_parameter("obstacle_escape_x_mps").value
        )
        self.obstacle_escape_y = float(
            self.get_parameter("obstacle_escape_y_mps").value
        )
        self.obstacle_escape_max_range = float(
            self.get_parameter("obstacle_escape_max_range_m").value
        )
        self.obstacle_escape_radius = float(
            self.get_parameter("obstacle_escape_radius_m").value
        )
        self.obstacle_escape_clearance = float(
            self.get_parameter("obstacle_escape_clearance_m").value
        )
        self.obstacle_escape_data_timeout = float(
            self.get_parameter("obstacle_escape_data_timeout_sec").value
        )
        self.body_to_base_x = float(
            self.get_parameter("body_to_base_x_m").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        self.yaw_diagnostic_period = float(
            self.get_parameter("yaw_diagnostic_period_sec").value
        )
        self.plan_timeout = float(self.get_parameter("plan_timeout_sec").value)
        self.yaw_goal_tolerance = float(
            self.get_parameter("yaw_goal_tolerance_rad").value
        )
        self.goal_rotation_activation_distance = float(
            self.get_parameter("goal_rotation_activation_distance_m").value
        )
        self.goal_rotation_fine_tune_angle = float(
            self.get_parameter("goal_rotation_fine_tune_angle_rad").value
        )
        self.goal_rotation_speed = float(
            self.get_parameter("goal_rotation_speed_rad_s").value
        )
        self.min_linear = float(self.get_parameter("min_linear_speed_mps").value)
        self.linear_deadband = float(
            self.get_parameter("linear_deadband_mps").value
        )
        self.lateral_motion_threshold = float(
            self.get_parameter("lateral_motion_threshold_mps").value
        )
        self.forward_turn_in_place_angle = float(
            self.get_parameter("forward_turn_in_place_angle_rad").value
        )
        self.max_desired_x = float(
            self.get_parameter("max_desired_linear_x_mps").value
        )
        self.max_desired_y = float(
            self.get_parameter("max_desired_linear_y_mps").value
        )
        self.max_desired_w = float(
            self.get_parameter("max_desired_angular_z_rad_s").value
        )
        self.enable_reverse_recovery = bool(
            self.get_parameter("enable_reverse_recovery").value
        )
        self.max_desired_reverse_x = float(
            self.get_parameter("max_desired_reverse_x_mps").value
        )
        self.max_output_x = float(
            self.get_parameter("max_output_linear_x_mps").value
        )
        self.max_output_y = float(
            self.get_parameter("max_output_linear_y_mps").value
        )
        self.max_output_w = float(
            self.get_parameter("max_output_angular_z_rad_s").value
        )
        self.linear_command_scale = float(
            self.get_parameter("linear_command_scale").value
        )
        self.velocity_passthrough = bool(
            self.get_parameter("velocity_passthrough").value
        )
        if self.velocity_passthrough:
            self.get_logger().warning(
                "velocity_passthrough ON: /cmd_vel_safe is forwarded verbatim; "
                "the calibration inverse map, breakaway boost, rear-turn latch, "
                "goal rotation and output clamps are all bypassed"
            )
        self.speed_calibrations = {}
        for axis in ("x", "y"):
            for direction in ("positive", "negative"):
                name = f"{axis}_{direction}"
                actual = tuple(
                    float(value)
                    for value in self.get_parameter(f"{name}_actual_mps").value
                )
                commands = tuple(
                    float(value)
                    for value in self.get_parameter(f"{name}_command_mps").value
                )
                _validate_speed_calibration(actual, commands, name)
                self.speed_calibrations[name] = (actual, commands)
        self.motion_detect_on = float(
            self.get_parameter("motion_detect_on_mps").value
        )
        self.motion_detect_off = float(
            self.get_parameter("motion_detect_off_mps").value
        )
        self.max_localization_step = float(
            self.get_parameter("max_localization_step_m").value
        )
        self.max_localization_speed = float(
            self.get_parameter("max_localization_speed_mps").value
        )
        self.max_localization_yaw_step = float(
            self.get_parameter("max_localization_yaw_step_rad").value
        )
        self.localization_min_sample_dt = float(
            self.get_parameter("localization_min_sample_dt_sec").value
        )
        self.localization_min_speed_jump_step = float(
            self.get_parameter("localization_min_speed_jump_step_m").value
        )
        self.initialpose_frame_id = str(
            self.get_parameter("initialpose_frame_id").value
        ).lstrip("/")
        initialpose_hold = float(self.get_parameter("initialpose_hold_sec").value)
        initialpose_stable_samples = int(
            self.get_parameter("initialpose_stable_samples").value
        )
        self.localization_jump_auto_recovery_enabled = bool(
            self.get_parameter("localization_jump_auto_recovery_enabled").value
        )
        localization_jump_recovery_hold = float(
            self.get_parameter("localization_jump_recovery_hold_sec").value
        )
        localization_jump_recovery_stable_samples = int(
            self.get_parameter("localization_jump_recovery_stable_samples").value
        )
        self.localization_jump_recovery_max_attempts = int(
            self.get_parameter("localization_jump_recovery_max_attempts").value
        )
        self.localization_jump_recovery_max_linear_speed = float(
            self.get_parameter(
                "localization_jump_recovery_max_linear_speed_mps"
            ).value
        )
        self.localization_jump_recovery_max_angular_speed = float(
            self.get_parameter(
                "localization_jump_recovery_max_angular_speed_rad_s"
            ).value
        )
        rotation_watchdog_command = float(
            self.get_parameter("rotation_watchdog_command_rad_s").value
        )
        rotation_watchdog_angle = float(
            self.get_parameter("rotation_watchdog_max_angle_rad").value
        )
        rotation_watchdog_duration = float(
            self.get_parameter("rotation_watchdog_max_duration_sec").value
        )
        self.rotation_fault_recovery_timeout = float(
            self.get_parameter("rotation_fault_recovery_timeout_sec").value
        )
        self.rear_turn_sample_distance = float(
            self.get_parameter("rear_turn_sample_distance_m").value
        )
        rear_turn_engage_angle = float(
            self.get_parameter("rear_turn_engage_angle_rad").value
        )
        rear_turn_release_angle = float(
            self.get_parameter("rear_turn_release_angle_rad").value
        )
        rear_turn_alignment_release_angle = float(
            self.get_parameter("rear_turn_alignment_release_angle_rad").value
        )
        self.rear_turn_alignment_speed = float(
            self.get_parameter("rear_turn_alignment_speed_rad_s").value
        )
        rear_turn_command_threshold = float(
            self.get_parameter("rear_turn_command_threshold_rad_s").value
        )
        rear_turn_translation_threshold = float(
            self.get_parameter("rear_turn_translation_threshold_mps").value
        )
        rear_turn_pi_tie_hysteresis = float(
            self.get_parameter("rear_turn_pi_tie_hysteresis_rad").value
        )
        if (
            publish_rate <= 0.0
            or self.command_timeout <= 0.0
            or self.odometry_timeout <= 0.0
            or self.yaw_diagnostic_period <= 0.0
            or self.plan_timeout <= 0.0
        ):
            raise ValueError("publish, timeout and diagnostic periods must be positive")
        if min(
            self.max_desired_x,
            self.max_desired_y,
            self.max_desired_w,
            self.max_output_x,
            self.max_output_y,
            self.max_output_w,
            self.yaw_goal_tolerance,
            self.goal_rotation_activation_distance,
            self.goal_rotation_fine_tune_angle,
            self.goal_rotation_speed,
            self.linear_command_scale,
        ) <= 0.0:
            raise ValueError("velocity limits must be positive")
        if not 0.0 <= self.linear_deadband < self.min_linear:
            raise ValueError("linear deadband must be nonnegative and below minimum speed")
        if not 0.0 <= self.lateral_motion_threshold <= self.max_desired_y:
            raise ValueError("lateral motion threshold must be within the Y limit")
        if not 0.0 < self.forward_turn_in_place_angle < math.pi / 2.0:
            raise ValueError("forward turn-in-place angle must be in (0, pi/2)")
        if self.goal_rotation_speed > self.max_desired_w:
            raise ValueError("goal rotation speed must not exceed the yaw limit")
        if not self.yaw_goal_tolerance < self.goal_rotation_fine_tune_angle < math.pi:
            raise ValueError("goal rotation fine-tune angle must exceed yaw tolerance")
        if self.min_linear > min(self.max_output_x, self.max_output_y):
            raise ValueError("minimum linear speed must not exceed an axis limit")
        if not 0.0 <= self.motion_detect_off < self.motion_detect_on:
            raise ValueError("motion detection hysteresis is invalid")
        if min(
            self.max_localization_step,
            self.max_localization_speed,
            self.max_localization_yaw_step,
        ) <= 0.0:
            raise ValueError("localization jump limits must be positive")
        if (
            not self.initialpose_frame_id
            or initialpose_hold <= 0.0
            or initialpose_stable_samples <= 0
        ):
            raise ValueError("initial-pose frame, hold and stable samples are invalid")
        if (
            localization_jump_recovery_hold <= 0.0
            or localization_jump_recovery_stable_samples <= 0
            or self.localization_jump_recovery_max_attempts < 0
            or self.localization_jump_recovery_max_linear_speed < 0.0
            or self.localization_jump_recovery_max_angular_speed < 0.0
            or (
                self.localization_jump_auto_recovery_enabled
                and self.localization_jump_recovery_max_attempts == 0
            )
        ):
            raise ValueError("localization-jump recovery parameters are invalid")
        if not 0.0 < rotation_watchdog_command <= self.max_desired_w:
            raise ValueError("rotation watchdog threshold must be in (0, max yaw]")
        if min(rotation_watchdog_angle, rotation_watchdog_duration) <= 0.0:
            raise ValueError("rotation watchdog limits must be positive")
        if (
            self.rear_turn_sample_distance <= 0.0
            or not 0.0 < rear_turn_release_angle < rear_turn_engage_angle <= math.pi
            or not 0.0
            < rear_turn_alignment_release_angle
            < rear_turn_release_angle
            or not 0.0 < self.rear_turn_alignment_speed <= self.max_desired_w
            or not 0.0 < rear_turn_command_threshold <= self.max_desired_w
            or rear_turn_translation_threshold < 0.0
            or not 0.0 <= rear_turn_pi_tie_hysteresis < math.pi / 2.0
        ):
            raise ValueError("rear-turn direction latch parameters are invalid")

        self.last_command: Twist | None = None
        self.last_command_time = 0.0
        self.last_desired_command: Twist | None = None
        self.last_desired_command_time = 0.0
        self.latest_obstacle_xy: tuple[float, float] | None = None
        self.latest_left_clearance: float | None = None
        self.latest_right_clearance: float | None = None
        self.last_obstacle_time = 0.0
        self.obstacle_block_started: float | None = None
        self.obstacle_escape_started: float | None = None
        self.obstacle_escape_cooldown_until = 0.0
        self.last_localization_time = 0.0
        self.last_odometry_time = 0.0
        self.measured_linear_speed = 0.0
        self.measured_twist = (0.0, 0.0, 0.0)
        self.chassis_moving = False
        self.previous_localization: tuple[float, float, float, float] | None = None
        self.current_yaw: float | None = None
        self.plan_goal_xy: tuple[float, float] | None = None
        self.plan_goal_yaw: float | None = None
        self.last_plan_time = 0.0
        self.last_yaw_diagnostic_time = 0.0
        self.last_output = Twist()
        self.last_effective_angular_command = 0.0
        self.pose_seen = False
        self.powered = False
        self.power_request_pending = False
        self.localization_fault_latched = False
        self.localization_fault_reason: str | None = None
        self.localization_jump_recovery_attempts = 0
        self.command_conflict_latched = False
        self.rotation_fault_latched = False
        self.rotation_fault_latched_time = 0.0
        self.rotation_watchdog = RotationWatchdog(
            rotation_watchdog_command,
            rotation_watchdog_angle,
            rotation_watchdog_duration,
        )
        self.rear_turn_latch = RearTurnDirectionLatch(
            rear_turn_engage_angle,
            rear_turn_release_angle,
            rear_turn_alignment_release_angle,
            rear_turn_command_threshold,
            rear_turn_translation_threshold,
            rear_turn_pi_tie_hysteresis,
        )
        self.relocalization_gate = RelocalizationGate(
            initialpose_hold,
            initialpose_stable_samples,
        )
        self.localization_jump_recovery = LocalizationJumpRecovery(
            localization_jump_recovery_hold,
            localization_jump_recovery_stable_samples,
        )
        self.reported_waiting = False
        self.reported_waiting_odometry = False
        self.reported_static_friction_compensation = False
        self.reported_rear_turn_override = False
        self.reported_rear_alignment_forced = False
        self.reported_lateral_suppression = False
        self.reported_forward_alignment = False
        self.reported_goal_rotation_limit = False

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.yaw_diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("yaw_tracking_topic").value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._on_command,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("desired_velocity_topic").value),
            self._on_desired_command,
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("obstacle_topic").value),
            self._on_obstacle_points,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("localization_topic").value),
            self._on_localization,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            self._on_initial_pose,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("alignment_status_topic").value),
            self._on_alignment_status,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            20,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("plan_topic").value),
            self._on_plan,
            10,
        )
        self.power_client = self.create_client(
            SetCtrlSrc, str(self.get_parameter("power_service").value)
        )
        self.create_timer(1.0 / publish_rate, self._on_timer)

        state = "ARM REQUESTED" if self.enable_motion else "DISARMED"
        self.get_logger().warning(
            f"{self.protocol_version.upper()} Nav2 command bridge {state}: desired limits "
            f"x={self.max_desired_x:.2f}, y={self.max_desired_y:.2f} m/s, "
            f"yaw={self.max_desired_w:.2f} rad/s; raw limits "
            f"x={self.max_output_x:.2f}, y={self.max_output_y:.2f} m/s, "
            f"yaw={self.max_output_w:.2f} rad/s; breakaway="
            f"{self.min_linear:.2f} m/s"
        )
        if not self.enable_motion:
            self.get_logger().warning(
                "Only zero velocity is sent. Relaunch with enable_motion:=true "
                "after localization and costmaps are verified."
            )

    def _on_command(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("Rejected non-finite Nav2 velocity command")
            return
        self.last_command = msg
        self.last_command_time = time.monotonic()

    def _on_desired_command(self, msg: Twist) -> None:
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            return
        self.last_desired_command = msg
        self.last_desired_command_time = time.monotonic()

    def _on_obstacle_points(self, msg: PointCloud2) -> None:
        nearest: tuple[float, float] | None = None
        left_clearance: float | None = None
        right_clearance: float | None = None
        best_distance = self.obstacle_escape_max_range
        best_left = self.obstacle_escape_max_range
        best_right = self.obstacle_escape_max_range
        try:
            for point in pc2.read_points(
                msg, field_names=("x", "y"), skip_nans=True
            ):
                x = float(point[0]) - self.body_to_base_x
                y = float(point[1])
                distance = math.hypot(x, y)
                if 0.05 < distance <= best_distance:
                    best_distance = distance
                    nearest = (x, y)
                if y >= 0.02 and distance <= best_left:
                    best_left = distance
                    left_clearance = distance
                elif y <= -0.02 and distance <= best_right:
                    best_right = distance
                    right_clearance = distance
        except Exception:
            nearest = None
        self.latest_obstacle_xy = nearest
        self.latest_left_clearance = left_clearance
        self.latest_right_clearance = right_clearance
        self.last_obstacle_time = time.monotonic()

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if (
            msg.header.frame_id.lstrip("/") != self.initialpose_frame_id
            or not all(math.isfinite(value) for value in values)
            or quaternion_norm <= 1.0e-6
        ):
            self.get_logger().error(
                "Ignored invalid initial pose; expected a finite pose in frame "
                f"'{self.initialpose_frame_id}'"
            )
            return

        now = time.monotonic()
        self.relocalization_gate.begin(now)
        self.localization_jump_recovery.cancel()
        self.localization_jump_recovery_attempts = 0
        if (
            self.localization_fault_latched
            and self.localization_fault_reason == "jump"
        ):
            self.localization_fault_latched = False
            self.localization_fault_reason = None
            self.get_logger().warning(
                "Cleared localization-jump latch for authorized initial-pose reset"
            )
        self.previous_localization = None
        self.current_yaw = None
        self.plan_goal_xy = None
        self.plan_goal_yaw = None
        self.last_plan_time = 0.0
        self.pose_seen = False
        self.last_localization_time = 0.0
        self.last_odometry_time = 0.0
        self.last_command = None
        self.last_command_time = 0.0
        self.measured_linear_speed = 0.0
        self.measured_twist = (0.0, 0.0, 0.0)
        self.chassis_moving = False
        self.rotation_watchdog.reset()
        self.rear_turn_latch.reset()
        self.last_effective_angular_command = 0.0
        self.reported_waiting = False
        self.reported_waiting_odometry = False
        self.reported_static_friction_compensation = False
        self.reported_rear_turn_override = False
        self.reported_rear_alignment_forced = False
        self.reported_lateral_suppression = False
        self.reported_forward_alignment = False
        self.reported_goal_rotation_limit = False
        self._publish_zero()
        self.get_logger().warning(
            "Initial-pose reset accepted: zero chassis velocity sent; holding at "
            f"least {self.relocalization_gate.hold_sec:.1f} s "
            "for stable localization"
        )

    def _on_localization(self, msg: PoseWithCovarianceStamped) -> None:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        if stamp <= 0.0:
            stamp = time.monotonic()
        current = (
            stamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _yaw_from_pose(msg),
        )
        self.current_yaw = current[3]
        previous = self.previous_localization
        self.previous_localization = current
        comparable = False
        jump_detected = False
        step = 0.0
        yaw_step = 0.0
        speed = 0.0
        if previous is not None:
            dt = current[0] - previous[0]
            if 0.0 < dt <= self.localization_timeout * 2.0:
                comparable = True
                step = math.hypot(current[1] - previous[1], current[2] - previous[2])
                yaw_step = abs(_normalize_angle(current[3] - previous[3]))
                speed = step / dt
                speed_jump = (
                    dt >= self.localization_min_sample_dt
                    and step >= self.localization_min_speed_jump_step
                    and speed > self.max_localization_speed
                )
                jump_detected = (
                    step > self.max_localization_step
                    or speed_jump
                    or (
                        dt >= self.localization_min_sample_dt
                        and yaw_step > self.max_localization_yaw_step
                    )
                )

        if self.localization_fault_latched:
            self.localization_jump_recovery.observe_pose(
                comparable and not jump_detected,
                time.monotonic(),
            )
            self.pose_seen = True
            return

        if (
            jump_detected
            and self.powered
            and not self.relocalization_gate.active
        ):
            self.localization_fault_latched = True
            self.localization_fault_reason = "jump"
            self.last_localization_time = 0.0
            can_auto_recover = (
                self.localization_jump_auto_recovery_enabled
                and self.localization_jump_recovery_attempts
                < self.localization_jump_recovery_max_attempts
            )
            if can_auto_recover:
                self.localization_jump_recovery.begin()
            else:
                self.localization_jump_recovery.cancel()
            self.get_logger().error(
                "Localization jump detected; motion latched disabled: "
                f"step={step:.3f} m speed={speed:.3f} m/s "
                f"yaw_step={yaw_step:.3f} rad"
            )
            if can_auto_recover:
                self.get_logger().warning(
                    "Attempting guarded localization-jump recovery while holding "
                    "zero chassis velocity"
                )
            else:
                self.get_logger().error(
                    "Automatic localization-jump recovery is unavailable; use an "
                    "accurate 2D Pose Estimate to re-arm motion"
                )
            return
        self.pose_seen = True

    def _on_odometry(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        angular = msg.twist.twist.angular.z
        if not all(math.isfinite(value) for value in (vx, vy, angular)):
            return
        now = time.monotonic()
        self.last_odometry_time = now
        self.measured_twist = (vx, vy, angular)
        self.measured_linear_speed = math.hypot(vx, vy)
        if self.measured_linear_speed >= self.motion_detect_on:
            self.chassis_moving = True
        elif self.measured_linear_speed <= self.motion_detect_off:
            self.chassis_moving = False

        command_fresh = (
            self.last_command is not None
            and now - self.last_command_time <= self.command_timeout
        )
        command_angular = (
            self.last_effective_angular_command
            if self.powered and command_fresh and not self.rotation_fault_latched
            else 0.0
        )
        if self.rotation_watchdog.update(now, command_angular, angular):
            self.rotation_fault_latched = True
            self.rotation_fault_latched_time = now
            self.get_logger().error(
                "Continuous rotation limit exceeded; motion latched disabled: "
                f"angle={self.rotation_watchdog.measured_angle:.3f} rad "
                f"duration={now - self.rotation_watchdog.start_time:.1f} s"
            )

    def _on_plan(self, msg: Path) -> None:
        if not msg.poses:
            return
        goal = msg.poses[-1].pose
        self.plan_goal_xy = (goal.position.x, goal.position.y)
        self.plan_goal_yaw = _yaw_from_orientation(goal.orientation)
        self.last_plan_time = time.monotonic()
        current = self.previous_localization
        if current is None:
            return
        heading_error = _path_heading_error(
            current[1],
            current[2],
            current[3],
            tuple(
                (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
            ),
            self.rear_turn_sample_distance,
        )
        previous_direction = self.rear_turn_latch.direction
        previous_alignment = self.rear_turn_latch.alignment_active
        if not self.rear_turn_latch.update(heading_error):
            return
        self.reported_rear_turn_override = False
        if previous_alignment != self.rear_turn_latch.alignment_active:
            self.reported_rear_alignment_forced = False
        if previous_direction != self.rear_turn_latch.direction:
            if self.rear_turn_latch.direction == 0:
                self.get_logger().info(
                    "Rear-path turn-direction latch released; translation remains "
                    "suppressed until the path heading is aligned"
                )
            else:
                direction = (
                    "counter-clockwise"
                    if self.rear_turn_latch.direction > 0
                    else "clockwise"
                )
                self.get_logger().warning(
                    f"Rear path detected; holding initial {direction} turn direction "
                    f"(heading error {heading_error:+.3f} rad)"
                )
        if not previous_alignment and self.rear_turn_latch.alignment_active:
            self.get_logger().warning(
                "Rear-path alignment active; suppressing translation until heading "
                f"error is within {self.rear_turn_latch.alignment_release_angle:.3f} rad"
            )
        elif previous_alignment and not self.rear_turn_latch.alignment_active:
            self.get_logger().info(
                "Rear-path alignment complete; translation released "
                f"(heading error {heading_error:+.3f} rad)"
            )

    def _on_alignment_status(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            values = {entry.key: entry.value for entry in status.values}
            if "fitness_score" not in values:
                continue
            reinit = values.get("reinitialization_requested", "false").lower()
            healthy = status.level in (0, b"\x00") and reinit != "true"
            recovery_healthy = (
                healthy
                and values.get("has_converged", "false").lower() == "true"
                and values.get("failure_category", "") == "healthy"
            )
            self.last_localization_time = time.monotonic() if healthy else 0.0
            self.relocalization_gate.observe_alignment(healthy)
            self.localization_jump_recovery.observe_alignment(recovery_healthy)
            return

    def _localization_fresh(self, now: float) -> bool:
        return (
            not self.require_localization
            or (
                self.pose_seen
                and now - self.last_localization_time <= self.localization_timeout
            )
        )

    def _odometry_fresh(self, now: float) -> bool:
        return now - self.last_odometry_time <= self.odometry_timeout

    def _block_reason(self, now: float) -> str:
        if not self.enable_motion:
            return "enable_motion_false"
        if self.command_conflict_latched or self.count_publishers(
            self.output_topic
        ) > 1:
            return "competing_publisher"
        if self.relocalization_gate.active:
            return "initialpose_hold"
        if self.localization_fault_latched:
            if self.localization_jump_recovery.active:
                return "localization_jump_recovery"
            return "localization_jump_latched"
        if self.rotation_fault_latched:
            return "rotation_fault_latched"
        if self.obstacle_escape_started is not None:
            return "obstacle_escape"
        if not self._localization_fresh(now):
            return "localization_stale"
        if not self._odometry_fresh(now):
            return "odometry_stale"
        command_fresh = (
            self.last_command is not None
            and now - self.last_command_time <= self.command_timeout
        )
        desired_x = self.last_command.linear.x if command_fresh else 0.0
        desired_y = self.last_command.linear.y if command_fresh else 0.0
        desired_w = self.last_command.angular.z if command_fresh else 0.0
        if not command_fresh:
            return "command_timeout"
        if not _requests_motion(
            desired_x,
            desired_y,
            desired_w,
            self.linear_deadband,
            allow_reverse=self.enable_reverse_recovery,
        ):
            return "no_motion_request"
        if not self.powered:
            return "power_pending"
        return "outputting"

    def _request_power(self, value: int) -> None:
        if self.power_request_pending or not self.power_client.service_is_ready():
            return
        self.power_request_pending = True
        request = SetCtrlSrc.Request()
        request.value = value
        future = self.power_client.call_async(request)

        def done(result_future) -> None:
            self.power_request_pending = False
            try:
                response = result_future.result()
            except Exception as exc:  # rclpy service exceptions vary by RMW
                self.get_logger().error(
                    f"{self.protocol_version.upper()} power service failed: {exc}"
                )
                return
            if not response.success:
                self.get_logger().error(
                    f"{self.protocol_version.upper()} power request {value} rejected: "
                    f"{response.message}"
                )
                return
            self.powered = value != 0
            self.get_logger().warning(
                f"{self.protocol_version.upper()} whole-robot software power enabled"
                if self.powered
                else f"{self.protocol_version.upper()} whole-robot software power disabled"
            )

        future.add_done_callback(done)

    def _publish_zero(self) -> None:
        self.last_effective_angular_command = 0.0
        self.last_output = Twist()
        self.publisher.publish(self.last_output)

    def _obstacle_escape_xy(self) -> tuple[float, float]:
        desired_x = 0.0
        left = self.latest_left_clearance
        right = self.latest_right_clearance
        if right is None and left is None:
            desired_y = self.obstacle_escape_y
        elif right is None or (left is not None and left <= right):
            desired_y = -self.obstacle_escape_y
        else:
            desired_y = self.obstacle_escape_y
        return desired_x, desired_y

    def _publish_yaw_diagnostic(self, now: float) -> None:
        if (
            self.plan_goal_yaw is None
            or self.current_yaw is None
            or now - self.last_plan_time > self.plan_timeout
            or now - self.last_yaw_diagnostic_time < self.yaw_diagnostic_period
        ):
            return
        self.last_yaw_diagnostic_time = now
        error = _normalize_angle(self.plan_goal_yaw - self.current_yaw)
        command_fresh = (
            self.last_command is not None
            and now - self.last_command_time <= self.command_timeout
        )
        desired_x = self.last_command.linear.x if command_fresh else 0.0
        desired_y = self.last_command.linear.y if command_fresh else 0.0
        desired_w = self.last_command.angular.z if command_fresh else 0.0
        measured_x, measured_y, measured_w = self.measured_twist
        direction_consistent = (
            abs(desired_w) < 1.0e-3 or desired_w * error >= 0.0
        )
        block_reason = self._block_reason(now)

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "Nav2 final yaw tracking"
        status.hardware_id = f"mantis_{self.protocol_version}_chassis"
        status.message = (
            "within tolerance"
            if abs(error) <= self.yaw_goal_tolerance
            else "tracking final plan yaw"
        )
        values = {
            "target_yaw_rad": self.plan_goal_yaw,
            "current_yaw_rad": self.current_yaw,
            "yaw_error_rad": error,
            "yaw_tolerance_rad": self.yaw_goal_tolerance,
            "nav2_linear_x_mps": desired_x,
            "output_linear_x_command_mps": self.last_output.linear.x,
            "measured_linear_x_mps": measured_x,
            "linear_x_error_mps": desired_x - measured_x,
            "nav2_linear_y_mps": desired_y,
            "output_linear_y_command_mps": self.last_output.linear.y,
            "measured_linear_y_mps": measured_y,
            "linear_y_error_mps": desired_y - measured_y,
            "nav2_angular_z_rad_s": desired_w,
            "output_angular_z_rad_s": self.last_output.angular.z,
            "measured_angular_z_rad_s": measured_w,
            "block_reason": block_reason,
            "localization_jump_recovery_active": str(
                self.localization_jump_recovery.active
            ).lower(),
            "localization_jump_recovery_stable_samples": (
                str(self.localization_jump_recovery.stable_samples)
            ),
            "localization_jump_recovery_attempts": (
                str(self.localization_jump_recovery_attempts)
            ),
            "command_direction_consistent": str(direction_consistent).lower(),
        }
        status.values = [
            KeyValue(
                key=key,
                value=f"{value:.6f}" if isinstance(value, float) else value,
            )
            for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.yaw_diagnostic_publisher.publish(message)
        self.get_logger().info(
            "Yaw tracking: "
            f"reason={block_reason} "
            f"target={self.plan_goal_yaw:+.3f} current={self.current_yaw:+.3f} "
            f"error={error:+.3f} nav2_w={desired_w:+.3f} "
            f"raw_w={self.last_output.angular.z:+.3f} measured_w={measured_w:+.3f}; "
            f"desired_xy=({desired_x:+.3f},{desired_y:+.3f}) "
            f"raw_xy=({self.last_output.linear.x:+.3f},"
            f"{self.last_output.linear.y:+.3f}) "
            f"measured_xy=({measured_x:+.3f},{measured_y:+.3f})"
        )

    def _map_translation(
        self, desired_x: float, desired_y: float
    ) -> tuple[float, float, bool]:
        desired_x = _clamp(desired_x, self.max_desired_x)
        if desired_x < 0.0:
            if not self.enable_reverse_recovery:
                desired_x = 0.0
            else:
                desired_x = max(desired_x, -self.max_desired_reverse_x)
        desired_y = _clamp(desired_y, self.max_desired_y)
        if math.hypot(desired_x, desired_y) < self.linear_deadband:
            return 0.0, 0.0, False

        x_positive = self.speed_calibrations["x_positive"]
        x_negative = self.speed_calibrations["x_negative"]
        y_positive = self.speed_calibrations["y_positive"]
        y_negative = self.speed_calibrations["y_negative"]
        raw_x = _map_signed_speed(desired_x, *x_positive, *x_negative)
        raw_y = _map_signed_speed(desired_y, *y_positive, *y_negative)
        if desired_x < 0.0:
            # Recovery backing still needs the measured breakaway command to
            # overcome static friction, but the desired speed is capped low.
            return _shape_translation(
                raw_x * self.linear_command_scale,
                raw_y * self.linear_command_scale,
                self.min_linear,
                0.0,
                self.max_output_x,
                self.max_output_y,
                startup_boost=not self.chassis_moving,
            )
        return _shape_translation(
            raw_x * self.linear_command_scale,
            raw_y * self.linear_command_scale,
            self.min_linear,
            0.0,
            self.max_output_x,
            self.max_output_y,
            startup_boost=not self.chassis_moving,
        )

    def _on_timer(self) -> None:
        now = time.monotonic()
        self._publish_yaw_diagnostic(now)
        localization_fresh = self._localization_fresh(now)
        odometry_fresh = self._odometry_fresh(now)

        if not self.enable_motion:
            self._publish_zero()
            return

        if self.command_conflict_latched or self.count_publishers(
            self.output_topic
        ) > 1:
            self._publish_zero()
            if not self.command_conflict_latched:
                self.command_conflict_latched = True
                self.get_logger().error(
                    f"Competing publisher detected on {self.output_topic}; "
                    "zero velocity sent and motion latched disabled. Stop the "
                    "teleop process and restart navigation."
                )
            return

        if self.relocalization_gate.active:
            self._publish_zero()
            if self.relocalization_gate.try_release(
                now,
                localization_fresh,
                odometry_fresh,
            ):
                self.previous_localization = None
                self.get_logger().warning(
                    "Initial-pose reset stabilized; navigation motion may re-arm"
                )
            return

        if self.localization_fault_latched:
            self._publish_zero()
            measured_x, measured_y, measured_w = self.measured_twist
            chassis_stationary = (
                math.hypot(measured_x, measured_y)
                <= self.localization_jump_recovery_max_linear_speed
                and abs(measured_w)
                <= self.localization_jump_recovery_max_angular_speed
            )
            if self.localization_jump_recovery.try_release(
                now,
                localization_fresh,
                odometry_fresh,
                chassis_stationary,
            ):
                self.localization_fault_latched = False
                self.localization_fault_reason = None
                self.localization_jump_recovery_attempts += 1
                self.previous_localization = None
                self.last_command = None
                self.last_command_time = 0.0
                self.plan_goal_xy = None
                self.plan_goal_yaw = None
                self.last_plan_time = 0.0
                self.rotation_watchdog.reset()
                self.rear_turn_latch.reset()
                self.last_effective_angular_command = 0.0
                self.get_logger().warning(
                    "Localization-jump auto-recovery stabilized; navigation motion "
                    "may re-arm on the next fresh command"
                )
            return

        if self.rotation_fault_latched:
            rotation_command_active = (
                self.last_command is not None
                and now - self.last_command_time <= self.command_timeout
                and abs(self.last_command.angular.z)
                >= self.rotation_watchdog.command_threshold
            )
            if (
                not rotation_command_active
                and now - self.rotation_fault_latched_time
                >= self.rotation_fault_recovery_timeout
            ):
                self.rotation_fault_latched = False
                self.rotation_watchdog.reset()
                self.get_logger().warning(
                    "Rotation watchdog reset after sustained stop; "
                    "navigation motion may resume"
                )
            else:
                self._publish_zero()
                return

        if not localization_fresh:
            self._publish_zero()
            if not self.reported_waiting:
                self.reported_waiting = True
                self.get_logger().warning(
                    "3D localization is stale; zero chassis velocity sent. "
                    "Motion will resume automatically when localization recovers"
                )
            return

        if self.reported_waiting:
            self.get_logger().warning(
                "3D localization recovered; navigation motion may resume"
            )
        self.reported_waiting = False
        if not odometry_fresh:
            self._publish_zero()
            if not self.reported_waiting_odometry:
                self.reported_waiting_odometry = True
                self.get_logger().warning(
                    "Planar odometry is stale; zero chassis velocity sent. "
                    "Motion will resume automatically when /nav2_odom recovers"
                )
            return

        if self.reported_waiting_odometry:
            self.get_logger().warning(
                "Planar odometry recovered; navigation motion may resume"
            )
        self.reported_waiting_odometry = False
        command_fresh = (
            self.last_command is not None
            and now - self.last_command_time <= self.command_timeout
        )
        desired_fresh = (
            self.last_desired_command is not None
            and now - self.last_desired_command_time <= self.command_timeout
        )
        desired_command_x = self.last_command.linear.x if command_fresh else 0.0
        desired_command_y = self.last_command.linear.y if command_fresh else 0.0
        desired_command_w = self.last_command.angular.z if command_fresh else 0.0
        nav2_x = self.last_desired_command.linear.x if desired_fresh else 0.0
        nav2_y = self.last_desired_command.linear.y if desired_fresh else 0.0
        nav2_w = self.last_desired_command.angular.z if desired_fresh else 0.0
        if self.velocity_passthrough:
            if not command_fresh:
                self._publish_zero()
                return
            if not self.powered:
                self._publish_zero()
                self._request_power(1)
                return
            output = _passthrough_command(self.last_command)
            self.last_effective_angular_command = output.angular.z
            self.last_output = output
            self.publisher.publish(output)
            return
        safe_requests_motion = command_fresh and _requests_motion(
            desired_command_x,
            desired_command_y,
            desired_command_w,
            self.linear_deadband,
            allow_reverse=self.enable_reverse_recovery,
        )
        desired_requests_motion = desired_fresh and _requests_motion(
            nav2_x,
            nav2_y,
            nav2_w,
            self.linear_deadband,
        )
        if self.enable_obstacle_escape:
            if self.obstacle_escape_started is not None:
                obstacle_data_fresh = (
                    now - self.last_obstacle_time
                    <= self.obstacle_escape_data_timeout
                )
                obstacle_xy = self.latest_obstacle_xy
                obstacle_distance = (
                    math.hypot(obstacle_xy[0], obstacle_xy[1])
                    if obstacle_xy is not None and obstacle_data_fresh
                    else None
                )
                clearance_ok = (
                    not obstacle_data_fresh
                    or obstacle_distance is None
                    or obstacle_distance > self.obstacle_escape_clearance
                )
                if (
                    now - self.obstacle_escape_started
                    >= self.obstacle_escape_duration
                    or clearance_ok
                ):
                    self._publish_zero()
                    self.obstacle_escape_started = None
                    self.obstacle_escape_cooldown_until = (
                        now + self.obstacle_escape_cooldown
                    )
                    self.obstacle_block_started = None
                    return
                escape_x, escape_y = self._obstacle_escape_xy()
                escape_output = Twist()
                (
                    escape_output.linear.x,
                    escape_output.linear.y,
                    _,
                ) = self._map_translation(escape_x, escape_y)
                escape_output.angular.z = 0.0
                self.last_output = escape_output
                self.publisher.publish(escape_output)
                return
            if (
                now >= self.obstacle_escape_cooldown_until
                and self.powered
                and desired_requests_motion
            ):
                obstacle_xy = self.latest_obstacle_xy
                obstacle_data_fresh = (
                    now - self.last_obstacle_time
                    <= self.obstacle_escape_data_timeout
                )
                obstacle_distance = (
                    math.hypot(obstacle_xy[0], obstacle_xy[1])
                    if obstacle_xy is not None and obstacle_data_fresh
                    else None
                )
                within_radius = (
                    obstacle_distance is not None
                    and obstacle_distance <= self.obstacle_escape_radius
                )
                if not safe_requests_motion or within_radius:
                    if self.obstacle_block_started is None:
                        self.obstacle_block_started = now
                    elif (
                        now - self.obstacle_block_started
                        >= self.obstacle_escape_block_timeout
                    ):
                        self.obstacle_escape_started = now
                        self.obstacle_block_started = None
                        self.get_logger().warning(
                            "Obstacle inside footprint radius or Nav2 motion "
                            "blocked; starting lateral obstacle escape"
                        )
                        return
            else:
                self.obstacle_block_started = None
        path_heading_error = (
            self.rear_turn_latch.heading_error
            if now - self.last_plan_time <= self.plan_timeout
            else None
        )
        (
            desired_command_x,
            desired_command_y,
            desired_command_w,
            lateral_suppressed,
            forward_alignment_forced,
        ) = _shape_routine_lateral_command(
            desired_command_x,
            desired_command_y,
            desired_command_w,
            self.lateral_motion_threshold,
            path_heading_error,
            self.forward_turn_in_place_angle,
            self.rear_turn_alignment_speed,
        )
        if lateral_suppressed and not self.reported_lateral_suppression:
            self.get_logger().info(
                "Converting routine lateral trim below "
                f"{self.lateral_motion_threshold:.2f} m/s into forward-aligned "
                "motion; decisive obstacle sidesteps remain enabled"
            )
        self.reported_lateral_suppression = lateral_suppressed
        if forward_alignment_forced and not self.reported_forward_alignment:
            self.get_logger().warning(
                "Routine lateral command requires a heading change; turning in "
                "place along the shortest path before moving forward"
            )
        self.reported_forward_alignment = forward_alignment_forced
        command_requests_motion = (
            command_fresh
            and _requests_motion(
                desired_command_x,
                desired_command_y,
                desired_command_w,
                self.linear_deadband,
                allow_reverse=self.enable_reverse_recovery,
            )
        )
        if not command_requests_motion:
            self._publish_zero()
            self.reported_static_friction_compensation = False
            return

        if not self.powered:
            self._publish_zero()
            self._request_power(1)
            return

        output = Twist()
        desired_x, desired_y, translation_suppressed = (
            self.rear_turn_latch.shape_translation(
                desired_command_x,
                desired_command_y,
            )
        )
        output.linear.x, output.linear.y, compensated = self._map_translation(
            desired_x,
            desired_y,
        )
        if translation_suppressed:
            self.reported_static_friction_compensation = False
        if compensated and not self.reported_static_friction_compensation:
            self.reported_static_friction_compensation = True
            self.get_logger().warning(
                "Static-friction compensation active: raising nonzero "
                f"translation to {self.min_linear:.2f} m/s"
            )
        desired_w = desired_command_w
        desired_w, rear_turn_overridden = self.rear_turn_latch.shape(
            desired_x,
            desired_y,
            desired_w,
        )
        if rear_turn_overridden and not self.reported_rear_turn_override:
            self.reported_rear_turn_override = True
            self.get_logger().warning(
                "Ignored a transient Nav2 yaw-command reversal while escaping "
                "the rear-heading ambiguity"
            )
        desired_w, alignment_forced = self.rear_turn_latch.shape_alignment_rotation(
            desired_w,
            self.rear_turn_alignment_speed,
        )
        if alignment_forced and not self.reported_rear_alignment_forced:
            self.reported_rear_alignment_forced = True
            self.get_logger().warning(
                "Nav2 yaw command was insufficient during rear-path alignment; "
                f"forcing in-place turn at {desired_w:+.3f} rad/s"
            )
        localization = self.previous_localization
        current_xy = (
            (localization[1], localization[2]) if localization is not None else None
        )
        plan_fresh = now - self.last_plan_time <= self.plan_timeout
        desired_w, goal_rotation_active, goal_yaw_error = _shape_goal_rotation(
            desired_x,
            desired_y,
            desired_w,
            current_xy,
            self.current_yaw,
            self.plan_goal_xy if plan_fresh else None,
            self.plan_goal_yaw if plan_fresh else None,
            self.goal_rotation_activation_distance,
            self.linear_deadband,
            self.goal_rotation_fine_tune_angle,
            self.goal_rotation_speed,
            self.yaw_goal_tolerance,
        )
        if goal_rotation_active and not self.reported_goal_rotation_limit:
            self.get_logger().warning(
                "Final goal-yaw adjustment active: shortest-direction rotation "
                f"limited to {self.goal_rotation_speed:.2f} rad/s "
                f"(yaw error {goal_yaw_error:+.3f} rad)"
            )
        self.reported_goal_rotation_limit = goal_rotation_active
        self.last_effective_angular_command = desired_w
        output.angular.z = _clamp(desired_w, self.max_output_w)
        self.last_output = output
        self.publisher.publish(output)

    def stop_chassis(self) -> None:
        for _ in range(5):
            self._publish_zero()


def main(args=None) -> None:
    # Keep the context alive while handling SIGINT so the shutdown path can
    # publish its final zero chassis commands first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavCmdBridge()
    shutdown_requested = False

    def request_shutdown(_signum, _frame) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_chassis()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
