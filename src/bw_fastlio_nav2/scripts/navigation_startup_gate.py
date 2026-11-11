#!/usr/bin/env python3
"""Exit successfully once localization is stable enough to start Nav2."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import signal
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


@dataclass
class ReadinessWindow:
    """Require consecutive ready observations over a minimum wall-clock hold."""

    min_samples: int
    hold_sec: float
    sample_count: int = 0
    ready_since: float | None = None

    def observe(self, ready: bool, now: float) -> bool:
        if not ready:
            self.sample_count = 0
            self.ready_since = None
            return False

        if self.ready_since is None or now < self.ready_since:
            self.ready_since = now
            self.sample_count = 1
        else:
            self.sample_count += 1
        return (
            self.sample_count >= self.min_samples
            and now - self.ready_since >= self.hold_sec
        )


def _pose_is_valid(msg: PoseWithCovarianceStamped, global_frame: str) -> bool:
    if msg.header.frame_id.lstrip("/") != global_frame.lstrip("/"):
        return False

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
    if not all(math.isfinite(value) for value in values):
        return False

    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    return abs(quaternion_norm - 1.0) <= 0.10


class NavigationStartupGate(Node):
    def __init__(self) -> None:
        super().__init__("navigation_startup_gate")
        self.declare_parameter(
            "pose_topic", "/localization/pose_with_covariance"
        )
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "camera_init")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("stable_samples", 3)
        self.declare_parameter("stable_hold_sec", 0.5)
        self.declare_parameter("require_startup_verification", False)
        self.declare_parameter(
            "startup_status_topic", "/startup_initialization/status")
        self.declare_parameter("startup_timeout_sec", 600.0)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        stable_samples = int(self.get_parameter("stable_samples").value)
        stable_hold_sec = float(self.get_parameter("stable_hold_sec").value)
        if stable_samples <= 0 or stable_hold_sec < 0.0:
            raise ValueError("stable_samples must be positive and stable_hold_sec nonnegative")

        self.require_startup_verification = bool(
            self.get_parameter("require_startup_verification").value)
        self.startup_timeout_sec = float(
            self.get_parameter("startup_timeout_sec").value)
        self.startup_verified = not self.require_startup_verification
        self.started_at_sec = time.monotonic()
        self.last_pose = None

        self.readiness = ReadinessWindow(stable_samples, stable_hold_sec)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ready = False
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self._on_pose,
            10,
        )
        if self.require_startup_verification:
            status_qos = QoSProfile(depth=1)
            status_qos.reliability = ReliabilityPolicy.RELIABLE
            status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.status_subscription = self.create_subscription(
                String,
                str(self.get_parameter("startup_status_topic").value),
                self._on_startup_status,
                status_qos,
            )
        self.get_logger().info(
            "Waiting for stable localization before starting Nav2"
            + (
                " and a verified startup pose"
                if self.require_startup_verification else "")
        )

    def _on_startup_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.get_logger().warning("ignoring malformed startup status message")
            return
        if payload.get("state") != "active":
            return
        if not self.startup_verified:
            self.startup_verified = True
            self.get_logger().info(
                "Startup initialization reported an active verified pose")
        if self.last_pose is not None:
            self._evaluate(self.last_pose)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.last_pose = msg
        self._evaluate(msg)

    def _evaluate(self, msg: PoseWithCovarianceStamped) -> None:
        transform_ready = False
        if _pose_is_valid(msg, self.global_frame):
            global_transform_ready = self.tf_buffer.can_transform(
                self.global_frame,
                self.base_frame,
                Time(),
            )
            odom_transform_ready = self.tf_buffer.can_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
            transform_ready = global_transform_ready and odom_transform_ready

        if not self.readiness.observe(
                transform_ready and self.startup_verified, time.monotonic()):
            return

        self.ready = True
        self.get_logger().info(
            "Localization and map transform are stable; starting Nav2"
        )

    def startup_timed_out(self) -> bool:
        return (
            self.require_startup_verification
            and not self.startup_verified
            and self.startup_timeout_sec > 0.0
            and time.monotonic() - self.started_at_sec
            > self.startup_timeout_sec
        )


def main(args=None) -> int:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationStartupGate()
    shutdown_requested = False

    def request_shutdown(_signum, _frame) -> None:
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    timed_out = False
    try:
        while rclpy.ok() and not shutdown_requested and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.startup_timed_out():
                timed_out = True
                node.get_logger().error(
                    "startup pose was not verified before the timeout; "
                    "refusing to start Nav2")
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if timed_out:
        return 1
    return 0 if node.ready or shutdown_requested else 1


if __name__ == "__main__":
    raise SystemExit(main())
