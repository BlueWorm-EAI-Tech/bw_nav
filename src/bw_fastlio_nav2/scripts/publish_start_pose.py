#!/usr/bin/env python3
"""Publish the named start point once on /initialpose at navigation startup."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_named_point(
    path: Path,
    name: str,
    expected_frame: str,
    expected_child_frame: str,
) -> dict:
    """Read one named point and validate the fields needed for /initialpose."""
    source = Path(path).expanduser()
    try:
        data = json.loads(source.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ValueError(f"points file not found: {source}") from exc
    except OSError as exc:
        raise ValueError(f"cannot read points file {source}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in points file {source}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"points file {source} must contain a JSON object")
    point = data.get(name)
    if not isinstance(point, dict):
        available = "、".join(sorted(data)) if data else "（无）"
        raise ValueError(f"point {name!r} not found in {source}; available: {available}")
    if point.get("frame_id") != expected_frame:
        raise ValueError(
            f"point {name!r} frame_id is {point.get('frame_id')!r}, "
            f"expected {expected_frame!r}"
        )
    if point.get("child_frame_id") != expected_child_frame:
        raise ValueError(
            f"point {name!r} child_frame_id is {point.get('child_frame_id')!r}, "
            f"expected {expected_child_frame!r}"
        )
    try:
        x = float(point["x"])
        y = float(point["y"])
        z = float(point.get("z", 0.0))
        qx = float(point["qx"])
        qy = float(point["qy"])
        qz = float(point["qz"])
        qw = float(point["qw"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"point {name!r} has invalid pose fields") from exc
    values = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"point {name!r} contains non-finite values")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-9:
        raise ValueError(f"point {name!r} has a zero-length quaternion")
    return {
        "x": x,
        "y": y,
        "z": z,
        "qx": qx / norm,
        "qy": qy / norm,
        "qz": qz / norm,
        "qw": qw / norm,
    }


def _rotate_vector(q, vector):
    """Rotate ``vector`` by the unit quaternion ``q``."""
    qx, qy, qz, qw = q
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def point_to_body_pose(point: dict, body_to_base_xyz) -> dict:
    """Convert map -> base_footprint to the map -> body pose used by the localizer.

    The launch publishes a static body -> base_footprint transform with aligned
    axes (translation only), so only the translation needs to be undone.
    """
    q = (point["qx"], point["qy"], point["qz"], point["qw"])
    rotated = _rotate_vector(q, body_to_base_xyz)
    return {
        "x": point["x"] - rotated[0],
        "y": point["y"] - rotated[1],
        "z": point["z"] - rotated[2],
        "qx": q[0],
        "qy": q[1],
        "qz": q[2],
        "qw": q[3],
    }


def make_pose_message(node: Node, pose: dict, frame: str,
                      position_std_m: float, yaw_std_rad: float):
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame
    msg.pose.pose.position.x = pose["x"]
    msg.pose.pose.position.y = pose["y"]
    msg.pose.pose.position.z = pose["z"]
    msg.pose.pose.orientation.x = pose["qx"]
    msg.pose.pose.orientation.y = pose["qy"]
    msg.pose.pose.orientation.z = pose["qz"]
    msg.pose.pose.orientation.w = pose["qw"]
    msg.pose.covariance[0] = position_std_m ** 2
    msg.pose.covariance[7] = position_std_m ** 2
    msg.pose.covariance[35] = yaw_std_rad ** 2
    return msg


class StartPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("publish_start_pose")
        self.declare_parameter("points_file", "")
        self.declare_parameter("point_name", "start")
        self.declare_parameter("initialpose_topic", "/initialpose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("point_child_frame", "base_footprint")
        self.declare_parameter("body_to_base_x", -0.14)
        self.declare_parameter("body_to_base_y", 0.0)
        self.declare_parameter("body_to_base_z", -0.18)
        self.declare_parameter("wait_timeout_sec", 120.0)
        self.declare_parameter("position_std_m", 0.5)
        self.declare_parameter("yaw_std_rad", 0.25)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("initialpose_topic").value),
            qos,
        )
        self.pose = None
        self.message = None

    def prepare(self) -> None:
        points_file = str(self.get_parameter("points_file").value).strip()
        if not points_file:
            raise ValueError("points_file parameter is empty")
        point_name = str(self.get_parameter("point_name").value).strip()
        if not point_name:
            raise ValueError("point_name parameter is empty")
        point = load_named_point(
            Path(points_file),
            point_name,
            str(self.get_parameter("global_frame").value),
            str(self.get_parameter("point_child_frame").value),
        )
        body_pose = point_to_body_pose(
            point,
            (
                float(self.get_parameter("body_to_base_x").value),
                float(self.get_parameter("body_to_base_y").value),
                float(self.get_parameter("body_to_base_z").value),
            ),
        )
        self.pose = body_pose
        self.message = make_pose_message(
            self,
            body_pose,
            str(self.get_parameter("global_frame").value),
            float(self.get_parameter("position_std_m").value),
            float(self.get_parameter("yaw_std_rad").value),
        )
        self.get_logger().info(
            f"start point {point_name!r}: map -> body "
            f"x={body_pose['x']:.3f} y={body_pose['y']:.3f} "
            f"z={body_pose['z']:.3f}"
        )

    def wait_for_subscriber_and_publish(self) -> None:
        if self.message is None:
            raise RuntimeError("prepare() must be called before publishing")
        timeout = max(0.0, float(self.get_parameter("wait_timeout_sec").value))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.publisher.get_subscription_count() > 0:
                break
        if self.publisher.get_subscription_count() <= 0:
            raise TimeoutError(
                "no /initialpose subscriber appeared within "
                f"{timeout:.0f} s; start pose not published"
            )
        self.publisher.publish(self.message)
        # Give DDS a moment to hand the reliable sample to the subscriber.
        flush_deadline = time.monotonic() + 0.2
        while time.monotonic() < flush_deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(
            "published start pose once on "
            f"{str(self.get_parameter('initialpose_topic').value)}"
        )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = StartPosePublisher()
        node.prepare()
        node.wait_for_subscriber_and_publish()
        return 0
    except (ValueError, OSError, TimeoutError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"publish_start_pose: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
