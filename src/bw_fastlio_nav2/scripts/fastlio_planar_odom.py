#!/usr/bin/env python3
"""Derive planar base velocity from FAST-LIO pose-only odometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


@dataclass(frozen=True)
class PlanarPose:
    stamp_sec: float
    x: float
    y: float
    yaw: float


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _quaternion_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _normalized_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(component / norm for component in quaternion)


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = _normalized_quaternion(quaternion)
    vx, vy, vz = vector
    # q * v * conjugate(q), expanded to avoid another dependency.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _yaw_from_quaternion(
    quaternion: tuple[float, float, float, float],
) -> float:
    x, y, z, w = _normalized_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _compose_base_pose(
    body_position: tuple[float, float, float],
    body_orientation: tuple[float, float, float, float],
    body_to_base_translation: tuple[float, float, float],
    body_to_base_yaw: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    offset = _rotate_vector(body_orientation, body_to_base_translation)
    base_position = tuple(
        body_position[index] + offset[index] for index in range(3)
    )
    half_yaw = body_to_base_yaw * 0.5
    body_to_base_rotation = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    base_orientation = _normalized_quaternion(
        _quaternion_multiply(body_orientation, body_to_base_rotation)
    )
    return base_position, base_orientation


def _derive_planar_twist(
    previous: PlanarPose,
    current: PlanarPose,
) -> tuple[float, float, float, float]:
    dt = current.stamp_sec - previous.stamp_sec
    if dt <= 0.0:
        raise ValueError("odometry timestamps must increase")
    world_vx = (current.x - previous.x) / dt
    world_vy = (current.y - previous.y) / dt
    cos_yaw = math.cos(current.yaw)
    sin_yaw = math.sin(current.yaw)
    base_vx = cos_yaw * world_vx + sin_yaw * world_vy
    base_vy = -sin_yaw * world_vx + cos_yaw * world_vy
    angular_z = _normalize_angle(current.yaw - previous.yaw) / dt
    return base_vx, base_vy, angular_z, dt


class FastlioPlanarOdom(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_planar_odom")
        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/nav2_odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("body_to_base_x", -0.14)
        self.declare_parameter("body_to_base_y", 0.0)
        self.declare_parameter("body_to_base_z", -0.18)
        self.declare_parameter("body_to_base_yaw", 0.0)
        self.declare_parameter("min_dt_sec", 0.01)
        self.declare_parameter("max_dt_sec", 0.50)
        self.declare_parameter("max_pose_step_m", 0.40)
        self.declare_parameter("max_yaw_step_rad", 0.60)
        self.declare_parameter("max_linear_speed_mps", 1.0)
        self.declare_parameter("max_angular_speed_rad_s", 1.2)
        self.declare_parameter("filter_alpha", 0.35)
        self.declare_parameter("linear_deadband_mps", 0.015)
        self.declare_parameter("angular_deadband_rad_s", 0.015)

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        self.body_to_base_translation = (
            float(self.get_parameter("body_to_base_x").value),
            float(self.get_parameter("body_to_base_y").value),
            float(self.get_parameter("body_to_base_z").value),
        )
        self.body_to_base_yaw = float(
            self.get_parameter("body_to_base_yaw").value
        )
        self.min_dt = float(self.get_parameter("min_dt_sec").value)
        self.max_dt = float(self.get_parameter("max_dt_sec").value)
        self.max_step = float(self.get_parameter("max_pose_step_m").value)
        self.max_yaw_step = float(self.get_parameter("max_yaw_step_rad").value)
        self.max_linear = float(self.get_parameter("max_linear_speed_mps").value)
        self.max_angular = float(
            self.get_parameter("max_angular_speed_rad_s").value
        )
        self.alpha = float(self.get_parameter("filter_alpha").value)
        self.linear_deadband = float(
            self.get_parameter("linear_deadband_mps").value
        )
        self.angular_deadband = float(
            self.get_parameter("angular_deadband_rad_s").value
        )
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if not 0.0 < self.min_dt < self.max_dt:
            raise ValueError("odometry dt limits are invalid")
        if min(self.max_step, self.max_yaw_step, self.max_linear, self.max_angular) <= 0:
            raise ValueError("odometry safety limits must be positive")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._on_odometry,
            qos,
        )
        self.previous: PlanarPose | None = None
        self.filtered_twist = (0.0, 0.0, 0.0)
        self.rejected_samples = 0
        self.get_logger().info(
            f"FAST-LIO planar odometry: {self.get_parameter('input_topic').value} "
            f"-> {self.output_topic}, child={self.base_frame}"
        )

    def _zero_filter(self) -> None:
        self.filtered_twist = (0.0, 0.0, 0.0)

    def _publish(
        self,
        source: Odometry,
        base_position: tuple[float, float, float],
        base_orientation: tuple[float, float, float, float],
        twist: tuple[float, float, float],
    ) -> None:
        output = Odometry()
        output.header = source.header
        output.child_frame_id = self.base_frame
        output.pose.pose.position.x = base_position[0]
        output.pose.pose.position.y = base_position[1]
        output.pose.pose.position.z = base_position[2]
        output.pose.pose.orientation.x = base_orientation[0]
        output.pose.pose.orientation.y = base_orientation[1]
        output.pose.pose.orientation.z = base_orientation[2]
        output.pose.pose.orientation.w = base_orientation[3]
        output.pose.covariance = source.pose.covariance
        output.twist.twist.linear.x = twist[0]
        output.twist.twist.linear.y = twist[1]
        output.twist.twist.angular.z = twist[2]
        output.twist.covariance[0] = 0.01
        output.twist.covariance[7] = 0.01
        output.twist.covariance[35] = 0.02
        self.publisher.publish(output)

    def _on_odometry(self, msg: Odometry) -> None:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        body_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        body_orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        try:
            base_position, base_orientation = _compose_base_pose(
                body_position,
                body_orientation,
                self.body_to_base_translation,
                self.body_to_base_yaw,
            )
        except ValueError as exc:
            self.get_logger().error(f"Rejected invalid FAST-LIO orientation: {exc}")
            return

        current = PlanarPose(
            stamp,
            base_position[0],
            base_position[1],
            _yaw_from_quaternion(base_orientation),
        )
        if self.previous is None:
            self.previous = current
            self._publish(msg, base_position, base_orientation, (0.0, 0.0, 0.0))
            return

        dt = current.stamp_sec - self.previous.stamp_sec
        if dt < self.min_dt or dt > self.max_dt:
            self.previous = current
            self._zero_filter()
            self._publish(msg, base_position, base_orientation, self.filtered_twist)
            return

        dx = current.x - self.previous.x
        dy = current.y - self.previous.y
        translation = math.hypot(dx, dy)
        yaw_step = abs(_normalize_angle(current.yaw - self.previous.yaw))
        raw_vx, raw_vy, raw_w, _ = _derive_planar_twist(self.previous, current)
        speed = math.hypot(raw_vx, raw_vy)
        invalid = (
            translation > self.max_step
            or yaw_step > self.max_yaw_step
            or speed > self.max_linear
            or abs(raw_w) > self.max_angular
        )
        self.previous = current
        if invalid:
            self.rejected_samples += 1
            self._zero_filter()
            self.get_logger().warning(
                "Rejected FAST-LIO odometry jump: "
                f"step={translation:.3f} m yaw={yaw_step:.3f} rad "
                f"speed={speed:.3f} m/s omega={raw_w:.3f} rad/s"
            )
            self._publish(msg, base_position, base_orientation, self.filtered_twist)
            return

        raw = (raw_vx, raw_vy, raw_w)
        self.filtered_twist = tuple(
            self.alpha * raw[index]
            + (1.0 - self.alpha) * self.filtered_twist[index]
            for index in range(3)
        )
        vx, vy, angular = self.filtered_twist
        if math.hypot(vx, vy) < self.linear_deadband:
            vx = 0.0
            vy = 0.0
        if abs(angular) < self.angular_deadband:
            angular = 0.0
        self.filtered_twist = (vx, vy, angular)
        self._publish(msg, base_position, base_orientation, self.filtered_twist)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FastlioPlanarOdom()
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
