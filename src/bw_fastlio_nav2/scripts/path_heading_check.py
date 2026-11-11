#!/usr/bin/env python3
"""Passive path/heading auditor.

Prints, once per second, what the *controller* thinks the path direction is and
what it is commanding. Nothing is published, nothing is commanded; it only
reads topics and TF. Run it next to a normal navigation session:

    python3 ~/nav/src/bw_fastlio_nav2/scripts/path_heading_check.py

One line per second:

    Path heading: pts=412 frame=map robot=(2.41,1.07) yaw=+1.276
      nearest=63 gap=+0.284 m side=left tangent=+0.489 err=-0.787 rad (-45.1 deg)
      cmd=(vx=+0.270, wz=+0.052) fresh=yes map_to_camera_init=+0.000

Field meanings
  yaw         robot heading in the plan frame, from TF map -> base_footprint
  gap / side  signed lateral offset from the path (positive = left of the path)
  tangent     direction of the path 0.5 m ahead of the nearest point
  err         tangent - yaw: this is exactly the quantity PathAngleCritic scores
  cmd         what MPPI asked the chassis for (/cmd_vel_nav)
  map_to_camera_init  yaw of the map -> camera_init correction

How to read it
  * err large (say > 30 deg) and cmd.wz small (< 0.15) -> the controller is not
    trying to turn: a critic/authority problem, not a chassis problem.
  * err small (< 10 deg) while the car visibly drives at an angle to the drawn
    path -> the frame chain is rotated (body -> base_footprint yaw or the
    map -> camera_init correction); the controller is being lied to about which
    way the nose points.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(quat) -> float:
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def nearest_index(points, x: float, y: float):
    best, best_d2 = 0, float("inf")
    for i, (px, py) in enumerate(points):
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best, best_d2 = i, d2
    return best, math.sqrt(best_d2)


def tangent_ahead(points, index: int, lookahead: float) -> float:
    """Direction of the chord from points[index] to the point lookahead m ahead."""
    travelled = 0.0
    last = points[index]
    for i in range(index + 1, len(points)):
        px, py = points[i]
        travelled += math.hypot(px - last[0], py - last[1])
        last = (px, py)
        if travelled >= lookahead:
            return math.atan2(py - points[index][1], px - points[index][0])
    end = points[-1]
    return math.atan2(end[1] - points[index][1], end[0] - points[index][0])


def lateral_offset(points, index: int, x: float, y: float) -> float:
    """Signed distance from the path at index: positive means left of the path."""
    i0 = max(0, index - 1)
    i1 = min(len(points) - 1, index + 1)
    tx = points[i1][0] - points[i0][0]
    ty = points[i1][1] - points[i0][1]
    norm = math.hypot(tx, ty)
    if norm < 1e-9:
        return 0.0
    # left normal of the tangent
    nx, ny = -ty / norm, tx / norm
    return (x - points[index][0]) * nx + (y - points[index][1]) * ny


class PathHeadingCheck(Node):
    def __init__(self) -> None:
        super().__init__("path_heading_check")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("lookahead_m", 0.5)
        self.declare_parameter("period_sec", 1.0)

        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.lookahead = float(self.get_parameter("lookahead_m").value)

        self.path = []
        self.command = None
        self.command_time = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Path, str(self.get_parameter("plan_topic").value),
                                 self.on_path, 10)
        self.create_subscription(Twist, str(self.get_parameter("cmd_topic").value),
                                 self.on_cmd, 10)
        period = float(self.get_parameter("period_sec").value)
        self.create_timer(period, self.on_timer)

    def on_path(self, msg: Path) -> None:
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_cmd(self, msg: Twist) -> None:
        self.command = msg
        self.command_time = self.get_clock().now()

    def lookup(self, target: str):
        try:
            return self.tf_buffer.lookup_transform(
                self.global_frame, target, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - TF raises several types
            self.get_logger().warn(
                f"tf {self.global_frame}->{target}: {exc}",
                throttle_duration_sec=5.0)
            return None

    def on_timer(self) -> None:
        if len(self.path) < 2:
            return
        tf_robot = self.lookup(self.robot_frame)
        if tf_robot is None:
            return
        t = tf_robot.transform
        x, y = t.translation.x, t.translation.y
        yaw = yaw_of(t.rotation)

        index, gap = nearest_index(self.path, x, y)
        tangent = tangent_ahead(self.path, index, self.lookahead)
        err = wrap(tangent - yaw)
        side = "left" if lateral_offset(self.path, index, x, y) >= 0 else "right"
        signed_gap = lateral_offset(self.path, index, x, y)

        vx = wz = float("nan")
        fresh = "no"
        if self.command is not None:
            vx, wz = self.command.linear.x, self.command.angular.z
            age = (self.get_clock().now() - self.command_time).nanoseconds * 1e-9
            fresh = "yes" if age <= 0.5 else f"stale {age:.1f}s"

        correction = "n/a"
        tf_ci = self.lookup("camera_init")
        if tf_ci is not None:
            correction = f"{math.degrees(yaw_of(tf_ci.transform.rotation)):+.1f}deg"

        self.get_logger().info(
            f"Path heading: pts={len(self.path)} frame={self.global_frame} "
            f"robot=({x:+.2f},{y:+.2f}) yaw={yaw:+.3f} "
            f"nearest={index} gap={signed_gap:+.3f} m side={side} "
            f"tangent={tangent:+.3f} err={err:+.3f} rad ({math.degrees(err):+.1f} deg) "
            f"cmd=(vx={vx:+.3f}, wz={wz:+.3f}) fresh={fresh} "
            f"map_to_camera_init={correction}"
        )


def main() -> None:
    rclpy.init()
    node = PathHeadingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
