#!/usr/bin/env python3
"""Minimal replacement for v2_nav_cmd_bridge.py.

The bridge used to do a lot: V2 speed-table inversion, 0.427 m/s breakaway
boost, lateral suppression, rear-turn latching, goal-rotation shaping, output
clamping, localization/odometry freshness gates and an obstacle escape mode.
All of that is gone. What is left is the only two things the chassis actually
needs:

  1. forward ``/cmd_vel`` to ``Teleop/cmd_vel`` byte for byte, and emit
     zero velocity when no fresh command arrived within ``command_timeout_sec``
     (Nav2 stalls for seconds at a time after a FollowPath abort, and the
     chassis must not keep the last command during that gap);
  2. request the V3 arm/lower-body enable once (``set_ctrl_src``), because the
     chassis powers up in "software off" and nothing else in the stack asks
     for it.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros

from bw_serial.srv import SetCtrlSrc


def corridor_clearance(data, width, height, resolution, origin_x, origin_y,
                       pose_x, pose_y, pose_yaw, direction, max_range,
                       blocked, half_width, lateral_step=0.05,
                       self_min_x=-0.48, self_max_x=0.12, self_half_y=0.33):
    """Distance along ``direction`` that a body-wide corridor stays clear.

    Costs use nav2's OccupancyGrid encoding (``253 -> 99``, ``254 -> 100``).
    Only cells at or above ``blocked`` count, so the 253 inscribed band - which
    surrounds every wall of a 1 m corridor - is not treated as an obstacle.

    Cells inside the robot's own footprint box are skipped: the cloud is only
    self-filtered over ``x in [-0.45, 0] y in [-0.25, 0.25]`` of the LiDAR
    frame, so the chassis' own side walls survive as lethal cells about 0.31 m
    away.  An outside obstacle cannot be inside the hull, so this loses
    nothing, and without it the look-ahead would report 0.31 m in every
    direction and hold the base still forever.

    Pure function so it can be tested without ROS.
    """
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    cos_dir = math.cos(pose_yaw + direction)
    sin_dir = math.sin(pose_yaw + direction)
    half_steps = max(1, int(round(half_width / lateral_step)))
    offsets = [index * lateral_step
               for index in range(-half_steps, half_steps + 1)]

    steps = max(1, int(round(max_range / resolution)))
    for step in range(1, steps + 1):
        distance = step * resolution
        centre_x = pose_x + distance * cos_dir
        centre_y = pose_y + distance * sin_dir
        for offset in offsets:
            x = centre_x + offset * -sin_dir
            y = centre_y + offset * cos_dir
            delta_x = x - pose_x
            delta_y = y - pose_y
            along = delta_x * cos_yaw + delta_y * sin_yaw
            across = -delta_x * sin_yaw + delta_y * cos_yaw
            if self_min_x <= along <= self_max_x and abs(across) <= self_half_y:
                continue
            column = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            if column < 0 or row < 0 or column >= width or row >= height:
                continue
            if data[row * width + column] >= blocked:
                return distance
    return max_range


class NavCmdRelay(Node):
    def __init__(self) -> None:
        super().__init__("nav_cmd_relay")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "Teleop/cmd_vel")
        # escape_node publishes here while it drives the robot out of a corner.
        # It takes priority because it is the only thing that can move the base
        # while the look-ahead bumper pins the relayed command at zero, i.e.
        # exactly while the robot is stuck against whatever it hit.
        self.declare_parameter("escape_topic", "/escape_cmd_vel")
        self.declare_parameter("escape_timeout_sec", 0.30)
        self.declare_parameter("command_timeout_sec", 0.25)
        # The chassis takes linear.x as a normalised throttle (1.0 = full
        # speed), not as m/s. Measured from the 2026-10-31 bag: 0.30 in ->
        # 0.091 m/s out, i.e. a factor 3.2. MPPI models the command as m/s, so
        # without this factor every predicted arc is 3.2x wider than the one
        # the base actually drives and the controller overshoots every corner.
        self.declare_parameter("linear_scale", 1.0)
        # Sideways speed needs the same m/s -> normalised conversion as forward
        # (the base took ~0.32 of the requested value on x before scaling).
        self.declare_parameter("lateral_scale", 3.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("power_service", "set_ctrl_src")
        self.declare_parameter("power_value", 1)
        self.declare_parameter("power_retry_period_sec", 2.0)

        # ------------------------------------------------------- look-ahead bumper
        # Nothing else in the stack can stop the base *before* contact: RPP's
        # use_collision_detection has to stay off (it tests the padded footprint
        # against lethal+inscribed, and the body already sits inside that band in
        # a 1 m corridor), and collision_monitor only reacts once a point is
        # inside hull+5 cm.  The relay sees the final command, so it can sweep
        # the direction it points at and brake in time.
        self.declare_parameter("bumper_enabled", True)
        self.declare_parameter("bumper_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("bumper_costmap_timeout_sec", 0.60)
        # Only lethal cells block: 99 is the inscribed band, which is everywhere
        # in a narrow corridor.
        self.declare_parameter("bumper_blocked_cost", 100)
        self.declare_parameter("bumper_stop_m", 0.28)
        self.declare_parameter("bumper_slow_m", 0.90)
        self.declare_parameter("bumper_half_width_m", 0.32)
        self.declare_parameter("bumper_stop_log_period_sec", 1.0)
        self.declare_parameter("bumper_base_frame", "base_footprint")
        # The self-filter that runs before the cloud ever reaches a costmap is
        # smaller than the chassis (see corridor_clearance), so the sweep has to
        # ignore the hull's own cells.
        self.declare_parameter("bumper_self_min_x", -0.48)
        self.declare_parameter("bumper_self_max_x", 0.12)
        self.declare_parameter("bumper_self_half_y", 0.33)

        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        if self.linear_scale <= 0.0:
            raise ValueError("linear_scale must be positive")
        self.lateral_scale = float(self.get_parameter("lateral_scale").value)
        if self.lateral_scale <= 0.0:
            raise ValueError("lateral_scale must be positive")
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.power_value = int(self.get_parameter("power_value").value)
        self.power_retry_period = float(
            self.get_parameter("power_retry_period_sec").value
        )

        self.bumper_enabled = bool(self.get_parameter("bumper_enabled").value)
        self.bumper_costmap_timeout = float(
            self.get_parameter("bumper_costmap_timeout_sec").value
        )
        self.bumper_blocked = int(
            self.get_parameter("bumper_blocked_cost").value
        )
        self.bumper_stop = float(self.get_parameter("bumper_stop_m").value)
        self.bumper_slow = float(self.get_parameter("bumper_slow_m").value)
        if self.bumper_slow <= self.bumper_stop:
            raise ValueError("bumper_slow_m must be greater than bumper_stop_m")
        self.bumper_half_width = float(
            self.get_parameter("bumper_half_width_m").value
        )
        self.bumper_base_frame = str(
            self.get_parameter("bumper_base_frame").value
        )
        self.bumper_self_min_x = float(
            self.get_parameter("bumper_self_min_x").value
        )
        self.bumper_self_max_x = float(
            self.get_parameter("bumper_self_max_x").value
        )
        self.bumper_self_half_y = float(
            self.get_parameter("bumper_self_half_y").value
        )
        self.bumper_log_period = float(
            self.get_parameter("bumper_stop_log_period_sec").value
        )
        self.bumper_costmap = None
        self.bumper_costmap_stamp = None
        self.bumper_last_log = 0.0
        self.bumper_last_scale = 1.0

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        escape_topic = str(self.get_parameter("escape_topic").value)
        self.escape_timeout = float(
            self.get_parameter("escape_timeout_sec").value
        )

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, input_topic, self.on_command, 10
        )
        self.escape_subscription = self.create_subscription(
            Twist, escape_topic, self.on_escape, 10
        )

        self.tf_buffer = None
        if self.bumper_enabled:
            self.bumper_subscription = self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("bumper_costmap_topic").value),
                self.on_costmap,
                5,
            )
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_command = Twist()
        self.last_command_monotonic = None
        self.last_escape = None
        self.last_escape_monotonic = None
        self.stale_logged = False

        self.armed = not self.enable_motion
        self.power_pending = False
        self.last_power_attempt = 0.0
        self.power_client = None
        if self.enable_motion:
            self.power_client = self.create_client(
                SetCtrlSrc, str(self.get_parameter("power_service").value)
            )

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            "nav_cmd_relay: %s -> %s, linear.x scaled by %.3f and linear.y by "
            "%.3f, both clamped to [-1, 1], stale after %.2f s, "
            "enable_motion=%s"
            % (input_topic, output_topic, self.linear_scale,
               self.lateral_scale, self.command_timeout, self.enable_motion)
        )

    # ------------------------------------------------------------------ input
    def on_command(self, msg: Twist) -> None:
        self.last_command = msg
        self.last_command_monotonic = time.monotonic()
        if self.stale_logged:
            self.stale_logged = False
            self.get_logger().info("command stream resumed")

    def on_escape(self, msg: Twist) -> None:
        # A zero escape command means "I am not driving the base", so it must
        # release control rather than latch it: the relay treats any message on
        # this topic as authoritative for escape_timeout_sec, and an idle node
        # that keeps publishing zeros would out-rank the relay input forever.
        if (abs(msg.linear.x) < 1e-6 and abs(msg.linear.y) < 1e-6
                and abs(msg.angular.z) < 1e-6):
            self.last_escape = None
            return
        self.last_escape = msg
        self.last_escape_monotonic = time.monotonic()

    # ------------------------------------------------------------------ power
    def request_power(self, now: float) -> None:
        if self.armed or self.power_pending:
            return
        if now - self.last_power_attempt < self.power_retry_period:
            return
        if self.power_client is None or not self.power_client.service_is_ready():
            return

        self.last_power_attempt = now
        self.power_pending = True
        request = SetCtrlSrc.Request()
        request.value = self.power_value
        future = self.power_client.call_async(request)
        future.add_done_callback(self.on_power_done)

    def on_power_done(self, future) -> None:
        self.power_pending = False
        try:
            response = future.result()
        except Exception as exc:  # rclpy service exceptions vary by RMW
            self.get_logger().error(f"power request failed: {exc}")
            return
        if response.success:
            self.armed = True
            self.get_logger().warning(
                f"chassis enable accepted (value={self.power_value}): "
                f"{response.message}"
            )
        else:
            self.get_logger().error(
                f"chassis enable rejected: {response.message}"
            )

    # ------------------------------------------------------------------ bumper
    def on_costmap(self, msg: OccupancyGrid) -> None:
        self.bumper_costmap = msg
        self.bumper_costmap_stamp = time.monotonic()

    def robot_pose_in(self, frame):
        """(x, y, yaw) of the base in ``frame``, or None when the TF is missing."""
        if self.tf_buffer is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                frame, self.bumper_base_frame, Time()
            )
        except Exception:  # tf2 raises several unrelated types here
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return translation.x, translation.y, yaw

    def apply_bumper(self, command: Twist) -> Twist:
        """Scale the command down so the base stops before it touches something.

        Works on the direction the command actually points at, so it protects
        the front while driving forwards, the back while backing up and the
        flank while strafing out of a corner.  Nav2 keeps asking for the same
        velocity, so the base simply stands still with the command pinned at
        zero until the path or the obstacle changes - which is the behaviour
        wanted in front of a person in a corridor too narrow to pass.
        """
        if not self.bumper_enabled or self.bumper_costmap is None:
            return command
        if time.monotonic() - self.bumper_costmap_stamp > self.bumper_costmap_timeout:
            return command
        speed = math.hypot(command.linear.x, command.linear.y)
        if speed < 0.02:
            return command
        grid = self.bumper_costmap
        pose = self.robot_pose_in(grid.header.frame_id)
        if pose is None:
            return command
        info = grid.info
        direction = math.atan2(command.linear.y, command.linear.x)
        distance = corridor_clearance(
            grid.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            pose[0], pose[1], pose[2], direction, self.bumper_slow,
            self.bumper_blocked, self.bumper_half_width,
            self_min_x=self.bumper_self_min_x,
            self_max_x=self.bumper_self_max_x,
            self_half_y=self.bumper_self_half_y,
        )
        if distance <= self.bumper_stop:
            scale = 0.0
        elif distance < self.bumper_slow:
            scale = (distance - self.bumper_stop) / (
                self.bumper_slow - self.bumper_stop
            )
        else:
            scale = 1.0

        now = time.monotonic()
        if scale < 0.999 and now - self.bumper_last_log > self.bumper_log_period:
            self.bumper_last_log = now
            self.get_logger().warning(
                "bumper: obstacle at %.2f m towards %+.0f deg, scaling %.2f m/s "
                "to %.0f%%" % (distance, math.degrees(direction), speed,
                               scale * 100.0)
            )
        self.bumper_last_scale = scale
        if scale >= 0.999:
            return command
        out = Twist()
        out.linear.x = command.linear.x * scale
        out.linear.y = command.linear.y * scale
        out.angular.z = command.angular.z
        return out

    # ------------------------------------------------------------------ output
    def output_command(self, command: Twist) -> Twist:
        """Copy the command and convert m/s into the chassis' normalised unit."""
        out = Twist()
        out.linear.x = max(-1.0, min(1.0, command.linear.x * self.linear_scale))
        out.linear.y = max(
            -1.0, min(1.0, command.linear.y * self.lateral_scale))
        out.linear.z = command.linear.z
        out.angular.x = command.angular.x
        out.angular.y = command.angular.y
        out.angular.z = command.angular.z
        return out

    # ------------------------------------------------------------------ timer
    def on_timer(self) -> None:
        now = time.monotonic()
        self.request_power(now)

        # An escape command outranks the relayed velocity: during a stuck
        # condition the bumper pins that command at zero by design and only
        # this path can move the base away from the obstacle.
        if (
            self.last_escape is not None
            and now - self.last_escape_monotonic <= self.escape_timeout
        ):
            self.publisher.publish(self.output_command(self.last_escape))
            return

        fresh = (
            self.last_command_monotonic is not None
            and now - self.last_command_monotonic <= self.command_timeout
        )
        if fresh:
            self.publisher.publish(
                self.output_command(self.apply_bumper(self.last_command)))
            return

        if not self.stale_logged:
            self.stale_logged = True
            self.get_logger().warning(
                "no fresh command for %.2f s, publishing zero velocity"
                % self.command_timeout
            )
        self.publisher.publish(Twist())


def main() -> None:
    rclpy.init()
    node = NavCmdRelay()
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
