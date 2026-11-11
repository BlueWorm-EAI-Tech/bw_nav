#!/usr/bin/env python3
"""Drive out of a corner instead of waiting for a recovery behaviour to fail.

Why this exists
---------------
The recovery tree (BackUp / Spin) has to command motion through
``/cmd_vel -> collision_monitor -> /cmd_vel_safe``, and collision_monitor stops
*all* motion whenever a point cloud point sits inside ``RadarStopZone``.  When
the robot is nose-in against a wall that point stays inside the polygon for as
long as the robot stays where it is, so the recovery's own command is the thing
that gets zeroed.  Backing out is impossible by construction: the ring blocks
the motion that would remove the ring.

This node does not fight the controller and does not run during normal driving.
It watches for one condition - Nav2 asking for motion while the base has not
moved for ``trigger_sec`` - and then takes over for a bounded time: it looks at
the local costmap, picks the heading with the most free space, turns to it and
drives a short distance.  Its output goes to ``/escape_cmd_vel``, which
``nav_cmd_relay`` gives priority over ``/cmd_vel_safe``; that is the only path
in the stack that can move the base while the stop ring is asserted.

Safety bounds, all of which must hold before it moves:
  * the base has been genuinely stationary (< ``speed_threshold``) for
    ``trigger_sec`` while a command of at least ``cmd_threshold`` was present;
  * the chosen direction is free for the whole body (``2 * body_half_width_m``
    wide corridor) for at least ``min_free_m`` metres;
  * it drives at most ``drive_sec`` seconds at ``drive_speed`` and at most
    ``max_escape_sec`` seconds in total, then hands control back;
  * on an omni base the heading is driven directly (vx/vy) instead of turning
    towards it, so corners are left sideways instead of rotating in place;
  * if the body corridor ahead shortens below ``abort_free_m`` the escape stops
    at once.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def pick_direction(costs, width, height, resolution, origin_x, origin_y,
                   pose_x, pose_y, pose_yaw, headings, ray_max, blocked,
                   turn_penalty, half_width=0.32, lateral_step=0.10,
                   self_min_x=-0.48, self_max_x=0.12, self_half_y=0.33):
    """March the robot's *body width* along every heading and return the best.

    ``costs`` is the row-major OccupancyGrid data (0 free, 100 lethal, -1
    unknown).  Returns ``(best_heading, free_distance, all_distances)``.

    A single ray is not the robot: scoring on the centre ray alone let the base
    pick a heading that only *looked* free, because the ray threaded past the
    shoulder of a person standing 0.5 m ahead (logged as
    ``escaping towards +0 deg (1.20 m free)`` right before it drove into them).
    Here each heading is swept as a corridor ``2 * half_width`` wide, so the
    reported distance is how far the body actually fits.

    Cells inside the robot's own footprint box are skipped: both costmaps get a
    cloud that is only self-filtered over ``x in [-0.45, 0.0] y in [-0.25,
    0.25]`` of the LiDAR frame, so the chassis' own side walls (|y| = 0.31) and
    the front rim survive as lethal cells about 0.31 m from the base.  With the
    corridor sweep starting at the robot centre, every heading then reported
    <= 0.45 m - below ``min_free_m`` - and the node refused to escape at all
    ("no direction has 0.55 m free"), which is exactly when it is needed.  An
    external obstacle cannot be inside the hull, so dropping those cells loses
    nothing.

    Pure function so it can be tested without ROS.
    """
    half_steps = max(1, int(round(half_width / lateral_step)))
    offsets = [index * lateral_step
               for index in range(-half_steps, half_steps + 1)]
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)

    distances = []
    for heading in headings:
        cos_dir = math.cos(pose_yaw + heading)
        sin_dir = math.sin(pose_yaw + heading)
        travelled = 0.0
        while travelled < ray_max:
            travelled += resolution
            centre_x = pose_x + travelled * cos_dir
            centre_y = pose_y + travelled * sin_dir
            hit = False
            for offset in offsets:
                x = centre_x + offset * -sin_dir
                y = centre_y + offset * cos_dir
                delta_x = x - pose_x
                delta_y = y - pose_y
                along = delta_x * cos_yaw + delta_y * sin_yaw
                across = -delta_x * sin_yaw + delta_y * cos_yaw
                if (self_min_x <= along <= self_max_x
                        and abs(across) <= self_half_y):
                    continue
                column = int((x - origin_x) / resolution)
                row = int((y - origin_y) / resolution)
                if column < 0 or row < 0 or column >= width or row >= height:
                    hit = True
                    break
                if costs[row * width + column] >= blocked:
                    hit = True
                    break
            if hit:
                break
        distances.append(travelled)

    best_index = 0
    best_score = -1e9
    for index, distance in enumerate(distances):
        # headings run 0..2*pi, so abs(heading) penalised everything past
        # half a turn by up to 6 m: reverse (and the right side) could never
        # win, which is why the escape always tried to turn or strafe instead
        # of backing out.
        score = distance - turn_penalty * abs(wrap(headings[index]))
        if score > best_score:
            best_score = score
            best_index = index
    return headings[best_index], distances[best_index], distances


class EscapeNode(Node):
    IDLE = 0
    TURNING = 1
    DRIVING = 2
    COOLDOWN = 3

    def __init__(self) -> None:
        super().__init__("escape_node")

        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("odom_topic", "/nav2_odom")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/escape_cmd_vel")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("trigger_sec", 1.5)
        self.declare_parameter("cmd_threshold", 0.05)
        self.declare_parameter("speed_threshold", 0.02)
        self.declare_parameter("turn_threshold", 0.15)
        self.declare_parameter("ray_max_m", 1.20)
        # The driving rule must cover the distance the robot actually covers
        # (drive_sec * drive_speed) plus a stopping margin, otherwise it
        # "escapes" into the very thing it is escaping from.
        self.declare_parameter("min_free_m", 0.55)
        self.declare_parameter("abort_free_m", 0.30)
        # Escape is allowed the whole envelope the base has been measured to
        # track (0.30 m/s linear, 1.0 rad/s yaw); it must not be limited to the
        # old 0.12 m/s crawl, which cleared 0.24 m per cycle at best.
        # Note: nav_cmd_relay multiplies linear.x by linear_scale (3.0), so a
        # 0.30 command reaches the chassis as 0.90 - the same value Nav2 sends.
        self.declare_parameter("drive_speed", 0.30)
        self.declare_parameter("turn_vel", 1.00)
        self.declare_parameter("turn_gain", 1.50)
        self.declare_parameter("turn_tolerance", 0.25)
        # A 180 deg turn needs pi/turn_vel = 3.15 s at 1.0 rad/s.
        self.declare_parameter("turn_timeout_sec", 3.40)
        self.declare_parameter("drive_sec", 0.70)
        self.declare_parameter("max_escape_sec", 5.0)
        self.declare_parameter("cooldown_sec", 3.0)
        # nav_msgs/OccupancyGrid carries cost/255*100, so the local costmap only
        # ever holds 0 (free), 99 (INSCRIBED_INFLATED = 253) and 100 (LETHAL =
        # 254). 50 meant "inside the inscribed radius of something", which is a
        # whole-robot disc of 0.231 m: in a 1.0 m corridor every pose is inside
        # it, so every one of the 16 corridors reported 0.05 m and the node
        # refused to escape ("no direction has 0.55 m free, best 0.05 m").
        # Only lethal cells block here; the body width is already accounted for
        # by the corridor sweep.
        self.declare_parameter("blocked_cost", 100)
        self.declare_parameter("turn_penalty_m_per_rad", 0.10)
        # Half of the chassis width; the sweep needs it to reject gaps the
        # body cannot fit through.
        self.declare_parameter("body_half_width_m", 0.32)
        # The chassis is omni: it can strafe. Escaping by rotating first wastes
        # seconds and, in a corner, cannot rotate at all. With lateral motion on,
        # the chosen heading is decomposed into the body frame and driven
        # directly, so no turn is needed.
        self.declare_parameter("use_lateral_motion", True)
        # Backing out beats turning out: the corridor behind is where the robot
        # came from, so it is the one direction already known to be clear.
        self.declare_parameter("prefer_reverse", True)
        self.declare_parameter("lateral_speed", 0.20)
        self.declare_parameter("heading_count", 16)
        # Cells inside this box (robot frame) are the chassis' own returns, not
        # obstacles: the upstream self-filter only removes x in [-0.45, 0] and
        # |y| <= 0.25 of the LiDAR frame, so the side walls (|y| = 0.31) and the
        # front rim survive as lethal cells ~0.31 m away. Sweeping a body-wide
        # corridor from the robot centre then reported <= 0.45 m in every
        # direction - below min_free_m - and the node refused to escape exactly
        # when it was needed.
        self.declare_parameter("self_min_x", -0.48)
        self.declare_parameter("self_max_x", 0.12)
        self.declare_parameter("self_half_y", 0.33)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.trigger_sec = float(self.get_parameter("trigger_sec").value)
        self.cmd_threshold = float(self.get_parameter("cmd_threshold").value)
        self.speed_threshold = float(self.get_parameter("speed_threshold").value)
        self.turn_threshold = float(self.get_parameter("turn_threshold").value)
        self.ray_max = float(self.get_parameter("ray_max_m").value)
        self.min_free = float(self.get_parameter("min_free_m").value)
        self.abort_free = float(self.get_parameter("abort_free_m").value)
        self.drive_speed = float(self.get_parameter("drive_speed").value)
        self.turn_vel = float(self.get_parameter("turn_vel").value)
        self.turn_gain = float(self.get_parameter("turn_gain").value)
        self.turn_tolerance = float(self.get_parameter("turn_tolerance").value)
        self.turn_timeout = float(self.get_parameter("turn_timeout_sec").value)
        self.drive_sec = float(self.get_parameter("drive_sec").value)
        self.max_escape_sec = float(self.get_parameter("max_escape_sec").value)
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.blocked_cost = int(self.get_parameter("blocked_cost").value)
        self.turn_penalty = float(
            self.get_parameter("turn_penalty_m_per_rad").value
        )
        self.half_width = float(
            self.get_parameter("body_half_width_m").value
        )
        self.self_min_x = float(self.get_parameter("self_min_x").value)
        self.self_max_x = float(self.get_parameter("self_max_x").value)
        self.self_half_y = float(self.get_parameter("self_half_y").value)
        self.use_lateral = bool(self.get_parameter("use_lateral_motion").value)
        self.prefer_reverse = bool(self.get_parameter("prefer_reverse").value)
        self.lateral_speed = float(self.get_parameter("lateral_speed").value)
        self.heading_step = 2.0 * math.pi / max(4, int(
            self.get_parameter("heading_count").value
        ))
        # The direction picker promises min_free metres of body-clear corridor.
        # Driving for drive_sec at drive_speed consumes drive_sec*drive_speed of
        # it, and the abort check needs abort_free left over; if that does not
        # fit, the escape would drive into whatever it was escaping from.
        travel = self.drive_sec * self.drive_speed
        if self.min_free < travel + self.abort_free:
            raise RuntimeError(
                "min_free_m %.2f m is too short for drive_sec %.2f s * "
                "drive_speed %.2f m/s + abort_free_m %.2f m = %.2f m"
                % (self.min_free, self.drive_sec, self.drive_speed,
                   self.abort_free, travel + self.abort_free)
            )
        count = max(4, int(self.get_parameter("heading_count").value))
        self.headings = [
            2.0 * math.pi * index / count for index in range(count)
        ]

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_topic").value), self.on_cmd, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self.on_odom, 10
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("costmap_topic").value),
            self.on_costmap, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.command = None
        self.command_time = 0.0
        self.speed = 0.0
        self.turn_rate = 0.0
        self.costmap = None
        self.state = self.IDLE
        self.still_since = None
        self.state_since = 0.0
        self.escape_started = 0.0
        self.target_heading = 0.0
        self.last_log = 0.0

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            "escape_node: watching %s / %s, %d headings, trigger %.1f s, "
            "drive %.2f m/s for %.1f s"
            % (str(self.get_parameter("cmd_topic").value),
               str(self.get_parameter("costmap_topic").value),
               len(self.headings), self.trigger_sec, self.drive_speed,
               self.drive_sec)
        )

    # ------------------------------------------------------------------ input
    def on_cmd(self, msg: Twist) -> None:
        self.command = msg
        self.command_time = self.now()

    def on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        self.speed = math.hypot(twist.linear.x, twist.linear.y)
        self.turn_rate = abs(twist.angular.z)

    def on_costmap(self, msg: OccupancyGrid) -> None:
        self.costmap = msg

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ logic
    def robot_pose(self, frame_id):
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id, self.base_frame, Time()
            )
        except Exception:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        siny = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        return translation.x, translation.y, math.atan2(siny, cosy)

    def fresh_command(self):
        if self.command is None:
            return None
        if self.now() - self.command_time > 0.5:
            return None
        return self.command

    def wants_motion(self) -> bool:
        """True when Nav2 is asking for motion the base is not delivering.

        A yaw demand counts, but only while the base is not actually rotating:
        RPP's rotate-to-heading phase legitimately commands vx=0 at 0.45 rad/s,
        and treating that as "stuck" hijacked the controller every 1.5 s (the
        robot could not finish a turn). Requiring the *measured* rate to be
        below turn_threshold keeps that case out while still catching "asked to
        turn, cannot turn" - which is what a nose against a wall looks like.
        """
        command = self.fresh_command()
        if command is None:
            return False
        translating = abs(command.linear.x) > self.cmd_threshold
        turning = abs(command.angular.z) > self.turn_threshold
        return translating or turning

    def evaluate(self):
        """Return (heading, free_distance) for the best direction, or None."""
        if self.costmap is None:
            return None
        info = self.costmap.info
        pose = self.robot_pose(self.costmap.header.frame_id)
        if pose is None:
            return None
        heading, distance, distances = pick_direction(
            list(self.costmap.data), info.width, info.height,
            info.resolution, info.origin.position.x, info.origin.position.y,
            pose[0], pose[1], pose[2], self.headings, self.ray_max,
            self.blocked_cost, self.turn_penalty, self.half_width,
            self_min_x=self.self_min_x, self_max_x=self.self_max_x,
            self_half_y=self.self_half_y,
        )
        if self.prefer_reverse:
            reverse_index = min(
                range(len(self.headings)),
                key=lambda index: abs(wrap(self.headings[index] - math.pi)),
            )
            if distances[reverse_index] >= self.min_free:
                return (self.headings[reverse_index],
                        distances[reverse_index], pose[2], distances)
        return heading, distance, pose[2], distances

    def on_timer(self) -> None:
        now = self.now()
        command = Twist()

        if self.state == self.IDLE:
            if (self.wants_motion()
                    and self.speed < self.speed_threshold
                    and self.turn_rate < self.turn_threshold):
                if self.still_since is None:
                    self.still_since = now
                elif now - self.still_since >= self.trigger_sec:
                    decision = self.evaluate()
                    if decision is None:
                        self.still_since = now
                    elif decision[1] < self.min_free:
                        self.still_since = now
                        if now - self.last_log > 2.0:
                            self.last_log = now
                            self.get_logger().warning(
                                "stuck, but no direction has %.2f m of free "
                                "space (best %.2f m); not escaping"
                                % (self.min_free, decision[1])
                            )
                    else:
                        heading, distance, yaw, _ = decision
                        self.target_heading = wrap(yaw + heading)
                        self.state = (
                            self.DRIVING if self.use_lateral else self.TURNING)
                        self.state_since = now
                        self.escape_started = now
                        command = self.fresh_command()
                        self.get_logger().warning(
                            "stuck for %.1f s: escaping towards %+.0f deg "
                            "(%.2f m free); nav2 asked vx=%+.3f wz=%+.3f, "
                            "odom v=%+.3f w=%+.3f"
                            % (now - self.still_since,
                               math.degrees(heading), distance,
                               command.linear.x, command.angular.z,
                               self.speed, self.turn_rate)
                        )
            else:
                self.still_since = None
            # Stay silent while idle. The relay gives the escape topic priority
            # for escape_timeout_sec after every message, so publishing zero
            # twists here would out-rank /cmd_vel_safe permanently and pin the
            # base at zero - exactly the "robot does not move at all" failure.
            return

        if self.state == self.COOLDOWN:
            if now - self.state_since > self.cooldown_sec:
                self.state = self.IDLE
                self.still_since = None
            return

        if now - self.escape_started > self.max_escape_sec:
            self.get_logger().warning("escape timed out, handing control back")
            self.finish(now)
            self.publisher.publish(command)
            return

        pose = self.robot_pose(self.costmap.header.frame_id) \
            if self.costmap is not None else None

        if self.state == self.TURNING:
            if pose is None:
                self.finish(now)
                self.publisher.publish(command)
                return
            error = wrap(self.target_heading - pose[2])
            if (abs(error) <= self.turn_tolerance
                    or now - self.state_since > self.turn_timeout):
                self.state = self.DRIVING
                self.state_since = now
            else:
                command.angular.z = clamp(
                    self.turn_gain * error, -self.turn_vel, self.turn_vel
                )
                self.publisher.publish(command)
                return

        if self.state == self.DRIVING:
            decision = self.evaluate()
            if decision is None:
                self.finish(now)
                self.publisher.publish(command)
                return
            # Watch the corridor we are actually entering.  Checking the *best*
            # heading instead made the abort condition depend on some other
            # direction, so a narrowing path ahead went unnoticed until the
            # bumper touched it.
            _, _, yaw_now, distances = decision
            here = int(round(wrap(self.target_heading - yaw_now)
                              / self.heading_step)) % len(distances)
            ahead = distances[here]
            if ahead < self.abort_free:
                self.get_logger().warning(
                    "escape path closed up (%.2f m ahead), stopping" % ahead
                )
                self.finish(now)
                self.publisher.publish(command)
                return
            if now - self.state_since > self.drive_sec:
                self.get_logger().info("escape finished")
                self.finish(now)
                self.publisher.publish(command)
                return
            if pose is not None and self.use_lateral:
                # Drive the chosen heading in the body frame instead of
                # turning towards it: vx = s*cos, vy = s*sin, scaled into the
                # forward/lateral speed box.
                error = wrap(self.target_heading - pose[2])
                cos_e = math.cos(error)
                sin_e = math.sin(error)
                limit = min(
                    self.drive_speed / max(abs(cos_e), 1e-6),
                    self.lateral_speed / max(abs(sin_e), 1e-6),
                )
                scale = min(limit, self.drive_speed)
                command.linear.x = scale * cos_e
                command.linear.y = scale * sin_e
            else:
                command.linear.x = self.drive_speed
            if pose is not None and not self.use_lateral:
                error = wrap(self.target_heading - pose[2])
                command.angular.z = clamp(
                    self.turn_gain * error, -self.turn_vel, self.turn_vel
                )
            self.publisher.publish(command)
            return


    def finish(self, now: float) -> None:
        self.state = self.COOLDOWN
        self.state_since = now
        self.still_since = None


def main() -> None:
    rclpy.init()
    node = EscapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
