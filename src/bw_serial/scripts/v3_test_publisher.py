#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from bw_serial.srv import SetCtrlSrc


ARM_JOINT_NAMES = [
    "left_shoulder_pitch_joint",
    "left_shoulder_yaw_joint",
    "left_shoulder_roll_joint",
    "left_elbow_pitch_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_yaw_joint",
    "right_shoulder_roll_joint",
    "right_elbow_pitch_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

HEAD_JOINT_NAMES = ["head_pitch_joint", "head_yaw_joint"]
GRIPPER_NAMES = ["left_gripper", "right_gripper"]


def build_arm_profile(
    now_sec: float,
    active_joint: int,
    amplitude: float,
    period_sec: float,
    velocity_limit: float,
) -> Tuple[List[float], List[float], List[float]]:
    positions = [0.0] * len(ARM_JOINT_NAMES)
    velocities = [velocity_limit] * len(ARM_JOINT_NAMES)
    efforts = [0.0] * len(ARM_JOINT_NAMES)

    if 0 <= active_joint < len(positions) and amplitude > 0.0 and period_sec > 0.0:
        omega = 2.0 * math.pi / period_sec
        positions[active_joint] = amplitude * math.sin(omega * now_sec)

    return positions, velocities, efforts


class V3TestPublisher(Node):
    def __init__(self) -> None:
        super().__init__("v3_test_publisher")

        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("auto_enable", True)
        self.declare_parameter("ctrl_src_value", 1)
        self.declare_parameter("active_joint", -1)
        self.declare_parameter("joint_amplitude", 0.0)
        self.declare_parameter("joint_period_sec", 4.0)
        self.declare_parameter("joint_velocity_limit", 0.3)
        self.declare_parameter("head_pitch", 0.0)
        self.declare_parameter("head_yaw", 0.0)
        self.declare_parameter("left_gripper", 0.0)
        self.declare_parameter("right_gripper", 0.0)
        self.declare_parameter("vx", 0.0)
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("omega", 0.0)

        publish_rate = float(self.get_parameter("publish_rate").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.ctrl_src_value = int(self.get_parameter("ctrl_src_value").value)
        self.active_joint = int(self.get_parameter("active_joint").value)
        self.joint_amplitude = float(self.get_parameter("joint_amplitude").value)
        self.joint_period_sec = float(self.get_parameter("joint_period_sec").value)
        self.joint_velocity_limit = float(self.get_parameter("joint_velocity_limit").value)
        self.head_pitch = float(self.get_parameter("head_pitch").value)
        self.head_yaw = float(self.get_parameter("head_yaw").value)
        self.left_gripper = float(self.get_parameter("left_gripper").value)
        self.right_gripper = float(self.get_parameter("right_gripper").value)
        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.omega = float(self.get_parameter("omega").value)

        self.arm_pub = self.create_publisher(JointState, "Teleop/joint_angle_solution/smooth", 10)
        self.head_pub = self.create_publisher(JointState, "Teleop/head_pose", 10)
        self.gripper_pub = self.create_publisher(JointState, "Teleop/gripper_pos", 10)
        self.cmd_pub = self.create_publisher(Twist, "Teleop/cmd_vel", 10)

        self.ctrl_src_client = self.create_client(SetCtrlSrc, "set_ctrl_src")
        self.ctrl_src_sent = False

        period = 1.0 / max(publish_rate, 1.0)
        self.create_timer(period, self.publish_cycle)

        self.get_logger().info(
            "V3 test publisher started: rate=%.1fHz active_joint=%d amplitude=%.3f ctrl_src=%d"
            % (publish_rate, self.active_joint, self.joint_amplitude, self.ctrl_src_value)
        )

    def maybe_enable(self) -> None:
        if not self.auto_enable or self.ctrl_src_sent:
            return
        if not self.ctrl_src_client.wait_for_service(timeout_sec=0.0):
            return

        request = SetCtrlSrc.Request()
        request.value = self.ctrl_src_value
        self.ctrl_src_client.call_async(request)
        self.ctrl_src_sent = True
        self.get_logger().info("Sent set_ctrl_src=%d" % self.ctrl_src_value)

    def publish_cycle(self) -> None:
        self.maybe_enable()
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        arm_positions, arm_velocities, arm_efforts = build_arm_profile(
            now_sec=now_sec,
            active_joint=self.active_joint,
            amplitude=self.joint_amplitude,
            period_sec=self.joint_period_sec,
            velocity_limit=self.joint_velocity_limit,
        )

        arm_msg = JointState()
        arm_msg.header.stamp = now.to_msg()
        arm_msg.name = ARM_JOINT_NAMES
        arm_msg.position = arm_positions
        arm_msg.velocity = arm_velocities
        arm_msg.effort = arm_efforts
        self.arm_pub.publish(arm_msg)

        head_msg = JointState()
        head_msg.header.stamp = now.to_msg()
        head_msg.name = HEAD_JOINT_NAMES
        head_msg.position = [self.head_pitch, self.head_yaw]
        self.head_pub.publish(head_msg)

        gripper_msg = JointState()
        gripper_msg.header.stamp = now.to_msg()
        gripper_msg.name = GRIPPER_NAMES
        gripper_msg.position = [self.left_gripper, self.right_gripper]
        self.gripper_pub.publish(gripper_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x = self.vx
        cmd_msg.linear.y = self.vy
        cmd_msg.angular.z = self.omega
        self.cmd_pub.publish(cmd_msg)


def main() -> None:
    rclpy.init()
    node = V3TestPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
