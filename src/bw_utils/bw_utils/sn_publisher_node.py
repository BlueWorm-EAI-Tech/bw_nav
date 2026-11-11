#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from bw_interface.msg import RobotIdentity
from bw_utils import get_local_ip, get_robot_sn


class SNPublisherNode(Node):
    """节点入口：周期发布机器人 SN 与 IP 到全局 /sn 话题。"""

    def __init__(self) -> None:
        super().__init__("sn_publisher_node")

        # 参数说明：
        # - topic_name: 发布话题，默认全局绝对话题 /sn（不进入机器人 SN namespace）
        # - publish_frequency: 发布频率（Hz）
        # - ip_override: 非空时覆盖自动探测的 IP
        self.declare_parameter("topic_name", "/sn")
        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("ip_override", "")

        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        publish_frequency = self.get_parameter("publish_frequency").get_parameter_value().double_value
        ip_override = self.get_parameter("ip_override").get_parameter_value().string_value

        if publish_frequency <= 0.0:
            self.get_logger().warn("publish_frequency <= 0，已回退到 1.0 Hz")
            publish_frequency = 1.0

        self.robot_sn = get_robot_sn()
        self.robot_ip = ip_override if ip_override else get_local_ip()

        # 发布 Topic: /sn, 消息类型: bw_interface/msg/RobotIdentity, QoS: depth=10
        self.publisher = self.create_publisher(RobotIdentity, self.topic_name, 10)
        self.timer = self.create_timer(1.0 / publish_frequency, self.publish_identity)

        self.publish_identity()
        self.get_logger().info(
            f"SN 发布节点启动成功: topic={self.topic_name}, sn={self.robot_sn}, ip={self.robot_ip}"
        )

    def publish_identity(self) -> None:
        """周期发布机器人标识信息。"""
        msg = RobotIdentity()
        msg.ip = self.robot_ip
        msg.sn = self.robot_sn
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SNPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
