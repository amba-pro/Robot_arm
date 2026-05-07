#!/usr/bin/env python3
import json
import os
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class AngleCachePublisher(Node):
    def __init__(self) -> None:
        super().__init__("angle_cache_publisher")

        self.declare_parameter("cache_file", "angles_cache.json")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter(
            "joint_names",
            ["joint_a0", "joint_a1", "joint_a2", "joint_a3", "joint_a4"],
        )

        self.cache_file = self.get_parameter("cache_file").get_parameter_value().string_value
        self.joint_names = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )

        if publish_rate_hz <= 0.0:
            publish_rate_hz = 10.0

        self.angles_pub = self.create_publisher(Float32MultiArray, "/arm4/angles", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(period, self._publish_from_cache)

        self.get_logger().info(f"Reading angle cache from: {self.cache_file}")

    def _read_angles(self) -> List[float]:
        default = [0.0] * 5
        if not os.path.exists(self.cache_file):
            return default

        try:
            with open(self.cache_file, "r", encoding="utf-8") as file:
                payload = json.load(file)
            raw = payload.get("angles", {})
            return [
                float(raw.get("A0", 0.0)),
                float(raw.get("A1", 0.0)),
                float(raw.get("A2", 0.0)),
                float(raw.get("A3", 0.0)),
                float(raw.get("A4", 0.0)),
            ]
        except Exception as exc:
            self.get_logger().warning(f"Failed to parse cache file: {exc}")
            return default

    def _publish_from_cache(self) -> None:
        angles = self._read_angles()

        msg = Float32MultiArray()
        msg.data = angles
        self.angles_pub.publish(msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = list(self.joint_names)
        # Keep units identical to source script output (degrees)
        joint_msg.position = [float(value) for value in angles]
        self.joint_pub.publish(joint_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AngleCachePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
