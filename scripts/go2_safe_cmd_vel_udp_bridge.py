#!/usr/bin/env python3
"""Bridge /go2_safe/cmd_vel to a local UDP Unitree SDK worker.

Run on the Go2 dock as a ROS 2 process. This file intentionally does not import
unitree_sdk2py. It only serializes already-gated velocity commands.
"""

import json
import math
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    return max(-limit, min(limit, value))


class Go2SafeCmdVelUdpBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_safe_cmd_vel_udp_bridge")

        self.declare_parameter("input_topic", "/go2_safe/cmd_vel")
        self.declare_parameter("status_topic", "/go2_safe/udp_bridge_status")
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 15001)
        self.declare_parameter("max_linear_x", 0.10)
        self.declare_parameter("max_angular_z", 0.30)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.udp_host = str(self.get_parameter("udp_host").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (self.udp_host, self.udp_port)
        self.seq = 0

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.on_cmd, 10)
        self.get_logger().warn(
            "Go2 UDP bridge started: "
            f"{self.input_topic} -> udp://{self.udp_host}:{self.udp_port}. "
            "This process does not import Unitree SDK."
        )

    def on_cmd(self, msg: Twist) -> None:
        self.seq += 1
        vx = clamp(finite_or_zero(msg.linear.x), self.max_linear_x)
        vyaw = clamp(finite_or_zero(msg.angular.z), self.max_angular_z)
        payload = {
            "seq": self.seq,
            "stamp": time.time(),
            "vx": vx,
            "vy": 0.0,
            "vyaw": vyaw,
        }
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.sock.sendto(data, self.addr)
        self.status_pub.publish(
            String(
                data=(
                    f"seq={self.seq} udp={self.udp_host}:{self.udp_port} "
                    f"vx={vx:.3f} vyaw={vyaw:.3f}"
                )
            )
        )


def main() -> None:
    rclpy.init()
    node = Go2SafeCmdVelUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
