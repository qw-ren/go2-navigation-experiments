#!/usr/bin/python3
"""Project Fast-LIO 6-DoF odometry to a planar Go2 base_footprint TF.

Fast-LIO publishes a full 6-DoF body pose:

    camera_init -> body

For Nav2 on flat ground we need a yaw-only navigation frame that does not
inherit body roll/pitch:

    camera_init -> unitree_go2/base_footprint

This node subscribes to /Odometry, extracts x/y/yaw, applies a planar body-frame
offset, and publishes the projected TF. It deliberately does not publish
body -> base_footprint, because that static edge would inherit body pitch/roll.
"""

import math
import os
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import ok as rclpy_ok
from tf2_ros import TransformBroadcaster

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    try:
        from rclpy._rclpy import RCLError
    except ImportError:
        class RCLError(Exception):
            pass


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class FastlioFootprintProjector(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_base_footprint_projector")

        self.odom_topic = self.declare_parameter("odom_topic", "/Odometry").value
        self.parent_frame = self.declare_parameter("parent_frame", "camera_init").value
        self.child_frame = self.declare_parameter(
            "child_frame", "unitree_go2/base_footprint"
        ).value

        # Planar offset from Fast-LIO body frame to navigation footprint frame.
        self.offset_x = float(
            self.declare_parameter(
                "body_to_footprint_x",
                float(os.environ.get("BODY_TO_FOOTPRINT_X", "-0.164143635")),
            ).value
        )
        self.offset_y = float(
            self.declare_parameter(
                "body_to_footprint_y",
                float(os.environ.get("BODY_TO_FOOTPRINT_Y", "0.0")),
            ).value
        )
        self.offset_z = float(
            self.declare_parameter(
                "body_to_footprint_z",
                float(os.environ.get("BODY_TO_FOOTPRINT_Z", "0.0")),
            ).value
        )
        self.max_abs_xy = float(
            self.declare_parameter(
                "max_abs_xy",
                float(os.environ.get("FASTLIO_FOOTPRINT_MAX_ABS_XY", "20.0")),
            ).value
        )
        self.max_jump_xy = float(
            self.declare_parameter(
                "max_jump_xy",
                float(os.environ.get("FASTLIO_FOOTPRINT_MAX_JUMP_XY", "0.75")),
            ).value
        )
        self.last_xy: Optional[Tuple[float, float]] = None
        self.last_warn_time = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.on_odom,
            20,
        )

        self.get_logger().info(
            "Publishing projected planar TF %s -> %s from %s, "
            "offset=(%.6f, %.6f, %.6f), max_abs_xy=%.2f, max_jump_xy=%.2f"
            % (
                self.parent_frame,
                self.child_frame,
                self.odom_topic,
                self.offset_x,
                self.offset_y,
                self.offset_z,
                self.max_abs_xy,
                self.max_jump_xy,
            )
        )

    def warn_throttled(self, message: str, period_s: float = 1.0) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_warn_time >= period_s:
            self.get_logger().error(message)
            self.last_warn_time = now

    def odom_is_healthy(self, x: float, y: float, yaw: float) -> bool:
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.warn_throttled("Rejecting Fast-LIO odometry: non-finite pose")
            return False

        if max(abs(x), abs(y)) > self.max_abs_xy:
            self.warn_throttled(
                "Rejecting Fast-LIO odometry: position outside sanity bound "
                f"x={x:.3f}, y={y:.3f}, max_abs_xy={self.max_abs_xy:.3f}"
            )
            return False

        if self.last_xy is not None:
            dx = x - self.last_xy[0]
            dy = y - self.last_xy[1]
            jump = math.hypot(dx, dy)
            if jump > self.max_jump_xy:
                self.warn_throttled(
                    "Rejecting Fast-LIO odometry: single-frame jump too large "
                    f"jump={jump:.3f} m, max_jump_xy={self.max_jump_xy:.3f}"
                )
                return False

        return True

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = p.x + cos_yaw * self.offset_x - sin_yaw * self.offset_y
        tf.transform.translation.y = p.y + sin_yaw * self.offset_x + cos_yaw * self.offset_y
        tf.transform.translation.z = self.offset_z

        if not self.odom_is_healthy(
            tf.transform.translation.x, tf.transform.translation.y, yaw
        ):
            return

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)
        self.last_xy = (tf.transform.translation.x, tf.transform.translation.y)


def main() -> None:
    rclpy.init()
    node = FastlioFootprintProjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy_ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
