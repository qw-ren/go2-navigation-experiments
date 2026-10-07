#!/usr/bin/python3
"""Safety gate for Nav2 velocity commands.

This node is deliberately not a Unitree command sender. It only filters Nav2
debug velocity commands and republishes a safe, limited command on a separate
topic. A future robot-specific bridge may subscribe to the output topic after
manual validation.
"""

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


@dataclass
class Command:
    linear_x: float = 0.0
    angular_z: float = 0.0


def clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    return max(-limit, min(limit, value))


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


class SafeCmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__("go2_safe_cmd_vel_gate")

        self.declare_parameter("input_topic", "/nav2_debug/cmd_vel")
        self.declare_parameter("output_topic", "/go2_safe/cmd_vel")
        self.declare_parameter("status_topic", "/go2_safe/status")
        self.declare_parameter("enable_topic", "/go2_safe/enable")
        self.declare_parameter("estop_topic", "/go2_safe/estop")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("input_timeout_s", 0.35)
        self.declare_parameter("enabled_on_start", False)

        # Conservative first-motion limits. Keep these low until manual tests
        # prove the complete bridge + robot response.
        self.declare_parameter("max_linear_x", 0.10)
        self.declare_parameter("max_angular_z", 0.30)
        self.declare_parameter("max_accel_linear_x", 0.20)
        self.declare_parameter("max_accel_angular_z", 0.60)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.enable_topic = self.get_parameter("enable_topic").value
        self.estop_topic = self.get_parameter("estop_topic").value

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.enabled = bool(self.get_parameter("enabled_on_start").value)

        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.max_accel_linear_x = float(self.get_parameter("max_accel_linear_x").value)
        self.max_accel_angular_z = float(self.get_parameter("max_accel_angular_z").value)

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")
        if self.input_timeout_s <= 0.0:
            raise ValueError("input_timeout_s must be > 0")

        self.estop = False
        self.last_input_time = None
        self.last_timer_time = self.get_clock().now()
        self.raw_cmd = Command()
        self.out_cmd = Command()
        self.last_status = ""

        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(Twist, self.input_topic, self.on_cmd, 10)
        self.create_subscription(Bool, self.enable_topic, self.on_enable, 10)
        self.create_subscription(Bool, self.estop_topic, self.on_estop, 10)

        self.create_service(SetBool, "/go2_safe/set_enabled", self.on_set_enabled)
        self.create_service(Trigger, "/go2_safe/stop", self.on_stop)
        self.create_service(Trigger, "/go2_safe/reset_estop", self.on_reset_estop)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.get_logger().info(
            "Safe cmd_vel gate started: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"enabled={self.enabled}, max_x={self.max_linear_x:.3f}, "
            f"max_yaw={self.max_angular_z:.3f}, timeout={self.input_timeout_s:.3f}s"
        )

    def on_cmd(self, msg: Twist) -> None:
        self.raw_cmd = Command(
            linear_x=finite_or_zero(msg.linear.x),
            angular_z=finite_or_zero(msg.angular.z),
        )
        self.last_input_time = self.get_clock().now()

    def on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data) and not self.estop
        if not self.enabled:
            self.out_cmd = Command()
        self.get_logger().warn(f"enable topic set enabled={self.enabled}")

    def on_estop(self, msg: Bool) -> None:
        if msg.data:
            self.estop = True
            self.enabled = False
            self.out_cmd = Command()
            self.get_logger().error("E-stop asserted; output forced to zero")

    def on_set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and self.estop:
            response.success = False
            response.message = "Cannot enable while E-stop is asserted. Call /go2_safe/reset_estop first."
            return response
        self.enabled = bool(request.data)
        if not self.enabled:
            self.out_cmd = Command()
        response.success = True
        response.message = f"enabled={self.enabled}"
        self.get_logger().warn(response.message)
        return response

    def on_stop(self, request: Trigger.Request, response: Trigger.Response):
        del request
        self.enabled = False
        self.out_cmd = Command()
        response.success = True
        response.message = "Stopped and disabled; output forced to zero."
        self.get_logger().warn(response.message)
        return response

    def on_reset_estop(self, request: Trigger.Request, response: Trigger.Response):
        del request
        self.estop = False
        self.enabled = False
        self.out_cmd = Command()
        response.success = True
        response.message = "E-stop reset. Gate remains disabled."
        self.get_logger().warn(response.message)
        return response

    def input_age_s(self):
        if self.last_input_time is None:
            return None
        now = self.get_clock().now()
        return (now - self.last_input_time).nanoseconds / 1e9

    def slew_limit(self, current: float, target: float, accel: float, dt: float) -> float:
        step = abs(accel) * max(dt, 0.0)
        return current + clamp(target - current, step)

    def desired_cmd(self) -> Command:
        age = self.input_age_s()
        stale = age is None or age > self.input_timeout_s
        if not self.enabled or self.estop or stale:
            return Command()
        return Command(
            linear_x=clamp(self.raw_cmd.linear_x, self.max_linear_x),
            angular_z=clamp(self.raw_cmd.angular_z, self.max_angular_z),
        )

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now

        target = self.desired_cmd()
        age = self.input_age_s()
        stale = age is None or age > self.input_timeout_s

        if not self.enabled or self.estop or stale:
            # Fail closed: publish immediate zero. Do not ramp down on stale
            # commands or E-stop.
            self.out_cmd = Command()
        else:
            self.out_cmd = Command(
                linear_x=self.slew_limit(
                    self.out_cmd.linear_x, target.linear_x, self.max_accel_linear_x, dt
                ),
                angular_z=self.slew_limit(
                    self.out_cmd.angular_z, target.angular_z, self.max_accel_angular_z, dt
                ),
            )

        msg = Twist()
        msg.linear.x = self.out_cmd.linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.out_cmd.angular_z
        self.cmd_pub.publish(msg)

        age_text = "none" if age is None else f"{age:.3f}"
        status = (
            f"enabled={self.enabled} estop={self.estop} stale={stale} "
            f"age_s={age_text} raw_x={self.raw_cmd.linear_x:.3f} "
            f"raw_yaw={self.raw_cmd.angular_z:.3f} out_x={self.out_cmd.linear_x:.3f} "
            f"out_yaw={self.out_cmd.angular_z:.3f}"
        )
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.last_status = status


def main() -> None:
    rclpy.init()
    node = SafeCmdVelGate()
    try:
        rclpy.spin(node)
    finally:
        zero = Twist()
        node.cmd_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
