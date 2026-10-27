"""Apply limits and a command timeout to the Husky's planar velocity input."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class HuskyCommandGuard(Node):
    def __init__(self):
        super().__init__("husky_command_guard")
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 0.8)
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        guard_values = (
            self.command_timeout,
            self.max_linear_speed,
            self.max_angular_speed,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in guard_values):
            raise ValueError("Husky guard parameters must be finite and positive.")

        self.last_command = Twist()
        self.last_command_time = None
        self.publisher = self.create_publisher(
            Twist, "/husky_runner/cmd_vel_safe", 10
        )
        self.subscription = self.create_subscription(
            Twist, "/husky_runner/cmd_vel", self._receive_command, 10
        )
        self.timer = self.create_timer(0.05, self._publish_safe_command)

    def _receive_command(self, message):
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().warning("Ignoring a non-finite Husky velocity command.")
            self.last_command = Twist()
            self.last_command_time = None
            return

        safe = Twist()
        safe.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, linear))
        safe.angular.z = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular)
        )
        self.last_command = safe
        self.last_command_time = time.monotonic()

    def _publish_safe_command(self):
        command_is_fresh = (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time <= self.command_timeout
        )
        self.publisher.publish(self.last_command if command_is_fresh else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HuskyCommandGuard()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        # A launch shutdown can already have shut down the default context
        # before this finally block runs.
        if rclpy.ok():
            rclpy.shutdown()
