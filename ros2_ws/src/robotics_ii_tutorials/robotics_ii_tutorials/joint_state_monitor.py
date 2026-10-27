#!/usr/bin/env python3
"""Subscribe to JointState feedback and publish a derived warning."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class JointStateMonitor(Node):
    """Warn when one named joint exceeds an absolute position threshold."""

    def __init__(self):
        super().__init__("joint_state_monitor")
        self.declare_parameter("joint_name", "tutorial_joint")
        self.declare_parameter("warning_position", 0.75)
        self.declare_parameter("report_period", 1.0)

        self.joint_name = str(self.get_parameter("joint_name").value)
        self.warning_position = float(
            self.get_parameter("warning_position").value
        )
        report_period = float(self.get_parameter("report_period").value)
        if self.warning_position <= 0.0 or report_period <= 0.0:
            raise ValueError("warning_position and report_period must be positive.")

        self.latest_position = None
        self.warning_publisher = self.create_publisher(
            Bool,
            "tutorial/limit_warning",
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            "tutorial/joint_states",
            self.receive_state,
            10,
        )
        self.report_timer = self.create_timer(report_period, self.report)
        self.get_logger().info(
            f"Monitoring {self.joint_name}; warning at "
            f"|q| >= {self.warning_position:.3f} rad"
        )

    def receive_state(self, message):
        if len(message.name) != len(message.position):
            self.get_logger().warning("Ignoring malformed JointState message.")
            return

        by_name = dict(zip(message.name, message.position))
        if self.joint_name not in by_name:
            return

        position = float(by_name[self.joint_name])
        if not math.isfinite(position):
            self.get_logger().warning("Ignoring non-finite joint position.")
            return

        self.latest_position = position
        warning = Bool()
        warning.data = abs(position) >= self.warning_position
        self.warning_publisher.publish(warning)

    def report(self):
        if self.latest_position is None:
            self.get_logger().info("Waiting for joint feedback...")
            return
        state = "LIMIT WARNING" if abs(self.latest_position) >= self.warning_position else "ok"
        self.get_logger().info(
            f"{self.joint_name}: q={self.latest_position:+.3f} rad ({state})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

