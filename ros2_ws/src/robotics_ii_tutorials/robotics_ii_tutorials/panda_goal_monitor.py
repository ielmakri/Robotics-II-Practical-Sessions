#!/usr/bin/env python3
"""Compute maximum named-joint tracking error from Panda feedback."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from robotics_ii_tutorials.validation import (
    PANDA_HOME,
    PANDA_JOINT_NAMES,
    named_positions,
    validate_panda_target,
)


class PandaGoalMonitor(Node):
    def __init__(self):
        super().__init__("panda_goal_monitor")
        self.declare_parameter("target_positions", list(PANDA_HOME))
        self.declare_parameter("goal_tolerance", 0.03)

        self.target = validate_panda_target(
            self.get_parameter("target_positions").value
        )
        self.tolerance = float(self.get_parameter("goal_tolerance").value)
        if self.tolerance <= 0.0:
            raise ValueError("goal_tolerance must be positive.")

        self.latest_positions = None
        self.goal_publisher = self.create_publisher(
            Bool,
            "panda/goal_reached",
            10,
        )
        self.subscription = self.create_subscription(
            JointState,
            "sim_sys/joint_states",
            self.receive_state,
            10,
        )
        self.timer = self.create_timer(1.0, self.report)

    def receive_state(self, message):
        self.latest_positions = named_positions(
            message.name,
            message.position,
            PANDA_JOINT_NAMES,
        )

    def report(self):
        if self.latest_positions is None:
            self.get_logger().info("Waiting for valid Panda joint feedback...")
            return

        max_error = max(
            abs(desired - measured)
            for desired, measured in zip(self.target, self.latest_positions)
        )
        reached = Bool()
        reached.data = max_error <= self.tolerance
        self.goal_publisher.publish(reached)
        state = "GOAL REACHED" if reached.data else "tracking"
        self.get_logger().info(f"{state}; max error={max_error:.4f} rad")


def main(args=None):
    rclpy.init(args=args)
    node = PandaGoalMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

