#!/usr/bin/env python3
"""Publish a validated named-joint setpoint for the controlled Panda."""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robotics_ii_tutorials.validation import (
    PANDA_HOME,
    PANDA_JOINT_NAMES,
    validate_panda_target,
)


class PandaJointCommander(Node):
    """Repeated publication lets late-starting subscribers receive the target."""

    def __init__(self):
        super().__init__("panda_joint_commander")
        self.declare_parameter("target_positions", list(PANDA_HOME))
        self.declare_parameter("publish_period", 0.5)

        self.target = validate_panda_target(
            self.get_parameter("target_positions").value
        )
        publish_period = float(self.get_parameter("publish_period").value)
        if publish_period <= 0.0:
            raise ValueError("publish_period must be positive.")

        self.publisher = self.create_publisher(
            JointTrajectory,
            "panda/joint_trajectory",
            10,
        )
        self.timer = self.create_timer(publish_period, self.publish_target)
        self.get_logger().info(
            "Validated Panda target: "
            + ", ".join(f"{value:+.3f}" for value in self.target)
        )

    def publish_target(self):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(PANDA_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = self.target
        point.velocities = [0.0] * len(PANDA_JOINT_NAMES)
        message.points = [point]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PandaJointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

