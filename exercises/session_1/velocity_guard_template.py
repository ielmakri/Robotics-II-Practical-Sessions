#!/usr/bin/env python3
"""Session 1 starter: derive a velocity warning from JointState feedback."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class VelocityGuard(Node):
    def __init__(self):
        super().__init__("velocity_guard")

        # TODO 1: declare joint_name (default "tutorial_joint") and
        # velocity_limit (default 0.5), then read and validate both.

        # TODO 2: create a Bool publisher on "tutorial/velocity_warning".

        # TODO 3: create a JointState subscription on
        # "tutorial/joint_states" using self.receive_state as callback.

    def receive_state(self, message):
        # TODO 4: reject messages whose name/velocity lengths differ.
        # Build a dictionary, select self.joint_name and reject non-finite data.
        # Publish Bool(data=abs(velocity) >= self.velocity_limit).
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

