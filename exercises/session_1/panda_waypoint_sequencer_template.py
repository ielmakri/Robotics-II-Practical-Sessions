#!/usr/bin/env python3
"""Session 1 capstone: command two Panda targets using feedback.

Copy this file into the robotics_ii_tutorials Python package, rename it
``panda_waypoint_sequencer.py`` and complete every TODO.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robotics_ii_tutorials.validation import (
    PANDA_HOME,
    PANDA_JOINT_NAMES,
    named_positions,
    validate_panda_target,
)


class PandaWaypointSequencer(Node):
    """Publish target 1, wait for convergence, then publish target 2."""

    def __init__(self):
        super().__init__("panda_waypoint_sequencer")

        self.declare_parameter("first_target", list(PANDA_HOME))
        self.declare_parameter("second_target", list(PANDA_HOME))
        self.declare_parameter("goal_tolerance", 0.04)
        self.declare_parameter("hold_time", 1.0)
        self.declare_parameter("command_period", 0.5)

        self.targets = [
            validate_panda_target(self.get_parameter("first_target").value),
            validate_panda_target(self.get_parameter("second_target").value),
        ]
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.command_period = float(self.get_parameter("command_period").value)
        if self.goal_tolerance <= 0.0:
            raise ValueError("goal_tolerance must be positive.")
        if self.hold_time < 0.0:
            raise ValueError("hold_time must be non-negative.")
        if self.command_period <= 0.0:
            raise ValueError("command_period must be positive.")

        self.target_index = 0
        self.inside_tolerance_since = None
        self.sequence_complete = False

        # TODO 1: create a JointTrajectory publisher on
        # "panda/joint_trajectory" with queue depth 10.

        # TODO 2: create a JointState subscription on
        # "sim_sys/joint_states" using self.receive_state as callback.

        # TODO 3: create a timer with self.command_period that calls
        # self.publish_target.

        self.get_logger().info("Starting Panda waypoint 1/2")

    @property
    def current_target(self):
        return self.targets[self.target_index]

    def publish_target(self):
        """Repeated publication also serves late-starting subscribers."""
        if self.sequence_complete:
            return

        # TODO 4: construct and publish one JointTrajectory message.
        # - stamp it with self.get_clock().now().to_msg()
        # - set joint_names to PANDA_JOINT_NAMES
        # - add one JointTrajectoryPoint containing self.current_target
        # - give that point seven zero desired velocities
        pass

    def receive_state(self, message):
        """Advance only after the current target is stable for hold_time."""
        if self.sequence_complete:
            return

        measured = named_positions(
            message.name,
            message.position,
            PANDA_JOINT_NAMES,
        )
        if measured is None:
            return

        # TODO 5: calculate the maximum absolute joint error between
        # self.current_target and measured.
        max_error = float("inf")

        now_seconds = self.get_clock().now().nanoseconds * 1e-9

        # TODO 6: implement the state transition.
        # - outside tolerance: set inside_tolerance_since back to None
        # - first sample inside tolerance: store now_seconds
        # - inside tolerance for hold_time at target 1: select target 2,
        #   reset inside_tolerance_since, log
        #   "Waypoint 1 reached; commanding waypoint 2/2" and immediately call
        #   publish_target()
        # - inside tolerance for hold_time at target 2: set
        #   sequence_complete True and log "SEQUENCE COMPLETE"
        _ = max_error, now_seconds


def main(args=None):
    rclpy.init(args=args)
    node = PandaWaypointSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
