#!/usr/bin/env python3
"""Execute the tote placement as short, vertical-orientation arm segments."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

from solve_pick_place_ik import DEFAULT_SCENE, _find_instance, _solve_vertical_placement_path
from sim_systems.panda_model import PANDA_ARM_JOINT_NAMES, PANDA_HOME


ROOT = Path(__file__).resolve().parents[2]
JOINT_STATES_TOPIC = "/panda_inspector/joint_states"
JOINT_TRAJECTORY_TOPIC = "/panda_inspector/joint_trajectory"


def _plan_path(scene_file, segments):
    plant = MultibodyPlant(time_step=0.001)
    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(ROOT / "models"))
    model_parser.AddModels(file_name=str(scene_file))
    plant.Finalize()
    panda = _find_instance(plant, "panda_inspector")
    plant.SetDefaultPositions(panda, PANDA_HOME)
    return _solve_vertical_placement_path(plant, panda, segments=segments)


class VerticalPlaceExecutor(Node):
    """Send one joint waypoint at a time and wait for measured settling."""

    def __init__(self, tolerance, timeout):
        super().__init__("vertical_tote_place")
        self.tolerance = float(tolerance)
        self.timeout = float(timeout)
        self.publisher = self.create_publisher(
            JointTrajectory, JOINT_TRAJECTORY_TOPIC, 10
        )
        self.subscription = self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self._on_joint_state, 10
        )
        self.latest_positions = None
        self.feedback_sequence = 0
        self.last_feedback_time = 0.0

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in PANDA_ARM_JOINT_NAMES):
            return
        self.latest_positions = [
            float(positions[name]) for name in PANDA_ARM_JOINT_NAMES
        ]
        self.feedback_sequence += 1
        self.last_feedback_time = time.monotonic()

    def wait_for_target(self, target, label):
        target = [float(value) for value in target]
        deadline = time.monotonic() + self.timeout
        settled_messages = 0
        last_sequence = self.feedback_sequence

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.feedback_sequence == last_sequence:
                if (
                    self.last_feedback_time
                    and time.monotonic() - self.last_feedback_time > 2.0
                ):
                    raise RuntimeError("Panda joint-state feedback became stale.")
                continue
            last_sequence = self.feedback_sequence

            errors = [
                abs(actual - desired)
                for actual, desired in zip(self.latest_positions, target)
            ]
            if max(errors) <= self.tolerance:
                settled_messages += 1
                if settled_messages >= 4:
                    return
            else:
                settled_messages = 0

        raise RuntimeError(f"Panda did not settle at {label} before timeout.")

    def wait_for_command_subscriber(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("No Panda trajectory controller is subscribed.")

    def publish_target(self, target):
        message = JointTrajectory()
        message.joint_names = list(PANDA_ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in target]
        message.points = [point]
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--segments", type=int, default=32)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--tolerance", type=float, default=0.02)
    args = parser.parse_args()

    scene_file = args.scene.expanduser().resolve()
    if not scene_file.is_file():
        parser.error(f"SDF scene does not exist: {scene_file}")
    if args.timeout <= 0.0 or args.tolerance <= 0.0:
        parser.error("timeout and tolerance must be positive.")

    try:
        path = _plan_path(scene_file, args.segments)
        rclpy.init(args=None)
        node = VerticalPlaceExecutor(args.tolerance, args.timeout)
        try:
            node.get_logger().info(
                f"Waiting for Panda at the lifted tote pose; "
                f"planned {len(path) - 1} vertical-orientation waypoints."
            )
            node.wait_for_target(path[0], "the lifted tote pose")
            node.wait_for_command_subscriber()
            for index, target in enumerate(path[1:], start=1):
                node.publish_target(target)
                node.wait_for_target(target, f"waypoint {index}/{len(path) - 1}")
                node.get_logger().info(
                    f"Reached vertical placement waypoint {index}/{len(path) - 1}."
                )
            node.get_logger().info(
                "Final placement pose reached. Open the gripper to release the tote."
            )
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except Exception as error:
        print(f"Vertical placement stopped: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
