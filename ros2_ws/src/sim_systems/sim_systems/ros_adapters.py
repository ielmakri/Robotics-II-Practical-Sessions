"""Typed adapters between Drake vector ports and ROS messages."""

import math

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem


def _set_stamp_from_context(stamp, context):
    seconds = context.get_time()
    stamp.sec = int(math.floor(seconds))
    stamp.nanosec = int(round((seconds - stamp.sec) * 1_000_000_000))
    if stamp.nanosec >= 1_000_000_000:
        stamp.sec += 1
        stamp.nanosec -= 1_000_000_000


class JointStateMessageSystem(LeafSystem):
    """Convert q, v and actuation vectors into sensor_msgs/JointState."""

    def __init__(self, joint_names):
        super().__init__()
        self.joint_names = list(joint_names)
        self.num_joints = len(self.joint_names)
        self.DeclareVectorInputPort("state", 2 * self.num_joints)
        self.DeclareVectorInputPort("actuation", self.num_joints)
        self.DeclareAbstractOutputPort(
            "joint_state",
            lambda: AbstractValue.Make(JointState()),
            self._calc_output,
        )

    def _calc_output(self, context, output):
        state = self.get_input_port(0).Eval(context)
        actuation = self.get_input_port(1).Eval(context)
        message = output.get_mutable_value()
        _set_stamp_from_context(message.header.stamp, context)
        message.name = self.joint_names
        message.position = state[: self.num_joints].tolist()
        message.velocity = state[self.num_joints :].tolist()
        message.effort = actuation.tolist()


class VectorToFloatArraySystem(LeafSystem):
    """Convert a fixed-size Drake vector into Float64MultiArray."""

    def __init__(self, vector_size):
        super().__init__()
        self.DeclareVectorInputPort("vector", vector_size)
        self.DeclareAbstractOutputPort(
            "message",
            lambda: AbstractValue.Make(Float64MultiArray()),
            self._calc_output,
        )

    def _calc_output(self, context, output):
        output.get_mutable_value().data = self.get_input_port(0).Eval(context).tolist()


class DesiredPositionMessageSystem(LeafSystem):
    """Expose the q part of a desired [q, v] vector for plotting."""

    def __init__(self, num_joints):
        super().__init__()
        self.num_joints = num_joints
        self.DeclareVectorInputPort("desired_state", 2 * num_joints)
        self.DeclareAbstractOutputPort(
            "desired_positions",
            lambda: AbstractValue.Make(Float64MultiArray()),
            self._calc_output,
        )

    def _calc_output(self, context, output):
        state = self.get_input_port(0).Eval(context)
        output.get_mutable_value().data = state[: self.num_joints].tolist()


class JointTrajectoryToStateSystem(LeafSystem):
    """Convert the last valid trajectory point to desired [q, v].

    This example implements a setpoint contract. It intentionally does not
    interpolate points or interpret time_from_start.
    """

    def __init__(
        self,
        joint_names,
        default_positions,
        lower_limits,
        upper_limits,
        commanded_joint_names=None,
    ):
        super().__init__()
        self.joint_names = list(joint_names)
        self.num_joints = len(self.joint_names)
        self.default_positions = np.asarray(default_positions, dtype=float)
        self.lower_limits = np.asarray(lower_limits, dtype=float)
        self.upper_limits = np.asarray(upper_limits, dtype=float)
        self.commanded_joint_names = list(commanded_joint_names or self.joint_names)
        if len(set(self.commanded_joint_names)) != len(self.commanded_joint_names):
            raise ValueError("commanded_joint_names must not contain duplicates.")
        unknown_names = set(self.commanded_joint_names) - set(self.joint_names)
        if unknown_names:
            raise ValueError(
                "commanded_joint_names contains unknown names: "
                + ", ".join(sorted(unknown_names))
            )
        self.commanded_indices = [
            self.joint_names.index(name) for name in self.commanded_joint_names
        ]
        self.num_commanded_joints = len(self.commanded_joint_names)

        expected_shape = (self.num_joints,)
        for label, vector in (
            ("default_positions", self.default_positions),
            ("lower_limits", self.lower_limits),
            ("upper_limits", self.upper_limits),
        ):
            if vector.shape != expected_shape:
                raise ValueError(f"{label} must have shape {expected_shape}.")

        self.DeclareAbstractInputPort(
            "joint_trajectory",
            AbstractValue.Make(JointTrajectory()),
        )
        self.DeclareVectorOutputPort(
            "desired_state",
            2 * self.num_joints,
            self._calc_output,
        )

    def _calc_output(self, context, output):
        message = self.get_input_port(0).Eval(context)
        positions = self.default_positions.copy()
        velocities = np.zeros(self.num_joints)

        valid_names = (
            len(message.joint_names) == self.num_commanded_joints
            and len(set(message.joint_names)) == self.num_commanded_joints
            and set(message.joint_names) == set(self.commanded_joint_names)
        )
        if message.points and valid_names:
            point = message.points[-1]
            by_name = dict(zip(message.joint_names, point.positions))
            if len(point.positions) == self.num_commanded_joints:
                candidate = np.asarray(
                    [by_name[name] for name in self.commanded_joint_names],
                    dtype=float,
                )
                selected_lower_limits = self.lower_limits[self.commanded_indices]
                selected_upper_limits = self.upper_limits[self.commanded_indices]
                inside_limits = np.logical_and(
                    candidate >= selected_lower_limits,
                    candidate <= selected_upper_limits,
                )
                if np.all(np.isfinite(candidate)) and np.all(inside_limits):
                    positions[self.commanded_indices] = candidate

                    if len(point.velocities) == self.num_commanded_joints:
                        velocity_by_name = dict(
                            zip(message.joint_names, point.velocities)
                        )
                        candidate_velocity = np.asarray(
                            [
                                velocity_by_name[name]
                                for name in self.commanded_joint_names
                            ],
                            dtype=float,
                        )
                        if np.all(np.isfinite(candidate_velocity)):
                            velocities[self.commanded_indices] = candidate_velocity

        output.SetFromVector(np.concatenate([positions, velocities]))
