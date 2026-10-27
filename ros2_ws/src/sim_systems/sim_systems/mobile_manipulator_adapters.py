"""ROS and Drake adapters for the Husky-mounted Panda baseline."""

import math

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pydrake.common.value import AbstractValue
from pydrake.systems.framework import LeafSystem
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from sim_systems.ros_adapters import _set_stamp_from_context


class TwistToWheelRates(LeafSystem):
    """Convert planar velocity commands to the four Husky wheel rates."""

    WHEEL_RADIUS = 0.1651
    WHEEL_SEPARATION = 0.5708

    def __init__(self, wheel_joint_names):
        super().__init__()
        self.wheel_joint_names = list(wheel_joint_names)
        expected = {
            "front_left_wheel",
            "front_right_wheel",
            "rear_left_wheel",
            "rear_right_wheel",
        }
        if len(self.wheel_joint_names) != 4 or set(self.wheel_joint_names) != expected:
            raise ValueError(
                "Husky actuator order must contain each of the four wheel joints."
            )
        self.DeclareAbstractInputPort("twist", AbstractValue.Make(Twist()))
        self.DeclareVectorOutputPort(
            "wheel_rates", len(self.wheel_joint_names), self._calc_output
        )

    def _calc_output(self, context, output):
        command = self.get_input_port(0).Eval(context)
        linear = float(command.linear.x)
        angular = float(command.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            output.SetZero()
            return

        half_track = 0.5 * self.WHEEL_SEPARATION
        left_rate = (linear - angular * half_track) / self.WHEEL_RADIUS
        right_rate = (linear + angular * half_track) / self.WHEEL_RADIUS
        rates = {
            "front_left_wheel": left_rate,
            "rear_left_wheel": left_rate,
            "front_right_wheel": right_rate,
            "rear_right_wheel": right_rate,
        }
        output.SetFromVector([rates[name] for name in self.wheel_joint_names])


class HuskyOdometryMessageSystem(LeafSystem):
    """Report the measured Husky base pose and velocity as nav_msgs/Odometry."""

    def __init__(self, plant, base_body, state_size):
        super().__init__()
        self.plant = plant
        self.base_body = base_body
        self.plant_context = plant.CreateDefaultContext()
        self.num_positions = plant.num_positions()
        self.DeclareVectorInputPort("plant_state", state_size)
        self.DeclareAbstractOutputPort(
            "odometry",
            lambda: AbstractValue.Make(Odometry()),
            self._calc_output,
        )

    def _calc_output(self, context, output):
        state = self.get_input_port(0).Eval(context)
        self.plant.SetPositions(self.plant_context, state[: self.num_positions])
        self.plant.SetVelocities(self.plant_context, state[self.num_positions :])

        pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.base_body)
        velocity = self.plant.EvalBodySpatialVelocityInWorld(
            self.plant_context, self.base_body
        )
        rotation_world_from_base = pose.rotation().matrix()
        rotation_base_from_world = rotation_world_from_base.T
        angular_velocity_base = rotation_base_from_world @ velocity.rotational()
        linear_velocity_base = rotation_base_from_world @ velocity.translational()

        message = output.get_mutable_value()
        _set_stamp_from_context(message.header.stamp, context)
        message.header.frame_id = "world"
        message.child_frame_id = "husky_runner/base_link"
        position = pose.translation()
        orientation = pose.rotation().ToQuaternion().wxyz()
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.w = float(orientation[0])
        message.pose.pose.orientation.x = float(orientation[1])
        message.pose.pose.orientation.y = float(orientation[2])
        message.pose.pose.orientation.z = float(orientation[3])
        message.twist.twist.linear.x = float(linear_velocity_base[0])
        message.twist.twist.linear.y = float(linear_velocity_base[1])
        message.twist.twist.linear.z = float(linear_velocity_base[2])
        message.twist.twist.angular.x = float(angular_velocity_base[0])
        message.twist.twist.angular.y = float(angular_velocity_base[1])
        message.twist.twist.angular.z = float(angular_velocity_base[2])


class PandaGravityCompensationSystem(LeafSystem):
    """Feed forward Panda gravity torques in a plant shared with the Husky.

    The torque is sampled at 50 Hz and held between updates. This keeps the
    Python adapter out of the 1 kHz plant-step loop while remaining fast for
    the setpoint controller used in this teaching simulation.
    """

    def __init__(self, plant, panda_instance, state_size):
        super().__init__()
        self.plant = plant
        self.panda_instance = panda_instance
        self.num_positions = plant.num_positions()
        self.update_period = 0.02
        self.entries = []
        for index in plant.GetJointActuatorIndices(panda_instance):
            actuator = plant.get_joint_actuator(index)
            joint = actuator.joint()
            if actuator.num_inputs() != 1:
                raise RuntimeError(
                    f"Expected a one-DOF Panda actuator, got {actuator.name()}."
                )
            self.entries.append((actuator.input_start(), joint.velocity_start()))
        if len(self.entries) != 9:
            raise RuntimeError(
                "The Panda gravity compensation path expects seven arm and two "
                f"finger actuators; found {len(self.entries)}."
            )

        self.DeclareVectorInputPort("plant_state", state_size)
        self.torque_state_index = self.DeclareDiscreteState(
            plant.num_actuators()
        )
        self.DeclareVectorOutputPort(
            "feedforward_torque",
            plant.num_actuators(),
            self._calc_output,
        )
        self.DeclarePeriodicDiscreteUpdateEvent(
            self.update_period, 0.0, self._update_gravity_torque
        )

    def _calc_output(self, context, output):
        output.SetFromVector(
            context.get_discrete_state(self.torque_state_index).get_value()
        )

    def _update_gravity_torque(self, context, discrete_state):
        state = self.get_input_port(0).Eval(context)
        plant_context = self.plant.CreateDefaultContext()
        self.plant.SetPositions(plant_context, state[: self.num_positions])
        self.plant.SetVelocities(plant_context, state[self.num_positions :])
        gravity = self.plant.CalcGravityGeneralizedForces(plant_context)
        torques = np.zeros(self.plant.num_actuators())
        for actuator_start, velocity_start in self.entries:
            # Gravity contributes tau_g to the equations of motion, so its
            # static compensation command is -tau_g at the matching actuator.
            torques[actuator_start] = -gravity[velocity_start]
        discrete_state.get_mutable_vector(self.torque_state_index).SetFromVector(
            torques
        )


class LatchedJointTrajectorySetpointSystem(LeafSystem):
    """Interpolate valid named Panda arm goals from measured joint positions.

    Each final trajectory point is a position goal. A quintic profile starts
    at the measured Panda state and ends at the goal with zero velocity at
    both ends. A positive ``time_from_start`` can lengthen the automatically
    selected duration. Repeated copies of the same ROS message do not restart
    a motion, and malformed commands leave the previous goal unchanged.
    """

    MINIMUM_MOVE_DURATION = 1.2
    MAX_REFERENCE_VELOCITY = 1.0
    MAX_REFERENCE_ACCELERATION = 2.0
    QUINTIC_MAX_NORMALIZED_VELOCITY = 1.875
    QUINTIC_MAX_NORMALIZED_ACCELERATION = 5.773502691896258
    TARGET_MATCH_TOLERANCE = 1e-12

    def __init__(
        self,
        joint_names,
        default_positions,
        lower_limits,
        upper_limits,
        commanded_joint_names,
        measured_position_indices=None,
        update_period=0.02,
    ):
        super().__init__()
        self.joint_names = list(joint_names)
        self.commanded_joint_names = list(commanded_joint_names)
        self.num_joints = len(self.joint_names)
        self.num_commanded_joints = len(self.commanded_joint_names)
        self.measured_position_indices = list(
            range(self.num_joints)
            if measured_position_indices is None
            else measured_position_indices
        )
        self.default_positions = np.asarray(default_positions, dtype=float)
        self.lower_limits = np.asarray(lower_limits, dtype=float)
        self.upper_limits = np.asarray(upper_limits, dtype=float)
        self.update_period = float(update_period)

        if len(set(self.joint_names)) != self.num_joints:
            raise ValueError("joint_names must not contain duplicates.")
        if len(set(self.commanded_joint_names)) != self.num_commanded_joints:
            raise ValueError("commanded_joint_names must not contain duplicates.")
        if self.num_joints == 0 or self.num_commanded_joints == 0:
            raise ValueError("joint_names and commanded_joint_names must not be empty.")
        if len(self.measured_position_indices) != self.num_joints:
            raise ValueError(
                "measured_position_indices must contain one index per joint."
            )
        if (
            len(set(self.measured_position_indices)) != self.num_joints
            or any(
                not isinstance(index, int)
                or index < 0
                or index >= self.num_joints
                for index in self.measured_position_indices
            )
        ):
            raise ValueError(
                "measured_position_indices must be unique indices into the "
                "measured Panda position vector."
            )
        unknown_names = set(self.commanded_joint_names) - set(self.joint_names)
        if unknown_names:
            raise ValueError(
                "commanded_joint_names contains unknown names: "
                + ", ".join(sorted(unknown_names))
            )
        expected_shape = (self.num_joints,)
        for label, vector in (
            ("default_positions", self.default_positions),
            ("lower_limits", self.lower_limits),
            ("upper_limits", self.upper_limits),
        ):
            if vector.shape != expected_shape:
                raise ValueError(f"{label} must have shape {expected_shape}.")
        if self.update_period <= 0.0 or not np.isfinite(self.update_period):
            raise ValueError("update_period must be finite and positive.")

        self.commanded_indices = [
            self.joint_names.index(name) for name in self.commanded_joint_names
        ]
        self.motion_state_index = self.DeclareDiscreteState(
            np.concatenate(
                [self.default_positions, self.default_positions, [0.0, 0.0]]
            )
        )
        self.DeclareVectorInputPort("measured_state", 2 * self.num_joints)
        self.DeclareAbstractInputPort(
            "joint_trajectory", AbstractValue.Make(JointTrajectory())
        )
        self.DeclareVectorOutputPort(
            "desired_state", 2 * self.num_joints, self._calc_output
        )
        self.DeclarePeriodicDiscreteUpdateEvent(
            self.update_period, 0.0, self._accept_valid_target
        )

    def _calc_output(self, context, output):
        state = context.get_discrete_state(self.motion_state_index).get_value()
        start = state[: self.num_joints]
        target = state[self.num_joints : 2 * self.num_joints]
        start_time, duration = state[-2:]
        positions, velocities = self._quintic_reference(
            start,
            target,
            context.get_time() - start_time,
            duration,
        )
        output.SetFromVector(np.concatenate([positions, velocities]))

    @staticmethod
    def _quintic_reference(start, target, elapsed, duration):
        """Return smooth position and velocity references for one motion."""
        start = np.asarray(start, dtype=float)
        target = np.asarray(target, dtype=float)
        if duration <= 0.0 or elapsed >= duration:
            return target.copy(), np.zeros_like(target)
        if elapsed <= 0.0:
            return start.copy(), np.zeros_like(start)

        progress = elapsed / duration
        blend = (
            10.0 * progress**3
            - 15.0 * progress**4
            + 6.0 * progress**5
        )
        blend_rate = (
            30.0 * progress**2 * (1.0 - progress) ** 2 / duration
        )
        displacement = target - start
        return start + blend * displacement, blend_rate * displacement

    def _minimum_move_duration(self, start, target):
        largest_step = float(
            np.max(
                np.abs(
                    target[self.commanded_indices]
                    - start[self.commanded_indices]
                )
            )
        )
        velocity_duration = (
            self.QUINTIC_MAX_NORMALIZED_VELOCITY
            * largest_step
            / self.MAX_REFERENCE_VELOCITY
        )
        acceleration_duration = math.sqrt(
            self.QUINTIC_MAX_NORMALIZED_ACCELERATION
            * largest_step
            / self.MAX_REFERENCE_ACCELERATION
        )
        return max(
            self.MINIMUM_MOVE_DURATION,
            velocity_duration,
            acceleration_duration,
        )

    @staticmethod
    def _requested_duration(point):
        duration = (
            float(point.time_from_start.sec)
            + 1e-9 * float(point.time_from_start.nanosec)
        )
        return duration

    def _accept_valid_target(self, context, discrete_state):
        message = self.get_input_port(1).Eval(context)
        names_are_valid = (
            len(message.joint_names) == self.num_commanded_joints
            and len(set(message.joint_names)) == self.num_commanded_joints
            and set(message.joint_names) == set(self.commanded_joint_names)
        )
        if not message.points or not names_are_valid:
            return

        positions = message.points[-1].positions
        if len(positions) != self.num_commanded_joints:
            return
        by_name = dict(zip(message.joint_names, positions))
        candidate = np.asarray(
            [by_name[name] for name in self.commanded_joint_names], dtype=float
        )
        lower = self.lower_limits[self.commanded_indices]
        upper = self.upper_limits[self.commanded_indices]
        inside_limits = np.logical_and(candidate >= lower, candidate <= upper)
        if not np.all(np.isfinite(candidate)) or not np.all(inside_limits):
            return

        requested_duration = self._requested_duration(message.points[-1])
        if not math.isfinite(requested_duration) or requested_duration < 0.0:
            return

        motion_vector = discrete_state.get_mutable_vector(self.motion_state_index)
        previous_motion = motion_vector.get_value().copy()
        previous_target = previous_motion[
            self.num_joints : 2 * self.num_joints
        ]
        if np.all(
            np.abs(candidate - previous_target[self.commanded_indices])
            <= self.TARGET_MATCH_TOLERANCE
        ):
            return

        measured_state = self.get_input_port(0).Eval(context)
        measured_positions = np.asarray(
            measured_state[: self.num_joints], dtype=float
        )[self.measured_position_indices]
        if not np.all(np.isfinite(measured_positions)):
            return

        start = measured_positions.copy()
        target = start.copy()
        target[self.commanded_indices] = candidate
        duration = max(
            self._minimum_move_duration(start, target), requested_duration
        )
        motion_vector.SetFromVector(
            np.concatenate([start, target, [context.get_time(), duration]])
        )


class GripperPositionSetpointSystem(LeafSystem):
    """Latch a bounded per-finger position command for the Panda hand.

    The input is a one-value Float64MultiArray containing the desired position
    of each opposing finger in metres: 0.04 m is open and 0.0 m is closed.
    Empty, incorrectly sized, or invalid commands leave the previous setpoint
    unchanged.
    """

    def __init__(self, default_position=0.035, update_period=0.02):
        super().__init__()
        self.default_position = float(default_position)
        self.update_period = float(update_period)
        if not math.isfinite(self.default_position) or not 0.0 <= self.default_position <= 0.04:
            raise ValueError("default_position must be within [0.0, 0.04] metres.")
        if not math.isfinite(self.update_period) or self.update_period <= 0.0:
            raise ValueError("update_period must be finite and positive.")

        self.state_index = self.DeclareDiscreteState(
            np.full(2, self.default_position, dtype=float)
        )
        self.DeclareAbstractInputPort(
            "gripper_position", AbstractValue.Make(Float64MultiArray())
        )
        self.DeclareVectorOutputPort("finger_positions", 2, self._calc_output)
        self.DeclarePeriodicDiscreteUpdateEvent(
            self.update_period, 0.0, self._accept_command
        )

    def _calc_output(self, context, output):
        output.SetFromVector(
            context.get_discrete_state(self.state_index).get_value()
        )

    def _accept_command(self, context, discrete_state):
        message = self.get_input_port(0).Eval(context)
        if len(message.data) != 1:
            return
        position = float(message.data[0])
        if not math.isfinite(position) or not 0.0 <= position <= 0.04:
            return
        discrete_state.get_mutable_vector(self.state_index).SetFromVector(
            [position, position]
        )


class PandaGripperStateMergeSystem(LeafSystem):
    """Replace the two finger setpoints while preserving the seven-arm goal."""

    def __init__(self, joint_names):
        super().__init__()
        self.joint_names = list(joint_names)
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique.")
        self.finger_names = ["panda_finger_joint1", "panda_finger_joint2"]
        if not set(self.finger_names).issubset(self.joint_names):
            raise ValueError("joint_names must include both Panda finger joints.")
        self.finger_indices = [
            self.joint_names.index(name) for name in self.finger_names
        ]
        self.num_joints = len(self.joint_names)
        self.DeclareVectorInputPort("arm_and_default_gripper_state", 2 * self.num_joints)
        self.DeclareVectorInputPort("finger_positions", 2)
        self.DeclareVectorOutputPort(
            "desired_state", 2 * self.num_joints, self._calc_output
        )

    def _calc_output(self, context, output):
        desired = self.get_input_port(0).Eval(context).copy()
        finger_positions = self.get_input_port(1).Eval(context)
        desired[self.finger_indices] = finger_positions
        desired[self.num_joints + np.asarray(self.finger_indices)] = 0.0
        output.SetFromVector(desired)


class VectorReorderSystem(LeafSystem):
    """Reorder one-actuator-per-joint vectors to a requested joint-name order."""

    def __init__(self, source_joint_names, destination_joint_names):
        super().__init__()
        self.source_joint_names = list(source_joint_names)
        self.destination_joint_names = list(destination_joint_names)
        if len(set(self.source_joint_names)) != len(self.source_joint_names):
            raise ValueError("Source joint names must be unique.")
        if set(self.source_joint_names) != set(self.destination_joint_names):
            raise ValueError("Source and destination joint names must match.")
        self.destination_indices = [
            self.source_joint_names.index(name)
            for name in self.destination_joint_names
        ]
        self.DeclareVectorInputPort("source", len(self.source_joint_names))
        self.DeclareVectorOutputPort(
            "reordered", len(self.destination_joint_names), self._calc_output
        )

    def _calc_output(self, context, output):
        source = self.get_input_port(0).Eval(context)
        output.SetFromVector(source[self.destination_indices])
