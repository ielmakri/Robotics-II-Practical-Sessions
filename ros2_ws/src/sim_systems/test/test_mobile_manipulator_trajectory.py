import unittest

import numpy as np
from pydrake.systems.analysis import Simulator
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sim_systems.mobile_manipulator_adapters import (
    LatchedJointTrajectorySetpointSystem,
)


def _trajectory(joint_names, positions, duration_seconds=4.0):
    message = JointTrajectory()
    message.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = int(duration_seconds)
    point.time_from_start.nanosec = int(
        round((duration_seconds - int(duration_seconds)) * 1e9)
    )
    message.points = [point]
    return message


def _make_simulator(duration_seconds=4.0):
    system = LatchedJointTrajectorySetpointSystem(
        joint_names=["panda_joint1", "panda_joint2"],
        default_positions=[0.0, 0.0],
        lower_limits=[-2.0, -2.0],
        upper_limits=[2.0, 2.0],
        commanded_joint_names=["panda_joint1", "panda_joint2"],
        # Deliberately reverse the measured-state order to check the mapping.
        measured_position_indices=[1, 0],
        update_period=0.02,
    )
    simulator = Simulator(system)
    context = simulator.get_mutable_context()
    # Measured order is [joint2, joint1], so the adapter's start pose is
    # [joint1, joint2] = [0.25, -0.30].
    system.get_input_port(0).FixValue(context, [-0.30, 0.25, 0.0, 0.0])
    system.get_input_port(1).FixValue(
        context,
        _trajectory(
            ["panda_joint2", "panda_joint1"],
            [0.5, 1.0],
            duration_seconds=duration_seconds,
        ),
    )
    simulator.Initialize()
    simulator.AdvanceTo(0.02)
    return system, simulator


class TestLatchedJointTrajectorySetpointSystem(unittest.TestCase):
    def test_automatic_duration_respects_reference_limits(self):
        system, simulator = _make_simulator(duration_seconds=0.0)
        state = simulator.get_context().get_discrete_state(
            system.motion_state_index
        ).get_value()
        duration = state[-1]
        largest_step = 0.8

        self.assertGreaterEqual(duration, system.MINIMUM_MOVE_DURATION)
        self.assertLessEqual(
            system.QUINTIC_MAX_NORMALIZED_VELOCITY * largest_step / duration,
            system.MAX_REFERENCE_VELOCITY,
        )
        self.assertLessEqual(
            system.QUINTIC_MAX_NORMALIZED_ACCELERATION
            * largest_step
            / duration**2,
            system.MAX_REFERENCE_ACCELERATION,
        )

    def test_joint_command_uses_smooth_profile_and_ends_at_target(self):
        system, simulator = _make_simulator()
        context = simulator.get_mutable_context()
        state = context.get_discrete_state(system.motion_state_index).get_value()
        start_time = state[-2]
        duration = state[-1]

        simulator.AdvanceTo(start_time + duration / 2.0)
        midpoint = system.get_output_port(0).Eval(context)
        np.testing.assert_allclose(midpoint[:2], [0.625, 0.1], atol=1e-8)
        np.testing.assert_allclose(midpoint[2:], [0.3515625, 0.375], atol=1e-8)

        simulator.AdvanceTo(start_time + duration)
        final = system.get_output_port(0).Eval(context)
        np.testing.assert_allclose(final[:2], [1.0, 0.5], atol=1e-8)
        np.testing.assert_allclose(final[2:], [0.0, 0.0], atol=1e-8)

    def test_new_goal_starts_from_measured_state(self):
        system, simulator = _make_simulator()
        context = simulator.get_mutable_context()
        motion = context.get_discrete_state(system.motion_state_index).get_value()
        midpoint_time = motion[-2] + motion[-1] / 2.0
        simulator.AdvanceTo(midpoint_time)
        measured_start = system.get_output_port(0).Eval(context)[:2].copy()

        # Supply that measured pose in the plant-state ordering used by the port.
        system.get_input_port(0).FixValue(
            context,
            [measured_start[1], measured_start[0], 0.0, 0.0],
        )
        system.get_input_port(1).FixValue(
            context,
            _trajectory(["panda_joint1", "panda_joint2"], [-0.2, 0.3]),
        )
        simulator.AdvanceTo(midpoint_time + 0.02)

        restarted = system.get_output_port(0).Eval(context)
        np.testing.assert_allclose(restarted[:2], measured_start, atol=1e-5)
        np.testing.assert_allclose(restarted[2:], [0.0, 0.0], atol=2e-4)


if __name__ == "__main__":
    unittest.main()
