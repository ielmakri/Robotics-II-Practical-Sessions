#!/usr/bin/env python3
"""Drake checks for the vertical-orientation tote placement path."""

import unittest
from pathlib import Path

import numpy as np
from pydrake.math import RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

from solve_pick_place_ik import _find_instance, _solve_vertical_placement_path
from sim_systems.panda_model import (
    PANDA_ARM_JOINT_NAMES,
    PANDA_HOME,
    PANDA_LOWER_LIMITS,
    PANDA_UPPER_LIMITS,
)


ROOT = Path(__file__).resolve().parents[2]
START = np.array([-0.20, -0.55, 0.9475])
FINAL = np.array([-0.55, -0.55, 0.51913])


class TestVerticalPlacementPath(unittest.TestCase):
    def test_path_stays_upright_above_platform_and_reaches_final_pose(self):
        plant = MultibodyPlant(time_step=0.001)
        model_parser = Parser(plant)
        model_parser.package_map().Add("robotics_ii_models", str(ROOT / "models"))
        model_parser.AddModels(
            file_name=str(ROOT / "models/team_projects/husky_panda_pick_place.sdf")
        )
        plant.Finalize()
        panda = _find_instance(plant, "panda_inspector")
        plant.SetDefaultPositions(panda, PANDA_HOME)
        path = np.asarray(_solve_vertical_placement_path(plant, panda))
        self.assertEqual(path.shape, (33, len(PANDA_ARM_JOINT_NAMES)))

        arm_indices = [
            plant.GetJointByName(name, panda).position_start()
            for name in PANDA_ARM_JOINT_NAMES
        ]
        hand = plant.GetFrameByName("panda_hand", panda)
        target_rotation = RotationMatrix(RollPitchYaw(np.pi, 0.0, 0.0)).matrix()
        context = plant.CreateDefaultContext()
        positions = plant.GetPositions(context).copy()
        measured_poses = []

        lower = PANDA_LOWER_LIMITS[:7] + 0.05
        upper = PANDA_UPPER_LIMITS[:7] - 0.05
        for target in path:
            self.assertTrue(np.all(target >= lower - 1e-8))
            self.assertTrue(np.all(target <= upper + 1e-8))
            positions[arm_indices] = target
            plant.SetPositions(context, positions)
            pose = plant.CalcRelativeTransform(context, plant.world_frame(), hand)
            measured_poses.append(pose)
            angle_error = np.arccos(
                np.clip(
                    (np.trace(target_rotation.T @ pose.rotation().matrix()) - 1.0)
                    / 2.0,
                    -1.0,
                    1.0,
                )
            )
            self.assertLessEqual(angle_error, 0.02 + 1e-8)

        np.testing.assert_allclose(measured_poses[0].translation(), START, atol=1e-6)
        np.testing.assert_allclose(measured_poses[-1].translation(), FINAL, atol=1e-6)
        for pose in measured_poses[1:-1]:
            self.assertGreaterEqual(pose.translation()[2], FINAL[2] + 0.005 - 1e-6)

        max_tilt = 0.0
        min_interpolated_height = float("inf")
        for start, finish in zip(path[:-1], path[1:]):
            for progress in np.linspace(0.0, 1.0, 21):
                positions[arm_indices] = (1.0 - progress) * start + progress * finish
                plant.SetPositions(context, positions)
                pose = plant.CalcRelativeTransform(context, plant.world_frame(), hand)
                min_interpolated_height = min(
                    min_interpolated_height, float(pose.translation()[2])
                )
                rotation = pose.rotation().matrix()
                angle_error = np.arccos(
                    np.clip((np.trace(target_rotation.T @ rotation) - 1.0) / 2.0, -1.0, 1.0)
                )
                max_tilt = max(max_tilt, angle_error)
        self.assertLess(max_tilt, np.deg2rad(2.0))
        self.assertGreaterEqual(min_interpolated_height, FINAL[2] - 1e-6)


if __name__ == "__main__":
    unittest.main()
