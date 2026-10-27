#!/usr/bin/env python3
"""Solve one Panda hand pose in the supplied mounted-Husky SDF scene."""

import argparse
from pathlib import Path

import numpy as np
from pydrake.math import RollPitchYaw, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.solvers import Solve
from pydrake.multibody.inverse_kinematics import InverseKinematics

from sim_systems.panda_model import (
    PANDA_ARM_JOINT_NAMES,
    PANDA_HOME,
    PANDA_LOWER_LIMITS,
    PANDA_UPPER_LIMITS,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE = ROOT / "models/team_projects/husky_panda_pick_place.sdf"
ALTERNATE_IK_SEEDS = (
    np.array([1.0, 0.0, -1.0, -2.5, 2.0, 2.5, -2.0]),
    np.array([1.0, -0.4, -1.0, -2.8, 2.0, 3.4, -2.0]),
)
JOINT_LIMIT_FALLBACK_MARGIN = 0.05


def _find_instance(plant, requested_name):
    matches = []
    for index in range(plant.num_model_instances()):
        model = ModelInstanceIndex(index)
        name = plant.GetModelInstanceName(model)
        if name == requested_name or name.endswith(f"::{requested_name}"):
            matches.append(model)
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one model instance ending in {requested_name!r}; "
            f"found {len(matches)}."
        )
    return matches[0]


def _solve_from_seed(plant, panda, panda_arm_indices, target_xyz, target_rpy, seed):
    context = plant.CreateDefaultContext()
    q_nominal = plant.GetPositions(context).copy()
    q_nominal[panda_arm_indices] = seed
    plant.SetPositions(context, q_nominal)
    q_nominal = plant.GetPositions(context)

    ik = InverseKinematics(plant, context)
    q = ik.q()
    hand = plant.GetFrameByName("panda_hand", panda)
    pinned_indices = [
        index for index in range(plant.num_positions()) if index not in panda_arm_indices
    ]
    if pinned_indices:
        ik.prog().AddBoundingBoxConstraint(
            q_nominal[pinned_indices],
            q_nominal[pinned_indices],
            q[pinned_indices],
        )

    target_position = np.asarray(target_xyz, dtype=float).reshape((3, 1))
    ik.AddPositionConstraint(
        frameB=hand,
        p_BQ=np.zeros((3, 1)),
        frameA=plant.world_frame(),
        p_AQ_lower=target_position,
        p_AQ_upper=target_position,
    )
    target_rotation = RotationMatrix(RollPitchYaw(*target_rpy))
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=target_rotation,
        frameBbar=hand,
        R_BbarB=RotationMatrix(),
        theta_bound=0.02,
    )
    ik.prog().AddQuadraticErrorCost(
        np.eye(plant.num_positions()), q_nominal, q
    )
    result = Solve(ik.prog(), q_nominal)
    if not result.is_success():
        return None
    solution = result.GetSolution(q)
    return [float(solution[index]) for index in panda_arm_indices]


def _joint_limit_margin(solution):
    lower = PANDA_LOWER_LIMITS[: len(PANDA_ARM_JOINT_NAMES)]
    upper = PANDA_UPPER_LIMITS[: len(PANDA_ARM_JOINT_NAMES)]
    values = np.asarray(solution, dtype=float)
    return float(np.min(np.minimum(values - lower, upper - values)))


def _solve_hand_pose(plant, panda, target_xyz, target_rpy):
    plant.SetDefaultPositions(panda, PANDA_HOME)
    panda_arm_indices = [
        plant.GetJointByName(name, panda).position_start()
        for name in PANDA_ARM_JOINT_NAMES
    ]

    home_solution = _solve_from_seed(
        plant,
        panda,
        panda_arm_indices,
        target_xyz,
        target_rpy,
        PANDA_HOME[: len(PANDA_ARM_JOINT_NAMES)],
    )
    if (
        home_solution is not None
        and _joint_limit_margin(home_solution) >= JOINT_LIMIT_FALLBACK_MARGIN
    ):
        return home_solution

    # Preserve the home-seeded result as a fallback, then compare alternative
    # elbow postures and return the one with the largest joint-limit margin.
    solutions = [home_solution] if home_solution is not None else []
    for seed in ALTERNATE_IK_SEEDS:
        solution = _solve_from_seed(
            plant,
            panda,
            panda_arm_indices,
            target_xyz,
            target_rpy,
            seed,
        )
        if solution is not None:
            solutions.append(solution)

    if not solutions:
        raise RuntimeError(
            "Drake could not reach this hand pose. Choose a nearer point or "
            "a different approach orientation."
        )
    return max(solutions, key=_joint_limit_margin)


def _solve_vertical_placement_path(
    plant,
    panda,
    start_xyz=(-0.20, -0.55, 0.9475),
    final_xyz=(-0.55, -0.55, 0.51913),
    segments=32,
    clearance=0.005,
):
    """Build joint waypoints that keep panda_hand vertical during placement.

    Intermediate IK samples preserve the starting-to-final joint posture
    trend, constrain hand orientation, and keep the hand above the final
    release height. The final sample reaches the exact placement pose.
    """
    if not isinstance(segments, int) or segments < 2:
        raise ValueError("segments must be an integer of at least 2.")
    if not np.isfinite(clearance) or clearance < 0.0:
        raise ValueError("clearance must be finite and nonnegative.")

    start_position = np.asarray(start_xyz, dtype=float)
    final_position = np.asarray(final_xyz, dtype=float)
    if start_position.shape != (3,) or final_position.shape != (3,):
        raise ValueError("start_xyz and final_xyz must each contain three values.")

    plant.SetDefaultPositions(panda, PANDA_HOME)
    target_rpy = (np.pi, 0.0, 0.0)
    start = np.asarray(_solve_hand_pose(plant, panda, start_position, target_rpy))
    final = np.asarray(_solve_hand_pose(plant, panda, final_position, target_rpy))
    arm_indices = [
        plant.GetJointByName(name, panda).position_start()
        for name in PANDA_ARM_JOINT_NAMES
    ]
    lower = PANDA_LOWER_LIMITS[: len(PANDA_ARM_JOINT_NAMES)] + 0.05
    upper = PANDA_UPPER_LIMITS[: len(PANDA_ARM_JOINT_NAMES)] - 0.05
    target_rotation = RotationMatrix(RollPitchYaw(*target_rpy))
    path = [start.tolist()]

    for step in range(1, segments + 1):
        progress = step / segments
        nominal_arm = (1.0 - progress) * start + progress * final
        context = plant.CreateDefaultContext()
        q_nominal = plant.GetPositions(context).copy()
        q_nominal[arm_indices] = nominal_arm
        plant.SetPositions(context, q_nominal)
        q_nominal = plant.GetPositions(context)

        ik = InverseKinematics(plant, context)
        q = ik.q()
        pinned_indices = [
            index for index in range(plant.num_positions())
            if index not in arm_indices
        ]
        if pinned_indices:
            ik.prog().AddBoundingBoxConstraint(
                q_nominal[pinned_indices],
                q_nominal[pinned_indices],
                q[pinned_indices],
            )
        ik.prog().AddBoundingBoxConstraint(lower, upper, q[arm_indices])
        hand = plant.GetFrameByName("panda_hand", panda)
        ik.AddOrientationConstraint(
            frameAbar=plant.world_frame(),
            R_AbarA=target_rotation,
            frameBbar=hand,
            R_BbarB=RotationMatrix(),
            theta_bound=0.002,
        )

        if step == segments:
            target = final_position.reshape((3, 1))
            position_lower = target
            position_upper = target
        else:
            position_lower = np.array(
                [-np.inf, -np.inf, final_position[2] + clearance]
            ).reshape((3, 1))
            position_upper = np.full((3, 1), np.inf)
        ik.AddPositionConstraint(
            frameB=hand,
            p_BQ=np.zeros((3, 1)),
            frameA=plant.world_frame(),
            p_AQ_lower=position_lower,
            p_AQ_upper=position_upper,
        )
        ik.prog().AddQuadraticErrorCost(
            np.eye(plant.num_positions()), q_nominal, q
        )
        result = Solve(ik.prog(), q_nominal)
        if not result.is_success():
            raise RuntimeError(
                f"Drake could not solve vertical placement waypoint "
                f"{step}/{segments}."
            )
        solution = result.GetSolution(q)[arm_indices]
        if _joint_limit_margin(solution) < 0.05 - 1e-8:
            raise RuntimeError(
                f"Vertical placement waypoint {step}/{segments} is too close "
                "to a Panda joint limit."
            )
        path.append([float(value) for value in solution])

    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("x", type=float, help="world X position of panda_hand in metres")
    parser.add_argument("y", type=float, help="world Y position of panda_hand in metres")
    parser.add_argument("z", type=float, help="world Z position of panda_hand in metres")
    parser.add_argument("--roll", type=float, default=np.pi, help="hand roll in radians")
    parser.add_argument("--pitch", type=float, default=0.0, help="hand pitch in radians")
    parser.add_argument("--yaw", type=float, default=0.0, help="hand yaw in radians")
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    args = parser.parse_args()

    scene = args.scene.expanduser().resolve()
    if not scene.is_file():
        parser.error(f"SDF scene does not exist: {scene}")

    plant = MultibodyPlant(time_step=0.001)
    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(ROOT / "models"))
    model_parser.AddModels(file_name=str(scene))
    plant.Finalize()
    panda = _find_instance(plant, "panda_inspector")
    solution = _solve_hand_pose(
        plant,
        panda,
        (args.x, args.y, args.z),
        (args.roll, args.pitch, args.yaw),
    )
    print("Panda arm target in radians, joint order panda_joint1..7:")
    print(", ".join(f"{value:.6f}" for value in solution))


if __name__ == "__main__":
    main()
