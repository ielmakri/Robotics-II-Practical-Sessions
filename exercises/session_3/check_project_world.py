#!/usr/bin/env python3
"""Parse one course project SDF and report its Drake model instances."""

import argparse
from pathlib import Path

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.multibody.tree import JointIndex, ModelInstanceIndex
from pydrake.systems.framework import DiagramBuilder


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PROJECT_SCENES = {
    "two_husky_corridor.sdf": {"husky_a", "husky_b"},
    "dual_panda_inspection.sdf": {"panda_a", "panda_b"},
    "husky_panda_inspection.sdf": {"husky_runner", "panda_inspector"},
    "husky_panda_pick_place.sdf": {"husky_runner", "panda_inspector"},
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scene",
        help=(
            "SDF filename in models/team_projects/. For an unlisted team SDF, "
            "the checker expects husky_runner and panda_inspector."
        ),
    )
    args = parser.parse_args()

    scene_root = (REPOSITORY_ROOT / "models" / "team_projects").resolve()
    scene_path = Path(args.scene).expanduser()
    if not scene_path.is_absolute():
        scene_path = scene_root / scene_path
    scene_path = scene_path.resolve()
    if scene_root not in scene_path.parents:
        parser.error("The SDF must be inside models/team_projects/.")
    if not scene_path.is_file():
        parser.error(
            f"SDF file does not exist: {scene_path}. For a team scene, create "
            "it first by copying models/team_projects/husky_panda_pick_place.sdf."
        )

    expected_names = PROJECT_SCENES.get(
        scene_path.name, {"husky_runner", "panda_inspector"}
    )
    builder = DiagramBuilder()
    plant, _scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001), builder
    )
    model_parser = Parser(plant)
    model_parser.package_map().Add(
        "robotics_ii_models", str(REPOSITORY_ROOT / "models")
    )
    model_parser.AddModels(file_name=str(scene_path))
    plant.Finalize()

    # Enumerate the finalized plant with the index type and count APIs.
    instances = {
        plant.GetModelInstanceName(ModelInstanceIndex(index)):
        ModelInstanceIndex(index)
        for index in range(plant.num_model_instances())
    }
    model_names = set(instances)
    present_expected_names = {
        expected
        for expected in expected_names
        if any(
            actual == expected or actual.endswith(f"::{expected}")
            for actual in model_names
        )
    }
    missing = expected_names - present_expected_names
    if missing:
        raise RuntimeError(
            "Project SDF did not create the expected robot model instances: "
            + ", ".join(sorted(missing))
            + ". Model instances found in the plant: "
            + (", ".join(sorted(model_names)) or "(none)")
        )

    if scene_path.name == "husky_panda_pick_place.sdf":
        def find_instance(requested_name):
            matches = [
                model_instance
                for name, model_instance in instances.items()
                if name == requested_name or name.endswith(f"::{requested_name}")
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    f"Expected one model instance {requested_name!r}; "
                    f"found {len(matches)}."
                )
            return matches[0]

        husky = find_instance("husky_runner")
        panda = find_instance("panda_inspector")
        mounts = [
            plant.get_joint(JointIndex(index))
            for index in range(plant.num_joints())
            if plant.get_joint(JointIndex(index)).name() == "panda_mount_joint"
        ]
        if len(mounts) != 1:
            raise RuntimeError(
                "Expected exactly one SDF joint named 'panda_mount_joint'; "
                f"found {len(mounts)}."
            )
        mount = mounts[0]
        if mount.num_positions() != 0:
            raise RuntimeError("The Panda mount must be a fixed, zero-DOF joint.")

        # Check the actual bodies connected by Drake's joint and their
        # resolved poses. A fixed joint can still preserve a bad include pose,
        # so checking only its endpoints would miss a visually floating arm.
        top_plate_body = plant.GetBodyByName("top_plate_link", husky)
        panda_base_body = plant.GetBodyByName("panda_link0", panda)
        if mount.parent_body().index() != top_plate_body.index():
            raise RuntimeError(
                "Drake attached 'panda_mount_joint' to the wrong parent body: "
                f"{mount.parent_body().name()!r}; expected 'top_plate_link'."
            )
        if mount.child_body().index() != panda_base_body.index():
            raise RuntimeError(
                "Drake attached 'panda_mount_joint' to the wrong child body: "
                f"{mount.child_body().name()!r}; expected 'panda_link0'."
            )
        context = plant.CreateDefaultContext()
        X_top_panda = plant.CalcRelativeTransform(
            context, top_plate_body.body_frame(), panda_base_body.body_frame()
        )
        expected_translation = (0.0, 0.0, 0.00635)
        actual_translation = X_top_panda.translation()
        if any(
            abs(float(actual) - expected) > 1e-6
            for actual, expected in zip(actual_translation, expected_translation)
        ):
            raise RuntimeError(
                "Panda base is not sitting on the Husky top plate. Expected "
                f"relative translation {expected_translation} m, got "
                f"{tuple(float(value) for value in actual_translation)} m."
            )
        print(
            "Verified mount: Drake fixes Panda body 'panda_link0' to Husky "
            "body 'top_plate_link', with the Panda base 6.35 mm above the "
            "plate-link origin."
        )

    print(f"Loaded: {scene_path.relative_to(REPOSITORY_ROOT)}")
    print(f"Positions: {plant.num_positions()}")
    print(f"Actuated degrees of freedom: {plant.num_actuated_dofs()}")
    print("Model instances:")
    for name in sorted(model_names):
        print(f"  {name}")
    print("PASS: both robot model instances are in one finalized plant.")


if __name__ == "__main__":
    main()
