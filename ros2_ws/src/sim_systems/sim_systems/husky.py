#!/usr/bin/env python3
"""Session 3 Husky scene: model loading, contact geometry and visualization.

The scene intentionally stops at a verified multibody model. A ROS command
interface and odometry are project extensions described in the project guide.
"""

import drake_ros.core
from drake_ros.core import ClockSystem, RosInterfaceSystem
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType

from sim_systems.course_utils import repository_root, run_simulator, simulation_arguments


def main():
    args = simulation_arguments("Husky dynamics with ROS 2 visualization.")

    root = repository_root()
    husky_file = root / "models/descriptions/robots/mobile/clearpath_husky/urdf/husky.urdf"
    floor_file = root / "models/objects_scenes/floor.sdf"
    if not husky_file.is_file() or not floor_file.is_file():
        raise FileNotFoundError("Husky or floor model is missing from models/")

    builder = DiagramBuilder()
    drake_ros.core.init()
    ros_interface = builder.AddSystem(RosInterfaceSystem("husky"))
    ClockSystem.AddToBuilder(builder, ros_interface.get_ros_interface())
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001),
        builder,
    )

    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(root / "models"))
    (husky,) = model_parser.AddModels(file_name=str(husky_file))
    model_parser.AddModels(file_name=str(floor_file))
    plant.Finalize()

    # The URDF's base frame is at the chassis center. Lift it by the wheel
    # radius/offset so the initial pose is not embedded in the floor.
    from pydrake.math import RigidTransform

    husky_base = plant.GetBodyByName("base_link", husky)
    set_default_floating_pose = getattr(
        plant, "SetDefaultFloatingBaseBodyPose", None
    )
    if set_default_floating_pose is None:
        # Drake used this name before renaming the API for floating bases.
        set_default_floating_pose = getattr(plant, "SetDefaultFreeBodyPose", None)
    if set_default_floating_pose is None:
        raise RuntimeError(
            "This Husky launch requires Drake's default floating-base pose API."
        )
    set_default_floating_pose(
        husky_base,
        RigidTransform(p=[0.0, 0.0, 0.13228]),
    )

    viz_period = 1.0 / 32.0
    tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            ros_interface.get_ros_interface(),
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_period,
            ),
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(),
        tf_broadcaster.get_graph_query_input_port(),
    )
    visualizer = builder.AddSystem(
        RvizVisualizer(
            ros_interface.get_ros_interface(),
            params=RvizVisualizerParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=viz_period,
            ),
        )
    )
    builder.Connect(scene_graph.get_query_output_port(), visualizer.get_graph_query_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(args.realtime_rate)
    simulator.Initialize()
    run_simulator(simulator, args.simulation_sec)


if __name__ == "__main__":
    main()
