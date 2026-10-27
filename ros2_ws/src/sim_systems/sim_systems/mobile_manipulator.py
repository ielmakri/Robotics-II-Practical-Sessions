#!/usr/bin/env python3
"""Session 3 composed Husky + Panda scene.

The SDF file composes the supplied Husky and Panda descriptions and fixes the
Panda base to the Husky top plate. Students extend this explicit scene in their
project; the starter includes the hand geometry and finger joints but does not
conceal a navigation or gripper command interface.
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
    args = simulation_arguments(
        "Husky and Panda arm-and-hand dynamics with ROS 2 visualization."
    )

    root = repository_root()
    scene_file = root / "models/descriptions/robots/mobile_manipulator/husky_panda.sdf"
    floor_file = root / "models/objects_scenes/floor.sdf"
    if not scene_file.is_file() or not floor_file.is_file():
        raise FileNotFoundError("Combined scene or floor model is missing from models/")

    builder = DiagramBuilder()
    drake_ros.core.init()
    ros_interface = builder.AddSystem(RosInterfaceSystem("mobile_manipulator"))
    ClockSystem.AddToBuilder(builder, ros_interface.get_ros_interface())
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001),
        builder,
    )
    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(root / "models"))
    model_parser.AddModels(file_name=str(scene_file))
    model_parser.AddModels(file_name=str(floor_file))
    plant.Finalize()

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
