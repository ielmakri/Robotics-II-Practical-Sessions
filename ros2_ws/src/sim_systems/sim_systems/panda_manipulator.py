#!/usr/bin/env python3
"""Session 3 baseline: Panda dynamics, Drake scene and ROS visualization.

This file is intentionally explicit. Read it top-to-bottom in Session 3 before
factoring code into helpers. An internal controller holds the Panda near its
home pose while this baseline publishes state. The controlled variant adds a
ROS subscriber and accepts external joint targets.
"""

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import drake_ros.core
from drake_ros.core import ClockSystem, PySerializer, RosInterfaceSystem, RosPublisherSystem
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams
from pydrake.common.value import AbstractValue
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import JointStiffnessController
from pydrake.systems.framework import DiagramBuilder, LeafSystem, TriggerType
from pydrake.systems.primitives import Adder, ConstantVectorSource

from sim_systems.course_utils import repository_root, run_simulator, simulation_arguments
from sim_systems.panda_model import (
    PANDA_HOME,
    PANDA_KD,
    PANDA_KP,
    PANDA_JOINT_NAMES,
    panda_hand_urdf,
    validate_panda_model,
)
from sim_systems.mobile_manipulator_adapters import PandaGravityCompensationSystem
from sim_systems.ros_adapters import JointStateMessageSystem, VectorToFloatArraySystem


def main():
    args = simulation_arguments(
        "Panda arm-and-hand dynamics with ROS 2 visualization."
    )

    root = repository_root()
    panda_file = panda_hand_urdf(root)
    if not panda_file.is_file():
        raise FileNotFoundError(f"Panda model not found: {panda_file}")

    # 1. A DiagramBuilder is the directed graph of Drake systems.
    builder = DiagramBuilder()
    drake_ros.core.init()

    ros_interface = builder.AddSystem(RosInterfaceSystem("panda_manipulator"))
    ClockSystem.AddToBuilder(builder, ros_interface.get_ros_interface())

    # 2. AddMultibodyPlant contributes the continuous/discrete dynamics and a
    # SceneGraph contributes geometry/query ports. Loading a model and welding
    # frames must happen before Finalize().
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001),
        builder,
    )
    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(root / "models"))
    (panda,) = model_parser.AddModels(file_name=str(panda_file))
    panda_link0 = plant.GetFrameByName("panda_link0", panda)
    plant.WeldFrames(plant.world_frame(), panda_link0)
    plant.Finalize()
    validate_panda_model(plant, panda)
    plant.SetDefaultPositions(panda, PANDA_HOME)

    # SetDefaultPositions chooses only the initial configuration; it does not
    # hold an articulated robot against gravity. Use an internal constant
    # home-pose target so the introductory visualization stays still without
    # adding a ROS command topic.
    home_state = builder.AddSystem(
        ConstantVectorSource(
            PANDA_HOME.tolist() + [0.0] * len(PANDA_HOME)
        )
    )
    hold_controller = builder.AddSystem(
        JointStiffnessController(plant, PANDA_KP, PANDA_KD)
    )
    builder.Connect(
        plant.get_state_output_port(panda),
        hold_controller.get_input_port_estimated_state(),
    )
    builder.Connect(
        home_state.get_output_port(),
        hold_controller.get_input_port_desired_state(),
    )
    gravity_compensation = builder.AddSystem(
        PandaGravityCompensationSystem(
            plant, panda, plant.get_state_output_port().size()
        )
    )
    builder.Connect(
        plant.get_state_output_port(), gravity_compensation.get_input_port()
    )
    compensated_actuation = builder.AddSystem(
        Adder(2, plant.num_actuators())
    )
    builder.Connect(
        hold_controller.get_output_port_actuation(),
        compensated_actuation.get_input_port(0),
    )
    builder.Connect(
        gravity_compensation.get_output_port(),
        compensated_actuation.get_input_port(1),
    )
    builder.Connect(
        compensated_actuation.get_output_port(),
        plant.get_actuation_input_port(panda),
    )

    # 3. A constant joint-space controller holds the home pose. Drake-ROS
    # visualization converts SceneGraph queries into ROS TF and
    # visualization messages. It does not replace the simulator or RViz.
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

    joint_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )
    joint_serializer = PySerializer(JointState)
    acceleration_serializer = PySerializer(Float64MultiArray)
    joint_state_message = builder.AddSystem(JointStateMessageSystem(PANDA_JOINT_NAMES))
    acceleration_message = builder.AddSystem(
        VectorToFloatArraySystem(len(PANDA_JOINT_NAMES))
    )
    joint_state_pub = builder.AddSystem(
        RosPublisherSystem(
            joint_serializer,
            "sim_sys/joint_states",
            joint_qos,
            ros_interface.get_ros_interface(),
            {TriggerType.kPeriodic},
            0.02,
        )
    )
    acceleration_pub = builder.AddSystem(
        RosPublisherSystem(
            acceleration_serializer,
            "sim_sys/joint_acceleration",
            joint_qos,
            ros_interface.get_ros_interface(),
            {TriggerType.kPeriodic},
            0.02,
        )
    )
    builder.Connect(plant.get_state_output_port(panda), joint_state_message.get_input_port(0))
    builder.Connect(plant.get_net_actuation_output_port(), joint_state_message.get_input_port(1))
    builder.Connect(joint_state_message.get_output_port(), joint_state_pub.get_input_port())
    builder.Connect(
        plant.get_generalized_acceleration_output_port(),
        acceleration_message.get_input_port(),
    )
    builder.Connect(acceleration_message.get_output_port(), acceleration_pub.get_input_port())

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(args.realtime_rate)
    simulator.Initialize()
    run_simulator(simulator, args.simulation_sec)


if __name__ == "__main__":
    main()
