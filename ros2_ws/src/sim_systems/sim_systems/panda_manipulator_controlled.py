#!/usr/bin/env python3
"""ROS-commanded Panda simulation used at the end of Session 1 and in Session 3."""

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

import drake_ros.core
from drake_ros.core import (
    ClockSystem,
    PySerializer,
    RosInterfaceSystem,
    RosPublisherSystem,
    RosSubscriberSystem,
)
from drake_ros.tf2 import SceneTfBroadcasterParams, SceneTfBroadcasterSystem
from drake_ros.viz import RvizVisualizer, RvizVisualizerParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import JointStiffnessController
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import Adder

from sim_systems.course_utils import repository_root, run_simulator, simulation_arguments
from sim_systems.panda_model import (
    PANDA_ARM_JOINT_NAMES,
    PANDA_HOME,
    PANDA_JOINT_NAMES,
    PANDA_KD,
    PANDA_KP,
    PANDA_LOWER_LIMITS,
    PANDA_UPPER_LIMITS,
    panda_hand_urdf,
    validate_panda_model,
)
from sim_systems.mobile_manipulator_adapters import PandaGravityCompensationSystem
from sim_systems.ros_adapters import (
    JointStateMessageSystem,
    JointTrajectoryToStateSystem,
    DesiredPositionMessageSystem,
    VectorToFloatArraySystem,
)


def main():
    args = simulation_arguments("ROS-commanded Panda arm-and-hand simulation.")

    root = repository_root()
    panda_file = panda_hand_urdf(root)
    if not panda_file.is_file():
        raise FileNotFoundError(f"Panda model not found: {panda_file}")

    builder = DiagramBuilder()
    drake_ros.core.init()
    ros_interface = builder.AddSystem(
        RosInterfaceSystem("panda_manipulator_controlled")
    )
    ClockSystem.AddToBuilder(builder, ros_interface.get_ros_interface())

    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001),
        builder,
    )
    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(root / "models"))
    (panda,) = model_parser.AddModels(file_name=str(panda_file))
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("panda_link0", panda),
    )
    plant.Finalize()
    validate_panda_model(plant, panda)
    plant.SetDefaultPositions(panda, PANDA_HOME)

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

    controller = builder.AddSystem(
        JointStiffnessController(plant, PANDA_KP, PANDA_KD)
    )
    builder.Connect(
        plant.get_state_output_port(panda),
        controller.get_input_port_estimated_state(),
    )
    gravity_compensation = builder.AddSystem(
        PandaGravityCompensationSystem(
            plant, panda, plant.get_state_output_port().size()
        )
    )
    builder.Connect(
        plant.get_state_output_port(), gravity_compensation.get_input_port()
    )
    compensated_actuation = builder.AddSystem(Adder(2, plant.num_actuators()))
    builder.Connect(
        controller.get_output_port_actuation(),
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

    joint_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )
    trajectory_serializer = PySerializer(JointTrajectory)
    reference_subscriber = builder.AddSystem(
        RosSubscriberSystem(
            trajectory_serializer,
            "panda/joint_trajectory",
            joint_qos,
            ros_interface.get_ros_interface(),
        )
    )
    reference_state = builder.AddSystem(
        JointTrajectoryToStateSystem(
            PANDA_JOINT_NAMES,
            PANDA_HOME,
            PANDA_LOWER_LIMITS,
            PANDA_UPPER_LIMITS,
            commanded_joint_names=PANDA_ARM_JOINT_NAMES,
        )
    )
    builder.Connect(reference_subscriber.get_output_port(), reference_state.get_input_port(0))
    builder.Connect(reference_state.get_output_port(), controller.get_input_port_desired_state())

    # Publish the validated desired positions as a simple plotting signal. The
    # command topic remains a standard JointTrajectory; this is a diagnostic
    # mirror, not a second command interface.
    desired_serializer = PySerializer(Float64MultiArray)
    joint_serializer = PySerializer(JointState)
    acceleration_serializer = PySerializer(Float64MultiArray)
    desired_position_message = builder.AddSystem(
        DesiredPositionMessageSystem(len(PANDA_JOINT_NAMES))
    )
    desired_position_pub = builder.AddSystem(
        RosPublisherSystem(
            desired_serializer,
            "sim_sys/desired_positions",
            joint_qos,
            ros_interface.get_ros_interface(),
            {TriggerType.kPeriodic},
            0.02,
        )
    )
    builder.Connect(reference_state.get_output_port(), desired_position_message.get_input_port(0))
    builder.Connect(
        desired_position_message.get_output_port(),
        desired_position_pub.get_input_port(),
    )

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
