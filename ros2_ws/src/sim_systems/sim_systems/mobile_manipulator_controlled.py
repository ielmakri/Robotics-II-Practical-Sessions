#!/usr/bin/env python3
"""Commandable Husky-mounted Panda mobile manipulator in one Drake plant."""

from pathlib import Path

import drake_ros.core
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

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
from pydrake.multibody.tree import ModelInstanceIndex, PdControllerGains
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, TriggerType
from pydrake.systems.primitives import Integrator, Multiplexer

from sim_systems.course_utils import (
    repository_root,
    run_simulator,
    simulation_arguments,
)
from sim_systems.mobile_manipulator_adapters import (
    GripperPositionSetpointSystem,
    HuskyOdometryMessageSystem,
    LatchedJointTrajectorySetpointSystem,
    PandaGripperStateMergeSystem,
    PandaGravityCompensationSystem,
    TwistToWheelRates,
    VectorReorderSystem,
)
from sim_systems.panda_model import (
    PANDA_ARM_JOINT_NAMES,
    PANDA_HOME,
    PANDA_JOINT_NAMES,
    PANDA_KD,
    PANDA_KP,
    PANDA_LOWER_LIMITS,
    PANDA_UPPER_LIMITS,
    validate_panda_model,
)
from sim_systems.ros_adapters import JointStateMessageSystem


HUSKY_WHEEL_JOINT_NAMES = [
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
]
HUSKY_WHEEL_ACTUATOR_NAMES = [
    "front_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_left_wheel_motor",
    "rear_right_wheel_motor",
]
PANDA_ACTUATOR_NAMES = [f"panda_motor{i}" for i in range(1, 8)] + [
    "panda_finger_joint1",
    "panda_finger_joint2",
]
PANDA_FEEDBACK_PERIOD = 0.02
SCENE_MARKER_PERIOD = 1.0 / 32.0


def _add_scene_argument(parser):
    parser.add_argument(
        "--scene",
        default="husky_panda_pick_place.sdf",
        help=(
            "SDF filename in models/team_projects/. Keep the model instance "
            "names husky_runner and panda_inspector."
        ),
    )


def _set_actuator_gains(plant, model_instance, actuator_name, kp, kd):
    if not plant.HasJointActuatorNamed(actuator_name, model_instance):
        raise RuntimeError(
            f"Required actuator {actuator_name!r} is missing from model "
            f"{plant.GetModelInstanceName(model_instance)!r}."
        )
    actuator = plant.GetJointActuatorByName(actuator_name, model_instance)
    if actuator.num_inputs() != 1:
        raise RuntimeError(f"Actuator {actuator_name!r} must have one input.")
    actuator.set_controller_gains(PdControllerGains(p=float(kp), d=float(kd)))


def _actuator_joint_names(plant, model_instance):
    return [
        plant.get_joint_actuator(index).joint().name()
        for index in plant.GetJointActuatorIndices(model_instance)
    ]


def _find_model_instance(instances, required_name):
    """Find a root-level or nested SDF instance by its declared include name."""
    matches = [
        (name, model_instance)
        for name, model_instance in instances.items()
        if name == required_name or name.endswith(f"::{required_name}")
    ]
    if len(matches) != 1:
        found = ", ".join(sorted(instances)) or "(none)"
        raise RuntimeError(
            f"Expected exactly one model instance named {required_name!r} "
            f"(including nested SDF names); found {len(matches)}. "
            f"Model instances: {found}"
        )
    return matches[0][1]


def main():
    args = simulation_arguments(
        "ROS-commanded Husky-mounted Panda mobile-manipulator simulation.",
        configure_parser=_add_scene_argument,
    )
    root = repository_root()
    scene_file = Path(args.scene).expanduser()
    if not scene_file.is_absolute():
        scene_file = root / "models" / "team_projects" / scene_file
    if not scene_file.is_file():
        raise FileNotFoundError(f"Project SDF not found: {scene_file}")

    builder = DiagramBuilder()
    drake_ros.core.init()
    ros_interface = builder.AddSystem(
        RosInterfaceSystem("mobile_manipulator_controlled")
    )
    ClockSystem.AddToBuilder(builder, ros_interface.get_ros_interface())

    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.001), builder
    )
    model_parser = Parser(plant)
    model_parser.package_map().Add("robotics_ii_models", str(root / "models"))
    model_parser.AddModels(file_name=str(scene_file))
    instances = {}
    for index in range(plant.num_model_instances()):
        model_instance = ModelInstanceIndex(index)
        instances[plant.GetModelInstanceName(model_instance)] = model_instance
    husky = _find_model_instance(instances, "husky_runner")
    panda = _find_model_instance(instances, "panda_inspector")

    # Drake models the URDF transmissions as joint actuators. The built-in
    # discrete PD actuators track wheel and Panda joint setpoints.
    for actuator_name in HUSKY_WHEEL_ACTUATOR_NAMES:
        _set_actuator_gains(plant, husky, actuator_name, kp=35.0, kd=5.0)
    for joint_index, actuator_name in enumerate(PANDA_ACTUATOR_NAMES):
        _set_actuator_gains(
            plant,
            panda,
            actuator_name,
            kp=PANDA_KP[joint_index],
            kd=PANDA_KD[joint_index],
        )

    plant.Finalize()
    validate_panda_model(plant, panda)
    plant.SetDefaultPositions(panda, PANDA_HOME)

    husky_actuator_joint_names = _actuator_joint_names(plant, husky)
    if len(husky_actuator_joint_names) != 4 or set(husky_actuator_joint_names) != set(
        HUSKY_WHEEL_JOINT_NAMES
    ):
        raise RuntimeError(
            "The Husky model must expose exactly the four expected wheel "
            f"actuators; found {husky_actuator_joint_names}."
        )
    panda_actuator_joint_names = _actuator_joint_names(plant, panda)
    if len(panda_actuator_joint_names) != len(PANDA_JOINT_NAMES) or set(
        panda_actuator_joint_names
    ) != set(PANDA_JOINT_NAMES):
        raise RuntimeError(
            "The Panda actuator joints do not match the course command contract: "
            f"{panda_actuator_joint_names}."
        )
    panda_position_names = list(
        plant.GetPositionNames(panda, always_add_suffix=False)
    )
    panda_measured_position_indices = [
        panda_position_names.index(name) for name in panda_actuator_joint_names
    ]

    # Husky: Twist -> wheel-rate targets -> integrated wheel positions, with
    # odometry computed from the measured floating-base state.
    husky_command_serializer = PySerializer(Twist)
    husky_command = builder.AddSystem(
        RosSubscriberSystem(
            husky_command_serializer,
            "husky_runner/cmd_vel_safe",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
            ros_interface.get_ros_interface(),
        )
    )
    wheel_rates = builder.AddSystem(
        TwistToWheelRates(husky_actuator_joint_names)
    )
    wheel_position_integrator = builder.AddSystem(
        Integrator(len(husky_actuator_joint_names))
    )
    husky_desired_state = builder.AddSystem(
        Multiplexer(
            [len(husky_actuator_joint_names), len(husky_actuator_joint_names)]
        )
    )
    builder.Connect(husky_command.get_output_port(), wheel_rates.get_input_port())
    builder.Connect(
        wheel_rates.get_output_port(), wheel_position_integrator.get_input_port()
    )
    builder.Connect(
        wheel_position_integrator.get_output_port(),
        husky_desired_state.get_input_port(0),
    )
    builder.Connect(wheel_rates.get_output_port(), husky_desired_state.get_input_port(1))
    builder.Connect(
        husky_desired_state.get_output_port(),
        plant.get_desired_state_input_port(husky),
    )

    odometry = builder.AddSystem(
        HuskyOdometryMessageSystem(
            plant,
            plant.GetBodyByName("base_link", husky),
            plant.get_state_output_port().size(),
        )
    )
    husky_odometry_serializer = PySerializer(Odometry)
    odometry_publisher = builder.AddSystem(
        RosPublisherSystem(
            husky_odometry_serializer,
            "husky_runner/odom",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
            ros_interface.get_ros_interface(),
            {TriggerType.kPeriodic},
            PANDA_FEEDBACK_PERIOD,
        )
    )
    builder.Connect(plant.get_state_output_port(), odometry.get_input_port())
    builder.Connect(odometry.get_output_port(), odometry_publisher.get_input_port())

    # Panda: smooth named arm goals start from the measured joint positions; a
    # separate bounded command controls the gripper fingers.
    actuator_name_to_model_index = {
        name: index for index, name in enumerate(PANDA_JOINT_NAMES)
    }
    panda_default = np.asarray(
        [
            PANDA_HOME[actuator_name_to_model_index[name]]
            for name in panda_actuator_joint_names
        ]
    )
    panda_lower = np.asarray(
        [
            PANDA_LOWER_LIMITS[actuator_name_to_model_index[name]]
            for name in panda_actuator_joint_names
        ]
    )
    panda_upper = np.asarray(
        [
            PANDA_UPPER_LIMITS[actuator_name_to_model_index[name]]
            for name in panda_actuator_joint_names
        ]
    )
    panda_trajectory_serializer = PySerializer(JointTrajectory)
    panda_trajectory = builder.AddSystem(
        RosSubscriberSystem(
            panda_trajectory_serializer,
            "panda_inspector/joint_trajectory",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
            ros_interface.get_ros_interface(),
        )
    )
    panda_reference = builder.AddSystem(
        LatchedJointTrajectorySetpointSystem(
            panda_actuator_joint_names,
            panda_default,
            panda_lower,
            panda_upper,
            commanded_joint_names=PANDA_ARM_JOINT_NAMES,
            measured_position_indices=panda_measured_position_indices,
            update_period=PANDA_FEEDBACK_PERIOD,
        )
    )
    builder.Connect(
        plant.get_state_output_port(panda), panda_reference.get_input_port(0)
    )
    builder.Connect(
        panda_trajectory.get_output_port(), panda_reference.get_input_port(1)
    )
    gripper_command_serializer = PySerializer(Float64MultiArray)
    gripper_command = builder.AddSystem(
        RosSubscriberSystem(
            gripper_command_serializer,
            "panda_inspector/gripper_position",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
            ros_interface.get_ros_interface(),
        )
    )
    gripper_reference = builder.AddSystem(GripperPositionSetpointSystem())
    builder.Connect(
        gripper_command.get_output_port(), gripper_reference.get_input_port()
    )
    panda_desired_state = builder.AddSystem(
        PandaGripperStateMergeSystem(panda_actuator_joint_names)
    )
    builder.Connect(
        panda_reference.get_output_port(), panda_desired_state.get_input_port(0)
    )
    builder.Connect(
        gripper_reference.get_output_port(), panda_desired_state.get_input_port(1)
    )
    builder.Connect(
        panda_desired_state.get_output_port(),
        plant.get_desired_state_input_port(panda),
    )

    gravity_compensation = builder.AddSystem(
        PandaGravityCompensationSystem(
            plant, panda, plant.get_state_output_port().size()
        )
    )
    builder.Connect(plant.get_state_output_port(), gravity_compensation.get_input_port())
    builder.Connect(
        gravity_compensation.get_output_port(), plant.get_actuation_input_port()
    )

    joint_state_message = builder.AddSystem(JointStateMessageSystem(PANDA_JOINT_NAMES))
    panda_actuation_reorder = builder.AddSystem(
        VectorReorderSystem(panda_actuator_joint_names, PANDA_JOINT_NAMES)
    )
    panda_joint_state_serializer = PySerializer(JointState)
    joint_state_publisher = builder.AddSystem(
        RosPublisherSystem(
            panda_joint_state_serializer,
            "panda_inspector/joint_states",
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
            ros_interface.get_ros_interface(),
            {TriggerType.kPeriodic},
            PANDA_FEEDBACK_PERIOD,
        )
    )
    builder.Connect(
        plant.get_state_output_port(panda), joint_state_message.get_input_port(0)
    )
    builder.Connect(
        plant.get_net_actuation_output_port(panda),
        panda_actuation_reorder.get_input_port(),
    )
    builder.Connect(
        panda_actuation_reorder.get_output_port(),
        joint_state_message.get_input_port(1),
    )
    builder.Connect(
        joint_state_message.get_output_port(),
        joint_state_publisher.get_input_port(),
    )

    tf_broadcaster = builder.AddSystem(
        SceneTfBroadcasterSystem(
            ros_interface.get_ros_interface(),
            params=SceneTfBroadcasterParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=SCENE_MARKER_PERIOD,
            ),
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(), tf_broadcaster.get_graph_query_input_port()
    )
    visualizer = builder.AddSystem(
        RvizVisualizer(
            ros_interface.get_ros_interface(),
            params=RvizVisualizerParams(
                publish_triggers={TriggerType.kPeriodic},
                publish_period=SCENE_MARKER_PERIOD,
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
