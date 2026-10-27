"""Launch the controlled Panda with the student's waypoint sequencer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tutorial_share = get_package_share_directory("robotics_ii_tutorials")
    simulator_launch = os.path.join(
        get_package_share_directory("sim_systems"),
        "launch",
        "panda_manipulator_controller_py_launch.py",
    )
    sequence_config = os.path.join(
        tutorial_share,
        "config",
        "panda_sequence.yaml",
    )

    rviz = LaunchConfiguration("rviz")
    simulation_sec = LaunchConfiguration("simulation_sec")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("simulation_sec", default_value="inf"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch),
                launch_arguments={
                    "rviz": rviz,
                    "simulation_sec": simulation_sec,
                }.items(),
            ),
            Node(
                package="robotics_ii_tutorials",
                executable="panda_waypoint_sequencer",
                name="panda_waypoint_sequencer",
                parameters=[sequence_config],
                output="screen",
            ),
        ]
    )
