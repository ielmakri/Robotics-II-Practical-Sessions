import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("sim_systems")
    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_sec", default_value="inf"),
            DeclareLaunchArgument("realtime_rate", default_value="1.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "scene", default_value="husky_panda_pick_place.sdf"
            ),
            Node(
                package="sim_systems",
                executable="mobile_manipulator_controlled",
                name="mobile_manipulator_controlled",
                arguments=[
                    "--simulation_sec",
                    LaunchConfiguration("simulation_sec"),
                    "--realtime_rate",
                    LaunchConfiguration("realtime_rate"),
                    "--scene",
                    LaunchConfiguration("scene"),
                ],
                output="screen",
                on_exit=Shutdown(reason="Husky + Panda simulator exited"),
            ),
            Node(
                package="sim_systems",
                executable="husky_command_guard",
                name="husky_command_guard",
                parameters=[
                    {
                        "command_timeout_sec": 0.5,
                        "max_linear_speed": 0.25,
                        "max_angular_speed": 0.8,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(package_share, "rviz", "panda_manipulator.rviz"),
                ],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
