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
            Node(
                package="sim_systems",
                executable="husky",
                name="husky",
                arguments=[
                    "--simulation_sec",
                    LaunchConfiguration("simulation_sec"),
                    "--realtime_rate",
                    LaunchConfiguration("realtime_rate"),
                ],
                output="screen",
                on_exit=Shutdown(reason="Husky simulator exited"),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(package_share, "rviz", "husky.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
