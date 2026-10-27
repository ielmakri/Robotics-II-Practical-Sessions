import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target = LaunchConfiguration("target_positions")
    tolerance = LaunchConfiguration("goal_tolerance")
    simulator_launch = os.path.join(
        get_package_share_directory("sim_systems"),
        "launch",
        "panda_manipulator_controller_py_launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_positions",
                default_value="[0.20, -0.60, 0.00, -2.00, 0.00, 1.70, 0.70]",
                description="Seven ordered Panda joint targets in radians.",
            ),
            DeclareLaunchArgument(
                "goal_tolerance",
                default_value="0.03",
                description="Maximum joint error accepted as reached.",
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(simulator_launch)),
            Node(
                package="robotics_ii_tutorials",
                executable="panda_joint_commander",
                parameters=[{"target_positions": target}],
                output="screen",
            ),
            Node(
                package="robotics_ii_tutorials",
                executable="panda_goal_monitor",
                parameters=[
                    {
                        "target_positions": target,
                        "goal_tolerance": tolerance,
                    }
                ],
                output="screen",
            ),
        ]
    )

