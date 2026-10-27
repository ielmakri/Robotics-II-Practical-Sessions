import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robotics_ii_tutorials")
    default_config = os.path.join(package_share, "config", "session1.yaml")
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional namespace for both tutorial nodes.",
            ),
            Node(
                package="robotics_ii_tutorials",
                executable="joint_state_source",
                name="joint_state_source",
                namespace=namespace,
                parameters=[default_config],
                output="screen",
            ),
            Node(
                package="robotics_ii_tutorials",
                executable="joint_state_monitor",
                name="joint_state_monitor",
                namespace=namespace,
                parameters=[default_config],
                output="screen",
            ),
        ]
    )

