import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory("robotics_ii_description")
    visualizer_launch = os.path.join(
        get_package_share_directory("urdf_visualizer"),
        "launch",
        "robot_visualizer.launch.py",
    )
    default_model = os.path.join(
        description_share,
        "urdf",
        "tutorial_robot.urdf",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to a URDF or Xacro file.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="world",
                description="RViz fixed frame; it must exist in the model TF tree.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(visualizer_launch),
                launch_arguments={
                    "urdf": LaunchConfiguration("model"),
                    "fixed_frame": LaunchConfiguration("fixed_frame"),
                }.items(),
            ),
        ]
    )
