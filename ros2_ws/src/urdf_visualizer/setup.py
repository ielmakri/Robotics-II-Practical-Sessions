from setuptools import find_packages, setup
import os
from glob import glob


package_name = "urdf_visualizer"


setup(
    name=package_name,
    version="0.2.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (
            f"share/{package_name}",
            ["package.xml"],
        ),
        (
            f"share/{package_name}/launch",
            glob("launch/*.launch.py"),
        ),
        (
            f"share/{package_name}/rviz",
            glob("rviz/*"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="Mohayad Omer",
    maintainer_email="your_email@example.com",

    description=(
        "Generic ROS 2 URDF and Xacro visualizer."
    ),

    license="Apache-2.0",
)
