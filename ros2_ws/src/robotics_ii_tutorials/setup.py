from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robotics_ii_tutorials"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics II teaching team",
    maintainer_email="ilias.el.makrini@vub.be",
    description="Student-facing ROS 2 examples for Robotics II.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_state_source = robotics_ii_tutorials.joint_state_source:main",
            "joint_state_monitor = robotics_ii_tutorials.joint_state_monitor:main",
            "panda_joint_commander = robotics_ii_tutorials.panda_joint_commander:main",
            "panda_goal_monitor = robotics_ii_tutorials.panda_goal_monitor:main",
        ],
    },
)

