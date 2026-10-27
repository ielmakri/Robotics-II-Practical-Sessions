from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sim_systems'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*_launch.py'),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Pedagogical Drake systems connected to ROS 2 for Robotics II.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'panda_manipulator = sim_systems.panda_manipulator:main',
            'panda_manipulator_controlled = sim_systems.panda_manipulator_controlled:main',
            'husky = sim_systems.husky:main',
            'mobile_manipulator = sim_systems.mobile_manipulator:main',
            'mobile_manipulator_controlled = sim_systems.mobile_manipulator_controlled:main',
            'husky_command_guard = sim_systems.husky_command_guard:main',
        ],
    },
)
