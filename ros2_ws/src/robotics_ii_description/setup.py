from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robotics_ii_description"


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
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics II teaching team",
    maintainer_email="ilias.el.makrini@vub.be",
    description="Small URDF and Xacro models used in Robotics II.",
    license="Apache-2.0",
)

