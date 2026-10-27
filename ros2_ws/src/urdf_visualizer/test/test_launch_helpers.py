"""Checks for the launch-file helpers that pick the fixed frame and meshes.

Run inside the course environment with:
    python3 ros2_ws/src/urdf_visualizer/test/test_launch_helpers.py
"""

import importlib.util
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent.parent
LAUNCH_FILE = PACKAGE / "launch" / "robot_visualizer.launch.py"

spec = importlib.util.spec_from_file_location("robot_visualizer", LAUNCH_FILE)
robot_visualizer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(robot_visualizer)

TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="arm">
  <link name="base"/>
  <link name="tip"/>
  <joint name="j0" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
  </joint>
</robot>
"""


def test_root_link_is_the_link_no_joint_owns():
    assert robot_visualizer.find_root_link(TWO_LINK_URDF) == "base"


def test_root_link_of_a_real_course_model():
    fr3 = (PACKAGE / "urdfs" / "fr3" / "fr3.urdf").read_text()
    assert robot_visualizer.find_root_link(fr3) == "fr3_link0"


def test_package_uri_resolves_to_the_local_mesh():
    resolved = robot_visualizer.resolve_package_mesh(
        "package://iiwa_description/meshes/iiwa14/visual/link_0.stl",
        PACKAGE / "urdfs" / "iiwa14",
    )
    assert resolved is not None
    assert resolved.exists()
    assert resolved.parts[-2:] == ("visual", "link_0.stl")


def test_package_uri_without_a_local_match_is_left_alone():
    assert robot_visualizer.resolve_package_mesh(
        "package://some_pkg/meshes/visual/not_here.stl",
        PACKAGE / "urdfs" / "iiwa14",
    ) is None


if __name__ == "__main__":
    test_root_link_is_the_link_no_joint_owns()
    test_root_link_of_a_real_course_model()
    test_package_uri_resolves_to_the_local_mesh()
    test_package_uri_without_a_local_match_is_left_alone()
    print("all checks passed")
