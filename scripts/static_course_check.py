#!/usr/bin/env python3
"""Fast, ROS-independent checks for source and teaching material."""

from __future__ import annotations

import ast
from pathlib import Path
import re
import sys
from urllib.parse import unquote
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOTS = (
    ROOT / "ros2_ws/src/robotics_ii_tutorials",
    ROOT / "ros2_ws/src/robotics_ii_description",
    ROOT / "ros2_ws/src/sim_systems",
    ROOT / "ros2_ws/src/urdf_visualizer",
    ROOT / "exercises/session_3",
)
PANDA_URDF = (
    ROOT
    / "models/descriptions/robots/arms/panda_fer/urdf/panda_fer_hand.urdf"
)
HUSKY_URDF = (
    ROOT / "models/descriptions/robots/mobile/clearpath_husky/urdf/husky.urdf"
)
EXPECTED_MOVING_JOINTS = [
    *(f"panda_joint{index}" for index in range(1, 8)),
    "panda_finger_joint1",
    "panda_finger_joint2",
]
CAPSTONE_FILES = (
    ROOT / "exercises/session_1/panda_waypoint_sequencer_template.py",
    ROOT / "exercises/session_1/session1_panda_sequence.launch.py",
    ROOT / "exercises/session_1/panda_sequence.yaml",
)
SESSION2_URDF = (
    ROOT / "ros2_ws/src/robotics_ii_description/urdf/tutorial_robot.urdf"
)
SESSION2_XACRO = (
    ROOT / "ros2_ws/src/robotics_ii_description/urdf/tutorial_robot.urdf.xacro"
)
SESSION2_DESCRIPTION_LAUNCH = (
    ROOT / "ros2_ws/src/robotics_ii_description/launch/tutorial_robot.launch.py"
)
SESSION2_VISUALIZER_LAUNCH = (
    ROOT / "ros2_ws/src/urdf_visualizer/launch/robot_visualizer.launch.py"
)
SESSION2_RVIZ = ROOT / "ros2_ws/src/urdf_visualizer/rviz/urdf_visualizer.rviz"
TEAM_PROJECT_SCENES = {
    "two_husky_corridor.sdf": {"husky_a", "husky_b"},
    "dual_panda_inspection.sdf": {"panda_a", "panda_b"},
    "husky_panda_inspection.sdf": {"husky_runner", "panda_inspector"},
    "husky_panda_pick_place.sdf": {"husky_runner", "panda_inspector"},
}
COURSE_UTILS = ROOT / "ros2_ws/src/sim_systems/sim_systems/course_utils.py"
BASELINE_PANDA = ROOT / "ros2_ws/src/sim_systems/sim_systems/panda_manipulator.py"
CONTROLLED_PANDA = (
    ROOT / "ros2_ws/src/sim_systems/sim_systems/panda_manipulator_controlled.py"
)
PROJECT_WORLD_CHECKER = ROOT / "exercises/session_3/check_project_world.py"
MOBILE_MANIPULATOR_CONTROLLED = (
    ROOT / "ros2_ws/src/sim_systems/sim_systems/mobile_manipulator_controlled.py"
)
MOBILE_MANIPULATOR_ADAPTERS = (
    ROOT / "ros2_ws/src/sim_systems/sim_systems/mobile_manipulator_adapters.py"
)
HUSKY_COMMAND_GUARD = (
    ROOT / "ros2_ws/src/sim_systems/sim_systems/husky_command_guard.py"
)
MOBILE_MANIPULATOR_LAUNCH = (
    ROOT / "ros2_ws/src/sim_systems/launch/mobile_manipulator_controlled_py_launch.py"
)
SIM_SYSTEMS_SETUP = ROOT / "ros2_ws/src/sim_systems/setup.py"
SIM_SYSTEMS_PACKAGE = ROOT / "ros2_ws/src/sim_systems/package.xml"
SESSION2_GUIDE = ROOT / "docs/session_2_urdf_tf/README.md"
SESSION3_GUIDE = ROOT / "docs/session_3_sim_systems/README.md"
PROJECT_GUIDE = ROOT / "docs/projects/README.md"
PROJECT_TEMPLATE = ROOT / "exercises/projects/project_readme_template.md"


def check_python(errors: list[str]) -> None:
    for source_root in PYTHON_ROOTS:
        for path in sorted(source_root.rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            try:
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            except (OSError, SyntaxError, UnicodeError) as error:
                errors.append(f"Python: {path.relative_to(ROOT)}: {error}")


def check_panda_urdf(errors: list[str]) -> None:
    try:
        root = ET.parse(PANDA_URDF).getroot()
    except (OSError, ET.ParseError) as error:
        errors.append(f"Panda URDF: {error}")
        return

    moving_joints = [
        joint.attrib.get("name", "")
        for joint in root.findall("joint")
        if joint.attrib.get("type") != "fixed"
    ]
    if moving_joints != EXPECTED_MOVING_JOINTS:
        errors.append(
            "Panda URDF moving-joint order differs from the ROS contract: "
            f"{moving_joints}"
        )

    transmitted = {
        joint.attrib.get("name", "")
        for transmission in root.findall("transmission")
        for joint in transmission.findall("joint")
    }
    missing_transmissions = set(EXPECTED_MOVING_JOINTS) - transmitted
    if missing_transmissions:
        errors.append(
            "Panda URDF joints without transmissions: "
            + ", ".join(sorted(missing_transmissions))
        )

    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        try:
            mass = float(inertial.find("mass").attrib["value"])
            inertia = {
                name: float(value)
                for name, value in inertial.find("inertia").attrib.items()
            }
            ixx, iyy, izz = inertia["ixx"], inertia["iyy"], inertia["izz"]
            ixy, ixz, iyz = inertia["ixy"], inertia["ixz"], inertia["iyz"]
            minor_2 = ixx * iyy - ixy * ixy
            determinant = (
                ixx * (iyy * izz - iyz * iyz)
                - ixy * (ixy * izz - iyz * ixz)
                + ixz * (ixy * iyz - iyy * ixz)
            )
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            errors.append(
                f"Panda URDF malformed inertia on {link.attrib.get('name')}: {error}"
            )
            continue
        if mass <= 0.0 or ixx <= 0.0 or minor_2 <= 0.0 or determinant <= 0.0:
            errors.append(
                "Panda URDF non-positive mass/inertia on "
                f"{link.attrib.get('name')}"
            )

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename or "://" in filename:
            continue
        mesh_path = (PANDA_URDF.parent / filename).resolve()
        if not mesh_path.is_file():
            errors.append(
                f"Panda URDF missing mesh {filename!r} "
                f"({mesh_path.relative_to(ROOT)})"
            )


def check_markdown(errors: list[str]) -> None:
    excluded_parts = {
        "build",
        "install",
        "log",
        "drake-ros",
        "drake_tutorial_Robotics_I",
        ".git",
    }
    for path in sorted(ROOT.rglob("*.md")):
        if excluded_parts.intersection(path.relative_to(ROOT).parts):
            continue
        contents = path.read_text(encoding="utf-8")
        lines = contents.splitlines()
        fences = [line for line in lines if line.lstrip().startswith("```")]
        if len(fences) % 2:
            errors.append(f"Markdown: unclosed code fence in {path.relative_to(ROOT)}")
        for line_number, line in enumerate(lines, start=1):
            if line.strip().lower() == "svg":
                errors.append(
                    f"Markdown: stray 'svg' in {path.relative_to(ROOT)}:{line_number}"
                )
        for target in re.findall(r"\[[^\]]*\]\(([^)]+)\)", contents):
            target = unquote(target.strip().split(maxsplit=1)[0])
            if (
                not target
                or target.startswith("#")
                or "://" in target
                or target.startswith("mailto:")
            ):
                continue
            local_target = target.split("#", maxsplit=1)[0]
            if local_target and not (path.parent / local_target).resolve().exists():
                errors.append(
                    f"Markdown: missing local link {target!r} in "
                    f"{path.relative_to(ROOT)}"
                )


def check_capstone_starters(errors: list[str]) -> None:
    for path in CAPSTONE_FILES:
        if not path.is_file():
            errors.append(f"Session 1 capstone starter missing: {path.relative_to(ROOT)}")
    template = CAPSTONE_FILES[0]
    if template.is_file():
        contents = template.read_text(encoding="utf-8")
        numbered_todos = re.findall(r"TODO ([1-6]):", contents)
        if numbered_todos != list("123456"):
            errors.append(
                "Session 1 Panda template must contain TODO 1 through TODO 6 "
                "exactly once and in order."
            )
        for contract_text in ("panda/joint_trajectory", "sim_sys/joint_states"):
            if contract_text not in contents:
                errors.append(
                    f"Session 1 Panda template lacks contract {contract_text!r}."
                )


def check_session2_description(errors: list[str]) -> None:
    """Check the reference 2-DOF model and its RViz launch contract."""
    try:
        urdf_root = ET.parse(SESSION2_URDF).getroot()
    except (OSError, ET.ParseError) as error:
        errors.append(f"Session 2 URDF: {error}")
        return

    expected_links = ["world", "base_link", "link_1", "link_2", "tool0"]
    links = [link.attrib.get("name", "") for link in urdf_root.findall("link")]
    if links != expected_links:
        errors.append(
            "Session 2 reference link order differs from the teaching model: "
            f"{links}"
        )

    expected_joints = [
        ("world_to_base", "fixed", "world", "base_link"),
        ("joint_1", "revolute", "base_link", "link_1"),
        ("joint_2", "revolute", "link_1", "link_2"),
        ("tool_joint", "fixed", "link_2", "tool0"),
    ]
    joints = []
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        joints.append(
            (
                joint.attrib.get("name", ""),
                joint.attrib.get("type", ""),
                "" if parent is None else parent.attrib.get("link", ""),
                "" if child is None else child.attrib.get("link", ""),
            )
        )
    if joints != expected_joints:
        errors.append(
            "Session 2 reference joint tree differs from the teaching model: "
            f"{joints}"
        )

    for link_name in ("base_link", "link_1", "link_2"):
        link = urdf_root.find(f"link[@name='{link_name}']")
        if link is None:
            continue
        for element_name in ("visual", "collision", "inertial"):
            if link.find(element_name) is None:
                errors.append(
                    f"Session 2 link {link_name} lacks {element_name} data."
                )
        inertial = link.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass")
            if mass is None:
                errors.append(f"Session 2 link {link_name} lacks a mass value.")
            else:
                try:
                    if float(mass.attrib["value"]) <= 0.0:
                        errors.append(
                            f"Session 2 link {link_name} has non-positive mass."
                        )
                except (KeyError, TypeError, ValueError) as error:
                    errors.append(
                        f"Session 2 link {link_name} has malformed mass: {error}"
                    )

    try:
        ET.parse(SESSION2_XACRO)
    except (OSError, ET.ParseError) as error:
        errors.append(f"Session 2 Xacro: {error}")
    else:
        xacro_text = SESSION2_XACRO.read_text(encoding="utf-8")
        if 'name="link_length"' not in xacro_text:
            errors.append("Session 2 Xacro must expose the documented link_length property.")
        if "forearm_length" in xacro_text:
            errors.append("Session 2 Xacro uses the obsolete forearm_length property name.")
        for required_name in ("world", "base_link", "link_1", "link_2", "tool0"):
            if required_name not in xacro_text:
                errors.append(f"Session 2 Xacro lacks link/frame {required_name!r}.")
        for required_name in ("world_to_base", "joint_1", "joint_2", "tool_joint"):
            if required_name not in xacro_text:
                errors.append(f"Session 2 Xacro lacks joint {required_name!r}.")

    for path in (SESSION2_DESCRIPTION_LAUNCH, SESSION2_VISUALIZER_LAUNCH):
        try:
            contents = path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"Session 2 launch file {path.relative_to(ROOT)}: {error}")
            continue
        if '"fixed_frame"' not in contents:
            errors.append(f"Session 2 launch lacks fixed_frame: {path.relative_to(ROOT)}")

    try:
        rviz_contents = SESSION2_RVIZ.read_text(encoding="utf-8")
    except OSError as error:
        errors.append(f"Session 2 RViz config: {error}")
    else:
        if "Fixed Frame: world" not in rviz_contents:
            errors.append("Session 2 RViz config must default to the world frame.")
        if "panda_" in rviz_contents:
            errors.append("Session 2 RViz config contains Panda-specific frame entries.")


def check_panda_ros_contract(errors: list[str]) -> None:
    """Check the regression points reported for the controlled Panda launch."""
    try:
        utility_source = COURSE_UTILS.read_text(encoding="utf-8")
        simulator_source = CONTROLLED_PANDA.read_text(encoding="utf-8")
    except OSError as error:
        errors.append(f"Controlled Panda source: {error}")
        return

    if "remove_ros_args(args=sys.argv)" not in utility_source:
        errors.append(
            "Simulator CLI must filter ROS launch arguments before argparse."
        )
    if "simulation_arguments(" not in simulator_source:
        errors.append(
            "Controlled Panda must use the ROS-launch-safe argument parser."
        )
    if "signal.signal(signal.SIGINT, signal.SIG_IGN)" not in utility_source:
        errors.append(
            "Simulator shutdown must ignore duplicate Ctrl+C signals after stopping."
        )

    expected_topics = (
        "panda/joint_trajectory",
        "sim_sys/desired_positions",
        "sim_sys/joint_states",
        "sim_sys/joint_acceleration",
    )
    for topic in expected_topics:
        if topic not in simulator_source:
            errors.append(
                f"Controlled Panda source does not declare the {topic} interface."
            )


def check_session3_runtime_fixes(errors: list[str]) -> None:
    """Guard the Drake API and the Session 3 Panda hold behavior."""
    try:
        checker_source = PROJECT_WORLD_CHECKER.read_text(encoding="utf-8")
        mobile_source = MOBILE_MANIPULATOR_CONTROLLED.read_text(encoding="utf-8")
        launch_source = MOBILE_MANIPULATOR_LAUNCH.read_text(encoding="utf-8")
        guard_source = HUSKY_COMMAND_GUARD.read_text(encoding="utf-8")
        baseline_source = BASELINE_PANDA.read_text(encoding="utf-8")
        session3_guide = SESSION3_GUIDE.read_text(encoding="utf-8")
    except OSError as error:
        errors.append(f"Session 3 runtime source: {error}")
        return

    if "GetModelInstanceIndices" in checker_source:
        errors.append(
            "Project-world checker must not call the unavailable "
            "GetModelInstanceIndices API."
        )
    if (
        "ModelInstanceIndex(index)" not in checker_source
        or "range(plant.num_model_instances())" not in checker_source
    ):
        errors.append(
            "Project-world checker must enumerate model instances using "
            "ModelInstanceIndex and num_model_instances."
        )
    if (
        "ModelInstanceIndex(index)" not in mobile_source
        or "range(plant.num_model_instances())" not in mobile_source
    ):
        errors.append(
            "Husky + Panda simulation must find robot instances from the plant, "
            "not only Parser.AddModels' return value."
        )
    if "if rclpy.ok():" not in guard_source:
        errors.append(
            "Husky command guard must avoid shutting down an already stopped ROS context."
        )
    custom_scene_section = session3_guide.split(
        "The checker also accepts a team SDF filename", maxsplit=1
    )
    if len(custom_scene_section) != 2:
        errors.append("Session 3 must explain how to check a custom team SDF.")
    else:
        custom_scene_section = custom_scene_section[1]
        seed_command = "cp models/team_projects/husky_panda_pick_place.sdf"
        copy_line = next(
            (
                line
                for line in custom_scene_section.splitlines()
                if line.startswith(seed_command)
            ),
            "",
        )
        check_command = (
            "python3 exercises/session_3/check_project_world.py team_01_world.sdf"
        )
        if (
            seed_command not in custom_scene_section
            or check_command not in custom_scene_section
            or custom_scene_section.index(seed_command)
            > custom_scene_section.index(check_command)
        ):
            errors.append(
                "Session 3 must create the example team SDF before checking it."
            )
        if not re.search(r"(?<!\\)\\$", copy_line):
            errors.append(
                "Session 3 team SDF copy command must end with exactly one "
                "shell line-continuation backslash."
            )
    if "scene_root not in scene_path.parents" not in checker_source:
        errors.append("Project-world checker must keep team SDFs inside models/team_projects/.")
    if 'PROJECT_SCENES.get(' not in checker_source:
        errors.append("Project-world checker must support custom Husky + Panda SDF names.")

    required_hold_components = (
        "ConstantVectorSource",
        "JointStiffnessController",
        "get_input_port_estimated_state",
        "get_input_port_desired_state",
        "get_actuation_input_port(panda)",
    )
    for component in required_hold_components:
        if component not in baseline_source:
            errors.append(
                "Session 3 baseline Panda is missing hold-controller "
                f"component {component!r}."
            )

    for path in (BASELINE_PANDA, CONTROLLED_PANDA):
        source = path.read_text(encoding="utf-8")
        for required_component in ("PandaGravityCompensationSystem", "Adder("):
            if required_component not in source:
                errors.append(
                    f"{path.relative_to(ROOT)} lacks gravity-compensated "
                    f"actuation component {required_component!r}."
                )
    adapter_source = MOBILE_MANIPULATOR_ADAPTERS.read_text(encoding="utf-8")
    if "torques[actuator_start] = -gravity[velocity_start]" not in adapter_source:
        errors.append("Panda gravity compensation must apply the opposing gravity torque.")
    for required_component in (
        "class GripperPositionSetpointSystem",
        "class PandaGripperStateMergeSystem",
        "Float64MultiArray",
        "len(message.data) != 1",
        "0.0 <= position <= 0.04",
    ):
        if required_component not in adapter_source:
            errors.append(
                f"Mobile-manipulator adapters lack gripper contract {required_component!r}."
            )
    if "panda_inspector/gripper_position" not in mobile_source:
        errors.append("Mobile-manipulator controller lacks the Panda gripper topic.")
    if 'default="husky_panda_pick_place.sdf"' not in mobile_source:
        errors.append("Mobile-manipulator controller must default to the mounted SDF.")
    if 'default_value="husky_panda_pick_place.sdf"' not in launch_source:
        errors.append("Mobile-manipulator launch must default to the mounted SDF.")
    if "default_position=0.035" not in adapter_source:
        errors.append("Panda gripper must start in its open default position.")


def check_course_content(errors: list[str]) -> None:
    """Guard the student-facing math, figures, project count, and template tone."""
    try:
        session2 = SESSION2_GUIDE.read_text(encoding="utf-8")
        session3 = SESSION3_GUIDE.read_text(encoding="utf-8")
        projects = PROJECT_GUIDE.read_text(encoding="utf-8")
        template = PROJECT_TEMPLATE.read_text(encoding="utf-8")
    except OSError as error:
        errors.append(f"Course documentation: {error}")
        return

    if "\\[" in session2 or "\\]" in session2:
        errors.append(
            "Session 2 display math must use GitHub-rendered $$ delimiters, "
            "not \\[ and \\]."
        )

    session3_images = re.findall(r"!\[[^]]*\]\(([^)]+\.svg)\)", session3)
    required_session3_images = {
        "images/sdf_multi_robot_world.svg",
        "images/drake_ros_command_feedback.svg",
    }
    if not required_session3_images.issubset(session3_images):
        errors.append("Session 3 must embed both of its SDF and ROS-flow figures.")

    for number in range(1, 8):
        if projects.count(f"### {number}. ") != 1:
            errors.append(f"Project guide must contain task {number} exactly once.")
        section_match = re.search(
            rf"^### {number}\. .*?(?=^### [1-7]\. |^## |\Z)",
            projects,
            flags=re.MULTILINE | re.DOTALL,
        )
        if section_match is not None:
            section = re.sub(r"!\[[^]]*\]\([^)]+\)", "", section_match.group(0))
            for robot_name in ("Husky", "Panda"):
                if robot_name not in section:
                    errors.append(
                        f"Project {number} must explicitly use the {robot_name}."
                    )
    project_images = re.findall(r"!\[[^]]*\]\((images/[^)]+\.svg)\)", projects)
    if len(project_images) != 7:
        errors.append("Each of the seven project briefs must have one illustration.")

    normalized_projects = " ".join(projects.split())
    required_project_contracts = (
        "at least one ROS 2 mission-coordinator node",
        "at least two team-owned subscriptions",
        "two team-owned command publishers",
        "Every project uses the Husky mobile manipulator",
        "python3 exercises/session_3/check_project_world.py team_01_world.sdf",
    )
    for contract in required_project_contracts:
        if contract not in normalized_projects:
            errors.append(f"Project guide lacks contract {contract!r}.")
    if "mounted Panda arm" not in normalized_projects:
        errors.append("Project guide must use the Husky-mounted Panda platform.")
    if len(set(project_images)) != 7:
        errors.append("Each project brief must use its own illustration.")
    if "optional" not in template.splitlines()[0].lower():
        errors.append("The project README template must be clearly optional.")
    if "peer reviewer" in template.lower():
        errors.append("The optional project template must not require peer review.")

    student_docs = (
        ROOT / "docs/session_1_ros2/README.md",
        SESSION2_GUIDE,
        SESSION3_GUIDE,
        PROJECT_GUIDE,
        ROOT / "exercises/session_1/README.md",
        ROOT / "exercises/session_2/README.md",
        ROOT / "exercises/session_3/README.md",
    )
    for path in student_docs:
        if path.is_file() and "I checked the project scenes" in path.read_text(
            encoding="utf-8"
        ):
            errors.append(
                f"Student guide contains instructor-facing wording: {path.relative_to(ROOT)}"
            )


def check_team_project_scenes(errors: list[str]) -> None:
    """Check that project SDFs are valid XML and their package URIs exist."""
    scene_root = ROOT / "models/team_projects"
    for filename, expected_robot_names in TEAM_PROJECT_SCENES.items():
        path = scene_root / filename
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError) as error:
            errors.append(f"Project SDF {filename}: {error}")
            continue

        world = root.find("world")
        if world is None:
            errors.append(f"Project SDF {filename} has no <world> element.")
            continue

        include_names: set[str] = set()
        for include in root.iter("include"):
            uri = (include.findtext("uri") or "").strip()
            name = include.findtext("name")
            if not uri:
                errors.append(f"Project SDF {filename} has an include without a URI.")
            if name:
                include_names.add(name)

        for uri_node in root.iter("uri"):
            uri = (uri_node.text or "").strip()
            if not uri:
                continue
            package_prefix = "package://robotics_ii_models/"
            if uri.startswith(package_prefix):
                asset = ROOT / "models" / uri.removeprefix(package_prefix)
                if not asset.is_file():
                    errors.append(
                        f"Project SDF {filename} references missing asset {uri}."
                    )

        missing_robots = expected_robot_names - include_names
        if missing_robots:
            errors.append(
                f"Project SDF {filename} lacks robot includes: "
                + ", ".join(sorted(missing_robots))
            )
        if filename == "dual_panda_inspection.sdf":
            static_models = {
                model.attrib.get("name", "") for model in world.findall("model")
            }
            required_scene_models = {
                "central_keep_clear_strip",
                "shared_center_scan_marker",
            }
            missing_models = required_scene_models - static_models
            if missing_models:
                errors.append(
                    "Dual-Panda project SDF lacks shared-zone models: "
                    + ", ".join(sorted(missing_models))
                )

        if filename == "husky_panda_pick_place.sdf":
            world = root.find("world")
            if world is None:
                errors.append("Pick-and-place SDF must contain a world.")
                continue

            assembly = world.find("model[@name='mobile_manipulator']")
            robot_includes = {
                include.findtext("name")
                for include in ([] if assembly is None else assembly.findall("include"))
                if include.findtext("name")
            }
            if not {"husky_runner", "panda_inspector"}.issubset(robot_includes):
                errors.append(
                    "Pick-and-place Husky and Panda must be included in the "
                    "mobile_manipulator assembly."
                )

            panda_includes = [
                include
                for include in ([] if assembly is None else assembly.findall("include"))
                if include.findtext("name") == "panda_inspector"
            ]
            if len(panda_includes) != 1:
                errors.append(
                    "Pick-and-place assembly must include exactly one Panda."
                )
            else:
                panda_pose_element = panda_includes[0].find("pose")
                try:
                    panda_pose = [
                        float(value)
                        for value in (
                            "0 0 0 0 0 0"
                            if panda_pose_element is None
                            else panda_pose_element.text or ""
                        ).split()
                    ]
                except ValueError:
                    panda_pose = []
                if (
                    panda_pose_element is None
                    or panda_pose_element.attrib.get("relative_to")
                    != "husky_runner::top_plate_link"
                    or len(panda_pose) != 6
                    or any(
                        abs(actual - expected) > 1e-9
                        for actual, expected in zip(
                            panda_pose, (0.0, 0.0, 0.00635, 0.0, 0.0, 0.0)
                        )
                    )
                ):
                    errors.append(
                        "Panda include pose must place its base 6.35 mm above "
                        "the Husky top-plate link origin."
                    )

            mount = root.find(".//joint[@name='panda_mount_joint']")
            if mount is None or mount.attrib.get("type") != "fixed":
                errors.append("Pick-and-place SDF must fix the Panda to the Husky.")
            else:
                if mount.findtext("parent") != "husky_runner::top_plate_link":
                    errors.append("Pick-and-place Panda mount has the wrong parent link.")
                if mount.findtext("child") != "panda_inspector::panda_link0":
                    errors.append("Pick-and-place Panda mount has the wrong child link.")
                try:
                    mount_pose = [
                        float(value)
                        for value in (
                            mount.findtext("pose") or "0 0 0 0 0 0"
                        ).split()
                    ]
                except ValueError:
                    mount_pose = []
                if (
                    mount.attrib.get("type") == "fixed"
                    and (
                        mount.find("pose") is None
                        or mount.find("pose").attrib.get("relative_to")
                        != "husky_runner::top_plate_link"
                        or len(mount_pose) != 6
                        or any(
                            abs(actual - expected) > 1e-9
                            for actual, expected in zip(
                                mount_pose, (0.0, 0.0, 0.00635, 0.0, 0.0, 0.0)
                            )
                        )
                    )
                ):
                    errors.append(
                        "Panda mount must place panda_link0 on the 6.35 mm "
                        "Husky top plate."
                    )

            supply_bin = root.find(".//model[@name='supply_bin']")
            if supply_bin is None:
                errors.append("Pick-and-place SDF lacks the dynamic supply bin.")
            else:
                if supply_bin.findtext("static") == "true":
                    errors.append("Pick-and-place supply bin must be dynamic.")
                if supply_bin.find(".//inertial/mass") is None:
                    errors.append("Pick-and-place supply bin needs mass and inertia.")
                if supply_bin.find(".//collision") is None:
                    errors.append("Pick-and-place supply bin needs collision geometry.")

                def pose_values(element):
                    pose = element.findtext("pose")
                    return [float(value) for value in (pose or "0 0 0 0 0 0").split()]

                def box_size(element):
                    size = element.findtext("geometry/box/size")
                    return [float(value) for value in (size or "").split()]

                try:
                    bin_pose = pose_values(supply_bin)
                    bin_link = supply_bin.find("link")
                    bin_collisions = (
                        [] if bin_link is None else bin_link.findall("collision")
                    )
                    bin_min = [float("inf")] * 3
                    bin_max = [float("-inf")] * 3
                    for collision in bin_collisions:
                        local_pose = pose_values(collision)
                        size = box_size(collision)
                        if len(local_pose) < 3 or len(size) != 3:
                            raise ValueError("a tote collision is not a positioned box")
                        for axis in range(3):
                            bin_min[axis] = min(
                                bin_min[axis], local_pose[axis] - size[axis] / 2
                            )
                            bin_max[axis] = max(
                                bin_max[axis], local_pose[axis] + size[axis] / 2
                            )
                    bin_extent = [
                        upper - lower for lower, upper in zip(bin_min, bin_max)
                    ]
                    if len(bin_pose) < 3 or any(
                        abs(actual - expected) > 1e-6
                        for actual, expected in zip(bin_extent, (0.15, 0.065, 0.075))
                    ):
                        errors.append(
                            "Pick-and-place tote must be 0.15 x 0.065 x 0.075 m."
                        )
                    else:
                        bench = root.find(".//model[@name='replenishment_workbench']")
                        bench_pose = pose_values(bench) if bench is not None else []
                        tabletop = (
                            None
                            if bench is None
                            else bench.find("link/collision[@name='tabletop_collision']")
                        )
                        tabletop_size = [] if tabletop is None else box_size(tabletop)
                        if len(bench_pose) < 3 or len(tabletop_size) != 3:
                            errors.append("Pick-and-place workbench tabletop is malformed.")
                        elif abs(
                            (bin_pose[2] + bin_min[2])
                            - (bench_pose[2] + tabletop_size[2] / 2)
                        ) > 1e-6:
                            errors.append(
                                "Pick-and-place tote must rest on the workbench surface."
                            )
                except (ValueError, TypeError) as error:
                    errors.append(f"Pick-and-place tote geometry is invalid: {error}.")

            bench = root.find(".//model[@name='replenishment_workbench']")
            if bench is None:
                errors.append("Pick-and-place SDF lacks the workbench.")
            else:
                leg_names = {
                    "front_left_leg_collision",
                    "front_right_leg_collision",
                    "rear_left_leg_collision",
                    "rear_right_leg_collision",
                }
                legs = {
                    collision.attrib.get("name", ""): collision
                    for collision in bench.findall("link/collision")
                    if collision.attrib.get("name", "").endswith("_leg_collision")
                }
                if set(legs) != leg_names:
                    errors.append("Pick-and-place workbench must have four leg collisions.")
                try:
                    bench_pose = pose_values(bench)
                    for name, leg in legs.items():
                        leg_pose = pose_values(leg)
                        leg_size = box_size(leg)
                        if len(bench_pose) < 3 or len(leg_pose) < 3 or len(leg_size) != 3:
                            raise ValueError(f"invalid leg geometry {name}")
                        bottom_z = bench_pose[2] + leg_pose[2] - leg_size[2] / 2
                        if abs(bottom_z) > 1e-6:
                            errors.append(
                                f"Workbench leg {name} must reach the floor; "
                                f"its bottom is at z={bottom_z:.6f} m."
                            )
                    tabletop = bench.find("link/collision[@name='tabletop_collision']")
                    tabletop_size = [] if tabletop is None else box_size(tabletop)
                    if len(bench_pose) < 3 or len(tabletop_size) != 3:
                        raise ValueError("invalid tabletop geometry")
                    top_z = bench_pose[2] + tabletop_size[2] / 2
                    if abs(top_z - 0.73) > 1e-6:
                        errors.append(
                            f"Workbench surface should be at z=0.73 m, got {top_z:.6f} m."
                        )
                except (ValueError, TypeError) as error:
                    errors.append(f"Pick-and-place workbench geometry is invalid: {error}.")

            mesh_uri = "package://robotics_ii_models/team_projects/assets/workbench.obj"
            if not any(
                (uri.text or "").strip() == mesh_uri
                for uri in root.iter("uri")
            ):
                errors.append("Pick-and-place workbench must demonstrate an OBJ mesh URI.")

            workbench_mesh = root.find(
                ".//model[@name='replenishment_workbench']"
                "/link[@name='workbench']/visual/geometry/mesh"
            )
            scale_text = (
                workbench_mesh.findtext("scale")
                if workbench_mesh is not None
                else None
            )
            try:
                mesh_scale = [
                    float(value) for value in (scale_text or "1 1 1").split()
                ]
            except ValueError:
                mesh_scale = []
            if len(mesh_scale) != 3 or any(
                abs(value - 1.0) > 1e-9 for value in mesh_scale
            ):
                errors.append(
                    "Pick-and-place workbench OBJ dimensions must be baked into "
                    "the mesh and use unit scale for Drake compatibility."
                )

            workbench_obj = ROOT / "models/team_projects/assets/workbench.obj"
            vertices = []
            try:
                for line in workbench_obj.read_text(encoding="utf-8").splitlines():
                    fields = line.split()
                    if len(fields) >= 4 and fields[0] == "v":
                        vertices.append(tuple(float(value) for value in fields[1:4]))
            except (OSError, ValueError) as error:
                errors.append(f"Pick-and-place workbench OBJ vertices are invalid: {error}.")
            if vertices:
                extent = [
                    max(vertex[axis] for vertex in vertices)
                    - min(vertex[axis] for vertex in vertices)
                    for axis in range(3)
                ]
                if any(
                    abs(actual - expected) > 1e-6
                    for actual, expected in zip(extent, (0.60, 0.55, 0.06))
                ):
                    errors.append(
                        "Pick-and-place workbench OBJ must be 0.60 x 0.55 x 0.06 m."
                    )

    static_box = ROOT / "exercises/session_3/static_box.sdf"
    try:
        ET.parse(static_box)
    except (OSError, ET.ParseError) as error:
        errors.append(f"Session 3 static box SDF: {error}")


def check_husky_panda_starter(errors: list[str]) -> None:
    """Check the mixed starter's model assets and ROS entry points."""
    expected_husky_actuators = {
        "front_left_wheel_motor": "front_left_wheel",
        "front_right_wheel_motor": "front_right_wheel",
        "rear_left_wheel_motor": "rear_left_wheel",
        "rear_right_wheel_motor": "rear_right_wheel",
    }
    expected_panda_actuators = {
        **{
            f"panda_motor{index}": f"panda_joint{index}"
            for index in range(1, 8)
        },
        "panda_finger_joint1": "panda_finger_joint1",
        "panda_finger_joint2": "panda_finger_joint2",
    }
    for path, expected in (
        (HUSKY_URDF, expected_husky_actuators),
        (PANDA_URDF, expected_panda_actuators),
    ):
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError) as error:
            errors.append(f"Robot actuator URDF {path.name}: {error}")
            continue
        declared = {}
        for transmission in root.findall("transmission"):
            joint = transmission.find("joint")
            actuator = transmission.find("actuator")
            if joint is not None and actuator is not None:
                declared[actuator.attrib.get("name", "")] = joint.attrib.get(
                    "name", ""
                )
        missing_or_changed = {
            name: joint
            for name, joint in expected.items()
            if declared.get(name) != joint
        }
        if missing_or_changed:
            errors.append(
                f"{path.name} actuator transmissions differ from the mixed "
                f"starter contract: {missing_or_changed}"
            )

    required_files = (
        MOBILE_MANIPULATOR_CONTROLLED,
        MOBILE_MANIPULATOR_ADAPTERS,
        HUSKY_COMMAND_GUARD,
        MOBILE_MANIPULATOR_LAUNCH,
        SIM_SYSTEMS_SETUP,
        SIM_SYSTEMS_PACKAGE,
    )
    missing_files = [path for path in required_files if not path.is_file()]
    if missing_files:
        errors.extend(
            f"Husky + Panda starter file missing: {path.relative_to(ROOT)}"
            for path in missing_files
        )
        return

    simulator_source = MOBILE_MANIPULATOR_CONTROLLED.read_text(encoding="utf-8")
    for interface in (
        "husky_runner/cmd_vel_safe",
        "husky_runner/odom",
        "panda_inspector/joint_trajectory",
        "panda_inspector/joint_states",
        "PandaGravityCompensationSystem",
        "LatchedJointTrajectorySetpointSystem",
        "TwistToWheelRates",
    ):
        if interface not in simulator_source:
            errors.append(f"Mixed-robot simulator is missing {interface!r}.")

    required_serializer_names = {
        "husky_command_serializer",
        "husky_odometry_serializer",
        "panda_trajectory_serializer",
        "panda_joint_state_serializer",
    }
    try:
        simulator_ast = ast.parse(simulator_source)
    except SyntaxError as error:
        errors.append(f"Mixed-robot simulator Python syntax: {error}")
        simulator_ast = None
    if simulator_ast is not None:
        created_serializers = set()
        ros_serializer_arguments = set()
        for node in ast.walk(simulator_ast):
            if isinstance(node, ast.Assign):
                value = node.value
                if (
                    isinstance(value, ast.Call)
                    and isinstance(value.func, ast.Name)
                    and value.func.id == "PySerializer"
                ):
                    created_serializers.update(
                        target.id
                        for target in node.targets
                        if isinstance(target, ast.Name)
                    )
            if (
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id in {"RosPublisherSystem", "RosSubscriberSystem"}
                and node.args
                and isinstance(node.args[0], ast.Name)
            ):
                ros_serializer_arguments.add(node.args[0].id)
        missing_serializers = required_serializer_names - created_serializers
        short_lived_serializers = required_serializer_names - ros_serializer_arguments
        if missing_serializers or short_lived_serializers:
            errors.append(
                "Mixed-robot ROS systems must retain Python serializer objects "
                "for their full lifetime; missing assignments: "
                + ", ".join(sorted(missing_serializers))
                + "; not passed to ROS systems: "
                + ", ".join(sorted(short_lived_serializers))
            )

    setup_source = SIM_SYSTEMS_SETUP.read_text(encoding="utf-8")
    for executable in (
        "mobile_manipulator_controlled",
        "husky_command_guard",
    ):
        if f"{executable} = sim_systems." not in setup_source:
            errors.append(f"sim_systems does not install {executable!r}.")

    try:
        package_root = ET.parse(SIM_SYSTEMS_PACKAGE).getroot()
    except ET.ParseError as error:
        errors.append(f"sim_systems package.xml: {error}")
    else:
        dependencies = {
            element.text
            for element in package_root.findall("depend")
            if element.text
        }
        for dependency in ("geometry_msgs", "nav_msgs"):
            if dependency not in dependencies:
                errors.append(
                    f"sim_systems package.xml lacks {dependency!r}."
                )

    launch_source = MOBILE_MANIPULATOR_LAUNCH.read_text(encoding="utf-8")
    for launch_contract in (
        'executable="mobile_manipulator_controlled"',
        'executable="husky_command_guard"',
        '"scene", default_value=',
    ):
        if launch_contract not in launch_source:
            errors.append(
                f"Mixed-robot launch is missing {launch_contract!r}."
            )


def main() -> int:
    errors: list[str] = []
    check_python(errors)
    check_panda_urdf(errors)
    check_markdown(errors)
    check_capstone_starters(errors)
    check_session2_description(errors)
    check_panda_ros_contract(errors)
    check_session3_runtime_fixes(errors)
    check_team_project_scenes(errors)
    check_husky_panda_starter(errors)
    check_course_content(errors)
    if errors:
        for error in errors:
            print(f"FAIL  {error}", file=sys.stderr)
        print(f"\n{len(errors)} static check(s) failed.", file=sys.stderr)
        return 1
    print(
        "PASS  Python syntax, Panda hand URDF/assets, Markdown and "
        "Session 1 starters, Session 2 model/launch checks, Panda ROS "
        "interfaces, Session 3 figures and Drake contracts, seven project "
        "briefs, project SDF assets, and the Husky-Panda mobile-manipulator "
        "command/feedback starter"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
