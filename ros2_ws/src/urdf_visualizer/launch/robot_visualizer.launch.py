from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def find_root_link(robot_description: str) -> str:
    """Return the model's root link: the one that is no joint's child."""
    root = ET.fromstring(robot_description)
    links = [link.get("name") for link in root.iter("link")]
    children = {
        child.get("link")
        for joint in root.iter("joint")
        for child in joint.iter("child")
    }
    roots = [link for link in links if link not in children]

    if not roots:
        raise ValueError("The model has no root link; every link is a joint child.")
    if len(roots) > 1:
        print(
            f"[urdf_visualizer] WARNING: several root links {roots}; "
            f"using '{roots[0]}' as the RViz fixed frame."
        )

    return roots[0]


def resolve_package_mesh(uri: str, urdf_directory: Path):
    """Map a package:// mesh URI onto a file shipped next to the model.

    The models under urdfs/ were exported from description packages that are
    not installed here, and whose layout differs from this repository's: a URDF
    asks for meshes/iiwa14/visual/link_0.stl while the file sits at
    meshes/visual/link_0.stl. Match on the longest trailing path segments, and
    return None when nothing local fits, so an installed ROS package still
    resolves through RViz as usual.
    """
    wanted = Path(uri[len("package://"):]).parts[1:]
    if not wanted:
        return None

    def trailing_match(candidate: Path) -> int:
        shared = 0
        for mine, theirs in zip(reversed(candidate.parts), reversed(wanted)):
            if mine != theirs:
                break
            shared += 1
        return shared

    best = max(urdf_directory.rglob(wanted[-1]), key=trailing_match, default=None)

    return best.resolve() if best is not None else None


def resolve_mesh_paths(robot_description: str, urdf_path: str) -> str:
    """Resolve plain relative mesh paths before publishing robot_description."""
    urdf_directory = Path(urdf_path).resolve().parent
    root = ET.fromstring(robot_description)

    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue

        if filename.startswith("package://"):
            resolved = resolve_package_mesh(filename, urdf_directory)
            if resolved is None:
                print(f"[urdf_visualizer] WARNING: no local file matches: {filename}")
                continue
            print(f"[urdf_visualizer] Mesh: {filename} -> {resolved}")
            mesh.set("filename", resolved.as_uri())
            continue

        if "://" in filename:
            continue

        path = Path(filename)
        resolved = (
            path.resolve()
            if path.is_absolute()
            else (urdf_directory / path).resolve()
        )
        if not resolved.exists():
            print(f"[urdf_visualizer] WARNING: mesh not found: {resolved}")
        else:
            print(f"[urdf_visualizer] Mesh: {filename} -> {resolved}")
        mesh.set("filename", resolved.as_uri())

    return ET.tostring(root, encoding="unicode")


def load_robot_description(urdf_path: str) -> str:
    """Load either a plain URDF or an Xacro source file."""
    if urdf_path.lower().endswith(".xacro"):
        return xacro.process_file(urdf_path).toxml()

    return Path(urdf_path).read_text(encoding="utf-8")


def launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf").perform(context)
    fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
    model_path = Path(urdf_path).expanduser()

    if not model_path.is_file():
        raise FileNotFoundError(f"URDF or Xacro file not found: {model_path}")

    robot_description = resolve_mesh_paths(
        load_robot_description(str(model_path)),
        str(model_path),
    )

    # No single frame name suits every model under urdfs/, so an empty argument
    # means "use this model's own root link". An explicit value still wins.
    if not fixed_frame:
        fixed_frame = find_root_link(robot_description)

    rviz_config = (
        Path(get_package_share_directory("urdf_visualizer"))
        / "rviz"
        / "urdf_visualizer.rviz"
    )
    if not rviz_config.is_file():
        raise FileNotFoundError(f"RViz config not found: {rviz_config}")

    print(f"[urdf_visualizer] Model: {model_path}")
    print(f"[urdf_visualizer] RViz fixed frame: {fixed_frame}")
    print(f"[urdf_visualizer] RViz config: {rviz_config}")

    return [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                str(rviz_config),
                "-f",
                fixed_frame,
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf",
                description="Absolute path to a URDF or Xacro file.",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="",
                description=(
                    "RViz fixed frame; it must exist in the model TF tree. "
                    "Empty means the model's own root link."
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
