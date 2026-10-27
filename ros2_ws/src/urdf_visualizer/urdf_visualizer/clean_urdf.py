#!/usr/bin/env python3

from pathlib import Path
import copy
import re
import xml.etree.ElementTree as ET


def _local_name(tag):
    """Return XML tag name without namespace."""
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def _is_drake_element(element):
    """Check whether an XML element is Drake-specific."""
    tag = element.tag

    if not isinstance(tag, str):
        return False

    local_name = _local_name(tag).lower()

    # Namespaced Drake tags, e.g. {some_drake_namespace}foo
    if "drake" in tag.lower():
        return True

    # Non-namespaced tags explicitly starting with drake
    if local_name.startswith("drake"):
        return True

    return False


def _remove_drake_elements(parent):
    """Recursively remove Drake-specific XML elements."""

    for child in list(parent):

        if _is_drake_element(child):
            parent.remove(child)
            continue

        _remove_drake_elements(child)


def _get_root_link(robot):
    """Find the URDF root link."""

    links = {
        link.attrib["name"]
        for link in robot.findall("link")
    }

    child_links = {
        joint.find("child").attrib["link"]
        for joint in robot.findall("joint")
        if joint.find("child") is not None
    }

    root_links = links - child_links

    if len(root_links) != 1:
        raise RuntimeError(
            "Could not uniquely determine the URDF root link. "
            f"Found: {root_links}"
        )

    return root_links.pop()


def _add_dummy_root(robot, root_link):
    """
    Add a massless dummy root link if the current root link
    contains an inertial element.

    This avoids the KDL limitation that the root link cannot
    have inertia.
    """

    root_link_element = next(
        link
        for link in robot.findall("link")
        if link.attrib["name"] == root_link
    )

    if root_link_element.find("inertial") is None:
        return root_link

    dummy_name = "urdf_visualizer_root"

    # Avoid collisions if the name already exists.
    existing_names = {
        link.attrib["name"]
        for link in robot.findall("link")
    }

    if dummy_name in existing_names:
        i = 1
        while f"{dummy_name}_{i}" in existing_names:
            i += 1
        dummy_name = f"{dummy_name}_{i}"

    # Create dummy link.
    dummy_link = ET.Element(
        "link",
        {"name": dummy_name},
    )

    # Insert the dummy link at the beginning.
    robot.insert(0, dummy_link)

    # Fixed joint between dummy root and original root.
    dummy_joint = ET.Element(
        "joint",
        {
            "name": f"{dummy_name}_joint",
            "type": "fixed",
        },
    )

    ET.SubElement(
        dummy_joint,
        "parent",
        {"link": dummy_name},
    )

    ET.SubElement(
        dummy_joint,
        "child",
        {"link": root_link},
    )

    robot.append(dummy_joint)

    print(
        f"[urdf_visualizer] Added dummy root '{dummy_name}' "
        f"because '{root_link}' has inertia."
    )

    return dummy_name


def _resolve_mesh_paths(robot, urdf_directory):
    """
    Resolve mesh filenames into absolute file:// URIs.

    Supported inputs:
      - package://...
      - file://...
      - relative paths
      - absolute paths

    Drake model URIs such as

        package://drake_models/franka_description/meshes/visual/link0.gltf

    are resolved relative to the model-description directory containing
    the URDF.

    The original mesh format (.stl, .obj, .dae, .gltf, etc.) is preserved.
    """

    for mesh in robot.iter("mesh"):

        filename = mesh.get("filename")

        if not filename:
            continue

        # -------------------------------------------------------
        # Already a file URI
        # -------------------------------------------------------

        if filename.startswith("file://"):
            continue

        # -------------------------------------------------------
        # HTTP/HTTPS URI
        # -------------------------------------------------------

        if (
            filename.startswith("http://")
            or filename.startswith("https://")
        ):
            continue

        # -------------------------------------------------------
        # ROS / Drake package URI
        # -------------------------------------------------------

        if filename.startswith("package://"):

            package_path = filename[len("package://"):]

            # Split:
            #
            # package://drake_models/franka_description/...
            #
            # into:
            #
            # package_name = drake_models
            # relative_path = franka_description/...
            #
            parts = package_path.split("/", 1)

            if len(parts) != 2:
                print(
                    "[urdf_visualizer] Warning: malformed package URI:\n"
                    f"    {filename}"
                )
                continue

            package_name, relative_path = parts

            # ---------------------------------------------------
            # Standard ROS package
            # ---------------------------------------------------

            try:
                from ament_index_python.packages import (
                    get_package_share_directory,
                )

                package_directory = Path(
                    get_package_share_directory(package_name)
                )

                resolved = (
                    package_directory / relative_path
                ).resolve()

                if resolved.exists():
                    mesh.set(
                        "filename",
                        resolved.as_uri(),
                    )
                    continue

            except Exception:
                pass

            # ---------------------------------------------------
            # Drake-style model repository
            #
            # Example:
            #
            # package://drake_models/
            #     franka_description/
            #     meshes/visual/link0.gltf
            #
            # Actual:
            #
            # .../franka_description/
            #     meshes/visual/link0.gltf
            #
            # The URDF is:
            #
            # .../franka_description/urdf/panda_arm.urdf
            #
            # Therefore:
            #
            # urdf_directory.parent
            #     -> franka_description
            # ---------------------------------------------------

            if package_name == "drake_models":

                candidate = (
                    urdf_directory.parent / relative_path
                ).resolve()

                if candidate.exists():
                    mesh.set(
                        "filename",
                        candidate.as_uri(),
                    )

                    print(
                        "[urdf_visualizer] Resolved Drake mesh:\n"
                        f"    {filename}\n"
                        f"    -> {candidate}"
                    )

                    continue

            # ---------------------------------------------------
            # Could not resolve package URI
            # ---------------------------------------------------

            print(
                "[urdf_visualizer] Warning: could not resolve "
                "package mesh:\n"
                f"    {filename}"
            )

            continue

        # -------------------------------------------------------
        # Relative / absolute filesystem path
        # -------------------------------------------------------

        path = Path(filename)

        if path.is_absolute():
            resolved = path.resolve()
        else:
            resolved = (
                urdf_directory / path
            ).resolve()

        if not resolved.exists():
            print(
                "[urdf_visualizer] Warning: mesh not found:\n"
                f"    {resolved}"
            )

        mesh.set(
            "filename",
            resolved.as_uri(),
        )

def clean_urdf(input_urdf):
    """
    Clean a URDF for use with ROS tools.

    Parameters
    ----------
    input_urdf : str or Path
        Path to the original URDF.

    Returns
    -------
    Path
        Path to the cleaned URDF.
    """

    input_urdf = Path(input_urdf).resolve()

    if not input_urdf.is_file():
        raise FileNotFoundError(
            f"URDF file not found:\n{input_urdf}"
        )

    print(f"[urdf_visualizer] Loading URDF:\n  {input_urdf}")

    tree = ET.parse(input_urdf)
    robot = tree.getroot()

    if robot.tag != "robot":
        raise RuntimeError(
            f"Expected <robot> as root element, got <{robot.tag}>."
        )

    # -----------------------------------------------------------
    # Remove Drake-specific XML
    # -----------------------------------------------------------

    _remove_drake_elements(robot)

    # -----------------------------------------------------------
    # Resolve mesh paths
    # -----------------------------------------------------------

    _resolve_mesh_paths(
        robot,
        input_urdf.parent,
    )

    # -----------------------------------------------------------
    # Handle root-link inertia
    # -----------------------------------------------------------

    root_link = _get_root_link(robot)

    print(
        f"[urdf_visualizer] Root link: {root_link}"
    )

    root_link = _add_dummy_root(
        robot,
        root_link,
    )

    # -----------------------------------------------------------
    # Write cleaned URDF
    # -----------------------------------------------------------

    output_urdf = (
        input_urdf.parent
        / f".{input_urdf.stem}_ros.urdf"
    )

    # Pretty indentation (Python >= 3.9)
    ET.indent(tree, space="  ")

    tree.write(
        output_urdf,
        encoding="utf-8",
        xml_declaration=True,
    )

    print(
        f"[urdf_visualizer] Cleaned URDF:\n"
        f"  {output_urdf}"
    )

    return output_urdf


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert a URDF to a ROS-compatible URDF."
    )

    parser.add_argument(
        "urdf",
        help="Input URDF",
    )

    parser.add_argument(
        "-o",
        "--output",
        help="Output URDF (optional)",
    )

    args = parser.parse_args()

    output = clean_urdf(args.urdf)

    print(output)