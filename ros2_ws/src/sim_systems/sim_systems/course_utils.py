"""Small helpers shared by the course simulation executables."""

import argparse
import math
from pathlib import Path
import signal
import sys

from rclpy.utilities import remove_ros_args


def simulation_arguments(description, configure_parser=None):
    """Parse course arguments while accepting arguments added by ROS launch.

    ``launch_ros.actions.Node`` appends a ``--ros-args`` section (for example
    when it assigns a node name). A normal ``ArgumentParser.parse_args`` call
    rejects that section before drake_ros can use it. ``remove_ros_args`` keeps
    only the application arguments for this parser and leaves ``sys.argv``
    unchanged for ``drake_ros.core.init``.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--simulation_sec",
        type=float,
        default=float("inf"),
        help="Simulation duration; the default runs until Ctrl+C.",
    )
    parser.add_argument(
        "--realtime_rate",
        type=float,
        default=1.0,
        help="Requested simulator realtime rate.",
    )
    if configure_parser is not None:
        configure_parser(parser)
    args = parser.parse_args(remove_ros_args(args=sys.argv)[1:])
    if args.simulation_sec <= 0.0:
        parser.error("--simulation_sec must be positive")
    if args.realtime_rate <= 0.0:
        parser.error("--realtime_rate must be positive")
    return args


def repository_root():
    """Locate the source repository when built with --symlink-install."""
    module_path = Path(__file__).resolve()
    for candidate in module_path.parents:
        if (candidate / "models").is_dir() and (candidate / "ros2_ws").is_dir():
            return candidate
    raise RuntimeError(
        "Could not find the Robotics-II repository root. Build this workspace "
        "with colcon --symlink-install and source its install/setup.bash."
    )


def require_file(path, label):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"{label} not found: {path}")
    return path


def run_simulator(simulator, simulation_sec, step_sec=0.1):
    """Advance in finite chunks so Ctrl+C remains responsive."""
    if simulation_sec <= 0.0:
        raise ValueError("simulation_sec must be positive.")

    context = simulator.get_mutable_context()
    try:
        while context.get_time() < simulation_sec:
            next_time = context.get_time() + step_sec
            if math.isfinite(simulation_sec):
                next_time = min(next_time, simulation_sec)
            simulator.AdvanceTo(next_time)
    except KeyboardInterrupt:
        # Once shutdown has begun, ignore a second Ctrl+C. ros2 launch can
        # forward the terminal interrupt to a child after the terminal has
        # already delivered it to that child.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except RuntimeError as error:
        # drake_ros may attempt one last publication after rclcpp handles
        # Ctrl+C and invalidates its context. This is a normal shutdown race,
        # not a simulation failure. Preserve every other RuntimeError.
        # ros2 launch forwards the same Ctrl+C to its children. A second
        # KeyboardInterrupt can arrive while Python formats the original
        # RuntimeError, so keep classification interrupt-safe as well.
        try:
            message = str(error).lower()
        except KeyboardInterrupt:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            return
        normal_shutdown = (
            "context is invalid" in message and "publish" in message
        )
        if not normal_shutdown:
            raise
        signal.signal(signal.SIGINT, signal.SIG_IGN)
