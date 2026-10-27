"""ROS-independent validation shared by nodes and unit tests."""

import math


PANDA_JOINT_NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))
PANDA_HOME = (0.0, -0.79, 0.0, -2.36, 0.0, 1.63, 0.77)
PANDA_LOWER_LIMITS = (
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973,
)
PANDA_UPPER_LIMITS = (
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973,
)


def validate_panda_target(values):
    """Return a float list or raise ValueError for an unsafe target."""
    target = [float(value) for value in values]
    if len(target) != len(PANDA_JOINT_NAMES):
        raise ValueError("A Panda target must contain exactly seven positions.")
    if not all(math.isfinite(value) for value in target):
        raise ValueError("Panda positions must all be finite.")

    for name, value, lower, upper in zip(
        PANDA_JOINT_NAMES,
        target,
        PANDA_LOWER_LIMITS,
        PANDA_UPPER_LIMITS,
    ):
        if not lower <= value <= upper:
            raise ValueError(
                f"{name}={value} rad is outside [{lower}, {upper}] rad."
            )
    return target


def named_positions(message_names, message_positions, required_names):
    """Return required positions in canonical order, or None if malformed."""
    if len(message_names) != len(message_positions):
        return None
    if len(set(message_names)) != len(message_names):
        return None
    by_name = dict(zip(message_names, message_positions))
    if not all(name in by_name for name in required_names):
        return None
    ordered = [float(by_name[name]) for name in required_names]
    if not all(math.isfinite(value) for value in ordered):
        return None
    return ordered
