import math

import pytest

from robotics_ii_tutorials.validation import (
    PANDA_HOME,
    PANDA_JOINT_NAMES,
    named_positions,
    validate_panda_target,
)


def test_home_target_is_valid():
    assert validate_panda_target(PANDA_HOME) == list(PANDA_HOME)


@pytest.mark.parametrize("bad_value", [math.inf, -math.inf, math.nan])
def test_non_finite_target_is_rejected(bad_value):
    target = list(PANDA_HOME)
    target[0] = bad_value
    with pytest.raises(ValueError):
        validate_panda_target(target)


def test_named_positions_do_not_depend_on_message_order():
    reverse_names = list(reversed(PANDA_JOINT_NAMES))
    reverse_positions = list(reversed(range(7)))
    assert named_positions(reverse_names, reverse_positions, PANDA_JOINT_NAMES) == list(
        range(7)
    )


def test_malformed_joint_state_is_rejected():
    assert named_positions(["a", "b"], [1.0], ["a"]) is None


def test_duplicate_joint_names_are_rejected():
    assert named_positions(["a", "a"], [1.0, 2.0], ["a"]) is None
