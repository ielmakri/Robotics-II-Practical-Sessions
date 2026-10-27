"""Shared Panda arm-and-hand model constants.

The ROS command used in Session 1 addresses the seven arm joints. The
Husky-Panda mobile-manipulator launch also exposes a separate gripper command
for the two finger joints.
"""

import numpy as np


PANDA_ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
PANDA_FINGER_JOINT_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]
PANDA_JOINT_NAMES = PANDA_ARM_JOINT_NAMES + PANDA_FINGER_JOINT_NAMES

PANDA_HOME = np.array(
    [0.0, -0.79, 0.0, -2.36, 0.0, 1.63, 0.77, 0.035, 0.035]
)
PANDA_LOWER_LIMITS = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0.0, 0.0]
)
PANDA_UPPER_LIMITS = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 0.04, 0.04]
)

# Conservative gains for the course's joint-space setpoint controller. The
# finger gains hold the supplied open pose; Session 1 commands only the arm.
PANDA_KP = np.array([300.0, 300.0, 300.0, 300.0, 150.0, 100.0, 50.0, 60.0, 60.0])
PANDA_KD = np.array([30.0, 30.0, 30.0, 30.0, 15.0, 10.0, 5.0, 5.0, 5.0])


def panda_hand_urdf(repository_root):
    """Return the supplied Panda arm-and-hand URDF path."""
    return (
        repository_root
        / "models/descriptions/robots/arms/panda_fer/urdf/panda_fer_hand.urdf"
    )


def validate_panda_model(plant, model_instance):
    """Fail early if the URDF no longer matches the course ROS contract."""
    actual_names = list(
        plant.GetPositionNames(model_instance, always_add_suffix=False)
    )
    if actual_names != PANDA_JOINT_NAMES:
        raise RuntimeError(
            "Unexpected Panda position order. Expected "
            f"{PANDA_JOINT_NAMES}, received {actual_names}."
        )
    counts = (
        plant.num_positions(model_instance),
        plant.num_velocities(model_instance),
        plant.num_actuated_dofs(model_instance),
    )
    expected = len(PANDA_JOINT_NAMES)
    if counts != (expected, expected, expected):
        raise RuntimeError(
            "The Panda URDF must expose nine positions, velocities and "
            f"actuated DOFs; received {counts}."
        )
