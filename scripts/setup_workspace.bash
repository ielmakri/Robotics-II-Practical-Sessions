#!/usr/bin/env bash
# Source this file in every terminal that uses the Robotics II workspace.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "This file must be sourced, not executed:" >&2
    echo "  source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

# Preserve the caller's nounset (`set -u`) state. ROS and colcon setup files
# legitimately inspect variables that may be unset, so nounset must be disabled
# while they are sourced. Most interactive shells start with nounset disabled,
# and this helper must not silently enable it.
case $- in
    *u*) robotics_ii_restore_nounset=1 ;;
    *)   robotics_ii_restore_nounset=0 ;;
esac

set +u
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "ROS 2 Jazzy is not installed. Run ./scripts/install.sh first." >&2
    if (( robotics_ii_restore_nounset )); then
        set -u
    fi
    unset robotics_ii_restore_nounset
    return 1
fi
source /opt/ros/jazzy/setup.bash

# Use one middleware implementation in every course terminal. Mixing RMW
# implementations can make graph discovery unreliable, especially under WSL2.
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

robotics_ii_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
robotics_ii_overlay="${robotics_ii_root}/ros2_ws/install/setup.bash"

if [[ ! -f "${robotics_ii_overlay}" ]]; then
    echo "Robotics II workspace is not built: ${robotics_ii_overlay}" >&2
    echo "Run ./scripts/install.sh, or build ros2_ws with colcon first." >&2
    if (( robotics_ii_restore_nounset )); then
        set -u
    fi
    unset robotics_ii_restore_nounset robotics_ii_root robotics_ii_overlay
    return 1
fi

source "${robotics_ii_overlay}"

if (( robotics_ii_restore_nounset )); then
    set -u
else
    set +u
fi

unset robotics_ii_restore_nounset robotics_ii_root robotics_ii_overlay
