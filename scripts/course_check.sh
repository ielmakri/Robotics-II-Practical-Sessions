#!/usr/bin/env bash
# Read-only diagnostics for the Robotics II environment.

set -uo pipefail

readonly REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly WORKSPACE="${REPOSITORY_ROOT}/ros2_ws"
failures=0

pass() { printf 'PASS  %s\n' "$*"; }
fail() { printf 'FAIL  %s\n' "$*" >&2; failures=$((failures + 1)); }

valid_xml() {
  local xml_path="$1"
  if command -v xmllint >/dev/null 2>&1; then
    xmllint --noout "${xml_path}" >/dev/null 2>&1
  else
    python3 -c 'import sys, xml.etree.ElementTree as ET; ET.parse(sys.argv[1])' \
      "${xml_path}" >/dev/null 2>&1
  fi
}

if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${REPOSITORY_ROOT}/scripts/setup_workspace.bash"
  set -u
  pass 'workspace setup file loaded'
else
  fail "workspace is not built (${WORKSPACE}/install/setup.bash is missing)"
fi

for command in ros2 colcon python3 rviz2; do
  if command -v "${command}" >/dev/null 2>&1; then
    pass "command available: ${command}"
  else
    fail "command unavailable: ${command}"
  fi
done

if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" ]]; then
  pass 'ROS middleware: rmw_cyclonedds_cpp'
else
  fail "ROS middleware is '${RMW_IMPLEMENTATION:-unset}', expected rmw_cyclonedds_cpp"
fi

for module in rclpy pydrake drake_ros; do
  if python3 -c "import ${module}" >/dev/null 2>&1; then
    pass "Python import: ${module}"
  else
    fail "Python import failed: ${module}"
  fi
done

for package in robotics_ii_tutorials robotics_ii_description urdf_visualizer sim_systems turtlesim rmw_cyclonedds_cpp; do
  if ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    pass "ROS package visible: ${package}"
  else
    fail "ROS package not visible: ${package}"
  fi
done

reference_urdf="${WORKSPACE}/src/robotics_ii_description/urdf/tutorial_robot.urdf"
if [[ -f "${reference_urdf}" ]] && valid_xml "${reference_urdf}"; then
  pass 'Session 2 reference URDF is valid XML'
else
  fail 'Session 2 reference URDF is missing or invalid XML'
fi

reference_xacro="${WORKSPACE}/src/robotics_ii_description/urdf/tutorial_robot.urdf.xacro"
if [[ -f "${reference_xacro}" ]] && valid_xml "${reference_xacro}"; then
  pass 'Session 2 Xacro source is valid XML'
else
  fail 'Session 2 Xacro source is missing or invalid XML'
fi

if python3 "${REPOSITORY_ROOT}/scripts/static_course_check.py"; then
  pass 'course source and documentation static checks'
else
  fail 'course source or documentation static checks failed'
fi

if (( failures == 0 )); then
  printf '\nEnvironment ready for Robotics II.\n'
else
  printf '\n%d check(s) failed. Read docs/00_setup.md before asking for help.\n' "${failures}" >&2
  exit 1
fi
