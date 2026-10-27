# Sourced by every shell in the course container: interactive ones through
# /home/ubuntu/.bashrc, non-interactive ones through BASH_ENV.
# This file also runs before the base image entrypoint. Track the current Bash
# PID rather than a process-tree flag: terminals opened after the first build
# must still source the course overlay even if their parent started earlier.
if [[ "${COURSE_ENV_PID:-}" != "$$" ]]; then
  export COURSE_ENV_PID="$$"
  _course_nounset="$(shopt -po nounset 2>/dev/null || true)"
  set +u

  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  case ":${PATH:-}:" in
    *:/opt/drake/bin:*) ;;
    *) export PATH="/opt/drake/bin${PATH:+:${PATH}}" ;;
  esac
  _drake_python_path="/opt/drake/lib/python$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages"
  case ":${PYTHONPATH:-}:" in
    *:"${_drake_python_path}":*) ;;
    *) export PYTHONPATH="${_drake_python_path}${PYTHONPATH:+:${PYTHONPATH}}" ;;
  esac
  source /opt/drake_ros_ws/install/setup.bash

  # The first shell after container startup may run before the course workspace
  # has been built. Source the overlay automatically once it exists.
  COURSE_WS="${COURSE_WS:-/home/ubuntu/Robotics-II/ros2_ws}"
  if [[ -f "${COURSE_WS}/install/setup.bash" ]]; then
    source "${COURSE_WS}/install/setup.bash"
  fi

  eval "${_course_nounset}"
  unset _course_nounset _drake_python_path
fi
