#!/usr/bin/env bash
# Install the Robotics II workspace on Ubuntu 24.04 (native or WSL2).

set -euo pipefail

readonly COURSE_ROS_DISTRO="jazzy"
readonly DRAKE_VERSION="1.56.0"
readonly DRAKE_PACKAGE_VERSION="${DRAKE_VERSION}-1"
readonly DRAKE_ROS_COMMIT="5bd4e5e8aa7ff072d29f54e258fda446b11c95d2"
readonly REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly WORKSPACE="${REPOSITORY_ROOT}/ros2_ws"

ros_apt_deb=""
drake_deb=""
drake_ros_temp_dir=""

cleanup() {
  [[ -z "${ros_apt_deb}" ]] || rm -f -- "${ros_apt_deb}"
  [[ -z "${drake_deb}" ]] || rm -f -- "${drake_deb}"
  [[ -z "${drake_ros_temp_dir}" ]] || rm -rf -- "${drake_ros_temp_dir}"
}
trap cleanup EXIT

fail() {
  printf '\nERROR: %s\n' "$*" >&2
  exit 1
}

download() {
  local url="$1"
  local destination="$2"
  curl --fail --location --retry 3 --retry-delay 2 \
    --show-error --silent --output "${destination}" "${url}"
}

[[ -f "${WORKSPACE}/src/sim_systems/package.xml" ]] || \
  fail "Run ./scripts/install.sh from a complete Robotics-II repository clone."
[[ "$(id -u)" -ne 0 ]] || \
  fail "Run as your normal Ubuntu user. The script asks for sudo when required."

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  fail "Supported platform: Ubuntu 24.04 (Noble), native or WSL2. Detected: ${PRETTY_NAME:-unknown}."
fi

architecture="$(dpkg --print-architecture)"
case "${architecture}" in
  amd64|arm64) ;;
  *) fail "No course Drake package is configured for architecture '${architecture}'." ;;
esac

printf '%s\n' '[1/7] Checking sudo access...'
sudo -v

printf '%s\n' '[2/7] Installing download and build prerequisites...'
sudo apt-get update
sudo apt-get install --yes --no-install-recommends \
  ca-certificates curl git gnupg lsb-release software-properties-common tar wget

if [[ ! -f "/opt/ros/${COURSE_ROS_DISTRO}/setup.bash" ]]; then
  printf '%s\n' '[3/7] Adding the official ROS 2 APT source...'
  sudo add-apt-repository --yes universe
  ros_apt_source_version="$(
    curl --fail --silent --show-error \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | sed -n 's/.*"tag_name"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' \
      | head -n 1
  )"
  [[ -n "${ros_apt_source_version}" ]] || \
    fail "Could not determine the ROS APT-source release. Check internet access."
  ros_apt_deb="$(mktemp --suffix=.deb)"
  download \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${VERSION_CODENAME}_all.deb" \
    "${ros_apt_deb}"
  sudo dpkg -i "${ros_apt_deb}"
else
  printf '%s\n' '[3/7] ROS 2 APT source already configured.'
fi

printf '%s\n' '[4/7] Installing ROS 2 Jazzy and course tools...'
sudo apt-get update
sudo apt-get install --yes \
  "ros-${COURSE_ROS_DISTRO}-desktop" \
  "ros-${COURSE_ROS_DISTRO}-joint-state-publisher-gui" \
  "ros-${COURSE_ROS_DISTRO}-plotjuggler" \
  "ros-${COURSE_ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${COURSE_ROS_DISTRO}-rqt-graph" \
  "ros-${COURSE_ROS_DISTRO}-turtlesim" \
  "ros-${COURSE_ROS_DISTRO}-tf2-tools" \
  "ros-${COURSE_ROS_DISTRO}-xacro" \
  libxml2-utils \
  ros-dev-tools \
  shellcheck

installed_drake_version="$(dpkg-query -W -f='${Version}' drake-dev 2>/dev/null || true)"
if [[ "${installed_drake_version}" != "${DRAKE_PACKAGE_VERSION}" ]]; then
  printf '%s\n' "[5/7] Installing Drake ${DRAKE_PACKAGE_VERSION}..."
  drake_deb="$(mktemp --suffix=.deb)"
  download \
    "https://github.com/RobotLocomotion/drake/releases/download/v${DRAKE_VERSION}/drake-dev_${DRAKE_PACKAGE_VERSION}_${architecture}-${VERSION_CODENAME}.deb" \
    "${drake_deb}" \
    || fail "Could not download Drake ${DRAKE_VERSION} for ${architecture}/${VERSION_CODENAME}."
  sudo apt-get install --yes --no-install-recommends "${drake_deb}"
else
  printf '%s\n' "[5/7] Drake ${DRAKE_PACKAGE_VERSION} already installed."
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# shellcheck disable=SC1091
set +u
source "/opt/ros/${COURSE_ROS_DISTRO}/setup.bash"
set -u
export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
export PYTHONPATH="/opt/drake/lib/python${python_version}/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

drake_ros_destination="${WORKSPACE}/src/drake-ros"
if [[ ! -f "${drake_ros_destination}/drake_ros/package.xml" ]]; then
  if [[ -e "${drake_ros_destination}" ]]; then
    fail "${drake_ros_destination} exists but is incomplete. Move it aside, then rerun the installer."
  fi

  printf '%s\n' "[6/7] Downloading pinned Drake-ROS source (no Git credentials required)..."
  drake_ros_temp_dir="$(mktemp -d)"
  drake_ros_archive="${drake_ros_temp_dir}/drake-ros.tar.gz"
  download \
    "https://github.com/RobotLocomotion/drake-ros/archive/${DRAKE_ROS_COMMIT}.tar.gz" \
    "${drake_ros_archive}" \
    || fail "Could not download Drake-ROS. Check that GitHub is reachable, then rerun."

  mkdir -p "${drake_ros_temp_dir}/source"
  tar -xzf "${drake_ros_archive}" --strip-components=1 \
    -C "${drake_ros_temp_dir}/source"
  [[ -f "${drake_ros_temp_dir}/source/drake_ros/package.xml" ]] || \
    fail "The Drake-ROS archive did not contain the expected ROS package."
  mv "${drake_ros_temp_dir}/source" "${drake_ros_destination}"
  printf '%s\n' "${DRAKE_ROS_COMMIT}" > "${drake_ros_destination}/.robotics_ii_commit"
else
  printf '%s\n' '[6/7] Drake-ROS source already present.'
fi

printf '%s\n' '[7/7] Resolving dependencies and building the workspace...'
cd "${WORKSPACE}"
rosdep install --from-paths src --ignore-src --rosdistro "${COURSE_ROS_DISTRO}" -y \
  --skip-keys drake --skip-keys ament_python
# This is the one-time installation build. Session work must use targeted
# package builds instead of repeating the complete workspace build.
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# Remove any daemon started before the course selected CycloneDDS. The first
# graph command in a sourced course terminal will start it again consistently.
ros2 daemon stop >/dev/null 2>&1 || true

dpkg-query -W -f='${Package}\t${Version}\n' \
  drake-dev ros-dev-tools "ros-${COURSE_ROS_DISTRO}-desktop" \
  "ros-${COURSE_ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${COURSE_ROS_DISTRO}-plotjuggler" \
  > "${REPOSITORY_ROOT}/scripts/installed-versions.tsv"

printf '\nInstallation complete. Verify it with:\n'
printf '  source "%s/scripts/setup_workspace.bash"\n' "${REPOSITORY_ROOT}"
printf '  "%s/scripts/course_check.sh"\n' "${REPOSITORY_ROOT}"
