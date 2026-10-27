# Robotics II course environment for macOS (and any Docker host).
# The dated image tag fixes the Ubuntu 24.04 + ROS 2 Jazzy desktop base.
FROM tiryoh/ros2-desktop-vnc:jazzy-20260830T0953

# Keep these in sync with scripts/install.sh.
ARG DRAKE_VERSION=1.56.0
ARG DRAKE_PACKAGE_VERSION=1.56.0-1
ARG DRAKE_ROS_COMMIT=5bd4e5e8aa7ff072d29f54e258fda446b11c95d2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Match the explicit tools installed by scripts/install.sh. rosdep below
# installs dependencies declared in each current course package manifest.
RUN apt-get update \
 && apt-get install --yes --no-install-recommends \
      ca-certificates curl git \
      libxml2-utils shellcheck \
      ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-plotjuggler ros-jazzy-plotjuggler-ros \
      ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rqt-graph \
      ros-jazzy-tf2-tools ros-jazzy-turtlesim ros-jazzy-xacro \
      ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

RUN arch="$(dpkg --print-architecture)" \
 && curl --fail --location --show-error --silent \
      --output /tmp/drake.deb \
      "https://github.com/RobotLocomotion/drake/releases/download/v${DRAKE_VERSION}/drake-dev_${DRAKE_PACKAGE_VERSION}_${arch}-noble.deb" \
 && apt-get update \
 && apt-get install --yes --no-install-recommends /tmp/drake.deb \
 && rm -f /tmp/drake.deb \
 && rm -rf /var/lib/apt/lists/*

# Build the pinned drake-ros as an underlay once in the image. This keeps it
# out of the student's bind-mounted workspace and avoids rebuilding it there.
ENV DRAKE_ROS_WS=/opt/drake_ros_ws
RUN git clone https://github.com/RobotLocomotion/drake-ros.git "${DRAKE_ROS_WS}/src/drake-ros" \
 && git -C "${DRAKE_ROS_WS}/src/drake-ros" checkout --detach "${DRAKE_ROS_COMMIT}" \
 && apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths "${DRAKE_ROS_WS}/src" --ignore-src --rosdistro jazzy -y \
      --skip-keys drake --skip-keys ament_python \
 && source /opt/ros/jazzy/setup.bash \
 && export PATH="/opt/drake/bin:${PATH}" \
 && export PYTHONPATH="/opt/drake/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}" \
 && cd "${DRAKE_ROS_WS}" \
 && colcon build --cmake-args -DBUILD_TESTING=OFF \
 && rm -rf "${DRAKE_ROS_WS}/build" "${DRAKE_ROS_WS}/log" \
 && rm -rf /var/lib/apt/lists/*

# Include only manifests in the Docker build context. The course source itself
# is mounted at runtime, so source edits do not invalidate this dependency layer.
COPY ros2_ws/src /tmp/course_src
RUN apt-get update \
 && rosdep install --from-paths /tmp/course_src --ignore-src --rosdistro jazzy -y \
      --skip-keys drake --skip-keys drake_ros --skip-keys ament_python \
 && rm -rf /tmp/course_src /var/lib/apt/lists/*

# APT metadata is left available for students who install packages during labs.
RUN apt-get update

# Set up ROS, Drake, drake-ros and the course overlay for each container shell.
COPY docker/course_setup.bash /opt/course_setup.bash
RUN echo 'source /opt/course_setup.bash' >> /home/ubuntu/.bashrc
ENV BASH_ENV=/opt/course_setup.bash

WORKDIR /home/ubuntu/Robotics-II
