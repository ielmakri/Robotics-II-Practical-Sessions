# Robotics II — ROS 2 and Drake

This repository is the student workspace for a six-session Robotics II course.
The first three sessions are guided laboratories; the final three sessions are
reserved for a multi-robot team project. Every team controls a Husky-Panda
mobile manipulator: a Husky base carrying a mounted Panda arm, with distinct
base and arm commands and measured feedback.

## Course path

| Session | Main question | Student outcome |
| --- | --- | --- |
| [1 — ROS 2 foundations](docs/session_1_ros2/README.md) | How do independent robot programs exchange commands and feedback? | Build and inspect ROS 2 nodes, then program a feedback-driven Panda sequence. |
| [2 — URDF, TF and RViz](docs/session_2_urdf_tf/README.md) | How is robot geometry and a kinematic tree represented? | Build and debug a two-joint URDF/Xacro model and its TF tree. |
| [3 — Drake simulation systems](docs/session_3_sim_systems/README.md) | How do URDF and SDF models become a ROS-connected Drake simulation? | Trace Drake systems and ports, inspect SDF worlds, and extend a Panda scene. |
| [4 — Mobile-manipulator baseline](docs/projects/README.md) | How do the base and mounted arm command and measure separate motion? | Load the SDF scene and establish the Husky and Panda interfaces. |
| [5 — Coordination](docs/projects/README.md) | How do robots safely share tasks or space? | Implement and test a coordinator and safety behavior. |
| [6 — Validation](docs/projects/README.md) | Does the task work repeatedly and measurably? | Run trials, report metrics, and demonstrate the complete ROS graph. |

Each student guide contains the concepts, code paths, exercises, expected
observations, and troubleshooting needed for the course.

## Supported environment

The reproducible course baseline is:

| Component | Course version |
| --- | --- |
| Student host | Ubuntu 24.04 native/WSL2, or macOS through Docker Desktop |
| Ubuntu | 24.04 (Noble), native or WSL2 |
| ROS 2 | Jazzy Jalisco |
| Drake | 1.56.0 (`drake-dev` 1.56.0-1) |
| Drake-ROS | commit `5bd4e5e8aa7ff072d29f54e258fda446b11c95d2` |
| Visualization | RViz2 and PlotJuggler |

The installer supports Ubuntu 24.04 and ROS 2 Jazzy. On macOS, use the
[Docker Desktop setup](docs/00_setup.md#macos-docker), which runs the same Linux
environment in a container. On WSL2, keep this repository in the Linux
filesystem (for example `~/git/Robotics-II`), not under `/mnt/c`.

## Install once

The commands below install the environment on Ubuntu. On macOS, use the
[Docker Desktop setup](docs/00_setup.md#macos-docker) instead.

The course repository is public and can be cloned over HTTPS without a GitHub
account or SSH key.

```bash
mkdir -p ~/git
git clone https://github.com/ielmakri/Robotics-II-Practical-Sessions.git ~/git/Robotics-II
cd ~/git/Robotics-II
./scripts/install.sh
```

The installer requests the Ubuntu password for system packages. It does **not**
edit `.bashrc`. Drake-ROS is downloaded as a pinned source archive, so the
installation does not require GitHub authentication.

After installation, open a new terminal and run:

```bash
source ~/git/Robotics-II/scripts/setup_workspace.bash
./scripts/course_check.sh
```

Read [setup and recovery](docs/00_setup.md) before Session 1. It includes the
WSL2 and macOS procedures, rebuild commands and fixes for the most common
failures.

## Everyday commands

```bash
# In every new terminal
source ~/git/Robotics-II/scripts/setup_workspace.bash

# After changing files, rebuild only the package(s) affected by the change.
# For Session 2 URDF/RViz work:
cd ~/git/Robotics-II/ros2_ws
colcon build --symlink-install \
  --packages-select urdf_visualizer robotics_ii_description
source ../scripts/setup_workspace.bash

# Confirm that the teaching packages are visible
ros2 pkg prefix robotics_ii_tutorials
ros2 pkg prefix robotics_ii_description
ros2 pkg prefix sim_systems
```

## Repository map

```text
docs/                         student course and project guides
exercises/                    editable starter files and submission checklists
models/                       robot descriptions, objects, and team-project scenes
ros2_ws/src/
  robotics_ii_tutorials/      Session 1 ROS 2 Python examples
  robotics_ii_description/    Session 2 URDF/Xacro examples
  sim_systems/                Session 3 Drake-ROS simulations
  urdf_visualizer/            generic URDF + TF + RViz launcher
scripts/                      installation, environment and diagnostic tools
docker/                       macOS Docker image and shell environment
compose.yaml                  browser desktop and Meshcat ports
```

Generated `build/`, `install/` and `log/` directories are local build products
and must not be committed.

## Robot interfaces and honest capability boundary

The controlled Panda example accepts a standard
`trajectory_msgs/msg/JointTrajectory` on `/panda/joint_trajectory` and publishes
`sensor_msgs/msg/JointState` on `/sim_sys/joint_states`. The supplied model now
contains the Franka hand and two actuated finger joints. The Session 1 command
interface intentionally addresses only the seven arm joints and holds the
fingers open. The Husky-Panda mobile-manipulator simulation adds a separate,
validated gripper-position command.

The single-Panda examples provide joint-state and acceleration feedback; the
controlled Panda accepts named joint targets and includes a gravity-compensated
joint-space controller. The Husky-Panda mobile manipulator mounts the Panda on
the Husky top plate and provides base, arm, and gripper commands with measured
odometry and joint feedback in one Drake simulation. Teams build a distinct
SDF workcell and implement the mission coordinator, route, object flow,
recovery behavior, and task measurements. The two-Husky and dual-Panda SDFs
are model-loading examples without matching multi-robot controllers. See the
[seven project briefs](docs/projects/README.md) and
[interface and capability table](docs/reference/interfaces.md).

## Primary references

Course pages link directly to the relevant official documentation. Start with
the [ROS 2 Jazzy tutorials](https://docs.ros.org/en/jazzy/Tutorials.html), the
[Drake Python documentation](https://drake.mit.edu/pydrake/) and the
[Drake-ROS repository](https://github.com/RobotLocomotion/drake-ros). A curated
list is kept in [sources and further study](docs/reference/sources.md).
