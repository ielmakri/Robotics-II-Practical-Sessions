# Setup, build and recovery

Complete this page before Session 1. Do not spend laboratory time debugging a
half-installed operating system.

## 1. Requirements

- Ubuntu 24.04, either native or in WSL2.
- At least 15 GB of free space and a stable internet connection.
- A regular Ubuntu account with `sudo` access.
- This repository stored in the Linux filesystem, preferably
  `~/git/Robotics-II`.

On macOS, use the [Docker setup](#macos-docker) at the end of this page instead
of running the Ubuntu installer.

For WSL2, install Ubuntu from an **Administrator PowerShell**:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart Windows if requested, open Ubuntu 24.04, and create the Linux username
and password. The password is not displayed while typing in a terminal; that is
normal.

### Install Visual Studio Code

Visual Studio Code is the recommended editor for this course. When using WSL2,
install VS Code on **Windows**, not inside Ubuntu.

Open a regular Windows PowerShell and run:

```powershell
winget install --id Microsoft.VisualStudioCode -e
```

If `winget` is not available, download the Windows installer from the official
[Visual Studio Code download page](https://code.visualstudio.com/download).

After installing VS Code:

1. Open VS Code on Windows.
2. Open **Extensions** with `Ctrl+Shift+X`.
3. Search for **WSL** and install the extension published by Microsoft.
4. Close and reopen the Ubuntu terminal.
5. Check that the command is available:

   ```bash
   code --version
   ```

After cloning the course repository in the next section, open it from WSL with:

```bash
cd ~/git/Robotics-II
code .
```

VS Code should open the Linux copy of the repository. The lower-left corner of
the window should indicate that the folder is connected to WSL. Do not edit a
second copy of the repository stored under `/mnt/c`.

On native Ubuntu, install the Linux version from the same official download
page. On macOS, install VS Code on macOS and follow the Docker setup at the end
of this page for the ROS environment.

## 2. Clone the public course repository

The course repository is public. Clone it over HTTPS; you do not need a
GitHub account, password, token or SSH key to download the course materials:

```bash
mkdir -p ~/git
git clone https://github.com/ielmakri/Robotics-II-Practical-Sessions.git ~/git/Robotics-II
cd ~/git/Robotics-II
```

## 3. Install

```bash
cd ~/git/Robotics-II
./scripts/install.sh
```

The script installs ROS 2 Jazzy, Drake, visualization/development tools and a
pinned Drake-ROS snapshot, then builds the workspace. Drake-ROS is downloaded as
an archive rather than cloned, so no GitHub password or token is used during the
installation.

When installation completes:

```bash
source ~/git/Robotics-II/scripts/setup_workspace.bash
./scripts/course_check.sh
```

All lines should start with `PASS`.

## 4. Understand the workspace overlay

ROS searches packages in sourced environments. The first setup file exposes the
system ROS installation; the second exposes packages built in this repository:

```text
/opt/ros/jazzy/setup.bash
          ↓ underlay
ros2_ws/install/setup.bash
          ↓ overlay
robotics_ii_tutorials, robotics_ii_description, sim_systems, ...
```

`scripts/setup_workspace.bash` sources both in the correct order. Run it in
**every new terminal**. It intentionally does not modify `.bashrc` so that the
active environment is always explicit.

The same script also selects CycloneDDS for ROS communication:

```bash
echo "$RMW_IMPLEMENTATION"
```

The expected result is `rmw_cyclonedds_cpp`. Every ROS node in an exercise must
be started from a terminal where the course environment was sourced. If the
middleware is changed, stop and restart all running ROS nodes; an existing
process cannot change middleware after it has started.

Students do not normally need to start or restart the ROS daemon. The installer
removes an older daemon once, and ROS starts a new one automatically when it is
needed. Restart the daemon manually only when following a troubleshooting step.

## 5. Rebuild after editing

Do not rebuild the complete workspace after every edit. That can rebuild
`drake_ros` and take more than 30 minutes. Build only the package(s) affected
by the change. For the Session 2 URDF/RViz work, use:

```bash
cd ~/git/Robotics-II/ros2_ws
colcon build --symlink-install \
  --packages-select urdf_visualizer robotics_ii_description
source ../scripts/setup_workspace.bash
```

With `--symlink-install`, changes inside installed Python modules are usually
visible without a rebuild. Rebuild after changing `setup.py`, `package.xml`,
launch/configuration files, entry points or package data. Markdown and SVG
changes do not require a build.

## Recovery table

| Symptom | Diagnosis | Recovery |
| --- | --- | --- |
| `Package '...' not found` | The overlay is not sourced or build failed. | Source `scripts/setup_workspace.bash`; if still absent, rebuild and read the first build error. |
| `No module named rclpy` | Python is outside the ROS environment. | Source the workspace; use the Ubuntu `python3`, not a foreign Conda environment. |
| `No module named pydrake` | Drake Python path is missing. | Source `scripts/setup_workspace.bash`, then rerun `course_check.sh`. |
| `HTTP 401 ... expected flush after ref listing` | A Git operation is using invalid HTTPS credentials. | The new installer does not Git-clone Drake-ROS. Pull this version of `scripts/install.sh`, remove or move only an incomplete `ros2_ws/src/drake-ros`, and rerun. |
| RViz opens but the robot is absent | Wrong fixed frame, missing TF or bad mesh path. | Follow the Session 2 TF checks; inspect RViz's Displays panel for red error text. |
| Build output is confusing | Several packages failed after the first error. | Rerun with `colcon build --event-handlers console_direct+` and fix the first error only. |
| WSL is extremely slow | Repository is under `/mnt/c`. | Re-clone under `~/git`; do not move generated build directories between filesystems. |

## Clean rebuild

A clean rebuild of the complete workspace is an installation-recovery step,
not a normal student workflow. Do not remove `build`, `install` and `log` just
to work on Session 2. If the initial installation is broken, rerun the course
installer or contact the teaching team instead of rebuilding Drake-ROS during
the practical.

## Setup exit check

```bash
source ~/git/Robotics-II/scripts/setup_workspace.bash
ros2 pkg list | grep -E '^(robotics_ii_tutorials|robotics_ii_description|sim_systems)$'
python3 -c "import rclpy, pydrake, drake_ros; print('Python environment OK')"
```

Expected: the three package names and `Python environment OK`.

## macOS (Docker)

The course runs inside an Ubuntu 24.04 / ROS 2 Jazzy container on macOS. Install
Docker Desktop for Mac and give Docker at least **8 GB of memory** in
**Settings → Resources**. The image supports Intel and Apple silicon. The first
build downloads several gigabytes and compiles the pinned Drake-ROS underlay;
later starts reuse the image.

Clone the public course repository over HTTPS, then start the container from
the repository root:

```bash
mkdir -p ~/git
git clone https://github.com/ielmakri/Robotics-II-Practical-Sessions.git ~/git/Robotics-II
cd ~/git/Robotics-II
docker compose up -d --build
```

Open <http://localhost:6080> in a browser. In the remote desktop, open
**Menu → Terminal Emulator**, then build and verify the course workspace:

```bash
cd ~/Robotics-II/ros2_ws
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
cd ~/Robotics-II
./scripts/course_check.sh
```

All `course_check.sh` lines should start with `PASS`. The terminal environment
automatically sources ROS 2, CycloneDDS, Drake and the pinned Drake-ROS
underlay. After the build, run a simulation in that terminal; RViz appears in
the remote desktop. The repository is mounted into the container, so edits on
the Mac are visible there without copying files.

Use these commands from the repository root to start and stop the environment:

```bash
docker compose up -d
docker compose down
```

The browser desktop uses port **6080**; Drake Meshcat uses **7000**. If either
port is occupied, change the left-hand port number in `compose.yaml`. The
container does not change the Ubuntu installation instructions above.
