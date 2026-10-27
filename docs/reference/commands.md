# Command cheat sheet

## Environment and build

```bash
source ~/git/Robotics-II/scripts/setup_workspace.bash
cd ~/git/Robotics-II/ros2_ws
colcon list
# Session 2 URDF/RViz changes only
colcon build --symlink-install \
  --packages-select urdf_visualizer robotics_ii_description
source ../scripts/setup_workspace.bash
```

## ROS graph

For graph inspection in this course, use `--no-daemon` with node and topic
list, info, and echo commands. The `topic hz` and `topic bw` measurement
commands do not accept this option.

```bash
ros2 node list --no-daemon
ros2 node info --no-daemon /NODE_NAME
ros2 topic list --no-daemon -t
ros2 topic info --no-daemon /TOPIC --verbose
ros2 interface show sensor_msgs/msg/JointState
rqt_graph
```

## Topics

```bash
ros2 topic echo --no-daemon /TOPIC
ros2 topic echo --no-daemon /TOPIC --once
ros2 topic hz /TOPIC
ros2 topic bw /TOPIC
ros2 topic pub --once /TOPIC std_msgs/msg/Bool '{data: true}'
```

## Parameters

```bash
ros2 param list /NODE_NAME
ros2 param describe /NODE_NAME PARAMETER
ros2 param get /NODE_NAME PARAMETER
ros2 param set /NODE_NAME PARAMETER VALUE
ros2 param dump /NODE_NAME
```

## Launch and remapping

```bash
ros2 launch PACKAGE FILE.launch.py
ros2 launch PACKAGE FILE.launch.py --show-args
ros2 run PACKAGE EXECUTABLE --ros-args -r old_topic:=new_topic
ros2 run PACKAGE EXECUTABLE --ros-args -p name:=value
```

## TF and URDF

```bash
check_urdf robot.urdf
ros2 run xacro xacro robot.urdf.xacro > /tmp/robot.urdf
ros2 run tf2_ros tf2_echo world tool0
ros2 run tf2_tools view_frames
```

## Git during a project

```bash
git status --short
git diff
git add path/to/file
git diff --cached --stat
git commit -m "Explain one coherent change"
git push -u origin YOUR_BRANCH
```

Press `q` to leave a Git pager. Press `Ctrl+C` to stop a ROS process.
