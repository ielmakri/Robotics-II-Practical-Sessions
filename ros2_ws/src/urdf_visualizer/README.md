# `urdf_visualizer`

This small ROS 2 package starts `joint_state_publisher_gui`,
`robot_state_publisher` and RViz for one URDF or Xacro file. It is the visual
laboratory tool for Session 2; it is not a dynamics simulator.

```bash
source ~/git/Robotics-II/scripts/setup_workspace.bash
ros2 launch urdf_visualizer robot_visualizer.launch.py \
  urdf:=$HOME/git/Robotics-II/ros2_ws/src/robotics_ii_description/urdf/tutorial_robot.urdf

# Xacro is expanded automatically by the launch file.
ros2 launch urdf_visualizer robot_visualizer.launch.py \
  urdf:=$HOME/git/Robotics-II/ros2_ws/src/robotics_ii_description/urdf/tutorial_robot.urdf.xacro
```

The default RViz fixed frame is `world`. For a model with another root frame,
pass it explicitly, for example `fixed_frame:=panda_link0`.

Relative mesh paths are resolved against the model directory and converted to
`file://` URIs before the robot-state-publisher receives the description. The
launch file prints a warning when a mesh cannot be found.
