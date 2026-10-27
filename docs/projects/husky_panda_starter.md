# Husky-Panda mobile manipulator starter

The supported project platform is a Husky mobile base with a Panda arm mounted
on its top plate. We treat the base and the manipulator as two actuated robot
subsystems: each has its own command and measured feedback. Together they form
the mobile-manipulator system used by all seven project briefs.

The starter provides low-level wheel, arm, and gripper commands. A team still
creates its own SDF layout, ROS 2 mission coordinator, task-specific object
flow, safe-state logic, and evaluation. The one small object transfer in the
[Session 3 exercise](../../exercises/session_3/README.md) is a platform test,
not a completed project.

Run commands from the repository root unless a step says otherwise.

## Build and launch

```bash
source scripts/setup_workspace.bash
cd ros2_ws
colcon build --symlink-install --packages-select sim_systems
cd ..
source scripts/setup_workspace.bash
ros2 launch sim_systems mobile_manipulator_controlled_py_launch.py \
  rviz:=true simulation_sec:=inf
```

The default scene is `models/team_projects/husky_panda_pick_place.sdf`. It
mounts `panda_inspector::panda_link0` to
`husky_runner::top_plate_link`, adds a small dynamic supply bin, and uses an
OBJ workbench visual with a box collision proxy. Parse the scene directly with
Drake before editing it:

```bash
python3 exercises/session_3/check_project_world.py husky_panda_pick_place.sdf
```

## Check the command and feedback topics

In another terminal:

```bash
source scripts/setup_workspace.bash
ros2 node list --no-daemon
ros2 topic list --no-daemon -t
ros2 topic echo --no-daemon /husky_runner/odom --once
ros2 topic echo --no-daemon /panda_inspector/joint_states --once
```

The Husky accepts planar velocity commands on `/husky_runner/cmd_vel`; a guard
limits speed and stops the base after 0.5 s without a fresh command. The Panda
accepts named seven-joint position targets on
`/panda_inspector/joint_trajectory`. The final point is used as a setpoint;
`time_from_start` is not interpolated. Gripper position is commanded on
`/panda_inspector/gripper_position` as a one-entry
`std_msgs/msg/Float64MultiArray`: `[0.04]` m opens both fingers and `[0.0]` m
closes them. `/panda_inspector/joint_states` reports all seven arm and two
finger joints.

Example bounded base command:

```bash
ros2 topic pub --once /husky_runner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Example gripper-open command:

```bash
ros2 topic pub --once /panda_inspector/gripper_position \
  std_msgs/msg/Float64MultiArray "{data: [0.04]}"
```

For the full pick, lift, place, release, and short base-motion cycle, follow
the [Session 3 exercise](../../exercises/session_3/README.md). It includes an
IK helper so students can generate reachable arm setpoints from world-space
hand targets.

## Create a team environment

Copy the mobile-manipulator scene into a new SDF and modify that copy:

```bash
cp models/team_projects/husky_panda_pick_place.sdf \
  models/team_projects/team_01_world.sdf
code models/team_projects/team_01_world.sdf
```

Keep the model names `husky_runner` and `panda_inspector` so the supplied
interfaces continue to resolve. Add task-specific shelves, conveyors,
cabinets, pallets, machines, trays, stations, and collision geometry. To add a
CAD visual, convert STL to OBJ and reference the OBJ with the
`robotics_ii_models` package URI shown in the [SDF mesh tutorial](../session_3_sim_systems/README.md#add-an-obj-mesh-to-an-sdf-scene).
Use a simple box or cylinder as the collision proxy when detailed mesh contact
is not needed.

Check the new world and pass its filename to the launch file:

```bash
python3 exercises/session_3/check_project_world.py team_01_world.sdf
ros2 launch sim_systems mobile_manipulator_controlled_py_launch.py \
  scene:=team_01_world.sdf rviz:=true
```

## Implement the mission logic

Create a team-owned ROS 2 coordinator that subscribes to measured Husky
odometry and Panda joint states, then publishes commands to the base and arm.
For object tasks, also command the gripper and verify finger feedback. Use
measured state to advance mission phases; do not advance on a fixed delay alone.

Before coding, list the mission states, each transition condition, a timeout,
and the safe response to stale feedback. A typical work cycle stops and
confirms the base, moves the Panda, checks the measured arm target, and only
then allows the base to move again. Teams add the route, task-specific object
flow, recovery behavior, and performance measures required by their selected
[project brief](README.md).
