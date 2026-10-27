# `sim_systems`

This package contains the executable Drake examples for Session 3. The main
teaching file is `sim_systems/panda_manipulator.py`; read it line-by-line before
opening the controlled, fixed-base, or mobile-manipulator variants.

Run the launch commands from the repository root. Source the workspace in each
terminal that uses ROS 2 commands.

## Launches

```bash
source scripts/setup_workspace.bash

ros2 launch sim_systems panda_manipulator_py_launch.py
ros2 launch sim_systems panda_manipulator_controller_py_launch.py
ros2 launch sim_systems husky_py_launch.py
ros2 launch sim_systems mobile_manipulator_py_launch.py
ros2 launch sim_systems mobile_manipulator_controlled_py_launch.py
```

All launches accept:

```bash
simulation_sec:=0.5   # finite smoke test; default: inf, stop with Ctrl+C
realtime_rate:=1.0    # requested simulator rate
rviz:=false           # useful for headless smoke tests
```

## ROS contract

The controlled Panda subscribes to
`/panda/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) and publishes
`/sim_sys/joint_states` (`sensor_msgs/msg/JointState`) plus
`/sim_sys/joint_acceleration` (`std_msgs/msg/Float64MultiArray`).
It also publishes the validated target positions on
`/sim_sys/desired_positions` for PlotJuggler; that mirror is not a second
command interface.

The loaded model has nine actuated positions: seven arm joints and two finger
joints. The Session 1 command is intentionally a named seven-arm-joint setpoint.
The adapter validates names, finite positions and Panda URDF limits, then uses
the final point with zero velocity when no velocity array is provided. It holds
the safe home posture, including an open gripper, until a valid arm command is
received. `/sim_sys/joint_states` reports all nine joints. `time_from_start` is
intentionally not interpreted in this baseline; timed interpolation is an
extension.

The Husky-Panda mobile-manipulator starter defaults to
`models/team_projects/husky_panda_pick_place.sdf`. Its SDF fixes the Panda base
to the Husky top plate. It accepts
`/husky_runner/cmd_vel` (`geometry_msgs/msg/Twist`) and
`/panda_inspector/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).
Measured feedback is published on `/husky_runner/odom`
(`nav_msgs/msg/Odometry`) and `/panda_inspector/joint_states`
(`sensor_msgs/msg/JointState`). A command guard limits Husky speed and stops it
when commands are stale. The Panda trajectory adapter uses the final valid
point as a joint-space position setpoint, ignores requested velocities, and
keeps the last valid target if a new command is malformed or outside joint
limits. A separate bounded target on
`/panda_inspector/gripper_position` (`std_msgs/msg/Float64MultiArray`) commands
both finger joints with a one-value array: `[0.04]` m is open and `[0.0]` m is
closed. Empty or incorrectly sized arrays are ignored. The measured finger
positions are included in `/panda_inspector/joint_states`. The adapter does
not interpolate time. Panda gravity compensation is sampled at 50 Hz and
held between updates.

For the Session 3 object-transfer exercise, launch the same controller with
the mounted scene:

```bash
ros2 launch sim_systems mobile_manipulator_controlled_py_launch.py \
  scene:=husky_panda_pick_place.sdf rviz:=true
```

The SDF fixes the Panda base to the Husky top plate and adds a dynamic supply
bin, workbench, mesh visual, and collision proxy. This is one small known-pose
pick-and-place cycle, not an autonomous project solution.

To use a team SDF, copy the starter world into `models/team_projects/`, retain
the model instance names `husky_runner` and `panda_inspector`, and pass its
filename with `scene:=team_01_world.sdf`.

```bash
ros2 topic echo --no-daemon /sim_sys/joint_states --once
ros2 topic info --no-daemon /panda/joint_trajectory --verbose
ros2 topic hz /sim_sys/joint_states
```

## Source organization

- `course_utils.py`: repository path and responsive finite-step simulation loop;
- `panda_model.py`: shared arm/hand names, limits, home pose and gains;
- `ros_adapters.py`: typed Drake-vector ↔ ROS-message adapters;
- `panda_manipulator.py`: baseline scene, constant home-pose hold controller,
  and feedback publishers;
- `panda_manipulator_controlled.py`: safe command adapter and
  `JointStiffnessController`;
- `husky.py`: free-base Husky and floor scene;
- `mobile_manipulator.py`: visualization-only SDF-composed mobile manipulator;
- `mobile_manipulator_controlled.py`: Husky wheel, Panda arm, and Panda gripper
  control in one plant; its `scene` argument selects the mounted starter or an
  alternate SDF scene;
- `mobile_manipulator_adapters.py`: Twist-to-wheel-rate, odometry,
  gravity-compensation, and joint-vector adapters;
- `husky_command_guard.py`: velocity limits and a stale-command stop.

The mixed project starter supplies low-level command and feedback paths.
Students build their own SDF, task-specific state machine, route and object
flow, recovery behavior, and measurements. See `docs/reference/interfaces.md`
and `docs/projects/README.md`.

## Debugging checklist

1. Source `scripts/setup_workspace.bash`.
2. Run a finite smoke test with `rviz:=false`.
3. Check `ros2 node list --no-daemon`, `ros2 topic list --no-daemon -t` and `ros2 topic hz`.
4. Only then open RViz and inspect the fixed frame and TF tree.
5. After source changes, rebuild with `colcon build --symlink-install`.
