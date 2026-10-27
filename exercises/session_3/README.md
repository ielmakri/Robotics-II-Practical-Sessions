# Session 3 exercise — Drake scenes, SDF meshes, and one pick-and-place cycle

This exercise uses a Husky with a Panda arm fixed to its top plate. Treat the
mobile base and arm as two commanded robot subsystems in one simulation. The
exercise is deliberately one cycle: grasp one small supply bin, lift it clear
of the workbench, move the Husky backward by about 0.50 m while the Panda holds
the bin, then place it on the platform. Students still design the
project-specific environment, mission coordinator, route, object flow, and
tests in Sessions 4–6.

Run commands from the repository root unless a step says otherwise. The
simulation needs the supported ROS 2 Jazzy and Drake environment.

## Part A — distinguish simulation and publication rates

In Terminal A:

```bash
source scripts/setup_workspace.bash
ros2 launch sim_systems panda_manipulator_py_launch.py rviz:=true simulation_sec:=inf
```

In Terminal B:

```bash
source scripts/setup_workspace.bash
ros2 node list --no-daemon
ros2 topic list --no-daemon -t
ros2 topic echo --no-daemon /sim_sys/joint_states --once
ros2 topic hz /sim_sys/joint_states
```

Stop `topic hz` with `Ctrl+C`. The plant uses a 0.001 s time step, joint
states publish every 0.02 s, and visualization publishes at about 32 Hz.
Explain why numerical integration, ROS feedback, and RViz use different
rates. If the launch exits, no node or joint-state topic appears, or RViz is
empty, use the [Session 3 graph troubleshooting steps](../../docs/session_3_sim_systems/README.md#11-troubleshooting)
before continuing.

## Part B — parse a scene with two commanded robot subsystems

Check the mounted Panda scene and the two model instances:

```bash
python3 exercises/session_3/check_project_world.py husky_panda_pick_place.sdf
```

The checker should report `husky_runner` and `panda_inspector`. It verifies
that Drake parses and finalizes the scene. It does not test the ROS controller,
the mount pose, or grasping. The two-Husky and dual-Panda SDFs remain parser
examples, not working project launchers.

Open the SDF and find the mount joint, the supply bin, the workbench, the
collision proxy, and the world poses:

```bash
code models/team_projects/husky_panda_pick_place.sdf
```

Record the parent and child links for `panda_mount_joint`. Check that the
Panda base is attached to `husky_runner::top_plate_link`, and explain why a
fixed joint makes the arm move with the mobile base.

## Part C — add an OBJ mesh to an SDF scene

The pick-and-place SDF uses `models/team_projects/assets/workbench.obj` for
visual appearance and a box for collisions. The directory name is lowercase
`assets/`; it already exists and contains the workbench mesh. The Panda URDF is
stored separately under `models/descriptions/robots/arms/panda_fer/urdf/` and
is referenced by the scene SDF. See the
[`project assets guide`](../../models/team_projects/assets/README.md) and the
Session 3 notes on [mesh visuals](../../docs/session_3_sim_systems/README.md#33-add-an-obj-mesh-to-an-sdf-scene)
and [static or dynamic objects](../../docs/session_3_sim_systems/README.md#34-static-and-dynamic-objects).

This keeps the scene detailed enough to understand while keeping contact
calculations simple.

Drake's SDF parser accepts OBJ mesh URIs. If a CAD asset is an STL, import it
into Blender or FreeCAD, set its units and origin, and export a Wavefront OBJ.
Keep OBJ, optional MTL, and texture files together under
`models/team_projects/assets/`. Check scale in metres. Use a simple box or
cylinder as the collision proxy unless mesh contact is required. See Drake's
[mesh format guidance](https://drake.mit.edu/doxygen_cxx/group__geometry__file__formats.html)
and [model parsing reference](https://drake.mit.edu/doxygen_cxx/group__multibody__parsing.html).

In an SDF visual, the URI follows the registered package name:

```xml
<visual name="machine_visual">
  <geometry>
    <mesh>
      <uri>package://robotics_ii_models/team_projects/assets/machine.obj</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</visual>
```

The simulator registers `robotics_ii_models` to the repository's `models/`
directory. Drake in this course requires uniform mesh scaling. Bake the
desired dimensions, including any nonuniform scaling, into the OBJ and use
`<scale>1 1 1</scale>`. Keep collision geometry separate:

```xml
<collision name="simple_collision_proxy">
  <geometry><box><size>0.6 0.4 0.5</size></box></geometry>
</collision>
```

After adding a mesh, check that its file exists, validate the SDF XML, parse
the scene with Drake, and inspect it in RViz. A visible mesh alone does not
prove that the collision shape or inertia is suitable.

## Part D — inspect the object and ROS graph

Build the changed simulator package, then launch the combined scene:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sim_systems
source install/setup.bash
cd ..
```

Run the focused test for the Panda's smoothed joint references:

```bash
python3 ros2_ws/src/sim_systems/test/test_mobile_manipulator_trajectory.py
python3 exercises/session_3/test_vertical_place_path.py
```

In Terminal A:

```bash
source scripts/setup_workspace.bash
ros2 launch sim_systems mobile_manipulator_controlled_py_launch.py \
  scene:=husky_panda_pick_place.sdf rviz:=true simulation_sec:=inf
```

In Terminal B, check both command/feedback loops and inspect the initial
positions:

```bash
source scripts/setup_workspace.bash
ros2 node list --no-daemon
ros2 topic info --no-daemon /husky_runner/cmd_vel --verbose
ros2 topic echo --no-daemon /husky_runner/odom --once
ros2 topic info --no-daemon /panda_inspector/joint_trajectory --verbose
ros2 topic echo --no-daemon /panda_inspector/joint_states --once
ros2 topic info --no-daemon /panda_inspector/gripper_position --verbose
```

The supply bin starts on the mesh workbench. Panda feedback contains seven
arm joints and two finger joints. The gripper command is a one-value array
setting both finger positions: `[0.04]` m opens it and `[0.0]` m closes it.

Test opening and closing while the Husky is stopped:

```bash
ros2 topic pub --once /panda_inspector/gripper_position \
  std_msgs/msg/Float64MultiArray "{data: [0.04]}"
```

```bash
ros2 topic echo --no-daemon /panda_inspector/joint_states --once
```

Before the first command, confirm both fingers start near `0.035` m. Then
send a deliberately out-of-range command:

```bash
ros2 topic pub --once /panda_inspector/gripper_position \
  std_msgs/msg/Float64MultiArray "{data: [0.08]}"
ros2 topic echo --no-daemon /panda_inspector/joint_states --once
```

Both measured finger positions should remain unchanged. Repeat with
`{data: [0.0]}` and confirm both close. An empty array, an array with more than
one value, or a value outside `[0.0, 0.04]` is ignored; until a valid message
arrives, the gripper stays at its open default.

## Part E — transfer one supply bin onto the Husky platform

Keep the Husky stopped while the arm manipulates the bin. The SDF gives the
bin a known pose. Solve hand targets from the scene instead of guessing seven
joint angles. The solver freezes the Husky and object coordinates, keeps the
fingers open, and returns a Panda arm joint target for the requested world
pose of `panda_hand`.

For the pickup, begin with a hand target above the tote. Its center is
`(-0.20, -0.55, 0.7675)` metres. With the example tool orientation, the
gripper fingers extend about 0.058–0.112 m below `panda_hand`. Start above the
tote:

```bash
python3 exercises/session_3/solve_pick_place_ik.py -0.20 -0.55 0.9475
```

The helper constrains the hand pose and Panda joint limits; it does not plan a
collision-free path. Inspect each move in RViz and stop if the arm touches the
workbench, Husky, or object before contact. If the solution is not reachable,
change the hand height or approach orientation and solve again. If the home
arm posture would put a joint within 0.05 rad of a limit, the helper retries
alternate arm seeds and returns the reachable solution with the best limit
margin. It still does not check collisions. The controller smoothly
interpolates each arm command from measured joint positions using a quintic
profile. It chooses a duration that limits reference speed to at most 1 rad/s
and acceleration to at most 2 rad/s², with a 1.2 s minimum. A positive
`time_from_start` may make the move longer. This smooths joint motion; it does
not make the IK path collision-free. Execute approach, grasp, lift, and place
as separate stages and wait for the measured joints to settle between them.
Send the returned seven values as a position setpoint:

```bash
ros2 topic pub --once /panda_inspector/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [PASTE_THE_SEVEN_VALUES]}]}"
```

Wait for the measured joints to settle. Solve the lower grasp target so the
fingers overlap the tote walls:

```bash
python3 exercises/session_3/solve_pick_place_ik.py -0.20 -0.55 0.8655
```

Send the returned seven positions with the same trajectory command. Close the
gripper:

```bash
ros2 topic pub --once /panda_inspector/gripper_position \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

Lift back to the approach pose and check that the tote rises with the fingers:

```bash
python3 exercises/session_3/solve_pick_place_ik.py -0.20 -0.55 0.9475
```

Send the seven returned positions with the `ros2 topic pub` command above. If
the tote slips, check the finger contact, object width, collision geometry,
and contact friction; do not attach it to the hand with a fixed joint.

### Move the Husky backward before placing the tote

After the tote is securely held by the Panda and lifted clear of the workbench,
keep the gripper closed. Record the starting odometry:

```bash
ros2 topic echo --no-daemon /husky_runner/odom --once
```

In a second terminal, watch odometry continuously:

```bash
ros2 topic echo --no-daemon /husky_runner/odom
```

In the command terminal, move the Husky backward while it is holding the tote:

```bash
ros2 topic pub -r 10 /husky_runner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.08}, angular: {z: 0.0}}"
```

In this scene the Husky starts with yaw 0, so its odometry X should decrease by
about 0.50 m. At 0.08 m/s this takes about 6.25 seconds, but use the odometry
displacement to decide when to stop. Press `Ctrl+C` when X has decreased by
about 0.50 m, then send a stop command:

```bash
ros2 topic pub --once /husky_runner/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Confirm that the Husky has stopped and that the measured displacement is about
0.50 m before continuing. Keep the gripper closed while positioning the tote.

#### Place the tote while keeping it upright

The Husky top-plate surface is at z=0.38363 m. The tote center will be at
z=0.42113 m when it rests on the plate. The solver uses the Panda's pose in the
original SDF, before the Husky moves; its output is a joint target for the arm
mounted on the base. After the Husky retreats, that same joint target moves
with the platform.

Keep the gripper closed and the Husky stopped. Run the placement helper:

```bash
python3 exercises/session_3/execute_vertical_place.py
```

It computes a 32-step IK path from the lifted tote pose directly to the final
placement pose. The Panda hand stays vertical, so the tote stays upright as
long as the grasp holds. The planned tote pose remains at least 5 mm above the
plate until the final target. The helper sends each joint target only after
measured joints settle. It stops if the arm is not at the expected lifted
pose, feedback becomes stale, or a target is not reached in time. Inspect the
motion in RViz; Drake checks the waypoints but does not plan a collision-free
path. The modeled joint interpolation between waypoints deviates less than 2
degrees from vertical in this scene.

The final hand target is z=0.51913 m. It sits about 0.098 m above the tote
center when the tote rests on the plate because that is the grasp offset. This
is the final placement pose, not a separate pre-place waypoint. After the
helper reports that it reached the final pose, open the gripper:

```bash
ros2 topic pub --once /panda_inspector/gripper_position \
  std_msgs/msg/Float64MultiArray "{data: [0.04]}"
```

Confirm in RViz and with a new joint-state message that the tote rests on the
platform. The 0.50 m Husky retreat happens before placement to clear the
workbench and the held tote.

This cycle demonstrates one known object transfer and a short coordinated
base movement. It does not complete a baggage, pharmacy, palletizing, machine
tending, or other team project. Those require each team to design and
implement its own SDF, mission logic, object flow, error handling, and
evaluation.

## Checkpoint

Keep evidence of:

- a parsed SDF with both robot model instances and the Panda mount joint;
- the workbench OBJ and its simple box collision proxy;
- one accepted and one rejected gripper command;
- a grasp, lift, placement, release, and measured joint-state result;
- a short Husky movement with changed odometry and the object still on the
  platform;
- the launch/build commands and one deliberate timeout or stale-feedback
  observation.
