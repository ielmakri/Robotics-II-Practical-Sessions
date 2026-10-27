# Robot interfaces and capabilities

This page describes the ROS interfaces implemented by the course examples.
Verify live interfaces with `ros2 node info --no-daemon`,
`ros2 topic list --no-daemon -t`, and
`ros2 topic info --no-daemon /TOPIC --verbose`.

## Implemented ROS interfaces

| Example | Direction | Topic | Type | Meaning |
| --- | --- | --- | --- | --- |
| Controlled Panda | input | `/panda/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Named seven-joint setpoint; the current controller uses the final point. |
| Husky-Panda mobile manipulator | input | `/husky_runner/cmd_vel` | `geometry_msgs/msg/Twist` | Planar velocity command; capped and stopped after a 0.5 s command timeout. |
| Husky-Panda mobile manipulator | output | `/husky_runner/odom` | `nav_msgs/msg/Odometry` | Measured Husky pose in `world` and body velocity in `husky_runner/base_link`. |
| Husky-Panda mobile manipulator | input | `/panda_inspector/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Named seven-arm-joint position setpoint; invalid targets leave the last valid target unchanged. |
| Husky-Panda mobile manipulator | input | `/panda_inspector/gripper_position` | `std_msgs/msg/Float64MultiArray` | One value for each finger in metres: 0.0 closed, 0.04 open; empty and incorrectly sized arrays are ignored. |
| Husky-Panda mobile manipulator | output | `/panda_inspector/joint_states` | `sensor_msgs/msg/JointState` | Measured seven arm and two finger joint positions and velocities. |
| Controlled Panda | output | `/sim_sys/desired_positions` | `std_msgs/msg/Float64MultiArray` | Validated nine-position target for plotting and diagnostics. |
| Panda examples | output | `/sim_sys/joint_states` | `sensor_msgs/msg/JointState` | Seven arm and two finger joint states at 50 Hz. |
| Panda examples | output | `/sim_sys/joint_acceleration` | `std_msgs/msg/Float64MultiArray` | Nine generalized accelerations at 50 Hz. |
| All Drake examples | output | `/clock` | `rosgraph_msgs/msg/Clock` | Drake simulation time. |
| All Drake examples | output | `/tf` and visualization topics | TF and visualization messages | Scene frames and geometry for RViz. |
| Session 1 example | output | `/tutorial/joint_states` | `sensor_msgs/msg/JointState` | Synthetic one-joint feedback. |
| Session 1 monitor | input/output | `/tutorial/joint_states`, `/tutorial/limit_warning` | `sensor_msgs/msg/JointState`, `std_msgs/msg/Bool` | Example processing and derived safety flag. |

The supported project and Session 3 platform is the Husky-Panda mobile
manipulator. Its mobile base and mounted arm are distinct actuated subsystems
in one Drake plant, with separate command and measured-feedback topics. The
Husky command guard bridges `/husky_runner/cmd_vel` to the simulator's internal
`/husky_runner/cmd_vel_safe` topic.

## Project scenes and available code

| Platform or scene | Model assets in repository | Existing control support |
| --- | --- | --- |
| Panda | Arm-and-hand URDF with seven arm joints and two finger joints. | One Panda trajectory subscriber, joint-space controller, joint-state feedback, desired-position and acceleration publishers. |
| Husky | Husky URDF, wheel geometry, floor SDF. | Single-robot visualization example; no mobile command or odometry in that standalone launch. |
| Husky-Panda mobile manipulator | `models/team_projects/husky_panda_pick_place.sdf` mounts the Panda on the Husky top plate. | `mobile_manipulator_controlled_py_launch.py` controls Husky wheels, Panda joints, and gripper; publishes odometry and joint feedback in one plant. The SDF includes one dynamic supply bin and an OBJ workbench visual. |
| Legacy fixed-Panda composition | `models/team_projects/husky_panda_inspection.sdf` places a mobile Husky and a world-fixed Panda in one scene. | Mixed command and feedback starter remains available; retained as a parsing/controller example, not the project baseline. |
| Two-Husky aisle scene | Two Husky URDF instances, floor, box walls, start areas, and a shared-lane marker in `models/team_projects/two_husky_corridor.sdf`. | SDF scene only; no supplied two-Husky control or coordinator. |
| Dual-Panda panel scene | Two Panda arm-and-hand URDF instances, a panel, markers, and a keep-clear strip in `models/team_projects/dual_panda_inspection.sdf`. | SDF scene only; the single-Panda controller is not routed to both arms. |

The project scenes use repository geometry and do not need external airport,
warehouse, or hospital asset packages. The parser check in
`exercises/session_3/check_project_world.py` verifies model names in a finalized
plant; it does not start controllers or prove that an object can be grasped.

## Panda command contracts

A valid arm command contains each name `panda_joint1` through `panda_joint7`
exactly once, includes at least one trajectory point, and gives finite
positions within the Panda URDF limits. The mixed Husky-Panda controller uses
the final point as a joint-space setpoint, ignores requested velocities, and
holds the last valid target if a new command is malformed or outside limits.
It does not interpolate `time_from_start`.

The gripper command is a separate one-value `Float64MultiArray` target shared
by both fingers. The adapter accepts values in `[0.0, 0.04]` metres and
retains its previous target for an empty, incorrectly sized, out-of-range, or
non-finite command. This interface commands finger
position; a working grasp still depends on the SDF contact geometry, object
mass, friction, and motion. Test contact and object transfer in the supported
Drake runtime before relying on them.
