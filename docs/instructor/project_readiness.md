# Instructor note — multi-robot project readiness

This note describes the supported interfaces and the checks to run in the
course image before assigning the projects. A parser success or a static test
is not evidence that an object can be grasped or that a mobile base moves.

## Platform and project briefs

The project guide contains seven distinct operational briefs: airport baggage
sorting, hospital pharmacy replenishment, palletizing, CNC machine tending,
warehouse kitting, airport turnaround equipment, and laboratory batch
handling. Every brief requires the Husky mobile base and Panda arm to receive
commands, publish measured feedback, and participate in the task.

Teams create their own SDF scenes. The project guide does not supply seven
finished worlds or mission coordinators. The supported platform is the
Husky-Panda mobile manipulator, with the arm mounted on the base; the Session 3
scene demonstrates one small dynamic-object transfer. The two-Husky and
dual-Panda files are parser exercises, not commandable project platforms.

The mobile-manipulator controller supports four Husky wheel actuators, named
Panda arm targets, measured odometry/joint feedback, a bounded gripper target,
and a stale-command speed guard. The gripper topic commands both fingers with
positions from 0.0 m (closed) to 0.04 m (open). The command adapter does not
interpolate trajectory time. The supplied project stack does not include
Nav2, SLAM, camera perception, general collision avoidance, or a general task
planner.

## Preflight before assigning project tasks

Run these checks on the same Ubuntu 24.04, ROS 2 Jazzy, Drake 1.56.0, and pinned
drake-ros environment used by students:

1. Build a clean checkout and run `bash scripts/course_check.sh`.
2. Parse `husky_panda_pick_place.sdf` and `husky_panda_inspection.sdf` with
   `exercises/session_3/check_project_world.py`.
3. Launch `mobile_manipulator_controlled_py_launch.py` with the pick-and-place
   scene, first with RViz enabled and then disabled. Leave each run active for
   at least 10 seconds.
4. Confirm the Panda base is fixed to the Husky top plate and the bin begins on
   the workbench without initial contact instability.
5. Send open and closed gripper commands and verify both finger positions in
   `/panda_inspector/joint_states`.
6. Perform one grasp, lift, transfer onto the Husky platform, release, and
   retraction. Confirm that the bin remains on the platform after a bounded
   Husky command changes measured odometry.
7. Check one accepted and one rejected arm/gripper target, plus a Husky
   command timeout that stops the base.

Keep the launch logs, command messages, feedback samples, and an RViz view.
Do not describe the course platform as ready for object manipulation until
step 6 passes on the supported runtime.

## SDF and mesh constraints

- Students can create workcells from box and cylinder primitives.
- Drake's SDF mesh tutorial uses OBJ assets referenced through the
  `robotics_ii_models` package map. Convert an STL to OBJ before use.
- Prefer a simple collision proxy for shelves, machines, and workbenches.
  Keep mesh visuals and collision geometry separate when detailed contact is
  not required.
- Physical grasping depends on model scale, finger collision geometry, object
  inertia, initial pose, contact material, and the chosen motion. A correctly
  parsed SDF does not prove a grasp works.
- Do not claim perception, autonomous routing, production-machine physics,
  safety certification, or robust contact planning unless a team implements
  and tests it.

The parser checker verifies that named robot model instances are in one
finalized Drake plant. Runtime tests must still confirm the controller,
feedback, contact, and task behavior.
