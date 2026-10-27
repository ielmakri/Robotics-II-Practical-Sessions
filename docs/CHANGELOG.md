# Course redesign changelog

## v6 — Husky-mounted Panda and object handling

- Made the Husky-Panda mobile manipulator the shared multi-robot platform for
  all seven project briefs and the Session 3 manipulation exercise.
- Changed the controlled launch default to the mounted Husky/Panda SDF and
  added a bounded gripper command plus measured finger feedback.
- Added a dynamic-bin pick-and-place workcell, an IK helper, and an OBJ visual
  with a simple collision proxy. The grasp and ROS launch still require a
  runtime check in the supported course image.
- Replaced the seven repetitive inspection/service briefs with distinct
  airport, hospital, palletizing, CNC tending, kitting, airport turnaround,
  and laboratory batch scenarios.
- Added a student tutorial for inserting OBJ assets into SDF and checking a
  mounted mobile-manipulator scene.

## v5 — project feasibility and final review

- Replaced the three broad task cards with seven distinct, concrete Husky +
  fixed-Panda service tasks built around the supplied robot interfaces and
  local primitive geometry.
- Clarified that the two-Husky and dual-Panda scenes are SDF parsing examples,
  not supported project platforms; tied the project interface minimum to the
  mission coordinator rather than an arbitrary node count.
- Added seven project illustrations and two figures to the Session 3 student
  guide, including an SDF world view and a command/feedback path.
- Changed the Session 2 inertia equations to GitHub-rendered display math.
- Reused the gravity-compensation component in both single-Panda variants to
  reduce static sag around joint-space setpoints.
- Made the project README template explicitly optional and removed the
  peer-review requirement.
- Updated the capability summary and project readiness note to distinguish
  source/static checks from required runtime simulation tests.

## v4 — SDF project scenes and multi-robot readiness

- Corrected the Session 1/2 terminal commands, temporary-file editing steps,
  inertia calculation layout, joint-axis instructions, RViz frame guidance,
  and Xacro property name.
- Made `--no-daemon` explicit for ROS node/topic list, info, and echo commands.
- Expanded Session 3 with SDF world structure, parsing, multiple model
  instances, Drake port tracing, project-scene checks, and hands-on exercises.
- Added three concrete multi-robot project SDF layouts built from included
  Husky/Panda models and primitive geometry, plus one schematic illustration
  per brief.
- Added team size, session goals, ROS graph minimums, individual code/test
  ownership, and repeatable acceptance criteria to the project guide.
- Documented which multi-robot controllers and launch paths are still project
  work; the SDF files are not presented as complete robot applications.

## v3 — Panda hand and Session 1 reliability

- Integrated the Panda hand and two finger joints into the course architecture.
- Kept the Session 1 command contract focused on the seven arm joints while
  publishing feedback for all nine moving joints.
- Fixed ROS launch arguments in every Drake executable, serializer lifetime in
  the controlled Panda, clean Ctrl+C shutdown and the workspace nounset leak.
- Added static checks for Python syntax, the Panda URDF/transmissions/meshes and
  Markdown code fences.
- Added a Session 1 capstone in which students complete a two-waypoint Panda
  sequencer using a publisher, subscriber, YAML parameters and measured joint
  feedback instead of fixed delays.

## v2 — complete six-session student path

- Replaced the old high-level schedule with three detailed student tutorials and
  one concise project block for Sessions 4–6.
- Added official ROS 2 Jazzy links, command reference, graph/TF diagrams,
  expected observations and evidence-based checkpoints.
- Added `robotics_ii_tutorials`: parameterized JointState publisher/monitor,
  Panda JointTrajectory commander/goal monitor, YAML and launch examples, plus a
  starter exercise and ROS-independent validation tests.
- Added `robotics_ii_description`: reference URDF, incomplete student URDF,
  Xacro macro example and a launch path for both URDF and Xacro.
- Refactored `sim_systems`: safe named Panda command contract, safe home fallback,
  diagnostic desired-position topic, typed adapters, finite-step simulation loop,
  clean Husky and Husky+Panda scene examples, and updated launch arguments.
- Replaced the Drake-ROS Git clone in `scripts/install.sh` with a pinned archive
  download to avoid GitHub credential/HTTP 401 failures.
- Added `scripts/course_check.sh`, project interface/report templates and an
  honest capability table (the supplied Panda has no controllable gripper).

## Scope deliberately left as project work

The current baseline still does not provide timed trajectory interpolation,
Nav2, SLAM, perception, a general task planner, or collision-aware motion
planning. The mobile-manipulator object-transfer example is a short platform
exercise; project-specific scenes, mission logic, object flow, recovery, and
evaluation remain student work.
