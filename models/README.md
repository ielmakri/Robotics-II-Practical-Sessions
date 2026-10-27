# Models directory

This directory contains the robot descriptions, objects and scenes reused by the
three guided sessions and the final projects.

## Subdirectories

### descriptions
This folder contains detailed descriptions of robot and sensor models. Each
description provides a kinematic tree and physical properties for a multibody
model.

- **robots**: Specifications and configurations of robotic models.
- **sensors**: Specifications and configurations of sensor models.
<!-- - **hybrids**: Specifications and configurations of hybrid models (combinations of robots and sensors). -->

The course simulations load
`descriptions/robots/arms/panda_fer/urdf/panda_fer_hand.urdf`. It contains the
seven Panda arm joints, the Franka hand, two prismatic finger joints, visual and
collision meshes, inertial data and one transmission per moving joint.

### objects_scenes
This folder contains SDF (Simulation Description Format) files for floors,
objects and simulation environments.

### final_projects
These are reusable project worlds inherited from earlier course editions. Read
the current [project briefs](../docs/projects/README.md) before selecting one;
the worlds are starting points, not proof that a gripper or sensor interface is
already implemented.

### team_projects

The project scenes include a Husky-Panda mobile manipulator with a mounted
Panda, a dynamic-bin pick-and-place workcell, and parser examples for two
Huskies or two Panda arms. The controlled launch, ROS interfaces, and gripper
adapter are in `ros2_ws/src/sim_systems`. The older
`husky_panda_inspection.sdf` still shows a Husky with a world-fixed Panda; it
is retained as a parser/controller example and is not the current project
platform. See the [seven project briefs](../docs/projects/README.md).
