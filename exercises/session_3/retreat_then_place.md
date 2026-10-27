# Retreat the Husky, then place the tote

Run these commands from the repository root in a terminal with the ROS 2
workspace sourced. Keep the simulation running in another terminal.

After the Panda has closed its fingers around the tote and the tote rises with
the gripper, move the hand to the checked lift pose and send the seven joint
positions printed by the IK helper:

```bash
python3 exercises/session_3/solve_pick_place_ik.py -0.05 -0.55 0.66
```

Confirm the tote is clear of the workbench. Drive the Husky backward by 0.50 m
using odometry feedback:

```bash
python3 exercises/session_3/drive_husky_distance.py --distance 0.50 --speed 0.08
```

The helper measures displacement along the Husky's initial heading, stops
when odometry reaches the requested distance, and publishes zero velocity
before exiting. It returns an error if odometry becomes stale or the target is
not reached within 20 seconds.

Solve the placement target relative to the mounted Panda. The target stays on
the Husky platform after the base moves:

```bash
python3 exercises/session_3/solve_pick_place_ik.py \
  0.2188 0.0 0.1355 --frame panda_link0
```

Send the seven returned positions to `/panda_inspector/joint_trajectory`,
lower until the tote rests on the plate, then open the gripper with
`{data: [0.04]}`. Verify the tote pose in RViz and confirm the Husky is stopped.

For the corrected scene geometry, the top-plate surface is at z=0.38363 m and
the 75 mm tote center is at z=0.42113 m when placed. The default hand target is
0.098 m above the tote center. The expected arm solution for the placement
target is:

```text
-0.452472, 0.183404, 0.044289, -2.978060, -1.695683, 3.123710, 2.071198
```
