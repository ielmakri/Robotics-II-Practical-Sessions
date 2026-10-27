# Project scene assets

This directory is named `assets` in lowercase. It contains mesh and material
files used by the SDF scenes in `models/team_projects/`.

## Included asset

- `workbench.obj` is the visual mesh for the workbench in
  `husky_panda_pick_place.sdf`. The workbench uses separate box collision
  geometry for contact.

## Add a mesh

1. Copy the OBJ file and any MTL or texture files it references into this
   directory. Keep those files together and preserve their relative paths.
2. Export the mesh with dimensions in metres. Drake in this course requires
   uniform mesh scaling, so bake any nonuniform dimensions into the mesh and
   use `<scale>1 1 1</scale>` in the SDF.
3. Reference the file from the SDF with the registered package URI, for
   example:

   ```xml
   <uri>package://robotics_ii_models/team_projects/assets/machine.obj</uri>
   ```

4. Add a separate simple collision shape when the object needs contact. A
   visual mesh by itself is not a graspable collision model.
5. Parse the edited SDF with Drake and inspect it in RViz before using it in a
   simulation.

The Panda robot description is not stored here. Its hand URDF is under
`models/descriptions/robots/arms/panda_fer/urdf/panda_fer_hand.urdf` and is
included by the project SDF. Section 3.3 of the
[Session 3 guide](../../../docs/session_3_sim_systems/README.md#33-add-an-obj-mesh-to-an-sdf-scene)
covers mesh visuals. Section 3.4 covers static and dynamic object properties,
including collision geometry, mass, and inertia.
