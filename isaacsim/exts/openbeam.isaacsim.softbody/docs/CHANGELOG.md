# Changelog

## [0.2.0] - 2026-11-07

- Cables: `SoftBody` takes the integration step, node radius, node damping,
  a `.materials` file and friction, and holds or lets go of nodes with
  `attach()` and `release()` (and `attach=` from the first step).
- Kinematic colliders: prims the host moves by their USD transform, such as a
  gripper, a spool or fixed scenery, with their velocity taken from their
  pose one physics step back.
- The core binding exposes twist elements and per-collider friction.
- `scene.build_cable_patch()`: the data-centre patching scene, used by
  `isaacsim/demo_isaacsim_cable.py`.

## [0.1.0] - 2026-11-07

- First version: one OpenBeam JBeam soft body per `SoftBody`, coupled to PhysX
  rigid bodies (sphere, cube and capsule colliders) and the ground plane, drawn
  as a BasisCurves prim coloured by beam stress.
