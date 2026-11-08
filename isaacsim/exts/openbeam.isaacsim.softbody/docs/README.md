# OpenBeam Soft Body (Isaac Sim extension)

This extension runs OpenBeam's JBeam soft-body physics inside Isaac Sim and
couples it two ways to PhysX rigid bodies. Every PhysX step, before PhysX steps:

1. Each collider body's pose and velocity are read through the physics tensor
   API.
2. OpenBeam advances the physics dt in its own steps (0.5 ms unless set).
3. The force and torque that the structure put on each collider are applied
   to that body for the step PhysX takes next.

The structure is drawn as a `BasisCurves` prim, coloured by beam stress.

```python
from openbeam.isaacsim.softbody import SoftBody

crate = SoftBody("truss_crate.jbeam", position=(0.45, -0.6, 0.02),
                 colliders=["/World/Ball"], ground_height=0.0)
# play the timeline; crate.stats() -> beams broken/deformed, centroid, steps
```

For a cable (from the repository's `assets/make_cable.py`), pass its step,
contact radius, damping, materials and friction, and use kinematic colliders
for parts your script or controller moves:

```python
cable = SoftBody("cat6_cable.jbeam", position=(-0.45, 0.25, 0.0), materials="cat6_cable.materials",
                 step=33e-6, node_radius=0.00118, node_damping=6.0, friction=(0.5, 0.4),
                 kinematic_colliders=["/World/Outlet", "/World/Gripper/Grip"],
                 attach=[("s0_*", "/World/Outlet"), ("s1_*", "/World/Outlet")])
cable.attach("rj_*", "/World/Gripper/Grip")   # take the plug at the next physics step
cable.release("rj_*")                          # let it go over release_seconds
```

`isaacsim/demo_isaacsim_cable.py` routes a Cat6 cable through a rack D-ring
into a patch panel this way.

**Window > OpenBeam > Wrecking Ball Demo** builds the demo scene in the open
stage; press Play to run it.

The native library (`openbeam_core.dll`, or `libopenbeam_core.so` on Linux)
is looked up in this order:

1. `$OPENBEAM_CORE_DLL`
2. this extension's `bin/`
3. the repository's `build/install/bin/` (Windows) or `build/install/lib/`

For build steps, licensing and what the core does and does not simulate, see
the repository's `README.md`.
