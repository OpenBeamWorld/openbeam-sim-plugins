# OpenBeam Soft Body (Isaac Sim extension)

This extension runs OpenBeam's JBeam soft-body physics inside Isaac Sim and
couples it two ways to PhysX rigid bodies. Every PhysX step, before PhysX steps:

1. Each collider body's pose and velocity are read through the physics tensor
   API.
2. OpenBeam advances the physics dt in 0.5 ms steps.
3. The force and torque that the structure put on each collider are applied
   to that body for the step PhysX takes next.

The structure is drawn as a `BasisCurves` prim, coloured by beam stress.

```python
from openbeam.isaacsim.softbody import SoftBody

crate = SoftBody("truss_crate.jbeam", position=(0.45, -0.6, 0.02),
                 colliders=["/World/Ball"], ground_height=0.0)
# play the timeline; crate.stats() -> beams broken/deformed, centroid, steps
```

**Window > OpenBeam > Wrecking Ball Demo** builds the demo scene in the open
stage; press Play to run it.

The native library (`openbeam_core.dll`, or `libopenbeam_core.so` on Linux)
is looked up in this order:

1. `$OPENBEAM_CORE_DLL`
2. this extension's `bin/`
3. the repository's `build/install/bin/` (Windows) or `build/install/lib/`

For build steps, licensing and what the core does and does not simulate, see
the repository's `README.md`.
