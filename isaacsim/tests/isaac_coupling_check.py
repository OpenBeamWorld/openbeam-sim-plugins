"""In-Isaac checks of the openbeam.isaacsim.softbody extension (not pytest: Kit owns the process).

    <isaac-sim>/python.bat isaacsim/tests/isaac_coupling_check.py [--app stock|viewer|physics]

1. Kit's extension manager loads and enables the extension from isaacsim/exts.
2. Momentum: a free 300 kg PhysX sphere at 4 m/s hits the free crate with
   gravity off; ball plus crate momentum is conserved to 0.1 %.
3. Wrecking ball: the demo scene; the ball is stopped, the crate yields,
   tears and is pushed away.
4. Stopping the timeline resets the crate to its rest pose.
5. In apps with menus: the Window > OpenBeam > Wrecking Ball Demo action
   builds the scene, and on Play the drawn crate is hit and moves.
Prints one JSON line per check and exits non-zero on any failure.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ISAAC_DIR = HERE.parent

parser = argparse.ArgumentParser()
parser.add_argument("--app", choices=("stock", "viewer", "physics"), default="stock")
parser.add_argument("--physics-only", action="store_true", help="the same as --app physics")
parser.add_argument("--kit-arg", action="append", default=[], metavar="ARG", help="pass one argument to Kit")
args, _ = parser.parse_known_args()
if args.physics_only:
    args.app = "physics"
EXPERIENCES = {"stock": "", "viewer": str(ISAAC_DIR / "apps" / "openbeam.isaacsim.viewer.kit"),
               "physics": str(ISAAC_DIR / "apps" / "openbeam.isaacsim.physics.kit")}

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "extra_args": args.kit_arg}, experience=EXPERIENCES[args.app])

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402

DT = 1.0 / 200.0
results = []


def check(name: str, ok: bool, **data) -> None:
    results.append(ok)
    print("OPENBEAM_CHECK " + json.dumps({"check": name, "ok": bool(ok), **data}), flush=True)


def run(steps: int, body) -> list[np.ndarray]:
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    app.update()
    samples = []
    for _ in range(steps):
        SimulationManager.step(steps=1)
        if body._rb_view is not None:
            samples.append(body._host(body._rb_view.get_velocities(), 6)[0, :3].copy())
    return samples


def stop() -> None:
    omni.timeline.get_timeline_interface().stop()
    app.update()


manager = omni.kit.app.get_app().get_extension_manager()
manager.add_path(str(ISAAC_DIR / "exts"))
enabled = manager.set_extension_enabled_immediate("openbeam.isaacsim.softbody", True)
check("extension_enabled", enabled and manager.is_extension_enabled("openbeam.isaacsim.softbody"),
      id=manager.get_enabled_extension_id("openbeam.isaacsim.softbody"))

from openbeam.isaacsim.softbody import SoftBody  # noqa: E402
from openbeam.isaacsim.softbody.scene import CRATE_POSITION, DEFAULT_CRATE, build_wrecking_ball  # noqa: E402

# -- momentum ---------------------------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(0.0)
physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
physx.CreateTimeStepsPerSecondAttr(200)
physx.CreateEnableGPUDynamicsAttr(False)
ball = UsdGeom.Sphere.Define(stage, "/World/Ball")
ball.CreateRadiusAttr(0.4)
UsdGeom.Xformable(ball).AddTranslateOp().Set(Gf.Vec3d(-1.0, 0.6, 0.45))
rb = UsdPhysics.RigidBodyAPI.Apply(ball.GetPrim())
rb.CreateVelocityAttr(Gf.Vec3f(4.0, 0.0, 0.0))
UsdPhysics.MassAPI.Apply(ball.GetPrim()).CreateMassAttr(300.0)
prb = PhysxSchema.PhysxRigidBodyAPI.Apply(ball.GetPrim())
prb.CreateLinearDampingAttr(0.0)
prb.CreateAngularDampingAttr(0.0)
prb.CreateSleepThresholdAttr(0.0)
app.update()
body = SoftBody(DEFAULT_CRATE, position=(0, 0, 0), colliders=["/World/Ball"], ground_height=None)
samples = run(300, body)  # 1.5 s
crate = 2.0 * body.world.velocities().sum(axis=0)
before = np.array([300.0 * 4.0, 0.0, 0.0])
after = 300.0 * samples[-1] + crate
check("momentum_conserved", crate[0] > 100.0 and samples[-1][0] < 3.7
      and np.abs(after - before).max() < 1e-3 * before[0],
      crate_momentum=crate.tolist(), ball_velocity=samples[-1].tolist(), total_error=(after - before).tolist())
stop()
body.destroy()

# -- wrecking ball ----------------------------------------------------------
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
paths = build_wrecking_ball(stage, DT)
app.update()
body = SoftBody(DEFAULT_CRATE, position=CRATE_POSITION, colliders=[paths["ball"]])
c0 = body.world.positions().mean(axis=0)
speeds = [float(np.linalg.norm(v)) for v in run(600, body)]  # 3 s
s = body.stats()
peak = int(np.argmax(speeds))
shift = (np.array(s["centroid"]) - c0).tolist()
check("wrecking_ball", speeds[peak] > 5.0 and min(speeds[peak:]) < 1.0 and s["beams_deformed"] >= 10
      and s["beams_broken"] >= 1 and shift[0] > 0.2,
      peak_speed=speeds[peak], min_after_peak=min(speeds[peak:]), beams_broken=s["beams_broken"],
      beams_deformed=s["beams_deformed"], centroid_shift=shift)
stop()
# Stopping the timeline restores the rest pose.
check("stop_resets", body.world.stats()["steps"] == 0 and body.world.stats()["beams_broken"] == 0)
body.destroy()

# -- the menu route ---------------------------------------------------------
try:
    import omni.kit.actions.core as actions
    import omni.kit.menu.utils  # noqa: F401
except ImportError:
    actions = None
if actions is None:
    print("OPENBEAM_CHECK " + json.dumps({"check": "menu_action", "skipped": "this app has no menus"}), flush=True)
else:
    action = actions.get_action_registry().get_action("openbeam.isaacsim.softbody", "load_wrecking_ball_demo")
    omni.usd.get_context().new_stage()
    app.update()
    if action is not None:
        action.execute()
    stage = omni.usd.get_context().get_stage()
    curves = UsdGeom.BasisCurves(stage.GetPrimAtPath("/World/OpenBeam"))
    built = action is not None and curves.GetPrim().IsValid() and stage.GetPrimAtPath("/World/Ball").IsValid()
    moved = 0.0
    if built:
        before = np.array(curves.GetPointsAttr().Get())
        omni.timeline.get_timeline_interface().play()
        app.update()
        SimulationManager.step(steps=400)  # 2 s: past the impact
        app.update()
        moved = float(np.abs(np.array(curves.GetPointsAttr().Get()) - before).max())
        stop()
    check("menu_action", built and moved > 0.05, action_found=action is not None, scene_built=built,
          max_beam_point_shift_m=moved)

code = 0 if all(results) else 1
print(f"OPENBEAM_CHECKS {'PASSED' if code == 0 else 'FAILED'} ({sum(results)}/{len(results)})", flush=True)
app.close()
sys.exit(code)
