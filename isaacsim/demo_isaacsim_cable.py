# SPDX-License-Identifier: GPL-3.0-or-later
"""Data-centre patching in Isaac Sim: a gripper routes an OpenBeam Cat6 cable through a rack D-ring.

The scene and the gripper's route are those of mujoco/cable_demo.py
(mujoco/cable_route.py holds both). The gripper takes the cable's RJ45
plug, gives the cable some slack, threads it through the D-ring and pushes
the plug into a patch-panel port. OpenBeam simulates the cable and plug in
every PhysX pre-step:

- copper conductors that yield and take a set, an elastic jacket, torsion,
  and the 8 g plug with its boot;
- contact with the desk, the ring and the panel, which are kinematic
  colliders, as the gripper is.

Run with Isaac Sim's own Python, after building the repository (README.md):

    <isaac-sim>/python.bat isaacsim/demo_isaacsim_cable.py                 # headless, JSON summary
    <isaac-sim>/python.bat isaacsim/demo_isaacsim_cable.py --app viewer --capture out_dir
    <isaac-sim>/python.bat isaacsim/demo_isaacsim_cable.py --gui           # window, real time

--capture writes RTX frames at key moments: through the ring, and plugged in.
--app viewer (a lean RTX app) uses less memory than the stock one. Isaac Sim
asks you to accept the NVIDIA Omniverse License Agreement on first use (or
set OMNI_KIT_ACCEPT_EULA=YES).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXT_NAME = "openbeam.isaacsim.softbody"
sys.path.insert(0, str(HERE.parent / "mujoco"))
import cable_route as route  # noqa: E402  (pure Python: the scene's layout and the gripper's route)

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--seconds", type=float, default=route.DURATION)
parser.add_argument("--gui", action="store_true", help="open the Isaac Sim window and run in real time")
parser.add_argument("--capture", type=Path, help="write RTX frames into this directory")
parser.add_argument("--out", type=Path, help="write the JSON summary here too")
parser.add_argument("--turn-while-lifting", action="store_true", help="turn the plug while lifting it")
parser.add_argument("--app", choices=("stock", "viewer", "physics"), default="stock",
                    help="Isaac's stock Python app; apps/openbeam.isaacsim.viewer.kit (RTX, lean); "
                         "or apps/openbeam.isaacsim.physics.kit (no renderer, least memory)")
parser.add_argument("--kit-arg", action="append", default=[], metavar="ARG",
                    help="pass one argument to Kit, e.g. --kit-arg=--/app/vulkan=false")
args, _ = parser.parse_known_args()
if args.app == "physics" and (args.gui or args.capture):
    parser.error("the physics app has no renderer: drop --gui and --capture")
EXPERIENCES = {"stock": "", "viewer": str(HERE / "apps" / "openbeam.isaacsim.viewer.kit"),
               "physics": str(HERE / "apps" / "openbeam.isaacsim.physics.kit")}

from isaacsim import SimulationApp  # noqa: E402  (must precede every omni import)

app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720, "extra_args": args.kit_arg},
                    experience=EXPERIENCES[args.app])

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

PHYSICS_DT = 1.0 / 200.0
CAPTURES = {4.4: "isaac_cable_1_ring.png", route.DURATION: "isaac_cable_2_plugged.png"}


def enable_extension() -> str:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(HERE / "exts"))
    if not manager.set_extension_enabled_immediate(EXT_NAME, True) or not manager.is_extension_enabled(EXT_NAME):
        raise RuntimeError(f"Kit could not enable {EXT_NAME}")
    return manager.get_enabled_extension_id(EXT_NAME)


def capture(timeline, path: Path) -> None:
    """Renders the viewport to a PNG with physics paused."""
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

    timeline.pause()
    for _ in range(40):  # let the renderer converge
        app.update()
    capture_viewport_to_file(get_active_viewport(), str(path))
    for _ in range(10):
        app.update()
    timeline.play()
    app.update()


def main() -> int:
    ext_id = enable_extension()
    from openbeam.isaacsim.softbody import SoftBody
    from openbeam.isaacsim.softbody.scene import DEFAULT_CABLE, build_cable_patch

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    paths = build_cable_patch(stage, PHYSICS_DT, route.PLUG_START)
    app.update()
    colliders = [paths[k] for k in ("outlet", "grip", "ring_top", "ring_bottom", "ring_left", "ring_right",
                                    "panel_upper", "panel_lower", "panel_left", "panel_right")]
    body = SoftBody(DEFAULT_CABLE, position=route.CABLE_ORIGIN, kinematic_colliders=colliders,
                    materials=DEFAULT_CABLE.with_suffix(".materials"), step=0.000033, node_radius=0.00118,
                    node_damping=6.0, friction=(0.5, 0.4), attach=[("s0_*", paths["outlet"]), ("s1_*", paths["outlet"])],
                    stress_scale=30.0, strain_scale=0.02, beam_width=0.0025)
    gripper = UsdGeom.Xformable(stage.GetPrimAtPath(paths["gripper"]))
    translate, orient = gripper.GetOrderedXformOps()
    path = route.waypoints(turn_while_lifting=args.turn_while_lifting)

    if args.capture or args.gui:
        from omni.kit.viewport.utility import get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        viewport = get_active_viewport()
        if viewport is not None:
            camera = ViewportCameraState("/OmniverseKit_Persp", viewport)
            camera.set_position_world(Gf.Vec3d(0.62, 0.6, 0.36), True)
            camera.set_target_world(Gf.Vec3d(0.08, -0.02, 0.07), True)
            # The default near plane is 1 m in a metre stage: bring it in for a desk-top scene. The viewport's
            # camera lives in the session layer, so the edit goes there.
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                UsdGeom.Camera(stage.GetPrimAtPath("/OmniverseKit_Persp")).GetClippingRangeAttr().Set(
                    Gf.Vec2f(0.01, 100.0))
    if args.capture:
        args.capture.mkdir(parents=True, exist_ok=True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    app.update()
    tightest, below_rating, gripping, wall = np.inf, 0.0, False, []
    pending = sorted(CAPTURES) if args.capture else []
    start = time.perf_counter()
    for i in range(int(round(args.seconds / PHYSICS_DT))):
        t = body.sim_time
        pos, yaw, grip = route.gripper_target(t, path)
        translate.Set(Gf.Vec3d(*pos))
        q = route.yaw_quat(yaw)
        orient.Set(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        if grip and not gripping:
            body.attach("rj_*", paths["grip"])
        elif gripping and not grip:
            body.release("rj_*")
        gripping = grip
        t0 = time.perf_counter()
        if args.gui:
            app.update()
            behind = (i + 1) * PHYSICS_DT - (time.perf_counter() - start)
            if behind > 0:
                time.sleep(behind)
        else:
            SimulationManager.step(steps=1)
        wall.append(time.perf_counter() - t0)
        radius = float(route.bend_radii(route.centreline(body.world.positions())).min())
        tightest = min(tightest, radius)
        below_rating += PHYSICS_DT if radius < route.RATED_MIN_BEND_RADIUS else 0.0
        if pending and body.sim_time >= pending[0] - 1e-9:
            body.draw()
            capture(timeline, args.capture / CAPTURES[pending.pop(0)])

    points = body.world.positions()
    centre, axis, plug = route.plug_pose(points)
    plastic, damage = body.world.beam_damage()
    s = body.stats()
    version = next((p / "VERSION" for p in Path(sys.executable).resolve().parents if (p / "VERSION").exists()), None)
    summary = {
        "isaac_sim": version.read_text().strip() if version else "unknown",
        "extension": ext_id,
        "physics_dt": PHYSICS_DT,
        "openbeam_step_s": body.world.step,
        "simulated_s": body.sim_time,
        "threads_ring": route.threads_ring(route.centreline(points)),
        "plug_off_port_axis_mm": float(1000 * np.linalg.norm(centre[[0, 2]] - np.array(route.PORT)[[0, 2]])),
        "tightest_bend_radius_mm": 1000 * tightest,
        "rated_min_bend_radius_mm": 1000 * route.RATED_MIN_BEND_RADIUS,
        "time_below_rating_s": below_rating,
        "beams_yielded": s["beams_deformed"],
        "peak_plastic_strain": float(np.abs(plastic).max()),
        "beams_damaged": int((damage > 0).sum()),
        "beams_torn": s["beams_broken"],
        "physics_step_wall_ms_mean": float(np.mean(wall) * 1e3) if wall else None,
        "realtime_factor": float(PHYSICS_DT / np.mean(wall)) if wall else None,
    }
    text = json.dumps(summary, indent=1)
    print("OPENBEAM_ISAAC_CABLE_SUMMARY " + text, flush=True)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
    timeline.stop()
    body.destroy()
    return 0


code = 1
try:
    code = main()
finally:
    app.close()
sys.exit(code)
