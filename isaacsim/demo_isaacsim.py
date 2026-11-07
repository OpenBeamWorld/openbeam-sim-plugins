"""OpenBeam in Isaac Sim: a PhysX wrecking ball against an OpenBeam JBeam crate.

Run with Isaac Sim's own Python, after building the repository (README.md):

    <isaac-sim>/python.bat isaacsim/demo_isaacsim.py               # headless, JSON summary
    <isaac-sim>/python.bat isaacsim/demo_isaacsim.py --gui         # window, real time
    <isaac-sim>/python.bat isaacsim/demo_isaacsim.py --capture out.png
    ... add --app viewer (lean RTX app) or --app physics (no renderer) to use less memory

The script registers isaacsim/exts with Kit's extension manager,
enables "openbeam.isaacsim.softbody", builds the scene of
mujoco/models/wrecking_ball.xml, and steps PhysX at 200 Hz with
OpenBeam coupled in every pre-step. Isaac Sim asks you to accept the NVIDIA
Omniverse License Agreement on first use (or OMNI_KIT_ACCEPT_EULA=YES).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXT_NAME = "openbeam.isaacsim.softbody"

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--seconds", type=float, default=4.0)
parser.add_argument("--gui", action="store_true", help="open the Isaac Sim window and run in real time")
parser.add_argument("--capture", type=Path, help="render the viewport to this PNG at the end")
parser.add_argument("--out", type=Path, help="write the JSON summary here too")
parser.add_argument("--app", choices=("stock", "viewer", "physics"), default="stock",
                    help="Isaac's stock Python app; apps/openbeam.isaacsim.viewer.kit (RTX, lean); "
                         "or apps/openbeam.isaacsim.physics.kit (no renderer, least memory)")
parser.add_argument("--physics-only", action="store_true", help="the same as --app physics")
parser.add_argument("--kit-arg", action="append", default=[], metavar="ARG",
                    help="pass one argument to Kit, e.g. --kit-arg=--/app/vulkan=false")
args, _ = parser.parse_known_args()
if args.physics_only:
    args.app = "physics"
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

PHYSICS_DT = 1.0 / 200.0


def enable_extension() -> str:
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.add_path(str(HERE / "exts"))
    if not manager.set_extension_enabled_immediate(EXT_NAME, True) or not manager.is_extension_enabled(EXT_NAME):
        raise RuntimeError(f"Kit could not enable {EXT_NAME}")
    return manager.get_enabled_extension_id(EXT_NAME)


def main() -> int:
    ext_id = enable_extension()
    from openbeam.isaacsim.softbody import SoftBody
    from openbeam.isaacsim.softbody.scene import CRATE_POSITION, DEFAULT_CRATE, build_wrecking_ball

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    paths = build_wrecking_ball(stage, PHYSICS_DT)
    app.update()
    body = SoftBody(DEFAULT_CRATE, position=CRATE_POSITION, colliders=[paths["ball"]])
    centroid0 = np.array(body.stats()["centroid"])

    if args.capture or args.gui:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            from pxr import Gf
            from omni.kit.viewport.utility.camera_state import ViewportCameraState

            ViewportCameraState("/OmniverseKit_Persp", viewport).set_position_world(Gf.Vec3d(0.9, -5.2, 1.9), True)
            ViewportCameraState("/OmniverseKit_Persp", viewport).set_target_world(Gf.Vec3d(0.9, 0.0, 0.7), True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    app.update()  # starts physics; Isaac creates the tensor views here
    speeds, wall, contact_time = [], [], None
    steps = int(round(args.seconds / PHYSICS_DT))
    start = time.perf_counter()
    for i in range(steps):
        t0 = time.perf_counter()
        if args.gui:
            app.update()  # one rendered frame steps physics by its own dt
            behind = (i + 1) * PHYSICS_DT - (time.perf_counter() - start)
            if behind > 0:
                time.sleep(behind)
        else:
            SimulationManager.step(steps=1)
        wall.append(time.perf_counter() - t0)
        if body._rb_view is not None:
            v = body._host(body._rb_view.get_velocities(), 6)[0, :3]
            speeds.append(float(np.linalg.norm(v)))
            if contact_time is None and body.world.stats()["contacts"] and \
                    body.world.collider_wrench(0, body._origins[0])[0].any():
                contact_time = body.sim_time
        if args.gui and body.sim_time >= args.seconds:
            break

    if args.capture:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

        body.draw()
        for _ in range(30):  # let the renderer converge
            app.update()
        capture_viewport_to_file(get_active_viewport(), str(args.capture))
        for _ in range(10):
            app.update()

    s = body.stats()
    peak = int(np.argmax(speeds)) if speeds else 0
    version = next((p / "VERSION" for p in Path(sys.executable).resolve().parents if (p / "VERSION").exists()), None)
    summary = {
        "isaac_sim": version.read_text().strip() if version else "unknown",
        "extension": ext_id,
        "physics_dt": PHYSICS_DT,
        "simulated_s": body.sim_time,
        "first_contact_s": contact_time,
        "ball_speed_peak_mps": speeds[peak] if speeds else None,
        "ball_speed_end_mps": speeds[-1] if speeds else None,
        "ball_speed_after_peak_min_mps": min(speeds[peak:]) if speeds else None,
        "beams_broken": s["beams_broken"],
        "beams_deformed": s["beams_deformed"],
        "crate_centroid_shift_m": (np.array(s["centroid"]) - centroid0).tolist(),
        "openbeam_steps": s["steps"],
        "physics_step_wall_us_mean": float(np.mean(wall) * 1e6) if wall else None,
        "realtime_headroom": float(PHYSICS_DT / np.mean(wall)) if wall else None,
    }
    text = json.dumps(summary, indent=1)
    print("OPENBEAM_ISAAC_SUMMARY " + text, flush=True)
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
