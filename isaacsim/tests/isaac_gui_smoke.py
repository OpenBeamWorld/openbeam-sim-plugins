"""The extension in the full Isaac Sim GUI (run by Kit's --exec, not pytest).

    isaac-sim.bat --ext-folder isaacsim/exts --enable openbeam.isaacsim.softbody
        --portable-root <data-dir> --exec isaacsim/tests/isaac_gui_smoke.py

Once the window is up it finds Window > OpenBeam > Wrecking Ball Demo, runs
its action in a new stage, plays 3 s, saves the viewport to
$OPENBEAM_GUI_CAPTURE (default openbeam_gui.png in the working directory),
prints OPENBEAM_GUI {json} and quits.
"""

from __future__ import annotations

import asyncio
import json
import os

import numpy as np
import omni.kit.actions.core
import omni.kit.app
import omni.kit.menu.utils
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

EXT = "openbeam.isaacsim.softbody"


def walk(node, path=()):
    """(menu path, item) for every menu item, through whatever nesting of lists and dicts holds them."""
    if isinstance(node, dict):
        for key, value in node.items():
            yield from walk(value, path + (str(key),))
    elif isinstance(node, (list, tuple)):
        for value in node:
            yield from walk(value, path)
    elif hasattr(node, "name"):
        yield path + (node.name,), node
        yield from walk(getattr(node, "sub_menu", None) or [], path + (node.name,))


def find_entry(menus):
    """The Window > OpenBeam > Wrecking Ball Demo item.

    Kit's merged menus hold a submenu's items under their own key, here
    "Window_OpenBeam": {"parent": ..., "items": [...]}.
    """
    return next((item for path, item in walk(menus or {})
                 if path[0].startswith("Window") and "OpenBeam" in "/".join(path)
                 and path[-1] == "Wrecking Ball Demo"), None)


async def main() -> None:
    app = omni.kit.app.get_app()
    result = {"ok": False}
    try:
        for _ in range(60):  # let the window and menus build
            await app.next_update_async()
        menus = omni.kit.menu.utils.get_merged_menus()
        entry = find_entry(menus)
        result["menu_entry"] = entry is not None
        if entry is None:
            result["openbeam_menu_paths"] = [path for path, _ in walk(menus or {}) if "OpenBeam" in path][:5]
        result["menu_action"] = list(entry.onclick_action) if entry is not None and entry.onclick_action else None
        action = omni.kit.actions.core.get_action_registry().get_action(EXT, "load_wrecking_ball_demo")
        result["action_registered"] = action is not None

        await omni.usd.get_context().new_stage_async()
        await app.next_update_async()
        action.execute()
        stage = omni.usd.get_context().get_stage()
        curves = UsdGeom.BasisCurves(stage.GetPrimAtPath("/World/OpenBeam"))
        before = np.array(curves.GetPointsAttr().Get())

        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        from omni.kit.viewport.utility.camera_state import ViewportCameraState

        viewport = get_active_viewport()
        ViewportCameraState("/OmniverseKit_Persp", viewport).set_position_world(Gf.Vec3d(0.9, -5.2, 1.9), True)
        ViewportCameraState("/OmniverseKit_Persp", viewport).set_target_world(Gf.Vec3d(0.9, 0.0, 0.7), True)

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        frames = 0
        while timeline.get_current_time() < 3.0 and frames < 5000:
            await app.next_update_async()
            frames += 1
        result["sim_time_s"] = timeline.get_current_time()
        result["frames"] = frames
        timeline.pause()
        result["max_beam_point_shift_m"] = float(np.abs(np.array(curves.GetPointsAttr().Get()) - before).max())

        path = os.environ.get("OPENBEAM_GUI_CAPTURE", "openbeam_gui.png")
        capture_viewport_to_file(viewport, path)
        for _ in range(30):
            await app.next_update_async()
        result["capture"] = path if os.path.exists(path) else None
        result["ok"] = bool(result["menu_entry"] and result["menu_action"] == [EXT, "load_wrecking_ball_demo"]
                            and result["max_beam_point_shift_m"] > 0.05 and result["capture"])
        timeline.stop()
    except Exception as e:  # report, then quit either way
        result["error"] = repr(e)
    print("OPENBEAM_GUI " + json.dumps(result), flush=True)
    app.post_quit(0 if result["ok"] else 1)


asyncio.ensure_future(main())
