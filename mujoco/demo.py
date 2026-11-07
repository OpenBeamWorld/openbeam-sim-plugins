#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""OpenBeam in MuJoCo: a MuJoCo wrecking ball against an OpenBeam JBeam crate.

Loads the "openbeam.softbody" engine plugin, then either runs headless and
prints a JSON summary, renders frames with the plugin's own beam drawing, or
opens MuJoCo's interactive viewer.

usage:
  python demo.py                       # headless, 4 s, JSON summary
  python demo.py --frames out_dir      # also write PNG frames (needs pillow)
  python demo.py --viewer              # interactive viewer, real time

The plugin library is found at $OPENBEAM_MUJOCO_PLUGIN or in the repository's
build/install/mujoco_plugin/ (build_windows.ps1, build_linux.sh).
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
MODEL = HERE / "models" / "wrecking_ball.xml"
ASSETS = HERE.parent / "assets"
PLUGIN_NAME = {"win32": "openbeam_mujoco.dll", "darwin": "libopenbeam_mujoco.dylib"}.get(sys.platform,
                                                                                   "libopenbeam_mujoco.so")


def plugin_path() -> Path:
    env = os.environ.get("OPENBEAM_MUJOCO_PLUGIN")
    if env:
        return Path(env)
    for candidate in (REPO / "build" / "install" / "mujoco_plugin" / PLUGIN_NAME,
                      REPO / "build" / "mujoco" / PLUGIN_NAME):
        if candidate.exists():
            return candidate
    raise SystemExit(f"{PLUGIN_NAME} not found: build it (README.md) or set OPENBEAM_MUJOCO_PLUGIN")


_LOADED: dict[str, ctypes.CDLL] = {}


def load_plugin() -> ctypes.CDLL:
    """Registers the plugin with MuJoCo once and returns the DLL for the ob_* API."""
    path = str(plugin_path())
    if path not in _LOADED:
        mujoco.mj_loadPluginLibrary(path)
        lib = ctypes.CDLL(path)
        lib.openbeam_mujoco_world.restype = ctypes.c_void_p
        lib.openbeam_mujoco_world.argtypes = [ctypes.c_void_p, ctypes.c_int]
        for name in ("ob_world_node_count", "ob_world_beam_count"):
            getattr(lib, name).restype = ctypes.c_int
            getattr(lib, name).argtypes = [ctypes.c_void_p]
        lib.ob_world_get_node_velocities.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.ob_world_get_node_positions.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        lib.ob_world_get_beam_state.argtypes = [ctypes.c_void_p] * 5
        lib.ob_world_get_beam_damage.argtypes = [ctypes.c_void_p] * 3
        lib.ob_world_total_mass.restype = ctypes.c_double
        lib.ob_world_total_mass.argtypes = [ctypes.c_void_p]
        _LOADED[path] = lib
    os.environ.setdefault("OPENBEAM_ASSET_DIR", str(ASSETS))
    return _LOADED[path]


def soft_body(lib: ctypes.CDLL, data: mujoco.MjData, instance: int = 0):
    """(positions, velocities, total mass) of the OpenBeam world of a plugin instance."""
    world = lib.openbeam_mujoco_world(data._address, instance)
    n = lib.ob_world_node_count(world)
    pos = np.zeros((n, 3))
    vel = np.zeros((n, 3))
    lib.ob_world_get_node_positions(world, pos.ctypes.data)
    lib.ob_world_get_node_velocities(world, vel.ctypes.data)
    return pos, vel, lib.ob_world_total_mass(world)


def build(model_path: Path = MODEL):
    lib = load_plugin()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return lib, model, data


def run_headless(seconds: float, frames: Path | None) -> dict:
    lib, model, data = build()
    crate = model.sensor("crate_state")
    ball = model.sensor("ball_velocity")
    renderer = None
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
        renderer = mujoco.Renderer(model, 720, 1280)
    steps = int(round(seconds / model.opt.timestep))
    frame_every = max(1, int(round((1.0 / 30.0) / model.opt.timestep)))
    centroid0 = data.sensordata[crate.adr[0] + 2: crate.adr[0] + 5].copy()
    speeds, wall = [], []
    first_contact_t = None
    for i in range(steps):
        t0 = time.perf_counter()
        mujoco.mj_step(model, data)
        wall.append(time.perf_counter() - t0)
        v = data.sensordata[ball.adr[0]: ball.adr[0] + 3]
        speeds.append(float(np.linalg.norm(v)))
        if first_contact_t is None and data.qfrc_passive[0] != 0.0:
            first_contact_t = data.time
        if renderer and i % frame_every == 0:
            renderer.update_scene(data, camera="side")
            save_png(renderer.render(), frames / f"frame_{i // frame_every:04d}.png")
    s = data.sensordata[crate.adr[0]: crate.adr[0] + 6]
    peak = int(np.argmax(speeds))
    wall_us = np.array(wall) * 1e6
    return {
        "mujoco": mujoco.__version__,
        "plugin": str(plugin_path()),
        "simulated_s": data.time,
        "first_contact_s": first_contact_t,
        "ball_speed_peak_mps": speeds[peak],
        "ball_speed_end_mps": speeds[-1],
        "ball_speed_after_peak_min_mps": min(speeds[peak:]),
        "beams_broken": int(s[0]),
        "beams_deformed": int(s[1]),
        "crate_centroid_shift_m": (s[2:5] - centroid0).tolist(),
        "openbeam_steps": int(s[5]),
        "mj_step_us_mean": float(wall_us.mean()),
        "mj_step_us_p99": float(np.percentile(wall_us, 99)),
        "realtime_headroom": float(model.opt.timestep * 1e6 / wall_us.mean()),
    }


def save_png(pixels: np.ndarray, path: Path) -> None:
    try:
        from PIL import Image
    except ImportError:  # pragma: no cover - reported once
        path = path.with_suffix(".ppm")
        with open(path, "wb") as fh:
            fh.write(b"P6 %d %d 255\n" % (pixels.shape[1], pixels.shape[0]))
            fh.write(pixels.tobytes())
        return
    Image.fromarray(pixels).save(path)


def run_viewer() -> None:
    import mujoco.viewer

    _, model, data = build()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.perf_counter()
            mujoco.mj_step(model, data)
            viewer.sync()
            left = model.opt.timestep - (time.perf_counter() - start)
            if left > 0:
                time.sleep(left)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--frames", type=Path)
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--out", type=Path, help="write the JSON summary here too")
    args = ap.parse_args()
    if args.viewer:
        run_viewer()
        return 0
    summary = run_headless(args.seconds, args.frames)
    text = json.dumps(summary, indent=1)
    print(text)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
