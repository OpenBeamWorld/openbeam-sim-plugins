#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Coil memory in MuJoCo: a cable wound round a spool keeps the coil when it is let go.

Two identical OpenBeam Cat6 cables lie on a desk. A 120 mm spool winds one of
them about 330 degrees, holds it, lets it go and sinks below the desk; the
other is never touched. Nothing bends the cable but the spool: whether it
keeps a coil, and how tight, is the copper's yield, the jacket's spring-back
and the desk's friction.

The run reports the curvature the wound cable keeps (the radius of the circle
through its once-wound length), the same for the untouched cable, and what the
fibre-section model predicts for a free cable bent to the spool's radius and
let go (validation/bending_rig.py).

usage:
  python coil_demo.py                       # headless, JSON summary
  python coil_demo.py --frames out_dir      # also PNG frames
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

import demo

HERE = Path(__file__).resolve().parent
MODEL = HERE / "models" / "cable_coil.xml"
SECTIONS, PER_SECTION, SEGMENT = 41, 4, 0.02
SPOOL_RADIUS = 0.06
WIND_RATE, WIND_SECONDS, RAMP = -2.0, 2.9, 0.2   # rad/s about z (clockwise from above): about 330 degrees
RELEASE_AT, DROP_AT, DROP_SECONDS, DROP_DEPTH, END = 4.4, 4.6, 1.2, 0.2, 8.0


def smoothstep(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def spool_rate(t: float) -> float:
    """The winder's speed: ramps up and down over RAMP seconds, as a motor does."""
    return WIND_RATE * min(smoothstep(t / RAMP), 1.0 - smoothstep((t - WIND_SECONDS + RAMP) / RAMP))


def spool_height(t: float, z0: float) -> float:
    return z0 - DROP_DEPTH * smoothstep((t - DROP_AT) / DROP_SECONDS)


def centreline(lib, data, instance: int) -> np.ndarray:
    world = lib.openbeam_mujoco_world(data._address, instance)
    n = lib.ob_world_node_count(world)
    pos = np.zeros((n, 3))
    lib.ob_world_get_node_positions(world, pos.ctypes.data)
    return pos.reshape(SECTIONS, PER_SECTION, 3).mean(axis=1)


def fit_radius(points: np.ndarray) -> float:
    """Radius of the least-squares circle through points in the desk's plane."""
    x, y = points[:, 0], points[:, 1]
    sol, *_ = np.linalg.lstsq(np.c_[2 * x, 2 * y, np.ones_like(x)], x * x + y * y, rcond=None)
    cx, cy, c = sol
    return math.sqrt(c + cx * cx + cy * cy)


def run(frames: Path | None = None) -> dict:
    lib = demo.load_plugin()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 720, 1280) if frames else None
    shots = {0.0: "coil_0_straight.png", 3.0: "coil_1_wound.png", END: "coil_2_released.png"}
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
    released, angle = False, 0.0
    z0 = float(data.mocap_pos[0, 2])
    wound_sections = int(round(abs(WIND_RATE) * (WIND_SECONDS - RAMP) * (SPOOL_RADIUS + 0.003) / SEGMENT))
    sensor = model.sensor("wound_state").adr[0]
    world = lib.openbeam_mujoco_world(data._address, 0)
    roll, peak_roll = np.zeros(lib.ob_world_twist_count(world)), 0.0
    while data.time < END - 1e-9:
        t = data.time
        data.mocap_quat[0] = [math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]
        data.mocap_pos[0, 2] = spool_height(t, z0)
        angle += spool_rate(t + 0.5 * model.opt.timestep) * model.opt.timestep
        if t >= RELEASE_AT and not released:
            lib.openbeam_mujoco_detach(data._address, 0, b"s0_*")
            released = True
        if renderer and shots and t >= min(shots) - 1e-9:
            renderer.update_scene(data, camera="overview")
            demo.save_png(renderer.render(), frames / shots.pop(min(shots)))
        before = data.time
        mujoco.mj_step(model, data)
        if data.time < before:
            raise RuntimeError(f"MuJoCo reset an unstable simulation at {before:.4f} s")
        lib.ob_world_get_twist(world, roll.ctypes.data)
        peak_roll = max(peak_roll, float(np.abs(roll).max()))
    if renderer and shots:
        renderer.update_scene(data, camera="overview")
        demo.save_png(renderer.render(), frames / shots.pop(min(shots)))
    wound = centreline(lib, data, 0)
    fresh = centreline(lib, data, 1)
    kept = fit_radius(wound[2:wound_sections])
    untouched = fit_radius(fresh[2:wound_sections])
    sys.path.insert(0, str(HERE.parent / "validation"))
    from bending_rig import Cable

    theory_kappa = Cable(length=0.2, segment=SEGMENT).theory_set(SPOOL_RADIUS + 0.003)
    s = data.sensordata[sensor: sensor + 6]
    return {
        "spool_radius_mm": 1000 * SPOOL_RADIUS,
        "wound_length_m": wound_sections * SEGMENT,
        "kept_coil_radius_mm": 1000 * kept,
        "untouched_cable_radius_mm": 1000 * untouched,
        "fibre_theory_free_set_radius_mm": 1000 / abs(theory_kappa) if theory_kappa else float("inf"),
        "peak_section_roll_deg": math.degrees(peak_roll),
        "beams_yielded": int(s[1]),
        "beams_torn": int(s[0]),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=Path, help="write PNGs of the straight, wound and released cables here")
    args = ap.parse_args()
    print(json.dumps(run(args.frames), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
