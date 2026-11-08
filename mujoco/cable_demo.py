#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Data-centre patching in MuJoCo: a gripper plugs an OpenBeam Cat6 cable into a patch panel.

The gripper (a mocap body) takes the cable's RJ45 plug, gives the cable some
slack, threads it through a rack D-ring and pushes the plug into a patch-panel
port. OpenBeam simulates the cable and the plug: sag, bending and twist,
contact with the desk, ring and panel, and any permanent set or damage the
handling leaves. The gripper's path is the only scripted input, and the
gripper holds the plug rigidly, so where the plug ends up is the path's doing;
where the cable goes and what it keeps are the physics.

The run reports, kept apart as a cable's properties are:
  - the tightest bend along the centreline and the time spent below the
    cable's rated minimum bend radius (a performance rating, not a failure);
  - the permanent set the copper took (beams that yielded, peak plastic strain);
  - damage and tears, where the copper's ductility ran out;
whether the cable passes through the ring, and where the plug ended up. Two
routes differ in when the plug turns to face the port: while it is carried
(the default) or while it is lifted (--turn-while-lifting).

usage:
  python cable_demo.py                      # headless, JSON summary
  python cable_demo.py --frames out_dir     # also PNG frames from the overview camera
  python cable_demo.py --turn-while-lifting # the other route
  python cable_demo.py --viewer             # interactive viewer, real time
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

import demo
from cable_route import (DURATION, PLUG, PLUG_START, RATED_MIN_BEND_RADIUS, RING, SECTIONS,  # noqa: F401
                         PER_SECTION, WAYPOINTS, bend_radii, centreline, gripper_target, plug_pose,
                         threads_ring, waypoints, yaw_quat)

HERE = Path(__file__).resolve().parent
MODEL = HERE / "models" / "cable_patch.xml"


def nodes(lib, data) -> np.ndarray:
    return demo.soft_body(lib, data)[0]


def build():
    lib = demo.load_plugin()
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return lib, model, data


class Driver:
    """Moves the gripper along a path and grips or releases the plug at the path's grip changes."""

    def __init__(self, lib, model, data, path=WAYPOINTS):
        self.lib, self.model, self.data, self.path, self.gripping = lib, model, data, path, False

    def __call__(self, t: float) -> None:
        pos, yaw, grip = gripper_target(t, self.path)
        self.data.mocap_pos[0] = pos
        self.data.mocap_quat[0] = yaw_quat(yaw)
        if grip and not self.gripping:
            mujoco.mj_forward(self.model, self.data)          # the grip collider where it is now
            if self.lib.openbeam_mujoco_attach(self.model._address, self.data._address, 0, b"rj_*", b"grip") != 8:
                raise RuntimeError("the gripper did not take the plug")
        elif self.gripping and not grip:
            self.lib.openbeam_mujoco_detach(self.data._address, 0, b"rj_*")
        self.gripping = grip


def run(seconds: float = DURATION, frames: Path | None = None, frame_times=None, path=WAYPOINTS) -> dict:
    lib, model, data = build()
    world = lib.openbeam_mujoco_world(data._address, 0)
    n = lib.ob_world_beam_count(world)
    stress, damage = np.zeros(n, np.float32), np.zeros(n, np.float32)
    drive = Driver(lib, model, data, path)
    renderer = mujoco.Renderer(model, 720, 1280) if frames else None
    if frames:
        frames.mkdir(parents=True, exist_ok=True)
    frame_times = sorted(frame_times or [])
    tightest, peak_tension, wall, below_rating = np.inf, 0.0, 0.0, 0.0
    plastic = np.zeros(n, np.float32)
    roll, peak_roll = np.zeros(lib.ob_world_twist_count(world)), 0.0
    sensor = model.sensor("cable_state").adr[0]
    while data.time < seconds - 1e-9:
        drive(data.time)
        before = data.time
        t0 = time.perf_counter()
        mujoco.mj_step(model, data)
        wall += time.perf_counter() - t0
        if data.time < before:
            raise RuntimeError(f"MuJoCo reset an unstable simulation at {before:.4f} s")
        radius = float(bend_radii(centreline(nodes(lib, data))).min())
        tightest = min(tightest, radius)
        below_rating += model.opt.timestep if radius < RATED_MIN_BEND_RADIUS else 0.0
        lib.ob_world_get_beam_state(world, stress.ctypes.data, None, None, None)
        peak_tension = max(peak_tension, float(-stress.min()))   # stress is positive in compression
        lib.ob_world_get_twist(world, roll.ctypes.data)
        peak_roll = max(peak_roll, float(np.abs(roll).max()))
        if renderer and frame_times and data.time >= frame_times[0] - 1e-9:
            renderer.update_scene(data, camera="overview")
            demo.save_png(renderer.render(), frames / f"cable_{frame_times.pop(0):.1f}s.png")
    if renderer:
        renderer.update_scene(data, camera="port_view")
        demo.save_png(renderer.render(), frames / "cable_closeup.png")
    points = nodes(lib, data)
    centre, axis, plug = plug_pose(points)
    port = data.site("port").xpos
    lib.ob_world_get_beam_damage(world, plastic.ctypes.data, damage.ctypes.data)
    s = data.sensordata[sensor: sensor + 6]
    return {
        "mujoco": mujoco.__version__,
        "simulated_s": data.time,
        "openbeam_step_s": lib.ob_world_get_step(world),
        "threads_ring": threads_ring(centreline(points)),
        "plug_off_port_axis_mm": float(1000 * np.linalg.norm(centre[[0, 2]] - port[[0, 2]])),
        "plug_insertion_mm": float(1000 * ((port[1] + 0.008) - plug[4:, 1].mean())),   # past the panel face
        "plug_axis_along_port": float(-axis[1]),
        "tightest_bend_radius_mm": 1000 * tightest,
        "rated_min_bend_radius_mm": 1000 * RATED_MIN_BEND_RADIUS,
        "time_below_rating_s": below_rating,
        "peak_conductor_tension_n": peak_tension,
        "peak_section_roll_deg": math.degrees(peak_roll),
        "beams_yielded": int(s[1]),
        "peak_plastic_strain": float(np.abs(plastic).max()),
        "beams_damaged": int((damage > 0).sum()),
        "beams_torn": int(s[0]),
        "realtime_factor": data.time / wall,
    }


def run_viewer(path=WAYPOINTS) -> None:
    import mujoco.viewer

    lib, model, data = build()
    drive = Driver(lib, model, data, path)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and data.time < DURATION + 5:
            start = time.perf_counter()
            drive(data.time)
            mujoco.mj_step(model, data)
            viewer.sync()
            left = model.opt.timestep - (time.perf_counter() - start)
            if left > 0:
                time.sleep(left)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=DURATION)
    ap.add_argument("--frames", type=Path, help="write overview PNGs at key moments here")
    ap.add_argument("--turn-while-lifting", action="store_true", help="turn the plug while lifting it")
    ap.add_argument("--viewer", action="store_true")
    args = ap.parse_args()
    path = waypoints(turn_while_lifting=args.turn_while_lifting)
    if args.viewer:
        run_viewer(path)
        return 0
    summary = run(args.seconds, args.frames, [0.7, 1.5, 2.2, 3.4, 4.4, 6.8] if args.frames else None, path)
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
