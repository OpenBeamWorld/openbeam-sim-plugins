# SPDX-License-Identifier: GPL-3.0-or-later
"""The OpenBeam cable in MuJoCo: a mocap grip, coil memory and data-centre patching.

    python -m pytest mujoco/tests -q

The demo checks run the demos as shipped (about 30 s together) and hold them to
what the README reports: physics outcomes, not scripted ones.
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cable_demo  # noqa: E402
import coil_demo  # noqa: E402
import demo  # noqa: E402

LIB = demo.load_plugin()

MOCAP_XML = """<mujoco>
  <option timestep="0.002" gravity="0 0 0"/>
  <extension><plugin plugin="openbeam.softbody"><instance name="cable">
    <config key="jbeam" value="cat6_coil.jbeam"/>
    <config key="materials" value="cat6_coil.materials"/>
    <config key="step" value="0.000033"/>
    <config key="colliders" value="grip"/>
    <config key="attach" value="s0_*:grip s1_*:grip"/>
  </instance></plugin></extension>
  <worldbody>
    <body name="hand" mocap="true" pos="0.01 0 0">
      <geom name="grip" type="box" size="0.001 0.001 0.001" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""


def test_mocap_grip_carries_the_cable_smoothly():
    """A mocap body has no velocity in MuJoCo; the plugin takes it from the body's pose one step back,
    so the cable end it holds moves with it instead of jumping every MuJoCo step."""
    model = mujoco.MjModel.from_xml_string(MOCAP_XML)
    data = mujoco.MjData(model)
    world = LIB.openbeam_mujoco_world(data._address, 0)
    vel = np.zeros(3 * LIB.ob_world_node_count(world))
    fastest = 0.0
    for _ in range(250):
        speed = 0.1 * min(1.0, data.time / 0.1)                  # up to 0.1 m/s along the cable over 0.1 s
        data.mocap_pos[0, 0] += speed * model.opt.timestep
        mujoco.mj_step(model, data)
        LIB.ob_world_get_node_velocities(world, vel.ctypes.data)
        fastest = max(fastest, float(np.linalg.norm(vel.reshape(-1, 3), axis=1).max()))
    held = vel.reshape(-1, 3)[:8, 0]
    assert held == pytest.approx(0.1, rel=0.02)                  # zero, with jumps between, before
    assert fastest < 0.15


@pytest.fixture(scope="module")
def coil():
    return coil_demo.run()


def test_coil_memory_matches_fibre_theory(coil):
    """Wound round a 120 mm spool and let go, the cable keeps the set the fibre-section model predicts
    for its copper; the cable beside it, never touched, stays straight."""
    assert coil["kept_coil_radius_mm"] == pytest.approx(coil["fibre_theory_free_set_radius_mm"], rel=0.1)
    assert coil["untouched_cable_radius_mm"] > 10_000
    assert coil["beams_yielded"] > 0 and coil["beams_torn"] == 0
    assert coil["peak_section_roll_deg"] < 5.0        # bends its copper; no bay rolls over to dodge it


@pytest.fixture(scope="module")
def patch():
    return cable_demo.run()


def test_patching_threads_the_ring_without_damage(patch):
    """The cable follows the gripper through the D-ring, stays above its rated bend radius and takes
    a set but no damage. (The gripper holds the plug rigidly: the plug checks are the path's.)"""
    assert patch["threads_ring"]
    assert patch["plug_off_port_axis_mm"] < 2.0 and patch["plug_insertion_mm"] > 10.0
    assert patch["time_below_rating_s"] == 0.0
    assert patch["beams_yielded"] > 0
    assert patch["beams_torn"] == 0 and patch["beams_damaged"] == 0
    assert patch["peak_section_roll_deg"] < 20.0
