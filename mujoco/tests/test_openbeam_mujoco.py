# SPDX-License-Identifier: GPL-3.0-or-later
"""MuJoCo engine plugin "openbeam.softbody": loading, coupling and state contracts.

Run with the MuJoCo venv after building (README.md):
    python -m pytest mujoco/tests -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import demo  # noqa: E402

LIB = demo.load_plugin()
ASSETS = demo.ASSETS


def model_xml(body: str, gravity: str = "0 0 -9.81", colliders: str = "", sensor: bool = True,
              pos: str = "0 0 0.02", timestep: float = 0.002) -> str:
    collider_cfg = f'<config key="colliders" value="{colliders}"/>' if colliders else ""
    sensor_xml = '<sensor><plugin name="state" instance="crate"/></sensor>' if sensor else ""
    return f"""<mujoco>
  <option timestep="{timestep}" gravity="{gravity}" integrator="implicitfast"/>
  <extension><plugin plugin="openbeam.softbody"><instance name="crate">
    <config key="jbeam" value="{(ASSETS / 'truss_crate.jbeam').as_posix()}"/>
    <config key="pos" value="{pos}"/>{collider_cfg}
  </instance></plugin></extension>
  <worldbody>{body}</worldbody>{sensor_xml}
</mujoco>"""


def make(xml: str):
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def state(m, d):
    s = m.sensor("state")
    return d.sensordata[s.adr[0]: s.adr[0] + 6].copy()


def test_plugin_registers_and_compiles():
    m, d = make(model_xml('<geom type="plane" size="5 5 .1"/>'))
    assert m.nplugin == 1
    assert d.plugin_data[0] != 0
    pos, _, mass = demo.soft_body(LIB, d)
    assert pos.shape == (100, 3)
    assert mass == pytest.approx(200.0)
    # placed by the instance's pos attribute
    assert pos[:, 2].min() == pytest.approx(0.02, abs=1e-6)


def test_bad_jbeam_path_fails_model_creation():
    xml = model_xml("").replace("truss_crate.jbeam", "missing.jbeam")
    with pytest.raises(Exception):
        make(xml)


def test_crate_rests_on_the_model_plane():
    m, d = make(model_xml('<geom type="plane" size="5 5 .1"/>'))
    for _ in range(1000):  # 2 s
        mujoco.mj_step(m, d)
    pos, vel, _ = demo.soft_body(LIB, d)
    s = state(m, d)
    assert pos[:, 2].min() > -0.005
    assert np.abs(vel).max() < 0.05
    assert s[0] == 0 and s[1] == 0          # nothing broken or yielded at rest
    assert s[5] == 4000                     # 0.5 ms steps for 2 s


def test_two_way_coupling_conserves_momentum():
    """Free MuJoCo sphere into the free crate, no gravity: total momentum is kept."""
    body = ('<body name="ball" pos="-1 0.6 0.45"><freejoint/>'
            '<geom name="ball" type="sphere" size="0.4" mass="300" contype="0" conaffinity="0"/></body>')
    m, d = make(model_xml(body, gravity="0 0 0", colliders="ball"))
    d.qvel[0] = 4.0
    mujoco.mj_forward(m, d)
    _, vel, _ = demo.soft_body(LIB, d)
    before = 300.0 * d.qvel[0:3] + 2.0 * vel.sum(axis=0)
    for _ in range(750):  # 1.5 s: through the impact and well after
        mujoco.mj_step(m, d)
    _, vel, _ = demo.soft_body(LIB, d)
    crate = 2.0 * vel.sum(axis=0)
    after = 300.0 * d.qvel[0:3] + crate
    assert crate[0] > 100.0                 # the crate was hit
    assert d.qvel[0] < 4.0 - 0.3            # and slowed the ball
    assert np.abs(after - before).max() < 1e-3 * abs(before[0])


def test_one_advance_per_simulation_time():
    """The first forward at a time advances the interval the next mj_step
    integrates; more forwards at that time (a paused viewer) and that step
    itself do not advance again."""
    m, d = make(model_xml('<geom type="plane" size="5 5 .1"/>'))
    mujoco.mj_step(m, d)
    assert state(m, d)[5] == 4
    mujoco.mj_forward(m, d)
    assert state(m, d)[5] == 8
    for _ in range(5):
        mujoco.mj_forward(m, d)
    mujoco.mj_step(m, d)
    assert state(m, d)[5] == 8
    mujoco.mj_step(m, d)
    assert state(m, d)[5] == 12


def test_copy_and_reset():
    xml = Path(demo.MODEL).read_text(encoding="utf-8").replace(
        'value="truss_crate.jbeam"', f'value="{(ASSETS / "truss_crate.jbeam").as_posix()}"')
    m, d = make(xml)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    for _ in range(500):  # 1 s: past the first contact
        mujoco.mj_step(m, d)
    c = mujoco.MjData(m)
    mujoco.mj_copyData(c, m, d)
    for _ in range(100):
        mujoco.mj_step(m, d)
        mujoco.mj_step(m, c)
    np.testing.assert_array_equal(d.sensordata, c.sensordata)
    np.testing.assert_array_equal(demo.soft_body(LIB, d)[0], demo.soft_body(LIB, c)[0])
    mujoco.mj_resetData(m, d)
    pos, vel, _ = demo.soft_body(LIB, d)
    assert np.abs(vel).max() == 0.0
    assert pos[:, 2].min() == pytest.approx(0.02, abs=1e-6)


def test_wrecking_ball_demo():
    summary = demo.run_headless(3.0, None)
    assert summary["first_contact_s"] == pytest.approx(0.94, abs=0.05)
    assert summary["ball_speed_peak_mps"] > 5.0
    assert summary["ball_speed_after_peak_min_mps"] < 1.0   # stopped by the crate
    assert summary["beams_deformed"] >= 10
    assert summary["beams_broken"] >= 1
    assert summary["crate_centroid_shift_m"][0] > 0.2       # pushed away from the ball
