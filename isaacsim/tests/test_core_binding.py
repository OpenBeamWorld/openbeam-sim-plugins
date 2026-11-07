# SPDX-License-Identifier: GPL-3.0-or-later
"""The Isaac Sim extension's ctypes binding of openbeam_core, outside Isaac Sim.

    python -m pytest isaacsim/tests -q
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
CORE_PY = HERE.parent / "exts" / "openbeam.isaacsim.softbody" / "openbeam" / "isaacsim" / "softbody" / "core.py"
CRATE = HERE.parents[1] / "assets" / "truss_crate.jbeam"

_spec = importlib.util.spec_from_file_location("openbeam_core_binding", CORE_PY)
core = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(core)


def test_crate_loads_and_rests():
    w = core.World(CRATE, (0.0, 0.0, 0.02))
    assert (w.node_count, w.beam_count) == (100, 603)
    assert w.total_mass == pytest.approx(200.0)
    assert w.beam_nodes.shape == (603, 2)
    w.add_plane((0, 0, 0), (0, 0, 1))
    w.advance(2.0)
    p = w.positions()
    s = w.stats()
    assert p[:, 2].min() > -0.005
    assert p[:, 2].max() == pytest.approx(0.9, abs=0.02)   # holds its shape
    assert s["beams_broken"] == 0 and s["beams_deformed"] == 0
    # Only the 25 bottom nodes touch; at rest each re-enters contact on most steps.
    w.advance(0.05)
    per_step = (w.stats()["contacts"] - s["contacts"]) / 100.0
    assert 5.0 < per_step <= 25.0


def test_bad_path_raises():
    with pytest.raises(ValueError):
        core.World(HERE / "missing.jbeam")


def test_moving_collider_momentum_balance():
    w = core.World(CRATE, (0.0, 0.0, 0.0))
    w.set_gravity((0, 0, 0))
    cid = w.add_collider(core.SHAPE_SPHERE, (0.4, 0, 0))
    center = np.array([-0.6, 0.6, 0.45])
    vel = np.array([3.0, 0.0, 0.0])
    impulse = np.zeros(3)
    for _ in range(400):
        w.set_collider_state(cid, center, (1, 0, 0, 0), vel, (0, 0, 0))
        n = w.advance(1.0 / 200.0)
        f, _ = w.collider_wrench(cid, center)
        impulse += f * n * 0.0005
        center = center + vel / 200.0
    momentum = 2.0 * w.velocities().sum(axis=0)
    assert momentum[0] > 50.0
    np.testing.assert_allclose(momentum, -impulse, atol=1e-3 * momentum[0])


def test_reset_restores_digest():
    w = core.World(CRATE)
    w.add_plane((0, 0, 0), (0, 0, 1))
    d0 = w.digest()
    w.advance(0.5)
    assert w.digest() != d0
    w.reset()
    assert w.digest() == d0


def test_beam_damage_reports_yield_and_tears():
    """A sphere driven through the crate at 8 m/s yields and tears beams; beam_damage reports both
    and agrees with the counters, and a reset clears it."""
    w = core.World(CRATE, (0.0, 0.0, 0.0))
    w.add_plane((0, 0, 0), (0, 0, 1))
    cid = w.add_collider(core.SHAPE_SPHERE, (0.4, 0, 0))
    plastic, damage = w.beam_damage()
    assert plastic.shape == damage.shape == (w.beam_count,)
    assert not plastic.any() and not damage.any()
    center, vel = np.array([-0.6, 0.6, 0.45]), np.array([8.0, 0.0, 0.0])
    for _ in range(80):
        w.set_collider_state(cid, center, (1, 0, 0, 0), vel, (0, 0, 0))
        w.advance(1.0 / 200.0)
        center = center + vel / 200.0
    plastic, damage = w.beam_damage()
    _, _, _, broken = w.beam_state()
    s = w.stats()
    assert s["beams_broken"] >= 1 and s["beams_deformed"] >= 10
    assert np.all(damage[broken] == 1.0)
    assert int((np.abs(plastic) > 1e-4).sum()) >= s["beams_deformed"] - s["beams_broken"]
    w.reset()
    plastic, damage = w.beam_damage()
    assert not plastic.any() and not damage.any()
