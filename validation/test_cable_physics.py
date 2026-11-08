# SPDX-License-Identifier: GPL-3.0-or-later
"""Physics validation of OpenBeam cables against theory (bending_rig.py describes the rig).

    python -m pytest validation -q

Each check compares the 3-D lattice with an independent model that shares only
the inputs: beam theory for the elastic response, a fibre-section model for
yield and the set left after unloading. The tolerances are what the lattice
achieves today, and the README reports the measured values.
"""

from __future__ import annotations

import math

import pytest

from bending_rig import Cable, bend_test, cantilever_sag, twist_test


@pytest.fixture(scope="module")
def cable():
    return Cable(length=0.2, segment=0.01)


def test_cantilever_sag_matches_euler_bernoulli():
    """Clamped at one end, the cable sags under its own weight as w L^4 / 8 EI predicts (small
    deflection: 10 mm on 0.14 m). The lattice sags 7 % more: its bays also shear, which an
    Euler-Bernoulli beam does not."""
    sag, theory = cantilever_sag(Cable(length=0.15, segment=0.0125))
    assert sag == pytest.approx(theory, rel=0.1)


def test_bending_stiffness_and_no_set_below_yield(cable):
    """Bent more gently than the yield radius, the moment is EI kappa and the cable springs back straight."""
    r = bend_test(cable, 0.3, increments=40)
    assert r["peak_moment"] == pytest.approx(r["theory_peak_moment"], rel=0.05)
    assert r["yielded"] == 0 and r["torn"] == 0
    assert not abs(r["set_curvature"]) > 0.3          # a set radius of over 3 m: none


def test_permanent_set_matches_fibre_theory(cable):
    """Bent past yield, the conductors yield, and the set left after unloading matches the fibre-section
    model: the cable remembers the bend."""
    r = bend_test(cable, 0.07, increments=80)
    assert r["yielded"] > 0 and r["torn"] == 0
    assert r["peak_moment"] == pytest.approx(r["theory_peak_moment"], rel=0.05)
    assert r["set_curvature"] == pytest.approx(r["theory_set_curvature"], rel=0.08)
    assert 1.0 / abs(r["set_curvature"]) > 0.07       # it springs part of the way back


def test_torsion_matches_st_venant(cable):
    """Twisted between its grips, the cable resists with GJ angle / length: the lattice's own diagonals
    and its twist elements together give the cable's torsional stiffness, and it stays elastic. (The
    elements are sized from GJ, so this checks the implementation, not GJ itself.)"""
    r = twist_test(cable, 0.5)
    assert r["torque"] == pytest.approx(r["theory_torque"], rel=0.02)
    assert r["yielded"] == 0 and r["torn"] == 0
