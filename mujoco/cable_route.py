# SPDX-License-Identifier: GPL-3.0-or-later
"""The patching scene's layout and the gripper's route, shared by the MuJoCo
demo (cable_demo.py, models/cable_patch.xml) and the Isaac Sim one
(isaacsim/demo_isaacsim_cable.py). Pure Python and NumPy: no simulator."""

from __future__ import annotations

import math

import numpy as np

SECTIONS, PER_SECTION = 51, 4     # cat6_cable.jbeam: 51 cross-sections of 4 nodes, then the plug's 8
PLUG = slice(SECTIONS * PER_SECTION, SECTIONS * PER_SECTION + 8)
CABLE_ORIGIN = (-0.45, 0.25, 0.0)  # where the cable starts, in the outlet
PLUG_START = (0.565, 0.25, 0.0082)
RATED_MIN_BEND_RADIUS = 0.025      # m: Cat6, four diameters (TIA-568), a shape limit
RING = (0.25, 0.0, 0.12)
PORT = (0.30, -0.258, 0.12)        # the open port in the 1U panel at (0.30, -0.262, 0.12); its face at y -0.25

def waypoints(ring_x: float = RING[0], port_x: float = 0.30, port_z: float = 0.12, ring_z: float = RING[2],
              turn_while_lifting: bool = False):
    """Gripper waypoints (time s, position m, yaw degrees, gripping): take the plug, slide it back and aside
    to give the cable slack (it lies straight from the outlet, so lifting it as it lies would stretch it),
    lift, go through the ring along -y and on until the cable behind runs straight through, then across to
    the port and in. The plug turns to face the port while it is carried, or, with turn_while_lifting,
    while it is lifted next to the cable lying behind it."""
    x0, y0, z0 = PLUG_START
    lift_yaw = -90.0 if turn_while_lifting else 0.0
    return [
        (0.0, (x0, y0, 0.08), 0.0, False),
        (0.6, (x0, y0, z0), 0.0, False),
        (0.7, (x0, y0, z0), 0.0, True),
        (1.5, (x0 - 0.10, y0 - 0.06, z0 + 0.003), 0.0, True),
        (2.2, (x0 - 0.10, y0 - 0.06, 0.14), lift_yaw, True),
        (3.4, (ring_x, 0.12, ring_z), -90.0, True),
        (4.4, (ring_x, -0.12, ring_z), -90.0, True),
        (5.2, (port_x, -0.2, port_z), -90.0, True),
        (6.2, (port_x, -0.25, port_z), -90.0, True),     # the plug's front 15 mm past the panel face
        (6.8, (port_x, -0.25, port_z), -90.0, True),
    ]


WAYPOINTS = waypoints()
DURATION = WAYPOINTS[-1][0]


def gripper_target(t: float, path=WAYPOINTS):
    """Position, yaw (degrees) and grip at time t: smooth steps between waypoints."""
    for (t0, p0, y0, g0), (t1, p1, y1, _) in zip(path, path[1:]):
        if t <= t1:
            u = 0.0 if t1 == t0 else min(1.0, max(0.0, (t - t0) / (t1 - t0)))
            u = u * u * (3.0 - 2.0 * u)
            return np.array(p0) + u * (np.array(p1) - np.array(p0)), y0 + u * (y1 - y0), g0
    _, p1, y1, g1 = path[-1]
    return np.array(p1), y1, g1


def yaw_quat(degrees: float) -> np.ndarray:
    a = math.radians(degrees) / 2.0
    return np.array([math.cos(a), 0.0, 0.0, math.sin(a)])


def centreline(points: np.ndarray) -> np.ndarray:
    return points[: SECTIONS * PER_SECTION].reshape(SECTIONS, PER_SECTION, 3).mean(axis=1)


def bend_radii(points: np.ndarray) -> np.ndarray:
    """Radius of the circle through each three consecutive centreline points."""
    a, b, c = points[:-2], points[1:-1], points[2:]
    ab, bc, ca = np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)
    area2 = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    with np.errstate(divide="ignore"):
        return np.where(area2 > 1e-12, ab * bc * ca / (2.0 * area2), np.inf)


def threads_ring(points: np.ndarray, centre=RING, half_opening=0.034) -> bool:
    """Whether the centreline crosses the ring's plane inside its opening (30 mm plus the cable's radius)."""
    y = points[:, 1] - centre[1]
    for i in np.nonzero(np.sign(y[:-1]) != np.sign(y[1:]))[0]:
        u = y[i] / (y[i] - y[i + 1])
        p = points[i] + u * (points[i + 1] - points[i])
        if abs(p[0] - centre[0]) < half_opening and abs(p[2] - centre[2]) < half_opening:
            return True
    return False


def plug_pose(points: np.ndarray):
    """The plug's centre and its axis, back face (rj_0..3) to front face (rj_4..7)."""
    plug = points[PLUG]
    axis = plug[4:].mean(axis=0) - plug[:4].mean(axis=0)
    return plug.mean(axis=0), axis / np.linalg.norm(axis), plug
