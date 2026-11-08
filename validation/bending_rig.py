# SPDX-License-Identifier: GPL-3.0-or-later
"""A virtual bend-test machine for OpenBeam cables, and the theory it is checked against.

The rig grips a cable at both ends and bends it under displacement control:
the far grip moves along the arc of a uniform bend of curvature kappa and turns
with it, as a pure-bending fixture does. The grip's torque, from the collider
wrench, is the load cell. A test loads to 1/R, then reverses until the moment
crosses zero; the curvature there is the permanent set. Nothing is released,
so no release dynamics enter the result, as in a real materials test.

The theory is a fibre-section model of the same lattice and materials: each
longitudinal beam (the conductors) is a bilinear elastoplastic fibre at its
offset from the bending axis, and each diagonal (the jacket) an elastic one,
at its midpoint's offset and with its inclination. It knows nothing of the
3-D lattice, its shear, its contact or its end effects, so agreement is a real
check, not a restatement.

    python validation/bending_rig.py 0.08 0.05      # bend radii, m
"""

from __future__ import annotations

import math
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "isaacsim/exts/openbeam.isaacsim.softbody/openbeam/isaacsim/softbody"))
import core  # noqa: E402


class Cable:
    """A generated cable: its JBeam, materials and lattice data."""

    def __init__(self, length=0.3, segment=0.01, section=4, extra=(), directory: Path | None = None):
        self.length, self.segment, self.section = length, segment, section
        self.sections = round(length / segment) + 1
        directory = directory or Path(tempfile.mkdtemp(prefix="openbeam_cable_"))
        self.jbeam = directory / f"cable_n{section}.jbeam"
        out = subprocess.run([sys.executable, str(REPO / "assets/make_cable.py"), "--out", str(self.jbeam),
                              "--length", str(length), "--segment", str(segment), "--section", str(section), *extra],
                             check=True, capture_output=True, text=True).stdout
        self.max_step = float(out.split("largest stable step")[1].split("us")[0]) * 1e-6
        self.materials = self.jbeam.with_suffix(".materials")
        text = self.materials.read_text()
        self.lattice_radius = float(text.split("lattice radius")[1].split()[0])
        rows = [l.split() for l in text.splitlines() if l and not l.startswith("#")]
        (_, _, self.area, self.e_long, self.yield_long, self.hard_long, _, _), = [
            tuple(map(float, r)) for r in rows if r[0] != "twist"]
        self.gj = float(text.split("twist elements: GJ")[1].split()[0])     # the cable's, N m^2
        self.k_diag = [float(x) for x in re.findall(r'"beamSpring": ([0-9.e+-]+)', self.jbeam.read_text())][1]

    def world(self, gravity=(0.0, 0.0, 0.0)) -> core.World:
        w = core.World(self.jbeam)
        w.load_materials(self.materials)
        w.set_step(0.6 * self.max_step)
        w.set_node_damping(6.0)
        w.set_gravity(gravity)
        return w

    def centreline(self, w: core.World) -> np.ndarray:
        return w.positions()[: self.sections * self.section].reshape(self.sections, self.section, 3).mean(axis=1)

    def fibres(self):
        """(offset across the bend, area, E, yield stress, hardening, cos of inclination) per fibre."""
        n, r, s = self.section, self.lattice_radius, self.segment
        side = 2 * r * math.sin(math.pi / n)
        cos_phi = s / math.hypot(s, side)
        ang = [math.pi / 2 + (j + 0.5 * (n % 2 == 0)) * 2 * math.pi / n for j in range(n)]
        ys = [r * math.cos(a) for a in ang]
        out = [(y, self.area, self.e_long, self.yield_long, self.hard_long, 1.0) for y in ys]
        e_diag = self.k_diag * math.hypot(s, side) / self.area
        for j in range(n):
            for k in (1, -1):
                out.append((0.5 * (ys[j] + ys[(j + k) % n]), self.area, e_diag, math.inf, 0.0, cos_phi))
        return out

    def theory_moment(self, kappa_path) -> np.ndarray:
        """Bending moment along a curvature history, bilinear fibres with isotropic hardening."""
        fib = self.fibres()
        plastic, accumulated, moments = [0.0] * len(fib), [0.0] * len(fib), []
        for kappa in kappa_path:
            m = 0.0
            for i, (y, a, e, sy, h, c) in enumerate(fib):
                eps = kappa * y * c * c
                trial = e * (eps - plastic[i])
                flow = sy + h * accumulated[i]
                if abs(trial) > flow:
                    dl = (abs(trial) - flow) / (e + h)
                    plastic[i] += math.copysign(dl, trial)
                    accumulated[i] += dl
                    trial = e * (eps - plastic[i])
                m += -a * trial * y * c
            moments.append(m)
        return np.array(moments)

    def theory_set(self, radius: float) -> float:
        """Residual curvature after bending to `radius` and unloading to zero moment."""
        k = 1.0 / radius
        path = np.r_[np.linspace(0, k, 400), np.linspace(k, -k, 800)]
        m = self.theory_moment(path)
        for i in range(400, len(path) - 1):
            if np.sign(m[i]) != np.sign(m[i + 1]):
                return path[i] + (path[i + 1] - path[i]) * (0 - m[i]) / (m[i + 1] - m[i])
        return 0.0


def bend_test(cable: Cable, radius: float, increments: int = 100, settle: float = 0.004) -> dict:
    """Load to 1/radius and unload to zero moment; returns the peak moment, the set and the counters."""
    w = cable.world()
    grip = (0.002, 0.002, 0.002)
    clamp, end = w.add_collider(core.SHAPE_BOX, grip), w.add_collider(core.SHAPE_BOX, grip)
    seg, sections = cable.segment, cable.sections
    x_end = seg * (sections - 2)
    w.set_collider_state(clamp, (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
    w.set_collider_state(end, (x_end, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
    for k in (0, 1):
        w.attach(f"s{k}_*", clamp)
    for k in (sections - 2, sections - 1):
        w.attach(f"s{k}_*", end)
    span = x_end - seg

    def pose(kappa):
        th = kappa * span
        if abs(th) < 1e-9:
            return (x_end, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
        rr = 1.0 / kappa
        return (seg + rr * math.sin(th), rr * (1 - math.cos(th)), 0.0), (math.cos(th / 2), 0, 0, math.sin(th / 2))

    def moment_at(kappa):
        p, q = pose(kappa)
        for _ in range(int(round(settle / 0.001))):
            w.set_collider_state(end, p, q, (0, 0, 0), (0, 0, 0))
            w.advance(0.001)
        return w.collider_wrench(end, p)[1][2]

    def move(k0, k1, dt=0.002):
        substeps = max(1, int(abs(k1 - k0) * span / 0.0005))   # at most 0.5 mrad of grip rotation each
        for i in range(substeps):
            a, b = k0 + (k1 - k0) * i / substeps, k0 + (k1 - k0) * (i + 1) / substeps
            pa, qa = pose(a)
            pb, _ = pose(b)
            w.set_collider_state(end, pa, qa, tuple((y - x) / dt for x, y in zip(pa, pb)), (0, 0, (b - a) * span / dt))
            w.advance(dt)

    k_max, dk = 1.0 / radius, 2.0 / (radius * increments)
    k, peak = 0.0, 0.0
    while k < k_max - 1e-12:
        k_next = min(k_max, k + dk)
        move(k, k_next)
        k, peak = k_next, moment_at(k_next)
    m_prev, k_set = peak, float("nan")
    while k > -k_max:
        k_next = k - dk
        move(k, k_next)
        m = moment_at(k_next)
        if np.sign(m) != np.sign(m_prev):
            k_set = k_next + (k - k_next) * (0 - m) / (m_prev - m)
            break
        k, m_prev = k_next, m
    s = w.stats()
    return {"radius": radius, "peak_moment": abs(peak), "set_curvature": k_set,
            "theory_peak_moment": abs(cable.theory_moment(np.linspace(0, k_max, 400))[-1]),
            "theory_set_curvature": cable.theory_set(radius),
            "yielded": s["beams_deformed"], "torn": s["beams_broken"]}


def twist_test(cable: Cable, angle: float = 0.5, seconds: float = 0.5, settle: float = 0.3) -> dict:
    """Grip both ends, turn the far grip about the cable's axis by `angle` and hold: the torque on it
    against St-Venant's GJ angle / length, with GJ the cable's (make_cable.py's EI / 1.4 unless set)."""
    w = cable.world()
    grip = (0.002, 0.002, 0.002)
    clamp, end = w.add_collider(core.SHAPE_BOX, grip), w.add_collider(core.SHAPE_BOX, grip)
    seg, sections = cable.segment, cable.sections
    x_end = seg * (sections - 2)
    w.set_collider_state(clamp, (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
    w.set_collider_state(end, (x_end, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
    for k in (0, 1):
        w.attach(f"s{k}_*", clamp)
    for k in (sections - 2, sections - 1):
        w.attach(f"s{k}_*", end)
    dt, turned = 0.001, 0.0
    for i in range(int(round((seconds + settle) / dt))):
        rate = angle / seconds if i < seconds / dt else 0.0
        w.set_collider_state(end, (x_end, 0, 0), (math.cos(turned / 2), math.sin(turned / 2), 0, 0), (0, 0, 0),
                             (rate, 0, 0))
        w.advance(dt)
        turned = min(angle, turned + rate * dt)
    torque = -w.collider_wrench(end, (x_end, 0, 0))[1][0]
    gj = cable.gj
    span = x_end - seg
    s = w.stats()
    return {"angle": angle, "torque": torque, "theory_torque": gj * angle / span, "gj": gj,
            "yielded": s["beams_deformed"], "torn": s["beams_broken"]}


def cantilever_sag(cable: Cable, seconds: float = 2.5) -> tuple[float, float]:
    """Tip sag of the cable clamped at one end under gravity, and Euler-Bernoulli's w L^4 / 8 EI."""
    w = cable.world(gravity=(0.0, 0.0, -9.81))
    clamp = w.add_collider(core.SHAPE_BOX, (0.002, 0.002, 0.002))
    w.set_collider_state(clamp, (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
    for k in (0, 1):
        w.attach(f"s{k}_*", clamp)
    for _ in range(int(seconds / 0.01)):
        w.set_collider_state(clamp, (0, 0, 0), (1, 0, 0, 0), (0, 0, 0), (0, 0, 0))
        w.advance(0.01)
    sag = -float(cable.centreline(w)[-1, 2])
    free = cable.length - cable.segment
    weight_per_m = w.total_mass * 9.81 / cable.length
    ei = 0.002   # make_cable.py's default
    return sag, weight_per_m * free ** 4 / (8 * ei)


if __name__ == "__main__":
    cable = Cable()
    t = twist_test(cable)
    print(f"twist {t['angle']} rad: torque {t['torque']:.5f} N m (St-Venant {t['theory_torque']:.5f}, "
          f"GJ {t['gj']:.4g} N m^2)", flush=True)
    for radius in [float(a) for a in sys.argv[1:]] or [0.3, 0.12, 0.08, 0.05]:
        r = bend_test(cable, radius)

        def as_radius(k):
            return "none" if not k == k or abs(k) < 1e-3 else f"{1000 / abs(k):.0f} mm"
        print(f"R {1000 * radius:.0f} mm: peak moment {r['peak_moment']:.4f} N m (theory {r['theory_peak_moment']:.4f}); "
              f"set radius {as_radius(r['set_curvature'])} (theory {as_radius(r['theory_set_curvature'])}); "
              f"yielded {r['yielded']}, torn {r['torn']}", flush=True)
