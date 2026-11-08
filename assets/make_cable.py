#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Writes a JBeam cable, and its beam materials, from physical properties.

Geometry. A cable is a prism lattice: N nodes per cross-section (default
4), spaced `segment` apart along x, on a circle of radius r_lattice.
Longitudinal beams carry tension and bending; ring beams (sides and braces)
hold each section's shape; two helices of diagonal beams resist shear and
torsion. (In a straight chain of single nodes, skip beams give no bending
stiffness to first order.) With s the segment length and the diagonals, nearly
parallel to the axis, at k_diag = c k_long (c about 0.5):
  axial stiffness  EA = N (1 + 2 c) k_long s
  bending          EI = N k_long s r^2 (0.5 + c cos^2(pi / N))
which the script solves for k_long and r_lattice. N = 4 or more keeps the
cross-section symmetric about both bending axes; with N = 3 a fibre that has
yielded tilts the section's stiff axes, and in-plane bending then drags the
cable out of plane, exaggerating its permanent set.

Material. A cable is a composite: copper conductors that yield and a polymer
jacket that mostly springs back. The longitudinal beams are the conductors:
they get the engine's calibrated elastoplastic damage law with physical
parameters (written to <out>.materials, loaded with ob_world_load_materials or
the plugins' `materials` option). The diagonal and ring beams are the jacket
and stay elastic: they spread a bend along the cable instead of letting it fold
at one plastic hinge, and spring part of the conductors' set back, as a real
cable does. Bending to radius R strains a fibre at r_lattice by r/R:
  yield radius     bent tighter, the conductors yield and the cable keeps a
                   permanent set (coil memory)
  damage strain    accumulated plastic strain at which the conductors' ductility
                   starts to run out (copper: about 0.3, a kink rather than a
                   handling bend)
  break strain     accumulated plastic strain at which a conductor tears (copper
                   elongates 30 to 45 % before it breaks)
The rated minimum bend radius (TIA-568: four diameters for Cat6) is a
performance limit, not a mechanical one: it is written to the .materials
header for hosts to check the cable's shape against, and is not a damage
threshold. The engine's damage criterion accumulates plastic strain, so
damage onset is set from copper's ductility rather than from one bend: a cable
swinging in a gripper yields back and forth slightly and must not exhaust it.


What is physical and what is not:
  - mass per metre, diameter, bending stiffness and min bend radius are cable
    data (defaults: Cat6, from the sources in the README);
  - the damage and break strains are copper's ductility (handbook values);
  - the yield radius is an estimate until measured (README: calibration);
  - the torsional stiffness GJ defaults to an isotropic rod's, EI / (1 + nu)
    with nu = 0.4; published cables range from about 0.16 EI to 2.5 EI. A
    lattice of axial beams has almost none of its own (its diagonals lie
    nearly along the axis), so the .materials file adds twist elements for
    the rest: without them a tightly bent bay rolls over instead of bending
    its conductors, and the cable never takes the set it should;
  - EA is an effective value: copper cables are nearly inextensible, which an
    explicit integrator follows only with a tiny step. The defaults stretch
    about 0.06 % under 1 N. The script prints the largest stable step for what
    it writes.

  python make_cable.py --out cat6_cable.jbeam --rj45          # Cat6 with an RJ45 plug
  python make_cable.py --out lc_fibre.jbeam --diameter 0.002 --mass-per-metre 0.0095 \\
      --ei 0.0001 --ea 800 --min-bend-radius 0.0075 --yield-radius 0.05
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

# An RJ45 plug with its boot: 30 x 12 x 14 mm, 8 g (plug about 2 g, boot and strain relief the rest).
RJ45_SIZE = (0.030, 0.012, 0.014)
RJ45_MASS = 0.008


def build(args) -> tuple[str, str, dict]:
    length, segment = args.length, args.segment
    sections = max(2, round(length / segment) + 1)
    s = length / (sections - 1)
    n = args.section
    c = 0.5                                               # k_diag / k_long, refined below
    for _ in range(20):
        k_long = args.ea / (n * (1.0 + 2.0 * c) * s)
        r = math.sqrt(args.ei / (n * k_long * s * (0.5 + c * math.cos(math.pi / n) ** 2)))
        ring = 2.0 * r * math.sin(math.pi / n)            # one side of the cross-section
        diag = math.hypot(s, ring)
        c = 0.5 * s / diag
    node_mass = args.mass_per_metre * length / (n * sections)
    k_ring = k_long * s / ring                            # as stiff per unit length as the fibres
    k_diag = c * k_long

    def damp(k: float, m: float) -> float:               # a fraction of critical, per beam
        return 2.0 * args.damping_ratio * math.sqrt(k * m)

    def nid(i: int, j: int) -> str:
        return f"s{i}_{j}"

    nodes, beams_long, beams_diag, beams_ring = [], [], [], []
    for i in range(sections):
        for j in range(n):
            a = math.pi / 2.0 + (j + 0.5 * (n % 2 == 0)) * 2.0 * math.pi / n
            nodes.append((nid(i, j), i * s, r * math.cos(a), args.height + r * math.sin(a)))
            # sides, and braces across the section so it keeps its shape
            for step in range(1, n // 2 + 1):
                if step < n / 2 or j < n // 2:
                    beams_ring.append((nid(i, j), nid(i, (j + step) % n)))
            if i + 1 < sections:
                beams_long.append((nid(i, j), nid(i + 1, j)))
                beams_diag.append((nid(i, j), nid(i + 1, (j + 1) % n)))
                beams_diag.append((nid(i, j), nid(i + 1, (j - 1) % n)))

    plug_nodes, plug_beams, k_plug, plug_mass = [], [], 0.0, 0.0
    if args.rj45:
        lx, ly, lz = RJ45_SIZE
        plug_mass = RJ45_MASS / 8
        for k in range(8):                                # back face (x = length) first, then front
            x = length + (lx if k & 4 else 0.0)
            y = (ly / 2) * (1 if k & 1 else -1)
            z = args.height + (lz / 2) * (1 if k & 2 else -1)
            plug_nodes.append((f"rj_{k}", x, y, z))
        ids = [n for n, *_ in plug_nodes]
        plug_beams = [(ids[a], ids[b]) for a in range(8) for b in range(a + 1, 8)]
        last = [nid(sections - 1, j) for j in range(n)]
        before = [nid(sections - 2, j) for j in range(n)]
        plug_beams += [(n, ids[k]) for n in last for k in range(4)]       # the end section into the back face
        plug_beams += [(n, ids[k]) for n in before for k in range(4)]     # and the strain relief behind it
        # Rigid next to the cable: stiff, yet stable at the step the cable needs.
        k_plug = 0.15 * plug_mass * (2.0 / (1.5 * 60e-6)) ** 2 / 7

    def rows(pairs):
        return [f'        ["{a}", "{b}"],' for a, b in pairs]

    def node_rows(entries):
        return [f'        ["{n}", {x:.6f}, {y:.6f}, {z:.6f}],' for n, x, y, z in entries]

    lines = ["{", f'"{args.name}": {{',
             f'    "information": {{"name": "OpenBeam cable {args.name}", "authors": "OpenBeam"}},',
             '    "slotType": "main",',
             '    "nodes": [',
             '        ["id", "posX", "posY", "posZ"],',
             f'        {{"nodeWeight": {node_mass:.6g}, "frictionCoef": 0.8}},']
    lines += node_rows(nodes)
    if plug_nodes:
        lines += [f'        {{"nodeWeight": {plug_mass:.6g}, "frictionCoef": 0.5}},']
        lines += node_rows(plug_nodes)
    lines[-1] = lines[-1].rstrip(",")
    # Beam order matters: the .materials file addresses beams by index.
    lines += ["    ],", '    "beams": [', '        ["id1:", "id2:"],',
              f'        {{"beamSpring": {k_long:.6g}, "beamDamp": {damp(k_long, node_mass):.6g}, '
              '"beamDeform": "FLT_MAX", "beamStrength": "FLT_MAX"},']
    lines += rows(beams_long)
    lines += [f'        {{"beamSpring": {k_diag:.6g}, "beamDamp": {damp(k_diag, node_mass):.6g}}},']
    lines += rows(beams_diag)
    lines += [f'        {{"beamSpring": {k_ring:.6g}, "beamDamp": {damp(k_ring, node_mass):.6g}}},']
    lines += rows(beams_ring)
    if plug_beams:
        lines += [f'        {{"beamSpring": {k_plug:.6g}, "beamDamp": {damp(k_plug, plug_mass):.6g}}},']
        lines += rows(plug_beams)
    lines[-1] = lines[-1].rstrip(",")
    lines += ["    ]", "}", "}", ""]

    # Physical fibre material. Each fibre stands for 1/N of the cable's section.
    area = math.pi * (args.diameter / 2.0) ** 2 / n
    eps_y = r / args.yield_radius                         # strain where permanent set begins
    hardening_fraction = args.hardening

    def plastic_at(radius: float) -> float:               # fibre plastic strain when bent to radius
        total = r / radius
        return max(0.0, (total - eps_y) / (1.0 + hardening_fraction))

    onset, at_break = args.damage_strain, args.break_strain

    def material(k: float, beam_length: float) -> tuple:
        e = k * beam_length / area
        sigma_y = e * eps_y
        h = e * hardening_fraction
        # Damage-driver work from onset to the break radius (stress times plastic strain).
        capacity = (sigma_y + h * 0.5 * (onset + at_break)) * max(at_break - onset, 1e-9)
        return area, e, sigma_y, h, onset, capacity

    n_long, n_diag = len(beams_long), len(beams_diag)
    mat_long = material(k_long, s)
    fmt = "{} {} {:.6g} {:.6g} {:.6g} {:.6g} {:.6g} {:.6g}"
    materials = "\n".join([
        f"# Beam materials for {args.out.name} (make_cable.py): first count area_m2 E_pa yield_pa",
        "#   hardening_pa damage_onset_plastic_strain damage_capacity_j_per_m3",
        f"# yield radius {args.yield_radius} m, damage strain {args.damage_strain}, "
        f"break strain {args.break_strain}, lattice radius {r:.6g} m",
        f"# rated min bend radius {args.min_bend_radius} m (a shape limit for hosts to check)",
        "# longitudinal fibres: the conductors",
        fmt.format(0, n_long, *mat_long),
        "# the diagonal and ring beams (the jacket) and any plug stay elastic",
        "",
    ])
    # Torsion: what the lattice's diagonals give (each changes length r^2 sin(2 pi / n) / diag per radian of
    # roll between sections), and twist elements for the rest of the cable's GJ.
    gj = args.gj if args.gj is not None else args.ei / 1.4
    gj_lattice = 2 * n * k_diag * (r * r * math.sin(2 * math.pi / n) / diag) ** 2 * s
    gj_twist = max(0.0, gj - gj_lattice)
    materials += (f"# twist elements: GJ {gj:.4g} N m^2 for the cable, {gj_lattice:.3g} from the lattice's diagonals\n"
                  f"twist 0 {n} {sections} {gj_twist:.6g} {args.damping_ratio:.6g}\n")
    k_node = 2 * k_long + (n - 1) * k_ring + 4 * k_diag
    max_step = 2.0 / math.sqrt(k_node / node_mass)
    if gj_twist > 0:
        max_step = min(max_step, 2.0 / math.sqrt(4 * (gj_twist / s) / (n * node_mass * r * r)))
    if plug_nodes:
        max_step = min(max_step, 2.0 / math.sqrt(7 * k_plug / plug_mass))
    info = {"sections": sections, "per_section": n, "nodes": len(nodes) + len(plug_nodes),
            "beams": n_long + n_diag + len(beams_ring) + len(plug_beams), "lattice_radius_m": r,
            "node_mass_kg": node_mass, "max_stable_step_s": max_step,
            "contact_radius_m": max(0.0, args.diameter / 2.0 - r), "gj_n_m2": gj, "gj_lattice_n_m2": gj_lattice}
    return "\n".join(lines), materials, info


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("cat6_cable.jbeam"))
    ap.add_argument("--name", default="cable", help="the JBeam part name")
    ap.add_argument("--length", type=float, default=1.0, help="m")
    ap.add_argument("--segment", type=float, default=0.025, help="m between cross-sections")
    ap.add_argument("--section", type=int, default=4, help="nodes per cross-section, 3 or more (default 4)")
    ap.add_argument("--diameter", type=float, default=0.006, help="m (Cat6: 6 to 6.5 mm)")
    ap.add_argument("--mass-per-metre", type=float, default=0.043, help="kg/m (Cat6: about 0.043)")
    ap.add_argument("--ea", type=float, default=1600.0, help="effective axial stiffness, N")
    ap.add_argument("--ei", type=float, default=0.002, help="bending stiffness, N m^2 (Cat6: about 1e-3 to 1e-2)")
    ap.add_argument("--yield-radius", type=float, default=0.15,
                    help="m; bent tighter, the cable keeps a set (an estimate until measured)")
    ap.add_argument("--min-bend-radius", type=float, default=0.025,
                    help="m; the rated limit (Cat6: 4 x diameter, TIA-568), checked against the shape")
    ap.add_argument("--damage-strain", type=float, default=0.3,
                    help="accumulated plastic strain where the conductors' damage begins (copper: about 0.3)")
    ap.add_argument("--break-strain", type=float, default=0.45,
                    help="accumulated plastic strain where a conductor tears (copper: 0.3 to 0.45)")
    ap.add_argument("--hardening", type=float, default=0.05, help="post-yield slope as a fraction of E")
    ap.add_argument("--damping-ratio", type=float, default=0.05, help="fraction of critical damping per beam")
    ap.add_argument("--height", type=float, default=0.0, help="m; z of the cable's axis")
    ap.add_argument("--rj45", action="store_true", help="end the cable in an RJ45 plug (nodes rj_0 to rj_7)")
    ap.add_argument("--gj", type=float, help="torsional stiffness, N m^2 (default EI / 1.4, an isotropic rod's)")
    args = ap.parse_args()
    if not args.min_bend_radius < args.yield_radius or not 0 < args.damage_strain < args.break_strain:
        ap.error("need min bend radius < yield radius and 0 < damage strain < break strain")
    text, materials, info = build(args)
    args.out.write_bytes(text.encode("utf-8"))
    sidecar = args.out.with_suffix(".materials")
    sidecar.write_bytes(materials.encode("utf-8"))
    print(f"wrote {args.out} and {sidecar.name}: {info['nodes']} nodes, {info['beams']} beams; lattice radius "
          f"{1000 * info['lattice_radius_m']:.2f} mm, node mass {1000 * info['node_mass_kg']:.3f} g; "
          f"largest stable step {1e6 * info['max_stable_step_s']:.0f} us; "
          f"contact radius {1000 * info['contact_radius_m']:.2f} mm; GJ {info['gj_n_m2']:.3g} N m^2 "
          f"({info['gj_lattice_n_m2']:.2g} from the lattice)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
