#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Writes truss_crate.jbeam, the demo structure of the simulator plugins.

A box lattice of nodes joined along every edge and every face diagonal of
each cell (no body diagonals): a crate that holds its shape at rest, dents
when struck and tears when struck hard. Authored in BeamNG's Z-up SI frame
with its origin at the crate's bottom corner.

usage: python make_truss_crate.py [--out truss_crate.jbeam]

The LeRobot push task's soft box is the same lattice, smaller and lighter:

  python make_truss_crate.py --out ../lerobot/openbeam_push/soft_box.jbeam --name soft_box \
      --nodes 4 4 3 --spacing 0.1 --node-weight 0.1 --spring 20000 --damp 2 --deform 120 --strength 400
"""

from __future__ import annotations

import argparse
import itertools
import math
from pathlib import Path

NX, NY, NZ = 5, 5, 4      # nodes per axis
SPACING = 0.3             # m
NODE_WEIGHT = 2.0         # kg, 200 kg in all
# A yield force low enough that a 300 kg ball at 7.7 m/s caves in the struck
# face (about 300 beams yield, about 50 tear) and high enough that the crate
# does not yield under its own weight. Calibrated beams tear by accumulated
# damage, so beamStrength does not set when they break.
BEAM = {"beamSpring": 400000, "beamDamp": 40, "beamDeform": 600, "beamStrength": 6000}


def node_id(i: int, j: int, k: int) -> str:
    return f"c{i}{j}{k}"


def build(name: str = "truss_crate", title: str = "OpenBeam truss crate") -> str:
    nodes = [(node_id(i, j, k), i * SPACING, j * SPACING, k * SPACING)
             for k in range(NZ) for j in range(NY) for i in range(NX)]
    index = {(i, j, k): node_id(i, j, k) for i in range(NX) for j in range(NY) for k in range(NZ)}
    beams = []
    limit = SPACING * math.sqrt(2.0) + 1e-6
    keys = sorted(index)
    for a, b in itertools.combinations(keys, 2):
        d = [abs(a[n] - b[n]) for n in range(3)]
        if max(d) > 1 or sum(d) == 0:
            continue
        if math.dist([x * SPACING for x in a], [x * SPACING for x in b]) <= limit:
            beams.append((index[a], index[b]))

    lines = ['{', f'"{name}": {{',
             f'    "information": {{"name": "{title}", "authors": "OpenBeam"}},',
             '    "slotType": "main",',
             '    "nodes": [',
             '        ["id", "posX", "posY", "posZ"],',
             f'        {{"nodeWeight": {NODE_WEIGHT}, "frictionCoef": 1.0}},']
    lines += [f'        ["{n}", {x:.3f}, {y:.3f}, {z:.3f}],' for n, x, y, z in nodes]
    lines[-1] = lines[-1].rstrip(',')
    params = ", ".join(f'"{k}": {v:g}' for k, v in BEAM.items())
    lines += ['    ],', '    "beams": [', '        ["id1:", "id2:"],', f'        {{{params}}},']
    lines += [f'        ["{a}", "{b}"],' for a, b in beams]
    lines[-1] = lines[-1].rstrip(',')
    lines += ['    ]', '}', '}', '']
    return "\n".join(lines)


def main() -> int:
    global NX, NY, NZ, SPACING, NODE_WEIGHT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(Path(__file__).with_name("truss_crate.jbeam")))
    ap.add_argument("--name", default="truss_crate", help="the JBeam part name")
    ap.add_argument("--nodes", type=int, nargs=3, default=(NX, NY, NZ), metavar=("NX", "NY", "NZ"))
    ap.add_argument("--spacing", type=float, default=SPACING, help="m")
    ap.add_argument("--node-weight", type=float, default=NODE_WEIGHT, help="kg")
    ap.add_argument("--spring", type=float, default=BEAM["beamSpring"], help="beamSpring, N/m")
    ap.add_argument("--damp", type=float, default=BEAM["beamDamp"], help="beamDamp, N s/m")
    ap.add_argument("--deform", type=float, default=BEAM["beamDeform"], help="beamDeform, N")
    ap.add_argument("--strength", type=float, default=BEAM["beamStrength"], help="beamStrength, N")
    args = ap.parse_args()
    NX, NY, NZ = args.nodes
    SPACING, NODE_WEIGHT = args.spacing, args.node_weight
    BEAM.update(beamSpring=args.spring, beamDamp=args.damp, beamDeform=args.deform, beamStrength=args.strength)
    title = "OpenBeam truss crate" if args.name == "truss_crate" else f"OpenBeam {args.name.replace('_', ' ')}"
    text = build(args.name, title)
    Path(args.out).write_bytes(text.encode("utf-8"))
    print(f"wrote {args.out}: {NX * NY * NZ} nodes, {text.count('[\"c') - NX * NY * NZ} beams")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
