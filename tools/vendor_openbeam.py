#!/usr/bin/env python3
"""Keeps third_party/openbeam/ in step with OpenBeam's sources.

The soft-body core (openbeam_core/) compiles 22 files of OpenBeam and of its
BOARS engine as they are: the JBeam parser, part resolver and structural tier,
the beam laws and the calibrated beam material, and the two contact-law
headers. Those repositories are not public, so this one carries copies of the
files, as their GPL-3.0 licence allows, with their headers intact. Each copy is
the file as committed at the checkout's HEAD.
third_party/openbeam/SOURCES.json records the commit each file was last
changed in and its SHA-256.

  python tools/vendor_openbeam.py --from <openbeam checkout>   # copy them again
  python tools/vendor_openbeam.py --check                      # the copies match SOURCES.json

The checkout must have its engine/ submodule (BOARS) checked out.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "third_party" / "openbeam"
MANIFEST = VENDOR / "SOURCES.json"

# Paths as they are in an openbeam checkout; engine/ is the BOARS submodule.
FILES = [
    "engine/source/main/physics/BeamAxialKinematics.cpp",
    "engine/source/main/physics/BeamAxialKinematics.h",
    "engine/source/main/physics/BeamAxialResponse.h",
    "engine/source/main/physics/CalibratedBeamJBeamDerivation.h",
    "engine/source/main/physics/CalibratedBeamMaterial.h",
    "engine/source/main/physics/CalibratedBeamMaterialAdapter.h",
    "engine/source/main/physics/CalibratedBeamProductionStep.cpp",
    "engine/source/main/physics/CalibratedBeamProductionStep.h",
    "engine/source/main/physics/FixedPhysicsStep.h",
    "engine/source/main/physics/ProgressiveBeamDamping.h",
    "engine/source/main/physics/SimConstants.h",
    "engine/source/main/resources/beamng/JBeamCoordinateTransform.h",
    "engine/source/main/resources/beamng/JBeamExpressionEvaluator.cpp",
    "engine/source/main/resources/beamng/JBeamExpressionEvaluator.h",
    "engine/source/main/resources/beamng/JBeamPartResolver.cpp",
    "engine/source/main/resources/beamng/JBeamPartResolver.h",
    "engine/source/main/resources/beamng/JBeamStructuralIR.cpp",
    "engine/source/main/resources/beamng/JBeamStructuralIR.h",
    "engine/source/main/resources/beamng/JBeamSyntax.cpp",
    "engine/source/main/resources/beamng/JBeamSyntax.h",
    "source/main/physics/ContactFrictionResponse.h",
    "source/main/physics/ContactPenetration.h",
]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(repo: Path, *args: str) -> bytes:
    return subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True).stdout


def locate(checkout: Path, rel: str) -> tuple[str, Path, str]:
    """(repository name, git repository, path in it): BOARS for engine/ paths, openbeam otherwise."""
    if rel.startswith("engine/"):
        return "OpenBeamWorld/boars", checkout / "engine", rel[len("engine/"):]
    return "OpenBeamWorld/openbeam", checkout, rel


def copy_from(checkout: Path) -> int:
    """Copies each file as committed at the checkout's HEAD, whatever its working-tree line endings."""
    files = {}
    for rel in FILES:
        repository, repo, path = locate(checkout, rel)
        try:
            data = git(repo, "show", f"HEAD:{path}")
        except subprocess.CalledProcessError:
            print(f"not in {repository} at HEAD: {rel}", file=sys.stderr)
            return 1
        commit = git(repo, "log", "-1", "--format=%H", "HEAD", "--", path).decode().strip()
        target = VENDOR / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        files[rel] = {"sha256": sha256(target), "repository": repository, "last_changed_in": commit}
    manifest = {"licence": "GPL-3.0 (see each file's header and COPYING)", "files": files}
    MANIFEST.write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(f"copied {len(files)} files into {VENDOR.relative_to(ROOT)}")
    return 0


def check() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    problems = [rel for rel in FILES if rel not in manifest["files"]]
    problems += [rel for rel in manifest["files"] if rel not in FILES]
    for rel, entry in manifest["files"].items():
        path = VENDOR / rel
        if not path.is_file() or sha256(path) != entry["sha256"]:
            problems.append(rel)
    for rel in problems:
        print(f"differs from SOURCES.json: {rel}", file=sys.stderr)
    if not problems:
        print(f"{len(FILES)} vendored files match SOURCES.json")
    return 1 if problems else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--from", dest="checkout", type=Path, help="an openbeam checkout with engine/ checked out")
    group.add_argument("--check", action="store_true")
    args = ap.parse_args()
    return check() if args.check else copy_from(args.checkout.resolve())


if __name__ == "__main__":
    raise SystemExit(main())
