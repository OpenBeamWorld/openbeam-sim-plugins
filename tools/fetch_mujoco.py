#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Downloads the MuJoCo release the plugin builds against and checks its SHA-256.

  python tools/fetch_mujoco.py            # into third_party/mujoco-4.0.0
  python tools/fetch_mujoco.py --dest DIR

The plugin needs the release's headers and import library (include/, lib/).
The Python package that loads the plugin must be the same version:
pip install mujoco==4.0.0.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import platform
import shutil
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path

VERSION = "4.0.0"
ROOT = Path(__file__).resolve().parents[1]
# SHA-256 digests published with the release on GitHub.
RELEASES = {
    ("Windows", "AMD64"): ("mujoco-4.0.0-windows-x86_64.zip",
                           "217f96b6146d02e5eaf93b907bc24e2fca67d6a040d243aea6ce9de8724b365b"),
    ("Linux", "x86_64"): ("mujoco-4.0.0-linux-x86_64.tar.gz",
                          "3d5d2d96b6d286b758c75198aa7862590c960256626d3aac53ac5721b0a697f3"),
    ("Linux", "aarch64"): ("mujoco-4.0.0-linux-aarch64.tar.gz",
                           "479cc07d8a8d1cc77a64dbc477a0e3b08b2caefcee293f137a050e71717163b2"),
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dest", type=Path, default=ROOT / "third_party" / f"mujoco-{VERSION}")
    args = ap.parse_args()
    key = (platform.system(), platform.machine())
    if key not in RELEASES:
        print(f"no MuJoCo {VERSION} release pinned for {key}; unpack one yourself and pass MUJOCO_ROOT",
              file=sys.stderr)
        return 1
    name, digest = RELEASES[key]
    if (args.dest / "include" / "mujoco" / "mujoco.h").exists():
        print(f"{args.dest} already holds MuJoCo")
        return 0
    url = f"https://github.com/google-deepmind/mujoco/releases/download/{VERSION}/{name}"
    print(f"downloading {url}")
    with urllib.request.urlopen(url) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != digest:
        print(f"SHA-256 mismatch for {name}", file=sys.stderr)
        return 1
    with tempfile.TemporaryDirectory() as tmp:
        if name.endswith(".zip"):
            zipfile.ZipFile(io.BytesIO(data)).extractall(tmp)
        else:
            with tarfile.open(fileobj=io.BytesIO(data)) as archive:
                archive.extractall(tmp, filter="data")
        # The Linux archives hold one top-level mujoco-<version>/ directory; the Windows zip does not.
        top = [p for p in Path(tmp).iterdir()]
        source = top[0] if len(top) == 1 and top[0].is_dir() else Path(tmp)
        args.dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source, args.dest, dirs_exist_ok=True)
    print(f"MuJoCo {VERSION} in {args.dest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
