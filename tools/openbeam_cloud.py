#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Call an OpenBeam Cloud MCP account tool from the command line.

Moves robots, soft bodies and trajectories between an OpenBeam cloud game and
the hosts in this repository. Standard library only. See docs/CLOUD_MCP.md.

  export OPENBEAM_MCP_ACCESS_TOKEN=...      # an OAuth access token for https://cloud.openbeam.world/mcp
  python tools/openbeam_cloud.py cloud_session_list
  python tools/openbeam_cloud.py cloud_softbody_export '{"actor": 3}' --out cable.jbeam
  python tools/openbeam_cloud.py cloud_robot_export '{"format": "mjcf", "actuator_kp": 20}' --out arm.xml
  python tools/openbeam_cloud.py cloud_robot_import '{"format": "urdf", "install": true}' \
      --text document=arm.urdf --attach meshes/base.stl=arm/meshes/base.stl
  python tools/openbeam_cloud.py cloud_softbody_import '{"install": true}' --text jbeam=my_cable.jbeam

--out writes the answer's document (export_robot) or jbeam (export_softbody)
field to a file and prints the rest; without it the whole answer is printed.
"""

from __future__ import annotations

import argparse
import base64
import json
import hashlib
import os
import re
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_URL = "https://cloud.openbeam.world/mcp"


def download_operator_scene(host: str, digest: str, destination: Path, ssh_config: Path | None = None) -> None:
    """Fetch an immutable private scene using the operator's SSH identity."""
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.@-]*", host):
        raise ValueError("expected an SSH host or configured host alias")
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("expected the lowercase SHA-256 from the export receipt")
    destination = Path(destination)
    if destination.exists():
        raise FileExistsError(destination)
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        staged = Path(tmp) / "scene.zip"
        options = ["-F", str(ssh_config)] if ssh_config is not None else []
        with staged.open("xb") as stream:
            subprocess.run(["ssh", *options, "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
                            host, f"cat .local/share/openbeam/exports/{digest}.zip"],
                           stdout=stream, check=True)
        actual = hashlib.sha256()
        with staged.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                actual.update(chunk)
        if actual.hexdigest() != digest:
            raise ValueError("scene archive SHA-256 differs from the export receipt")
        os.chmod(staged, 0o600)
        if os.name == "nt":
            os.rename(staged, destination)  # Windows refuses an occupied destination.
        else:
            os.link(staged, destination)


def call(url: str, token: str, tool: str, arguments: dict, timeout: float = 360.0) -> dict:
    body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                       "params": {"name": tool, "arguments": arguments}}).encode()
    request = urllib.request.Request(url, data=body, method="POST", headers={
        "Content-Type": "application/json", "Accept": "application/json, text/event-stream",
        "Authorization": f"Bearer {token}", "MCP-Protocol-Version": "2025-06-18",
        "User-Agent": "openbeam-sim-plugins"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            text = response.read().decode("utf-8")
    except urllib.error.HTTPError as error:
        raise SystemExit(f"{tool}: HTTP {error.code}: {error.read().decode('utf-8', 'replace')[:500]}") from None
    if text.lstrip().startswith(("event:", "data:")):  # an event stream: the JSON-RPC message is a data line
        text = next(line[5:].strip() for line in text.splitlines() if line.startswith("data:"))
    message = json.loads(text)
    if "error" in message:
        raise SystemExit(f"{tool}: {message['error'].get('message')}")
    result = message["result"]
    data = result.get("structuredContent")
    if data is None:
        data = json.loads(result["content"][0]["text"])
    if result.get("isError"):
        raise SystemExit(f"{tool}: {data.get('error')}: {data.get('message')}")
    return data


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("tool", help="an account tool name, e.g. cloud_robot_export")
    parser.add_argument("arguments", nargs="?", default="{}", help="the tool's arguments as a JSON object")
    parser.add_argument("--text", action="append", default=[], metavar="KEY=FILE",
                        help="set argument KEY to FILE's text (document=arm.urdf, jbeam=cable.jbeam)")
    parser.add_argument("--attach", action="append", default=[], metavar="PATH=FILE",
                        help="add FILE to the files argument at PATH, base64 (meshes and MJCF includes)")
    parser.add_argument("--out", type=Path, help="write the document or jbeam field to this file")
    parser.add_argument("--url", default=os.environ.get("OPENBEAM_MCP_URL", DEFAULT_URL))
    parser.add_argument("--operator-host", help="SSH host alias for private operator scene downloads")
    parser.add_argument("--sha256", help="scene archive digest from the private export receipt")
    parser.add_argument("--ssh-config", type=Path, help="optional SSH config for a private operator connection")
    args = parser.parse_args(argv)
    if args.tool == "scene-download":
        if not args.operator_host or not args.sha256 or not args.out:
            parser.error("scene-download requires --operator-host, --sha256 and --out")
        download_operator_scene(args.operator_host, args.sha256, args.out, args.ssh_config)
        print(f"Downloaded private scene to {args.out}")
        return 0
    token = os.environ.get("OPENBEAM_MCP_ACCESS_TOKEN")
    if not token:
        parser.error("set OPENBEAM_MCP_ACCESS_TOKEN to an access token for the account endpoint")
    arguments = json.loads(args.arguments)
    for item in args.text:
        key, _, path = item.partition("=")
        arguments[key] = Path(path).read_text(encoding="utf-8")
    for item in args.attach:
        path, _, local = item.partition("=")
        arguments.setdefault("files", []).append(
            {"path": path, "content_base64": base64.b64encode(Path(local).read_bytes()).decode("ascii")})
    data = call(args.url, token, args.tool, arguments)
    if args.out is not None:
        field = next((k for k in ("document", "jbeam") if k in data), None)
        if field is None:
            raise SystemExit(f"{args.tool}'s answer has no document or jbeam field")
        args.out.write_text(data.pop(field), encoding="utf-8")
        print(f"wrote {args.out}", file=sys.stderr)
    json.dump(data, sys.stdout, indent=2)
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
