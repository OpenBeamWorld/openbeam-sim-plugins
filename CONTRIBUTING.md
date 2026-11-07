# Contributing

Thanks for helping. Before your first pull request is merged, sign the
[OpenBeam Contributor License Agreement](CLA.md) by posting the comment it
describes in that pull request.

## Before you open a pull request

- Build with `build_windows.ps1` or `build_linux.sh`. That also runs the core's
  tests.
- Run the Python suites that cover what you changed:
  `python -m pytest mujoco/tests isaacsim/tests lerobot/tests -q`. CI runs
  them on Windows and Linux.
- If you changed the Isaac Sim extension, also run
  `isaacsim/tests/isaac_coupling_check.py` inside Isaac Sim. CI cannot run it,
  so say in the pull request which app you ran it in (see the README).
- Start each new source file with `SPDX-License-Identifier: GPL-3.0-or-later`
  in a comment.
- Use LF line endings. `.gitattributes` enforces them.

## The vendored OpenBeam sources

Do not edit `third_party/openbeam/` by hand: it is a copy of OpenBeam's own
sources. A fix to a physics law belongs in OpenBeam itself. The copy is then
refreshed with `tools/vendor_openbeam.py --from <openbeam checkout>`, and CI
checks it against `SOURCES.json`.
