# Licensing

Copyright © 2026 OpenBeam and contributors.

| What | Licence |
| --- | --- |
| Code written for this repository: everything outside `third_party/`, apart from the assets below | [GPL-3.0-or-later](COPYING) |
| `third_party/openbeam/`: OpenBeam's and Rigs of Rods' law sources, vendored | GPL-3.0, version 3 only, as each file's header states |
| The built libraries, `openbeam_core` and `openbeam_mujoco`, which combine both | GPL-3.0, version 3 only, because of the vendored files |
| The JBeam structures `assets/truss_crate.jbeam` and `lerobot/openbeam_push/soft_box.jbeam`, and the images in `docs/images/` | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) |
| The OpenBeam name and logos | Not licensed: see [Trademarks](#trademarks) |

## In practice

This is a summary, not legal advice; the licence texts govern.

- You may use, study, run and change the code, privately or commercially, at
  no charge.
- If you distribute it, changed or unchanged, or a product that includes it,
  you must make the corresponding source available under the GPL too. Nobody
  can ship a closed-source fork of the physics.
- You may reuse the JBeam structures and images anywhere, including commercially,
  if you credit them. For example: *"OpenBeam sim plugins, © OpenBeam, CC BY
  4.0"*, with a link to this repository.

## Trademarks

These licences cover code and assets only. As GPL-3.0 section 7(e) allows,
they grant no rights in the OpenBeam name or logos. You may say truthfully that
your work is based on, uses or is compatible with OpenBeam. Do not name a fork,
product or service "OpenBeam", and do not suggest that OpenBeam made or endorses
it, without OpenBeam's written permission.

## Other software

This repository includes none of the following; each comes separately under
its own terms:

- MuJoCo: Apache-2.0, downloaded by `tools/fetch_mujoco.py`.
- LeRobot: Apache-2.0, installed with pip.
- NVIDIA Isaac Sim: NVIDIA's licence, which you accept when you install it.

Whether a binary bundle of the Isaac Sim extension and the GPL core library may
be distributed for Isaac Sim is not settled here.

## Contributions

Contributors sign the [OpenBeam Contributor License Agreement](CLA.md); see
[CONTRIBUTING.md](CONTRIBUTING.md).
