# OpenBeam Cloud MCP: moving robots, soft bodies and trajectories

OpenBeam Cloud runs the full OpenBeam game in a browser session, and its
account MCP server (`https://cloud.openbeam.world/mcp`) lets an agent or a
script drive it. Five of its tools connect a cloud game to the hosts in this
repository:

| Tool | Direction | What it moves |
| --- | --- | --- |
| `cloud_softbody_export` | cloud → MuJoCo, Isaac Sim, LeRobot | One actor's node-beam soft body, as JBeam text that `openbeam_core` loads |
| `cloud_robot_export` | cloud → MuJoCo, Isaac Sim | A robot's rigid-body model, as MJCF or URDF |
| `cloud_softbody_import` | this repository → cloud | JBeam text, for example a cable from `assets/make_cable.py` |
| `cloud_robot_import` | MuJoCo, Isaac Sim → cloud | A URDF or MJCF robot, rebuilt as an OpenBeam node-beam robot |
| `cloud_robot_replay` | LeRobot, MuJoCo, Isaac Sim → cloud | A joint trajectory, such as a LeRobot episode's actions, replayed under stepped physics |

The account endpoint lists them in `tools/list` once your account's Session
API has them; check there before relying on one. Their reference, with every
argument and error, is the cloud's MCP tools page
(<https://openbeam.world/docs/connect-openbeam-to-codex-or-claude/>).
Recordings export as LeRobot datasets through `cloud_recording_export_lerobot`.

## Connecting

- **An MCP client** (Claude, Codex and other clients): add
  `https://cloud.openbeam.world/mcp` as a remote MCP server and approve
  OpenBeam's sign-in page. The tools then appear by name.
- **A script**: [`tools/openbeam_cloud.py`](../tools/openbeam_cloud.py) posts one
  tool call with an access token from `OPENBEAM_MCP_ACCESS_TOKEN`. It has no
  dependencies. For OAuth sign-in, token refresh and training sessions from
  Python, use [openbeam-gym](https://github.com/OpenBeam-OSS/openbeam-gym)'s
  `CloudAccount`. Its `openbeam_gym.interop` module wraps these five tools.

Starting a cloud game uses your account's play time:
`cloud_session_start` with `{"kind": "training"}` starts one without a player
tab, as `cloud_robot_replay` needs. The export tools work in any session.

```bash
export OPENBEAM_MCP_ACCESS_TOKEN=...
python tools/openbeam_cloud.py cloud_session_list
```

## Frames and units

- Metres, kilograms, radians and seconds throughout.
- **JBeam** (`cloud_softbody_export`, `cloud_softbody_import`, `openbeam_core`):
  vehicle space, +Z up. The game's world is +Y up; the export maps it as
  JBeam (x, y, z) = game (z, x, y), the inverse of the game's own JBeam
  importer. It centres the body's footprint on the origin and puts its lowest
  node at z 0 (`offset_jbeam_m`). `openbeam_core` takes JBeam coordinates as
  the host's, so MuJoCo and Isaac Sim (both +Z up) need no further turn: place
  the body with the plugin's `pos` and `quat` (MuJoCo) or the `SoftBody` pose
  (Isaac Sim).
- **MJCF and URDF** (`cloud_robot_export`): the robot's base frame, turned so
  the world's up is +Z. `export_from_base` is that rotation.

## MuJoCo

**A soft body from a cloud game.** Export the actor, then name the JBeam in
the plugin instance:

```bash
python tools/openbeam_cloud.py cloud_softbody_export '{"actor": 3, "state": "live"}' --out from_cloud.jbeam
```

```xml
<instance name="body">
  <config key="jbeam" value="from_cloud.jbeam"/>
  <config key="step" value="0.0005"/>     <!-- the game's own step -->
  <config key="colliders" value="pusher floor"/>
</instance>
```

What the export carries:
- **Shape:** `state: "live"` keeps the shape the body has in the game now,
  permanent set included, as its new rest shape. `reference` exports the
  spawn shape.
- **Nodes:** each keeps its `nodeWeight`, `frictionCoef`, fixed flag and
  ground contact.
- **Beams:** each keeps the game's `beamSpring`, `beamDamp`, `beamDeform` and
  `beamStrength`.
- **Collision triangles** are kept.

What it drops: broken, disabled and virtual beams, and beams to another
actor. Hydros, shocks, ropes and supports become plain beams. The answer's
`warnings` say when any of this happened.

This repository's Cat6 cable and soft box, sent through the exporter in the
form the game reports them, load back into `openbeam_core` with identical
nodes, beams and rest lengths. They also move identically, step for step,
for 1 s on a ground plane.

**A robot from a cloud game.** `cloud_robot_export` with `format: "mjcf"`
writes bodies on the game's joint axes, at the zero configuration of the
game's own kinematic model. The same joint values give the same pose in both.
Each link's mass, centre of mass and inertia tensor are the exact rigid
equivalent of its soft-body node masses. Collision shapes are the convex
hulls of the link's nodes, an approximation. Pass `actuator_kp` for position
servos: the game reports no gains of its own.

```bash
python tools/openbeam_cloud.py cloud_robot_export '{"format": "mjcf", "actuator_kp": 20}' --out arm.xml
```

**A MuJoCo robot into the cloud.** `cloud_robot_import` rebuilds an MJCF with
OpenBeam's own robot importer:
- each body becomes a stiff node cage with the body's mass and centre of mass;
- each hinge becomes a driven hinge.

`install: true` puts it in your game and spawns it. Send meshes and
`<include>`d files at the paths the model uses; MuJoCo resolves those from the
main file's folder.

```bash
python tools/openbeam_cloud.py cloud_robot_import '{"format": "mjcf", "install": true}' \
    --text document=arm/arm.xml --attach assets/link1.stl=arm/assets/link1.stl
```

## Isaac Sim

**A soft body.** The same JBeam drives the extension's `SoftBody`:

```python
cable = SoftBody("from_cloud.jbeam", step=0.0005, kinematic_colliders=["/World/Gripper/Grip"])
```

**A robot.** Export URDF and open it with Isaac Sim's URDF importer (File >
Import, or `isaacsim.asset.importer.urdf`). The game reports no effort or
velocity limits, so pass yours: without them the file says 0 and the answer
warns.

```bash
python tools/openbeam_cloud.py cloud_robot_export '{"format": "urdf", "effort": 3, "velocity": 2}' --out arm.urdf
```

**An Isaac Sim robot into the cloud.** `cloud_robot_import` takes URDF; for
a USD asset, export URDF from Isaac Sim first. A `package://NAME/...` mesh
path is sent as `NAME/...`:

```bash
python tools/openbeam_cloud.py cloud_robot_import '{"format": "urdf", "install": true}' \
    --text document=franka/urdf/panda.urdf \
    --attach franka_description/meshes/link0.stl=franka/meshes/link0.stl
```

## LeRobot

- **Datasets from the cloud.** `cloud_recording_export_lerobot` turns stored
  cloud recordings into a LeRobotDataset v3.0 zip, with joint positions as
  `observation.state` and commanded joint targets as `action`.
- **Episodes into the cloud.** `cloud_robot_replay` plays a joint trajectory on
  a training session's robot and reports each joint's RMS, maximum and final
  tracking error, plus the broken beams before and after. That checks whether
  a demonstration recorded in MuJoCo, Isaac Sim or on hardware holds up on
  OpenBeam's soft-body robot. Give the dataset's `action` rows as `frames`,
  its `fps` and the joint names as `joints`, at most 600 frames per call.
  `openbeam_gym.interop.replay_lerobot_episode` reads a local dataset and
  sends it in parts. Check units first: LeRobot's real SO-101 datasets record
  degrees (pass `"units": "degrees"`) and a 0-100 gripper.

```json
{"name": "cloud_robot_replay", "arguments": {"session_id": "trn_...", "fps": 30, "units": "degrees",
  "joints": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
  "frames": [[0.0, -12.5, 30.1, 4.0, 0.0], [0.4, -12.1, 29.8, 4.2, 0.0]]}}
```

## A cable, both ways

A cable can make a round trip from this repository to the game and back:

1. Make it here and import it:

   ```bash
   python assets/make_cable.py --out patch.jbeam --length 1.0 --rj45
   python tools/openbeam_cloud.py cloud_softbody_import '{"install": true}' --text jbeam=patch.jbeam
   ```

2. Handle it in the game.
3. Export it with `"state": "live"` and load the result back in MuJoCo or
   Isaac Sim.

Two things don't carry across. The game reads only the JBeam: a
`.materials` file's elastoplastic copper and twist elements do not apply
there, and its own beam law takes over. On the way back, the exported beams
carry the game's spring, damping, deform and strength, and the plugins read
them as JBeam defines them.

Don't pair the original `.materials` file with an exported JBeam. It
addresses beams by their index, but the export lists beams in the game's
order and leaves out broken ones, so the indices need not line up.
