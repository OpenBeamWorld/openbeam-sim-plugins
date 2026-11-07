# OpenBeam soft-box push

A [LeRobot](https://github.com/huggingface/lerobot) EnvHub environment. A
pusher moves a soft box onto a goal without denting it. The box is a JBeam
structure that [OpenBeam](../../README.md) simulates through MuJoCo's
`openbeam.softbody` engine plugin. Its 254 beams flex under load, yield into
permanent dents past 120 N, and tear when overloaded. Contact with the pusher
is coupled two ways.

| | |
| --- | --- |
| Scene | A floor with a ±0.6 m workspace around the origin; MuJoCo 4.0 at a 2 ms step, OpenBeam at 0.5 ms |
| Box | `soft_box.jbeam`: a 4 × 4 × 3 lattice of nodes 10 cm apart (30 × 30 × 20 cm), 4.8 kg, centred at the origin |
| Pusher | A capsule 6 cm in radius on two slide joints. A position servo moves it (3000 N/m, at most 600 N) |
| Goal | A disc 25 to 40 cm from the box in a random direction; the pusher starts 42 cm out in another |
| Control | 20 Hz; an episode lasts at most 300 steps (15 s) |
| Action | `Box(-0.6, 0.6, (2,))`: the pusher's target x, y in metres |
| Success | The box's centroid within 5 cm of the goal, with no beam yielded or torn; the episode then terminates |
| Reward | −(distance to goal) − 0.02 × (yielded beams) − 0.1 × (torn beams) |

## Observations

`make_env(obs_type=...)`, or `make_env_kwargs={"obs_type": ...}` through
LeRobot:

| `obs_type` | Keys | LeRobot features |
| --- | --- | --- |
| `pixels_agent_pos` (default) | `pixels` (96 × 96 × 3 uint8, top camera) and `agent_pos` (pusher x, y) | `observation.image`, `observation.state` |
| `state` | `agent_pos` and `environment_state`: box centroid x, y, z; goal x, y; torn beams; yielded beams | `observation.state`, `observation.environment_state` |
| `pixels_state` | All three | All three |

Other options: `observation_size` (default 96), `render_size` (the side
camera of `render()`, default 384), and `max_episode_steps` (default 300).
`info` carries `is_success`, `distance`, `beams_deformed` and `beams_broken`.

## Use

Install `mujoco==4.0.0` and `lerobot` (`requirements.txt`). Build the plugin
(`openbeam_mujoco.dll`, or `libopenbeam_mujoco.so` on Linux) with this
repository's `build_windows.ps1` or `build_linux.sh`. `env.py` finds it at
`$OPENBEAM_MUJOCO_PLUGIN`, next to `env.py`, or in the repository's
`build/install/mujoco_plugin/`. Then, from the repository root:

```python
from lerobot.envs.configs import HubEnvConfig

envs = HubEnvConfig(hub_path="lerobot/openbeam_push", trust_remote_code=True).create_envs(n_envs=4)
env = envs["hub_env"][0]
obs, info = env.reset(seed=[0, 1, 2, 3])
```

On Windows, give `hub_path` as a relative path: LeRobot splits it at the first
`:`. Also keep `use_async_envs=False`: LeRobot 0.7.0 starts its workers with
the `forkserver` method, which Windows does not have.

`OpenBeamPushEnv.expert_action()` is a scripted expert for demonstrations. It
goes round the box and pushes it with a 60 N lead, moving slowly near the box.
It succeeds on all of the first 50 seeds without denting the box.

## Licence

`env.py` is GPL-3.0-or-later, and so is the plugin library it loads.
`soft_box.jbeam` is CC BY 4.0. See the repository's LICENSE.md.
