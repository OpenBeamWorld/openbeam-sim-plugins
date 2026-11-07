# OpenBeam sim plugins

[OpenBeam](https://github.com/OpenBeamWorld)'s soft-body physics as plugins for
three robotics and simulation hosts:

| Plugin | Host | Directory | Tests |
| --- | --- | --- | --- |
| `openbeam.softbody` engine plugin | [MuJoCo](https://mujoco.org) 4.0 | [`mujoco/`](mujoco/) | `mujoco/tests` (pytest) |
| `openbeam.isaacsim.softbody` Kit extension | [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac/sim) 6.2 | [`isaacsim/`](isaacsim/) | `isaacsim/tests` (pytest, plus checks inside Isaac Sim) |
| `openbeam_push` EnvHub environment | [LeRobot](https://github.com/huggingface/lerobot) 0.7 | [`lerobot/`](lerobot/) | `lerobot/tests` (pytest) |

All three share one C library, [`openbeam_core`](openbeam_core/), with its own
tests. The host simulates the rigid world: robots, props and the ground.
OpenBeam simulates one or more JBeam structures, node and beam networks that
bend, yield, dent and tear. Contact couples them both ways. The host's
colliders push OpenBeam's nodes, and the force the soft body puts back on each
collider is applied to the host's body over the same interval.

## What the core simulates, and what it does not

`openbeam_core` is not the whole OpenBeam simulator. It compiles OpenBeam's
own law sources (vendored in [`third_party/openbeam/`](third_party/openbeam/))
and steps them in the game's order at its fixed 0.5 ms step, in binary32 node
state:

1. **JBeam loading** uses OpenBeam's parser, part resolver and structural tier.
   Unauthored fields get OpenBeam's defaults. A `refNodes` frame is optional,
   because a structure need not be a vehicle.
2. **Beams.** Every NORMAL beam with a finite `beamDeform` uses the calibrated
   elastoplastic damage material, as OpenBeam's JBeam import does. Any other
   beam uses the classic spring and damper, with plastic rest-length
   deformation, the `deformLimitExpansion` ceiling and breaking at
   `beamStrength`. A broken beam breaks every beam in its `breakGroup`.
3. **Contact** uses OpenBeam's dry penetration and friction laws, with the
   concrete ground model, against the host's planes and its sphere, box and
   capsule colliders, in the collider's moving frame.
4. **Kick and drift**: v += F/m h, then x += v h, under gravity.

It does **not** simulate:

- wheels and tyres, hydros, shocks, or SUPPORT, BOUNDED and rope beams;
- collision triangles, so there is no self-collision and no contact between
  two soft bodies;
- terrain meshes, or ground models other than concrete;
- flexbodies, engines and drivetrains, or Lua.

It suits crates, props and deformable structures, not driving a JBeam vehicle.
Nodes are points, so a collider has to reach them: one narrower than the gaps
between nodes can pass between them. Each call to `ob_world_advance(dt)` runs
as many 0.5 ms steps as fit and carries the remainder to the next call.

## Build

| Needs | Windows | Linux |
| --- | --- | --- |
| C++17 compiler | Visual Studio 2022 or its Build Tools, with the x64 C++ tools | GCC or Clang |
| CMake | 3.24 or newer | 3.24 or newer |
| Ninja | optional | optional |
| Python | 3.12, for the scripts and tests | 3.12 |

Fetch the MuJoCo release the plugin builds against. It goes into
`third_party/mujoco-4.0.0` and is checked against its published SHA-256. Then
build:

```bash
python tools/fetch_mujoco.py
powershell -ExecutionPolicy Bypass -File build_windows.ps1
```

```bash
python3 tools/fetch_mujoco.py
./build_linux.sh
```

Each script configures `build/`, builds, installs to `build/install/` and runs
the core's tests:

| File | Windows | Linux |
| --- | --- | --- |
| The core, for the Isaac Sim extension | `build/install/bin/openbeam_core.dll` | `build/install/lib/libopenbeam_core.so` |
| The MuJoCo plugin, also used by LeRobot | `build/install/mujoco_plugin/openbeam_mujoco.dll` | `build/install/mujoco_plugin/libopenbeam_mujoco.so` |
| The C API | `build/install/include/openbeam_core.h` | the same |

Pass `-NoMujoco` (Windows) or `--no-mujoco` (Linux) to build the core alone,
or `-MujocoRoot` / `MUJOCO_ROOT=` to use another MuJoCo 4.0.0 release.

## MuJoCo

Install MuJoCo's Python package, at the same version as the release whose
headers built the plugin:

```bash
python -m venv .venv
.venv/Scripts/python -m pip install mujoco==4.0.0 pytest pillow      # Linux: .venv/bin/python
.venv/Scripts/python mujoco/demo.py                  # headless, prints a JSON summary
.venv/Scripts/python mujoco/demo.py --viewer         # MuJoCo's interactive viewer
.venv/Scripts/python -m pytest mujoco/tests isaacsim/tests -q
```

The demo swings a 300 kg MuJoCo wrecking ball into
[`truss_crate.jbeam`](assets/truss_crate.jbeam), a 100-node, 603-beam crate.
`--frames DIR` writes PNG frames instead.

To use the plugin in your own model, load it and declare an instance:

```python
mujoco.mj_loadPluginLibrary("build/install/mujoco_plugin/openbeam_mujoco.dll")
```

```xml
<extension>
  <plugin plugin="openbeam.softbody">
    <instance name="crate">
      <config key="jbeam" value="truss_crate.jbeam"/>   <!-- or an absolute path -->
      <config key="pos" value="0.45 -0.6 0.02"/>          <!-- placement of the JBeam origin -->
      <config key="quat" value="1 0 0 0"/>
      <config key="colliders" value="ball gripper_pad"/>  <!-- sphere/capsule/box geoms -->
      <config key="grounds" value=""/>                    <!-- plane geoms; empty = every plane -->
      <config key="stress_scale" value="2000"/>           <!-- N at full red in the drawing -->
    </instance>
  </plugin>
</extension>
<sensor><plugin name="crate_state" instance="crate"/></sensor>
```

The sensor gives six values: beams broken, beams deformed, the centroid
(x, y, z) and the number of OpenBeam steps run. A relative `jbeam` path is
tried from the working directory first, then from `$OPENBEAM_ASSET_DIR`. For
MuJoCo's own `simulate` viewer, copy the plugin into the release's
`bin/mujoco_plugin/`, set `OPENBEAM_ASSET_DIR` to `assets/`, and open
`mujoco/models/wrecking_ball.xml`.

Coupling rules:

- OpenBeam advances inside the passive-force stage, once per simulation time.
  It advances on the first `mj_forward` at a new `d->time`. More forwards at
  that time, or the `mj_step` that integrates it, re-apply the same wrench
  without stepping again.
- Use the `Euler`, `implicit` or `implicitfast` integrator. RK4 evaluates
  forces at intermediate times and is not supported.
- `mj_resetData` resets the soft body. `mj_copyData` deep-copies it, so
  rollouts from a copy stay independent.
- `openbeam_mujoco_world(const mjData*, int instance)` is exported for hosts
  that want the full `ob_*` C API, such as the tests reading node velocities.

## Isaac Sim

The extension `openbeam.isaacsim.softbody` lives in
[`isaacsim/exts/`](isaacsim/exts/). It is pure Python: it calls the core
library through ctypes, so it needs no build against Kit and no Isaac Sim SDK.
It finds the library at `$OPENBEAM_CORE_DLL`, then in its own `bin/`, then in
this repository's `build/install/`.

Install Isaac Sim 6.2.0 from NVIDIA's standalone package and check its
[system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html).
It asks you to accept the NVIDIA Omniverse License Agreement on first launch.
For an unattended run after you have accepted it, set `OMNI_KIT_ACCEPT_EULA=YES`
in that process's environment. `<isaac-sim>` below is the unpacked package.

```bash
<isaac-sim>/python.bat isaacsim/demo_isaacsim.py                      # headless, prints a JSON summary
<isaac-sim>/python.bat isaacsim/demo_isaacsim.py --gui                # window, real time
<isaac-sim>/python.bat isaacsim/demo_isaacsim.py --app viewer --capture crate.png
```

The demo registers `isaacsim/exts` with Kit's extension manager and enables
the extension. It builds the MuJoCo demo's scene: a PhysX ball on a revolute
pendulum and the same crate. It then steps PhysX at 200 Hz on the CPU pipeline.
`--app` picks the Kit app it runs in, and `--kit-arg=ARG` passes one more
argument to Kit, such as `--kit-arg=--/app/vulkan=false` for Direct3D 12:

| `--app` | App | Use |
| --- | --- | --- |
| `stock` (default) | Isaac's own Python app | The full Isaac Sim runtime. It needs the most memory |
| `viewer` | [`isaacsim/apps/openbeam.isaacsim.viewer.kit`](isaacsim/apps/openbeam.isaacsim.viewer.kit) | PhysX, the RTX renderer, a viewport, capture and menus, without the stock app's sensor, replicator and robotics extensions |
| `physics` | [`isaacsim/apps/openbeam.isaacsim.physics.kit`](isaacsim/apps/openbeam.isaacsim.physics.kit) | PhysX, the tensor API and the simulation manager, with no renderer or UI. Starts in about 5 s in about 1 GB |

In the full Isaac Sim app, add the extension folder and enable it. Open
**Window > OpenBeam > Wrecking Ball Demo**, then press Play:

```bash
<isaac-sim>/isaac-sim.bat --ext-folder <repo>/isaacsim/exts --enable openbeam.isaacsim.softbody
```

From Python, in any stage:

```python
from openbeam.isaacsim.softbody import SoftBody
crate = SoftBody("assets/truss_crate.jbeam", position=(0.45, -0.6, 0.02),
                 colliders=["/World/Ball"], ground_height=0.0)
```

Coupling rules:

- OpenBeam advances in `SimulationEvent.PHYSICS_PRE_STEP`, by the physics dt,
  in 0.5 ms steps (the remainder carries). The wrench goes to the collider
  bodies through `apply_forces_and_torques_at_position` for the step PhysX
  takes next.
- Colliders are rigid-body prims that are, or have as a direct child, a
  `UsdGeom` Sphere, Cube or Capsule with axis Z, centred on the body origin.
  The origin is taken as the centre of mass. The ground is a plane at
  `ground_height`.
- Stopping the timeline resets the soft body to its rest pose.
- Isaac's first physics step at Play runs before the tensor views exist. On
  that one step OpenBeam does not advance.

Memory: Isaac Sim 6.2 asks for 32 GB of RAM. The first start of an RTX app
compiles the ray-tracing pipelines in the driver. That took about 2.5 min,
with Kit at about 10.5 GB. On a 16 GB machine, close other applications first.
The driver caches the result, so later starts are quick.

## LeRobot

[`lerobot/openbeam_push/`](lerobot/openbeam_push/) is a LeRobot EnvHub
package. Its `env.py` exposes `make_env(n_envs, use_async_envs, **kwargs)` and
`LEROBOT_SIM`, the API that LeRobot loads environments through. In the task, a
position-controlled pusher (a MuJoCo capsule) pushes a 4.8 kg JBeam box
(`soft_box.jbeam`, 48 nodes, 254 beams) onto a goal. OpenBeam simulates the
box through the MuJoCo plugin. An episode succeeds when the box reaches the
goal undamaged. A steady push moves it cleanly, and a strike at speed dents it.
[The package README](lerobot/openbeam_push/README.md) gives the observations,
actions and options.

```bash
python -m venv .venv-lerobot
.venv-lerobot/Scripts/python -m pip install lerobot==0.7.0 mujoco==4.0.0 pytest
.venv-lerobot/Scripts/python -m pytest lerobot/tests -q
```

On Linux, add `--extra-index-url https://download.pytorch.org/whl/cpu` to get
a CPU-only torch instead of the CUDA build. From the repository root:

```python
from lerobot.envs.configs import HubEnvConfig
envs = HubEnvConfig(hub_path="lerobot/openbeam_push", trust_remote_code=True).create_envs(n_envs=4)
env = envs["hub_env"][0]  # observation.image (96x96 RGB) and observation.state (pusher x, y)
```

`trust_remote_code=True` lets LeRobot execute `env.py`. On Windows, with
LeRobot 0.7.0:

- Give `hub_path` as a relative path. LeRobot splits it at the first `:`, so a
  drive path such as `D:/...` is read as a Hub repository.
- `use_async_envs=True` fails. LeRobot starts its env workers with
  multiprocessing's `forkserver` method, which Windows does not have. Use
  synchronous envs.

Pixel observations need OpenGL. On a headless Linux machine, set
`MUJOCO_GL=egl` or `MUJOCO_GL=osmesa`.

## Tests

| Suite | Command | Needs | In CI |
| --- | --- | --- | --- |
| Core contract tests | `build_windows.ps1` / `build_linux.sh` (CTest) | a C++ compiler | Windows, Linux |
| MuJoCo plugin (7) | `python -m pytest mujoco/tests` | `mujoco==4.0.0` | Windows, Linux |
| Isaac extension binding (4) | `python -m pytest isaacsim/tests` | NumPy | Windows, Linux |
| Isaac extension in Isaac Sim (5) | `<isaac-sim>/python.bat isaacsim/tests/isaac_coupling_check.py --app physics` (or `viewer`, `stock`) | Isaac Sim 6.2 | no |
| Isaac extension in the GUI | see below | Isaac Sim 6.2, an RTX GPU | no |
| LeRobot environment (13) | `python -m pytest lerobot/tests` | `lerobot==0.7.0`, `mujoco==4.0.0`; OpenGL for 2 pixel tests | Windows (without the pixel tests), Linux |

What each covers:

- **Core:** JBeam loading; free fall against the closed form; rest on a plane;
  collider impulse equal to −Δ(momentum); classic break and static stretch
  m g/k; calibrated yield and fracture; `breakGroup`; clone and reset digests;
  placement.
- **MuJoCo plugin:** registration; a bad path refused; rest; two-way momentum
  with a free MuJoCo sphere (0.1 % bound); one advance per simulation time;
  `mj_copyData` and `mj_resetData`; the wrecking ball.
- **Isaac extension, outside Isaac:** the ctypes binding: load and rest, a bad
  path, momentum, reset digests.
- **Isaac extension, inside Isaac:** Kit enables it; momentum against a free
  PhysX sphere (0.1 %); the wrecking ball; stop resets; the Window menu action
  builds the scene, and the crate moves after Play.
- **LeRobot environment:** Gymnasium's env checker; rest and seeded resets; the
  scripted expert on 6 seeds, undamaged; ramming dents the box; two-way
  contact; LeRobot's `HubEnvConfig` loads and steps the package; the expert
  succeeds through LeRobot.

The GUI check runs inside the full app. It finds the menu entry, runs it,
plays 3 s, saves the viewport to `$OPENBEAM_GUI_CAPTURE` and quits:

```bash
<isaac-sim>/isaac-sim.bat --ext-folder <repo>/isaacsim/exts --enable openbeam.isaacsim.softbody --/app/file/ignoreUnsavedOnExit=true --exec <repo>/isaacsim/tests/isaac_gui_smoke.py
```

## Verification record (2026-11-07)

One Windows 11 laptop: Ryzen 7 5800H, 16 GB RAM, RTX 3070 Laptop with 8 GB,
NVIDIA driver 555.99, MSVC 14.44. MuJoCo 4.0.0, Isaac Sim 6.2.0
(`6.2.0-rc.14+release.52805.02189a84.gl`, standalone package), LeRobot 0.7.0
with torch 2.15.0 (CPU) and gymnasium 1.4.0.

| Check | Result |
| --- | --- |
| Core, MuJoCo, Isaac binding and LeRobot suites | All passed |
| `isaac_coupling_check.py`, `physics` and `viewer` (RTX) apps | 4/4 and 5/5. The menu action built the scene, and the crate moved 0.77 m |
| `isaac_gui_smoke.py` in the full GUI | Passed: the menu entry is bound to the extension's action; the crate moved 0.76 m in 3 s; the viewport was saved |
| Wrecking ball, MuJoCo vs Isaac Sim | Contact at 0.944 s vs 0.935 s. Ball 5.41 m/s at impact in both, stopped by the crate in both. 15 broken and 49 vs 48 yielded beams |
| Push task, scripted expert, 50 seeds | 50/50 successes, no beam yielded, 73 steps median (106 max) |

Not verified:

- Isaac Sim on its required driver, 595.97 or newer.
- Using the Isaac Sim GUI by hand: the menu entry was run by script.
- Training or evaluating a policy on the push task (`lerobot-train`,
  `lerobot-eval`), LeRobot's env server, and publishing to the Hub.
- The GPU (`cuda`) PhysX pipeline.
- Linux beyond CI, and macOS.

## Licence

GPL-3.0; see [`COPYING`](COPYING). The plugins link OpenBeam's GPL-3.0 law
sources, which this repository carries in `third_party/openbeam/` with their
headers intact. MuJoCo and LeRobot are Apache-2.0. Isaac Sim's Kit runtime is
proprietary. This repository distributes source only, and it does not settle
whether a binary bundle of the extension and the core library may be
distributed for Isaac Sim.

### The vendored OpenBeam sources

[`tools/vendor_openbeam.py`](tools/vendor_openbeam.py) copies the 22 files the
core compiles from an OpenBeam checkout, as committed at its HEAD.
[`SOURCES.json`](third_party/openbeam/SOURCES.json) records their SHA-256 and
the commit each was last changed in. CI runs `--check`, which fails when a copy
no longer matches that record:

```bash
python tools/vendor_openbeam.py --from <openbeam checkout>
python tools/vendor_openbeam.py --check
```
