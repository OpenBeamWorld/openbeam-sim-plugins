# SPDX-License-Identifier: GPL-3.0-or-later
"""OpenBeam cable routing: a LeRobot EnvHub environment on MuJoCo and OpenBeam.

A gripper routes a 1 m Cat6 patch cable from a wall outlet through a rack
D-ring and plugs it into a patch-panel port. OpenBeam simulates the cable and
its RJ45 plug through MuJoCo's "openbeam.softbody" engine plugin: copper
conductors that yield and take a permanent set, an elastic jacket, torsion,
the plug at its real 8 g, and contact with the desk, ring and panel. The ring
and the port move from episode to episode.

An episode succeeds when the plug latches in the port with the cable through
the ring and no conductor damaged. The latch is a rule, as an RJ45's is a
catch: once the gripper has pushed the plug at least 12 mm into the port,
within 2 mm of its axis and square to the panel, the plug is held by the
panel. How the cable bends on the way (its tightest bend, the time it spends
below its 25 mm rated minimum bend radius, the set its copper takes) is
reported and costs reward; damage fails the episode.

    from lerobot.envs.configs import HubEnvConfig
    envs = HubEnvConfig(hub_path="lerobot/openbeam_cable", trust_remote_code=True).create_envs(n_envs=1)

Observations ("obs_type"):
  pixels_agent_pos   pixels (128x128x3 uint8, top camera) and agent_pos (gripper x, y, z, yaw, grip)
  state              agent_pos and environment_state (48 values: plug centre and axis, ring centre,
                     port mouth, 11 points along the cable, yielded and damaged conductor beams, latched)
  pixels_state       all three
Action: the gripper's target x, y, z (m), yaw (rad) and grip (> 0.5 closes it). The gripper moves
towards the target at up to 0.4 m/s and 3 rad/s; closing it takes the plug if the plug is within 15 mm.

The plugin library is found at $OPENBEAM_MUJOCO_PLUGIN, next to this file, or
in the repository's build/install/mujoco_plugin/ (build_windows.ps1,
build_linux.sh). cat6_cable.jbeam and cat6_cable.materials must sit next to
this file.
"""

from __future__ import annotations

import ctypes
import math
import os
import sys
from functools import partial
from pathlib import Path

import gymnasium as gym
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
FPS = 20
TIMESTEP = 0.002
SUBSTEPS = round(1.0 / (FPS * TIMESTEP))
OPENBEAM_STEP = 0.000033          # s; cat6_cable's largest stable step is 56 us
SECTIONS, PER_SECTION = 51, 4     # cat6_cable.jbeam: 51 cross-sections of 4 nodes, then the plug's 8 nodes
PLUG = slice(SECTIONS * PER_SECTION, SECTIONS * PER_SECTION + 8)
OUTLET = (-0.30, 0.25)            # where the cable starts, held by the outlet
PLUG_START = (0.715, 0.25, 0.0082)
RING_X = (0.18, 0.30)             # the ring's range, and the port's from the ring: every route through the
PORT_FROM_RING = (-0.06, 0.08)    # ring to the port leaves at least 6 cm of the 1 m cable as slack
RING_Y, RING_Z, RING_HALF_OPENING = 0.0, 0.12, 0.034
PANEL_Y, PORT_Z = -0.262, 0.12    # the panel's centre plane; its face is at y = -0.25
PANEL_FACE_Y = PANEL_Y + 0.012
RATED_MIN_BEND_RADIUS = 0.025     # m: Cat6, four diameters (TIA-568)
MAX_SPEED, MAX_YAW_RATE = 0.4, 3.0
GRIP_REACH = 0.015
LATCH_DEPTH, LATCH_OFF_AXIS, LATCH_SQUARE = 0.012, 0.002, 0.98
LOW = np.array([-0.3, -0.45, 0.0, -math.pi, 0.0], np.float32)
HIGH = np.array([0.8, 0.55, 0.3, math.pi, 1.0], np.float32)
PLUGIN_NAME = {"win32": "openbeam_mujoco.dll", "darwin": "libopenbeam_mujoco.dylib"}.get(sys.platform,
                                                                                   "libopenbeam_mujoco.so")

LEROBOT_SIM = {
    "fps": FPS,
    "control_mode": "position",
    "state_names": ["gripper_x", "gripper_y", "gripper_z", "gripper_yaw", "grip"],
    "action_names": ["target_x", "target_y", "target_z", "target_yaw", "grip"],
}

MODEL_XML = """
<mujoco model="openbeam_cable">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <extension>
    <plugin plugin="openbeam.softbody">
      <instance name="cable">
        <config key="jbeam" value="{jbeam}"/>
        <config key="materials" value="{materials}"/>
        <config key="pos" value="{outlet_x} {outlet_y} 0.0"/>
        <config key="step" value="{ob_step}"/>
        <config key="node_radius" value="0.00118"/>
        <config key="node_damping" value="6"/>
        <config key="release" value="0.05"/>
        <config key="friction" value="0.5 0.4"/>
        <config key="colliders" value="outlet grip ring_top ring_bottom ring_left ring_right
                                       panel_upper panel_lower panel_left panel_right"/>
        <config key="attach" value="s0_*:outlet s1_*:outlet"/>
        <config key="stress_scale" value="30"/>
        <config key="strain_scale" value="0.02"/>
        <config key="beam_width" value="0.003"/>
      </instance>
    </plugin>
  </extension>
  <visual>
    <global offwidth="640" offheight="640"/>
    <quality shadowsize="2048"/>
    <headlight ambient=".35 .35 .35" diffuse=".5 .5 .5"/>
  </visual>
  <statistic center="0.05 0 0.08" extent="0.8"/>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".72 .78 .86" rgb2=".2 .24 .3" width="256" height="256"/>
    <texture name="desk" type="2d" builtin="checker" rgb1=".62 .63 .66" rgb2=".56 .57 .6" width="512" height="512"/>
    <material name="desk" texture="desk" texrepeat="12 12" reflectance=".08"/>
    <material name="metal" rgba=".25 .27 .3 1" reflectance=".2"/>
    <material name="panel" rgba=".12 .13 .15 1"/>
  </asset>
  <worldbody>
    <light pos="0.2 -0.6 1.4" dir="-0.1 0.4 -1" directional="true" castshadow="true"/>
    <geom name="desk" type="plane" size="1.5 1.5 0.05" material="desk"/>
    <geom name="outlet" type="box" pos="{outlet_box_x} {outlet_y} 0.02" size="0.03 0.03 0.02" material="metal"/>
    <body name="ring" pos="0.25 {ring_y} {ring_z}">
      <geom name="ring_top" type="box" pos="0 0 0.035" size="0.04 0.012 0.005" material="metal"/>
      <geom name="ring_bottom" type="box" pos="0 0 -0.035" size="0.04 0.012 0.005" material="metal"/>
      <geom name="ring_left" type="box" pos="-0.035 0 0" size="0.005 0.012 0.04" material="metal"/>
      <geom name="ring_right" type="box" pos="0.035 0 0" size="0.005 0.012 0.04" material="metal"/>
      <geom type="box" pos="0 0 -0.08" size="0.006 0.006 0.04" material="metal" contype="0" conaffinity="0"/>
    </body>
    <body name="panel" pos="0.30 {panel_y} {port_z}">
      <geom name="panel_upper" type="box" pos="0 0 0.0155" size="0.13 0.012 0.0065" material="panel"/>
      <geom name="panel_lower" type="box" pos="0 0 -0.0155" size="0.13 0.012 0.0065" material="panel"/>
      <geom name="panel_left" type="box" pos="-0.0715 0 0" size="0.0585 0.012 0.009" material="panel"/>
      <geom name="panel_right" type="box" pos="0.0715 0 0" size="0.0585 0.012 0.009" material="panel"/>
      <geom type="box" pos="-0.12 0 -0.071" size="0.006 0.01 0.049" material="metal" contype="0" conaffinity="0"/>
      <geom type="box" pos="0.12 0 -0.071" size="0.006 0.01 0.049" material="metal" contype="0" conaffinity="0"/>
      <geom type="box" pos="-0.035 0.0125 0" size="0.0085 0.001 0.0075" rgba=".18 .45 .85 1" contype="0" conaffinity="0"/>
      <geom type="box" pos="0.035 0.0125 0" size="0.0085 0.001 0.0075" rgba=".18 .45 .85 1" contype="0" conaffinity="0"/>
    </body>
    <body name="gripper" mocap="true" pos="{plug_x} {plug_y} 0.08">
      <geom type="box" pos="0 0 0.024" size="0.02 0.03 0.008" material="metal" contype="0" conaffinity="0"/>
      <geom type="box" pos="0 0.0095 0.004" size="0.01 0.0025 0.014" material="metal" contype="0" conaffinity="0"/>
      <geom type="box" pos="0 -0.0095 0.004" size="0.01 0.0025 0.014" material="metal" contype="0" conaffinity="0"/>
      <geom name="grip" type="box" size="0.001 0.001 0.001" rgba="0 0 0 0" contype="0" conaffinity="0"/>
    </body>
    <camera name="top" pos="0.15 -0.05 1.25" xyaxes="1 0 0 0 1 0" fovy="50"/>
    <camera name="side" pos="0.95 0.75 0.55" xyaxes="-0.693 0.720 0 -0.295 -0.284 0.912" fovy="48"/>
  </worldbody>
  <sensor>
    <plugin name="cable_state" instance="cable"/>
  </sensor>
</mujoco>
"""

_PLUGINS: dict[str, ctypes.CDLL] = {}


def plugin_path() -> Path:
    env = os.environ.get("OPENBEAM_MUJOCO_PLUGIN")
    if env:
        return Path(env)
    candidates = [HERE / PLUGIN_NAME]
    candidates += [p / "build" / "install" / "mujoco_plugin" / PLUGIN_NAME for p in HERE.parents]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"{PLUGIN_NAME} not found: build the OpenBeam sim plugins "
                            "(build_windows.ps1, build_linux.sh) or set OPENBEAM_MUJOCO_PLUGIN")


def load_plugin() -> ctypes.CDLL:
    """Registers the "openbeam.softbody" plugin with MuJoCo once per process, and binds the C calls this
    environment uses to read the cable and grip it."""
    path = str(plugin_path())
    if path not in _PLUGINS:
        mujoco.mj_loadPluginLibrary(path)
        lib = ctypes.CDLL(path)
        P, I = ctypes.c_void_p, ctypes.c_int
        for name, res, args in (
                ("openbeam_mujoco_world", P, [P, I]),
                ("openbeam_mujoco_attach", I, [P, P, I, ctypes.c_char_p, ctypes.c_char_p]),
                ("openbeam_mujoco_detach", I, [P, I, ctypes.c_char_p]),
                ("ob_world_node_count", I, [P]),
                ("ob_world_beam_count", I, [P]),
                ("ob_world_get_node_positions", None, [P, P]),
                ("ob_world_get_beam_damage", None, [P, P, P]),
                ("ob_world_find_node", I, [P, ctypes.c_char_p]),
                ("ob_world_detach_node", I, [P, I])):
            fn = getattr(lib, name)
            fn.restype, fn.argtypes = res, args
        _PLUGINS[path] = lib
    return _PLUGINS[path]


def smoothstep(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def bend_radii(points: np.ndarray) -> np.ndarray:
    """Radius of the circle through each three consecutive centreline points."""
    a, b, c = points[:-2], points[1:-1], points[2:]
    ab, bc, ca = np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1)
    area2 = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(area2 > 1e-12, ab * bc * ca / (2.0 * area2), np.inf)


class OpenBeamCableEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": FPS}
    task_description = "Route the cable through the D-ring and plug it into the open port without damaging it."

    def __init__(self, obs_type: str = "pixels_agent_pos", observation_size: int = 128,
                 render_size: int = 384, max_episode_steps: int = 200, render_mode: str = "rgb_array"):
        if obs_type not in ("pixels_agent_pos", "state", "pixels_state"):
            raise ValueError(f"unknown obs_type {obs_type!r}")
        self._lib = load_plugin()
        self.model = mujoco.MjModel.from_xml_string(MODEL_XML.format(
            timestep=TIMESTEP, ob_step=OPENBEAM_STEP, jbeam=(HERE / "cat6_cable.jbeam").as_posix(),
            materials=(HERE / "cat6_cable.materials").as_posix(), outlet_x=OUTLET[0], outlet_y=OUTLET[1],
            outlet_box_x=OUTLET[0] - 0.01, ring_y=RING_Y, ring_z=RING_Z, panel_y=PANEL_Y, port_z=PORT_Z,
            plug_x=PLUG_START[0], plug_y=PLUG_START[1]))
        self.data = mujoco.MjData(self.model)
        self.obs_type = obs_type
        self.observation_size = observation_size
        self.render_size = render_size
        self.render_mode = render_mode
        self._max_episode_steps = max_episode_steps
        self._sensor = self.model.sensor("cable_state").adr[0]
        self._ring_body = self.model.body("ring").id
        self._panel_body = self.model.body("panel").id
        self._world = None
        self._renderers: dict[int, mujoco.Renderer] = {}
        self._pose = np.zeros(5)             # gripper x, y, z, yaw, grip
        self._gripping = self._latched = False
        self._steps = 0
        self._tightest, self._below_rating = math.inf, 0.0
        self._expert_path: list = []

        spaces = {"agent_pos": gym.spaces.Box(LOW, HIGH, (5,), np.float32)}
        if obs_type != "pixels_agent_pos":
            spaces["environment_state"] = gym.spaces.Box(-2.0, 2.0e3, (48,), np.float32)
        if obs_type != "state":
            spaces["pixels"] = gym.spaces.Box(0, 255, (observation_size, observation_size, 3), np.uint8)
        self.observation_space = gym.spaces.Dict(spaces)
        self.action_space = gym.spaces.Box(LOW, HIGH, (5,), np.float32)

    # -- the cable -----------------------------------------------------------
    def _nodes(self) -> np.ndarray:
        world = self._lib.openbeam_mujoco_world(self.data._address, 0)
        out = np.zeros((self._lib.ob_world_node_count(world), 3))
        self._lib.ob_world_get_node_positions(world, out.ctypes.data)
        return out

    def _damage(self) -> tuple[np.ndarray, np.ndarray]:
        world = self._lib.openbeam_mujoco_world(self.data._address, 0)
        n = self._lib.ob_world_beam_count(world)
        plastic, damage = np.zeros(n, np.float32), np.zeros(n, np.float32)
        self._lib.ob_world_get_beam_damage(world, plastic.ctypes.data, damage.ctypes.data)
        return plastic, damage

    def ring_centre(self) -> np.ndarray:
        return self.data.xpos[self._ring_body].copy()

    def port_mouth(self) -> np.ndarray:
        """The centre of the open port in the panel's face."""
        p = self.data.xpos[self._panel_body].copy()
        p[1] = PANEL_FACE_Y
        return p

    def cable_state(self) -> dict:
        nodes = self._nodes()
        centreline = nodes[: SECTIONS * PER_SECTION].reshape(SECTIONS, PER_SECTION, 3).mean(axis=1)
        plug = nodes[PLUG]
        axis = plug[4:].mean(axis=0) - plug[:4].mean(axis=0)
        axis /= max(np.linalg.norm(axis), 1e-12)
        plastic, damage = self._damage()
        s = self.data.sensordata[self._sensor: self._sensor + 2]
        mouth = self.port_mouth()
        front = plug[4:].mean(axis=0)
        return {"centreline": centreline, "plug_centre": plug.mean(axis=0), "plug_axis": axis,
                "plug_off_axis": float(np.linalg.norm(front[[0, 2]] - mouth[[0, 2]])),
                "plug_insertion": float(mouth[1] - front[1]), "plug_square": float(-axis[1]),
                "threads_ring": self._threads_ring(centreline),
                "bend_radius": float(bend_radii(centreline).min()),
                "beams_yielded": int(s[1]), "beams_torn": int(s[0]),
                "beams_damaged": int((damage > 0).sum()), "peak_plastic_strain": float(np.abs(plastic).max())}

    def _threads_ring(self, points: np.ndarray) -> bool:
        """Whether the centreline crosses the ring's plane inside its opening."""
        centre = self.ring_centre()
        y = points[:, 1] - centre[1]
        for i in np.nonzero(np.sign(y[:-1]) != np.sign(y[1:]))[0]:
            u = y[i] / (y[i] - y[i + 1])
            p = points[i] + u * (points[i + 1] - points[i])
            if abs(p[0] - centre[0]) < RING_HALF_OPENING and abs(p[2] - centre[2]) < RING_HALF_OPENING:
                return True
        return False

    # -- observations --------------------------------------------------------
    def _obs(self, c: dict) -> dict:
        obs = {"agent_pos": self._pose.astype(np.float32)}
        if self.obs_type != "pixels_agent_pos":
            line = c["centreline"][np.linspace(0, SECTIONS - 1, 11).round().astype(int)]
            obs["environment_state"] = np.concatenate([
                c["plug_centre"], c["plug_axis"], self.ring_centre(), self.port_mouth(), line.ravel(),
                [c["beams_yielded"], c["beams_damaged"], float(self._latched)]]).astype(np.float32)
        if self.obs_type != "state":
            obs["pixels"] = self._render(self.observation_size, "top")
        return obs

    def _info(self, c: dict) -> dict:
        success = self._latched and c["threads_ring"] and c["beams_damaged"] == 0
        return {"is_success": success, "latched": self._latched, "threads_ring": c["threads_ring"],
                "plug_off_axis_mm": 1000 * c["plug_off_axis"], "plug_insertion_mm": 1000 * c["plug_insertion"],
                "tightest_bend_radius_mm": 1000 * self._tightest, "time_below_rating_s": self._below_rating,
                "beams_yielded": c["beams_yielded"], "beams_damaged": c["beams_damaged"],
                "beams_torn": c["beams_torn"], "peak_plastic_strain": c["peak_plastic_strain"]}

    # -- gym API ---------------------------------------------------------------
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)   # the plugin restores the cable's rest pose and grips
        rng = self.np_random
        ring_x = rng.uniform(*RING_X)
        self.model.body_pos[self._ring_body] = (ring_x, RING_Y, RING_Z)
        self.model.body_pos[self._panel_body] = (ring_x + rng.uniform(*PORT_FROM_RING), PANEL_Y, PORT_Z)
        self._pose = np.array([PLUG_START[0], PLUG_START[1], 0.08, 0.0, 0.0])
        self._set_mocap()
        self._gripping = self._latched = False
        self._steps = 0
        self._tightest, self._below_rating = math.inf, 0.0
        mujoco.mj_forward(self.model, self.data)
        self._expert_path = self._plan()
        c = self.cable_state()
        return self._obs(c), self._info(c)

    def step(self, action):
        target = np.clip(np.asarray(action, np.float64), LOW, HIGH)
        grip = target[4] > 0.5
        if grip and not self._gripping and not self._latched:
            self._take_plug()
        elif not grip and self._gripping:
            self._lib.openbeam_mujoco_detach(self.data._address, 0, b"rj_*")
            self._gripping = False
        self._pose[4] = float(grip)
        dt = self.model.opt.timestep
        for _ in range(SUBSTEPS):
            move = target[:3] - self._pose[:3]
            dist = float(np.linalg.norm(move))
            if dist > MAX_SPEED * dt:
                move *= MAX_SPEED * dt / dist
            self._pose[:3] += move
            turn = math.remainder(target[3] - self._pose[3], 2 * math.pi)
            self._pose[3] += max(-MAX_YAW_RATE * dt, min(MAX_YAW_RATE * dt, turn))
            self._set_mocap()
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        c = self.cable_state()
        self._tightest = min(self._tightest, c["bend_radius"])
        below = c["bend_radius"] < RATED_MIN_BEND_RADIUS
        self._below_rating += SUBSTEPS * dt if below else 0.0
        if self._gripping and not self._latched and self._can_latch(c):
            # The catch clicks: the panel holds the plug where it is, and the gripper's hold ends at once.
            world = self._lib.openbeam_mujoco_world(self.data._address, 0)
            for k in range(8):
                self._lib.ob_world_detach_node(world, self._lib.ob_world_find_node(world, f"rj_{k}".encode()))
            self._lib.openbeam_mujoco_attach(self.model._address, self.data._address, 0, b"rj_*", b"panel_lower")
            self._gripping, self._latched = False, True
        info = self._info(c)
        distance = float(np.linalg.norm(c["plug_centre"] - (self.port_mouth() - [0, 0.015, 0])))
        reward = -distance - 0.5 * below - 0.002 * c["beams_yielded"] - 1.0 * (c["beams_damaged"] > 0)
        terminated = bool(info["is_success"] or c["beams_damaged"] > 0)
        truncated = self._steps >= self._max_episode_steps
        return self._obs(c), reward, terminated, truncated, info

    def render(self):
        return self._render(self.render_size, "side")

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    # -- internals -------------------------------------------------------------
    def _set_mocap(self) -> None:
        x, y, z, yaw, _ = self._pose
        self.data.mocap_pos[0] = (x, y, z)
        self.data.mocap_quat[0] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    def _take_plug(self) -> None:
        plug = self._nodes()[PLUG].mean(axis=0)
        if float(np.linalg.norm(plug - self._pose[:3])) > GRIP_REACH:
            return
        mujoco.mj_forward(self.model, self.data)      # the grip where the gripper is now
        self._gripping = self._lib.openbeam_mujoco_attach(
            self.model._address, self.data._address, 0, b"rj_*", b"grip") == 8

    def _can_latch(self, c: dict) -> bool:
        return (c["plug_insertion"] >= LATCH_DEPTH and c["plug_off_axis"] <= LATCH_OFF_AXIS
                and c["plug_square"] >= LATCH_SQUARE)

    def _render(self, size: int, camera: str) -> np.ndarray:
        renderer = self._renderers.get(size)
        if renderer is None:
            renderer = self._renderers[size] = mujoco.Renderer(self.model, size, size)
        renderer.update_scene(self.data, camera=camera)
        return renderer.render()

    # -- a scripted expert, for tests and demonstrations ------------------------
    def _plan(self) -> list:
        """Waypoints (time s, x, y, z, yaw, grip) for this episode's ring and port: take the plug, slide it
        back and aside to give the cable slack, lift it, carry it through the ring along -y while turning it
        to face the port, on until the cable behind runs straight through, then to the port and in."""
        x0, y0, z0 = PLUG_START
        ring, mouth = self.ring_centre(), self.port_mouth()
        px, pz = mouth[0], mouth[2]
        y_in = mouth[1] - 0.003 - 0.015             # the plug's centre with its front 18 mm in
        return [
            (0.0, x0, y0, 0.08, 0.0, 0.0), (0.6, x0, y0, z0, 0.0, 0.0), (0.7, x0, y0, z0, 0.0, 1.0),
            (1.5, x0 - 0.10, y0 - 0.06, z0 + 0.003, 0.0, 1.0), (2.2, x0 - 0.10, y0 - 0.06, 0.14, 0.0, 1.0),
            (3.4, ring[0], 0.12, ring[2], -math.pi / 2, 1.0), (4.4, ring[0], -0.12, ring[2], -math.pi / 2, 1.0),
            (5.2, px, -0.20, pz, -math.pi / 2, 1.0), (6.2, px, y_in, pz, -math.pi / 2, 1.0),
            (6.6, px, y_in, pz, -math.pi / 2, 1.0)]

    def expert_action(self) -> np.ndarray:
        """The planned waypoints, smoothly interpolated at this step's end time. On the way into the port
        it corrects for where the plug's front actually is: the gripper took the plug as it lay."""
        t = (self._steps + 1) / FPS
        path = self._expert_path
        out = np.array(path[-1][1:])
        for (t0, *a), (t1, *b) in zip(path, path[1:]):
            if t <= t1:
                u = smoothstep((t - t0) / (t1 - t0))
                out = np.array(a) + u * (np.array(b) - np.array(a))
                out[4] = b[4] if b[4] > a[4] else a[4]  # close at the waypoint that closes, hold till it opens
                break
        if self._gripping and t > path[-3][0]:
            front = self._nodes()[PLUG][4:].mean(axis=0)
            out[[0, 2]] += (self.port_mouth() - front)[[0, 2]]
        return out.astype(np.float32)


def make_env(n_envs: int = 1, use_async_envs: bool = False, **kwargs):
    """EnvHub entry point: n_envs copies of the cable-routing task; kwargs go to OpenBeamCableEnv."""
    env_cls = gym.vector.AsyncVectorEnv if use_async_envs else gym.vector.SyncVectorEnv
    return env_cls([partial(OpenBeamCableEnv, **kwargs) for _ in range(n_envs)])
