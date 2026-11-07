"""OpenBeam soft-box push: a LeRobot EnvHub environment on MuJoCo and OpenBeam.

A pusher (a MuJoCo capsule on two position-controlled slide joints) pushes a
soft JBeam box across the floor to a goal disc. OpenBeam simulates the box
through MuJoCo's "openbeam.softbody" engine plugin: its 254 beams flex, yield
and tear, and pusher contact is coupled two ways. An episode succeeds when the
box's centroid is within 5 cm of the goal and the box is undamaged: no beam
has yielded (a permanent dent) or torn. A steady push of about 60 N moves it
cleanly; striking it at speed dents it.

    from lerobot.envs.configs import HubEnvConfig
    envs = HubEnvConfig(hub_path="lerobot/openbeam_push", trust_remote_code=True).create_envs(n_envs=1)

Observations ("obs_type"):
  pixels_agent_pos   pixels (96x96x3 uint8, top camera) and agent_pos (pusher x, y)
  state              agent_pos and environment_state (box centroid x, y, z, goal x, y,
                     torn beams, yielded beams)
  pixels_state       all three
Action: the pusher's target x, y in metres, in [-0.6, 0.6].

The plugin library is found at $OPENBEAM_MUJOCO_PLUGIN, next to this file,
or in the repository's build/install/mujoco_plugin/ (build_windows.ps1,
build_linux.sh). soft_box.jbeam must sit next to this file.
"""

from __future__ import annotations

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
WORKSPACE = 0.6            # m, the action and pusher range on each axis
BOX_HALF = 0.15            # m, half the box's footprint
PUSHER_RADIUS = 0.06       # m; contact is node against capsule, so wider than half the 0.1 m node gaps
GOAL_TOLERANCE = 0.05      # m
PUSHER_KP = 3000.0         # N/m, the position servo
PUSHER_FORCE = 600.0       # N, its limit: enough to dent the box by ramming it
PUSHER_DAMPING = 150.0     # N s/m
EXPERT_PUSH = 0.02         # m the expert leads the pusher by while pushing: kp * 2 cm = 60 N
EXPERT_STEP_NEAR_BOX = 0.05  # m, its largest lead near the box
PLUGIN_NAME = {"win32": "openbeam_mujoco.dll", "darwin": "libopenbeam_mujoco.dylib"}.get(sys.platform,
                                                                                   "libopenbeam_mujoco.so")

LEROBOT_SIM = {
    "fps": FPS,
    "control_mode": "position",
    "state_names": ["pusher_x", "pusher_y"],
    "action_names": ["target_x", "target_y"],
}

MODEL_XML = """
<mujoco model="openbeam_push">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <extension>
    <plugin plugin="openbeam.softbody">
      <instance name="box">
        <config key="jbeam" value="{jbeam}"/>
        <config key="pos" value="{corner} {corner} 0.005"/>
        <config key="colliders" value="pusher"/>
        <config key="stress_scale" value="150"/>
        <config key="beam_width" value="0.005"/>
      </instance>
    </plugin>
  </extension>
  <statistic center="0 0 0.1" extent="0.8"/>
  <visual>
    <global offwidth="640" offheight="640"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1=".75 .82 .9" rgb2=".25 .3 .38" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1=".55 .55 .58" rgb2=".45 .45 .48" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="18 18" reflectance=".05"/>
  </asset>
  <worldbody>
    <light pos="0.3 -0.6 2" dir="-0.1 0.3 -1" directional="true" castshadow="true"/>
    <geom name="floor" type="plane" size="3 3 0.05" material="grid"/>
    <body name="goal" mocap="true" pos="0.3 0 0.001">
      <geom type="cylinder" size="{goal_r} 0.001" rgba=".2 .8 .3 .6" contype="0" conaffinity="0"/>
    </body>
    <body name="pusher_body" pos="0 0 {pusher_z}">
      <joint name="x" type="slide" axis="1 0 0" damping="{damping}" range="-{ws} {ws}"/>
      <joint name="y" type="slide" axis="0 1 0" damping="{damping}" range="-{ws} {ws}"/>
      <geom name="pusher" type="capsule" size="{pusher_r} 0.07" mass="2" rgba=".2 .35 .9 1"
            contype="0" conaffinity="0"/>
    </body>
    <camera name="top" pos="0 0 1.9" xyaxes="1 0 0 0 1 0" fovy="40"/>
    <camera name="side" pos="0 -1.3 0.9" xyaxes="1 0 0 0 0.547 0.837" fovy="50"/>
  </worldbody>
  <actuator>
    <position name="target_x" joint="x" kp="{kp}" ctrlrange="-{ws} {ws}" forcerange="-{force} {force}"/>
    <position name="target_y" joint="y" kp="{kp}" ctrlrange="-{ws} {ws}" forcerange="-{force} {force}"/>
  </actuator>
  <sensor>
    <plugin name="box_state" instance="box"/>
  </sensor>
</mujoco>
"""

_PLUGINS_LOADED: set[str] = set()


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


def load_plugin() -> None:
    """Registers the "openbeam.softbody" plugin with MuJoCo, once per process."""
    path = str(plugin_path())
    if path not in _PLUGINS_LOADED:
        mujoco.mj_loadPluginLibrary(path)
        _PLUGINS_LOADED.add(path)


class OpenBeamPushEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": FPS}
    task_description = "Push the soft box onto the green goal without denting it."

    def __init__(self, obs_type: str = "pixels_agent_pos", observation_size: int = 96,
                 render_size: int = 384, max_episode_steps: int = 300, render_mode: str = "rgb_array"):
        if obs_type not in ("pixels_agent_pos", "state", "pixels_state"):
            raise ValueError(f"unknown obs_type {obs_type!r}")
        load_plugin()
        jbeam = (HERE / "soft_box.jbeam").as_posix()
        self.model = mujoco.MjModel.from_xml_string(MODEL_XML.format(
            timestep=TIMESTEP, jbeam=jbeam, corner=-BOX_HALF, goal_r=GOAL_TOLERANCE, pusher_r=PUSHER_RADIUS,
            pusher_z=PUSHER_RADIUS + 0.075,  # the capsule spans the box's height and clears the floor
            ws=WORKSPACE, kp=PUSHER_KP, force=PUSHER_FORCE, damping=PUSHER_DAMPING))
        self.data = mujoco.MjData(self.model)
        self.obs_type = obs_type
        self.observation_size = observation_size
        self.render_size = render_size
        self.render_mode = render_mode
        self._max_episode_steps = max_episode_steps
        self._sensor = self.model.sensor("box_state").adr[0]
        self._goal = np.zeros(2)
        self._steps = 0
        self._renderers: dict[int, mujoco.Renderer] = {}

        spaces = {"agent_pos": gym.spaces.Box(-WORKSPACE, WORKSPACE, (2,), np.float32)}
        if obs_type != "pixels_agent_pos":
            beams = 254  # in soft_box.jbeam
            low = np.array([-2, -2, -1, -WORKSPACE, -WORKSPACE, 0, 0], np.float32)
            high = np.array([2, 2, 1, WORKSPACE, WORKSPACE, beams, beams], np.float32)
            spaces["environment_state"] = gym.spaces.Box(low, high, (7,), np.float32)
        if obs_type != "state":
            spaces["pixels"] = gym.spaces.Box(0, 255, (observation_size, observation_size, 3), np.uint8)
        self.observation_space = gym.spaces.Dict(spaces)
        self.action_space = gym.spaces.Box(-WORKSPACE, WORKSPACE, (2,), np.float32)

    # -- state -------------------------------------------------------------
    def box_state(self) -> dict:
        s = self.data.sensordata[self._sensor: self._sensor + 6]
        return {"beams_broken": int(s[0]), "beams_deformed": int(s[1]), "centroid": s[2:5].copy(),
                "openbeam_steps": int(s[5])}

    def _distance(self) -> float:
        return float(np.linalg.norm(self.box_state()["centroid"][:2] - self._goal))

    def _obs(self) -> dict:
        obs = {"agent_pos": self.data.qpos[:2].astype(np.float32)}
        if self.obs_type != "pixels_agent_pos":
            b = self.box_state()
            obs["environment_state"] = np.array([*b["centroid"], *self._goal, b["beams_broken"],
                                                 b["beams_deformed"]], np.float32)
        if self.obs_type != "state":
            obs["pixels"] = self._render(self.observation_size, "top")
        return obs

    def _info(self) -> dict:
        b = self.box_state()
        distance = self._distance()
        undamaged = b["beams_broken"] == 0 and b["beams_deformed"] == 0
        return {"is_success": distance < GOAL_TOLERANCE and undamaged, "distance": distance,
                "beams_broken": b["beams_broken"], "beams_deformed": b["beams_deformed"]}

    # -- gym API -----------------------------------------------------------
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)  # the plugin restores the box's rest pose
        rng = self.np_random
        a, r = rng.uniform(-np.pi, np.pi), rng.uniform(0.25, 0.4)
        self._goal = np.array([r * np.cos(a), r * np.sin(a)])
        a = rng.uniform(-np.pi, np.pi)
        self.data.qpos[:2] = 0.42 * np.array([np.cos(a), np.sin(a)])
        self.data.ctrl[:] = self.data.qpos[:2]
        self.data.mocap_pos[0, :2] = self._goal
        mujoco.mj_forward(self.model, self.data)
        self._steps = 0
        return self._obs(), self._info()

    def step(self, action):
        self.data.ctrl[:] = np.clip(np.asarray(action, np.float64), -WORKSPACE, WORKSPACE)
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.model, self.data)
        self._steps += 1
        info = self._info()
        reward = -info["distance"] - 0.02 * info["beams_deformed"] - 0.1 * info["beams_broken"]
        terminated = bool(info["is_success"])
        truncated = self._steps >= self._max_episode_steps
        return self._obs(), reward, terminated, truncated, info

    def render(self):
        return self._render(self.render_size, "side")

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers.clear()

    def _render(self, size: int, camera: str) -> np.ndarray:
        renderer = self._renderers.get(size)
        if renderer is None:
            renderer = self._renderers[size] = mujoco.Renderer(self.model, size, size)
        renderer.update_scene(self.data, camera=camera)
        return renderer.render()

    # -- a scripted expert, for tests and demonstrations --------------------
    def expert_action(self) -> np.ndarray:
        """Goes round to the far side of the box from the goal, then pushes it there.

        In the frame of the push (u towards the goal, n across it): beside or in
        front of the box, step out sideways, then back past its corners; behind
        it, line up; lined up, push, easing off near the goal. Near the box it
        moves slowly, so it never strikes the box hard enough to dent it.
        """
        c = self.box_state()["centroid"][:2]
        p = self.data.qpos[:2].copy()
        to_goal = self._goal - c
        dist = float(np.linalg.norm(to_goal))
        u = to_goal / max(dist, 1e-9)
        n = np.array([-u[1], u[0]])
        along, lateral = float((p - c) @ u), float((p - c) @ n)
        clear = BOX_HALF * np.sqrt(2.0) + PUSHER_RADIUS + 0.03  # past a corner, however the box has turned
        if along < -BOX_HALF and abs(lateral) < 0.07:
            target = p + u * min(EXPERT_PUSH, 0.01 + dist) - n * lateral
        elif along < -clear + 0.02:
            target = c - u * clear
        else:
            side = n if lateral >= 0 else -n
            if abs(lateral) < clear:
                target = c + side * (clear + 0.04) + u * along
            else:
                target = c + side * (clear + 0.04) - u * clear
        lead = target - p
        if float(np.linalg.norm(p - c)) < clear + 0.1 and float(np.linalg.norm(lead)) > EXPERT_STEP_NEAR_BOX:
            target = p + lead * (EXPERT_STEP_NEAR_BOX / float(np.linalg.norm(lead)))
        return np.clip(target, -WORKSPACE, WORKSPACE).astype(np.float32)


def make_env(n_envs: int = 1, use_async_envs: bool = False, **kwargs):
    """EnvHub entry point: n_envs copies of the push task; kwargs go to OpenBeamPushEnv."""
    env_cls = gym.vector.AsyncVectorEnv if use_async_envs else gym.vector.SyncVectorEnv
    return env_cls([partial(OpenBeamPushEnv, **kwargs) for _ in range(n_envs)])
