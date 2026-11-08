# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests of the OpenBeam cable-routing EnvHub package, through LeRobot's own loader.

    python -m pytest lerobot/tests -q

They need the MuJoCo plugin build (build_windows.ps1 or build_linux.sh) and a venv
with lerobot and mujoco (README.md). Tests of pixel observations need OpenGL
(on a headless Linux machine, MUJOCO_GL=egl or osmesa) and skip without it. An
episode runs at about half real time, so these take a few minutes.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1] / "openbeam_cable"
REPO = PKG.parents[1]


def _load_env_module():
    spec = importlib.util.spec_from_file_location("openbeam_cable_env", PKG / "env.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ENV = _load_env_module()


def _can_render() -> bool:
    """Whether MuJoCo can render here; probed in a child process, since a failed GL context can abort it."""
    probe = "import mujoco; mujoco.Renderer(mujoco.MjModel.from_xml_string('<mujoco/>'), 8, 8).render()"
    return subprocess.run([sys.executable, "-c", probe], capture_output=True, timeout=120).returncode == 0


requires_gl = pytest.mark.skipif(not _can_render(), reason="MuJoCo cannot render here (no OpenGL 1.5)")


def rollout(env, seed: int, policy) -> tuple[dict, int]:
    env.reset(seed=seed)
    for step in range(1, env._max_episode_steps + 1):
        _, _, terminated, truncated, info = env.step(policy(env))
        if terminated or truncated:
            break
    return info, step


@pytest.fixture(scope="module")
def env():
    e = ENV.OpenBeamCableEnv(obs_type="state")
    yield e
    e.close()


@pytest.mark.parametrize("obs_type", ["state", pytest.param("pixels_state", marks=requires_gl)])
def test_gymnasium_env_checker(obs_type):
    from gymnasium.utils.env_checker import check_env

    e = ENV.OpenBeamCableEnv(obs_type=obs_type)
    check_env(e, skip_render_check=True)
    e.close()


def test_reset_is_deterministic_and_moves_the_ring_and_port(env):
    first, _ = env.reset(seed=3)
    again, _ = env.reset(seed=3)
    np.testing.assert_array_equal(again["environment_state"], first["environment_state"])
    other, _ = env.reset(seed=4)
    ring, port = slice(6, 9), slice(9, 12)
    assert not np.allclose(other["environment_state"][ring], first["environment_state"][ring])
    assert not np.allclose(other["environment_state"][port], first["environment_state"][port])


@pytest.mark.parametrize("seed", range(3))
def test_expert_routes_the_cable_and_latches_the_plug_undamaged(env, seed):
    """The cable takes a set on the way (its copper yields) but stays above its rated bend radius and is
    not damaged; the plug latches with the cable through the ring."""
    info, steps = rollout(env, seed, lambda e: e.expert_action())
    assert info["is_success"], info
    assert info["latched"] and info["threads_ring"]
    assert info["beams_damaged"] == 0 and info["beams_torn"] == 0 and info["beams_yielded"] > 0
    assert info["time_below_rating_s"] == 0.0
    assert steps < 130


def test_skipping_the_ring_does_not_succeed(env):
    """Straight from the lift to the port, past the ring: the plug latches, the cable misses the ring."""
    env.reset(seed=0)
    env._expert_path = [w for w in env._expert_path if not 3.0 < w[0] < 5.0]
    for _ in range(env._max_episode_steps):
        _, _, terminated, truncated, info = env.step(env.expert_action())
        if info["latched"] or terminated or truncated:
            break
    assert info["latched"] and not info["threads_ring"] and not info["is_success"]


@pytest.fixture
def at_repo_root(monkeypatch):
    # LeRobot splits hub_path at its first ':', so a Windows drive path cannot be used: load it relatively.
    monkeypatch.chdir(REPO)
    return os.path.relpath(PKG, REPO).replace(os.sep, "/")


@requires_gl
def test_lerobot_hub_env_config_loads_and_steps(at_repo_root):
    from lerobot.envs.configs import HubEnvConfig

    envs = HubEnvConfig(hub_path=at_repo_root, trust_remote_code=True).create_envs(n_envs=2)
    e = envs["hub_env"][0]
    obs, info = e.reset(seed=[0, 1])
    assert obs["observation.image"].shape == (2, 128, 128, 3) and obs["observation.image"].dtype == np.uint8
    assert obs["observation.state"].shape == (2, 5)
    assert info["task"][0] == ENV.OpenBeamCableEnv.task_description
    d = e.descriptor
    assert (d.fps, d.control_mode, d.max_episode_steps) == (20, "position", 200)
    assert d.action_feature.names == ("target_x", "target_y", "target_z", "target_yaw", "grip")
    hold = np.tile(np.array([0.715, 0.25, 0.08, 0.0, 0.0], np.float32), (2, 1))
    obs, reward, terminated, truncated, info = e.step(hold)
    assert reward.shape == (2,) and not terminated.any() and not truncated.any()
    e.close()


def test_lerobot_runs_the_expert_to_success(at_repo_root):
    """The state observation through LeRobot carries what a policy needs, and success reaches LeRobot."""
    from lerobot.envs.configs import HubEnvConfig

    cfg = HubEnvConfig(hub_path=at_repo_root, trust_remote_code=True, make_env_kwargs={"obs_type": "state"})
    e = cfg.create_envs(n_envs=1)["hub_env"][0]
    obs, _ = e.reset(seed=[2])
    expert = ENV.OpenBeamCableEnv(obs_type="state")  # the same seed gives the same scene: drive it in step
    expert.reset(seed=2)
    for _ in range(130):
        np.testing.assert_allclose(obs["observation.environment_state"][0], expert._obs(expert.cable_state())
                                   ["environment_state"], rtol=1e-6, atol=1e-6)
        action = expert.expert_action()
        expert.step(action)
        obs, _, terminated, _, info = e.step(action[None])
        if terminated[0]:
            break
    assert info["is_success"][0]
    e.close()
    expert.close()
