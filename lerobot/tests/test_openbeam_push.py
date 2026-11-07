# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests of the OpenBeam push EnvHub package, through LeRobot's own loader.

    python -m pytest lerobot/tests -q

They need the MuJoCo plugin build (build_windows.ps1 or build_linux.sh) and a venv
with lerobot and mujoco (README.md). Tests of pixel observations need OpenGL
(on a headless Linux machine, MUJOCO_GL=egl or osmesa) and skip without it.
"""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

PKG = Path(__file__).resolve().parents[1] / "openbeam_push"
REPO = PKG.parents[1]


def _load_env_module():
    spec = importlib.util.spec_from_file_location("openbeam_push_env", PKG / "env.py")
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
    obs, info = env.reset(seed=seed)
    for step in range(1, env._max_episode_steps + 1):
        obs, _, terminated, truncated, info = env.step(policy(env))
        if terminated or truncated:
            break
    return info, step


@pytest.mark.parametrize("obs_type", ["state", pytest.param("pixels_state", marks=requires_gl)])
def test_gymnasium_env_checker(obs_type):
    from gymnasium.utils.env_checker import check_env

    env = ENV.OpenBeamPushEnv(obs_type=obs_type)
    check_env(env, skip_render_check=True)
    env.close()


def test_box_rests_and_reset_is_deterministic():
    env = ENV.OpenBeamPushEnv(obs_type="state")
    first, _ = env.reset(seed=7)
    for _ in range(40):  # 2 s with the pusher held where it starts, clear of the box
        obs, *_ = env.step(first["agent_pos"])
    state = obs["environment_state"]
    assert np.abs(state[:2] - first["environment_state"][:2]).max() < 2e-3  # it does not wander
    assert 0.09 < state[2] < 0.11 and state[5] == 0 and state[6] == 0  # it stands, unharmed
    again, _ = env.reset(seed=7)
    np.testing.assert_array_equal(again["environment_state"], first["environment_state"])
    np.testing.assert_array_equal(again["agent_pos"], first["agent_pos"])
    env.close()


@pytest.mark.parametrize("seed", range(6))
def test_expert_pushes_box_onto_goal_undamaged(seed):
    env = ENV.OpenBeamPushEnv(obs_type="state")
    info, steps = rollout(env, seed, lambda e: e.expert_action())
    assert info["is_success"], info
    assert info["beams_deformed"] == 0 and info["beams_broken"] == 0
    assert steps < 150
    env.close()


def test_ramming_dents_the_box():
    """Driven through the box at full force, the pusher dents it, and the episode cannot succeed."""
    env = ENV.OpenBeamPushEnv(obs_type="state")
    env.reset(seed=0)
    env.data.qpos[:2] = env.data.ctrl[:] = (-0.55, 0.0)
    mujoco.mj_forward(env.model, env.data)
    for _ in range(40):
        _, _, terminated, _, info = env.step(np.array([0.6, 0.0], np.float32))
        assert not terminated
    assert info["beams_deformed"] >= 5 and not info["is_success"]
    env.close()


def test_contact_couples_both_ways():
    """The pusher moves the box, and the box holds the pusher back from its target."""
    env = ENV.OpenBeamPushEnv(obs_type="state")
    obs, _ = env.reset(seed=0)
    env.data.qpos[:2] = env.data.ctrl[:] = (-0.3, 0.0)
    mujoco.mj_forward(env.model, env.data)
    start = env.box_state()["centroid"].copy()
    for _ in range(20):  # 1 s, leading the pusher 2 cm into the box
        env.step(np.array([env.data.qpos[0] + 0.02, 0.0], np.float32))
    moved = env.box_state()["centroid"] - start
    assert moved[0] > 0.05 and abs(moved[1]) < 0.03
    assert env.data.qfrc_passive[0] < -1.0  # OpenBeam's reaction on the pusher
    env.close()


@pytest.fixture
def at_repo_root(monkeypatch):
    # LeRobot splits hub_path at its first ':', so a Windows drive path cannot be used: load it relatively.
    monkeypatch.chdir(REPO)
    return os.path.relpath(PKG, REPO).replace(os.sep, "/")


@requires_gl
def test_lerobot_hub_env_config_loads_and_steps(at_repo_root):
    from lerobot.envs.configs import HubEnvConfig

    envs = HubEnvConfig(hub_path=at_repo_root, trust_remote_code=True).create_envs(n_envs=2)
    env = envs["hub_env"][0]
    obs, info = env.reset(seed=[0, 1])
    assert obs["observation.image"].shape == (2, 96, 96, 3) and obs["observation.image"].dtype == np.uint8
    assert obs["observation.state"].shape == (2, 2)
    assert info["task"][0] == ENV.OpenBeamPushEnv.task_description
    d = env.descriptor
    assert (d.fps, d.control_mode, d.max_episode_steps) == (20, "position", 300)
    assert d.action_feature.names == ("target_x", "target_y")
    obs, reward, terminated, truncated, info = env.step(np.zeros((2, 2), np.float32))
    assert reward.shape == (2,) and not terminated.any() and not truncated.any()
    env.close()


def test_lerobot_runs_the_expert_to_success(at_repo_root):
    """The state observation through LeRobot carries what a policy needs, and success reaches LeRobot."""
    from lerobot.envs.configs import HubEnvConfig

    cfg = HubEnvConfig(hub_path=at_repo_root, trust_remote_code=True, make_env_kwargs={"obs_type": "state"})
    env = cfg.create_envs(n_envs=1)["hub_env"][0]
    obs, _ = env.reset(seed=[2])
    expert = ENV.OpenBeamPushEnv(obs_type="state")  # the same seed gives the same scene: drive it in step
    expert.reset(seed=2)
    for _ in range(150):
        np.testing.assert_allclose(obs["observation.environment_state"][0], expert._obs()["environment_state"])
        action = expert.expert_action()
        expert.step(action)
        obs, _, terminated, _, info = env.step(action[None])
        if terminated[0]:
            break
    assert info["is_success"][0]
    env.close()
    expert.close()
