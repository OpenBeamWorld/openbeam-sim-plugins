# SPDX-License-Identifier: GPL-3.0-or-later
"""ctypes binding of openbeam_core (openbeam_core/include/openbeam_core.h).

Pure Python and NumPy: it runs inside Isaac Sim's Python and outside it. The
library is found at $OPENBEAM_CORE_DLL, in this extension's bin/ folder, or in
the repository's build/install/ (build_windows.ps1, build_linux.sh).
"""

from __future__ import annotations

import ctypes
import os
import sys
from pathlib import Path

import numpy as np

SHAPE_SPHERE, SHAPE_BOX, SHAPE_CAPSULE = 0, 1, 2
API_VERSION = 2  # OB_CORE_API_VERSION this binding is written against

_EXT_ROOT = Path(__file__).resolve().parents[3]
_LIB_NAME = {"win32": "openbeam_core.dll", "darwin": "libopenbeam_core.dylib"}.get(sys.platform,
                                                                                 "libopenbeam_core.so")
_LIB_DIR = "bin" if sys.platform == "win32" else "lib"  # where CMake installs a shared library


class Stats(ctypes.Structure):
    _fields_ = [("steps", ctypes.c_ulonglong), ("contacts", ctypes.c_ulonglong),
                ("beams_broken", ctypes.c_uint), ("beams_deformed", ctypes.c_uint),
                ("beams_skipped", ctypes.c_uint), ("last_advance_us", ctypes.c_double)]


class GroundModel(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in ("ms", "mc", "va", "vs", "t2", "alpha", "strength")]


def library_path() -> Path:
    env = os.environ.get("OPENBEAM_CORE_DLL")
    if env:
        return Path(env)
    candidates = [_EXT_ROOT / "bin" / _LIB_NAME]
    candidates += [p / "build" / "install" / _LIB_DIR / _LIB_NAME for p in _EXT_ROOT.parents]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(f"{_LIB_NAME} not found: build the repository (README.md) or set OPENBEAM_CORE_DLL")


_lib = None


def _load() -> ctypes.CDLL:
    global _lib
    if _lib is not None:
        return _lib
    lib = ctypes.CDLL(str(library_path()))
    P, D, I, U = ctypes.c_void_p, ctypes.c_double, ctypes.c_int, ctypes.c_size_t
    sig = {
        "ob_core_api_version": (I, []),
        "ob_world_create_from_jbeam_file": (P, [ctypes.c_char_p, P, P, ctypes.c_char_p, U]),
        "ob_world_create_from_jbeam_text": (P, [ctypes.c_char_p, ctypes.c_char_p, P, P, ctypes.c_char_p, U]),
        "ob_world_destroy": (None, [P]),
        "ob_world_clone": (P, [P]),
        "ob_world_reset": (None, [P]),
        "ob_world_set_gravity": (None, [P, P]),
        "ob_world_add_plane": (I, [P, P, P, P]),
        "ob_world_add_collider": (I, [P, I, P, P]),
        "ob_world_set_collider_state": (None, [P, I, P, P, P, P]),
        "ob_world_advance": (I, [P, D]),
        "ob_world_get_collider_wrench": (None, [P, I, P, P, P]),
        "ob_world_node_count": (I, [P]),
        "ob_world_beam_count": (I, [P]),
        "ob_world_node_id": (ctypes.c_char_p, [P, I]),
        "ob_world_total_mass": (D, [P]),
        "ob_world_get_node_positions": (None, [P, P]),
        "ob_world_get_node_velocities": (None, [P, P]),
        "ob_world_get_beam_nodes": (None, [P, P]),
        "ob_world_get_beam_state": (None, [P, P, P, P, P]),
        "ob_world_get_beam_damage": (None, [P, P, P]),
        "ob_world_set_step": (I, [P, D]),
        "ob_world_get_step": (D, [P]),
        "ob_world_set_node_radius": (None, [P, D]),
        "ob_world_set_node_damping": (None, [P, D]),
        "ob_world_attach_node": (I, [P, I, I]),
        "ob_world_detach_node": (I, [P, I]),
        "ob_world_release_node": (I, [P, I, D]),
        "ob_world_detach_all": (None, [P]),
        "ob_world_attached_count": (I, [P]),
        "ob_world_find_node": (I, [P, ctypes.c_char_p]),
        "ob_world_load_materials": (I, [P, ctypes.c_char_p, ctypes.c_char_p, U]),
        "ob_world_get_stats": (None, [P, ctypes.POINTER(Stats)]),
        "ob_world_state_digest": (ctypes.c_ulonglong, [P]),
    }
    for name, (res, args) in sig.items():
        fn = getattr(lib, name)
        fn.restype = res
        fn.argtypes = args
    if lib.ob_core_api_version() != API_VERSION:
        raise RuntimeError(f"openbeam_core API version {lib.ob_core_api_version()}, expected {API_VERSION}: "
                           "rebuild the library from this repository")
    _lib = lib
    return lib


def _vec(values, n: int) -> np.ndarray:
    return np.ascontiguousarray(np.asarray(values, dtype=np.float64).reshape(n))


class World:
    """One OpenBeam soft body. Units SI, Z up, quaternions (w, x, y, z)."""

    def __init__(self, jbeam_path: str | os.PathLike, position=(0.0, 0.0, 0.0), quaternion=(1.0, 0.0, 0.0, 0.0)):
        self._lib = _load()
        err = ctypes.create_string_buffer(512)
        pos, quat = _vec(position, 3), _vec(quaternion, 4)
        handle = self._lib.ob_world_create_from_jbeam_file(
            str(jbeam_path).encode("utf-8"), pos.ctypes.data, quat.ctypes.data, err, len(err))
        if not handle:
            raise ValueError(f"OpenBeam could not load {jbeam_path}: {err.value.decode('utf-8', 'replace')}")
        self._h = handle
        self.node_count = self._lib.ob_world_node_count(handle)
        self.beam_count = self._lib.ob_world_beam_count(handle)
        pairs = np.zeros((self.beam_count, 2), dtype=np.int32)
        self._lib.ob_world_get_beam_nodes(handle, pairs.ctypes.data)
        self.beam_nodes = pairs

    def __del__(self):
        h = getattr(self, "_h", None)
        if h:
            self._lib.ob_world_destroy(h)
            self._h = None

    @property
    def total_mass(self) -> float:
        return self._lib.ob_world_total_mass(self._h)

    def node_id(self, i: int) -> str:
        return self._lib.ob_world_node_id(self._h, i).decode("utf-8")

    def reset(self) -> None:
        self._lib.ob_world_reset(self._h)

    # Every array handed to C is bound to a name first: `.ctypes.data` is a
    # bare address and does not keep a temporary array alive for the call.

    def set_gravity(self, g) -> None:
        g = _vec(g, 3)
        self._lib.ob_world_set_gravity(self._h, g.ctypes.data)

    def add_plane(self, point, normal) -> int:
        point, normal = _vec(point, 3), _vec(normal, 3)
        return self._lib.ob_world_add_plane(self._h, point.ctypes.data, normal.ctypes.data, None)

    def add_collider(self, shape: int, size) -> int:
        size = _vec(size, 3)
        cid = self._lib.ob_world_add_collider(self._h, shape, size.ctypes.data, None)
        if cid < 0:
            raise ValueError(f"bad collider shape {shape} or size {size}")
        return cid

    def set_collider_state(self, cid: int, position, quaternion, linear_velocity, angular_velocity) -> None:
        p, q = _vec(position, 3), _vec(quaternion, 4)
        v, w = _vec(linear_velocity, 3), _vec(angular_velocity, 3)
        self._lib.ob_world_set_collider_state(self._h, cid, p.ctypes.data, q.ctypes.data, v.ctypes.data,
                                              w.ctypes.data)

    def advance(self, dt: float) -> int:
        return self._lib.ob_world_advance(self._h, float(dt))

    def collider_wrench(self, cid: int, reference_point):
        force, torque, ref = np.zeros(3), np.zeros(3), _vec(reference_point, 3)
        self._lib.ob_world_get_collider_wrench(self._h, cid, ref.ctypes.data, force.ctypes.data,
                                               torque.ctypes.data)
        return force, torque

    def positions(self) -> np.ndarray:
        out = np.zeros((self.node_count, 3))
        self._lib.ob_world_get_node_positions(self._h, out.ctypes.data)
        return out

    def velocities(self) -> np.ndarray:
        out = np.zeros((self.node_count, 3))
        self._lib.ob_world_get_node_velocities(self._h, out.ctypes.data)
        return out

    def beam_state(self):
        """(stress N, rest length m, strength N, broken bool) per beam."""
        stress = np.zeros(self.beam_count, dtype=np.float32)
        rest = np.zeros(self.beam_count, dtype=np.float32)
        strength = np.zeros(self.beam_count, dtype=np.float32)
        broken = np.zeros(self.beam_count, dtype=np.uint8)
        self._lib.ob_world_get_beam_state(self._h, stress.ctypes.data, rest.ctypes.data, strength.ctypes.data,
                                          broken.ctypes.data)
        return stress, rest, strength, broken.astype(bool)

    def set_step(self, step: float) -> None:
        if self._lib.ob_world_set_step(self._h, float(step)) != 0:
            raise ValueError(f"step {step} s is outside 1e-6 to 0.0005 s")

    @property
    def step(self) -> float:
        return self._lib.ob_world_get_step(self._h)

    def set_node_radius(self, radius: float) -> None:
        self._lib.ob_world_set_node_radius(self._h, float(radius))

    def set_node_damping(self, per_second: float) -> None:
        self._lib.ob_world_set_node_damping(self._h, float(per_second))

    def find_node(self, node_id: str) -> int:
        return self._lib.ob_world_find_node(self._h, node_id.encode())

    def nodes_matching(self, pattern: str) -> list[int]:
        """Node indices for a JBeam id, or for every id starting with the text before a trailing '*'."""
        if pattern.endswith("*"):
            return [i for i in range(self.node_count) if self.node_id(i).startswith(pattern[:-1])]
        i = self.find_node(pattern)
        return [i] if i >= 0 else []

    def attach(self, pattern: str, collider: int) -> int:
        """Welds the matching nodes to a collider where they are now; returns how many."""
        return sum(self._lib.ob_world_attach_node(self._h, i, collider) == 0 for i in self.nodes_matching(pattern))

    def detach(self, pattern: str | None = None, seconds: float = 0.0) -> None:
        """Releases the matching nodes (all if pattern is None), the grip fading over `seconds`."""
        if pattern is None and seconds == 0.0:
            self._lib.ob_world_detach_all(self._h)
            return
        nodes = range(self.node_count) if pattern is None else self.nodes_matching(pattern)
        for i in nodes:
            self._lib.ob_world_release_node(self._h, i, float(seconds))

    def load_materials(self, path) -> int:
        err = ctypes.create_string_buffer(512)
        n = self._lib.ob_world_load_materials(self._h, str(path).encode(), err, len(err))
        if n < 0:
            raise ValueError(err.value.decode(errors="replace"))
        return n

    def beam_damage(self):
        """(permanent plastic strain, damage 0 intact to 1 torn) per beam."""
        plastic = np.zeros(self.beam_count, dtype=np.float32)
        damage = np.zeros(self.beam_count, dtype=np.float32)
        self._lib.ob_world_get_beam_damage(self._h, plastic.ctypes.data, damage.ctypes.data)
        return plastic, damage

    def stats(self) -> dict:
        s = Stats()
        self._lib.ob_world_get_stats(self._h, ctypes.byref(s))
        return {name: getattr(s, name) for name, _ in Stats._fields_}

    def digest(self) -> int:
        return self._lib.ob_world_state_digest(self._h)
