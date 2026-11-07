# SPDX-License-Identifier: GPL-3.0-or-later
"""An OpenBeam soft body in an Isaac Sim stage, coupled to PhysX rigid bodies.

PhysX simulates the rigid bodies; OpenBeam simulates the JBeam structure. On
every physics step, before PhysX steps (SimulationEvent.PHYSICS_PRE_STEP):
  1. each collider body's pose and velocity is read through the physics
     tensor API and handed to OpenBeam;
  2. OpenBeam advances the physics dt in 0.5 ms steps against the ground
     plane and those colliders;
  3. the force and torque the soft body put on each collider over that
     interval are applied to its body for the step PhysX is about to take.
The structure is drawn as a BasisCurves prim, two linear half-curves per
beam: green at rest, yellow under load (stress_scale newtons), orange to red
where the beam has yielded for good (strain_scale of plastic strain), and a
torn beam as two red stubs snapped back towards its nodes.

Colliders are rigid-body prims that are, or have as a direct child, a
UsdGeom Sphere, Cube or Capsule (axis Z) centred on the body's origin.
"""

from __future__ import annotations

import os

import carb
import numpy as np
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, Vt

from . import core

_SHAPES = {"Sphere": core.SHAPE_SPHERE, "Cube": core.SHAPE_BOX, "Capsule": core.SHAPE_CAPSULE}


def _collider_shape(prim) -> tuple[int, np.ndarray, str]:
    """(core shape, size, gprim path) for a collider body prim."""
    candidates = [prim] + list(prim.GetChildren())
    for p in candidates:
        kind = p.GetTypeName()
        if kind not in _SHAPES:
            continue
        world = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0.0)
        scale = np.array(Gf.Transform(world).GetScale())
        if kind == "Sphere":
            r = UsdGeom.Sphere(p).GetRadiusAttr().Get() * float(scale.max())
            return core.SHAPE_SPHERE, np.array([r, 0.0, 0.0]), str(p.GetPath())
        if kind == "Cube":
            half = 0.5 * UsdGeom.Cube(p).GetSizeAttr().Get() * scale
            return core.SHAPE_BOX, half, str(p.GetPath())
        cap = UsdGeom.Capsule(p)
        if cap.GetAxisAttr().Get() != "Z":
            raise ValueError(f"{p.GetPath()}: OpenBeam capsule colliders must use axis Z")
        r = cap.GetRadiusAttr().Get() * float(max(scale[0], scale[1]))
        return core.SHAPE_CAPSULE, np.array([r, 0.5 * cap.GetHeightAttr().Get() * scale[2], 0.0]), str(p.GetPath())
    raise ValueError(f"{prim.GetPath()}: no Sphere, Cube or Capsule to use as an OpenBeam collider")


def _scene_gravity(stage) -> np.ndarray:
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene = UsdPhysics.Scene(prim)
            direction = np.array(scene.GetGravityDirectionAttr().Get() or (0.0, 0.0, -1.0), dtype=float)
            magnitude = scene.GetGravityMagnitudeAttr().Get()
            if direction.any() and magnitude is not None and np.isfinite(magnitude):
                if magnitude < 0:  # "use the default", 9.81 m/s^2 in metres
                    magnitude = 9.81
                return direction / np.linalg.norm(direction) * magnitude
    return np.array([0.0, 0.0, -9.81])


class SoftBody:
    """One JBeam structure simulated by OpenBeam inside the current Isaac Sim stage."""

    def __init__(self, jbeam_path: str | os.PathLike, *, prim_path: str = "/World/OpenBeam",
                 position=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0), colliders=(),
                 ground_height: float | None = 0.0, stress_scale: float = 2000.0, strain_scale: float = 0.05,
                 beam_width: float = 0.012,
                 stage=None):
        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

        self._sm = SimulationManager
        self.stage = stage or omni.usd.get_context().get_stage()
        self.world = core.World(jbeam_path, position, orientation)
        self.world.set_gravity(_scene_gravity(self.stage))
        if ground_height is not None:
            self.world.add_plane((0.0, 0.0, ground_height), (0.0, 0.0, 1.0))
        self.collider_paths = [str(p) for p in colliders]
        self._collider_ids = []
        for path in self.collider_paths:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(f"{path}: an OpenBeam collider must be a rigid body prim")
            shape, size, _ = _collider_shape(prim)
            self._collider_ids.append(self.world.add_collider(shape, size))
        self.stress_scale = float(stress_scale)
        self.strain_scale = float(strain_scale)
        self.prim_path = prim_path
        self._make_curves(beam_width)
        self._rb_view = None
        self._indices = None
        self._last_draw_time = -1.0
        self.sim_time = 0.0
        self._subs = [
            SimulationManager.register_callback(self._on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP),
            SimulationManager.register_callback(self._on_stopped, event=SimulationEvent.SIMULATION_STOPPED),
        ]
        self.draw()

    # -- lifetime -----------------------------------------------------------

    def destroy(self) -> None:
        for uid in self._subs:
            self._sm.deregister_callback(uid)
        self._subs = []
        self._rb_view = None

    def reset(self) -> None:
        self.world.reset()
        self.sim_time = 0.0
        self.draw()

    def _on_stopped(self, *_):
        # The timeline was stopped: PhysX restores the stage, OpenBeam does the same.
        self._rb_view = None
        self.reset()

    # -- coupling -----------------------------------------------------------

    def _ensure_view(self) -> bool:
        if not self.collider_paths:
            return True
        if self._rb_view is not None:
            return True
        sim_view = self._sm.get_physics_simulation_view()
        if sim_view is None:
            return False
        import warp as wp

        self._rb_view = sim_view.create_rigid_body_view(self.collider_paths)
        if self._rb_view.count != len(self.collider_paths):
            carb.log_error(f"OpenBeam: {self._rb_view.count} of {len(self.collider_paths)} colliders found by PhysX")
            self._rb_view = None
            return False
        self._device = getattr(sim_view, "device", "cpu")
        self._wp = wp
        self._indices = wp.array(np.arange(self._rb_view.count, dtype=np.int32), dtype=wp.int32, device=self._device)
        return True

    def _host(self, array, width: int) -> np.ndarray:
        data = array.numpy() if hasattr(array, "numpy") else np.asarray(array)
        return np.asarray(data, dtype=np.float64).reshape(-1, width)

    def _on_pre_step(self, dt: float, context=None) -> None:
        if not self._ensure_view():
            return
        n = len(self._collider_ids)
        if n:
            tf = self._host(self._rb_view.get_transforms(), 7)
            vel = self._host(self._rb_view.get_velocities(), 6)
            self._origins = tf[:, :3].copy()
            for i, cid in enumerate(self._collider_ids):
                quat = (tf[i, 6], tf[i, 3], tf[i, 4], tf[i, 5])  # tensor API is x y z w
                # Shapes are centred on the body origin, taken as the centre of
                # mass of these primitives, so the COM velocity is the origin's.
                self.world.set_collider_state(cid, tf[i, :3], quat, vel[i, :3], vel[i, 3:])
        self.world.advance(dt)
        self.sim_time += dt
        if n:
            forces = np.zeros((n, 3), dtype=np.float32)
            torques = np.zeros((n, 3), dtype=np.float32)
            for i, cid in enumerate(self._collider_ids):
                f, t = self.world.collider_wrench(cid, self._origins[i])
                forces[i], torques[i] = f, t
            if np.any(forces) or np.any(torques):
                wp = self._wp
                self._rb_view.apply_forces_and_torques_at_position(
                    wp.array(forces, dtype=wp.float32, device=self._device),
                    wp.array(torques, dtype=wp.float32, device=self._device),
                    wp.array(self._origins.astype(np.float32), dtype=wp.float32, device=self._device),
                    self._indices, True)
        if self.sim_time - self._last_draw_time >= 1.0 / 60.0 - 1e-9:
            self.draw()

    # -- drawing ------------------------------------------------------------

    def _make_curves(self, width: float) -> None:
        curves = UsdGeom.BasisCurves.Define(self.stage, Sdf.Path(self.prim_path))
        curves.CreateTypeAttr(UsdGeom.Tokens.linear)
        curves.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
        curves.CreateCurveVertexCountsAttr(Vt.IntArray([2] * (2 * self.world.beam_count)))  # two halves per beam
        curves.CreateWidthsAttr(Vt.FloatArray([width]))
        curves.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        self._color = curves.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
        self._curves = curves
        self._points = curves.CreatePointsAttr()

    def draw(self) -> None:
        pos = self.world.positions()
        stress, rest, _, broken = self.world.beam_state()
        plastic, damage = self.world.beam_damage()
        ends = pos[self.world.beam_nodes]                    # (beams, 2, 3)
        a, c = ends[:, 0], ends[:, 1]
        # Severity 0 (green) to 0.5 (yellow) for live load; 0.6 (orange) to
        # 0.95 (red) once the beam has yielded or taken damage.
        s = 0.5 * np.clip(np.abs(stress) / self.stress_scale, 0.0, 1.0)
        permanent = np.maximum(np.clip(np.abs(plastic) / self.strain_scale, 0.0, 1.0), damage)
        s = np.where((np.abs(plastic) > 1e-4) | (damage > 0.0), 0.6 + 0.35 * permanent, s)
        color = np.stack([np.minimum(1.0, 2.0 * s), np.minimum(1.0, 2.0 * (1.0 - s)), np.full_like(s, 0.1)], 1)
        color[broken] = (1.0, 0.05, 0.05)
        # An intact beam's halves meet in the middle; a torn beam's are stubs at each node.
        span = np.linalg.norm(c - a, axis=1)
        f = np.minimum(0.25, 0.3 * rest / np.maximum(span, 1e-9))[:, None]
        mid = 0.5 * (a + c)
        first_end = np.where(broken[:, None], a + f * (c - a), mid)
        second_start = np.where(broken[:, None], c + f * (a - c), mid)
        points = np.stack([a, first_end, second_start, c], axis=1)
        self._points.Set(Vt.Vec3fArray.FromNumpy(points.reshape(-1, 3).astype(np.float32)))
        self._color.Set(Vt.Vec3fArray.FromNumpy(np.repeat(color, 2, axis=0).astype(np.float32)))
        extent = UsdGeom.Curves.ComputeExtent(self._points.Get(), Vt.FloatArray([0.02]))
        if extent:
            self._curves.GetExtentAttr().Set(extent)
        self._last_draw_time = self.sim_time

    # -- readout ------------------------------------------------------------

    def stats(self) -> dict:
        s = self.world.stats()
        s["centroid"] = self.world.positions().mean(axis=0).tolist()
        return s
