# SPDX-License-Identifier: GPL-3.0-or-later
"""The wrecking-ball demo scene, the same as the MuJoCo one
(mujoco/models/wrecking_ball.xml): a 300 kg PhysX ball on a
3 m revolute pendulum, released from horizontal, swings into OpenBeam's
JBeam truss crate."""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

# assets/, from isaacsim/exts/<ext>/openbeam/isaacsim/softbody/.
DEFAULT_CRATE = Path(__file__).resolve().parents[6] / "assets" / "truss_crate.jbeam"

PIVOT = Gf.Vec3f(0.0, 0.0, 3.6)
ARM = 3.0
RELEASE = math.radians(90.0)
CRATE_POSITION = (0.45, -0.6, 0.02)


def build_wrecking_ball(stage, physics_dt: float = 1.0 / 200.0) -> dict:
    """Adds the scene to `stage` (Z up, metres) and returns its prim paths."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateTimeStepsPerSecondAttr(round(1.0 / physics_dt))
    physx.CreateEnableGPUDynamicsAttr(False)
    physx.CreateBroadphaseTypeAttr("MBP")

    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    ground.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateDisplayColorAttr([(0.5, 0.5, 0.53)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(3000.0)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 20.0, 0.0))
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(600.0)

    # The ball starts on its arc, swung back towards -x.
    start = PIVOT + Gf.Vec3f(-ARM * math.sin(RELEASE), 0.0, -ARM * math.cos(RELEASE))
    ball = UsdGeom.Sphere.Define(stage, "/World/Ball")
    ball.CreateRadiusAttr(0.4)
    ball.CreateDisplayColorAttr([(0.15, 0.15, 0.18)])
    UsdGeom.Xformable(ball).AddTranslateOp().Set(Gf.Vec3d(start))
    UsdPhysics.RigidBodyAPI.Apply(ball.GetPrim())
    UsdPhysics.CollisionAPI.Apply(ball.GetPrim())
    UsdPhysics.MassAPI.Apply(ball.GetPrim()).CreateMassAttr(300.0)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(ball.GetPrim())
    rb.CreateLinearDampingAttr(0.0)
    rb.CreateAngularDampingAttr(0.0)

    joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/BallPivot")
    joint.CreateAxisAttr("Y")
    joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Ball")])
    joint.CreateLocalPos0Attr(PIVOT)
    joint.CreateLocalPos1Attr(PIVOT - start)
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0))

    rod = UsdGeom.BasisCurves.Define(stage, "/World/Ball/Rod")
    rod.CreateTypeAttr(UsdGeom.Tokens.linear)
    rod.CreateCurveVertexCountsAttr([2])
    rod.CreatePointsAttr([Gf.Vec3f(0.0), PIVOT - start])
    rod.CreateWidthsAttr([0.05])
    rod.CreateDisplayColorAttr([(0.25, 0.25, 0.25)])
    return {"ball": "/World/Ball", "joint": "/World/BallPivot", "crate_position": CRATE_POSITION}


# -- the data-centre patching scene of mujoco/models/cable_patch.xml -----------------------------------

DEFAULT_CABLE = Path(__file__).resolve().parents[6] / "assets" / "cat6_cable.jbeam"


def _material(stage, name: str, color, metallic: float = 0.0, roughness: float = 0.6):
    """A UsdPreviewSurface material, which RTX renders as given."""
    path = f"/World/Looks/{name}"
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _box(stage, path: str, centre, half, material=None, visible: bool = True) -> str:
    """A Cube gprim of the given half extents (size 2, scaled), as OpenBeam's collider reads it."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(2.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
    xf.AddScaleOp().Set(Gf.Vec3f(*half))
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    if not visible:
        cube.MakeInvisible()
    return path


def build_cable_patch(stage, physics_dt: float = 1.0 / 200.0, plug_start=(0.565, 0.25, 0.0082)) -> dict:
    """A desk with a wall outlet, a rack D-ring and a 1U patch panel, and a gripper above the cable's
    plug, laid out as in the MuJoCo scene. Every part OpenBeam touches is a kinematic collider: an
    unsimulated USD prim the script moves (the gripper) or leaves still. Returns the prim paths."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateTimeStepsPerSecondAttr(round(1.0 / physics_dt))
    physx.CreateEnableGPUDynamicsAttr(False)

    metal = _material(stage, "Metal", (0.32, 0.34, 0.37), 0.7, 0.35)
    panel_black = _material(stage, "Panel", (0.05, 0.05, 0.06), 0.1, 0.5)
    desk = UsdGeom.Mesh.Define(stage, "/World/Desk")
    desk.CreatePointsAttr([(-1.5, -1.5, 0), (1.5, -1.5, 0), (1.5, 1.5, 0), (-1.5, 1.5, 0)])
    desk.CreateFaceVertexCountsAttr([4])
    desk.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    UsdShade.MaterialBindingAPI.Apply(desk.GetPrim()).Bind(_material(stage, "Desk", (0.42, 0.43, 0.46), 0.0, 0.8))
    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(900.0)
    light.CreateAngleAttr(1.5)
    UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 20.0, 35.0))
    UsdLux.DomeLight.Define(stage, "/World/Sky").CreateIntensityAttr(250.0)

    paths = {"outlet": _box(stage, "/World/Outlet", (-0.46, 0.25, 0.02), (0.03, 0.03, 0.02), metal)}
    ring = (0.25, 0.0, 0.12)
    for name, off, half in (("top", (0, 0, 0.035), (0.04, 0.012, 0.005)), ("bottom", (0, 0, -0.035), (0.04, 0.012, 0.005)),
                            ("left", (-0.035, 0, 0), (0.005, 0.012, 0.04)), ("right", (0.035, 0, 0), (0.005, 0.012, 0.04))):
        paths[f"ring_{name}"] = _box(stage, f"/World/Ring/{name.capitalize()}",
                                     [r + o for r, o in zip(ring, off)], half, metal)
    _box(stage, "/World/Ring/Post", (ring[0], ring[1], ring[2] - 0.08), (0.006, 0.006, 0.04), metal)
    panel = (0.30, -0.262, 0.12)
    for name, off, half in (("upper", (0, 0, 0.0155), (0.13, 0.012, 0.0065)), ("lower", (0, 0, -0.0155), (0.13, 0.012, 0.0065)),
                            ("left", (-0.0715, 0, 0), (0.0585, 0.012, 0.009)), ("right", (0.0715, 0, 0), (0.0585, 0.012, 0.009))):
        paths[f"panel_{name}"] = _box(stage, f"/World/Panel/{name.capitalize()}",
                                      [p + o for p, o in zip(panel, off)], half, panel_black)
    for i, (dx, color) in enumerate(((-0.07, (0.85, 0.55, 0.15)), (-0.035, (0.18, 0.45, 0.85)),
                                     (0.035, (0.18, 0.45, 0.85)), (0.07, (0.2, 0.7, 0.35)))):
        _box(stage, f"/World/Panel/Port{i}", (panel[0] + dx, panel[1] + 0.0125, panel[2]), (0.0085, 0.001, 0.0075),
             _material(stage, f"Port{i}", color))
    for i, dx in enumerate((-0.12, 0.12)):
        _box(stage, f"/World/Panel/Leg{i}", (panel[0] + dx, panel[1], panel[2] - 0.071), (0.006, 0.01, 0.049), metal)

    gripper = UsdGeom.Xform.Define(stage, "/World/Gripper")
    gxf = UsdGeom.Xformable(gripper)
    gxf.AddTranslateOp().Set(Gf.Vec3d(plug_start[0], plug_start[1], 0.08))
    gxf.AddOrientOp().Set(Gf.Quatf(1.0))
    _box(stage, "/World/Gripper/Palm", (0, 0, 0.024), (0.02, 0.03, 0.008), metal)
    _box(stage, "/World/Gripper/FingerA", (0, 0.0095, 0.004), (0.01, 0.0025, 0.014), metal)
    _box(stage, "/World/Gripper/FingerB", (0, -0.0095, 0.004), (0.01, 0.0025, 0.014), metal)
    paths["grip"] = _box(stage, "/World/Gripper/Grip", (0, 0, 0), (0.001, 0.001, 0.001), visible=False)
    paths["gripper"] = "/World/Gripper"
    return paths
