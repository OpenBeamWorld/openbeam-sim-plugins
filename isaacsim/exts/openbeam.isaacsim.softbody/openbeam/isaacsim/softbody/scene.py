# SPDX-License-Identifier: GPL-3.0-or-later
"""The wrecking-ball demo scene, the same as the MuJoCo one
(mujoco/models/wrecking_ball.xml): a 300 kg PhysX ball on a
3 m revolute pendulum, released at 60 degrees, swings into OpenBeam's
JBeam truss crate."""

from __future__ import annotations

import math
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

# assets/, from isaacsim/exts/<ext>/openbeam/isaacsim/softbody/.
DEFAULT_CRATE = Path(__file__).resolve().parents[6] / "assets" / "truss_crate.jbeam"

PIVOT = Gf.Vec3f(0.0, 0.0, 3.6)
ARM = 3.0
RELEASE = math.radians(60.0)
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
