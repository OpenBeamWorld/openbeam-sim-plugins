"""Play OpenBeam native physics steps in an Isaac Sim USD stage and capture RTX frames."""
import traceback
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from openbeam_step_motion import Motion, arguments


def mesh_arrays(path):
    vertices, indices = [], []
    for row in path.read_text().splitlines():
        fields = row.split()
        if fields[:1] == ["v"]:
            vertices.append([float(s) for s in fields[1:]])
        if fields[:1] == ["f"]:
            indices.extend(int(s)-1 for s in fields[1:])
    return np.array(vertices,np.float32),np.array(indices,np.int32)


class IsaacMotion:
    def __init__(self, m, stage):
        from pxr import Gf, Sdf, UsdGeom, UsdLux, Vt
        self.motion, self.stage = m, stage
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1)
        stage.SetFramesPerSecond(1/(m.scene["step_s"]*m.scene["frame_stride"]))
        stage.SetTimeCodesPerSecond(1/m.scene["step_s"])
        stage.SetStartTimeCode(m.first)
        stage.SetEndTimeCode(m.last)
        world = UsdGeom.Xform.Define(stage,"/World")
        stage.SetDefaultPrim(world.GetPrim())
        self.links = {}
        for name in m.links:
            body = UsdGeom.Xform.Define(stage, "/World/"+name)
            self.links[name] = body.AddTransformOp()
        for i, spec in enumerate(m.scene["meshes"]):
            mesh = UsdGeom.Mesh.Define(stage,"/World/"+spec["link"]+"/mesh_"+str(i))
            vertices, faces = mesh_arrays(m.mesh_path(spec))
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(faces)//3,3,np.int32)))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateDisplayColorAttr([Gf.Vec3f(*spec["rgba"][:3])])
        cable = UsdGeom.BasisCurves.Define(stage, "/World/LC_cable")
        cable.CreateTypeAttr("linear")
        cable.CreateCurveVertexCountsAttr([len(m.indices)])
        cable.CreateWidthsAttr([2*m.cable["radius_m"]])
        cable.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        cable.CreateDisplayColorAttr([Gf.Vec3f(.93,.74,.06)])
        self.points = cable.CreatePointsAttr()
        self.velocity = cable.CreateVelocitiesAttr()
        self.telemetry = cable.GetPrim().CreateAttribute("openbeam:physicsReadouts",Sdf.ValueTypeNames.DoubleArray)
        cam = UsdGeom.Camera.Define(stage,"/World/SourceCamera")
        self.camera = cam
        self.camera_op = cam.AddTransformOp()
        cam.CreateClippingRangeAttr(Gf.Vec2f(.008,100))
        UsdLux.DomeLight.Define(stage,"/World/Environment").CreateIntensityAttr(680)
        sun = UsdLux.DistantLight.Define(stage,"/World/KeyLight")
        sun.CreateIntensityAttr(1300)
        sun.AddRotateXYZOp().Set(Gf.Vec3f(24,-35,-25))

    @staticmethod
    def matrix(p,q):
        from pxr import Gf
        return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(float(q[0]),Gf.Vec3d(*q[1:]))),Gf.Vec3d(*p))

    def apply(self, tick, timecode, frame=None):
        from pxr import Vt
        m = self.motion
        i = m.index(tick)
        for n, op in enumerate(self.links.values()):
            op.Set(self.matrix(m.pos[i,n],m.quat[i,n]),timecode)
        self.points.Set(Vt.Vec3fArray.FromNumpy(m.cable_positions[i]),timecode)
        self.velocity.Set(Vt.Vec3fArray.FromNumpy(m.cable_velocities[i]),timecode)
        self.telemetry.Set(Vt.DoubleArray.FromNumpy(m.telemetry[i]),timecode)
        if frame:
            p,q,fov = m.camera(frame)
            self.camera_op.Set(self.matrix(p,q),timecode)
            aperture = 24.
            self.camera.GetVerticalApertureAttr().Set(aperture,timecode)
            self.camera.GetHorizontalApertureAttr().Set(aperture*frame["width"]/frame["height"],timecode)
            self.camera.GetFocalLengthAttr().Set(aperture/(2*np.tan(np.deg2rad(fov/2))),timecode)


def run(app, args, m):
    import omni.usd
    from pxr import Usd
    import omni.timeline
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    sim = IsaacMotion(m,stage)
    frames = {f["tick"]:f for f in m.frames}
    viewport = None
    if args.frames:
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        args.frames.mkdir(exist_ok=False)
        viewport = get_active_viewport()
        viewport.camera_path = "/World/SourceCamera"
        f = m.frames[0]
        viewport.set_texture_resolution((f["width"],f["height"]))
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    h, vel = hashlib.sha256(), hashlib.sha256()
    max_pos = 0.
    for tick in range(m.first,m.last+1):
        f = frames.get(tick)
        t = Usd.TimeCode(tick) if args.usd else Usd.TimeCode.Default()
        sim.apply(tick,t,f)
        h.update(np.asarray(sim.points.Get(t),dtype="<f4").tobytes())
        vel.update(np.asarray(sim.velocity.Get(t),dtype="<f4").tobytes())
        if not np.array_equal(np.asarray(sim.telemetry.Get(t)),m.telemetry[m.index(tick)]):
            raise ValueError("Isaac changed the imported telemetry at tick " + str(tick))
        if f:
            for j, op in enumerate(sim.links.values()):
                position = np.asarray(op.Get(t).ExtractTranslation())
                max_pos = max(max_pos,np.max(np.abs(position-m.pos[m.index(tick),j])))
        if f and viewport:
            if args.usd:
                timeline.set_current_time(tick*m.scene["step_s"])
            for _ in range(45 if tick == m.first else 3):
                app.update()
            dest = (args.frames/(str(tick)+".png")).resolve()
            capture_viewport_to_file(viewport,str(dest))
            deadline = time.monotonic()+70
            while time.monotonic()<deadline:
                app.update()
                try:
                    with dest.open("rb") as png:
                        png.seek(-12,2)
                        if png.read(8) == bytes.fromhex("0000000049454e44"):
                            break
                except (OSError,ValueError):
                    continue
            else:
                raise RuntimeError("Isaac viewport did not return frame " + str(tick))
        if f and m.index(tick) % (m.scene["frame_stride"]*50) == 0:
            print("Isaac imported tick",tick,flush=True)
    if h.hexdigest() != m.cable["position_sha256"] or vel.hexdigest() != m.cable["velocity_sha256"]:
        raise ValueError("Isaac readback differs from OpenBeam cable steps")
    if args.usd:
        if args.usd.exists() or not stage.GetRootLayer().Export(str(args.usd.resolve())):
            raise ValueError("could not create the animated USD")
    args.receipt.write_text(json.dumps(dict(host="Isaac Sim",steps=m.count,first_tick=m.first,last_tick=m.last,
        rendered_frames=len(frames) if viewport else 0,frame_ticks_sha256=m.frames_hash(),
        cable_position_sha256=h.hexdigest(),cable_velocity_sha256=vel.hexdigest(),
        max_video_link_translation_error_m=float(max_pos),telemetry_readback_steps=m.count),indent=2))


def bake_usd(m, destination):
    """Build a standalone time-sampled USD on the cloud exporter using OpenUSD."""
    from pxr import Usd
    import tempfile, os
    with tempfile.TemporaryDirectory(dir=destination.parent) as tmp:
        path = Path(tmp)/"motion.usdc"
        stage = Usd.Stage.CreateNew(str(path))
        stage.SetInterpolationType(Usd.InterpolationTypeHeld)
        sim = IsaacMotion(m,stage)
        sim.apply(m.first,Usd.TimeCode.Default(),m.frames[0])
        frames = {f["tick"]:f for f in m.frames}
        for tick in range(m.first,m.last+1):
            sim.apply(tick,Usd.TimeCode(tick),frames.get(tick))
        stage.GetRootLayer().Save()
        del sim,stage
        stage = Usd.Stage.Open(str(path))
        cable = stage.GetPrimAtPath("/World/LC_cable")
        points = cable.GetAttribute("points")
        velocities = cable.GetAttribute("velocities")
        forces = cable.GetAttribute("openbeam:physicsReadouts")
        if points.GetNumTimeSamples() != m.count:
            raise ValueError("incomplete baked USD step range")
        ph, vh = hashlib.sha256(), hashlib.sha256()
        for tick in range(m.first,m.last+1):
            ph.update(np.asarray(points.Get(tick), dtype="<f4").tobytes())
            vh.update(np.asarray(velocities.Get(tick), dtype="<f4").tobytes())
            if not np.array_equal(np.asarray(forces.Get(tick)),m.telemetry[m.index(tick)]):
                raise ValueError("USD telemetry differs at tick " + str(tick))
        if (ph.hexdigest(),vh.hexdigest()) != (m.cable["position_sha256"],m.cable["velocity_sha256"]):
            raise ValueError("baked USD cable samples differ from OpenBeam")
        del points,velocities,forces,cable,stage
        os.link(path,destination)
    return dict(host="OpenUSD",version=Usd.GetVersion(),steps=m.count,
                first_tick=m.first,last_tick=m.last,cable_position_sha256=ph.hexdigest(),
                cable_velocity_sha256=vh.hexdigest(),telemetry_readback_steps=m.count)

def main():
    p = arguments(__doc__)
    p.add_argument("--experience",type=Path,help="Isaac Kit experience on this installation")
    p.add_argument("--usd",type=Path,help="write a native USD with every captured step as a time sample")
    p.add_argument("--bake-usd",type=Path,help="bake and verify USD using the OpenUSD Python runtime")
    args, _ = p.parse_known_args()
    motion = Motion(args.scene)
    if args.bake_usd:
        if args.frames or args.usd:
            p.error("--bake-usd is a standalone export")
        args.receipt.write_text(json.dumps(bake_usd(motion,args.bake_usd),indent=2))
        return
    from isaacsim import SimulationApp
    exp = args.experience or Path(__file__).with_name("openbeam.isaacsim.viewer.kit")
    f = motion.frames[0]
    app = SimulationApp({"headless":True,"width":f["width"],"height":f["height"],
                         "renderer":"RayTracedLighting"},experience=str(exp.resolve()) if exp.exists() else "")
    try:
        run(app,args,motion)
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    main()


