# SPDX-License-Identifier: GPL-3.0-or-later
"""Import completed OpenBeam steps into MuJoCo. Render directly from the imported state."""
import hashlib
import json
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from pathlib import Path
from openbeam_step_motion import Motion, arguments


class MujocoMotion:
    def __init__(self, motion):
        self.motion = motion
        m = motion
        first = m.frames[0]
        xml = ET.Element("mujoco", model="OpenBeam native motion")
        ET.SubElement(xml,"option", timestep=str(m.scene["step_s"]), gravity="0 0 0")
        ET.SubElement(xml,"size", nuserdata=str(m.telemetry.shape[1] + len(m.indices)*3))
        visual = ET.SubElement(xml, "visual")
        ET.SubElement(visual, "global", offwidth=str(first["width"]), offheight=str(first["height"]))
        ET.SubElement(visual, "map", znear=".001", zfar="60")
        ET.SubElement(visual,"headlight", ambient=".5 .5 .5",diffuse=".62 .62 .62",specular=".1 .1 .1")
        tex = ET.SubElement(xml, "asset")
        ET.SubElement(tex,"texture",type="skybox",builtin="gradient",rgb1=".35 .41 .46",
                      rgb2=".61 .65 .67",width="16",height="64")
        wb = ET.SubElement(xml, "worldbody")
        def vector(values):
            return " ".join(str(float(x)) for x in values)
        camera_pos, camera_quat, fov = m.camera(first)
        ET.SubElement(wb,"camera",name="source",pos=vector(camera_pos),quat=vector(camera_quat),fovy=str(fov))
        ET.SubElement(wb,"light",pos="0 1 5",dir="0 -.3 -1",directional="true",castshadow="false")
        bodies = {name:ET.SubElement(wb,"body",name=name,mocap="true",pos=vector(m.pos[0,i]),
                                     quat=vector(m.quat[0,i])) for i,name in enumerate(m.links)}
        assets = {}
        for i, mesh in enumerate(m.scene["meshes"]):
            path = m.mesh_path(mesh)
            assets[path.name] = path.read_bytes()
            name = "mesh_"+str(i)
            ET.SubElement(tex,"mesh",name=name,file=path.name)
            ET.SubElement(bodies[mesh["link"]],"geom",type="mesh",mesh=name,contype="0",conaffinity="0",
                          rgba=" ".join(map(str,mesh["rgba"])))
        self.node_names = ["cord_node_"+str(i) for i in range(len(m.indices))]
        for i,name in enumerate(self.node_names):
            b = ET.SubElement(wb,"body",name=name,mocap="true",pos=vector(m.cable_positions[0,i]))
            ET.SubElement(b,"site",name=name,size=".0001",rgba="0 0 0 0")
        tendons = ET.SubElement(xml,"tendon")
        for a,b in zip(self.node_names,self.node_names[1:]):
            t = ET.SubElement(tendons,"spatial",width=str(m.cable["radius_m"]),rgba=".93 .74 .06 1")
            ET.SubElement(t,"site",site=a)
            ET.SubElement(t,"site",site=b)
        ET.indent(xml)
        self.xml = ET.tostring(xml, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(self.xml, assets)
        self.data = mujoco.MjData(self.model)
        self.link_ids = [self.model.body(n).id for n in m.links]
        self.node_ids = [self.model.body(n).id for n in self.node_names]
        self.link_mocap = self.model.body_mocapid[self.link_ids]
        self.node_mocap = self.model.body_mocapid[self.node_ids]

    def apply(self, tick, frame=None):
        m, d = self.motion, self.data
        i = m.index(tick)
        d.mocap_pos[self.link_mocap] = m.pos[i]
        d.mocap_quat[self.link_mocap] = m.quat[i]
        d.mocap_pos[self.node_mocap] = m.cable_positions[i]
        n = m.telemetry.shape[1]
        d.userdata[:n] = m.telemetry[i]
        d.userdata[n:] = m.cable_velocities[i].ravel()
        d.time = tick * m.scene["step_s"]
        if frame:
            pos, quat, fov = m.camera(frame)
            self.model.cam_pos[0], self.model.cam_quat[0], self.model.cam_fovy[0] = pos, quat, fov
        mujoco.mj_forward(self.model, d)


def main():
    p = arguments(__doc__)
    p.add_argument("--mjcf", type=Path, help="save the measured initial scene beside its mesh assets")
    args = p.parse_args()
    m = Motion(args.scene)
    sim = MujocoMotion(m)
    if args.mjcf:
        if args.mjcf.resolve().parent != m.root:
            raise ValueError("save MJCF beside the extracted scene meshes")
        with args.mjcf.open("x") as stream:
            stream.write(sim.xml)
    frames = {f["tick"]:f for f in m.frames}
    renderer = None
    if args.frames:
        from PIL import Image
        args.frames.mkdir(exist_ok=False)
        f = m.frames[0]
        renderer = mujoco.Renderer(sim.model,height=f["height"],width=f["width"])
    h = hashlib.sha256()
    vel = hashlib.sha256()
    max_pos = 0.
    max_rot = 0.
    for tick in range(m.first, m.last+1):
        f = frames.get(tick)
        sim.apply(tick, f)
        i = m.index(tick)
        h.update(sim.data.xpos[sim.node_ids].astype("<f4").tobytes())
        vel.update(sim.data.userdata[m.telemetry.shape[1]:].astype("<f4").tobytes())
        max_pos = max(max_pos, np.max(np.abs(sim.data.xpos[sim.link_ids] - m.pos[i])))
        q = sim.data.xquat[sim.link_ids]
        expected = m.quat[i]
        max_rot = max(max_rot,float(np.minimum(np.linalg.norm(q-expected,axis=1),
                                                np.linalg.norm(q+expected,axis=1)).max()))
        if not np.array_equal(sim.data.userdata[:m.telemetry.shape[1]], m.telemetry[i]):
            raise ValueError("MuJoCo changed imported telemetry at tick " + str(tick))
        if renderer and f:
            renderer.update_scene(sim.data, camera="source")
            Image.fromarray(renderer.render()).save(args.frames/(str(tick)+".png"))
    if renderer:
        renderer.close()
    if h.hexdigest() != m.cable["position_sha256"] or vel.hexdigest() != m.cable["velocity_sha256"]:
        raise ValueError("MuJoCo readback differs from OpenBeam cable steps")
    args.receipt.write_text(json.dumps(dict(host="MuJoCo",version=mujoco.__version__,steps=m.count,
        first_tick=m.first,last_tick=m.last,rendered_frames=len(frames) if renderer else 0,
        frame_ticks_sha256=m.frames_hash(),cable_position_sha256=h.hexdigest(),cable_velocity_sha256=vel.hexdigest(),
        max_link_position_error_m=float(max_pos),max_link_quaternion_error=float(max_rot),telemetry_readback_steps=m.count),indent=2))


if __name__ == "__main__":
    main()
