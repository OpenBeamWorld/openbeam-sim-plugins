"""Read an extracted OpenBeam native-motion scene in either simulator."""
import hashlib
import json
from pathlib import Path

import numpy as np


def file_sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(4 << 20), b""):
            h.update(block)
    return h.hexdigest()


class Motion:
    def __init__(self, directory):
        self.root = Path(directory).resolve()
        self.scene = json.loads((self.root / "scene.json").read_text())
        s = self.scene
        if s["schema"] != "openbeam.native-motion/v1" or s["step_s"] != .0005:
            raise ValueError("unsupported OpenBeam motion clock or schema")
        self.first, self.last = s["first_tick"], s["last_tick"]
        self.count = self.last - self.first + 1
        self.frames = s["frames"]
        self.links = [link["name"] for link in s["links"]]
        if len(self.links) != len(set(self.links)):
            raise ValueError("ambiguous link name")
        self.cable = s["cable"]
        self.indices = self.cable["indices"]
        self.pos = self.array("link_positions")
        self.quat = self.array("link_orientations")
        self.nodes = {k:self.array("nodes_"+k) for k in s["actors"]}
        self.strand = self.array("strand")
        self.plugs = self.array("plugs")
        self.grips = self.array("grips")
        positions = self.nodes["lead"][:, self.indices, :3]
        velocities = self.nodes["lead"][:, self.indices, 3:]
        for array, key in ((positions, "position_sha256"), (velocities, "velocity_sha256")):
            if hashlib.sha256(np.ascontiguousarray(array)).hexdigest() != self.cable[key]:
                raise ValueError("cable samples differ from the source recording")
        if self.pos.shape != (self.count, len(self.links), 3) or self.quat.shape != (self.count,len(self.links),4):
            raise ValueError("incomplete rigid-link trajectory")
        if self.count < 2 or [f["tick"] for f in self.frames] != list(range(self.first, self.last+1, s["frame_stride"])):
            raise ValueError("invalid video step index")
        if self.frames[-1]["tick"] != self.last:
            raise ValueError("video and motion end on different ticks")
        self.cable_positions = np.ascontiguousarray(positions)
        self.cable_velocities = np.ascontiguousarray(velocities)
        # Flat source measurements are also exposed to the host. Each strand row is
        # state, contacts, max tension, stretch, end forces (xyz twice), followed
        # by plug rows (state, socket, port, depth, axial force) then grip states.
        p = self.plugs
        plug_values = np.stack([p[n] for n in p.dtype.names],axis=-1).reshape(self.count,-1)
        strand = self.strand
        self.telemetry = np.column_stack([strand["state"],strand["contacts"],strand["tension"],
                                          strand["stretch"],strand["reactions"].reshape(self.count,6),
                                          plug_values,self.grips])

    def array(self, name):
        a = np.load(self.root/(name+".npy"), mmap_mode="r", allow_pickle=False)
        if len(a) != self.count:
            raise ValueError("missing native steps in " + name)
        return a

    def mesh_path(self, mesh):
        path = (self.root/mesh["file"]).resolve()
        if path.parent != self.root or path.suffix != ".obj":
            raise ValueError("invalid scene mesh path")
        return path

    def index(self, tick):
        if not self.first <= tick <= self.last:
            raise IndexError("tick outside recorded motion")
        return tick - self.first

    def camera(self, frame):
        raw = frame["camera_pose"]
        p = raw["position"]
        if isinstance(p, dict):
            p = [p[k] for k in "xyz"]
        x,y,z,w = np.asarray(raw["orientation"], dtype=float)
        # OGRE and the hosts use a -Z looking camera with +Y up in camera space.
        # Quaternion for world conversion (x,y,z) -> (z,x,y) is (.5,.5,.5,.5).
        q = .5*np.array([w-x-y-z, w+x+z-y, w+y+x-z, w+z+y-x])
        return np.asarray(p)[[2,0,1]], q/np.linalg.norm(q), frame["intrinsics"]["fov_y_deg"]

    def frames_hash(self):
        return hashlib.sha256(np.array([f["tick"] for f in self.frames],dtype="<u8").tobytes()).hexdigest()


def arguments(description):
    import argparse
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--scene", type=Path, default=Path(__file__).resolve().parent,
                   help="extracted native-motion scene")
    p.add_argument("--frames", type=Path, help="write rendered frames to an empty directory")
    p.add_argument("--receipt", type=Path, required=True)
    return p
