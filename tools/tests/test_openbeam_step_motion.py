"""Verify fractional clock, camera axes and unmodified native measurements in MuJoCo."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from openbeam_step_motion import Motion


def save_scene(root):
    nodes=np.arange(3*3*6,dtype="<f4").reshape(3,3,6)/91
    p=np.zeros((3,1,3))
    p[:,0,2]=[.1,.2,.23]
    q=np.tile([1.,0.,0.,0.],(3,1,1))
    strand=np.zeros(3,dtype=[("state","<i4"),("contacts","<i4"),("tension","<f4"),
                             ("stretch","<f4"),("reactions","<f4",(2,3))])
    strand["contacts"]=[2,0,1]
    plugs=np.zeros((3,1),dtype=[("state","<i4"),("socket","<i4"),("port","<i4"),
                                ("depth","<f4"),("axial_force","<f4")])
    for name,a in dict(nodes_lead=nodes,link_positions=p,link_orientations=q,strand=strand,
                       plugs=plugs,grips=np.zeros((3,1),"<i4")).items():
        np.save(root/(name+".npy"),a)
    frames=[dict(tick=i,width=64,height=48,camera_pose=dict(position=[1,2,3],orientation=[0,0,0,1]),
                 intrinsics=dict(fov_y_deg=45)) for i in (9,11)]
    def digest(x):return hashlib.sha256(np.ascontiguousarray(x)).hexdigest()
    meta=dict(schema="openbeam.native-motion/v1",step_s=.0005,first_tick=9,last_tick=11,
              frame_stride=2,frames=frames,links=[dict(name="tool")],actors={"lead":{}},meshes=[],
              cable=dict(indices=[0,1,2],radius_m=.002,position_sha256=digest(nodes[:,:,:3]),
                         velocity_sha256=digest(nodes[:,:,3:])))
    (root/"scene.json").write_text(json.dumps(meta))
    return nodes


def test_measured_steps_and_camera(tmp_path):
    save_scene(tmp_path)
    m=Motion(tmp_path)
    p,q,fov=m.camera(m.frames[0])
    np.testing.assert_array_equal(p,[3,1,2])
    np.testing.assert_allclose(q,[.5,.5,.5,.5])
    assert fov==45
    with pytest.raises(IndexError):m.index(12)
    with (tmp_path/"nodes_lead.npy").open("r+b") as f:
        f.seek(-4,2);f.write(b"\xff\xff\xff\x00")
    with pytest.raises(ValueError,match="differ"):Motion(tmp_path)


def test_mujoco_readback_between_camera_frames(tmp_path):
    pytest.importorskip("mujoco")
    from openbeam_step_mujoco import MujocoMotion
    nodes=save_scene(tmp_path)
    m=Motion(tmp_path)
    sim=MujocoMotion(m)
    for i,tick in enumerate(range(9,12)):
        sim.apply(tick)
        np.testing.assert_array_equal(sim.data.xpos[sim.node_ids],nodes[i,:,:3])
        np.testing.assert_array_equal(sim.data.userdata[-9:].reshape(3,3),nodes[i,:,3:])
        np.testing.assert_array_equal(sim.data.xpos[sim.link_ids],m.pos[i])
        assert sim.data.userdata[1]==m.strand["contacts"][i]
        assert sim.data.time==tick*.0005

def test_video_can_cover_only_part_of_full_motion(tmp_path):
    save_scene(tmp_path)
    p=tmp_path/"scene.json"
    scene=json.loads(p.read_text())
    scene["frames"][0]["tick"]=10
    scene["frame_stride"]=1
    p.write_text(json.dumps(scene))
    m=Motion(tmp_path)
    assert m.first==9 and m.frames[0]["tick"]==10 and len(m.cable_positions)==3
