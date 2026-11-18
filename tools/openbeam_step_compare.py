# SPDX-License-Identifier: GPL-3.0-or-later
"""Stack source OpenBeam video with both imported host views at the captured tick indices."""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont
import numpy as np
from openbeam_step_motion import Motion, file_sha256


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("scene","openbeam","mujoco","isaac-frames","mujoco-receipt","isaac-receipt","video","receipt"):
        p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--ffmpeg",default=shutil.which("ffmpeg"))
    p.add_argument("--font",help="optional TrueType font for labels")
    p.add_argument("--preview",type=Path)
    args=p.parse_args()
    if not args.ffmpeg:
        p.error("FFmpeg executable required")
    for path in (args.video,args.receipt,args.preview):
        if path and path.exists():
            raise FileExistsError(path)
    m=Motion(args.scene)
    hosts=[json.loads(p.read_text()) for p in (args.mujoco_receipt,args.isaac_receipt)]
    if [host["host"] for host in hosts] != ["MuJoCo","Isaac Sim"]:
        raise ValueError("expected a receipt from each imported host")
    for host in hosts:
        if (host["steps"] != m.count or host["frame_ticks_sha256"] != m.frames_hash()
                or host["rendered_frames"] != len(m.frames)
                or host["cable_position_sha256"] != m.cable["position_sha256"]
                or host["cable_velocity_sha256"] != m.cable["velocity_sha256"]):
            raise ValueError("host receipt differs from the source run")
    width,height=m.frames[0]["width"],m.frames[0]["height"]
    fps=1/(m.scene["frame_stride"]*m.scene["step_s"])
    fps_arg=format(fps,".12g")
    bar,footer=34,44
    size=(width,3*(height+bar)+footer)
    font=ImageFont.truetype(args.font,20) if args.font else ImageFont.load_default(size=20)
    small=ImageFont.truetype(args.font,17) if args.font else ImageFont.load_default(size=17)
    states=["Free","Approach","Guided","Latched","Releasing"]
    with tempfile.TemporaryDirectory(dir=args.video.parent) as tmp, ExitStack() as cleanup:
        staged=Path(tmp)/"video.mp4"
        readers=[]
        for src in (args.openbeam,args.mujoco):
            reader=subprocess.Popen([args.ffmpeg,"-v","error","-i",str(src),"-map","0:v:0","-threads","1",
                                     "-f","rawvideo","-pix_fmt","rgb24","pipe:1"],stdout=subprocess.PIPE)
            readers.append(reader)
            cleanup.callback(reader.wait)
            cleanup.callback(reader.kill)
            cleanup.callback(reader.stdout.close)
        encoder=subprocess.Popen([args.ffmpeg,"-v","error","-f","rawvideo","-pix_fmt","rgb24",
            "-s",f"{size[0]}x{size[1]}","-r",fps_arg,"-i","pipe:0","-an","-c:v","libx264","-preset","fast",
            "-crf","17","-pix_fmt","yuv420p","-movflags","+faststart","-n",str(staged)],stdin=subprocess.PIPE)
        cleanup.callback(encoder.wait)
        cleanup.callback(encoder.kill)
        cleanup.callback(encoder.stdin.close)
        for i,f in enumerate(m.frames):
            tick=f["tick"]
            panels=[]
            for reader in readers:
                raw=reader.stdout.read(width*height*3)
                if len(raw)!=width*height*3:
                    raise ValueError("source video ended before its final native step")
                panels.append(Image.frombytes("RGB",(width,height),raw))
            with Image.open(args.isaac_frames/(str(tick)+".png")) as im:
                if im.size!=(width,height):
                    raise ValueError("Isaac viewport size differs from the source camera")
                panels.append(im.convert("RGB"))
            canvas=Image.new("RGB",size,(20,27,33))
            draw=ImageDraw.Draw(canvas)
            labels=[("OpenBeam","Source recording"),("MuJoCo","OpenBeam motion"),("Isaac Sim","OpenBeam motion")]
            for j,(name,description) in enumerate(labels):
                y=j*(height+bar)
                draw.text((15,y+3),name,font=font,fill=(248,250,252))
                draw.text((width-15,y+6),description,font=small,fill=(189,206,216),anchor="ra")
                canvas.paste(panels[j],(0,y+bar))
            y=size[1]-footer+9
            draw.text((15,y),f"OpenBeam step {tick:,}     {tick*m.scene['step_s']:.3f} s",font=font,fill="white")
            state=int(m.plugs["state"][m.index(tick),0])
            draw.text((width-15,y),"LC connector   "+states[state],font=font,fill=(242,209,92),anchor="ra")
            if args.preview and i==len(m.frames)//2:
                preview=canvas.copy()
                preview.thumbnail((640,1200),Image.Resampling.LANCZOS)
                preview.save(Path(tmp)/"preview.jpg",quality=90)
            encoder.stdin.write(canvas.tobytes())
        encoder.stdin.close()
        if encoder.wait()!=0:
            raise RuntimeError("FFmpeg comparison encoding failed")
        for reader in readers:
            if reader.stdout.read(1) or reader.wait()!=0:
                raise ValueError("source video has frames outside the native index")
        audit=subprocess.run([args.ffmpeg,"-hide_banner","-i",str(staged),"-map","0:v:0","-f","null","-"],
                             check=True,capture_output=True,text=True,encoding="utf8",errors="replace")
        count=int(re.findall(r"frame=\s*(\d+)",audit.stderr)[-1])
        if count!=len(m.frames):
            raise ValueError("encoded frame count differs from the native camera index")
        transitions=[]
        state=m.plugs["state"][:,0]
        for i in np.flatnonzero(state[1:]!=state[:-1])+1:
            transitions.append(dict(tick=int(m.first+i),state=states[state[i]]))
        receipt=dict(schema="openbeam.native-motion-comparison/v1",source=m.scene["source"],first_tick=m.first,
            last_tick=m.last,video_first_tick=m.frames[0]["tick"],video_last_tick=m.frames[-1]["tick"],
            steps=m.count,frames=count,fps=fps,duration_s=count/fps,node_count=len(m.indices),
            rigid_links=len(m.links),largest_link_fit_rms_m=max(l["fit_rms_m"] for l in m.scene["links"]),
            video_sha256=file_sha256(staged),frame_ticks_sha256=m.frames_hash(),connector_transitions=transitions,
            sources={h["host"]:h for h in hosts},vertical_order=["OpenBeam","MuJoCo","Isaac Sim"])
        os.link(staged,args.video)
        if args.preview:
            os.link(Path(tmp)/"preview.jpg",args.preview)
        with args.receipt.open("x") as f:
            json.dump(receipt,f,indent=2)


if __name__=="__main__":
    main()
