# SPDX-License-Identifier: GPL-3.0-or-later
"""Export a native OpenBeam recording as a local LeRobotDataset with its full-rate physics."""
import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np

from openbeam_step_motion import Motion, file_sha256


CAMERA = "observation.images.openbeam"
TICK = "openbeam.tick"


def write_physics(motion, root):
    path = root / "openbeam" / "physics.npz"
    path.parent.mkdir()
    arrays = {
        "cable_position": motion.cable_positions,
        "cable_velocity": motion.cable_velocities,
        "link_position": motion.pos,
        "link_orientation": motion.quat,
        "strand": motion.strand,
        "plugs": motion.plugs,
        "grips": motion.grips,
        "frame_ticks": np.array([f["tick"] for f in motion.frames], dtype="<i8"),
    }
    np.savez_compressed(path, **arrays)
    with np.load(path, allow_pickle=False) as saved:
        for key, original in arrays.items():
            if not np.array_equal(saved[key], original):
                raise ValueError(f"Physics archive differs from OpenBeam for {key}")
        for name in ("position", "velocity"):
            digest = hashlib.sha256(saved["cable_" + name].tobytes()).hexdigest()
            if digest != motion.cable[name + "_sha256"]:
                raise ValueError(f"Cable {name} differs from the source recording")
    return path


def vector(names, dtype="float32"):
    return {"dtype": dtype, "shape": (len(names),), "names": names}


def export(scene, video, output, repo_id, task, video_sha256=None):
    import av
    from lerobot.datasets import LeRobotDataset

    motion = Motion(scene)
    interval = Fraction(str(motion.scene["step_s"])) * motion.scene["frame_stride"]
    if (1 / interval).denominator != 1:
        raise ValueError("LeRobot requires an integer camera frame rate")
    fps = int(1 / interval)
    video_digest = file_sha256(video)
    if video_sha256 and video_digest != video_sha256:
        raise ValueError("Source video SHA-256 does not match the supplied recording")
    robot = [i for i, link in enumerate(motion.scene["links"]) if link.get("actor") == "robot"]
    if not robot:
        raise ValueError("No measured robot links in the scene")
    nodes = motion.cable["node_names"]
    first = motion.frames[0]
    height, width = first["height"], first["width"]
    link_axes = [f"{motion.links[i]}.{axis}_m" for i in robot for axis in "xyz"]
    cable_axes = [f"{name}.{axis}" for name in nodes for axis in "xyz"]
    features = {
        CAMERA: {"dtype": "image", "shape": (height, width, 3), "names": ["height", "width", "channels"]},
        "observation.state": vector(link_axes),
        TICK: vector(["completed_physics_step"], "int64"),
        "openbeam.cable.position": vector([axis + "_m" for axis in cable_axes]),
        "openbeam.cable.velocity": vector([axis + "_m_s" for axis in cable_axes]),
    }
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as tmp:
        staged = Path(tmp) / "dataset"
        dataset = LeRobotDataset.create(repo_id, root=staged, fps=fps, features=features, use_videos=False)
        image_digest = hashlib.sha256()
        with av.open(str(video)) as movie:
            stream = movie.streams.video[0]
            if stream.average_rate != fps or (stream.width, stream.height) != (width, height):
                raise ValueError("The video size or rate differs from the registered OpenBeam camera")
            origin = None
            for index, (frame, camera) in enumerate(zip(movie.decode(stream), motion.frames, strict=True)):
                if frame.pts is None:
                    raise ValueError("Source video is missing a presentation timestamp")
                stamp = frame.pts * frame.time_base
                if origin is None:
                    origin = stamp
                if abs(stamp - origin - index * interval) > frame.time_base / 2:
                    raise ValueError(f"Source video clock differs at camera frame {index}")
                if (frame.width, frame.height) != (camera["width"], camera["height"]):
                    raise ValueError(f"Source video size differs at camera frame {index}")
                rgb = frame.to_ndarray(format="rgb24")
                image_digest.update(rgb.tobytes())
                step = motion.index(camera["tick"])
                dataset.add_frame({
                    CAMERA: rgb,
                    "observation.state": motion.pos[step, robot].astype("float32").ravel(),
                    TICK: np.array([camera["tick"]], dtype="int64"),
                    "openbeam.cable.position": motion.cable_positions[step].ravel(),
                    "openbeam.cable.velocity": motion.cable_velocities[step].ravel(),
                    "task": task,
                })
        dataset.save_episode()
        dataset.finalize()
        sidecar = write_physics(motion, staged)
        manifest = {
            "schema": "openbeam.lerobot-native-motion/v1", "physics": "openbeam/physics.npz",
            "physics_sha256": file_sha256(sidecar), "first_tick": motion.first, "last_tick": motion.last,
            "steps": motion.count, "step_s": motion.scene["step_s"], "frame_stride": motion.scene["frame_stride"],
            "frames": len(motion.frames), "fps": fps, "frame_ticks_sha256": motion.frames_hash(),
            "cable": motion.cable, "links": motion.links, "frame": motion.scene["frame"],
            "quaternion": motion.scene["quaternion"], "source": motion.scene["source"],
            "video_sha256": video_digest, "decoded_rgb_sha256": image_digest.hexdigest(),
        }
        (sidecar.parent / "motion.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        staged.rename(output)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True, help="Extracted OpenBeam native-motion scene")
    parser.add_argument("--video", type=Path, required=True, help="Original registered OpenBeam camera MP4")
    parser.add_argument("--video-sha256", help="Verify the camera clip against its release receipt")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repo-id", default="openbeam/native-motion", help="Local LeRobot dataset identifier")
    parser.add_argument("--task", required=True, help="Task description for this episode")
    args = parser.parse_args()
    info = export(args.scene, args.video, args.out, args.repo_id, args.task, args.video_sha256)
    print(f"Exported {info['frames']} camera frames and {info['steps']} physics steps to {args.out}")


if __name__ == "__main__":
    main()
