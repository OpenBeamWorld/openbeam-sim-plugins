# SPDX-License-Identifier: GPL-3.0-or-later
"""Prepare a LeRobot viewer episode with the OpenBeam physics timeline."""
import argparse
from pathlib import Path
import json
import os
import tempfile

import numpy as np
from openbeam_step_motion import file_sha256


def write_viewer(dataset_root, repo_id, destination):
    from lerobot.datasets import LeRobotDataset
    from lerobot.scripts.lerobot_dataset_viz import visualize_dataset
    import rerun as rr
    import rerun.blueprint as rrb

    if destination.exists():
        raise FileExistsError(destination)
    record = json.loads((dataset_root / "openbeam/motion.json").read_text(encoding="utf-8"))
    physics = dataset_root / "openbeam/physics.npz"
    if record["schema"] != "openbeam.lerobot-native-motion/v1" or file_sha256(physics) != record["physics_sha256"]:
        raise ValueError("Dataset physics differs from its OpenBeam receipt")
    dataset = LeRobotDataset(repo_id, root=dataset_root, video_backend="pyav")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as temp, np.load(physics, allow_pickle=False) as samples:
        camera = samples["frame_ticks"]
        ticks = np.arange(record["first_tick"], record["last_tick"] + 1, dtype=np.int64)
        if len(camera) != len(dataset) or len(ticks) != len(samples["strand"]):
            raise ValueError("Incomplete OpenBeam physics or camera sequence")
        recording = visualize_dataset(dataset, episode_index=0, save=True, output_dir=Path(temp),
                                      num_workers=0, batch_size=8, display_compressed_images=True)
        try:
            # Outside the camera interval the native step timeline is still available.
            # On the clip's timestamp timeline the first camera frame is time zero.
            start = int(camera[0] - ticks[0])
            stop = int(camera[-1] - ticks[0] + 1)
            tension = samples["strand"]["tension"]
            forces = samples["plugs"]["axial_force"]
            for first, end, in_video in ((0, start, False), (start, stop, True), (stop, len(ticks), False)):
                if first == end:
                    continue
                batch = slice(first, end)
                indexes = [rr.TimeColumn("openbeam_step", sequence=ticks[batch])]
                if in_video:
                    seconds = (ticks[batch] - camera[0]) * record["step_s"]
                    indexes.append(rr.TimeColumn("timestamp", timestamp=seconds))
                rr.send_columns("cable/tension", indexes, rr.Scalars.columns(scalars=tension[batch]), strict=True)
                for plug_index in range(forces.shape[1]):
                    rr.send_columns(f"connector/plug_{plug_index + 1}", indexes,
                                    rr.Scalars.columns(scalars=forces[batch, plug_index]), strict=True)
            rr.send_blueprint(rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial2DView(origin=dataset.meta.camera_keys[0], name="OpenBeam camera"),
                    rrb.Vertical(rrb.TimeSeriesView(origin="/cable", name="Cable tension (N)"),
                                 rrb.TimeSeriesView(origin="/connector", name="Connector axial force (N)")),
                    column_shares=[3, 2]),
                rrb.TimePanel(timeline="timestamp", expanded=True)))
        finally:
            rr.disconnect()
        if recording is None or not recording.is_file():
            raise RuntimeError("LeRobot did not produce a viewer recording")
        os.link(recording, destination)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--repo-id", default="openbeam/native-motion")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    write_viewer(args.dataset, args.repo_id, args.out)
