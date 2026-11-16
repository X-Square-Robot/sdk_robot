"""Convert Python SDK collector episodes for quanta_x3 to LeRobot (joint21).

Actions are the collector's next-observed-state labels, NOT command recordings.
No robot connection, upload, interpolation, model fallback or zero filling.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

PARTS = {
    "left_arm": [f"left_arm_joint{i}" for i in range(1, 8)],
    "right_arm": [f"right_arm_joint{i}" for i in range(1, 8)],
    "left_gripper": ["left_arm_gripper"],
    "right_gripper": ["right_arm_gripper"],
    "waist": ["lift_joint", "waist_pitch_joint", "waist_yaw_joint"],
    "head": ["head_pitch_joint", "head_yaw_joint"],
}
JOINT_NAMES = [name for names in PARTS.values() for name in names]
CAMERA_ALIASES = {
    "head_left_eye_stream": "head_left_eye_camera",
    "head_right_eye_stream": "head_right_eye_camera",
    "left_arm_rgb_stream": "left_arm_camera",
    "right_arm_rgb_stream": "right_arm_camera",
}


def read_json(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def local_path(root, relative):
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise ValueError(f"Expected a relative input path: {relative!r}")
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"Input path escapes dataset: {relative}")
    return path


def joint_vector(frame, episode, action=False):
    vector = []
    section = frame["action" if action else "observation"]
    for part, expected in PARTS.items():
        names = episode["joint_names"][part]
        if len(names) != len(expected) or set(names) != set(expected):
            raise ValueError(f"Invalid {part} joint_names: {names}")
        key = part + ("_actions" if action else "_joint_states")
        positions = section[key]["positions"]
        if len(positions) != len(names):
            raise ValueError(f"Invalid {key} dimension")
        for name in expected:
            value = positions[names.index(name)]
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f"Non-finite/non-numeric {key}.{name}")
            vector.append(float(value))
    return vector


class Images:
    def __init__(self, directory, episode, cameras):
        self.directory, self.episode, self.cameras = directory, episode, cameras
        self.captures = {}

    def read(self, frame, camera):
        import cv2
        reference = frame["images"][camera]
        if self.episode.get("storage_format", "images") == "video":
            if isinstance(reference, bool) or not isinstance(reference, int) or reference < 0:
                raise ValueError(f"Invalid video index for {camera}: {reference}")
            if camera not in self.captures:
                path = local_path(self.directory, self.episode["video_files"][camera])
                capture = cv2.VideoCapture(str(path))
                self.captures[camera] = capture
                if not capture.isOpened():
                    raise ValueError(f"Cannot open video: {path}")
            capture = self.captures[camera]
            capture.set(cv2.CAP_PROP_POS_FRAMES, reference)
            ok, image = capture.read()
            if not ok:
                raise ValueError(f"Cannot decode {camera} frame {reference}")
        else:
            path = local_path(self.directory, reference)
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError(f"Cannot decode image: {path}")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def close(self):
        for capture in self.captures.values():
            capture.release()


def inspect_dataset(input_dir):
    """Validate every selected field and image before creating output."""
    metadata = read_json(input_dir / "dataset_metadata.json")
    fps = metadata["fps"]
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0 or int(fps) != fps:
        raise ValueError("LeRobot export requires a positive integer source fps")
    entries = metadata["episodes"]
    if not entries:
        raise ValueError("Dataset has no episodes")
    cameras, shapes, paths, frame_count = None, {}, [], 0
    for entry in entries:
        directory = local_path(input_dir, entry.get("path", f"episode_{entry['episode_id']:04d}"))
        if directory in paths:
            raise ValueError("Duplicate episode path")
        paths.append(directory)
        episode = read_json(directory / "episode.json")
        if episode.get("model") != "quanta_x3" or entry.get("model") != "quanta_x3":
            raise ValueError("Only Python SDK quanta_x3 episodes are supported")
        frames = episode["frames"]
        if not frames or episode.get("num_frames") != len(frames):
            raise ValueError("Empty episode or num_frames mismatch")
        if not isinstance(episode.get("task"), str) or not episode["task"].strip():
            raise ValueError("Episode task must be a nonempty string")
        if episode.get("storage_format", "images") not in ("images", "video"):
            raise ValueError("Unknown storage_format")
        episode_cameras = sorted(frames[0]["images"])
        if cameras is None:
            cameras = episode_cameras
        if episode_cameras != cameras:
            raise ValueError("Camera set changed between episodes")
        if not cameras or {CAMERA_ALIASES.get(c, c) for c in cameras} != {CAMERA_ALIASES.get(c, c) for c in metadata["camera_names"]}:
            raise ValueError("Camera set does not match dataset_metadata.json")
        reader = Images(directory, episode, cameras)
        previous = None
        try:
            for frame in frames:
                stamp = frame["timestamp"]
                if isinstance(stamp, bool) or not isinstance(stamp, (float, int)) or not math.isfinite(stamp) or (previous is not None and stamp <= previous):
                    raise ValueError("Source timestamps must be finite and strictly increasing")
                if previous is not None and not math.isclose(stamp - previous, 1 / fps, rel_tol=0.1, abs_tol=0.001):
                    raise ValueError("Frames are not on the source FPS grid; resample explicitly before export")
                previous = stamp
                joint_vector(frame, episode)
                joint_vector(frame, episode, action=True)
                if sorted(frame["images"]) != cameras:
                    raise ValueError("Missing or unexpected camera in frame")
                for camera in cameras:
                    shape = tuple(reader.read(frame, camera).shape)
                    if camera in shapes and shapes[camera] != shape:
                        raise ValueError(f"Image resolution changed for {camera}")
                    shapes[camera] = shape
                frame_count += 1
        finally:
            reader.close()
    return int(fps), cameras, shapes, paths, frame_count


def convert(input_dir, output_dir=None, repo_id=None, validate_only=False):
    fps, cameras, shapes, paths, count = inspect_dataset(input_dir)
    print(f"Validated {len(paths)} episodes, {count} frames, {fps} FPS, {len(cameras)} cameras, joint21")
    if validate_only:
        return
    if output_dir is None or not repo_id:
        raise ValueError("--output-dir and --repo-id are required for conversion")
    if output_dir.exists():
        raise ValueError("Output directory already exists; choose a new directory")
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = {
        name: {"dtype": "float32", "shape": (21,), "names": JOINT_NAMES}
        for name in ("observation.state", "action")
    }
    features["observation.source_timestamp"] = {"dtype": "float64", "shape": (1,), "names": ["seconds"]}
    for camera in cameras:
        height, width, channels = shapes[camera]
        features[f"observation.images.{camera}"] = {
            "dtype": "image", "shape": (channels, height, width),
            "names": ["channels", "height", "width"],
        }
    dataset = LeRobotDataset.create(repo_id=repo_id, root=output_dir, fps=fps,
                                   robot_type="quanta_x3", features=features, use_videos=False)
    for directory in paths:
        episode = read_json(directory / "episode.json")
        reader = Images(directory, episode, cameras)
        try:
            for frame in episode["frames"]:
                data = {
                    "observation.state": np.asarray(joint_vector(frame, episode), dtype=np.float32),
                    "action": np.asarray(joint_vector(frame, episode, True), dtype=np.float32),
                    "observation.source_timestamp": np.asarray([frame["timestamp"]], dtype=np.float64),
                    "task": episode["task"],
                }
                for camera in cameras:
                    data[f"observation.images.{camera}"] = reader.read(frame, camera)
                dataset.add_frame(data)
            dataset.save_episode()
        finally:
            reader.close()
    dataset.finalize()
    manifest = {
        "profile": "quanta_x3_joint21", "joint_names": JOINT_NAMES,
        "units": ["m" if name == "lift_joint" else "rad" for name in JOINT_NAMES],
        "action_semantics": "collector next-observed-state labels; not recorded commands",
        "terminal_action": "collector retains current state at final frame",
        "fps": fps, "episodes": len(paths), "frames": count,
        "input": str(input_dir.resolve()), "camera_names": cameras,
        "excluded_from_training_vector": ["end_pose", "odometry", "imu", "ultrasonic"],
    }
    (output_dir / "meta" / "sdk_conversion.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--repo-id")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    try:
        convert(args.input_dir, args.output_dir, args.repo_id, args.validate_only)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Conversion failed: {error}\n")


if __name__ == "__main__":
    main()
