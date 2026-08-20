#!/usr/bin/env python3
"""
Convert multiple rosbag archives into one LeRobot V3 dataset package.

The script is intentionally a thin batch layer over convert_rosbag_to_lerobot.py:
each source bag is converted with the selected episode policy, then the
resulting datasets are merged into one V3-style package with explicit
task_index/task metadata.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import shlex
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import numpy as np
    import pyarrow as pa
    import pyarrow.parquet as pq
except ImportError as exc:  # Defer dependency errors so --help remains usable.
    np = None
    pa = None
    pq = None
    _RUNTIME_DEPENDENCY_ERROR: ImportError | None = exc
else:
    _RUNTIME_DEPENDENCY_ERROR = None

try:
    import yaml
except ImportError:  # pragma: no cover - JSON manifests still work.
    yaml = None


DEFAULT_JOBS = 1


@dataclass(frozen=True)
class FileSpec:
    path: Path


@dataclass(frozen=True)
class TaskSpec:
    task_index: int
    task_id: str
    task: str
    files: tuple[FileSpec, ...]


@dataclass(frozen=True)
class ConvertedEpisode:
    source_order: int
    task_index: int
    task_id: str
    task: str
    source_path: Path
    dataset_dir: Path


@dataclass(frozen=True)
class SourceWorkItem:
    source_order: int
    task_index: int
    file_index: int
    task_id: str
    task: str
    source_path: Path
    output_dir: Path
    repo_id: str
    log_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Batch convert multi-file/multi-task rosbag data into one LeRobot V3 dataset.",
    )
    parser.add_argument("--manifest", required=True, help="YAML or JSON manifest describing tasks and source files")
    parser.add_argument("--output-dir", required=True, help="Final LeRobot dataset output directory")
    parser.add_argument("--repo-id", default=None, help="Override dataset repo_id from the manifest")
    parser.add_argument(
        "--config",
        required=True,
        help="Robot topic/state/action conversion config used for each source file, "
        "e.g. config/quanta_x1/lerobot_v3_16d.yaml",
    )
    parser.add_argument("--fps", type=int, default=None, help="Override manifest fps")
    parser.add_argument("--resize-width", type=int, default=None, help="Override manifest resize width")
    parser.add_argument("--resize-height", type=int, default=None, help="Override manifest resize height")
    parser.add_argument("--video-codec", default="h264", choices=["libsvtav1", "h264", "hevc"])
    parser.add_argument("--video-backend", default=None, choices=["pyav", "opencv"])
    parser.add_argument(
        "--episode-policy",
        choices=["file", "step"],
        default=None,
        help="Episode policy for each source bag. Defaults to manifest dataset.episode_policy, or file.",
    )
    parser.add_argument("--max-frames-per-file", type=int, default=None, help="Optional frame limit per source file")
    parser.add_argument("--converter", default=None, help="Path to convert_rosbag_to_lerobot.py")
    parser.add_argument("--work-dir", default=None, help="Temporary conversion workspace")
    parser.add_argument(
        "--jobs",
        type=int,
        default=None,
        help=f"Number of source files to convert concurrently. Defaults to manifest dataset.jobs, then {DEFAULT_JOBS} (serial).",
    )
    parser.add_argument("--keep-temp", action="store_true", help="Keep per-file temporary LeRobot outputs")
    parser.add_argument(
        "--write-source-manifest",
        action="store_true",
        help="Write optional meta/source_manifest.json with task/file mapping. Disabled by default.",
    )
    return parser.parse_args()


def require_runtime_dependencies() -> None:
    if _RUNTIME_DEPENDENCY_ERROR is not None:
        raise ImportError(
            "Missing runtime dependency for LeRobot batch conversion. "
            "Install the package requirements first, for example: "
            "python3 -m pip install -r requirements.txt"
        ) from _RUNTIME_DEPENDENCY_ERROR


def read_manifest(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        if yaml is None:
            raise ImportError("PyYAML is required for YAML manifests. Use JSON or install pyyaml.")
        data = yaml.safe_load(text)
    if not isinstance(data, dict):
        raise ValueError(f"Manifest must contain an object: {path}")
    return data


def text_value(raw: Any, label: str) -> str:
    if raw is None:
        raise ValueError(f"Missing required manifest field: {label}")
    value = str(raw).strip()
    if not value:
        raise ValueError(f"Manifest field must not be empty: {label}")
    return value


def resolve_path(path: str, base_dir: Path) -> Path:
    candidate = Path(path).expanduser()
    if not candidate.is_absolute():
        candidate = base_dir / candidate
    return candidate.resolve()


def resolve_cli_path(path: str, repo_root: Path) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute():
        return candidate
    cwd_candidate = (Path.cwd() / candidate).resolve()
    if cwd_candidate.exists():
        return cwd_candidate
    return (repo_root / candidate).resolve()


def parse_manifest(path: Path) -> tuple[dict[str, Any], list[TaskSpec]]:
    raw = read_manifest(path)
    dataset = raw.get("dataset") or {}
    if not isinstance(dataset, dict):
        raise ValueError("manifest.dataset must be an object")

    raw_tasks = raw.get("tasks")
    if not isinstance(raw_tasks, list) or not raw_tasks:
        raise ValueError("manifest.tasks must be a non-empty list")

    tasks: list[TaskSpec] = []
    for task_index, raw_task in enumerate(raw_tasks):
        if not isinstance(raw_task, dict):
            raise ValueError(f"manifest.tasks[{task_index}] must be an object")
        task_id = str(raw_task.get("task_id") or raw_task.get("id") or f"task_{task_index:03d}")
        task = text_value(
            raw_task.get("task") or raw_task.get("task_desc") or raw_task.get("description"),
            f"tasks[{task_index}].task",
        )
        raw_files = raw_task.get("files") or raw_task.get("bags") or raw_task.get("bag_paths")
        if not isinstance(raw_files, list) or not raw_files:
            raise ValueError(f"tasks[{task_index}] must contain non-empty files/bags")

        files: list[FileSpec] = []
        for file_index, raw_file in enumerate(raw_files):
            if isinstance(raw_file, str):
                raw_path = raw_file
            elif isinstance(raw_file, dict):
                raw_path = text_value(raw_file.get("path") or raw_file.get("bag_path"), f"tasks[{task_index}].files[{file_index}].path")
            else:
                raise ValueError(f"tasks[{task_index}].files[{file_index}] must be a path string or object")
            source_path = resolve_path(raw_path, path.parent)
            if not source_path.exists():
                raise FileNotFoundError(f"Source bag path not found: {source_path}")
            files.append(FileSpec(source_path))

        tasks.append(TaskSpec(task_index=task_index, task_id=task_id, task=task, files=tuple(files)))

    return dataset, tasks


def ensure_empty_output(output_dir: Path) -> None:
    if output_dir.exists() and output_dir.is_file():
        raise FileExistsError(f"Output path is a file: {output_dir}")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"Output directory already exists and is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)


def validate_jobs(value: int) -> int:
    if value <= 0:
        raise ValueError("jobs must be a positive integer")
    return value


def prepare_work_dir_parent(raw_work_dir: str | None) -> Path | None:
    if raw_work_dir is None:
        return None
    work_dir = Path(raw_work_dir).expanduser().resolve()
    work_dir.mkdir(parents=True, exist_ok=True)
    return work_dir


def run_single_file_converter(
    *,
    python_bin: str,
    converter: Path,
    repo_root: Path,
    source_path: Path,
    output_dir: Path,
    repo_id: str,
    config: str,
    task: str,
    fps: int,
    resize_width: int,
    resize_height: int,
    video_codec: str,
    video_backend: str | None,
    max_frames: int | None,
    episode_policy: str,
    log_path: Path | None = None,
) -> None:
    cmd = [
        python_bin,
        str(converter),
        "--bag-path",
        str(source_path),
        "--output-dir",
        str(output_dir),
        "--repo-id",
        repo_id,
        "--config",
        config,
        "--task",
        task,
        "--episode-policy",
        episode_policy,
        "--fps",
        str(fps),
        "--resize-width",
        str(resize_width),
        "--resize-height",
        str(resize_height),
        "--use-videos",
        "--video-codec",
        video_codec,
    ]
    if video_backend:
        cmd.extend(["--video-backend", video_backend])
    if max_frames is not None:
        cmd.extend(["--max-frames", str(max_frames)])

    print(f"\n[batch] Converting source file: {source_path}")
    if log_path is None:
        subprocess.run(cmd, cwd=repo_root, check=True)
        return

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        log_file.write(f"$ {shlex.join(cmd)}\n\n")
        log_file.flush()
        subprocess.run(cmd, cwd=repo_root, stdout=log_file, stderr=subprocess.STDOUT, check=True)


def tail_text(path: Path, max_lines: int = 40) -> str:
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        return f"<failed to read log: {exc}>"
    return "\n".join(lines[-max_lines:])


def build_source_work_items(tasks: list[TaskSpec], repo_id: str, temp_dir: Path) -> list[SourceWorkItem]:
    logs_dir = temp_dir / "logs"
    work_items: list[SourceWorkItem] = []
    source_order = 0
    for task in tasks:
        for file_index, file_spec in enumerate(task.files):
            item_name = f"source-{source_order:03d}_task-{task.task_index:03d}_file-{file_index:03d}"
            work_items.append(
                SourceWorkItem(
                    source_order=source_order,
                    task_index=task.task_index,
                    file_index=file_index,
                    task_id=task.task_id,
                    task=task.task,
                    source_path=file_spec.path,
                    output_dir=temp_dir / item_name,
                    repo_id=part_repo_id(repo_id, task.task_index, file_index),
                    log_path=logs_dir / f"{item_name}.log",
                )
            )
            source_order += 1
    return work_items


def convert_source_work_item(
    item: SourceWorkItem,
    *,
    python_bin: str,
    converter: Path,
    repo_root: Path,
    config: str,
    fps: int,
    resize_width: int,
    resize_height: int,
    video_codec: str,
    video_backend: str | None,
    max_frames: int | None,
    episode_policy: str,
    log_to_file: bool,
) -> ConvertedEpisode:
    run_single_file_converter(
        python_bin=python_bin,
        converter=converter,
        repo_root=repo_root,
        source_path=item.source_path,
        output_dir=item.output_dir,
        repo_id=item.repo_id,
        config=config,
        task=item.task,
        fps=fps,
        resize_width=resize_width,
        resize_height=resize_height,
        video_codec=video_codec,
        video_backend=video_backend,
        max_frames=max_frames,
        episode_policy=episode_policy,
        log_path=item.log_path if log_to_file else None,
    )
    return ConvertedEpisode(
        source_order=item.source_order,
        task_index=item.task_index,
        task_id=item.task_id,
        task=item.task,
        source_path=item.source_path,
        dataset_dir=item.output_dir,
    )


def convert_source_work_items(
    work_items: list[SourceWorkItem],
    *,
    jobs: int,
    python_bin: str,
    converter: Path,
    repo_root: Path,
    config: str,
    fps: int,
    resize_width: int,
    resize_height: int,
    video_codec: str,
    video_backend: str | None,
    max_frames: int | None,
    episode_policy: str,
) -> list[ConvertedEpisode]:
    if not work_items:
        return []

    if jobs == 1:
        converted = [
            convert_source_work_item(
                item,
                python_bin=python_bin,
                converter=converter,
                repo_root=repo_root,
                config=config,
                fps=fps,
                resize_width=resize_width,
                resize_height=resize_height,
                video_codec=video_codec,
                video_backend=video_backend,
                max_frames=max_frames,
                episode_policy=episode_policy,
                log_to_file=False,
            )
            for item in work_items
        ]
        return sorted(converted, key=lambda item: item.source_order)

    max_workers = min(jobs, len(work_items))
    print(f"[batch] Converting {len(work_items)} source files with jobs={max_workers}")
    print(f"[batch] Per-source logs: {work_items[0].log_path.parent}")

    converted: list[ConvertedEpisode] = []
    failures: list[tuple[SourceWorkItem, BaseException]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = {
            executor.submit(
                convert_source_work_item,
                item,
                python_bin=python_bin,
                converter=converter,
                repo_root=repo_root,
                config=config,
                fps=fps,
                resize_width=resize_width,
                resize_height=resize_height,
                video_codec=video_codec,
                video_backend=video_backend,
                max_frames=max_frames,
                episode_policy=episode_policy,
                log_to_file=True,
            ): item
            for item in work_items
        }
        for future in concurrent.futures.as_completed(futures):
            item = futures[future]
            try:
                converted_item = future.result()
            except BaseException as exc:
                failures.append((item, exc))
                print(f"[batch] FAILED source {item.source_order}: {item.source_path}")
            else:
                converted.append(converted_item)
                print(f"[batch] DONE source {item.source_order}: {item.source_path}")

    if failures:
        messages = [
            "One or more source files failed during conversion. Final merge was not run.",
            "Use --keep-temp to preserve full per-source logs and intermediate outputs.",
        ]
        for item, exc in sorted(failures, key=lambda pair: pair[0].source_order):
            messages.append(
                "\n".join(
                    [
                        f"- source {item.source_order}: {item.source_path}",
                        f"  task_index={item.task_index}, file_index={item.file_index}",
                        f"  error={type(exc).__name__}: {exc}",
                        f"  log={item.log_path}",
                        "  log tail:",
                        tail_text(item.log_path),
                    ]
                )
            )
        raise RuntimeError("\n".join(messages))

    return sorted(converted, key=lambda item: item.source_order)


def first_parquet(root: Path, pattern: str) -> Path:
    matches = sorted(root.glob(pattern))
    if len(matches) != 1:
        raise FileNotFoundError(f"Expected exactly one parquet matching {pattern} under {root}, found {len(matches)}")
    return matches[0]


def vector_to_list(value: Any, width: int, column: str) -> list[float]:
    if isinstance(value, np.ndarray):
        array = value
    elif hasattr(value, "to_numpy"):
        array = value.to_numpy()
    elif hasattr(value, "as_py"):
        array = np.asarray(value.as_py())
    else:
        array = np.asarray(value)
    array = np.asarray(array, dtype=np.float32).reshape(-1)
    if array.size != width:
        raise ValueError(f"{column} vector width mismatch: expected {width}, got {array.size}")
    return array.tolist()


def cell_value(value: Any, default: Any = None) -> Any:
    if value is None:
        return default
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            pass
    if isinstance(value, float) and np.isnan(value):
        return default
    return value


def read_episode_rows(dataset_dir: Path) -> list[dict[str, Any]]:
    episodes_path = first_parquet(dataset_dir, "meta/episodes/chunk-*/file-*.parquet")
    pdf = pq.read_table(episodes_path).to_pandas()
    rows = [dict(row) for row in pdf.to_dict(orient="records")]
    rows.sort(key=lambda row: int(cell_value(row.get("episode_index"), 0)))
    return rows


def episode_task_text(source_row: dict[str, Any], fallback_task: str) -> str:
    tasks = source_row.get("tasks")
    if isinstance(tasks, np.ndarray):
        tasks = tasks.tolist()
    if isinstance(tasks, (list, tuple)) and tasks:
        value = str(tasks[0]).strip()
        if value:
            return value
    return fallback_task


def load_converted_episode_data(
    dataset_dir: Path,
    *,
    task_index: int,
    task: str,
    global_episode_index: int,
    global_start: int,
    width: int,
) -> tuple[list[pa.Table], list[dict[str, Any]], list[list[float]], list[list[float]], int, int]:
    data_path = first_parquet(dataset_dir, "data/chunk-*/file-*.parquet")
    table = pq.read_table(data_path)
    pdf = table.to_pandas()
    if len(pdf) <= 0:
        raise ValueError(f"Converted dataset has no rows: {dataset_dir}")

    tables: list[pa.Table] = []
    episode_rows: list[dict[str, Any]] = []
    all_state_vectors: list[list[float]] = []
    all_action_vectors: list[list[float]] = []
    next_episode_index = global_episode_index
    next_global_start = global_start

    source_episode_rows = read_episode_rows(dataset_dir)
    for source_row in source_episode_rows:
        source_episode_index = int(cell_value(source_row.get("episode_index"), 0))
        episode_pdf = pdf[pdf["episode_index"] == source_episode_index].copy()
        if episode_pdf.empty:
            raise ValueError(f"Episode {source_episode_index} has no data rows in {dataset_dir}")
        if "index" in episode_pdf:
            episode_pdf = episode_pdf.sort_values("index")

        row_count = len(episode_pdf)
        timestamps = (
            episode_pdf["timestamp"].astype(float).tolist()
            if "timestamp" in episode_pdf
            else [float(index) for index in range(row_count)]
        )
        state_vectors = [vector_to_list(value, width, "observation.state") for value in episode_pdf["observation.state"]]
        action_vectors = [vector_to_list(value, width, "action") for value in episode_pdf["action"]]

        tables.append(
            pa.table(
                {
                    "timestamp": pa.array(timestamps, type=pa.float64()),
                    "frame_index": pa.array(list(range(row_count)), type=pa.int64()),
                    "episode_index": pa.array([next_episode_index] * row_count, type=pa.int64()),
                    "index": pa.array(list(range(next_global_start, next_global_start + row_count)), type=pa.int64()),
                    "task_index": pa.array([task_index] * row_count, type=pa.int64()),
                    "observation.state": pa.array(state_vectors, type=pa.list_(pa.float32())),
                    "action": pa.array(action_vectors, type=pa.list_(pa.float32())),
                }
            )
        )
        episode_rows.append(
            {
                "episode_index": next_episode_index,
                "source_episode_index": source_episode_index,
                "task": task,
                "source_task": episode_task_text(source_row, task),
                "length": row_count,
                "dataset_from_index": next_global_start,
                "dataset_to_index": next_global_start + row_count,
                "from_timestamp": float(timestamps[0]),
                "to_timestamp": float(timestamps[-1]),
                "source_episode_row": source_row,
            }
        )
        all_state_vectors.extend(state_vectors)
        all_action_vectors.extend(action_vectors)
        next_global_start += row_count
        next_episode_index += 1

    return tables, episode_rows, all_state_vectors, all_action_vectors, next_episode_index, next_global_start


def feature_camera_keys(info: dict[str, Any]) -> list[str]:
    features = info.get("features") or {}
    if not isinstance(features, dict):
        return []
    return [
        key
        for key, value in features.items()
        if key.startswith("observation.images.") and isinstance(value, dict) and value.get("dtype") == "video"
    ]


def video_file_index(path: Path) -> int:
    prefix = "file-"
    if not path.stem.startswith(prefix):
        raise ValueError(f"Unexpected video file name: {path}")
    return int(path.stem[len(prefix):])


def copy_source_videos(
    source_dataset: Path,
    output_dir: Path,
    camera_keys: list[str],
    next_target_file_index: int,
) -> tuple[dict[int, int], int]:
    first_camera_dir = source_dataset / "videos" / camera_keys[0] / "chunk-000"
    local_indices = [video_file_index(path) for path in sorted(first_camera_dir.glob("file-*.mp4"))]
    if not local_indices:
        raise FileNotFoundError(f"No source videos found in {first_camera_dir}")

    file_index_map = {
        local_index: next_target_file_index + offset
        for offset, local_index in enumerate(local_indices)
    }

    for camera_key in camera_keys:
        source_dir = source_dataset / "videos" / camera_key / "chunk-000"
        source_files = {video_file_index(path): path for path in sorted(source_dir.glob("file-*.mp4"))}
        missing = [index for index in local_indices if index not in source_files]
        if missing:
            raise FileNotFoundError(f"Missing video indices for {camera_key} in {source_dir}: {missing}")
        target_dir = output_dir / "videos" / camera_key / "chunk-000"
        target_dir.mkdir(parents=True, exist_ok=True)
        for local_index, source_file in source_files.items():
            if local_index not in file_index_map:
                continue
            shutil.copy2(source_file, target_dir / f"file-{file_index_map[local_index]:03d}.mp4")

    return file_index_map, next_target_file_index + len(local_indices)


def write_tasks(output_dir: Path, tasks: list[TaskSpec]) -> None:
    table = pa.table(
        {
            "task_index": pa.array([task.task_index for task in tasks], type=pa.int64()),
            "task": pa.array([task.task for task in tasks], type=pa.string()),
        }
    )
    pq.write_table(table, output_dir / "meta" / "tasks.parquet")


def write_episodes(output_dir: Path, rows: list[dict[str, Any]], camera_keys: list[str]) -> None:
    columns: dict[str, Any] = {
        "episode_index": pa.array([row["episode_index"] for row in rows], type=pa.int64()),
        "tasks": pa.array([[row["task"]] for row in rows], type=pa.list_(pa.string())),
        "length": pa.array([row["length"] for row in rows], type=pa.int64()),
        "dataset_from_index": pa.array([row["dataset_from_index"] for row in rows], type=pa.int64()),
        "dataset_to_index": pa.array([row["dataset_to_index"] for row in rows], type=pa.int64()),
        "data/chunk_index": pa.array([0] * len(rows), type=pa.int64()),
        "data/file_index": pa.array([0] * len(rows), type=pa.int64()),
        "meta/episodes/chunk_index": pa.array([0] * len(rows), type=pa.int64()),
        "meta/episodes/file_index": pa.array([0] * len(rows), type=pa.int64()),
    }
    for camera_key in camera_keys:
        columns[f"videos/{camera_key}/chunk_index"] = pa.array([0] * len(rows), type=pa.int64())
        columns[f"videos/{camera_key}/file_index"] = pa.array(
            [row["videos"][camera_key]["file_index"] for row in rows],
            type=pa.int64(),
        )
        columns[f"videos/{camera_key}/from_timestamp"] = pa.array(
            [row["videos"][camera_key]["from_timestamp"] for row in rows],
            type=pa.float64(),
        )
        columns[f"videos/{camera_key}/to_timestamp"] = pa.array(
            [row["videos"][camera_key]["to_timestamp"] for row in rows],
            type=pa.float64(),
        )

    table = pa.table(columns)
    episode_dir = output_dir / "meta" / "episodes" / "chunk-000"
    episode_dir.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, episode_dir / "file-000.parquet")


def _stats_tree_to_lists(stats: dict[str, Any]) -> dict[str, Any]:
    return {
        key: {stat_name: np.asarray(value).tolist() for stat_name, value in feature.items()}
        for key, feature in stats.items()
    }


def aggregate_source_feature_stats(converted: list[ConvertedEpisode], key_prefix: str) -> dict[str, Any]:
    """Aggregate per-source meta/stats.json entries (e.g. image features) into merged stats.

    The merge below rewrites observation.state/action stats exactly from the merged
    frames, but image statistics can only come from the per-source datasets, so they
    are aggregated here (count-weighted mean/std, global min/max) instead of being
    dropped. Uses LeRobot's own aggregate_stats when available.
    """
    stats_list: list[dict[str, dict[str, Any]]] = []
    for converted_source in converted:
        stats_path = converted_source.dataset_dir / "meta" / "stats.json"
        if not stats_path.exists():
            continue
        source_stats = json.loads(stats_path.read_text(encoding="utf-8"))
        filtered = {
            key: {stat_name: np.asarray(value) for stat_name, value in feature.items()}
            for key, feature in source_stats.items()
            if key.startswith(key_prefix)
        }
        if filtered:
            stats_list.append(filtered)

    if not stats_list:
        return {}

    try:
        from lerobot.datasets.compute_stats import aggregate_stats  # type: ignore

        return _stats_tree_to_lists(aggregate_stats(stats_list))
    except ImportError:
        pass

    # Fallback: count-weighted aggregation (quantiles approximated by weighted mean).
    merged: dict[str, Any] = {}
    keys = {key for stats in stats_list for key in stats}
    for key in keys:
        features = [stats[key] for stats in stats_list if key in stats]
        counts = np.array([float(np.asarray(f["count"]).ravel()[0]) for f in features])
        weights = counts / counts.sum()
        means = np.stack([np.asarray(f["mean"], dtype=np.float64) for f in features])
        stds = np.stack([np.asarray(f["std"], dtype=np.float64) for f in features])
        mean = np.einsum("i,i...->...", weights, means)
        second_moment = np.einsum("i,i...->...", weights, stds**2 + means**2)
        merged_feature = {
            "count": [int(counts.sum())],
            "mean": mean.tolist(),
            "std": np.sqrt(np.maximum(0.0, second_moment - mean**2)).tolist(),
            "min": np.min(np.stack([np.asarray(f["min"], dtype=np.float64) for f in features]), axis=0).tolist(),
            "max": np.max(np.stack([np.asarray(f["max"], dtype=np.float64) for f in features]), axis=0).tolist(),
        }
        for quantile_name in ("q01", "q10", "q50", "q90", "q99"):
            if all(quantile_name in f for f in features):
                quantiles = np.stack([np.asarray(f[quantile_name], dtype=np.float64) for f in features])
                merged_feature[quantile_name] = np.einsum("i,i...->...", weights, quantiles).tolist()
        merged[key] = merged_feature
    return merged


def stats_for(vectors: list[list[float]]) -> dict[str, list[float]]:
    matrix = np.asarray(vectors, dtype=np.float32)
    return {
        "min": matrix.min(axis=0).astype(float).tolist(),
        "max": matrix.max(axis=0).astype(float).tolist(),
        "mean": matrix.mean(axis=0).astype(float).tolist(),
        "std": matrix.std(axis=0).astype(float).tolist(),
    }


def write_stats(
    output_dir: Path,
    state_vectors: list[list[float]],
    action_vectors: list[list[float]],
    extra_stats: dict[str, Any] | None = None,
) -> None:
    stats = dict(extra_stats or {})
    stats["observation.state"] = stats_for(state_vectors)
    stats["action"] = stats_for(action_vectors)
    (output_dir / "meta" / "stats.json").write_text(json.dumps(stats, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_info(
    output_dir: Path,
    base_info: dict[str, Any],
    *,
    repo_id: str,
    total_episodes: int,
    total_frames: int,
    total_tasks: int,
    camera_keys: list[str],
    fps: int,
    resize_width: int,
    resize_height: int,
    vector_width: int,
    total_video_files: int,
) -> None:
    info = dict(base_info)
    info["repo_id"] = repo_id
    info["fps"] = fps
    info["total_episodes"] = total_episodes
    info["total_frames"] = total_frames
    info["total_tasks"] = total_tasks
    info["total_videos"] = total_video_files * len(camera_keys)
    info["total_chunks"] = 1
    info["chunks_size"] = int(info.get("chunks_size") or 1000)
    info["splits"] = {"train": f"0:{total_episodes}"}
    info["data_path"] = "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
    info["video_path"] = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"

    features = dict(info.get("features") or {})
    for key in ("observation.state", "action"):
        feature = dict(features.get(key) or {})
        feature["dtype"] = feature.get("dtype") or "float32"
        feature["shape"] = [vector_width]
        features[key] = feature
    for camera_key in camera_keys:
        feature = dict(features.get(camera_key) or {})
        feature["dtype"] = "video"
        feature["shape"] = [3, resize_height, resize_width]
        video_info = dict(feature.get("info") or {})
        video_info["video.height"] = resize_height
        video_info["video.width"] = resize_width
        video_info["video.fps"] = fps
        feature["info"] = video_info
        features[camera_key] = feature
    info["features"] = features

    (output_dir / "meta" / "info.json").write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_conversion_manifest(
    output_dir: Path,
    *,
    manifest_path: Path,
    tasks: list[TaskSpec],
    converted: list[ConvertedEpisode],
    resize_width: int,
    resize_height: int,
    vector_width: int,
    episode_policy: str,
) -> None:
    payload = {
        "note": "Optional source file mapping generated by the batch converter. Source raw bags are not bundled in this LeRobot package.",
        "manifest": str(manifest_path),
        "episode_policy": episode_policy,
        "state_action_width": vector_width,
        "resize": {"width": resize_width, "height": resize_height},
        "tasks": [
            {
                "task_index": task.task_index,
                "task_id": task.task_id,
                "task": task.task,
                "files": [str(file_spec.path) for file_spec in task.files],
            }
            for task in tasks
        ],
        "source_files": [
            {
                "source_file_index": index,
                "task_index": episode.task_index,
                "task_id": episode.task_id,
                "task": episode.task,
                "source_path": str(episode.source_path),
            }
            for index, episode in enumerate(converted)
        ],
    }
    (output_dir / "meta" / "source_manifest.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def merge_converted_datasets(
    *,
    output_dir: Path,
    repo_id: str,
    manifest_path: Path,
    tasks: list[TaskSpec],
    converted: list[ConvertedEpisode],
    fps: int,
    resize_width: int,
    resize_height: int,
    vector_width: int,
    episode_policy: str,
    write_conversion_manifest_file: bool,
) -> None:
    (output_dir / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)
    (output_dir / "meta").mkdir(parents=True, exist_ok=True)

    base_info = json.loads((converted[0].dataset_dir / "meta" / "info.json").read_text(encoding="utf-8"))
    camera_keys = feature_camera_keys(base_info)

    tables: list[pa.Table] = []
    episode_rows: list[dict[str, Any]] = []
    all_state_vectors: list[list[float]] = []
    all_action_vectors: list[list[float]] = []
    global_start = 0
    global_episode_index = 0
    next_video_file_index = 0

    for converted_source in converted:
        video_file_map, next_video_file_index = copy_source_videos(
            converted_source.dataset_dir,
            output_dir,
            camera_keys,
            next_video_file_index,
        )
        source_tables, source_episode_rows, source_state_vectors, source_action_vectors, global_episode_index, global_start = load_converted_episode_data(
            converted_source.dataset_dir,
            task_index=converted_source.task_index,
            task=converted_source.task,
            global_episode_index=global_episode_index,
            global_start=global_start,
            width=vector_width,
        )
        tables.extend(source_tables)
        all_state_vectors.extend(source_state_vectors)
        all_action_vectors.extend(source_action_vectors)

        for row in source_episode_rows:
            source_row = row.pop("source_episode_row")
            videos: dict[str, dict[str, float | int]] = {}
            for camera_key in camera_keys:
                local_file_index = int(cell_value(source_row.get(f"videos/{camera_key}/file_index"), 0))
                if local_file_index not in video_file_map:
                    raise KeyError(
                        f"Video file index {local_file_index} for {camera_key} not found in source {converted_source.source_path}"
                    )
                videos[camera_key] = {
                    "file_index": video_file_map[local_file_index],
                    "from_timestamp": float(cell_value(source_row.get(f"videos/{camera_key}/from_timestamp"), row["from_timestamp"])),
                    "to_timestamp": float(cell_value(source_row.get(f"videos/{camera_key}/to_timestamp"), row["to_timestamp"])),
                }
            row["videos"] = videos
            row["source_path"] = str(converted_source.source_path)
            row["task_id"] = converted_source.task_id
            episode_rows.append(row)

    if not tables:
        raise ValueError("No converted episode data found")

    merged_table = pa.concat_tables(tables)
    pq.write_table(merged_table, output_dir / "data" / "chunk-000" / "file-000.parquet")
    write_tasks(output_dir, tasks)
    write_episodes(output_dir, episode_rows, camera_keys)
    image_stats = aggregate_source_feature_stats(converted, "observation.images.")
    write_stats(output_dir, all_state_vectors, all_action_vectors, extra_stats=image_stats)
    write_info(
        output_dir,
        base_info,
        repo_id=repo_id,
        total_episodes=len(episode_rows),
        total_frames=global_start,
        total_tasks=len(tasks),
        camera_keys=camera_keys,
        fps=fps,
        resize_width=resize_width,
        resize_height=resize_height,
        vector_width=vector_width,
        total_video_files=next_video_file_index,
    )
    if write_conversion_manifest_file:
        write_conversion_manifest(
            output_dir,
            manifest_path=manifest_path,
            tasks=tasks,
            converted=converted,
            resize_width=resize_width,
            resize_height=resize_height,
            vector_width=vector_width,
            episode_policy=episode_policy,
        )


def part_repo_id(repo_id: str, task_index: int, file_index: int) -> str:
    owner, name = repo_id.split("/", 1)
    safe_name = name.replace("/", "_")
    return f"{owner}/{safe_name}_part_{task_index:03d}_{file_index:03d}"


def main() -> int:
    args = parse_args()
    require_runtime_dependencies()
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent
    manifest_path = Path(args.manifest).expanduser().resolve()
    dataset, tasks = parse_manifest(manifest_path)

    repo_id = args.repo_id or text_value(dataset.get("repo_id"), "dataset.repo_id")
    fps = int(args.fps or dataset.get("fps") or 30)
    resize_width = int(args.resize_width or dataset.get("resize_width") or 1280)
    resize_height = int(args.resize_height or dataset.get("resize_height") or 720)
    vector_width = int(dataset.get("state_action_width") or 16)
    if vector_width <= 0:
        raise ValueError("state_action_width must be a positive integer")
    episode_policy = args.episode_policy or str(dataset.get("episode_policy") or "file")
    if episode_policy not in {"file", "step"}:
        raise ValueError("episode_policy must be 'file' or 'step'")
    jobs = validate_jobs(int(args.jobs if args.jobs is not None else dataset.get("jobs") or DEFAULT_JOBS))

    output_dir = Path(args.output_dir).expanduser().resolve()
    ensure_empty_output(output_dir)

    converter = Path(args.converter).expanduser().resolve() if args.converter else script_dir / "convert_rosbag_to_lerobot.py"
    if not converter.exists():
        raise FileNotFoundError(f"Converter script not found: {converter}")
    config = resolve_cli_path(args.config, repo_root)
    work_dir_parent = prepare_work_dir_parent(args.work_dir)

    if args.keep_temp:
        temp_context = None
        temp_dir = Path(tempfile.mkdtemp(prefix="lerobot_v3_batch_", dir=work_dir_parent))
    else:
        temp_context = tempfile.TemporaryDirectory(prefix="lerobot_v3_batch_", dir=work_dir_parent)
        temp_dir = Path(temp_context.name)
    try:
        work_items = build_source_work_items(tasks, repo_id, temp_dir)
        converted = convert_source_work_items(
            work_items,
            jobs=jobs,
            python_bin=sys.executable,
            converter=converter,
            repo_root=repo_root,
            config=str(config),
            fps=fps,
            resize_width=resize_width,
            resize_height=resize_height,
            video_codec=args.video_codec,
            video_backend=args.video_backend,
            max_frames=args.max_frames_per_file,
            episode_policy=episode_policy,
        )

        merge_converted_datasets(
            output_dir=output_dir,
            repo_id=repo_id,
            manifest_path=manifest_path,
            tasks=tasks,
            converted=converted,
            fps=fps,
            resize_width=resize_width,
            resize_height=resize_height,
            vector_width=vector_width,
            episode_policy=episode_policy,
            write_conversion_manifest_file=args.write_source_manifest,
        )
    except Exception:
        if output_dir.exists() and not any(output_dir.iterdir()):
            output_dir.rmdir()
        raise
    finally:
        if args.keep_temp:
            print(f"[batch] Temporary per-file outputs kept at: {temp_dir}")
        elif temp_context is not None:
            temp_context.cleanup()

    print("\n[batch] LeRobot V3 package completed.")
    print(f"  Output dir: {output_dir}")
    print(f"  Repo id: {repo_id}")
    print(f"  Tasks: {len(tasks)}")
    print(f"  Source files: {len(converted)}")
    print(f"  Episode policy: {episode_policy}")
    print(f"  Jobs: {jobs}")
    print(f"  Resolution: {resize_width}x{resize_height}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
