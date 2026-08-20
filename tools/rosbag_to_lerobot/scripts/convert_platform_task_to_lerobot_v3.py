#!/usr/bin/env python3
"""
Convert a platform task.json description into a LeRobot V3 dataset.

The platform integration layer is expected to download remote bag links to
local paths first. This script receives one task.json, normalizes it into the
batch manifest format, then calls convert_rosbag_to_lerobot_v3_batch.py.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


SUPPORTED_SOURCE_SUFFIXES = (".tar", ".tar.gz", ".tgz", ".mcap", ".db3")
DEFAULT_JOBS = 1

# Default conversion configs keyed by robot_type / model alias.
# Paths are relative to the rosbag_to_lerobot tool root (parent of scripts/).
DEFAULT_ROBOT_CONFIGS = {
    "quanta_x1": "config/quanta_x1/lerobot_v3_16d.yaml",
    "desktop": "config/desktop/lerobot_v3_16d.yaml",
    "quanta_x2": "config/quanta_x2/lerobot_v3_16d.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert platform task.json data into one LeRobot V3 dataset.",
    )
    parser.add_argument("--task-json", required=True, help="Platform task JSON file")
    parser.add_argument("--output-dir", default=None, help="Override output directory from task.json")
    parser.add_argument("--archive-path", default=None, help="Override optional output tar.gz path")
    parser.add_argument(
        "--config",
        default=None,
        help="Override conversion config. If omitted, uses dataset.config / dataset.robotType mapping.",
    )
    parser.add_argument(
        "--robot-type",
        default=None,
        help="Robot type alias used to pick a default config when --config is omitted (quanta_x1, desktop, quanta_x2, ...).",
    )
    parser.add_argument("--batch-converter", default=None, help="Path to convert_rosbag_to_lerobot_v3_batch.py")
    parser.add_argument("--single-converter", default=None, help="Path to convert_rosbag_to_lerobot.py")
    parser.add_argument("--work-dir", default=None, help="Override optional temporary conversion workspace")
    parser.add_argument(
        "--jobs",
        type=int,
        default=None,
        help=f"Number of source files to convert concurrently. Defaults to task.json dataset.jobs, then {DEFAULT_JOBS} (serial).",
    )
    parser.add_argument("--print-manifest-only", action="store_true", help="Print normalized manifest and exit")
    return parser.parse_args()


def resolve_default_config(
    *,
    repo_root: Path,
    cli_config: str | None,
    cli_robot_type: str | None,
    dataset: dict[str, Any],
    raw: dict[str, Any],
) -> Path:
    if cli_config:
        return Path(cli_config).expanduser()

    config_from_dataset = first_present(dataset, ("config", "conversionConfig", "conversion_config"))
    if config_from_dataset:
        return Path(str(config_from_dataset)).expanduser()

    robot_type = (
        cli_robot_type
        or optional_text(first_present(dataset, ("robot_type", "robotType", "model")))
        or optional_text(first_present(raw, ("robot_type", "robotType", "model")))
        or "quanta_x1"
    )
    relative = DEFAULT_ROBOT_CONFIGS.get(robot_type.strip().lower())
    if relative is None:
        supported = ", ".join(sorted(DEFAULT_ROBOT_CONFIGS))
        raise ValueError(
            f"No default conversion config for robot_type '{robot_type}'. "
            f"Pass --config explicitly. Known aliases: {supported}"
        )
    return repo_root / relative


def read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"task-json must contain a JSON object: {path}")
    return data


def text_value(raw: Any, label: str) -> str:
    if raw is None:
        raise ValueError(f"Missing required field: {label}")
    value = str(raw).strip()
    if not value:
        raise ValueError(f"Field must not be empty: {label}")
    return value


def optional_text(raw: Any) -> str | None:
    if raw is None:
        return None
    value = str(raw).strip()
    return value or None


def first_present(mapping: dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def resolve_path(raw_path: str, base_dir: Path) -> Path:
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path.resolve()


def resolve_cli_path(raw_path: str) -> Path:
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path.resolve()


def validate_jobs(value: int) -> int:
    if value <= 0:
        raise ValueError("jobs must be a positive integer")
    return value


def has_supported_suffix(path: Path) -> bool:
    text = path.name.lower()
    return any(text.endswith(suffix) for suffix in SUPPORTED_SOURCE_SUFFIXES)


def scan_bag_dir(raw_dir: str, base_dir: Path) -> list[Path]:
    directory = resolve_path(raw_dir, base_dir)
    if not directory.is_dir():
        raise NotADirectoryError(f"bag_dir is not a directory: {directory}")
    paths = [path for path in sorted(directory.iterdir()) if path.is_file() and has_supported_suffix(path)]
    if not paths:
        raise FileNotFoundError(f"No supported bag files found in directory: {directory}")
    return paths


def task_description(raw_task: dict[str, Any], label: str) -> str:
    direct = optional_text(
        first_present(raw_task, ("task", "task_desc", "taskDesc", "description", "taskDescription"))
    )
    if direct:
        return direct

    name = optional_text(first_present(raw_task, ("taskName", "task_name", "name")))
    action_desc = optional_text(first_present(raw_task, ("action_desc", "actionDesc")))
    scene_desc = optional_text(first_present(raw_task, ("scene_desc", "sceneDesc")))
    parts = [part for part in (name, action_desc, scene_desc) if part]
    if parts:
        return "；".join(parts)
    raise ValueError(f"Missing task description for {label}")


def source_paths_from_task(
    raw_task: dict[str, Any],
    base_dir: Path,
    fallback_bag_dir: str | None,
    *,
    validate_paths: bool,
) -> list[Path]:
    raw_sources = first_present(
        raw_task,
        (
            "bags",
            "bagList",
            "bag_list",
            "bag_paths",
            "bagPaths",
            "bagPathList",
            "files",
            "fileList",
            "file_list",
            "file_paths",
            "filePaths",
            "filePathList",
            "dataList",
            "data_list",
        ),
    )
    if raw_sources is None:
        raw_dir = optional_text(
            first_present(
                raw_task,
                ("bag_dir", "bagDir", "rosbag_dir", "rosbagDir", "input_dir", "inputDir", "inputPath"),
            )
        ) or fallback_bag_dir
        if raw_dir:
            return scan_bag_dir(raw_dir, base_dir)
        raise ValueError("Each task must provide bags/bagList/files/fileList/dataList or bagDir")

    if not isinstance(raw_sources, list) or not raw_sources:
        raise ValueError("bags/bagList/files/fileList/dataList must be a non-empty list")

    paths: list[Path] = []
    for index, raw_source in enumerate(raw_sources):
        if isinstance(raw_source, str):
            raw_path = raw_source
        elif isinstance(raw_source, dict):
            raw_path = text_value(
                first_present(
                    raw_source,
                    (
                        "path",
                        "bag_path",
                        "bagPath",
                        "file_path",
                        "filePath",
                        "local_path",
                        "localPath",
                        "localFilePath",
                        "download_path",
                        "downloadPath",
                    ),
                ),
                f"bags[{index}].path",
            )
        else:
            raise ValueError(f"Unsupported bag entry at index {index}: {raw_source!r}")

        if validate_paths:
            path = resolve_path(raw_path, base_dir)
            if not path.exists():
                raise FileNotFoundError(f"Bag path not found: {path}")
        else:
            path = Path(raw_path).expanduser()
        paths.append(path)
    return paths


def normalize_tasks(raw: dict[str, Any], base_dir: Path, *, validate_paths: bool) -> list[dict[str, Any]]:
    raw_tasks = first_present(raw, ("tasks", "taskList", "task_list"))
    if raw_tasks is None:
        raw_tasks = [raw]
    if not isinstance(raw_tasks, list) or not raw_tasks:
        raise ValueError("task-json must contain a non-empty tasks list or a single task object")

    dataset = raw.get("dataset") if isinstance(raw.get("dataset"), dict) else {}
    fallback_bag_dir = optional_text(
        first_present(raw, ("bag_dir", "bagDir", "rosbag_dir", "rosbagDir", "input_dir", "inputDir", "inputPath"))
    ) or optional_text(
        first_present(dataset, ("bag_dir", "bagDir", "rosbag_dir", "rosbagDir", "input_dir", "inputDir", "inputPath"))
    )

    tasks: list[dict[str, Any]] = []
    for task_index, raw_task_any in enumerate(raw_tasks):
        if not isinstance(raw_task_any, dict):
            raise ValueError(f"tasks[{task_index}] must be an object")
        raw_task = raw_task_any
        task_id = optional_text(
            first_present(raw_task, ("task_id", "taskId", "id"))
        ) or f"task_{task_index:03d}"
        task = task_description(raw_task, f"tasks[{task_index}]")
        source_paths = source_paths_from_task(raw_task, base_dir, fallback_bag_dir, validate_paths=validate_paths)
        tasks.append(
            {
                "task_id": task_id,
                "task": task,
                "files": [str(path) for path in source_paths],
            }
        )
    return tasks


def default_repo_id(tasks: list[dict[str, Any]]) -> str:
    task_id = str(tasks[0]["task_id"]).strip() if tasks else "task"
    safe = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in task_id)
    return f"x2robot/{safe or 'task'}"


def normalize_manifest(
    raw: dict[str, Any],
    task_json_path: Path,
    output_dir_override: str | None,
    *,
    validate_paths: bool,
) -> tuple[dict[str, Any], Path, Path | None, dict[str, Any]]:
    base_dir = task_json_path.parent
    dataset = raw.get("dataset") if isinstance(raw.get("dataset"), dict) else {}
    tasks = normalize_tasks(raw, base_dir, validate_paths=validate_paths)

    output_dir_raw = output_dir_override or first_present(
        dataset,
        ("output_dir", "outputDir", "output_path", "outputPath", "lerobot_dir", "lerobotDir", "lerobot_path", "lerobotPath"),
    )
    if output_dir_override:
        output_dir = resolve_cli_path(text_value(output_dir_raw, "--output-dir"))
    else:
        output_dir = resolve_path(text_value(output_dir_raw, "dataset.output_dir"), base_dir)

    archive_raw = first_present(
        dataset,
        ("archive_path", "archivePath", "tar_path", "tarPath", "output_bag_path", "outputBagPath", "package_path", "packagePath"),
    )
    archive_path = resolve_path(str(archive_raw), base_dir) if archive_raw else None

    manifest_dataset = {
        "repo_id": optional_text(first_present(dataset, ("repo_id", "repoId"))) or default_repo_id(tasks),
        "fps": int(dataset.get("fps") or raw.get("fps") or 30),
        "resize_width": int(first_present(dataset, ("resize_width", "resizeWidth")) or raw.get("resize_width") or 1280),
        "resize_height": int(first_present(dataset, ("resize_height", "resizeHeight")) or raw.get("resize_height") or 720),
        "state_action_width": int(first_present(dataset, ("state_action_width", "stateActionWidth")) or 16),
        "episode_policy": optional_text(
            first_present(dataset, ("episode_policy", "episodePolicy"))
            or first_present(raw, ("episode_policy", "episodePolicy"))
        ) or "file",
    }
    manifest = {
        "dataset": manifest_dataset,
        "tasks": tasks,
    }
    options = {
        "video_codec": optional_text(first_present(dataset, ("video_codec", "videoCodec"))) or "h264",
        "video_backend": optional_text(first_present(dataset, ("video_backend", "videoBackend"))),
        "max_frames_per_file": first_present(dataset, ("max_frames_per_file", "maxFramesPerFile")),
        "jobs": first_present(dataset, ("jobs", "parallelJobs", "parallel_jobs", "workers", "numWorkers")),
        "work_dir": optional_text(first_present(dataset, ("work_dir", "workDir", "temp_dir", "tempDir"))),
        "write_source_manifest": bool(first_present(dataset, ("write_source_manifest", "writeSourceManifest")) or False),
    }
    return manifest, output_dir, archive_path, options


def make_archive(output_dir: Path, archive_path: Path) -> None:
    if archive_path.exists():
        raise FileExistsError(f"Archive path already exists: {archive_path}")
    archive_path.parent.mkdir(parents=True, exist_ok=True)
    if archive_path.name.endswith(".tar.gz"):
        base_name = archive_path.with_suffix("").with_suffix("")
    elif archive_path.suffix == ".tgz":
        base_name = archive_path.with_suffix("")
    else:
        raise ValueError("archive_path must end with .tar.gz or .tgz")
    created = Path(shutil.make_archive(str(base_name), "gztar", root_dir=output_dir.parent, base_dir=output_dir.name))
    if created != archive_path:
        shutil.move(str(created), archive_path)


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent
    task_json_path = Path(args.task_json).expanduser().resolve()
    raw = read_json(task_json_path)
    manifest, output_dir, archive_path, options = normalize_manifest(
        raw,
        task_json_path,
        args.output_dir,
        validate_paths=not args.print_manifest_only,
    )
    if args.archive_path:
        archive_path = resolve_cli_path(args.archive_path)
    jobs = validate_jobs(int(args.jobs if args.jobs is not None else options["jobs"] or DEFAULT_JOBS))
    work_dir = resolve_cli_path(args.work_dir) if args.work_dir else (
        resolve_path(str(options["work_dir"]), task_json_path.parent) if options["work_dir"] else None
    )

    if args.print_manifest_only:
        print(json.dumps(manifest, ensure_ascii=False, indent=2))
        return 0

    batch_converter = Path(args.batch_converter).expanduser().resolve() if args.batch_converter else script_dir / "convert_rosbag_to_lerobot_v3_batch.py"
    if not batch_converter.exists():
        raise FileNotFoundError(f"Batch converter not found: {batch_converter}")

    dataset = raw.get("dataset") if isinstance(raw.get("dataset"), dict) else {}
    config_path = resolve_default_config(
        repo_root=repo_root,
        cli_config=args.config,
        cli_robot_type=args.robot_type,
        dataset=dataset,
        raw=raw,
    )
    if not config_path.is_absolute():
        candidate = (repo_root / config_path).resolve()
        config_path = candidate if candidate.exists() else config_path.expanduser().resolve()
    else:
        config_path = config_path.resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"Conversion config not found: {config_path}")

    single_converter = Path(args.single_converter).expanduser().resolve() if args.single_converter else script_dir / "convert_rosbag_to_lerobot.py"

    with tempfile.TemporaryDirectory(prefix="platform_task_") as temp_dir:
        manifest_path = Path(temp_dir) / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        cmd = [
            sys.executable,
            str(batch_converter),
            "--manifest",
            str(manifest_path),
            "--output-dir",
            str(output_dir),
            "--config",
            str(config_path),
            "--converter",
            str(single_converter),
            "--video-codec",
            str(options["video_codec"]),
            "--episode-policy",
            str(manifest["dataset"]["episode_policy"]),
        ]
        if options["video_backend"]:
            cmd.extend(["--video-backend", str(options["video_backend"])])
        if options["max_frames_per_file"] is not None:
            cmd.extend(["--max-frames-per-file", str(options["max_frames_per_file"])])
        cmd.extend(["--jobs", str(jobs)])
        if work_dir:
            cmd.extend(["--work-dir", str(work_dir)])
        if options["write_source_manifest"]:
            cmd.append("--write-source-manifest")

        print(f"[platform] Converting task json: {task_json_path}")
        print(f"[platform] Output dir: {output_dir}")
        print(f"[platform] Config: {config_path}")
        print(f"[platform] Jobs: {jobs}")
        subprocess.run(cmd, cwd=repo_root, check=True)

    if archive_path:
        make_archive(output_dir, archive_path)
        print(f"[platform] Archive written: {archive_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
