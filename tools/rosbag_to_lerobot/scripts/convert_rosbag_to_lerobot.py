"""
Convert rosbag2 data directly to LeRobot dataset format.

This script is intentionally self-contained so we can add a rosbag -> LeRobot
path without changing the existing JSON-based conversion scripts.

Current assumptions:
    - One rosbag is converted into one episode unless step_index.json provides splits.
    - Robot topics / state / action mapping come from a YAML config (quanta_x1, quanta_x2, ...).
    - action.mode can be:
        - next_state: action[t] = state[t+1] (last frame repeats state[t])
        - from_sources: action fields are extracted from configured observation topics

Recommended environment:
    source .venv/bin/activate
    pip install rosbags torch
    pip install lerobot-0.4.2-py3-none-any.whl

Example:
    python3 convert_rosbag_to_lerobot.py \
        --bag-path /path/to/rosbag2_dir \
        --output-dir ./lerobot_data \
        --repo-id my_robot/dataset \
        --config config/quanta_x1/lerobot_v3_16d.yaml \
        --robot-type quanta_x1 \
        --use-videos
"""

from __future__ import annotations

import argparse
import bisect
import contextlib
import json
import math
import shutil
import sys
import tarfile
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


ROBOT_TYPE_ALIASES = {
    "quanta_x1": "quanta_x1",
    "desktop": "desktop",
    "quanta_x2": "quanta_x2",
}

ACTION_MODE_NEXT_STATE = "next_state"
ACTION_MODE_FROM_SOURCES = "from_sources"
SUPPORTED_ACTION_MODES = {ACTION_MODE_NEXT_STATE, ACTION_MODE_FROM_SOURCES}
SUPPORTED_DECODER_KINDS = {
    "pose_stamped",
    "joint_state",
    "odometry",
    "float32",
    "float64",
    "float64_multi_array",
}

VIDEO_CODEC_CHOICES = ("libsvtav1", "h264", "hevc")
_LEROBOT_VIDEO_CODEC_OVERRIDE: str | None = None


@dataclass(frozen=True)
class TopicSpec:
    field_name: str
    decoder_kind: str
    required: bool = True
    priority: int = 0


@dataclass(frozen=True)
class CameraTopicSpec:
    camera_name: str
    priority: int = 0
    required: bool = False


@dataclass(frozen=True)
class StateFieldSpec:
    name: str
    source: str | None
    extractor: str
    enabled: bool = True


@dataclass(frozen=True)
class ConversionConfig:
    profile: str
    robot_type: str
    lerobot_robot_type: str
    observation_topic_specs: dict[str, TopicSpec]
    camera_topic_specs: dict[str, CameraTopicSpec]
    state_fields: tuple[StateFieldSpec, ...]
    action_mode: str = ACTION_MODE_NEXT_STATE
    action_fields_from: str = "state"
    action_fields: tuple[StateFieldSpec, ...] = ()


@dataclass(frozen=True)
class SerializedBagMessage:
    timestamp_ns: int
    msgtype: str
    rawdata: bytes


@dataclass
class CameraStreamState:
    camera_name: str
    series: list[SerializedBagMessage]
    decoder_kind: str
    codec_name: str | None = None
    codec_context: Any | None = None
    next_decode_index: int = 0
    last_requested_index: int | None = None
    last_requested_frame: Any | None = None
    last_decoded_frame: Any | None = None


@dataclass
class BagScanStats:
    skipped_observation_messages: int = 0
    empty_camera_messages: int = 0
    camera_decode_failures: int = 0
    skipped_observation_topics: dict[str, int] = field(default_factory=dict)
    empty_camera_topics: dict[str, int] = field(default_factory=dict)
    camera_decode_topics: dict[str, int] = field(default_factory=dict)

    def record_skipped_observation(self, topic_name: str) -> None:
        self.skipped_observation_messages += 1
        self.skipped_observation_topics[topic_name] = self.skipped_observation_topics.get(topic_name, 0) + 1

    def record_empty_camera(self, topic_name: str) -> None:
        self.empty_camera_messages += 1
        self.empty_camera_topics[topic_name] = self.empty_camera_topics.get(topic_name, 0) + 1

    def record_camera_decode_failure(self, camera_name: str) -> None:
        self.camera_decode_failures += 1
        self.camera_decode_topics[camera_name] = self.camera_decode_topics.get(camera_name, 0) + 1


@dataclass(frozen=True)
class EpisodeSpec:
    start_index: int
    end_index: int
    task: str
    step_number: int | None = None


def enabled_state_fields(config: ConversionConfig) -> tuple[StateFieldSpec, ...]:
    return tuple(field_spec for field_spec in config.state_fields if field_spec.enabled)


def enabled_action_fields(config: ConversionConfig) -> tuple[StateFieldSpec, ...]:
    return tuple(field_spec for field_spec in config.action_fields if field_spec.enabled)


def state_names(config: ConversionConfig) -> list[str]:
    return [field_spec.name for field_spec in enabled_state_fields(config)]


def action_names(config: ConversionConfig) -> list[str]:
    if config.action_mode == ACTION_MODE_NEXT_STATE:
        return state_names(config)
    return [field_spec.name for field_spec in enabled_action_fields(config)]


# Backward-compatible alias used by older call sites / docs.
def state_action_names(config: ConversionConfig) -> list[str]:
    return state_names(config)


def observation_field_names(config: ConversionConfig) -> tuple[str, ...]:
    return tuple(dict.fromkeys(spec.field_name for spec in config.observation_topic_specs.values()))


def camera_names(config: ConversionConfig) -> tuple[str, ...]:
    return tuple(dict.fromkeys(spec.camera_name for spec in config.camera_topic_specs.values()))


def required_observation_fields(config: ConversionConfig) -> tuple[str, ...]:
    return tuple(
        dict.fromkeys(spec.field_name for spec in config.observation_topic_specs.values() if spec.required)
    )


def required_camera_names(config: ConversionConfig) -> tuple[str, ...]:
    return tuple(
        dict.fromkeys(spec.camera_name for spec in config.camera_topic_specs.values() if spec.required)
    )


def _as_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"Config field '{label}' must be a mapping")
    return value


def _as_sequence(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"Config field '{label}' must be a list")
    return value


def _is_enabled(mapping: dict[str, Any]) -> bool:
    return bool(mapping.get("enabled", True))


def _to_priority(value: Any, default: int, label: str) -> int:
    if value is None:
        return default
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Config field '{label}' must be an integer priority") from exc


def parse_topic_candidates(
    candidates: list[Any],
    group_label: str,
) -> list[tuple[str, int]]:
    parsed: list[tuple[str, int]] = []
    for index, raw_candidate in enumerate(candidates):
        candidate = _as_mapping(raw_candidate, f"{group_label}.candidates[{index}]")
        if not _is_enabled(candidate):
            continue

        topic = str(candidate.get("topic", "")).strip()
        if not topic:
            raise ValueError(f"Config field '{group_label}.candidates[{index}].topic' is required")

        priority = _to_priority(candidate.get("priority"), index, f"{group_label}.candidates[{index}].priority")
        parsed.append((topic, priority))
    return parsed


def parse_observation_topic_specs(topics_config: dict[str, Any]) -> dict[str, TopicSpec]:
    observations_config = _as_mapping(topics_config.get("observations", {}), "topics.observations")
    observation_specs: dict[str, TopicSpec] = {}

    for field_name, raw_observation_config in observations_config.items():
        observation_config = _as_mapping(raw_observation_config, f"topics.observations.{field_name}")
        if not _is_enabled(observation_config):
            continue

        decoder_kind = str(observation_config.get("decoder", "")).strip()
        if not decoder_kind:
            raise ValueError(f"Config field 'topics.observations.{field_name}.decoder' is required")

        required = bool(observation_config.get("required", True))
        candidates = _as_sequence(
            observation_config.get("candidates", []),
            f"topics.observations.{field_name}.candidates",
        )
        for topic, priority in parse_topic_candidates(candidates, f"topics.observations.{field_name}"):
            if topic in observation_specs:
                raise ValueError(f"Duplicate observation topic in config: {topic}")
            observation_specs[topic] = TopicSpec(
                field_name=str(field_name),
                decoder_kind=decoder_kind,
                required=required,
                priority=priority,
            )

    return observation_specs


def parse_camera_topic_specs(topics_config: dict[str, Any]) -> dict[str, CameraTopicSpec]:
    cameras_config = _as_mapping(topics_config.get("cameras", {}), "topics.cameras")
    camera_specs: dict[str, CameraTopicSpec] = {}

    for camera_name, raw_camera_config in cameras_config.items():
        camera_config = _as_mapping(raw_camera_config, f"topics.cameras.{camera_name}")
        if not _is_enabled(camera_config):
            continue

        required = bool(camera_config.get("required", False))
        candidates = _as_sequence(camera_config.get("candidates", []), f"topics.cameras.{camera_name}.candidates")
        for topic, priority in parse_topic_candidates(candidates, f"topics.cameras.{camera_name}"):
            if topic in camera_specs:
                raise ValueError(f"Duplicate camera topic in config: {topic}")
            camera_specs[topic] = CameraTopicSpec(
                camera_name=str(camera_name),
                priority=priority,
                required=required,
            )

    return camera_specs


def parse_vector_fields(vector_config: dict[str, Any], section_label: str) -> tuple[StateFieldSpec, ...]:
    raw_fields = _as_sequence(vector_config.get("fields", []), f"{section_label}.fields")
    fields: list[StateFieldSpec] = []

    for index, raw_field_config in enumerate(raw_fields):
        field_config = _as_mapping(raw_field_config, f"{section_label}.fields[{index}]")
        name = str(field_config.get("name", "")).strip()
        extractor = str(field_config.get("extractor", "")).strip()
        if not name:
            raise ValueError(f"Config field '{section_label}.fields[{index}].name' is required")
        if not extractor:
            raise ValueError(f"Config field '{section_label}.fields[{index}].extractor' is required")

        raw_source = field_config.get("source")
        source = str(raw_source).strip() if raw_source is not None else None
        fields.append(
            StateFieldSpec(
                name=name,
                source=source,
                extractor=extractor,
                enabled=bool(field_config.get("enabled", True)),
            )
        )

    return tuple(fields)


def parse_state_fields(state_config: dict[str, Any]) -> tuple[StateFieldSpec, ...]:
    return parse_vector_fields(state_config, "state")


def parse_action_fields(action_config: dict[str, Any]) -> tuple[StateFieldSpec, ...]:
    if "fields" not in action_config:
        return ()
    return parse_vector_fields(action_config, "action")


def resolve_config_path(raw_path: str) -> Path:
    config_path = Path(raw_path).expanduser()
    if config_path.exists():
        return config_path.resolve()

    repo_root = Path(__file__).resolve().parents[1]
    repo_relative_path = repo_root / config_path
    if repo_relative_path.exists():
        return repo_relative_path.resolve()

    raise FileNotFoundError(f"Config file not found: {raw_path}")


def load_yaml_mapping(config_path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise ImportError("Missing YAML dependency. Install with: pip install PyYAML") from exc

    with config_path.open("r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}

    if not isinstance(data, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {config_path}")
    return data


def load_conversion_config(raw_config_path: str | None) -> ConversionConfig:
    if raw_config_path is None:
        raise ValueError("A conversion config is required. Pass --config with a YAML under config/.")

    config_path = resolve_config_path(raw_config_path)
    config_data = load_yaml_mapping(config_path)

    profile = str(config_data.get("profile") or config_path.stem).strip()
    robot_type = str(config_data.get("robot_type") or "quanta_x1").strip()
    lerobot_robot_type = str(
        config_data.get("lerobot_robot_type")
        or ROBOT_TYPE_ALIASES.get(robot_type.lower(), robot_type.lower())
    ).strip()

    topics_config = _as_mapping(config_data.get("topics", {}), "topics")
    action_config = _as_mapping(config_data.get("action", {}), "action")

    config = ConversionConfig(
        profile=profile,
        robot_type=robot_type,
        lerobot_robot_type=lerobot_robot_type,
        observation_topic_specs=parse_observation_topic_specs(topics_config),
        camera_topic_specs=parse_camera_topic_specs(topics_config),
        state_fields=parse_state_fields(_as_mapping(config_data.get("state", {}), "state")),
        action_mode=str(action_config.get("mode", ACTION_MODE_NEXT_STATE)).strip(),
        action_fields_from=str(action_config.get("fields_from", "state")).strip(),
        action_fields=parse_action_fields(action_config),
    )
    validate_conversion_config(config)
    return config


def validate_extractor(extractor: str, field_label: str) -> None:
    if extractor.startswith("constant:"):
        try:
            float(extractor.split(":", 1)[1])
        except ValueError as exc:
            raise ValueError(f"{field_label} has invalid constant extractor: {extractor}") from exc
        return

    supported_extractors = {
        "pose.position.x",
        "pose.position.y",
        "pose.position.z",
        "pose.euler.roll",
        "pose.euler.pitch",
        "pose.euler.yaw",
        "odom.position.x",
        "odom.position.y",
        "odom.euler.yaw",
        "float.value",
    }
    if extractor in supported_extractors:
        return

    if extractor.startswith("joint.position[") and extractor.endswith("]"):
        raw_index = extractor[len("joint.position[") : -1]
        try:
            index = int(raw_index)
        except ValueError as exc:
            raise ValueError(f"{field_label} has invalid joint index extractor: {extractor}") from exc
        if index < 0:
            raise ValueError(f"{field_label} has negative joint index extractor: {extractor}")
        return

    if extractor.startswith("array[") and extractor.endswith("]"):
        raw_index = extractor[len("array[") : -1]
        try:
            index = int(raw_index)
        except ValueError as exc:
            raise ValueError(f"{field_label} has invalid array index extractor: {extractor}") from exc
        if index < 0:
            raise ValueError(f"{field_label} has negative array index extractor: {extractor}")
        return

    raise ValueError(f"{field_label} has unsupported extractor: {extractor}")


def _validate_vector_fields(
    fields: tuple[StateFieldSpec, ...],
    *,
    section_label: str,
    configured_observation_fields: set[str],
) -> None:
    if not fields:
        raise ValueError(f"At least one enabled {section_label} field must be configured")

    seen_names: set[str] = set()
    for index, field_spec in enumerate(fields):
        field_label = f"{section_label}.fields[{index}] ({field_spec.name})"
        if field_spec.name in seen_names:
            raise ValueError(f"Duplicate enabled {section_label} field name: {field_spec.name}")
        seen_names.add(field_spec.name)

        validate_extractor(field_spec.extractor, field_label)
        if not field_spec.extractor.startswith("constant:"):
            if not field_spec.source:
                raise ValueError(f"{field_label} must define source unless using constant extractor")
            if field_spec.source not in configured_observation_fields:
                raise ValueError(
                    f"{field_label} source '{field_spec.source}' is not defined in topics.observations"
                )


def validate_conversion_config(config: ConversionConfig) -> None:
    if not config.profile:
        raise ValueError("Config profile must not be empty")
    if not config.robot_type:
        raise ValueError("Config robot_type must not be empty")
    if not config.lerobot_robot_type:
        raise ValueError("Config lerobot_robot_type must not be empty")
    if config.action_mode not in SUPPORTED_ACTION_MODES:
        supported = ", ".join(sorted(SUPPORTED_ACTION_MODES))
        raise ValueError(f"Unsupported action.mode '{config.action_mode}'. Supported: {supported}")
    if not config.observation_topic_specs:
        raise ValueError("At least one observation topic must be configured")

    for topic_name, topic_spec in config.observation_topic_specs.items():
        if topic_spec.decoder_kind not in SUPPORTED_DECODER_KINDS:
            raise ValueError(
                f"Observation topic '{topic_name}' has unsupported decoder '{topic_spec.decoder_kind}'"
            )

    configured_observation_fields = set(observation_field_names(config))
    active_state_fields = enabled_state_fields(config)
    _validate_vector_fields(
        active_state_fields,
        section_label="state",
        configured_observation_fields=configured_observation_fields,
    )

    if config.action_mode == ACTION_MODE_NEXT_STATE:
        if config.action_fields_from != "state":
            raise ValueError("action.mode: next_state currently requires action.fields_from: state")
        if enabled_action_fields(config):
            raise ValueError(
                "action.mode: next_state does not use action.fields; "
                "remove action.fields or switch to action.mode: from_sources"
            )
    elif config.action_mode == ACTION_MODE_FROM_SOURCES:
        active_action_fields = enabled_action_fields(config)
        _validate_vector_fields(
            active_action_fields,
            section_label="action",
            configured_observation_fields=configured_observation_fields,
        )
        if len(active_action_fields) != len(active_state_fields):
            raise ValueError(
                "action.mode: from_sources requires the same number of enabled action fields "
                f"as state fields ({len(active_action_fields)} != {len(active_state_fields)})"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert rosbag2 data directly into LeRobot dataset format.",
    )
    parser.add_argument("--bag-path", type=str, required=True, help="rosbag2 directory, metadata.yaml, or mcap/db3 file path")
    parser.add_argument("--output-dir", type=str, required=True, help="Output LeRobot dataset directory")
    parser.add_argument("--repo-id", type=str, required=True, help="Dataset repo ID, e.g. my_robot/dataset")
    parser.add_argument(
        "--config",
        type=str,
        default="config/quanta_x1/lerobot_v3_16d.yaml",
        help="YAML config for topic/state/action mapping. Defaults to config/quanta_x1/lerobot_v3_16d.yaml.",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default=None,
        help="Robot type alias override. Defaults to robot_type from config.",
    )
    parser.add_argument("--fps", type=int, default=30, help="Target frame rate for resampling")
    parser.add_argument("--resize-width", type=int, default=640, help="Output image width")
    parser.add_argument("--resize-height", type=int, default=480, help="Output image height")
    parser.add_argument("--task", type=str, default=None, help="Task string stored in LeRobot frames")
    parser.add_argument(
        "--task-meta",
        type=str,
        default=None,
        help="Optional task metadata JSON. Defaults to task_meta.json inside the bag when present.",
    )
    parser.add_argument(
        "--step-index",
        type=str,
        default=None,
        help="Optional step index JSON for splitting one rosbag into multiple LeRobot episodes. "
        "Defaults to step_index.json inside the bag when present.",
    )
    parser.add_argument(
        "--episode-policy",
        choices=["step", "file"],
        default="step",
        help=(
            "Episode splitting policy. 'step' preserves the previous behavior and uses step_index.json when present; "
            "'file' ignores step_index.json for episode splitting and writes one LeRobot episode per input bag file."
        ),
    )
    parser.add_argument("--use-videos", action="store_true", help="Use LeRobot video backend instead of storing image frames")
    parser.add_argument(
        "--video-backend",
        type=str,
        default=None,
        choices=["pyav", "opencv"],
        help="Optional LeRobot video backend",
    )
    parser.add_argument(
        "--video-codec",
        type=str,
        default=None,
        choices=VIDEO_CODEC_CHOICES,
        help="Optional LeRobot output video codec override. Defaults to LeRobot 0.4.2 behavior (libsvtav1).",
    )
    parser.add_argument("--max-frames", type=int, default=None, help="Optional frame limit for conversion checks")
    parser.add_argument("--dry-run", action="store_true", help="Only parse bag and print summary without creating LeRobot output")
    return parser.parse_args()


def warn(message: str) -> None:
    print(f"Warning: {message}", file=sys.stderr)


def normalize_ros2_msgtype(msgtype: str) -> str:
    """Normalize legacy ROS type names to ROS 2 typestore form.

    Some recorders write ``pkg/Type`` instead of ``pkg/msg/Type``.
    """
    text = str(msgtype).strip()
    parts = text.split("/")
    if len(parts) == 2:
        return f"{parts[0]}/msg/{parts[1]}"
    return text


def deserialize_cdr(typestore: Any, rawdata: bytes, msgtype: str) -> Any:
    return typestore.deserialize_cdr(rawdata, normalize_ros2_msgtype(msgtype))


def validate_args(args: argparse.Namespace) -> None:
    if args.fps <= 0:
        raise ValueError("fps must be a positive integer")
    if args.resize_width <= 0 or args.resize_height <= 0:
        raise ValueError("resize-width and resize-height must be positive integers")
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError("max-frames must be a positive integer when provided")
    if args.video_codec is not None and not args.use_videos:
        raise ValueError("video-codec requires --use-videos")

    repo_id = args.repo_id.strip()
    if not repo_id or "/" not in repo_id:
        raise ValueError("repo-id must look like 'owner/dataset_name'")

    if args.dry_run:
        return

    output_dir = Path(args.output_dir).expanduser()
    if output_dir.exists() and output_dir.is_file():
        raise FileExistsError(f"Output path is an existing file, not a directory: {output_dir}")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(
            f"Output directory already exists and is not empty: {output_dir}. "
            "Please use a new directory or remove the previous conversion output first."
        )


def canonical_robot_type(robot_type: str, config: ConversionConfig) -> str:
    key = robot_type.strip().lower()
    aliases = dict(ROBOT_TYPE_ALIASES)
    aliases[config.robot_type.strip().lower()] = config.lerobot_robot_type
    aliases[config.lerobot_robot_type.strip().lower()] = config.lerobot_robot_type
    if key not in aliases:
        supported = ", ".join(sorted(aliases))
        raise ValueError(f"Unsupported robot type '{robot_type}'. Supported aliases: {supported}")
    return aliases[key]


def is_tar_archive(path: Path) -> bool:
    name = path.name.lower()
    return name.endswith(".tar") or name.endswith(".tar.gz") or name.endswith(".tgz")


def extract_tar_bag_to_tempdir(archive_path: Path, extract_root: Path) -> Path:
    with tarfile.open(archive_path, "r:*") as tar:
        members = tar.getmembers()
        for member in members:
            member_path = Path(member.name)
            if member_path.is_absolute() or ".." in member_path.parts:
                raise ValueError(f"Unsafe path found inside rosbag archive: {member.name}")

        metadata_members = [member for member in members if Path(member.name).name == "metadata.yaml"]
        if len(metadata_members) != 1:
            raise ValueError(
                f"Expected exactly one metadata.yaml in rosbag archive, found {len(metadata_members)}: {archive_path}"
            )

        try:
            tar.extractall(extract_root, filter="data")
        except TypeError:  # pragma: no cover - older tarfile fallback
            tar.extractall(extract_root)

    bag_dir = (extract_root / Path(metadata_members[0].name).parent).resolve()
    metadata_yaml = bag_dir / "metadata.yaml"
    if not metadata_yaml.exists():
        raise FileNotFoundError(f"Extracted rosbag archive is missing metadata.yaml: {metadata_yaml}")
    return bag_dir


def prepare_bag_path(raw_path: str, exit_stack: contextlib.ExitStack) -> tuple[Path, Path]:
    bag_path = Path(raw_path).expanduser().resolve()
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")

    if bag_path.is_dir():
        metadata_yaml = bag_path / "metadata.yaml"
        if metadata_yaml.exists():
            return bag_path, bag_path
        raise FileNotFoundError(f"rosbag2 directory missing metadata.yaml: {bag_path}")

    if bag_path.name == "metadata.yaml":
        return bag_path.parent, bag_path

    if bag_path.suffix in {".mcap", ".db3"}:
        metadata_yaml = bag_path.parent / "metadata.yaml"
        if metadata_yaml.exists():
            return bag_path.parent, bag_path

    if is_tar_archive(bag_path):
        extract_root = Path(exit_stack.enter_context(tempfile.TemporaryDirectory(prefix="rosbag2_extract_")))
        return extract_tar_bag_to_tempdir(bag_path, extract_root), bag_path

    raise ValueError(
        "Unsupported bag path. Please provide a rosbag2 directory, metadata.yaml, an mcap/db3 file inside a rosbag2 "
        "directory, or a tar/tar.gz/tgz archive containing a rosbag2 directory."
    )


def load_optional_json(path: Path | None, label: str) -> dict[str, Any] | None:
    if path is None:
        return None
    if not path.exists():
        raise FileNotFoundError(f"{label} JSON not found: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid {label} JSON: {path}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{label} JSON must contain an object: {path}")
    return data


def discover_sidecar_json(bag_dir: Path, explicit_path: str | None, file_name: str, label: str) -> dict[str, Any] | None:
    if explicit_path:
        return load_optional_json(Path(explicit_path).expanduser().resolve(), label)
    candidate = bag_dir / file_name
    if not candidate.exists():
        return None
    return load_optional_json(candidate, label)


def task_meta_title(task_meta: dict[str, Any] | None) -> str | None:
    if not task_meta:
        return None
    for key in ("taskName", "name", "task_name"):
        value = task_meta.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    task_id = task_meta.get("taskId")
    if task_id is not None:
        return str(task_id)
    return None


def task_meta_action(task_meta: dict[str, Any] | None) -> str | None:
    if not task_meta:
        return None
    for key in ("actionDesc", "action_desc", "description"):
        value = task_meta.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def step_action_from_task_meta(task_meta: dict[str, Any] | None, step_number: int | None) -> str | None:
    if not task_meta or step_number is None:
        return None
    step_list = task_meta.get("stepList") or task_meta.get("steps")
    if not isinstance(step_list, list):
        return None
    for raw_step in step_list:
        if not isinstance(raw_step, dict):
            continue
        number = raw_step.get("number", raw_step.get("step_number"))
        try:
            if number is None or int(number) != step_number:
                continue
        except (TypeError, ValueError):
            continue
        action = raw_step.get("action")
        if isinstance(action, str) and action.strip():
            return action.strip()
    return None


def build_task_name(
    override_task: str | None,
    fallback_task: str,
    task_meta: dict[str, Any] | None,
    step_number: int | None = None,
    step_action: str | None = None,
) -> str:
    if override_task:
        return override_task

    action = step_action or step_action_from_task_meta(task_meta, step_number)
    if action:
        return action

    title = task_meta_title(task_meta)
    action_desc = task_meta_action(task_meta)
    if title and action_desc:
        return f"{title}: {action_desc}"
    if title:
        return title
    if action_desc:
        return action_desc
    return fallback_task


def normalize_timestamp_to_ns(value: Any) -> int | None:
    if value is None:
        return None
    try:
        timestamp = float(value)
    except (TypeError, ValueError):
        return None

    magnitude = abs(timestamp)
    if magnitude >= 1e17:
        return int(timestamp)
    if magnitude >= 1e14:
        return int(timestamp * 1_000)
    if magnitude >= 1e11:
        return int(timestamp * 1_000_000)
    return int(timestamp * 1_000_000_000)


def first_present(mapping: dict[str, Any], keys: tuple[str, ...]) -> Any:
    for key in keys:
        if key in mapping:
            return mapping[key]
    return None


def build_episode_specs(
    sample_times: list[int],
    fallback_task: str,
    args_task: str | None,
    task_meta: dict[str, Any] | None,
    step_index: dict[str, Any] | None,
) -> list[EpisodeSpec]:
    default_task = build_task_name(args_task, fallback_task, task_meta)
    if not step_index:
        return [EpisodeSpec(0, len(sample_times), default_task)]

    raw_steps = step_index.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        return [EpisodeSpec(0, len(sample_times), default_task)]

    episode_specs: list[EpisodeSpec] = []
    for raw_step in raw_steps:
        if not isinstance(raw_step, dict):
            continue

        step_number_raw = first_present(raw_step, ("step_number", "number", "stepNumber"))
        try:
            step_number = int(step_number_raw) if step_number_raw is not None else None
        except (TypeError, ValueError):
            step_number = None

        start_ns = normalize_timestamp_to_ns(
            first_present(raw_step, ("start_ts", "startTs", "start_time", "startTime", "start_ns", "startNs"))
        )
        end_ns = normalize_timestamp_to_ns(
            first_present(raw_step, ("end_ts", "endTs", "end_time", "endTime", "end_ns", "endNs"))
        )
        if start_ns is None or end_ns is None:
            warn(f"Skipped step {step_number or '?'} because start/end timestamp is missing or invalid.")
            continue
        if end_ns <= sample_times[0] or start_ns >= sample_times[-1]:
            warn(f"Skipped step {step_number or '?'} because it is outside the sampled time window.")
            continue
        if end_ns <= start_ns:
            warn(f"Skipped step {step_number or '?'} because end timestamp is not after start timestamp.")
            continue

        start_index = bisect.bisect_left(sample_times, max(start_ns, sample_times[0]))
        end_index = bisect.bisect_right(sample_times, min(end_ns, sample_times[-1]))
        if end_index <= start_index:
            warn(f"Skipped step {step_number or '?'} because no sampled frames fall within its time range.")
            continue

        step_action = raw_step.get("action")
        if not isinstance(step_action, str) or not step_action.strip():
            step_action = None
        task_name = build_task_name(args_task, fallback_task, task_meta, step_number, step_action)
        episode_specs.append(EpisodeSpec(start_index, end_index, task_name, step_number))

    if not episode_specs:
        warn("step_index.json was found, but no valid steps were usable. Falling back to one episode.")
        return [EpisodeSpec(0, len(sample_times), default_task)]

    episode_specs.sort(key=lambda item: item.start_index)
    return episode_specs


def copy_conversion_sidecars(output_dir: str, task_meta: dict[str, Any] | None, step_index: dict[str, Any] | None) -> None:
    if not task_meta and not step_index:
        return
    meta_dir = Path(output_dir).expanduser() / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    if task_meta:
        (meta_dir / "x2_task_meta.json").write_text(
            json.dumps(task_meta, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    if step_index:
        (meta_dir / "x2_step_index.json").write_text(
            json.dumps(step_index, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


def import_runtime_dependencies(require_lerobot: bool) -> dict[str, Any]:
    modules: dict[str, Any] = {}

    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise ImportError("Missing runtime dependency. Install with: pip install numpy opencv-python") from exc

    try:
        from rosbags.rosbag2.reader import Reader  # type: ignore
        from rosbags.typesys import Stores, get_typestore  # type: ignore
    except ImportError as exc:
        raise ImportError("Missing rosbag reader dependency. Install with: pip install rosbags") from exc

    modules["cv2"] = cv2
    modules["np"] = np
    modules["Reader"] = Reader
    modules["Stores"] = Stores
    modules["get_typestore"] = get_typestore

    try:
        import av  # type: ignore
    except ImportError:
        av = None
    modules["av"] = av

    if require_lerobot:
        try:
            import torch  # type: ignore
            from lerobot.datasets.lerobot_dataset import LeRobotDataset  # type: ignore
            from lerobot.datasets import lerobot_dataset as lerobot_dataset_module  # type: ignore
            from lerobot.datasets.video_utils import encode_video_frames  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "Missing LeRobot conversion dependency. Install with: pip install torch and then install lerobot v0.4.2."
            ) from exc
        modules["torch"] = torch
        modules["LeRobotDataset"] = LeRobotDataset
        modules["lerobot_dataset_module"] = lerobot_dataset_module
        modules["encode_video_frames"] = encode_video_frames

    return modules


def msg_array_to_list(values: Any) -> list[float]:
    if values is None:
        return []
    if isinstance(values, (int, float)):
        return [float(values)]
    return [float(v) for v in values]


def decode_pose_stamped(msg: Any) -> dict[str, Any]:
    return {
        "position": {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.position.z),
        },
        "orientation": {
            "x": float(msg.pose.orientation.x),
            "y": float(msg.pose.orientation.y),
            "z": float(msg.pose.orientation.z),
            "w": float(msg.pose.orientation.w),
        },
    }


def decode_joint_state(msg: Any) -> dict[str, Any]:
    decoded = {
        "names": list(msg.name),
        "positions": msg_array_to_list(msg.position),
    }
    velocities = msg_array_to_list(getattr(msg, "velocity", []))
    efforts = msg_array_to_list(getattr(msg, "effort", []))
    if velocities:
        decoded["velocities"] = velocities
    if efforts:
        decoded["efforts"] = efforts
    return decoded


def decode_odometry(msg: Any) -> dict[str, Any]:
    return {
        "pose": {
            "position": {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z),
            },
            "orientation": {
                "x": float(msg.pose.pose.orientation.x),
                "y": float(msg.pose.pose.orientation.y),
                "z": float(msg.pose.pose.orientation.z),
                "w": float(msg.pose.pose.orientation.w),
            },
        },
        "twist": {
            "linear": {
                "x": float(msg.twist.twist.linear.x),
                "y": float(msg.twist.twist.linear.y),
                "z": float(msg.twist.twist.linear.z),
            },
            "angular": {
                "x": float(msg.twist.twist.angular.x),
                "y": float(msg.twist.twist.angular.y),
                "z": float(msg.twist.twist.angular.z),
            },
        },
    }


def decode_float_scalar(msg: Any) -> dict[str, Any]:
    return {"value": float(msg.data)}


def decode_float64_multi_array(msg: Any) -> dict[str, Any]:
    return {"data": [float(value) for value in list(msg.data)]}


def decode_observation_message(msg: Any, decoder_kind: str) -> dict[str, Any]:
    if decoder_kind == "pose_stamped":
        return decode_pose_stamped(msg)
    if decoder_kind == "joint_state":
        return decode_joint_state(msg)
    if decoder_kind == "odometry":
        return decode_odometry(msg)
    if decoder_kind in {"float32", "float64"}:
        return decode_float_scalar(msg)
    if decoder_kind == "float64_multi_array":
        return decode_float64_multi_array(msg)
    raise ValueError(f"Unsupported decoder kind: {decoder_kind}")


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def extract_bracket_index(extractor: str, prefix: str) -> int:
    return int(extractor[len(prefix) : -1])


def extract_configured_field_value(
    field_spec: StateFieldSpec,
    sample: dict[str, dict[str, Any] | None],
) -> float:
    extractor = field_spec.extractor
    if extractor.startswith("constant:"):
        return float(extractor.split(":", 1)[1])

    source_payload = sample.get(field_spec.source or "")
    if not source_payload:
        return 0.0

    if extractor.startswith("pose.position."):
        axis = extractor.rsplit(".", 1)[-1]
        return float(source_payload.get("position", {}).get(axis, 0.0))

    if extractor.startswith("pose.euler."):
        component = extractor.rsplit(".", 1)[-1]
        ori = source_payload.get("orientation", {})
        roll, pitch, yaw = quaternion_to_euler(
            float(ori.get("x", 0.0)),
            float(ori.get("y", 0.0)),
            float(ori.get("z", 0.0)),
            float(ori.get("w", 1.0)),
        )
        return {"roll": roll, "pitch": pitch, "yaw": yaw}[component]

    if extractor.startswith("joint.position["):
        positions = source_payload.get("positions", [])
        index = extract_bracket_index(extractor, "joint.position[")
        if index >= len(positions):
            return 0.0
        return float(positions[index])

    if extractor.startswith("array["):
        values = source_payload.get("data", [])
        index = extract_bracket_index(extractor, "array[")
        if index >= len(values):
            return 0.0
        return float(values[index])

    if extractor == "float.value":
        return float(source_payload.get("value", 0.0))

    if extractor.startswith("odom.position."):
        axis = extractor.rsplit(".", 1)[-1]
        return float(source_payload.get("pose", {}).get("position", {}).get(axis, 0.0))

    if extractor == "odom.euler.yaw":
        ori = source_payload.get("pose", {}).get("orientation", {})
        _, _, yaw = quaternion_to_euler(
            float(ori.get("x", 0.0)),
            float(ori.get("y", 0.0)),
            float(ori.get("z", 0.0)),
            float(ori.get("w", 1.0)),
        )
        return yaw

    raise ValueError(f"Unsupported extractor at runtime: {extractor}")


def build_vector_from_fields(
    sample: dict[str, dict[str, Any] | None],
    fields: tuple[StateFieldSpec, ...],
    np_module: Any,
) -> Any:
    values = [extract_configured_field_value(field_spec, sample) for field_spec in fields]
    return np_module.array(values, dtype=np_module.float32)


def build_state_vector(
    sample: dict[str, dict[str, Any] | None],
    config: ConversionConfig,
    np_module: Any,
) -> Any:
    return build_vector_from_fields(sample, enabled_state_fields(config), np_module)


def build_action_vector(
    sample: dict[str, dict[str, Any] | None],
    config: ConversionConfig,
    np_module: Any,
) -> Any:
    return build_vector_from_fields(sample, enabled_action_fields(config), np_module)


def create_lerobot_dataset(
    output_dir: str,
    repo_id: str,
    robot_type: str,
    fps: int,
    state_names: list[str],
    action_names: list[str],
    camera_names: list[str],
    resize_width: int,
    resize_height: int,
    use_videos: bool,
    video_backend: str | None,
    lerobot_dataset_cls: Any,
) -> Any:
    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(state_names),),
            "names": state_names,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(action_names),),
            "names": action_names,
        },
    }

    for camera_name in camera_names:
        features[f"observation.images.{camera_name}"] = {
            "dtype": "video" if use_videos else "image",
            "shape": (3, resize_height, resize_width),
            "names": ["channels", "height", "width"],
        }

    return lerobot_dataset_cls.create(
        repo_id=repo_id,
        fps=fps,
        robot_type=robot_type,
        features=features,
        use_videos=use_videos,
        video_backend=video_backend,
        root=output_dir,
    )


def _encode_lerobot_video_worker_override(video_key: str, episode_index: int, root: Path, fps: int) -> Path:
    if _LEROBOT_VIDEO_CODEC_OVERRIDE is None:
        raise RuntimeError("LeRobot video codec override is not configured")

    # Import locally so the helper remains picklable when LeRobot dispatches
    # encoding work through multiprocessing.
    from lerobot.datasets import lerobot_dataset as lerobot_dataset_module  # type: ignore
    from lerobot.datasets.video_utils import encode_video_frames  # type: ignore

    temp_path = Path(tempfile.mkdtemp(dir=root)) / f"{video_key}_{episode_index:03d}.mp4"
    image_path = lerobot_dataset_module.DEFAULT_IMAGE_PATH.format(
        image_key=video_key,
        episode_index=episode_index,
        frame_index=0,
    )
    img_dir = (Path(root) / image_path).parent
    encode_video_frames(img_dir, temp_path, fps, vcodec=_LEROBOT_VIDEO_CODEC_OVERRIDE, overwrite=True)
    shutil.rmtree(img_dir)
    return temp_path


def configure_lerobot_video_codec(runtime: dict[str, Any], video_codec: str | None) -> None:
    global _LEROBOT_VIDEO_CODEC_OVERRIDE
    _LEROBOT_VIDEO_CODEC_OVERRIDE = video_codec
    if video_codec is None:
        return

    lerobot_dataset_module = runtime["lerobot_dataset_module"]
    lerobot_dataset_module._encode_video_worker = _encode_lerobot_video_worker_override


def patch_lerobot_image_stats() -> None:
    """Work around a LeRobot 0.4.2 bug in image statistics.

    RunningQuantileStats.update() computes E[X^2] with ``batch**2`` while
    sample_images() loads frames as uint8, so the squares overflow modulo 256.
    The resulting variance goes negative and is clipped to 0, which writes
    std=[0, 0, 0] for every image feature into meta/stats.json and breaks
    MEAN_STD visual normalization downstream. Casting the batch to float64
    before the update restores correct statistics.
    """
    import numpy as np
    from lerobot.datasets import compute_stats as compute_stats_module

    original_update = compute_stats_module.RunningQuantileStats.update
    if getattr(original_update, "_float64_stats_patch", False):
        return

    def patched_update(self, batch, *args, **kwargs):
        return original_update(self, np.asarray(batch, dtype=np.float64), *args, **kwargs)

    patched_update._float64_stats_patch = True
    compute_stats_module.RunningQuantileStats.update = patched_update


def resample_latest_indices(timestamps: list[int], sample_times: list[int]) -> list[int | None]:
    result: list[int | None] = []
    latest_index: int | None = None
    source_index = 0
    total = len(timestamps)

    for sample_time in sample_times:
        while source_index < total and timestamps[source_index] <= sample_time:
            latest_index = source_index
            source_index += 1
        result.append(latest_index)

    return result


def sort_series_by_timestamp(
    observation_series: dict[str, list[tuple[int, dict[str, Any]]]],
    camera_series: dict[str, list[SerializedBagMessage]],
) -> None:
    for series in observation_series.values():
        series.sort(key=lambda item: item[0])
    for series in camera_series.values():
        series.sort(key=lambda item: item.timestamp_ns)


def select_preferred_series(
    candidates: dict[str, dict[str, list[Any]]],
    priority_by_topic: dict[str, int],
) -> tuple[dict[str, list[Any]], dict[str, str | None]]:
    selected_series: dict[str, list[Any]] = {}
    selected_topics: dict[str, str | None] = {}

    for field_name, series_by_topic in candidates.items():
        ranked: list[tuple[bool, int, int, str, list[Any]]] = []
        for topic_name, series in series_by_topic.items():
            ranked.append((not bool(series), priority_by_topic[topic_name], -len(series), topic_name, series))

        if not ranked:
            selected_series[field_name] = []
            selected_topics[field_name] = None
            continue

        _, _, _, topic_name, series = min(ranked)
        selected_series[field_name] = series
        selected_topics[field_name] = topic_name if series else None

    return selected_series, selected_topics


def collect_bag_data(
    reader_cls: Any,
    typestore: Any,
    bag_dir: Path,
    config: ConversionConfig,
) -> tuple[
    dict[str, list[tuple[int, dict[str, Any]]]],
    dict[str, list[SerializedBagMessage]],
    BagScanStats,
    dict[str, str | None],
    dict[str, str | None],
]:
    observation_candidates: dict[str, dict[str, list[tuple[int, dict[str, Any]]]]] = {
        field_name: {} for field_name in observation_field_names(config)
    }
    for topic_name, spec in config.observation_topic_specs.items():
        observation_candidates[spec.field_name][topic_name] = []

    camera_candidates: dict[str, dict[str, list[SerializedBagMessage]]] = {
        camera_name: {} for camera_name in camera_names(config)
    }
    for topic_name, spec in config.camera_topic_specs.items():
        camera_candidates[spec.camera_name][topic_name] = []

    scan_stats = BagScanStats()

    with reader_cls(bag_dir) as reader:
        for connection, timestamp_ns, rawdata in reader.messages():
            topic_name = connection.topic

            if topic_name in config.observation_topic_specs:
                spec = config.observation_topic_specs[topic_name]
                try:
                    msg = deserialize_cdr(typestore, rawdata, connection.msgtype)
                    decoded = decode_observation_message(msg, spec.decoder_kind)
                except Exception as exc:
                    scan_stats.record_skipped_observation(topic_name)
                    warn(f"Skipped unreadable observation message on {topic_name} at {timestamp_ns}: {exc}")
                    continue

                observation_candidates[spec.field_name][topic_name].append((int(timestamp_ns), decoded))
                continue

            if topic_name in config.camera_topic_specs:
                if not rawdata:
                    scan_stats.record_empty_camera(topic_name)
                    warn(f"Skipped empty camera message on {topic_name} at {timestamp_ns}")
                    continue

                camera_name = config.camera_topic_specs[topic_name].camera_name
                camera_candidates[camera_name][topic_name].append(
                    SerializedBagMessage(
                        timestamp_ns=int(timestamp_ns),
                        msgtype=connection.msgtype,
                        rawdata=bytes(rawdata),
                    )
                )

    observation_series, observation_sources = select_preferred_series(
        observation_candidates,
        {topic_name: spec.priority for topic_name, spec in config.observation_topic_specs.items()},
    )
    camera_series, camera_sources = select_preferred_series(
        camera_candidates,
        {topic_name: spec.priority for topic_name, spec in config.camera_topic_specs.items()},
    )
    sort_series_by_timestamp(observation_series, camera_series)
    return observation_series, camera_series, scan_stats, observation_sources, camera_sources


def validate_required_topics(
    observation_series: dict[str, list[tuple[int, dict[str, Any]]]],
    camera_series: dict[str, list[Any]],
    config: ConversionConfig,
) -> None:
    missing_observations = [
        field_name for field_name in required_observation_fields(config) if not observation_series.get(field_name)
    ]
    if missing_observations:
        raise ValueError(
            f"Missing required observation topics in rosbag: {', '.join(missing_observations)}"
        )

    missing_cameras = [
        camera_name for camera_name in required_camera_names(config) if not camera_series.get(camera_name)
    ]
    if missing_cameras:
        raise ValueError(f"Missing required camera topics in rosbag: {', '.join(missing_cameras)}")


def build_sampling_window(
    observation_series: dict[str, list[tuple[int, dict[str, Any]]]],
    active_camera_series: dict[str, list[SerializedBagMessage]],
) -> tuple[int, int]:
    first_timestamps: list[int] = []
    last_timestamps: list[int] = []

    for series in observation_series.values():
        if not series:
            continue
        first_timestamps.append(series[0][0])
        last_timestamps.append(series[-1][0])

    for series in active_camera_series.values():
        if not series:
            continue
        first_timestamps.append(series[0].timestamp_ns)
        last_timestamps.append(series[-1].timestamp_ns)

    if not first_timestamps or not last_timestamps:
        raise ValueError("No usable observation or camera samples were found in the rosbag.")

    start_ns = max(first_timestamps)
    end_ns = min(last_timestamps)
    if start_ns >= end_ns:
        raise ValueError(
            f"Sampling window is empty after alignment. start_ns={start_ns}, end_ns={end_ns}. Check topic coverage."
        )
    return start_ns, end_ns


def build_sample_times(start_ns: int, end_ns: int, fps: int, max_frames: int | None) -> list[int]:
    step_ns = int(1e9 / fps)
    if step_ns <= 0:
        raise ValueError("Computed frame interval is invalid")

    sample_times = list(range(start_ns, end_ns + 1, step_ns))
    if max_frames is not None:
        sample_times = sample_times[:max_frames]

    if not sample_times:
        raise ValueError("No frames were generated from the selected sampling window")

    return sample_times


def sample_observation_series(
    observation_series: dict[str, list[tuple[int, dict[str, Any]]]],
    sample_times: list[int],
) -> list[dict[str, dict[str, Any] | None]]:
    sampled: list[dict[str, dict[str, Any] | None]] = []
    resampled_by_field: dict[str, list[dict[str, Any] | None]] = {}

    for field_name, series in observation_series.items():
        timestamps = [timestamp_ns for timestamp_ns, _ in series]
        payloads = [payload for _, payload in series]
        indices = resample_latest_indices(timestamps, sample_times)
        resampled_by_field[field_name] = [payloads[index] if index is not None else None for index in indices]

    for frame_index in range(len(sample_times)):
        sampled.append({field_name: values[frame_index] for field_name, values in resampled_by_field.items()})

    return sampled


def sample_camera_indices(
    camera_series: dict[str, list[SerializedBagMessage]],
    sample_times: list[int],
) -> dict[str, list[int | None]]:
    sampled: dict[str, list[int | None]] = {}
    for camera_name, series in camera_series.items():
        timestamps = [entry.timestamp_ns for entry in series]
        sampled[camera_name] = resample_latest_indices(timestamps, sample_times)
    return sampled


def resize_rgb_frame(
    cv2_module: Any,
    np_module: Any,
    image_rgb: Any,
    resize_width: int,
    resize_height: int,
) -> Any:
    resized = cv2_module.resize(image_rgb, (resize_width, resize_height), interpolation=cv2_module.INTER_LANCZOS4)
    if resized.ndim != 3 or resized.shape[2] != 3:
        raise ValueError(f"Decoded camera frame has unexpected shape: {tuple(resized.shape)}")
    return np_module.ascontiguousarray(resized.transpose(2, 0, 1))


def decode_still_camera_frame(
    typestore: Any,
    cv2_module: Any,
    np_module: Any,
    entry: SerializedBagMessage,
    resize_width: int,
    resize_height: int,
) -> Any:
    try:
        msg = deserialize_cdr(typestore, entry.rawdata, entry.msgtype)
    except Exception as exc:
        raise ValueError(f"Failed to deserialize compressed image message ({entry.msgtype}): {exc}") from exc

    if not hasattr(msg, "data") or msg.data is None:
        raise ValueError(f"Compressed image message has no data payload ({entry.msgtype})")

    np_buffer = np_module.frombuffer(msg.data, np_module.uint8)
    image_bgr = cv2_module.imdecode(np_buffer, cv2_module.IMREAD_COLOR)
    if image_bgr is None:
        raise ValueError("Failed to decode compressed image frame from rosbag")

    image_rgb = cv2_module.cvtColor(image_bgr, cv2_module.COLOR_BGR2RGB)
    return resize_rgb_frame(cv2_module, np_module, image_rgb, resize_width, resize_height)


def infer_camera_decoder_kind(typestore: Any, series: list[SerializedBagMessage]) -> tuple[str, str | None]:
    for entry in series:
        try:
            msg = deserialize_cdr(typestore, entry.rawdata, entry.msgtype)
        except Exception:
            continue

        format_name = str(getattr(msg, "format", "") or "").strip().lower()
        if "h264" in format_name or "avc" in format_name:
            return "h26x", "h264"
        if "h265" in format_name or "hevc" in format_name:
            return "h26x", "hevc"

        return "compressed_image", None

    return "compressed_image", None


def create_camera_stream_state(
    typestore: Any,
    av_module: Any,
    camera_name: str,
    series: list[SerializedBagMessage],
) -> CameraStreamState:
    decoder_kind, codec_name = infer_camera_decoder_kind(typestore, series)
    if decoder_kind == "h26x" and av_module is None:
        raise ImportError(
            f"Camera {camera_name} uses {codec_name or 'h26x'} compressed frames, but PyAV is not installed. "
            "Install with: pip install av"
        )
    return CameraStreamState(camera_name=camera_name, series=series, decoder_kind=decoder_kind, codec_name=codec_name)


def resolve_camera_frame(
    camera_state: CameraStreamState,
    typestore: Any,
    av_module: Any,
    cv2_module: Any,
    np_module: Any,
    sampled_index: int,
    resize_width: int,
    resize_height: int,
) -> Any | None:
    if camera_state.last_requested_index == sampled_index and camera_state.last_requested_frame is not None:
        return camera_state.last_requested_frame

    if camera_state.decoder_kind == "compressed_image":
        frame = decode_still_camera_frame(
            typestore,
            cv2_module,
            np_module,
            camera_state.series[sampled_index],
            resize_width,
            resize_height,
        )
        camera_state.last_requested_index = sampled_index
        camera_state.last_requested_frame = frame
        camera_state.last_decoded_frame = frame
        return frame

    if sampled_index < camera_state.next_decode_index - 1:
        raise ValueError(
            f"Camera {camera_state.camera_name} requires monotonic access for h26x decoding, "
            f"but received sampled index {sampled_index} after advancing to {camera_state.next_decode_index}."
        )

    if camera_state.codec_context is None:
        camera_state.codec_context = av_module.CodecContext.create(camera_state.codec_name or "h264", "r")

    while camera_state.next_decode_index <= sampled_index:
        entry = camera_state.series[camera_state.next_decode_index]
        msg = deserialize_cdr(typestore, entry.rawdata, entry.msgtype)
        payload = getattr(msg, "data", None)
        if payload is not None and len(payload) > 0:
            packet = av_module.Packet(bytes(payload))
            try:
                frames = camera_state.codec_context.decode(packet)
            except Exception:
                frames = []

            if frames:
                image_rgb = frames[-1].to_ndarray(format="rgb24")
                camera_state.last_decoded_frame = resize_rgb_frame(
                    cv2_module,
                    np_module,
                    image_rgb,
                    resize_width,
                    resize_height,
                )
        camera_state.next_decode_index += 1

    camera_state.last_requested_index = sampled_index
    camera_state.last_requested_frame = camera_state.last_decoded_frame
    return camera_state.last_requested_frame


def print_series_summary(
    observation_series: dict[str, list[tuple[int, dict[str, Any]]]],
    camera_series: dict[str, list[SerializedBagMessage]],
    observation_sources: dict[str, str | None],
    camera_sources: dict[str, str | None],
) -> None:
    print("\nDetected observation topics:")
    for field_name, series in observation_series.items():
        source_topic = observation_sources.get(field_name) or "missing"
        print(f"  - {field_name}: {len(series)} messages (source: {source_topic})")

    print("\nDetected camera topics:")
    for camera_name, series in camera_series.items():
        source_topic = camera_sources.get(camera_name) or "missing"
        print(f"  - {camera_name}: {len(series)} messages (source: {source_topic})")


def print_warning_summary(scan_stats: BagScanStats) -> None:
    has_warnings = (
        scan_stats.skipped_observation_messages > 0
        or scan_stats.empty_camera_messages > 0
        or scan_stats.camera_decode_failures > 0
    )
    if not has_warnings:
        return

    print("\nWarning summary:")
    if scan_stats.skipped_observation_messages:
        print(f"  - Skipped unreadable observation messages: {scan_stats.skipped_observation_messages}")
        for topic_name, count in sorted(scan_stats.skipped_observation_topics.items()):
            print(f"    * {topic_name}: {count}")
    if scan_stats.empty_camera_messages:
        print(f"  - Skipped empty camera messages: {scan_stats.empty_camera_messages}")
        for topic_name, count in sorted(scan_stats.empty_camera_topics.items()):
            print(f"    * {topic_name}: {count}")
    if scan_stats.camera_decode_failures:
        print(f"  - Camera frames replaced with black images: {scan_stats.camera_decode_failures}")
        for camera_name, count in sorted(scan_stats.camera_decode_topics.items()):
            print(f"    * {camera_name}: {count}")


def convert_rosbag(args: argparse.Namespace) -> int:
    validate_args(args)
    config = load_conversion_config(args.config)
    input_robot_type = args.robot_type or config.robot_type
    canonical_type = canonical_robot_type(input_robot_type, config)
    active_state_names = state_names(config)
    active_action_names = action_names(config)
    with contextlib.ExitStack() as exit_stack:
        bag_dir, input_path = prepare_bag_path(args.bag_path, exit_stack)
        task_meta = discover_sidecar_json(bag_dir, args.task_meta, "task_meta.json", "task metadata")
        step_index = discover_sidecar_json(bag_dir, args.step_index, "step_index.json", "step index")
        step_index_for_episode = step_index if args.episode_policy == "step" else None

        runtime = import_runtime_dependencies(require_lerobot=not args.dry_run)
        av_module = runtime["av"]
        cv2_module = runtime["cv2"]
        np_module = runtime["np"]
        reader_cls = runtime["Reader"]
        stores = runtime["Stores"]
        get_typestore = runtime["get_typestore"]

        typestore = get_typestore(stores.ROS2_JAZZY)

        print("Scanning rosbag2 data...")
        print(f"  Input path: {input_path}")
        print(f"  Prepared bag path: {bag_dir}")
        print(f"  Conversion config: {config.profile}")
        if args.config:
            print(f"  Config path: {resolve_config_path(args.config)}")
        print(f"  Input robot type: {input_robot_type}")
        print(f"  Canonical robot type: {canonical_type}")
        print(f"  State fields: {len(active_state_names)}")
        print(f"  Action mode: {config.action_mode}")
        print(f"  Action fields: {len(active_action_names)}")
        if task_meta:
            print("  Task metadata: found")
        if step_index:
            step_count = len(step_index.get("steps", [])) if isinstance(step_index.get("steps"), list) else 0
            print(f"  Step index: found ({step_count} steps)")
        print(f"  Episode policy: {args.episode_policy}")
        if args.use_videos:
            print(f"  Output video codec: {args.video_codec or 'libsvtav1 (LeRobot default)'}")

        observation_series, camera_series, scan_stats, observation_sources, camera_sources = collect_bag_data(
            reader_cls,
            typestore,
            bag_dir,
            config,
        )
        print_series_summary(observation_series, camera_series, observation_sources, camera_sources)
        validate_required_topics(observation_series, camera_series, config)

        active_camera_series = {camera_name: series for camera_name, series in camera_series.items() if series}
        active_camera_sources = {
            camera_name: source_topic
            for camera_name, source_topic in camera_sources.items()
            if camera_name in active_camera_series
        }
        if not active_camera_series:
            warn("No camera topics were found in the rosbag. The output dataset will contain state/action only.")

        start_ns, end_ns = build_sampling_window(observation_series, active_camera_series)
        sample_times = build_sample_times(start_ns, end_ns, args.fps, args.max_frames)
        sampled_observations = sample_observation_series(observation_series, sample_times)
        sampled_camera_entries = sample_camera_indices(active_camera_series, sample_times)

        state_vectors = [build_state_vector(sample, config, np_module) for sample in sampled_observations]
        if config.action_mode == ACTION_MODE_FROM_SOURCES:
            source_action_vectors = [
                build_action_vector(sample, config, np_module) for sample in sampled_observations
            ]
        else:
            source_action_vectors = None

        duration_sec = (sample_times[-1] - sample_times[0]) / 1e9 if len(sample_times) > 1 else 0.0
        episode_specs = build_episode_specs(sample_times, bag_dir.name, args.task, task_meta, step_index_for_episode)
        print("\nSampling summary:")
        print(f"  Start time (ns): {start_ns}")
        print(f"  End time (ns): {end_ns}")
        print(f"  Target FPS: {args.fps}")
        print(f"  Sampled frames: {len(sample_times)}")
        print(f"  Duration: {duration_sec:.2f}s")
        print(f"  Episodes: {len(episode_specs)}")
        for episode_number, episode_spec in enumerate(episode_specs):
            frames = episode_spec.end_index - episode_spec.start_index
            step_label = f", step {episode_spec.step_number}" if episode_spec.step_number is not None else ""
            print(f"    - episode {episode_number}: {frames} frames{step_label}, task: {episode_spec.task}")
        print_warning_summary(scan_stats)

        if args.dry_run:
            print("\nDry run completed. No LeRobot files were written.")
            return 0

        torch_module = runtime["torch"]
        lerobot_dataset_cls = runtime["LeRobotDataset"]
        configure_lerobot_video_codec(runtime, args.video_codec)
        patch_lerobot_image_stats()
        active_camera_names = list(active_camera_series.keys())
        dataset = create_lerobot_dataset(
            output_dir=args.output_dir,
            repo_id=args.repo_id,
            robot_type=canonical_type,
            fps=args.fps,
            state_names=active_state_names,
            action_names=active_action_names,
            camera_names=active_camera_names,
            resize_width=args.resize_width,
            resize_height=args.resize_height,
            use_videos=args.use_videos,
            video_backend=args.video_backend,
            lerobot_dataset_cls=lerobot_dataset_cls,
        )

        camera_states = {
            camera_name: create_camera_stream_state(typestore, av_module, camera_name, series)
            for camera_name, series in active_camera_series.items()
        }
        black_image = np_module.zeros((3, args.resize_height, args.resize_width), dtype=np_module.uint8)

        print("\nWriting LeRobot dataset...")
        processed_frames = 0
        total_episode_frames = sum(spec.end_index - spec.start_index for spec in episode_specs)
        for episode_number, episode_spec in enumerate(episode_specs):
            print(f"  Writing episode {episode_number + 1}/{len(episode_specs)}")
            for frame_index in range(episode_spec.start_index, episode_spec.end_index):
                state_vector = state_vectors[frame_index]
                if source_action_vectors is not None:
                    action_vector = source_action_vectors[frame_index]
                else:
                    next_index = min(frame_index + 1, episode_spec.end_index - 1)
                    action_vector = state_vectors[next_index]

                frame = {
                    "task": episode_spec.task,
                    "observation.state": torch_module.from_numpy(state_vector.copy()).type(torch_module.float32),
                    "action": torch_module.from_numpy(action_vector.copy()).type(torch_module.float32),
                }

                for camera_name, series in active_camera_series.items():
                    sampled_index = sampled_camera_entries[camera_name][frame_index]
                    if sampled_index is None:
                        frame[f"observation.images.{camera_name}"] = black_image
                        continue

                    try:
                        decoded_frame = resolve_camera_frame(
                            camera_states[camera_name],
                            typestore,
                            av_module,
                            cv2_module,
                            np_module,
                            sampled_index,
                            args.resize_width,
                            args.resize_height,
                        )
                    except Exception as exc:
                        scan_stats.record_camera_decode_failure(camera_name)
                        warn(
                            f"Using black frame for {camera_name} at sampled frame {frame_index} "
                            f"(source index {sampled_index}): {exc}"
                        )
                        frame[f"observation.images.{camera_name}"] = black_image
                        continue

                    if decoded_frame is None:
                        scan_stats.record_camera_decode_failure(camera_name)
                        frame[f"observation.images.{camera_name}"] = black_image
                        continue

                    frame[f"observation.images.{camera_name}"] = decoded_frame

                dataset.add_frame(frame)
                processed_frames += 1

                if processed_frames % 100 == 0 or processed_frames == total_episode_frames:
                    print(f"  Processed {processed_frames}/{total_episode_frames} frames")

            dataset.save_episode()

        copy_conversion_sidecars(args.output_dir, task_meta, step_index)

        print("\nConversion completed.")
        print(f"  Output dir: {args.output_dir}")
        print(f"  Repo id: {args.repo_id}")
        print(f"  Robot type: {canonical_type}")
        print(f"  Cameras: {active_camera_names}")
        print(f"  Episodes written: {len(episode_specs)}")
        print(f"  Frames written: {processed_frames}")
        print_warning_summary(scan_stats)
        return 0


def main() -> int:
    args = parse_args()
    try:
        return convert_rosbag(args)
    except KeyboardInterrupt:  # pragma: no cover - CLI interrupt path
        print("\nInterrupted by user.", file=sys.stderr)
        return 130
    except Exception as exc:  # pragma: no cover - CLI error path
        print(f"\nError: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
