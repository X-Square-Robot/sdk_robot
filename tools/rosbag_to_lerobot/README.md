# Rosbag → LeRobot V3 Conversion Tools

Shared tools for converting rosbag2 collection packages into a LeRobot V3 dataset.
Robot topic mappings live under `config/` (one directory per robot model).

中文版见 [README_CN.md](./README_CN.md)。

## Layout

```text
rosbag_to_lerobot/
  scripts/
    convert_platform_task_to_lerobot_v3.py   Platform task.json entrypoint
    convert_rosbag_to_lerobot_v3_batch.py    Multi-task/multi-file batch converter
    convert_rosbag_to_lerobot.py             Single package converter
  config/
    quanta_x1/                              Quanta X1 configs
      lerobot_v3_16d.yaml
      lerobot_v3_16d_h26x.yaml
      lerobot_v3_16d_action_from_sources.yaml
    desktop/                                Desktop configs
      lerobot_v3_16d.yaml
      lerobot_v3_16d_action_from_sources.yaml
    quanta_x2/                              Quanta X2 configs
      lerobot_v3_16d.yaml
      lerobot_v3_16d_action_from_sources.yaml
      g_gripper_lerobot_v3_16d.yaml
  requirements.txt
  README.md
  README_CN.md
```

## Install Dependencies

Use a **local virtualenv** under this tool directory. Do **not** `pip install` into the system Python.

Python 3.12 is recommended (LeRobot 0.4.2 / torch need a supported ABI; 3.14 is not).

```bash
cd tools/rosbag_to_lerobot

# create venv (once)
uv venv --python 3.12 .venv
# or: python3.12 -m venv .venv

# install deps into .venv only
uv pip install \
  --python .venv/bin/python \
  --index-strategy unsafe-best-match \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  -r requirements.txt

# if you have a local lerobot wheel:
# uv pip install --python .venv/bin/python /path/to/lerobot-0.4.2-py3-none-any.whl

# always run scripts with the venv python
.venv/bin/python scripts/convert_rosbag_to_lerobot.py --help
```

With plain `pip` after activating:

```bash
source .venv/bin/activate
python -m pip install \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  -r requirements.txt
```

## Robot / Config Selection

Conversion topic mapping is entirely driven by `--config` YAML under `config/<project_or_model>/`.

| Robot | Default config | Notes |
| --- | --- | --- |
| `quanta_x1` | `config/quanta_x1/lerobot_v3_16d.yaml` | Arm end-pose + JointState grippers |
| `desktop` | `config/desktop/lerobot_v3_16d.yaml` | Arm end-pose + JointState grippers (no head joint topics) |
| `quanta_x2` | `config/quanta_x2/lerobot_v3_16d.yaml` | WBC wrist pose + Float32 C-gripper |
| `quanta_x2` G-gripper | `config/quanta_x2/g_gripper_lerobot_v3_16d.yaml` | WBC wrist pose + G-gripper JointState |
| `quanta_x1` (H.26x streams) | `config/quanta_x1/lerobot_v3_16d_h26x.yaml` | Prefers H.26x camera streams; pass via `--config` |

Platform entrypoint can also pick the default from `dataset.robotType` / `--robot-type`,
or an explicit `dataset.config` path.

```bash
# Quanta X1 default
python3 scripts/convert_platform_task_to_lerobot_v3.py --task-json /path/to/task.json

# Quanta X2
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --robot-type quanta_x2

# Explicit config (e.g. action from command topics)
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --config config/quanta_x2/lerobot_v3_16d_action_from_sources.yaml
```

## Action Modes

Configured under the YAML `action:` section:

### `next_state` (default)

```yaml
action:
  mode: next_state
  fields_from: state
```

`action[t] = state[t+1]` within each episode (last frame repeats `state[t]`).

### `from_sources`

```yaml
action:
  mode: from_sources
  fields:
    - name: left_ee_pos_x
      source: left_arm_end_pose_cmd
      extractor: pose.position.x
    # ... same width as state.fields
```

Action values are extracted from observation topics (typically command / cmd-echo topics)
at the same sample timestamps as state. Enabled action field count must match state.

## Supported Observation Decoders / Extractors

Decoders: `pose_stamped`, `joint_state`, `odometry`, `float32`, `float64`, `float64_multi_array`

Common extractors:

```text
pose.position.{x,y,z}
pose.euler.{roll,pitch,yaw}
joint.position[N]
float.value
array[N]
odom.position.{x,y}
odom.euler.yaw
constant:<float>
```

## Entrypoints

There are three ways to run conversion, depending on whether you have a single bag path or a platform `task.json`.

### 1. Direct bag / archive path (single package)

Use this when you already have a local rosbag2 directory, `.mcap` / `.db3`, or `.tar` / `.tar.gz` / `.tgz` package:

```bash
# Quanta X2 C-gripper
python3 scripts/convert_rosbag_to_lerobot.py \
  --bag-path /path/to/collection.tar \
  --output-dir /path/to/output_lerobot_dataset \
  --repo-id my_org/my_dataset \
  --config config/quanta_x2/lerobot_v3_16d.yaml \
  --robot-type quanta_x2 \
  --use-videos \
  --video-codec h264 \
  --fps 30 \
  --resize-width 1280 \
  --resize-height 720

# Quanta X2 G-gripper
python3 scripts/convert_rosbag_to_lerobot.py \
  --bag-path /path/to/rosbag2_dir_or_mcap \
  --output-dir /path/to/output_lerobot_dataset \
  --repo-id my_org/my_dataset \
  --config config/quanta_x2/g_gripper_lerobot_v3_16d.yaml \
  --robot-type quanta_x2 \
  --use-videos \
  --video-codec h264

# Parse-only check (no LeRobot output written)
python3 scripts/convert_rosbag_to_lerobot.py \
  --bag-path /path/to/collection.tar \
  --output-dir /tmp/unused \
  --repo-id my_org/probe \
  --config config/quanta_x2/g_gripper_lerobot_v3_16d.yaml \
  --robot-type quanta_x2 \
  --dry-run
```

`--bag-path` accepts:

```text
rosbag2 directory (with metadata.yaml)
metadata.yaml
.mcap / .db3 (parent directory must contain metadata.yaml)
.tar / .tar.gz / .tgz (archive containing a rosbag2 directory)
```

### 2. Platform task.json (recommended for batch / platform integration)

Download remote bag/tar links to local paths first, then generate one `task.json` and call:

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json
```

To also create a tar.gz archive of the output dataset:

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --archive-path /path/to/output_lerobot_dataset.tar.gz
```

### 3. Manifest batch

For an internal YAML/JSON manifest of multiple tasks/files, call `convert_rosbag_to_lerobot_v3_batch.py` directly (see Batch Converter Parameters below).

## task.json Shape

Recommended camelCase fields:

```json
{
  "schemaVersion": "platform_lerobot_v3_task_v1",
  "dataset": {
    "repoId": "x2robot/platform_task_package",
    "outputDir": "/path/to/output_lerobot_dataset",
    "archivePath": "/path/to/output_lerobot_dataset.tar.gz",
    "robotType": "quanta_x2",
    "fps": 30,
    "resizeWidth": 1280,
    "resizeHeight": 720,
    "stateActionWidth": 16,
    "episodePolicy": "file",
    "videoCodec": "h264",
    "jobs": 1
  },
  "taskList": [
    {
      "taskId": "TASK_A",
      "task": "平台下发任务A：完整任务描述",
      "bagList": [
        {
          "dataId": "DATA_A1",
          "filePath": "/downloaded/path/task_a_file_001.tar",
          "fileType": "tar"
        }
      ]
    }
  ]
}
```

`task` is written to:

```text
meta/tasks.parquet.task
meta/episodes/chunk-*/file-*.parquet.tasks
```

Every frame-level `task_index` points to `meta/tasks.parquet`.

If `task` is omitted, the script builds the description from `taskName`, `actionDesc`, and `sceneDesc` when available.

Optional overrides in `dataset`:

- `config` / `conversionConfig`: explicit conversion YAML path
- `robotType` / `model`: selects a default YAML when `config` is omitted

## Default Semantics

```text
one platform task description -> one task_index
one collection file           -> one LeRobot episode
multiple files under one task  -> multiple episodes with the same task_index
multiple platform tasks        -> one merged LeRobot V3 dataset
```

The default `episodePolicy` is `file`.

`episodePolicy=step` is also supported for collection packages containing `step_index.json`. In that mode, one collection package may be split into multiple LeRobot episodes by step metadata.

## Concurrency

The default is serial conversion:

```text
jobs = 1
```

For larger batches, the caller can explicitly set concurrency based on CPU, memory, and disk IO capacity:

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --jobs 2
```

`jobs` can also be set in `task.json` as `dataset.jobs`.

Concurrency only applies to the intermediate per-source conversion stage:

```text
source bag/tar -> temporary single-file LeRobot dataset
```

The final merge always runs serially in the original task/file order, so `episode_index`, `task_index`, and video file indexes remain deterministic.

## Temporary Workspace

For large batches, use a local SSD-backed temporary workspace:

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --work-dir /path/to/fast_local_tmp
```

## Single Converter Parameters

Script: `scripts/convert_rosbag_to_lerobot.py`

| Parameter | Required | Description |
| --- | --- | --- |
| `--bag-path` | yes | Local rosbag2 directory, `metadata.yaml`, `.mcap` / `.db3`, or `.tar` / `.tar.gz` / `.tgz`. |
| `--output-dir` | yes | Output LeRobot dataset directory; must be empty or non-existent (ignored with `--dry-run`). |
| `--repo-id` | yes | Dataset repo ID, e.g. `my_org/my_dataset`. |
| `--config` | no | Topic/state/action YAML. Defaults to `config/quanta_x1/lerobot_v3_16d.yaml`. |
| `--robot-type` | no | Robot alias override; defaults to `robot_type` from config. |
| `--fps` | no | Resample rate; default `30`. |
| `--resize-width` / `--resize-height` | no | Output image size; defaults `640x480` for this script. |
| `--task` | no | Task string stored on frames. |
| `--episode-policy` | no | `step` or `file`; default `step`. |
| `--use-videos` | no | Write video features instead of image frames. |
| `--video-codec` | no | `h264`, `hevc`, or `libsvtav1`; requires `--use-videos`. |
| `--video-backend` | no | Optional LeRobot video backend: `pyav` or `opencv`. |
| `--max-frames` | no | Optional frame limit for conversion checks. |
| `--dry-run` | no | Scan/parse bag and print summary without writing LeRobot output. |

## Platform Entrypoint Parameters

Script: `scripts/convert_platform_task_to_lerobot_v3.py`

| Parameter | Required | Description |
| --- | --- | --- |
| `--task-json` | yes | Platform task JSON containing task descriptions and local bag/tar paths. |
| `--output-dir` | no | Override `dataset.outputDir`. |
| `--archive-path` | no | Override `dataset.archivePath`; must end with `.tar.gz` or `.tgz`. |
| `--config` | no | Override conversion config path. |
| `--robot-type` | no | Robot alias used to choose a default config when `--config` is omitted. |
| `--batch-converter` | no | Override path to `convert_rosbag_to_lerobot_v3_batch.py`. |
| `--single-converter` | no | Override path to `convert_rosbag_to_lerobot.py`. |
| `--work-dir` | no | Override temporary conversion workspace. |
| `--jobs` | no | Number of source files to convert concurrently. Defaults to `dataset.jobs`, then `1` (serial). |
| `--print-manifest-only` | no | Print normalized internal manifest and exit without conversion. |

## Batch Converter Parameters

Script: `scripts/convert_rosbag_to_lerobot_v3_batch.py`

| Parameter | Required | Description |
| --- | --- | --- |
| `--manifest` | yes | YAML or JSON manifest describing tasks and source files. |
| `--output-dir` | yes | Final LeRobot V3 dataset output directory; must not already contain files. |
| `--config` | yes | Topic/state/action config, e.g. `config/quanta_x1/lerobot_v3_16d.yaml`. |
| `--repo-id` | no | Override manifest `dataset.repo_id`. |
| `--fps` | no | Override manifest fps. |
| `--resize-width` | no | Override output video width. |
| `--resize-height` | no | Override output video height. |
| `--video-codec` | no | `h264`, `hevc`, or `libsvtav1`; default is `h264`. |
| `--video-backend` | no | Optional LeRobot video backend: `pyav` or `opencv`. |
| `--episode-policy` | no | `file` or `step`; defaults to manifest `dataset.episode_policy`, then `file`. |
| `--max-frames-per-file` | no | Optional frame limit per source file for conversion checks. |
| `--converter` | no | Path to `convert_rosbag_to_lerobot.py`; defaults to the sibling script. |
| `--work-dir` | no | Temporary workspace directory. |
| `--jobs` | no | Number of source files to convert concurrently. Defaults to manifest `dataset.jobs`, then `1` (serial). |
| `--keep-temp` | no | Keep per-file intermediate datasets and logs for debugging. |
| `--write-source-manifest` | no | Write optional `meta/source_manifest.json`; disabled by default. |

## Input Requirements

Supported source paths (via `--bag-path` or `task.json` `filePath`):

```text
rosbag2 directory
metadata.yaml
.tar, .tar.gz, .tgz
.mcap, .db3
```

For rosbag2 tar packages, the package must contain rosbag2 metadata and data files, for example:

```text
metadata.yaml
*.db3 or *.mcap or *.mcap.zstd
```

A bare `.mcap` / `.db3` file must sit next to `metadata.yaml` in the same directory.

Default 16D configs require left/right arm end pose, left/right gripper state, optional head joint state, and RGB camera streams when present. Exact topic names depend on the robot YAML.

## Output Layout (LeRobot V3)

Example from a real Quanta X2 G-gripper conversion (`--use-videos`, 1 episode / 822 frames):

```text
output_lerobot_dataset/
  meta/
    info.json                 # dataset schema, fps, robot_type, feature shapes
    stats.json                # per-feature mean/std/min/max
    tasks.parquet             # task_index -> task text
    episodes/
      chunk-000/
        file-000.parquet      # episode-level rows (length, task, timestamps, ...)
  data/
    chunk-000/
      file-000.parquet        # frame rows: observation.state, action, indexes, ...
  videos/
    observation.images.wrist_left/
      chunk-000/
        file-000.mp4
    observation.images.wrist_right/
      chunk-000/
        file-000.mp4
```

Typical `meta/info.json` highlights for the 16D configs:

```text
codebase_version: v3.0
robot_type:       quanta_x2
fps:              30
total_episodes:   N
total_frames:     M
features:
  observation.state                        float32  [16]
  action                                   float32  [16]
  observation.images.<camera>              video    [3, H, W]   # when --use-videos
```

Notes:

- Use `--use-videos` for mp4 under `videos/`; without it, frames are stored as images instead.
- Camera keys come from the YAML `topics.cameras` names (e.g. `wrist_left`, `head`).
- Output datasets do not include `raw/`.
- The tools do not synthesize camera calibration files; `meta/calibration` should come from real calibration data if required.
- Temporary `images/` may appear during encoding and can be empty after videos are written.
