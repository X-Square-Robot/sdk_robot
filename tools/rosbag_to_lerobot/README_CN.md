# Rosbag → LeRobot V3 转换工具

将 rosbag2 采集包转换为 LeRobot V3 数据集的共享工具。
机器人的 topic 映射位于 `config/`（按机型分目录）。

英文版见 [README.md](./README.md)。

## 目录结构

```text
rosbag_to_lerobot/
  scripts/
    convert_platform_task_to_lerobot_v3.py   平台 task.json 入口
    convert_rosbag_to_lerobot_v3_batch.py    多任务 / 多文件批处理
    convert_rosbag_to_lerobot.py             单包转换
  config/
    quanta_x1/                              Quanta X1 配置
      lerobot_v3_16d.yaml
      lerobot_v3_16d_h26x.yaml
      lerobot_v3_16d_action_from_sources.yaml
    desktop/                                Desktop 配置
      lerobot_v3_16d.yaml
      lerobot_v3_16d_action_from_sources.yaml
    quanta_x2/                              Quanta X2 配置
      lerobot_v3_16d.yaml
      lerobot_v3_16d_action_from_sources.yaml
      g_gripper_lerobot_v3_16d.yaml
  requirements.txt
  README.md
  README_CN.md
```

## 安装依赖

请在本工具目录下使用**本地虚拟环境**。不要把依赖安装到系统 Python。

推荐 Python 3.12（LeRobot 0.4.2 / torch 需要受支持的 ABI；3.14 不可用）。

```bash
cd tools/rosbag_to_lerobot

# 创建 venv（仅需一次）
uv venv --python 3.12 .venv
# 或: python3.12 -m venv .venv

# 仅安装到 .venv
uv pip install \
  --python .venv/bin/python \
  --index-strategy unsafe-best-match \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  -r requirements.txt

# 若有本地 lerobot wheel:
# uv pip install --python .venv/bin/python /path/to/lerobot-0.4.2-py3-none-any.whl

# 始终用 venv 中的 python 运行脚本
.venv/bin/python scripts/convert_rosbag_to_lerobot.py --help
```

激活后使用普通 `pip`：

```bash
source .venv/bin/activate
python -m pip install \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  -r requirements.txt
```

## 机型 / 配置选择

转换用的 topic 映射完全由 `config/<project_or_model>/` 下的 `--config` YAML 决定。

| 机型 | 默认配置 | 说明 |
| --- | --- | --- |
| `quanta_x1` | `config/quanta_x1/lerobot_v3_16d.yaml` | 手臂末端位姿 + JointState 夹爪 |
| `desktop` | `config/desktop/lerobot_v3_16d.yaml` | 手臂末端位姿 + JointState 夹爪（无头部关节 topic） |
| `quanta_x2` | `config/quanta_x2/lerobot_v3_16d.yaml` | WBC 腕部位姿 + Float32 C 夹爪 |
| `quanta_x2` G 夹爪 | `config/quanta_x2/g_gripper_lerobot_v3_16d.yaml` | WBC 腕部位姿 + G 夹爪 JointState |
| `quanta_x1`（H.26x 流） | `config/quanta_x1/lerobot_v3_16d_h26x.yaml` | 优先使用 H.26x 相机流；通过 `--config` 指定 |

平台入口也可根据 `dataset.robotType` / `--robot-type`，
或显式的 `dataset.config` 路径选择默认配置。

```bash
# Quanta X1 默认
python3 scripts/convert_platform_task_to_lerobot_v3.py --task-json /path/to/task.json

# Quanta X2
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --robot-type quanta_x2

# 显式配置（例如 action 来自命令 topic）
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --config config/quanta_x2/lerobot_v3_16d_action_from_sources.yaml
```

## Action 模式

在 YAML 的 `action:` 段配置：

### `next_state`（默认）

```yaml
action:
  mode: next_state
  fields_from: state
```

同一 episode 内 `action[t] = state[t+1]`（最后一帧重复 `state[t]`）。

### `from_sources`

```yaml
action:
  mode: from_sources
  fields:
    - name: left_ee_pos_x
      source: left_arm_end_pose_cmd
      extractor: pose.position.x
    # ... 字段宽度需与 state.fields 一致
```

Action 值从观测 topic（通常为 command / cmd-echo）按与 state 相同的采样时间戳提取。
启用的 action 字段数量必须与 state 一致。

## 支持的观测 Decoder / Extractor

Decoder：`pose_stamped`、`joint_state`、`odometry`、`float32`、`float64`、`float64_multi_array`

常用 extractor：

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

## 入口方式

按是否已有单个 bag 路径或平台 `task.json`，有三种运行方式。

### 1. 直接指定 bag / 压缩包路径（单包）

本地已有 rosbag2 目录、`.mcap` / `.db3`，或 `.tar` / `.tar.gz` / `.tgz` 包时使用：

```bash
# Quanta X2 C 夹爪
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

# Quanta X2 G 夹爪
python3 scripts/convert_rosbag_to_lerobot.py \
  --bag-path /path/to/rosbag2_dir_or_mcap \
  --output-dir /path/to/output_lerobot_dataset \
  --repo-id my_org/my_dataset \
  --config config/quanta_x2/g_gripper_lerobot_v3_16d.yaml \
  --robot-type quanta_x2 \
  --use-videos \
  --video-codec h264

# 仅解析检查（不写 LeRobot 输出）
python3 scripts/convert_rosbag_to_lerobot.py \
  --bag-path /path/to/collection.tar \
  --output-dir /tmp/unused \
  --repo-id my_org/probe \
  --config config/quanta_x2/g_gripper_lerobot_v3_16d.yaml \
  --robot-type quanta_x2 \
  --dry-run
```

`--bag-path` 支持：

```text
rosbag2 目录（含 metadata.yaml）
metadata.yaml
.mcap / .db3（父目录须含 metadata.yaml）
.tar / .tar.gz / .tgz（归档内含 rosbag2 目录）
```

### 2. 平台 task.json（推荐用于批处理 / 平台对接）

先将远端 bag/tar 下载到本地，再生成一份 `task.json` 并调用：

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json
```

同时打包输出数据集为 tar.gz：

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --archive-path /path/to/output_lerobot_dataset.tar.gz
```

### 3. Manifest 批处理

若有内部多任务 / 多文件的 YAML/JSON manifest，可直接调用
`convert_rosbag_to_lerobot_v3_batch.py`（参数见下文「批处理转换参数」）。

## task.json 结构

推荐 camelCase 字段：

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

`task` 会写入：

```text
meta/tasks.parquet.task
meta/episodes/chunk-*/file-*.parquet.tasks
```

每一帧的 `task_index` 都指向 `meta/tasks.parquet`。

若省略 `task`，脚本会在可用时用 `taskName`、`actionDesc`、`sceneDesc` 拼出描述。

`dataset` 中可选覆盖：

- `config` / `conversionConfig`：显式转换 YAML 路径
- `robotType` / `model`：未指定 `config` 时用于选择默认 YAML

## 默认语义

```text
一条平台任务描述 -> 一个 task_index
一个采集文件     -> 一个 LeRobot episode
同一任务下多个文件 -> 多个 episode，共享同一 task_index
多个平台任务     -> 合并为一个 LeRobot V3 数据集
```

默认 `episodePolicy` 为 `file`。

若采集包内含 `step_index.json`，也支持 `episodePolicy=step`。
该模式下一个采集包可按 step 元数据拆成多个 LeRobot episode。

## 并发

默认串行转换：

```text
jobs = 1
```

大批量时可按 CPU、内存与磁盘 IO 能力显式设置并发：

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --jobs 2
```

也可在 `task.json` 中设置 `dataset.jobs`。

并发仅作用于中间的「按源文件」转换阶段：

```text
源 bag/tar -> 临时单文件 LeRobot 数据集
```

最终合并始终按原任务 / 文件顺序串行执行，保证 `episode_index`、`task_index`
与视频文件索引确定性。

## 临时工作目录

大批量建议使用本地 SSD 上的临时目录：

```bash
python3 scripts/convert_platform_task_to_lerobot_v3.py \
  --task-json /path/to/task.json \
  --work-dir /path/to/fast_local_tmp
```

## 单包转换参数

脚本：`scripts/convert_rosbag_to_lerobot.py`

| 参数 | 必填 | 说明 |
| --- | --- | --- |
| `--bag-path` | 是 | 本地 rosbag2 目录、`metadata.yaml`、`.mcap` / `.db3`，或 `.tar` / `.tar.gz` / `.tgz`。 |
| `--output-dir` | 是 | 输出 LeRobot 数据集目录；须为空或不存在（`--dry-run` 时忽略）。 |
| `--repo-id` | 是 | 数据集 repo ID，例如 `my_org/my_dataset`。 |
| `--config` | 否 | topic / state / action YAML。省略时默认使用 `config/quanta_x1/lerobot_v3_16d.yaml`。 |
| `--robot-type` | 否 | 机型别名覆盖；默认取配置中的 `robot_type`。 |
| `--fps` | 否 | 重采样帧率；默认 `30`。 |
| `--resize-width` / `--resize-height` | 否 | 输出图像尺寸；本脚本默认 `640x480`。 |
| `--task` | 否 | 写入帧的任务字符串。 |
| `--episode-policy` | 否 | `step` 或 `file`；默认 `step`。 |
| `--use-videos` | 否 | 写视频特征而非逐帧图像。 |
| `--video-codec` | 否 | `h264`、`hevc` 或 `libsvtav1`；需配合 `--use-videos`。 |
| `--video-backend` | 否 | 可选 LeRobot 视频后端：`pyav` 或 `opencv`。 |
| `--max-frames` | 否 | 可选帧数上限，用于转换检查。 |
| `--dry-run` | 否 | 扫描 / 解析 bag 并打印摘要，不写 LeRobot 输出。 |

## 平台入口参数

脚本：`scripts/convert_platform_task_to_lerobot_v3.py`

| 参数 | 必填 | 说明 |
| --- | --- | --- |
| `--task-json` | 是 | 含任务描述与本地 bag/tar 路径的平台任务 JSON。 |
| `--output-dir` | 否 | 覆盖 `dataset.outputDir`。 |
| `--archive-path` | 否 | 覆盖 `dataset.archivePath`；须以 `.tar.gz` 或 `.tgz` 结尾。 |
| `--config` | 否 | 覆盖转换配置路径。 |
| `--robot-type` | 否 | 未指定 `--config` 时用于选择默认配置的机型别名。 |
| `--batch-converter` | 否 | 覆盖 `convert_rosbag_to_lerobot_v3_batch.py` 路径。 |
| `--single-converter` | 否 | 覆盖 `convert_rosbag_to_lerobot.py` 路径。 |
| `--work-dir` | 否 | 覆盖临时转换工作目录。 |
| `--jobs` | 否 | 并发转换的源文件数。默认取 `dataset.jobs`，再默认 `1`（串行）。 |
| `--print-manifest-only` | 否 | 仅打印规范化后的内部 manifest 后退出，不转换。 |

## 批处理转换参数

脚本：`scripts/convert_rosbag_to_lerobot_v3_batch.py`

| 参数 | 必填 | 说明 |
| --- | --- | --- |
| `--manifest` | 是 | 描述任务与源文件的 YAML 或 JSON manifest。 |
| `--output-dir` | 是 | 最终 LeRobot V3 数据集输出目录；目录内不能已有文件。 |
| `--config` | 是 | topic / state / action 配置，例如 `config/quanta_x1/lerobot_v3_16d.yaml`。 |
| `--repo-id` | 否 | 覆盖 manifest 的 `dataset.repo_id`。 |
| `--fps` | 否 | 覆盖 manifest fps。 |
| `--resize-width` | 否 | 覆盖输出视频宽度。 |
| `--resize-height` | 否 | 覆盖输出视频高度。 |
| `--video-codec` | 否 | `h264`、`hevc` 或 `libsvtav1`；默认 `h264`。 |
| `--video-backend` | 否 | 可选 LeRobot 视频后端：`pyav` 或 `opencv`。 |
| `--episode-policy` | 否 | `file` 或 `step`；默认取 manifest `dataset.episode_policy`，再默认 `file`。 |
| `--max-frames-per-file` | 否 | 每个源文件的可选帧数上限，用于转换检查。 |
| `--converter` | 否 | `convert_rosbag_to_lerobot.py` 路径；默认同目录脚本。 |
| `--work-dir` | 否 | 临时工作目录。 |
| `--jobs` | 否 | 并发转换的源文件数。默认取 manifest `dataset.jobs`，再默认 `1`（串行）。 |
| `--keep-temp` | 否 | 保留每个源文件的中间数据集与日志，便于调试。 |
| `--write-source-manifest` | 否 | 写入可选的 `meta/source_manifest.json`；默认关闭。 |

## 输入要求

支持的源路径（经 `--bag-path` 或 `task.json` 的 `filePath`）：

```text
rosbag2 目录
metadata.yaml
.tar, .tar.gz, .tgz
.mcap, .db3
```

rosbag2 tar 包内须包含 rosbag2 元数据与数据文件，例如：

```text
metadata.yaml
*.db3 或 *.mcap 或 *.mcap.zstd
```

单独的 `.mcap` / `.db3` 必须与同目录下的 `metadata.yaml` 放在一起。

默认 16D 配置需要左右臂末端位姿、左右夹爪状态、可选头部关节状态，
以及存在时的 RGB 相机流。具体 topic 名以各机型 YAML 为准。

## 输出结构（LeRobot V3）

真实 Quanta X2 G 夹爪转换示例（`--use-videos`，1 个 episode / 822 帧）：

```text
output_lerobot_dataset/
  meta/
    info.json                 # 数据集 schema、fps、robot_type、特征形状
    stats.json                # 各特征 mean/std/min/max
    tasks.parquet             # task_index -> 任务文本
    episodes/
      chunk-000/
        file-000.parquet      # episode 级行（长度、task、时间戳等）
  data/
    chunk-000/
      file-000.parquet        # 帧行：observation.state、action、索引等
  videos/
    observation.images.wrist_left/
      chunk-000/
        file-000.mp4
    observation.images.wrist_right/
      chunk-000/
        file-000.mp4
```

16D 配置下典型的 `meta/info.json` 要点：

```text
codebase_version: v3.0
robot_type:       quanta_x2
fps:              30
total_episodes:   N
total_frames:     M
features:
  observation.state                        float32  [16]
  action                                   float32  [16]
  observation.images.<camera>              video    [3, H, W]   # 使用 --use-videos 时
```

说明：

- 使用 `--use-videos` 时在 `videos/` 下写 mp4；否则以图像帧存储。
- 相机 key 来自 YAML `topics.cameras` 的 name（如 `wrist_left`、`head`）。
- 输出数据集不包含 `raw/`。
- 工具不会合成相机标定文件；若需要，`meta/calibration` 应来自真实标定数据。
- 编码过程中可能出现临时 `images/`，视频写完后可能为空。
