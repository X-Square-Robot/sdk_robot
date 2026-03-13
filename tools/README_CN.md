# convert_to_lerobot.py

将采集的机器人数据转换为 [LeRobot](https://github.com/huggingface/lerobot) 数据集格式，用于模仿学习和机器人策略训练。

## 功能特性

- **机型支持**：quanta_x1（带主臂）、quanta_x2（无主臂）、desktop（仅手臂+夹爪）
- **数据来源**：
   - 关节状态
   - 末端执行器位姿（位置 + 姿态）
   - 动作（多模式：master_arm_control、vr_control 等）
   - 图像（JPG 或 MP4 视频）
   - 额外传感器：里程计、底盘 IMU、位姿、触觉等

## 依赖安装

下载[v0.4.2版本的lerobot安装包](https://github.com/huggingface/lerobot/releases/download/v0.4.2/lerobot-0.4.2-py3-none-any.whl)

```bash
pip install lerobot-0.4.2-py3-none-any.whl
```

## 快速开始

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --robot-type "quanta_x1"
```

## 参数说明

| 参数 | 必填 | 默认值 | 说明 |
|------|------|--------|------|
| `--input-dir` | 是 | - | 输入目录（需包含 `dataset_metadata.json` 和 `episode_*/` 文件夹） |
| `--output-dir` | 是 | - | 输出 LeRobot 数据集目录 |
| `--repo-id` | 是 | - | 数据集仓库 ID（如 `"my_robot/dataset"`） |
| `--robot-type` | 否 | `quanta_x1` | 机型：`quanta_x1`、`quanta_x2`、`desktop` |
| `--fps` | 否 | 30 | 目标帧率 |
| `--resize-width` | 否 | 640 | 图像宽度 |
| `--resize-height` | 否 | 480 | 图像高度 |
| `--delta-action` | 否 | False | 使用增量动作（action = next_state - current_state） |
| `--relative-start` | 否 | False | 以首帧为基准，对 state/action 做相对化 |
| `--select-joints` | 否 | - | 逗号分隔的关节名，仅包含指定关节 |
| `--episodes` | 否 | 全部 | 要转换的 episode（如 `"0,1,2"` 或 `"0-5"`） |
| `--use-videos` | 否 | False | 使用 MP4 视频而非图像 |
| `--video-backend` | 否 | - | 视频后端：`pyav` 或 `opencv` |
| `--include-sensors` | 否 | - | 额外传感器（如 `odometry,chassis_imu,left_arm_end_pose`） |
| `--future-action-chunk-size` | 否 | 32 | 无 action 时，用未来 state 作为 action 的帧数 |

## 机型说明

### quanta_x1

- 主臂 + 从手结构
- **State/Action 字段**：`master_left_ee_cartesian_pos`、`master_left_ee_rotation`、`follow_left_gripper`、`master_right_ee_cartesian_pos`、`master_right_ee_rotation`、`follow_right_gripper`、`head_rotation`、`height`（升降）、`velocity_decomposed_odom`
- **数据来源**：`master_left_arm_end_pose`、`master_right_arm_end_pose`、`lift_joint_states` 等

### quanta_x2

- 无主臂，左右臂直接控制
- **State/Action 字段**：`left_ee_cartesian_pos`、`left_ee_rotation`、`follow_left_gripper`、`right_ee_cartesian_pos`、`right_ee_rotation`、`follow_right_gripper`、`head_rotation`、`waist`（4 维）、`velocity_decomposed_odom`
- **数据来源**：`left_arm_end_pose`、`right_arm_end_pose`、`waist_joint_states`、`left_gripper_position` 等

### desktop

- 仅手臂 + 夹爪，无腰、头部、底盘/里程计
- **State/Action 字段**：`left_ee_cartesian_pos`、`left_ee_rotation`、`follow_left_gripper`、`right_ee_cartesian_pos`、`right_ee_rotation`、`follow_right_gripper`
- **数据来源**：`left_arm_end_pose`、`right_arm_end_pose`、`left_gripper_joint_states`、`right_gripper_joint_states` 等

## 输入目录结构

```
input_dir/
├── dataset_metadata.json    # 必需：fps、camera_names、robot_type、episodes
└── episode_0000/
    ├── episode.json         # 帧数据：observation、action、images
    └── *.mp4 或 *.jpg       # 视频或图像文件
```

## 使用示例

### 使用视频格式（节省存储，推荐）

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --robot-type "quanta_x1" \
    --use-videos
```

### 基本转换（quanta_x1）, 图片格式存储，比较耗费存储空间

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "quanta_x1/dataset" \
    --robot-type "quanta_x1"
```

### 基本转换（quanta_x2）

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "quanta_x2/dataset" \
    --robot-type "quanta_x2"
```

### 指定 episode 转换

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --episodes "0,1,2" \
    --robot-type "quanta_x1"
```

### 包含额外传感器

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --include-sensors "odometry,chassis_imu,left_arm_end_pose,right_arm_end_pose" \
    --robot-type "quanta_x1"
```

## 注意事项

1. **可自定义转换内容**：可通过以下方式自定义需要转换的数据：(1) 修改采集代码，只采集所需字段；(2) 修改转换脚本（如 `ROBOT_DATA_CONFIG`、`state_source_mapping`、`action_source_mapping`）以定义新机型或数据映射。命令行参数 `--select-joints`、`--episodes`、`--include-sensors` 也可做基础定制。

2. **task 字段**：LeRobot 中 `task` 为特殊字段，每帧需包含该字段（字符串）。它不在 `features` 中定义，由 LeRobot 自动处理。

3. **动作格式**：脚本会自动识别 V3 格式（末端位姿动作）和 V2 格式（关节动作），并按对应方式提取动作。

4. **无 action 字段**：若帧中无 `action` 字段，可使用未来 state 作为 action（通过 `--future-action-chunk-size` 配置）。

5. **相机名称**：`dataset_metadata.json` 中的相机名称需与 `episode.json` 帧内一致（如 `head_camera`、`left_arm_camera`、`right_arm_camera`）。

## 加载数据集

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset("my_robot/dataset", root="./lerobot_data")
```
