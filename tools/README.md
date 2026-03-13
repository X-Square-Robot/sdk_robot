# convert_to_lerobot.py

Convert collected robot data to [LeRobot](https://github.com/huggingface/lerobot) dataset format for imitation learning and robot policy training.

## Features

- **Robot types**: quanta_x1 (with master arm), quanta_x2 (without master arm), desktop (arm + gripper only)
- **Data sources**:
   - Joint states
   - End-effector pose (position + orientation)
   - Actions (multi-mode: master_arm_control, vr_control, etc.)
   - Images (JPG or MP4 video)
   - Extra sensors: odometry, chassis_imu, pose, tactile, etc.

## Dependencies

Download the [lerobot v0.4.2 wheel](https://github.com/huggingface/lerobot/releases/download/v0.4.2/lerobot-0.4.2-py3-none-any.whl), then:

```bash
pip install lerobot-0.4.2-py3-none-any.whl
```

## Quick Start

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --robot-type "quanta_x1"
```

## Arguments

| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--input-dir` | Yes | - | Input directory (must contain `dataset_metadata.json` and `episode_*/` folders) |
| `--output-dir` | Yes | - | Output LeRobot dataset directory |
| `--repo-id` | Yes | - | Dataset repo ID (e.g., `"my_robot/dataset"`) |
| `--robot-type` | No | `quanta_x1` | Robot type: `quanta_x1`, `quanta_x2`, `desktop` |
| `--fps` | No | 30 | Target frame rate |
| `--resize-width` | No | 640 | Image width |
| `--resize-height` | No | 480 | Image height |
| `--delta-action` | No | False | Use delta action (action = next_state - current_state) |
| `--relative-start` | No | False | Subtract first frame from state/action |
| `--select-joints` | No | - | Comma-separated joint names to include |
| `--episodes` | No | all | Episodes to convert (e.g., `"0,1,2"` or `"0-5"`) |
| `--use-videos` | No | False | Use MP4 video instead of images |
| `--video-backend` | No | - | Video backend: `pyav` or `opencv` |
| `--include-sensors` | No | - | Extra sensors (e.g., `odometry,chassis_imu,left_arm_end_pose`) |
| `--future-action-chunk-size` | No | 32 | Frames per chunk when using future state as action |

## Robot Types

### quanta_x1

- Master arm + follow arm setup
- **State/Action fields**: `master_left_ee_cartesian_pos`, `master_left_ee_rotation`, `follow_left_gripper`, `master_right_ee_cartesian_pos`, `master_right_ee_rotation`, `follow_right_gripper`, `head_rotation`, `height` (lift), `velocity_decomposed_odom`
- **Data sources**: `master_left_arm_end_pose`, `master_right_arm_end_pose`, `lift_joint_states`, etc.

### quanta_x2

- No master arm; direct left/right arm control
- **State/Action fields**: `left_ee_cartesian_pos`, `left_ee_rotation`, `follow_left_gripper`, `right_ee_cartesian_pos`, `right_ee_rotation`, `follow_right_gripper`, `head_rotation`, `waist` (4 dims), `velocity_decomposed_odom`
- **Data sources**: `left_arm_end_pose`, `right_arm_end_pose`, `waist_joint_states`, `left_gripper_position`, etc.

### desktop

- Arm + gripper only; no waist, head, or chassis/odometry
- **State/Action fields**: `left_ee_cartesian_pos`, `left_ee_rotation`, `follow_left_gripper`, `right_ee_cartesian_pos`, `right_ee_rotation`, `follow_right_gripper`
- **Data sources**: `left_arm_end_pose`, `right_arm_end_pose`, `left_gripper_joint_states`, `right_gripper_joint_states`, etc.

## Input Structure

```
input_dir/
├── dataset_metadata.json    # Required: fps, camera_names, robot_type, episodes
└── episode_0000/
    ├── episode.json          # Frames with observation, action, images
    └── *.mp4 or *.jpg       # Video or image files
```

## Examples

### Use video format (smaller storage, recommended)

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --robot-type "quanta_x1" \
    --use-videos
```

### Basic conversion (quanta_x1) with image format — uses more disk space

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "quanta_x1/dataset" \
    --robot-type "quanta_x1"
```

### Basic conversion (quanta_x2)

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "quanta_x2/dataset" \
    --robot-type "quanta_x2"
```

### Select specific episodes

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --episodes "0,1,2" \
    --robot-type "quanta_x1"
```

### Include extra sensors

```bash
python3 convert_to_lerobot.py \
    --input-dir ./collected_data \
    --output-dir ./lerobot_data \
    --repo-id "my_robot/dataset" \
    --include-sensors "odometry,chassis_imu,left_arm_end_pose,right_arm_end_pose" \
    --robot-type "quanta_x1"
```

## Important Notes

1. **Customizable conversion**: You can customize which data to convert by (1) modifying the data collection code to collect the fields you need, and (2) editing the conversion script (e.g., `ROBOT_DATA_CONFIG`, `state_source_mapping`, `action_source_mapping`) to define new robot types or data mappings. Command-line options like `--select-joints`, `--episodes`, and `--include-sensors` also allow basic customization.

2. **Task field**: In LeRobot, `task` is a special field. Each frame must contain it (string). It is not defined in `features`; LeRobot handles it automatically.

3. **Action format**: The script detects V3 format (end-effector pose actions) and V2 format (joint actions), and extracts actions accordingly.

4. **No action field**: If frames have no `action` field, the script can use future state as action (configurable via `--future-action-chunk-size`).

5. **Camera names**: Camera names in `dataset_metadata.json` should match those in `episode.json` frames (e.g., `head_camera`, `left_arm_camera`, `right_arm_camera`).

## Load Dataset

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

dataset = LeRobotDataset("my_robot/dataset", root="./lerobot_data")
```
