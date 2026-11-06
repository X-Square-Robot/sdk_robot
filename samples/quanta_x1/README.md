# X2Robot Client SDK (Quanta_X1)

This repository provides a sample client for interacting with the **X2Robot model service** using the **Quanta_X1** SDK workflow.
**This sample is only compatible with Quanta_X1.**

---

## 📦 Requirements

- The SDK must be installed in the same Python virtual environment as described in
  <https://github.com/X-Square-Robot/sdk_robot#virtual-environment-setup>
- Linux
- Network access to the model service

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install -r samples/quanta_x1/requirements.txt
```

### 2. Run the SDK Client

```bash
bash samples/quanta_x1/scripts/start_sdk_ex001.sh
```

### 🧠 Inference Workflow

- **High-level flow**:
  - Connect to the **model service** (WebSocket) and receive server `metadata`
  - Connect to the **robot SDK service** (`x2://<ROBOT_SDK_URL>`), set SDK work mode and initialize chassis/navigation/manipulator control modes
  - Run the loop: read robot observations (state + camera images) → send to the model service → receive a predicted action chunk → execute actions on the robot via SDK (optional interpolation for smoother motion)
- **Per-iteration (code-level) steps**:
  - `_collect_sensor_data()`: read robot state and camera views, build the model input payload
  - `predict_sync()`: send the payload over WebSocket (msgpack) and wait for the model response
  - `_execute_actions()`: decode model outputs (arms/grippers/head/lift/chassis, etc.) and call SDK APIs to drive the robot
- **Step-by-step debugging (Enter)**:
  - The launcher enables `--debug-step` by default, which pauses before collecting data / requesting the next action chunk, so you can press Enter to proceed each iteration.

### ⚙️ Default Configuration

The script uses the following default parameters:
| Parameter | Description | Default |
|  ----  | ----  | ----  |
|`MODEL_ADDRESS` | Model service IP address | `39.101.65.229` |
|`MODEL_PORT` | Model service port | `1174` |
|`INSTRUCTION` | Initial instruction sent to the model |（空）|
|`CONTROL_MODE` | Control mode for the robot | `end_pose` |
|`INTERPOLATE_MULTIPLIER` | Motion interpolation multiplier |`20` |
|`ROBOT_SDK_URL` | Robot SDK address | `192.168.10.1:50051` |

### 📌 Notes

- Ensure the model service is running and reachable before starting the client.
- This sample is intended for EX001 inference and SDK validation.
- Logs and responses will be printed directly to the terminal.

### v1.1.1 hotfix: training and inference data contract

- `velocity_decomposed_odom` contains body-frame `[vx, vy, vyaw]` in `[m/s, m/s, rad/s]`. The converter reads observations from `odometry.twist` and actions from `odometry_action.twist` for both Quanta X1 and X2. The LeRobot feature names are `odom_vx`, `odom_vy`, and `odom_vyaw`.
- The online client reads measured velocity with `chassis.get_odometry()`. Predicted velocities are still integrated into chassis position trajectories; `car_pose_odom` remains a position trajectory.
- Training `head_rotation` and online `head_pos` both use `[pitch, yaw]` in radians, including head commands, axis limits, and debug plots.
- Camera arrays are RGB. They are converted to BGR before OpenCV JPEG encoding so the model receives the correct colors.

Datasets produced by the old converter with chassis poses in velocity fields must be regenerated from the original recordings, with normalization statistics recomputed and affected models retrained. Updating only the client is insufficient. Original recordings must contain valid `twist` measurements; missing velocity components retain the converter's zero-fill behavior and are never inferred from pose. Chassis action labels still represent the next frame's measured velocity, not the operator's requested velocity.

This client consumes absolute joint/end-effector targets and chassis velocity. Do not use `--delta-action` or `--relative-start` for datasets intended for this client, since those options transform the entire vector.

Run the offline regression tests from the repository root after installing the converter dependencies, sample dependencies, `pytest`, and `scipy`:

```bash
PYTHONPATH=client python -m pytest -q tests/test_sdk_inference_contract.py
```

The tests use simulated robot interfaces without connecting to a robot or model service.

### 📄 License

This project is provided for internal testing and SDK integration purposes.
