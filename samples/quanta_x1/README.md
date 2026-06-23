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

### 📄 License

This project is provided for internal testing and SDK integration purposes.
