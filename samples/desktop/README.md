# X2Robot Client SDK (Desktop)

This repository provides a sample client for interacting with the **X2Robot model service** using the **Desktop** SDK workflow.
**This sample is only compatible with Desktop.**

---

## 📦 Requirements

- The SDK must be installed in the same Python virtual environment as described in
  <https://github.com/X-Square-Robot/sdk_robot#virtual-environment-setup>
- Linux
- Network access to the model service

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install -r samples/desktop/requirements.txt
```

### 2. Run the SDK Client

```bash
bash samples/desktop/scripts/start_sdk_desktop.sh
```

### 🧠 Inference Workflow

- **High-level flow**:
  - Connect to the **model service** (WebSocket) and receive server `metadata`
  - Connect to the **robot SDK service** (`x2://<ROBOT_SDK_URL>`), set SDK work mode and control mode
  - Run the loop: read robot observations (state + camera images) → send to the model service → receive predicted action chunk → execute actions on the robot via SDK (optional interpolation for smoother motion)
- **Per-iteration (code-level) steps**:
  - `_collect_sensor_data()`: read robot state and camera views, build the model input payload
  - `predict_sync()`: send the payload over WebSocket (msgpack) and wait for the model response
  - `_execute_actions()`: decode model outputs and call SDK APIs to drive the robot
- **About “press Enter to continue”**:
  - The Desktop launcher passes `--debug-step`, but `DesktopClient` currently doesn’t pause on Enter. The client runs continuously by default (for step-by-step behavior, see the `Quanta_X1` sample or add an interactive pause in `DesktopClient`).

### ⚙️ Default Configuration

The script uses the following default parameters:
| Parameter | Description | Default |
|  ----  | ----  | ----  |
|`MODEL_ADDRESS` | Model service IP address | `39.101.65.229` |
|`MODEL_PORT` | Model service port | `1175` |
|`INSTRUCTION` | Initial instruction sent to the model |`Pick up the green cup and place it on the tray`|
|`CONTROL_MODE` | Control mode for the robot | `end_pose` |
|`INTERPOLATE_MULTIPLIER` | Motion interpolation multiplier |`10` |
|`ROBOT_SDK_URL` | Robot SDK address | `192.168.10.1:50051` |

### 📌 Notes

- Ensure the model service is running and reachable before starting the client.
- This sample is intended for Desktop inference and SDK validation.
- Logs and responses will be printed directly to the terminal.

### 📄 License

This project is provided for internal testing and SDK integration purposes.
