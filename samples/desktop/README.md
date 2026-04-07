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

- Each inference step corresponds to sending an instruction and receiving a response.
- After each inference completes, press Enter to continue to the next step.
- This allows step-by-step interaction and debugging.

### ⚙️ Default Configuration

The script uses the following default parameters:
| Parameter | Description | Default |
|  ----  | ----  | ----  |
|`MODEL_ADDRESS` | Model service IP address | `39.101.65.229` |
|`MODEL_PORT` | Model service port | `1175` |
|`INSTRUCTION` | Initial instruction sent to the model |`Pick up the green cup and place it on the tray`|
|`CONTROL_MODE` | Control mode for the robot | `end_pose` |
|`INTERPOLATE_MULTIPLIER` | Motion interpolation multiplier |`20` |
|`ROBOT_SDK_URL` | Robot SDK address | `192.168.10.1:50051` |

### 📌 Notes

- Ensure the model service is running and reachable before starting the client.
- This sample is intended for Desktop inference and SDK validation.
- Logs and responses will be printed directly to the terminal.

### 📄 License

This project is provided for internal testing and SDK integration purposes.
