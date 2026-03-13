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

- Each inference step corresponds to sending an instruction and receiving a response.
- After each inference completes, press Enter to continue to the next step.
- This allows step-by-step interaction and debugging.

### ⚙️ Default Configuration

The script uses the following default parameters:
| Parameter | Description | Default |
|  ----  | ----  | ----  |
|`MODEL_ADDRESS` | Model service IP address | `39.101.65.229` |
|`MODEL_PORT` | Model service port | `30012` |
|`INSTRUCTION` | Initial instruction sent to the model |（空）|
|`CONTROL_MODE` | Control mode for the robot | `end_pose` |
|`INTERPOLATE_MULTIPLIER` | Motion interpolation multiplier |`20` |

### 📌 Notes

- Ensure the model service is running and reachable before starting the client.
- This sample is intended for EX001 inference and SDK validation.
- Logs and responses will be printed directly to the terminal.

### 📄 License

This project is provided for internal testing and SDK integration purposes.
