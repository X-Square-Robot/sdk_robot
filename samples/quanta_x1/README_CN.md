# X2Robot Client SDK（Quanta_X1）

本目录提供了一个 **X2Robot 模型服务** 的 **Quanta_X1** 示例客户端，用于演示 SDK 推理流程与交互方式。
**该 sample 仅支持 Quanta_X1 机型**

---

## 📦 环境要求

- 需在与
  <https://github.com/X-Square-Robot/sdk_robot#virtual-environment-setup>
  的 **同一 Python 虚拟环境** 中完成安装
- Linux
- 可访问模型服务的网络环境

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install -r samples/quanta_x1/requirements.txt
```

### 2. 启动 SDK 客户端

```bash
bash samples/quanta_x1/scripts/start_sdk_ex001.sh
```

### 🧠 推理流程说明

- **整体流程（简述）**：
  - 连接 **模型服务**（WebSocket），并接收服务端 `metadata`
  - 连接 **机器人 SDK 服务**（`x2://<ROBOT_SDK_URL>`），设置机器人为 SDK 工作模式，并初始化底盘/导航/机械臂等控制模式
  - 进入循环：采集机器人当前观测（状态 + 相机图像）→ 发送给模型服务 → 接收模型预测的动作序列 → 通过 SDK 下发到机器人执行（可选插值平滑）
- **一次推理循环对应的代码逻辑**：
  - `_collect_sensor_data()`：从机器人读取状态/相机图像，组装为模型输入
  - `predict_sync()`：通过 WebSocket 发送输入（msgpack 序列化）并等待模型返回
  - `_execute_actions()`：解析模型输出（臂/夹爪/头/升降/底盘等），并调用 SDK 接口执行
- **分步调试（Enter）**：
  - 启动脚本默认带 `--debug-step`，在每轮采集与请求下一段动作 chunk 之前，会提示按回车继续，方便交互验证。

### ⚙️ 默认配置说明

脚本内置以下默认参数：

| 参数名 | 说明 | 默认值 |
|  ----  | ----  | ----  |
|`MODEL_ADDRESS` | 模型服务IP地址 |  `39.101.65.229` |
|`MODEL_PORT` | 模型服务端口 | `1174` |
|`INSTRUCTION` | 初始发送给模型的指令 |（空）|
|`CONTROL_MODE` | 机器人控制模式 | `end_pose` |
|`INTERPOLATE_MULTIPLIER` | 动作插值倍率 |`20` |
|`ROBOT_SDK_URL` | 机器人SDK地址 | `192.168.10.1:50051` |

### 📌 注意事项

- 启动前请确认模型服务已正常运行并可访问。
- 该示例主要用于 `Quanta_X1` 类型推理流程验证与 SDK 集成测试。
- 推理日志与模型返回结果将直接输出在终端中。

### v1.1.1 hotfix：训练与推理数据约定

- `velocity_decomposed_odom` 的顺序为机体坐标系速度 `[vx, vy, vyaw]`，单位为 `[m/s, m/s, rad/s]`。转换脚本从 `odometry.twist` 提取观测、从 `odometry_action.twist` 提取动作；Quanta X1/X2 共用的转换配置均已修正。LeRobot 特征名相应改为 `odom_vx`、`odom_vy`、`odom_vyaw`。
- 在线客户端通过 `chassis.get_odometry()` 读取实测速度。模型输出的速度仍按原流程积分为底盘位置轨迹；`car_pose_odom` 保留位置轨迹含义。
- 训练数据的 `head_rotation` 和在线接口的 `head_pos` 均采用 `[pitch, yaw]`，单位为弧度；头部下发、限幅和调试图遵循相同顺序。
- 相机数组使用 RGB；调用 OpenCV 的 JPEG 编码前转换为 BGR，保证模型解码后颜色正确。

旧转换脚本曾把底盘位姿写入速度字段。包含这些错误字段的数据集需要从原始采集数据重新转换、重新计算归一化统计量，并重新训练相关模型，不能仅替换客户端。原始数据应包含真实的 `twist`；缺失的速度分量仍按转换器约定填零，不会通过位姿推算。当前采集器的底盘动作仍是下一帧实测速度，不代表操作者的目标速度指令。

本客户端消费绝对关节/末端目标及底盘速度；转换数据用于该客户端时，不应启用 `--delta-action` 或 `--relative-start`，这两个选项会改变整个向量的含义。

离线回归测试（安装转换脚本依赖、本示例依赖以及 `pytest`、`scipy` 后，在仓库根目录运行）：

```bash
PYTHONPATH=client python -m pytest -q tests/test_sdk_inference_contract.py
```

测试使用模拟机器人接口，不连接机器人或模型服务。

### 📄 说明

本示例仅用于 SDK 测试、功能验证及集成参考。
