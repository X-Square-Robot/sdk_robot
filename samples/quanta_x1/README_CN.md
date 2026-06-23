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

### 📄 说明

本示例仅用于 SDK 测试、功能验证及集成参考。
