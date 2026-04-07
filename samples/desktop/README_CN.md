# X2Robot Client SDK（Desktop）

本目录提供了一个 **X2Robot 模型服务** 的 **Desktop** 示例客户端，用于演示 SDK 推理流程与交互方式。
**该 sample 仅支持 Desktop 机型**

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
pip install -r samples/desktop/requirements.txt
```

### 2. 启动 SDK 客户端

```bash
bash samples/desktop/scripts/start_sdk_desktop.sh
```

### 🧠 推理流程说明

- 每一次推理表示 发送一条指令并接收模型返回结果。
- 每完成一次推理后，需要按回车键（Enter） 才会继续执行下一步。
- 该方式适合进行分步调试与交互验证。

### ⚙️ 默认配置说明

脚本内置以下默认参数：

| 参数名 | 说明 | 默认值 |
|  ----  | ----  | ----  |
|`MODEL_ADDRESS` | 模型服务IP地址 |  `39.101.65.229` |
|`MODEL_PORT` | 模型服务端口 | `1175` |
|`INSTRUCTION` | 初始发送给模型的指令 | `Pick up the green cup and place it on the tray` |
|`CONTROL_MODE` | 机器人控制模式 | `end_pose` |
|`INTERPOLATE_MULTIPLIER` | 动作插值倍率 |`20` |
|`ROBOT_SDK_URL` | 机器人SDK地址 | `192.168.10.1:50051` |

### 📌 注意事项

- 启动前请确认模型服务已正常运行并可访问。
- 该示例主要用于 `Desktop` 类型推理流程验证与 SDK 集成测试。
- 推理日志与模型返回结果将直接输出在终端中。

### 📄 说明

本示例仅用于 SDK 测试、功能验证及集成参考。
