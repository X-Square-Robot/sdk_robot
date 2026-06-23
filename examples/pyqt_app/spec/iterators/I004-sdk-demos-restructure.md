# I004 — SDK demos 分层重构

## 本次目标

将 examples 下的 SDK 调用逻辑从 UI services 和 CLI demos 中抽离为独立的 `sdk_demos/` 层，
实现 SDK 用法演示与 UI/CLI 业务逻辑的分离。

## 本次范围

### 1. 新建 `examples/sdk_demos/` 目录结构

按 stub 所属机型和功能域拆分为独立文件：

```
examples/sdk_demos/
├── __init__.py
├── _connect.py               ← 统一的 connect(server, model) 封装
│
├── system_demo.py            ← 全机型 SystemStub
├── arm_demo.py               ← 全机型 左右臂 (left_arm / right_arm)
├── gripper_demo.py           ← 全机型 夹爪
├── camera_demo.py            ← 全机型 相机
├── head_demo.py              ← q1+q2  头部
├── chassis_demo.py           ← q1+q2  底盘
├── navigation_demo.py        ← q1+q2  导航
├── sensor_demo.py            ← q1+q2  雷达/超声/TOF/IMU/深度
├── audio_demo.py             ← q1+q2  音频
├── data_collection_demo.py   ← q1+q2  数据采集
├── master_arm_demo.py        ← q1+dt  主臂查询/控制/流
├── align_demo.py             ← q1+dt  主从对齐
├── lift_demo.py              ← q1     升降台
├── waist_demo.py             ← q2     腰部
└── hand_demo.py              ← q2     灵巧手 + 触觉
```

每个文件头上标注适用机型：

```python
# sdk_demos/lift_demo.py
"""
升降台控制。
适用机型: quanta_x1
"""
```

### 2. SDK demo 函数签名规范

- **入参**: 已连接的 `Robot` 对象
- **出参**: 原生 Python dict / dataclass / Iterator（不依赖 ServiceResult、QThread 等 UI 类型）
- **零外部依赖**: 只依赖 `x2robot`，不依赖 pyqt_app 的任何模块

```python
# 标准签名
def get_control_mode(robot: Robot, arm: str) -> dict:
    ...

def stream_joint_states(robot: Robot, arm: str) -> Iterator[dict]:
    ...
```

### 3. 从现有代码抽取 SDK demo 函数

| 来源 | 抽取目标 |
|------|---------|
| `services/robot_info.py` | `system_demo.py` + `master_arm_demo.py` |
| `services/arm_motion.py` | `arm_demo.py`（TOPP-RA 轨迹逻辑保留在 UI services 或单独的 motion demo） |
| `quanta_x1/arm_control.py` | `arm_demo.py`（arm 控制部分） |
| `quanta_x1/lift_control.py` | `lift_demo.py` |
| `quanta_x1/align_master_slave_demo.py` | `align_demo.py` |
| `quanta_x2/waist_control.py` | `waist_demo.py` |
| `quanta_x2/gripper_control.py` | `gripper_demo.py`（如有 quata_x1 没有的部分） |
| `data_collection/*.py` | `data_collection_demo.py` |

### 4. 重构 `pyqt_app/services/` 为薄包装

```python
# services/robot_info.py → 改为调 sdk_demos
from ...sdk_demos import master_arm_demo, system_demo

def run_get_model_type(server: str) -> ServiceResult:
    try:
        robot = _connect(server)
        result = system_demo.get_model_type(robot)
        return ServiceResult(True, f"model_type={result}")
    except grpc.RpcError as exc:
        return ServiceResult(False, str(exc))
```

### 5. 重构旧 CLI demo

`quanta_x1/`, `quanta_x2/`, `desktop/` 下的 typer CLI demo 的 SDK 调用逻辑改为 import `sdk_demos`，只保留 CLI 入口层。

## 本次不做什么

- 不新增功能、不新增页面
- 不修改 `client/x2robot/robot.py` 或 SDK 代码
- 不处理 `deploy_ops/` 和 `remote_ops/`（这些不是 SDK 调用层）
- 暂不写 `sdk_demos/` 的单元测试
- 不动 `samples/` 目录（独立维护的 sample 项目）

## 机型 -> stub 归属速查

```
全机型通用 (8)
  robot_control left_arm right_arm left_gripper right_gripper
  head_camera left_arm_camera right_arm_camera

q1+q2 共有 (10)
  head chassis navigation radar ultrasonic tof imu depth_points
  action_data_collection audio

q1+desktop 共有 (2)
  master_left_arm master_right_arm

quanta_x1 独有 (1)
  lift

quanta_x2 独有 (7)
  waist left_hand right_hand
  left_hand_tactile left_gripper_tactile right_hand_tactile right_gripper_tactile

desktop 独有 (0)
```

---

## 实现前 Plan

<!-- AI 在写代码前对方案的简述 -->

## 变更清单

<!-- 新增/修改了哪些文件，各做了什么 -->

---

## 完成总结 (Completion Summary)

<!-- 实现完成后填写：实际做了什么 -->

## 验证结果 (Verification)

<!-- 如何验证本次迭代正确 -->

```bash
# 验证命令
```

## 偏差和风险

<!-- 是否有偏离 plan 的地方，是否有已知风险 -->
