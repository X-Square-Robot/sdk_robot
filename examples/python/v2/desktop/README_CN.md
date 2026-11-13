# Desktop 桌面主从 SDK2

本目录面向桌面主从机型，接口范围仅为左右主臂、左右从臂、夹爪和相机。
服务端使用 `--model=desktop`，客户端地址为 `<机器 IP>:55051`。

## 只读接口验证

安装对应 Python 版本的 `x2robot_sdk` wheel，在 SDK 产物根目录执行：

```bash
python3 examples/python/v2/desktop/verify_readonly.py \
  --server 10.136.21.192:55051 --samples 10 --output desktop-results.json
```

该脚本使用明确的接口白名单，不切换控制模式、不发送运动目标。任一接口失败时
保留错误与 request ID，继续其他检查，最终以非零退出码报告失败，不将缺失数据算作通过。
默认退出码为 0，便于批量任务继续执行；需要在 CI 中严格失败时追加
`--fail-on-error`，它也只在全部检查结束后返回非零。

| 构件 | 检查内容 |
| --- | --- |
| 左右从臂 | 关节状态、末端位姿、世界/局部坐标系外力及对应流，电机状态 |
| 左右主臂 | 控制模式读取、关节状态、末端位姿、主夹爪位置及对应流，主夹爪关节状态流 |
| 左右夹爪 | 位置读取、位置流、关节状态流、电机状态 |
| 相机 | 前置 RGB、前置深度、左右腕部图像与视频流 |

前置相机在 SDK 中的访问名为 `head_camera`；左右腕部为 `left_arm_camera`、
`right_arm_camera`。此矩阵对应 desktop x86 相机配置。

每侧主臂必须返回对应的 6 个关节（`master_left_arm_joint1..6` 或
`master_right_arm_joint1..6`），不能返回全局关节集合。从臂输出关节数值，
末端输出位置 XYZ（米）和四元数 XYZW。其他字段沿用 SDK 原始单位，不擅自换算。

流式检查不插入固定 `sleep`。相机启动默认最多等待 5 秒，可用
`--camera-deadline-ms` 调整；这是首帧超时上限，不是采样间隔。
`--samples` 控制采样数量，报告中的 `arrival_hz`
是客户端收到这些样本的频率，短窗口可能受网络缓冲影响，不代表设备额定频率。
相机检查非空载荷和流时间戳递增；图像解码显示可使用独立相机示例。

## 单项读取与打印

```bash
python3 examples/python/v2/desktop/arm_control.py --server 10.136.21.192:55051 --arm both --action read
python3 examples/python/v2/desktop/arm_control.py --server 10.136.21.192:55051 --arm left --action stream --mode end_pose --seconds 1
python3 examples/python/v2/desktop/master_arm_control.py --server 10.136.21.192:55051 --arm both --action read
python3 examples/python/v2/desktop/master_arm_control.py --server 10.136.21.192:55051 --arm left --action stream --stream joint-states --samples 10
python3 examples/python/v2/desktop/gripper_control.py --server 10.136.21.192:55051 --gripper both --action read
```

以上入口默认只读；流式打印默认不额外限频。

## 运动接口边界

`arm_control.py` 和 `gripper_control.py` 的 `move` / `verify` 会发送运动指令，
不是本页只读验证的一部分；其固定目标验证面向模拟器，不应直接用于桌面实机验收。
主臂示例中的写动作还要求显式 `--write`。

主臂命令与手臂共用同一道服务端门控：`set-mode` / `move` / `zero` 之前示例会
自动进入 SDK 模式（`System.SetWorkMode(SDK)`，对应整机 `/application/task_info.type`
变为 5），退出时交还 IDLE（回到 0）；只读/流式动作不会改动工作模式。机器人不在
SDK 模式时主臂写入会报 `Sdk mode is not running`，因此不要再依赖"恰好处于 SDK 模式"。

当前服务端主臂 `SetJointPositions`、`SetEndPose`、`SetGripperPosition` 后端返回
`UNIMPLEMENTED`；保留的示例命令不代表当前服务端已经支持这些动作。
运动闭环的验证需要单独确认现场条件、目标范围和恢复方式，不能用读取成功代替。
