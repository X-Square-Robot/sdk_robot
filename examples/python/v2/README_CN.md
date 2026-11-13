# Python v2 示例

本目录按 `python/v1/` 的同名文件保留场景逻辑，使用新版 SDK 接口实现。
纯 PyQt 界面与 SSH
运维代码不属于 SDK 接口示例，因此不在这里重复。

所有示例遵循同一规则：

- 使用 `x2robot_sdk.connect()`，只走 gRPC；
- 每次调用均返回 `SdkResult`，业务数据从 `result.data` 获取；
- 失败信息保留结构化 `error` 和 `request_id`；
- 流式接口的每个响应同样是 `SdkResult`；
- 控制示例均执行 Set -> Get，并断言读取值与设置值一致。

## 运行单个示例

录制式采集与对应回放集中在 [`data_collection_v2/`](data_collection_v2/SDK2_RECORDING_CN.md)：
`data_collection_v2.py` 负责开始/停止录制、下载和解析，`data_replay_example.py`
回放解析后的 `episode.json`，`config/` 提供含英文说明和启用开关的机型预设（包括 Quanta X1 双目头部）。完整命令见
[SDK2 录制与回放说明](data_collection_v2/SDK2_RECORDING_CN.md)。

从 `sdk_client` 仓库根目录执行：

```bash
python3 -m examples.python.v2.check_connect --server 127.0.0.1:55051
python3 -m examples.python.v2.camera --server 127.0.0.1:55051
python3 -m examples.python.v2.quanta_x2.waist_control \
  --server 127.0.0.1:55051 --action verify
```

传感器和控制示例保留 v1 的动作名，同时增加了非交互的 `verify` 动作。例如：

```bash
python3 examples/python/v2/imu.py --server 127.0.0.1:55051 --action stream --seconds 3
python3 examples/python/v2/tof.py --server 127.0.0.1:55051 --action both
python3 examples/python/v2/ultrasonic.py --server 127.0.0.1:55051 --action all-stream --seconds 3
python3 examples/python/v2/navigation.py --server 127.0.0.1:55051 --action list
python3 examples/python/v2/audio.py stream --server 127.0.0.1:55051 \
  --seconds 5 --output ./capture.wav
python3 examples/python/v2/quanta_x1/arm_control.py \
  --server 127.0.0.1:55051 --action move --arm left --mode joint_pos
python3 examples/python/v2/quanta_x2/waist_control.py \
  --server 127.0.0.1:55051 --action stream --mode joint_pos --seconds 3
```

未指定动作时使用与 v1 相同的安全单次动作（例如 IMU 使用 `single`、相机使用
`rgb-image`、手臂使用 `move`）；自动化测试可显式使用 `--action verify`。流式
动作默认保持 v1 的 Ctrl+C（或窗口 `q`）退出语义；测试/脚本可以传 `--seconds`
使用有限时长，退出时立即关闭流句柄，服务端会释放对应订阅。

如果 wheel 安装在虚拟环境中，只需保证当前目录是 `sdk_client` 仓库根目录；
源码构建环境也可以显式设置 `PYTHONPATH`：

```bash
PYTHONPATH="$PWD:$PWD/native/python" \
  python3 -m examples.python.v2.system --server 127.0.0.1:55051
```

## 批量端到端验证

Desktop 桌面主从环境请使用 [Desktop 专用说明](desktop/README_CN.md) 和
`desktop/verify_readonly.py`。该入口只验证主从臂、夹爪和相机；
`verify_all.py --model desktop` 也只调用这份专用矩阵，不执行整机验证列表。

`verify_all.py` 逐个调用新示例的 `verify()`，在模拟器中检查接口调用和设置后的读取：

```bash
python3 -m examples.python.v2.verify_all --server 127.0.0.1:55051 --model ex001
python3 -m examples.python.v2.verify_all --server 127.0.0.1:55051 --model cx002
```

机型不具备的示例会显示 `model_skip`：例如 CX002 没有 Lift/MasterArm，EX001
没有 WaistController。相机的眼部/肘部可选能力只有在服务端明确返回
`UNAVAILABLE` 或 `UNIMPLEMENTED` 时才会跳过，其他错误仍会导致验证失败。

相机示例优先用 OpenCV 解码 JPEG/PNG，并支持 `--show` 显示窗口；H.264/H.265
数据通过连续流等待关键帧后再解码（需要 `av`）。没有图形环境时省略 `--show`，
用 `--save-dir` 保存解码图像或原始编码帧。

## 采集数据回放

`data_replay_example.py` 支持 Python 采集的 JSON 和 C++ 采集的原始
`sdk-protobuf-record-v1` 数据目录，无需手动转换。只回放上半身时使用：

```bash
python examples/python/v2/data_replay_example.py ./collected_data/episode_000 \
  --server 127.0.0.1:55051 --mode end_pose --speed 1 --no-chassis
```

`--no-chassis` 会跳过所有底盘切模和运动调用，包括退出时的零速度指令。
Quanta X3 的末端回放要求双臂、腰部三路末端数据齐全，缺失时在切模式前报错，
不会退回关节模式。目标周期为 20 ms，每周期按同一回放时间采样三路目标，位置线性
插值、姿态用最短路径四元数 SLERP，与采集的数据对齐方法一致。记录间隙采用
插值，超出某路记录时间范围时保持该路边界位姿；切模期间未录到的动作无法还原。
头部和夹爪仍按记录时间发送。

RPC 耗时计入周期，只等待本周期剩余时间。调用成功但耗时超过周期时，回放时长
随之延长，不按实际流逝时间跳到后面的目标，也不集中补发跳过的周期。
取消固定的 100 ms 周期退出限制；三路末端 RPC 仍各有 300 ms 超时。末端 RPC
超时只打印 WARNING，继续其余目标和后续轨迹，不自动重发该指令；超时指令是否实际
执行无法确认，结束统计中的 `pose_timeouts` 汇总次数。其他错误仍停止回放，退出时
恢复待机。有线网络有助于接近设定回放速度。
执行成功表示命令流程完成，实际动作仍需现场确认。

## 目录映射

v1 与 v2 都是独立的示例目录。v1 保留历史路径；v2 按机型与桌面类型使用
和 C++ 一致的子目录，文件名保持场景本身，便于两种语言对照使用。下面列出
v1 到 v2 的目录映射及 v2 专属入口：

- 通用：`audio.py`、`camera.py`、`chassis_control.py`、`check_connect.py`、
  `data_collection_example.py`、`data_replay_example.py`、`depth_points.py`、
  `head_control.py`、`imu.py`、`navigation.py`、`radar.py`、`robot_control.py`、
  `robot_status.py`、`system.py`、`tf.py`、`tof.py`、`ultrasonic.py`；
- Desktop：`desktop/arm_control.py`、`desktop/gripper_control.py`、
  `desktop/master_arm_control.py`；
- Quanta X1：`quanta_x1/arm_control.py`、`quanta_x1/gripper_control.py`、
  `quanta_x1/lift_control.py`、`quanta_x1/master_arm_control.py` 以及
  `quanta_x1/align_master_slave_demo.py`；
- Quanta X2：`quanta_x2/arm_control.py`、`quanta_x2/gripper_control.py`、
  `quanta_x2/waist_control.py` 以及 `quanta_x2/end_pose_replay_new.py`；
- Quanta X3（仅 v2）：`quanta_x3/arm_control.py`、`quanta_x3/gripper_control.py`、
  `quanta_x3/waist_control.py`、`quanta_x3/lidar_points.py` 和
  `quanta_x3/audio_pattern.py`；
- 数据采集：`data_collection/collection_config.py` 和
  `data_collection/data_collector.py`（与 v1 路径一一对应）。

v2 的 `desktop/`、`quanta_x1/`、`quanta_x2/` 子目录与 C++ 示例保持一致；
通用示例和流式数据采集示例仍直接放在 v2 根目录，录制式采集与回放放在 `data_collection_v2/`。
v1 目录保留历史路径，旧脚本
不会被破坏。

接口的完整契约覆盖由 `sdk_server/test/robot_sim.py` 配合客户端测试完成；
本目录负责验证用户可直接运行的场景示例。

## quanta_x3

quanta_x3 的机型差异入口包括 arm、waist、gripper 和三维雷达点云；头部、底盘、
系统、音频、导航及其他公共传感器能力使用本目录根部的公共示例。
底盘深度相机点云使用 `depth_points.py`，Quanta X3 雷达点云使用独立入口：

```bash
python examples/python/v2/quanta_x3/lidar_points.py --server 127.0.0.1:55051 --action verify
```

完整入口和轨迹说明见 [quanta_x3 示例说明](../../QUANTA_X3_README.md)。
`waist_control.py` 演示三关节与末端平滑轨迹，默认只读，运动需显式指定小幅增量。
