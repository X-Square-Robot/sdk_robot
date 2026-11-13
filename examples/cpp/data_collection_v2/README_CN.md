# C++ v2 录制式采集与回放

本目录使用 SDK 的 Recording 和 Files 接口完成开始录制、停止录制和文件下载。
采集不会调用 GetXXXStream 或下发运动命令。下载使用 SDK 分片传输与长度、MD5 校验。

| 源文件 | 可执行程序 | 用途 |
| --- | --- | --- |
| `data_collection_v2.cpp` | `data_collection_v2` | 列出本地配置项、启停、下载及调用离线解析器 |
| `data_replay_example.cpp` | `data_replay_v2` | 回放 `stream_*.pb`，或离线检查回放计划 |
| `config/quanta_x1.conf`、`quanta_x1_dual_head_camera.conf`、`quanta_x2.conf`、`quanta_x3.conf` | — | 各机型默认采集项 |

在源码仓库执行 `./scripts/build.sh examples`，程序安装到 `artifacts/bin/`。
也可按 [C++ 构建说明](../README.md) 从 SDK 发布包构建。以下命令从发布包根目录执行；
源码构建后可先 `cd artifacts`。配置和解析器按发布包位置查找，支持从其他工作目录启动。

```bash
# 离线查看预设中的数据项及英文说明
./bin/data_collection_v2 --address ROBOT_IP:55051 --action list --preset quanta_x1_dual_head_camera
# 按机型默认配置采集 10 秒，下载并解析为 JSON/MP4 及两种回放格式
./bin/data_collection_v2 --address ROBOT_IP:55051 --preset quanta_x1_dual_head_camera --duration 10 --output-dir ./collected
# 自定义配置，仅下载 rosbag，不依赖 Python
./bin/data_collection_v2 --address ROBOT_IP:55051 \
  --config ./examples/cpp/data_collection_v2/config/quanta_x3.conf --raw-only
# 临时指定数据项，覆盖配置文件
./bin/data_collection_v2 --address ROBOT_IP:55051 --items imu odom --raw-only
```

Python 和 C++ 使用完全相同的 `.conf` 配置结构、数据项及英文说明。
提供 quanta_x1、quanta_x1_dual_head_camera、quanta_x2、quanta_x3 四套预设。
双目头部需显式选择 --preset quanta_x1_dual_head_camera，机型仍为 quanta_x1。

```ini
[preset]
name = custom_capture
model = quanta_x1

[item.joint_states]
enabled = true
description = Whole body joint feedback

[item.head_left_eye_rgb]
enabled = true
description = Head left eye color images; requires dual head cameras
```

优先级为 `--items` > `--config` > `--preset` > 连接机型的同名预设。
每个 item 必须有英文 description 和 enabled=true/false；只提交启用的 ID。
默认腕部相机适用于平行夹爪，专属末端状态/控制/触觉默认关闭，需按实际装配修改。
`--action list` 完全离线，省略配置时展示 quanta_x1。客户端不查询或自动筛选硬件能力；
recorder 在 Start 时校验实际支持，任一不支持项会使整个 Start 失败并返回具体 ID。
空选择、重复节/键、非法 ID、未知字段均报错；仅显式 `--items all` 选择全部。

```bash
# 分步操作：保留同一个 session-id
./bin/data_collection_v2 --address ROBOT_IP:55051 --action start --session-id demo_001
./bin/data_collection_v2 --address ROBOT_IP:55051 --action status --session-id demo_001
./bin/data_collection_v2 --address ROBOT_IP:55051 --action stop --session-id demo_001
./bin/data_collection_v2 --address ROBOT_IP:55051 --action download --session-id demo_001 --raw-only
```

Stop 等待 READY 后返回，不自动下载；download 只获取已结束录制的产物。
status/stop/download 不依赖采集配置。`--finalize-timeout` 默认 600 秒，用于停止后的等待与单次下载超时。
`--duration 0` 等待 Enter 或 stdin 关闭。collect 在 Ctrl-C/SIGTERM 或开始后的等待失败时尝试停止当前会话，
失败时保留原错误和会话 ID。单独 start 会持续录制直到显式 stop；Start 超时不表示没有开始，
应使用已打印的 ID 查询并停止。下载不覆盖已有文件，解析失败会保留原始 tar。

默认下载后调用共享的 `tools/rosbag_to_json_mp4/convert_rosbag.py`。
先在安装了 SDK 的 Python 环境中安装解析依赖：

```bash
python3 -m pip install -r tools/rosbag_to_json_mp4/requirements.txt
# raw-only 下载的包也可稍后独立解析
python3 tools/rosbag_to_json_mp4/convert_rosbag.py \
  --bag-path ./data_collection_v2/demo_001.tar --output-dir ./parsed/demo_001 --replay-format both
# 离线检查，两种模式均不会连接机器人
./bin/data_replay_v2 ./parsed/demo_001/replay --mode joint_pos --dry-run
./bin/data_replay_v2 ./parsed/demo_001/replay --mode end_pose --dry-run
```

可通过 `--python /path/to/python` 选择解析环境。`--replay-format none|json|both`
与 Python 采集示例一致。C++ 回放需要 `both` 产生的 `stream_*.pb`；
Python 回放读取同目录中的 `episode.json`。JSON/MP4 的字段和机型适配见
[完整录制说明](../../python/v2/data_collection_v2/SDK2_RECORDING_CN.md)。

真实回放使用 `--address ROBOT_IP:55051` 并去掉 `--dry-run`，按 Enter 确认后下发动作；
`--speed` 调整速度，`--max-lag-ms` 指定过期指令的丢弃阈值（默认 0 不丢弃），
`--yes` 跳过确认。退出时交还本示例占用的 SDK 模式。

下载自动打印进度、速度和预计剩余时间；完成长度/MD5 校验与本地发布后才显示 `download_complete`。
`--status-timeout` 默认 30 秒，控制状态查询超时，后台轮询间隔仍为 5 秒。
`collect`/`download` 默认在下载落盘和长度/MD5 校验成功后清理该 session 的机器人 rosbag 和 tar。
需要保留远端文件时加 `--keep-robot-data`；原 `--cleanup-after-download` 参数保留兼容，两者互斥。
`start`/`stop`/`status` 不自动清理；SDK 通用下载 API 本身不删除文件。清理失败单独报错、本地 tar 保留；可通过
`--action delete --session-id SESSION_ID` 重试。独立 delete 会直接删除远端数据，请先确认已备份。
清理发生在解析之前，解析失败可用本地 tar 重试。详细接口和状态语义见完整录制说明。

## 头部回放限位

Python/C++ v2 回放在开始运动前检查头部数据，并将 pitch/yaw 指令裁剪到机型允许范围（rad）：

| 机型 | pitch | yaw |
| --- | --- | --- |
| quanta_x1 | [-0.06, 0.9] | [-1.2, 1.2] |
| quanta_x2 | [-0.52, 0.87] | [-1.57, 1.57] |
| quanta_x3 | [-0.523599, 0.261799] | [-1.570796, 1.570796] |

实机回放使用连接机型；`--dry-run` 使用 `episode.json` 中的 `model`。`Head clamp plan` / `head_clamp` 汇总整段数据中将被裁剪的记录数和各轴次数，不表示已执行次数。裁剪只作用于内存中的回放指令，原始 rosbag、JSON 和 protobuf 文件保持不变；超出范围的头部动作会停留在边界。头部数据缺失轴、包含 NaN/Inf 或机型未知时，回放在进入 SDK 控制模式前报错。

## 回放告警与底盘控制

v2 回放中的单条运动指令被拒绝时输出告警，并继续后续指令；结束时汇总失败次数。Python 同一操作的示例告警最多每 5 秒输出一次；C++ 每个数据流首次失败时告警，其余失败计入汇总。SDK 自身的 RPC 错误日志仍保留。工作模式切换等启动准备失败时不会开始回放。

Python/C++ v2 底盘回放均设置速度控制模式，依据当前里程计与记录的目标位置计算限速跟随指令（平移各轴不超过 0.3 m/s，偏航不超过 0.5 rad/s）。C++ 此行为仅在 data_replay_v2 中启用，原有回放入口保持原行为。

## 回放起点检查与部分数据

回放默认在确认开始后、切换 SDK/控制模式前读取实际参与回放部位的状态，与各数据流第一条有效指令比较。任一状态读取失败、数据格式不合法或误差超限，都会拒绝整个回放，不发送运动指令，也不自动回零或对齐。

默认阈值在示例目录的 `replay.conf` 中，可通过 `--start-check-config FILE` 指定完整配置：关节及头部 0.1 rad、末端位置 0.03 m、末端姿态 0.1 rad、升降 0.02 m、夹爪 0.1（SDK position 单位）、底盘平面位置 0.05 m/偏航 0.1 rad，状态读取超时 3 s。姿态比较考虑四元数 q 与 -q 等价；关节按 SDK 指令顺序比较，双方有名称时还校验名称顺序。无名称的 SDK 状态仍必须与记录的关节维度一致。

`--check-only` 连接机器人并只执行起点检查，不切换模式、不发送运动指令；`--dry-run` 仍仅检查本地数据，不验证机器人的当前状态。只读检查通过后开始正式回放时会再次检查，不能用先前结果跳过。

未录制的数据流不查询状态，也不发送对应指令。`joint_pos` 只使用关节数据控制双臂/腰部；`end_pose` 只使用末端数据。头部、夹爪、升降、底盘按各自记录独立参与。未录制双臂/腰部时不切换操作臂控制模式；未录制底盘时不发送底盘模式或停止指令；当前模式没有任何可回放数据时直接退出，不连接机器人。
