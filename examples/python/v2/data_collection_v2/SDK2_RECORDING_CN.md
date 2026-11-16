# SDK2 录制式采集与回放

Python 示例、机型预设和本说明位于 `examples/python/v2/data_collection_v2/`。
C++ 对应目录为 [`examples/cpp/data_collection_v2/`](../../../cpp/data_collection_v2/README_CN.md)，
可执行程序为 `data_collection_v2`、`data_replay_v2`。原采集和回放入口保持原有行为。

采集通过 Recording 接口请求机器人 recorder 录制选中 topic，不打开 GetXXXStream。
停止后 SDK Server 将已关闭的 MCAP bag 与 manifest 归档为未压缩 tar，客户端通过
通用流式下载接口校验长度及 MD5。默认继续解析为 JSON、MP4 和 Python/C++ 回放数据。

## 本地预设

两种语言使用相同的 `.conf` 格式和配置内容，均在各自目录的 `config/` 下。
配置列出数据项 ID、英文描述和 `enabled` 开关；不包含内部平台、ROS 话题或类型。

| 预设 | 机型 | 默认启用范围 |
| --- | --- | --- |
| `quanta_x1` | Quanta X1 | 整机关节、双臂末端、双臂世界/局部力与力矩、ODOM；前置 RGB、左右腕部相机 |
| `quanta_x1_dual_head_camera` | Quanta X1 | 同上；相机为头部左右眼、左右腕部、左右肘部，共六路 |
| `quanta_x2` | Quanta X2 | 双臂/腰/头关节、双臂与腰末端、ODOM；前置 RGB、左右腕部相机 |
| `quanta_x3` | Quanta X3 | 整机关节、双臂与腰末端、ODOM、IMU；头部左右眼、左右腕部相机 |

双目头部是采集预设，机器人 `model` 仍为 `quanta_x1`。
未指定配置时，采集按连接机器人的公开机型选择同名预设；双目 Quanta X1 必须显式选择
`--preset quanta_x1_dual_head_camera`。不再通过服务端查询自动判断相机或末端装配。
默认腕部相机按平行夹爪配置启用，装配灵巧手时需按实际相机修改开关。
专属夹爪/灵巧手状态、控制和触觉项默认关闭，英文说明注明所需末端类型；用户启用实际装配的项。

```bash
# 本地查看全部配置项、英文说明及开关；不连接机器人，不依赖安装 SDK
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --action list --preset quanta_x1_dual_head_camera
# 在已安装 SDK 的 Python 环境中安装离线解析依赖
python -m pip install -r tools/rosbag_to_json_mp4/requirements.txt
# 双目 Quanta X1：录制、停止、下载、解析
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --preset quanta_x1_dual_head_camera --duration 10 --output-dir ./collected
# 只下载 rosbag，稍后可用独立脚本解析
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --preset quanta_x3 --raw-only --output-dir ./raw
python tools/rosbag_to_json_mp4/convert_rosbag.py \
  --bag-path ./raw/SESSION_ID.tar --output-dir ./parsed/SESSION_ID --replay-format both
```

可以复制预设并修改 `enabled`，或编写仅包含所需项的 `capture.conf`：

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

```bash
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --config ./capture.conf --duration 10
# 显式 ID 覆盖配置选择，不读取配置文件
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --items imu odom --raw-only
```

优先级为 `--items` > `--config` > `--preset` > 机型同名预设。
`--action list` 是本地配置说明，省略选择时展示 quanta_x1，不代表机器人当前硬件能力。
启录时直接提交所有启用的 ID，recorder 根据实际机型、平台和左右末端配置校验；
任一项未知或不适用时，整个 Start 失败，SDK 错误信息返回具体 item ID，不静默漏录。
选择项必须非空且不重复；只有显式 `--items all` 才选择 recorder 当前全部项。
配置不支持 `optional_items` 自动筛选；`enabled` 只接受 true/false，重复节/键、未知字段、
空描述和错误机型会报错。默认配置路径不依赖工作目录，status/stop/download 不读取配置。

整机 `/joint_states` 在机器人上只录一份原消息，由客户端按 `name` 拆分。
Quanta X1/X3 默认选择 `joint_states`；Quanta X2 保留各部位独立状态项，
其中 `head_joint_states` 对应 `/head/joint_states`，客户端按头部关节名称解析。
主臂和超声波仍不在本轮录制范围内；控制、触觉、深度图像等按配置说明手动启用。

`--duration 0` 交互式等待 Enter。也可分别调用 `--action start`、`status`、`stop`、
`download`；stop/download 必须提供 start 打印的 `--session-id`。Stop 只结束并等待
归档，不自动下载；download 只查询已完成产物。不要用新 ID 重试不确定的 Start。
默认全流程 `collect` 在 Ctrl-C 或等待失败时仍尝试停止已开始的会话。
服务超时/客户端异常退出不能证明已停录，可重连后用原 ID 查询并停止。

服务端需要显式配置 `--file_transfer_config`，参考 sdk_server 仓库
`sdk2/config/file_transfer.example.yaml`。`recording.raw_root` 必须等于 recorder SDK
catalog 的 output_root；文件服务 `recordings` root 指向独立的 artifacts 目录且只读。
普通上传目录单独配置。SDK 原始录制默认位于 `/home/xr/bagfiles/sdk/recordings`，
归档位于 `/home/xr/bagfiles/sdk/artifacts`；recorder 和 SDK server 容器都必须挂载
宿主机的 `/home/xr/bagfiles`。不要将 SDK 数据放进普通数采的 `bags`/`bags_temp` 目录。

recorder 的各机型 SDK `recording_config.conf` 使用 `minimum_free_space_gib: 10` 设置
最低可用空间（1 GiB = 1024³ 字节，未配置或为 0 时兼容使用 10 GiB）。开始录制时检查
`output_root` 实际所在卷的可用空间；低于阈值返回 `DISK_SPACE_LOW`，不创建录制会话。
录制期间每秒检查一次，即使客户端或 SDK server 退出也生效。低于阈值会停止订阅、
收尾 rosbag，最终状态为 `FAILED`，保留 `DISK_SPACE_LOW` 和具体原因；空间查询失败
使用 `DISK_CHECK_FAILED` 并停止。原始数据保留，不自动打包或删除。释放空间后才能
开启新会话；已停止的数据如需导出，应从原始目录取回，不能将 FAILED 当成 READY 下载。
Python/C++ 的 `StartRecording` 支持可选 callback：开始成功后在后台默认每 5 秒查询一次
现有 `GetRecordingStatus`，不增加事件 RPC。每次状态查询默认允许 30 秒，独立于示例开始/停止
请求的 10 秒超时；示例可用 `--status-timeout 60` 调整为 60 秒，轮询间隔仍为 5 秒。
当前查询结束后再等待轮询间隔，不会因为查询耗时长而叠加新的后台请求。示例主流程确认
READY 后会取消后台轮询，再开始下载。首次回调包含开始接口返回的状态，后续仅在
状态/错误变化时回调；同一次请求失败反复发生不会重复通知，查询恢复也会回调。
到达 `READY`/`FAILED`/`DELETING`/`DELETED` 后自动结束轮询；`STOPPING`/`FINALIZING` 继续查询。
轮询返回当前快照，可能跳过短暂中间状态，但 recorder 保留终止原因。通知延迟通常为
0～5 秒加 RPC 耗时，与 recorder 每秒执行的磁盘保护相互独立。

回调在后台线程执行，应尽快返回；异常会记录日志并结束该回调的轮询，不停止机器人录制。
`result.is_success == False`（C++ `!result.Ok()`）表示查询/通信失败，客户端每 5 秒重试；
成功查询中的 `state == FAILED` 或非空 `error_code` 才是录制自身的错误。
关闭连接会取消轮询及在途 RPC；建议在主线程关闭连接，避免等待自身回调。
同一连接重复为同一个 session 注册回调会替换旧轮询；不传 callback 的旧调用方式不创建轮询。
Python `cancel_recording_callback(id)` / C++ `CancelRecordingCallback(id)` 仅取消通知，
停止机器人录制仍需 `StopRecording`。独立 `start` 示例返回并关闭连接后不再通知，后续用
`status` 查询/`stop` 停止；限时 `collect` 示例会保持连接并在错误时尝试停止。

SDK server 的文件服务 YAML 中，`recording.minimum_free_space_gib: 10` 设置打包后的
最低预留空间（必须为正整数，默认 10），建议与 recorder 配置一致。修改后重启对应服务
生效。recorder 没有 MD5/打包职责；SDK 归档前检查包大小和“归档所需空间 + 预留空间”，
并在同一文件系统发布完成归档，归档期间拒绝新 Start。

`STARTING → RECORDING → STOPPING → FINALIZING → READY` 中，只有 READY 的 file_id
可以用于下载。返回的错误状态 FAILED 必须处理，不能只检查 RPC 是否成功。
调用例子：

```python
from x2robot.sdk.v2 import StartRecordingRequest, RecordingSessionRequest
def on_recording(result):
    if not result.is_success:
        print("Recording status query failed:", result.error)
        return
    status = result.data
    print(status.state, status.error_code, status.error_message)

started = robot.recording.start_recording(
    StartRecordingRequest(session_id="sample_001", data_items=["joint_states", "head_left_eye_rgb"]),
    timeout=10, callback=on_recording, poll_interval=5.0, status_timeout=30.0)  # 单位：秒，可省略
status = robot.recording.get_recording_status(RecordingSessionRequest(session_id="sample_001"), timeout=30)
stopped = robot.recording.stop_recording(RecordingSessionRequest(session_id="sample_001"), timeout=10)
# 持续查询到 READY，再调用 robot.files.download_file(status.data.file_id, "sample_001.tar")
```

C++ callback 重载为 `StartRecording(request, callback, poll_interval, deadline, status_deadline)`，
轮询间隔、开始请求超时、状态查询超时使用 `Deadline`（毫秒），默认分别为 `5000`、
`10000` 和 `30000`；callback 参数类型为
`const SdkResult<xr::sdk::v2::RecordingStatus>&`。例如：

```cpp
auto started = robot->GetRecording().StartRecording(request, [](const auto& result) {
    if (!result.Ok()) { std::cerr << result.ErrorValue().error_message << std::endl; return; }
    std::cout << result.Value().state() << " " << result.Value().error_code() << std::endl;
}, x2robot::sdk::Deadline{5000});
```

C++ 同时保留 `robot->GetRecording().StartRecording(request)`、`StopRecording(id)`、
`GetRecordingStatus(id)`，返回统一 SdkResult。查询本次结果不要求持有运动控制会话。

解析输出每个非图像 topic 一个 JSON，包含原始消息结构和整数纳秒 bag 时间戳；
`/joint_states` 的每条记录额外提供 `joints_by_name` 字典，例如
`{"left_arm_joint1": {"position": 0.1, "effort": 0.2}}`。客户端逐消息按 `name` 关联三个数组，
支持关节顺序变化和未列入回放配置的关节；缺失的 velocity/effort 等字段不补零，原始 `data` 仍完整保留。
重复/空关节名或非空数组长度与 name 不符时解析失败，避免错误关联。
非有限浮点值写为 JSON null。每个相机输出 MP4、`.frames.json` 帧序号/时间戳映射；
`summary.json` 记录消息数、实际解码帧数、首帧时间、解码错误与时间戳调整数量。
相机话题存在但没有收到消息时，跳过该相机，summary 标记 `skipped_reason: "no_messages"`、
`file: null`，其余数据继续解析；不会生成空 MP4。收到消息但始终无法解码出画面时仍报错。
H264/H265 跨 ROS 消息拼接解析，开头缺少关键帧时等待可解码数据，不补黑帧或复制帧。
输出保留可变时间间隔；MP4 微秒时间基引起的重复时间戳会做最小递增并计数。
完全无可解码帧、未知消息定义、损坏包或分辨率变化会失败，保留下载的原始包。
支持 MCAP/SQLite rosbag2 目录及 SDK tar；自定义类型需要包内嵌入消息定义。
输出目录必须不存在；解析成功后才发布完整目录。

## Python / C++ 回放格式

`data_collection_v2.py` 默认使用 `--replay-format both`，在 JSON/MP4 之外生成：

- `replay/episode.json`：Python v2 回放读取的逐事件 observation，关节字段为 `positions`。
- `replay/stream_*.pb`：C++ `data_replay_v2` 读取的 SDK protobuf 消息，
  每条记录为小端 `uint64 timestamp_ns + uint32 payload_size + payload`。
- `summary.json` 的 `replay` 字段：各流来源、记录数、首末时间，以及未转换成回放指令的话题。

```bash
# 以下均为离线规划，不连接机器人、不下发动作
python examples/python/v2/data_collection_v2/data_replay_example.py ./parsed/SESSION_ID/replay --mode joint_pos --dry-run
python examples/python/v2/data_collection_v2/data_replay_example.py ./parsed/SESSION_ID/replay --mode end_pose --dry-run
./artifacts/bin/data_replay_v2 ./parsed/SESSION_ID/replay --mode joint_pos --dry-run
./artifacts/bin/data_replay_v2 ./parsed/SESSION_ID/replay --mode end_pose --dry-run
# Python 末端回放入口也可以读取相同 episode.json
python examples/python/v2/quanta_x2/end_pose_replay_new.py ./parsed/SESSION_ID/replay --dry-run
```

录制状态与 SDK manifest 的 `model`、默认 `profile_id` 使用 `quanta_x1/quanta_x2/quanta_x3`。
解析器从 SDK manifest 读取机型；普通 rosbag 需要显式指定 `--replay-model quanta_x1|quanta_x3|quanta_x2`。
`--replay-format json` 只导出 Python 格式，`none` 保留原来的逐 topic JSON/MP4；
`both` 还需要安装本轮 SDK Python 包，以复用生成的 protobuf 类型。`--raw-only` 跳过全部解析。

回放导出使用反馈数据：双臂关节/末端位姿、腰/升降、头部、单自由度夹爪和 ODOM。
整机 `/joint_states` 按关节名称拆分并按 SDK 顺序排列；quanta_x3 腰部顺序为
`lift_joint, waist_pitch_joint, waist_yaw_joint`，不会直接复制底层控制器指令顺序。
选择 `joint_states` 即在客户端导出消息中实际包含的臂、头、腰/升降和单自由度夹爪反馈，
无需另外选择部位 item。机器人端仅录制原消息，不拆包或重发布关节状态。
解析历史包时仍支持独立部位反馈和旧 item 别名；同时存在整机与部位 topic 时优先使用部位 topic，避免重复。
SDK manifest 的 acceptedItems 限定所选数据；旧包仅选择某个部位别名时仍只导出该部位。
不使用补帧、插值或最近值填充；Python 逐事件调度使用绝对相对时间，C++ 保留整数纳秒时间戳。
Python 回放在没有 ODOM 时不启动底盘控制线程。

控制指令、灵巧手、力矩、触觉、IMU 和相机仍保留在原始 JSON/MP4 中，现有回放入口不会将
这些记录自动当成运动目标；`topics_without_replay_commands` 会计数。缺失完整部位反馈会计入
`missing_aggregate_parts`；重复关节名、部分关节缺失、错误维度或非有限值会使回放导出失败，
保留原始 rosbag。仅录相机/IMU 等数据时回放帧数为 0，仍可正常使用 JSON/MP4。

显式离线验证入口（需已构建 C++ examples 并安装解析依赖）：

```bash
SDK_ARTIFACT_ROOT=./artifacts bash scripts/run_rosbag_tests.sh python
```

## 数据项校验边界

SDK 和 ROS2 不再提供数据项目录查询接口。客户端预设负责选择和说明，recorder 的本机 catalog
负责 ID 到 ROS topic/type 的映射、硬件匹配和启录校验。修改实际硬件配置后需重启 recorder。
公开 ID 使用 left_hand_commands、left_gripper_commands、waist_commands、left_arm_wrench_local 等简短名称。
相机、IMU、ODOM 及所需部位的状态、控制、力矩和触觉按 catalog 提供；任务和音频不在目录中。
预设不保证发布者在线；录制到的实际消息数以 rosbag 和解析 summary.json 为准。
深度相机可以保留原始 bag；当前解析脚本不支持 compressedDepth 深度解码。

## 下载进度与远端清理

Python/C++ 采集示例在下载阶段自动打印已接收字节、总字节、百分比、平均速度及预计剩余时间，
每秒最多输出一条进度；最终 `download_complete` 表示长度/MD5 校验和本地文件发布均已成功。
开始等待元数据时也会打印提示。收到 100% 字节仍可能等待校验，不能据此自行删除机器人数据。
SDK 通用下载方法本身不打印，调用者可以传入进度回调：

```python
result = robot.files.download_file(file_id, "session.tar", timeout=600,
    progress=lambda received, total, verified: print(received, total, verified))
```

C++ 对应 `DownloadFile(file_id, local_path, Deadline{600000}, callback)`，callback 参数是
`(uint64_t received, uint64_t total, bool verified)`。最后一次 `verified=true` 在本地发布成功后触发；
回调抛异常会停止进度通知、记录日志，传输继续。下载错误不发出成功通知，也不覆盖已有本地文件。

Python/C++ 的 `collect`/`download` 示例默认在 SDK 下载落盘并通过长度/MD5 校验后，
调用 `Recording.DeleteRecording` 清理该 session 的机器人原始 rosbag 和 tar。
需要保留机器人数据时传入 `--keep-robot-data`。原 `--cleanup-after-download` 参数保留兼容，
与 `--keep-robot-data` 互斥；两者仅适用于 `collect`/`download`。`start`/`stop`/`status` 不自动清理。
SDK 通用 `Files.download_file`/`Files.DownloadFile` 本身不删除文件，由采集示例执行此默认策略。
清理发生在本地解析之前；即使随后解析失败，本地已验证的 tar 仍保留，可以重新解析。

```bash
# 下载并通过 MD5 校验后默认清理机器人上的该 session
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --action download --session-id SESSION_ID --raw-only
# 需要保留机器人数据时显式指定
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --action download --session-id SESSION_ID --raw-only --keep-robot-data
# 独立清理或重试清理，不要求重新下载；请先确认数据已备份或不再需要
python examples/python/v2/data_collection_v2/data_collection_v2.py \
  --address ROBOT_IP:55051 --action delete --session-id SESSION_ID
# C++ 支持相同参数
./bin/data_collection_v2 --address ROBOT_IP:55051 --action delete --session-id SESSION_ID
```

接口：Python `robot.recording.delete_recording(RecordingSessionRequest(session_id=...), timeout=30)`；
C++ `robot.GetRecording().DeleteRecording(session_id, Deadline{30000})`。两者返回 `RecordingStatus`，
成功状态为 `DELETED`，无可下载的 file_id。独立清理是显式删除操作，服务端不判断客户端是否已有备份。

SDK server 只按合法 session ID 清理配置目录内该会话的原始 bag、tar 和原状态 JSON，
不删除普通数采数据，不让 recorder 计算 MD5 或处理清理。录制启动/进行/停止、打包或下载期间拒绝清理；
允许清理终止失败的录制。路径含符号链接或异常目录结构时拒绝处理。
保留很小的 `SESSION_ID.tar.deletion.json` 标记以防止重启后重新打包或复用已删除的 ID。
清理中断时状态为 `DELETING`，再次调用同一 ID 可继续；完成后重复调用仍返回 `DELETED`。
下载失败、取消、MD5 不匹配或本地文件已存在都不会自动清理。清理失败会单独报错并返回非零退出码，
已成功下载的本地文件仍保留，解析仍会尝试执行；可用独立 delete 命令重试。

### 关节回放限位

普通和 `data_collection_v2` 的 Python/C++ 回放共用 `examples/config/replay_joint_limits.conf`，
根据连接机型裁剪实际发送的头部、左右臂、腰部关节位置，以及 Quanta X1 的升降位置。
配置来自各机型服务端 `state_limit`，支持 quanta_x1、quanta_x2、quanta_x3 和 desktop（仅双臂）；
当前 SDK 没有限位查询接口，机器人使用定制限位时需同步这份随包配置。
角度单位为 rad，升降为 m；Quanta X3 腰部顺序是 `[lift, pitch, yaw]`。
头部和升降的 float32 边界向允许范围内取整，避免序列化后再次越界。

`Joint clamp plan` / `joint_clamp` 按部位汇总计划记录数、被裁剪记录数和各关节次数，不表示已执行次数。
原始 JSON、protobuf 和 rosbag 保持不变。末端位姿不裁剪，末端模式中未使用的手臂/腰部关节记录也不参与校验；
该模式仍发送的头部和升降指令会裁剪。关节数量错误、非有限数值和缺失机型限位会在进入 SDK 模式前报错。
v2 的起点检查使用裁剪后的指令，v2 的 `--dry-run` 使用录制清单中的机型；实机回放使用连接机型。

### 回放告警与底盘控制

v2 回放中的单条运动指令被拒绝时输出告警，并继续后续指令；结束时汇总失败次数。Python 同一操作的示例告警最多每 5 秒输出一次；C++ 每个数据流首次失败时告警，其余失败计入汇总。SDK 自身的 RPC 错误日志仍保留。工作模式切换等启动准备失败时不会开始回放。

Python/C++ v2 底盘回放均设置速度控制模式，依据当前里程计与记录的目标位置计算限速跟随指令（平移各轴不超过 0.3 m/s，偏航不超过 0.5 rad/s）。C++ 此行为仅在 data_replay_v2 中启用，原有回放入口保持原行为。

### 回放起点检查与部分数据

回放默认在确认开始后、切换 SDK/控制模式前读取实际参与回放部位的状态，与各数据流第一条有效指令比较。任一状态读取失败、数据格式不合法或误差超限，都会拒绝整个回放，不发送运动指令，也不自动回零或对齐。

默认阈值在示例目录的 `replay.conf` 中，可通过 `--start-check-config FILE` 指定完整配置：关节及头部 0.1 rad、末端位置 0.03 m、末端姿态 0.1 rad、升降 0.02 m、夹爪 0.1（SDK position 单位）、底盘平面位置 0.05 m/偏航 0.1 rad，状态读取超时 3 s。姿态比较考虑四元数 q 与 -q 等价；关节按 SDK 指令顺序比较，双方有名称时还校验名称顺序。无名称的 SDK 状态仍必须与记录的关节维度一致。

`--check-only` 连接机器人并只执行起点检查，不切换模式、不发送运动指令；`--dry-run` 仍仅检查本地数据，不验证机器人的当前状态。只读检查通过后开始正式回放时会再次检查，不能用先前结果跳过。

未录制的数据流不查询状态，也不发送对应指令。`joint_pos` 只使用关节数据控制双臂/腰部；`end_pose` 只使用末端数据。头部、夹爪、升降、底盘按各自记录独立参与。未录制双臂/腰部时不切换操作臂控制模式；未录制底盘时不发送底盘模式或停止指令；当前模式没有任何可回放数据时直接退出，不连接机器人。
