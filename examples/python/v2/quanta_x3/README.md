# Quanta X3 Python v2 示例

公共头部、底盘、系统、音频、导航接口复用 `examples/python/v2/` 根目录入口。
本目录包含 `arm_control.py`、`gripper_control.py`、`waist_control.py`、雷达点云入口
`lidar_points.py` 和本地测试音生成器 `audio_pattern.py`。
腰部入口由 `lift_control.py` 重命名，使用 `robot.waist`。

## 三维雷达点云

`lidar_points.py` 只读取 Quanta X3 的雷达点云，调用
`DepthPoints.GetLidarPoints` / `GetLidarPointsStream`。底盘深度相机点云继续使用
上一级公共入口 `depth_points.py`。

从仓库根目录执行：

```bash
python examples/python/v2/quanta_x3/lidar_points.py --server 127.0.0.1:55051 --action single
python examples/python/v2/quanta_x3/lidar_points.py --server 127.0.0.1:55051 --action stream --seconds 3
python examples/python/v2/quanta_x3/lidar_points.py --server 127.0.0.1:55051 --action verify
```

流式读取默认持续到 Ctrl+C，也可用 `--seconds` 指定读取时长。验证会检查单次及短时流式读取。
C++ 对应入口为 `quanta_x3_lidar_points`，参数相同，见 [C++ X3 说明](../../../cpp/quanta_x3/README.md)。

## 腰部平滑控制

`waist.set_joint_positions(JointPositions(...))` 的顺序为
**升降（m）、俯仰（rad）、偏航（rad）**，不使用 ROS 发布端的排列顺序。
`waist.set_end_pose(geometry_msgs.Pose(...))` 支持保持目标姿态的平移，或保持目标 XYZ 的单轴旋转；
每周期同时发送双臂当前保持位姿和腰目标（`reasoning_target_head`）。

```bash
# 默认只读，不进入 SDK 模式。
python waist_control.py --server 127.0.0.1:55051 --action verify
python waist_control.py --server 127.0.0.1:55051 --action stream --mode joint_pos --seconds 5
# 末端反馈需要切换模式；确认后进入 SDK + 末端模式，读取后恢复 IDLE。
python waist_control.py --server 127.0.0.1:55051 --mode end_pose
python waist_control.py --server 127.0.0.1:55051 --action stream --mode end_pose --seconds 5
# 腰升高 5 mm，其他关节不变；也可分别给俯仰或偏航小幅增量。
python waist_control.py --server 127.0.0.1:55051 --action move \
  --mode joint_pos --joint-delta 0.005 0 0
# 腰末端目标沿反馈坐标系 z 方向增加 5 mm，双臂保持目标不变。
python waist_control.py --server 127.0.0.1:55051 --action move \
  --mode end_pose --pose-delta 0 0 0.005
# 偏航关节增加 10°，其他关节目标不变。
python waist_control.py --server 127.0.0.1:55051 --action move \
  --mode joint_pos --joint-delta 0 0 0.1745329252
# 腰末端目标绕反馈坐标系 Z 轴旋转 10°；pitch 表示绕 Y 轴俯仰。
python waist_control.py --server 127.0.0.1:55051 --action move \
  --mode end_pose --pose-rotation yaw 0.1745329252
```

默认关节 `verify` / `stream` 保持只读。末端 `verify` / `stream` 先进入 SDK 工作模式、
切换到末端控制模式，再读取 WBC 位姿反馈；切模可能引起机器人运动，因此与 `move`
一样需要交互确认，自动化可显式传 `--yes`。仅确认现场空间、急停和版本后运行。
末端验证和流读取不发送关节或末端目标。切模后首帧允许在 2 秒窗口内重试
`UNAVAILABLE` / `TIMEOUT`，每次读取仍为 300 ms；其他错误立即退出。
Ctrl+C、RPC 失败、关节越限或反馈超时均停止后续发送，并在关闭连接前尝试恢复
IDLE；恢复失败会报告错误并以非零状态退出，不代表急停或硬件已经停稳。

`--pose-rotation pitch|yaw ANGLE_RAD` 的角度单位为弧度，正负号表示旋转方向。
旋转围绕反馈坐标系的固定轴，目标四元数按 `q_delta * q_initial` 生成，保持初始目标 XYZ。
一次运动只选关节增量、末端平移或末端旋转中的一种；三种参数不能混用。
末端目标旋转不保证腰关节以相同角度运动，也可能伴随升降或另一角度变化。

## 速度参数

默认速度上限已提高；加速度、jerk 和限位裕量保持不变：

| 运动 | 原默认速度上限 | 当前默认速度上限 | 覆盖参数 |
| --- | --- | --- | --- |
| 关节升降 | 0.01 m/s | 0.02 m/s | `--linear-speed` |
| 关节俯仰 / 偏航 | 0.03 rad/s | 0.06 rad/s | `--angular-speed` |
| 末端平移 | 0.005 m/s | 0.01 m/s | `--linear-speed` |
| 末端俯仰 / 偏航 | 0.03 rad/s | 0.06 rad/s | `--angular-speed` |

速度参数只用于 `--action move`：`--linear-speed` 接受 `[0.001, 0.04]` m/s，
`--angular-speed` 接受 `[0.005, 0.12]` rad/s。这些是示例允许的参数范围，不是机械速度极限。
参数只替换速度上限，规划仍同时满足加速度和 jerk 约束，最短时长为 2 秒。
默认 `0.2 rad` 偏航规划约 6.26 秒，5 cm 关节升降约 4.70 秒；连接、切模、
终点保持和 RPC 超过周期的耗时另计。可显式调低速度：

```bash
python waist_control.py --server 127.0.0.1:55051 --action move \
  --joint-delta 0 0 0.2 --angular-speed 0.03
python waist_control.py --server 127.0.0.1:55051 --action move \
  --mode end_pose --pose-delta 0 0 0.01 --linear-speed 0.005
```

## 规划与检查

- 从当前反馈起步，五次时间缩放使起止速度、加速度为零；按速度、加速度和 jerk
  上限计算时长，至少 2 秒，最多 50 Hz，逐点发送，卡顿不追赶补发。
- 发送前检查整条轨迹、维度、有限数值、关节名、关节与末端范围、单位四元数。
  运行时检查反馈时戳及关节限位；关节模式保留跟踪误差与终点收敛检查。
  末端模式不要求反馈精确到达目标，轨迹结束后继续刷新三路目标 1 秒，再输出
  目标位置、反馈位置及实际腰部关节变化。WBC 多任务加权求解可能产生位置和姿态偏差。
  改变末端 Z 目标不保证升降关节按同方向、同距离运动；需要明确升降量时使用关节模式的第一个值。
  此时完成只表示命令序列执行完毕；未观察到净关节变化时打印 WARNING。
- 配置位于 `../config/quanta_x3/waist.json`。关节限位来自 服务端 Quanta X3 腰部配置：
  `[0, 0.371] m`、`[-0.0523, 0.872665] rad`、`[-1.570796, 1.570796] rad`。
  目标保留 2 mm / 0.005 rad 裕量。单次关节增量最多 5 cm / 0.2 rad / 0.2 rad；
  末端纯 Z 平移最多 5 cm；含 X/Y 分量的平移总长度仍最多 1 cm。默认速度上限为 10 mm/s；
  单次姿态旋转最多 0.2 rad（约 11.46°），
  默认角速度、角加速度、角 jerk 上限为 0.06 rad/s、0.06 rad/s²、0.24 rad/s³。
  末端模式仍要求关节处于限位裕量内，并限制相对启动位置的实际关节变化不超过
  `[0.05 m, 0.15 rad, 0.15 rad]`。姿态旋转时，仅对应旋转轴的边界调整为
  `max(0.15 rad, 请求角度绝对值 + 0.04 rad)`；机械限位与裕量检查仍保留。
  这是演示的停止边界，不是到位误差要求。
- 每周期把 RPC 耗时计入 20 ms 周期，只等待剩余时间；相邻目标发送至少间隔 20 ms。
  慢周期逐点继续，不跳点、不补发追赶。日志分别输出规划时长和实际轨迹发送耗时。
- 单次反馈读取最多等待 300 ms；目标写入最多等待 300 ms，且不能超过当前周期剩余预算。
  整周期预算为 400 ms，包含反馈检查和全部目标写入；末端三路写入共享该预算，
  不会各自独占 300 ms。超时或停顿后停止后续目标发送，不自动重试失败的运动指令，并尝试恢复 IDLE。
  机器人与客户端时钟必须同步，反馈最大允许年龄为 1 秒。
- 末端 ±5 m 范围仅为服务器命令校验框，**不是机械可达空间**。现有接口没有
  IK 或碰撞预检；小幅平移和运行时关节检查不能保证避障或可达，需实机空旷环境验证。

手臂示例使用 TOPP-RA，并在末端模式持续刷新双臂及腰三路目标。
夹爪示例保持原有读写与状态流接口。C++ `quanta_x3_waist_control` 与 Python 腰部示例
使用相同参数和控制逻辑，见 [C++ X3 说明](../../../cpp/quanta_x3/README.md)。
