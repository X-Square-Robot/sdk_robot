# Quanta X3 C++ 示例

`quanta_x3` 目录只放 Quanta X3 的差异接口。头部、底盘、机器人控制、
音频、导航、相机和其他只读示例直接使用 `examples/cpp` 根目录的公共入口。

X3 专用产物只有：

- `quanta_x3_arm_control`：七轴 `joint_pos` 和 `end_pose` 轨迹，末端模式每周期
  同时刷新双臂和腰末端位姿（`/whole_body_controller/reasoning_target_head`）；
- `quanta_x3_gripper_control`：夹爪位置和位置流；
- `quanta_x3_lidar_points`：三维雷达点云的单次读取、流式读取和快速验证；
- `quanta_x3_waist_control`：升降、俯仰、偏航三个腰关节，以及腰末端位姿控制和状态流。

```bash
./quanta_x3_arm_control 127.0.0.1:55051 left move joint_pos --yes
./quanta_x3_arm_control 127.0.0.1:55051 left move end_pose --yes
./quanta_x3_gripper_control 127.0.0.1:55051 move left

# 雷达点云参数与 Python lidar_points.py 一致，全部为只读操作。
./quanta_x3_lidar_points --server 127.0.0.1:55051 --action single
./quanta_x3_lidar_points --server 127.0.0.1:55051 --action stream --seconds 3
./quanta_x3_lidar_points --server 127.0.0.1:55051 --action verify

# 腰部默认关节读取保持只读；参数与 Python waist_control.py 一致。
./quanta_x3_waist_control --server 127.0.0.1:55051
# 末端读取会确认并切换到 SDK + 末端模式，结束后恢复 IDLE。
./quanta_x3_waist_control --server 127.0.0.1:55051 --mode end_pose
./quanta_x3_waist_control --server 127.0.0.1:55051 --action stream --mode end_pose --seconds 5
# 升高 5 mm；启动前交互确认，自动化需显式添加 --yes。
./quanta_x3_waist_control --server 127.0.0.1:55051 --action move --joint-delta 0.005 0 0
# 腰末端目标沿反馈坐标系 z 方向增加 5 mm，双臂保持目标不变。
./quanta_x3_waist_control --server 127.0.0.1:55051 --action move --mode end_pose --pose-delta 0 0 0.005
# 偏航关节增加 10°，其他关节目标不变。
./quanta_x3_waist_control --server 127.0.0.1:55051 --action move --joint-delta 0 0 0.1745329252
# 末端目标绕反馈坐标系 Z 轴旋转 10°，保持目标 XYZ；pitch 为绕 Y 轴俯仰。
./quanta_x3_waist_control --server 127.0.0.1:55051 --action move --mode end_pose --pose-rotation yaw 0.1745329252

# 公共入口直接使用根目录产物。
./head_control 127.0.0.1:55051 stream
./robot_status 127.0.0.1:55051
./audio 127.0.0.1:55051 stream /tmp/microphone.wav
```

雷达入口仅接受 Quanta X3，只调用 `GetLidarPoints` / `GetLidarPointsStream`，
不切换工作模式。默认 `single`；`stream` 未指定时长时持续读取至 Ctrl+C；
`verify` 检查一次读取和 0.5 秒流式读取。点云校验与公共 `depth_points` 共用
`point_cloud.hpp`，支持行填充、大小端和单行紧凑布局；空数据、布局错误或 RPC
错误返回非零退出码。底盘深度相机点云仍使用公共 `depth_points`。

腰部示例与 Python 使用相同的五次时间缩放、速度/加速度/jerk 限制：至少 2 秒，
最多 50 Hz，逐点发送，延迟后不追赶补发。默认关节 verify 和 stream 不进入 SDK 模式。
`--mode end_pose` 的 verify 和 stream 会在确认后进入 SDK 工作模式、切换到末端控制模式，
使 WBC 开始发布位姿反馈；切模可能引起机器人运动，支持显式 `--yes` 确认。
这两种读取动作不发送关节或末端目标，退出时恢复 IDLE，包括读取失败或中断。
切模后首帧允许在 2 秒窗口内重试 `UNAVAILABLE` / `TIMEOUT`，每次读取仍为 300 ms。
运动必须给出非零相对增量；关节增量上限为 5 cm / 0.2 rad / 0.2 rad。
末端纯 Z 平移上限为 5 cm，含 X/Y 分量时总长度上限仍为 1 cm；单次末端姿态旋转最多
0.2 rad（约 11.46°）。三种运动参数不能混用。
`--pose-rotation pitch|yaw ANGLE_RAD` 使用固定反馈坐标系轴，四元数按 `q_delta * q_initial`
生成，保持初始目标 XYZ；默认角速度上限为 0.06 rad/s。目标保留关节限位裕量，末端运动每周期
刷新左臂、右臂和腰三路目标，实际关节变化可能与末端目标角度不同并伴随其他轴运动。

默认关节升降速度上限为 0.02 m/s，关节俯仰/偏航为 0.06 rad/s；
末端平移为 0.01 m/s，末端旋转为 0.06 rad/s。`--action move` 可用
`--linear-speed`（`[0.001, 0.04]` m/s）、`--angular-speed`（`[0.005, 0.12]` rad/s）覆盖速度上限。
示例参数范围不代表机械速度极限；加速度、jerk 和实际关节变化保护保持不变。默认 0.2 rad 偏航规划约 6.26 秒。
例如 `--joint-delta 0 0 0.2 --angular-speed 0.03` 可恢复原角速度上限。

每周期将 RPC 耗时计入 20 ms，只等待剩余时间，相邻目标至少间隔 20 ms；延迟后不跳点或补发。
单次反馈读取最多等待 300 ms；目标写入最多 300 ms，且受当前周期剩余预算限制。
整个周期的 400 ms 预算由反馈检查和全部目标写入共用，末端三路不能各自等待满 300 ms。
超时或周期停顿后停止后续目标发送，不自动重试失败的运动指令，并尝试恢复 IDLE。
日志输出规划时长及实际轨迹发送耗时；连接、切模、终点保持等耗时另计。
运动前后检查反馈时戳、关节名、限位、四元数和参考坐标系。
关节模式保留跟踪误差和终点收敛检查。末端模式允许 WBC 多任务求解产生位置、姿态偏差，
轨迹结束后继续刷新三路目标 1 秒，输出目标、反馈位置和实际关节变化，不等待精确到位。
改变末端 Z 目标不保证升降关节按同方向、同距离运动；需要明确升降量时使用关节模式的第一个值。
末端模式的实际关节变化仍限制在启动位置的 `[0.05 m, 0.15 rad, 0.15 rad]` 内。
姿态旋转时，仅对应轴的停止边界调整为 `max(0.15 rad, 请求角度绝对值 + 0.04 rad)`；
机械限位与裕量检查仍保留。
完成表示命令序列执行完毕；未观察到净关节变化时打印 WARNING。退出时交还本示例占用的
SDK 模式；清理失败返回错误。
这些检查不提供 IK 或碰撞预检，运行前仍需确认现场空间与急停。参数边界见
[Python X3 说明](../../python/v2/quanta_x3/README.md)，离线测试逐点比较两种实现的轨迹。

手臂示例使用 TOPP-RA，末端模式同样每周期刷新双臂和腰的 `SetEndPose` 目标。
