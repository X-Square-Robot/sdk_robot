# Quanta X3 示例入口

Quanta X3 的示例按“公共入口 + 机型差异入口”组织：公共能力放在
`examples/python/v2` 或 `examples/cpp` 根目录，X3 差异入口包括 arm、waist、gripper、
雷达点云 `lidar_points` 以及 Python 本地测试音生成器。腰部包含升降、俯仰、偏航三个关节。

Quanta X3 的 Livox Mid-360 提供点云数据，没有 LaserScan `/scan` 数据源，因此 SDK2 不公开
`RadarService.GetLaserScan`；运行公共 `radar.py --action verify` 时会明确显示 skipped。
三维雷达数据通过 `DepthPoints.GetLidarPoints` / `GetLidarPointsStream` 返回原始
`PointCloud2`，保留坐标系、时间戳、字段布局与数据；底盘深度相机仍使用
`GetChassisDepthPoints`，两路不会混用。Python 底盘深度相机使用公共 `depth_points.py`，
雷达点云使用 `quanta_x3/lidar_points.py`，支持单次、流式读取和验证。
C++ `GetLidarPoints()` 默认等待首帧 3 秒，调用者可显式指定 deadline。

```bash
python examples/python/v2/quanta_x3/lidar_points.py --server 127.0.0.1:55051 --action verify
./build/examples/quanta_x3_lidar_points --server 127.0.0.1:55051 --action single
./build/examples/quanta_x3_lidar_points --server 127.0.0.1:55051 --action stream --seconds 3
./build/examples/quanta_x3_lidar_points --server 127.0.0.1:55051 --action verify
```

Quanta X3 硬件没有底盘 TOF，因此 SDK2 不公开 `Tof.GetChassisTof1/2`；运行公共 `tof.py --action verify`
时会明确显示 skipped。

## 入口布局

| 能力 | Python v2 | C++ |
|---|---|---|
| 手臂 | `python/v2/quanta_x3/arm_control.py` | `quanta_x3_arm_control` |
| 夹爪 | `python/v2/quanta_x3/gripper_control.py` | `quanta_x3_gripper_control` |
| 三关节腰部 | `python/v2/quanta_x3/waist_control.py` | `quanta_x3_waist_control` |
| 底盘深度相机点云 | `python/v2/depth_points.py` | `depth_points` |
| 三维雷达点云 | `python/v2/quanta_x3/lidar_points.py` | `quanta_x3_lidar_points` |
| 公共头部/底盘/系统/音频/导航/只读 | `python/v2/*.py` | `*.cpp` 根目录入口 |
| 测试音 | `python/v2/quanta_x3/audio_pattern.py` | — |

## 手臂轨迹

Python 和 C++ 的 X3 手臂示例使用七轴限位和 TOPP-RA 轨迹，并采用同一组 X3 实测起始姿态。
轨迹按单调时钟的实际经过时间取点，RPC 耗时计入轨迹进度；发送较慢时跳过过期采样点，
不把预设采样频率当作实际发送频率，也不集中补发旧命令。结束时发送最终目标。
`joint_pos` 先到实测起始关节姿态，再将关节 4 增加 `0.2 rad`；`end_pose` 先发送
`(0.125556, ±0.193477, 0.857640)` 的准备位目标，再将目标 z 增加 `0.1 m`（10 cm）。
另一臂和腰在整个动作中共用首次捕获的保持点。每段末端轨迹结束后，继续刷新
三路目标 1 秒，并以普通信息输出运动臂、另一臂和腰部的位置/姿态残余误差
（mm、deg）。WBC 多任务求解允许目标与反馈存在偏差；示例不按精度判断成功，
也不因未精确到位报错或输出 WARNING。完成表示轨迹发送与短时保持流程完成，
不代表精度验收通过。RPC 失败、非有限位姿或无效四元数仍报错退出并尝试恢复 IDLE。
SDK 位姿是控制器内部位姿，实机验收还需独立关节反馈核对实际到位和保持误差。
末端保持允许冗余关节微调，
不保证另一臂全部关节、腰和升降完全静止。
X3 末端模式每周期发送双臂和腰三路 `SetEndPose`；非运动部件保持当前反馈位姿。

```bash
python examples/python/v2/quanta_x3/arm_control.py \
  --server 127.0.0.1:55051 --arm left --action move --mode end_pose --yes
./build/examples/quanta_x3_arm_control \
  127.0.0.1:55051 left move end_pose --yes
```

运动入口会先提示确认；Ctrl+C 可强制退出。部署或实机验证前仍需确认 SDK2
监听端口、协议版本、现场空间和实体急停状态。

## 数据采集与回放

Python 和 C++ 的 Quanta X3 默认采集配置同时录制关节数据及左臂、右臂、腰部
三路末端位姿。采集只读取反馈，不切换工作模式或控制模式；开始录制前，应确保
当前模式已发布三路末端反馈。任一路缺失或出错会报告录制失败。

回放推荐显式使用 `--server ADDRESS --mode joint_pos|end_pose`。Quanta X3 的
`end_pose` 回放要求录制文件包含全部三路末端位姿，缺失时在切模和下发指令前失败，
不会自动退回关节模式，也不会由关节角计算末端位姿。接口分别为双臂及腰部的
`set_end_pose` / `SetEndPose`；头部、夹爪仍使用各自的控制接口。

## 腰部控制

Python `waist_control.py` 和 C++ `quanta_x3_waist_control` 提供 `joint_pos` 和 `end_pose`
两种平滑相对运动，参数一致，默认只读。C++ 源文件为 `quanta_x3/waist_control.cpp`。
腰部末端模式用于演示 WBC 控制：发送轨迹并短时保持后结束，输出目标、位姿反馈和
实际腰部关节变化，不将多任务求解产生的末端偏差判为到位失败。关节模式仍检查到位。
末端模式支持 `--pose-delta` 平移或 `--pose-rotation pitch|yaw ANGLE_RAD` 单轴姿态旋转，
两类参数不能混用；旋转围绕反馈坐标系的固定轴，保持初始目标 XYZ。
配置、运行命令与安全边界见 [Python X3 说明](python/v2/quanta_x3/README.md)。
C++ 的运行命令见 [C++ X3 说明](cpp/quanta_x3/README.md)。
