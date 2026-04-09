# 示例

---

## camera.py

**功能说明:**

- 获取头部相机 RGB 图像和深度图数据
- 获取左右手臂相机的 RGB 图像数据

**使用方法:**

```bash
# 头部 RGB 图像
python3 camera.py head rgb-image --server 192.168.10.1:50051

# 头部相机深度图
python3 camera.py head depth-image --server 192.168.10.1:50051

# 头部 RGB 流
python3 camera.py head rgb-stream --server 192.168.10.1:50051

# 头部深度流
python3 camera.py head depth-stream --server 192.168.10.1:50051

# 左臂单张 RGB 图像
python3 camera.py left-arm raw-image --server 192.168.10.1:50051

# 左臂 RGB 流
python3 camera.py left-arm stream --server 192.168.10.1:50051
```

---

## chassis_control.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `move_to_global_position()`: 移动到全局位置
- `move_to_relative_position()`: 移动到相对位置
- `move_by_velocity()`: 速度控制
- `get_chassis_odometry()`: 获取里程计数据
- `move_by_map()`: 基于地图的导航
- `move_by_keyboard()`: 键盘控制，基于速度控制的封装

**使用方法:**

```bash
# 键盘控制
python3 chassis_control.py --server 192.168.10.1:50051 --control_mode keyboard
# 地图控制，会自动转圈建图
python3 chassis_control.py --server 192.168.10.1:50051 --control_mode map
```

---

## check_connect.py

**功能说明:**

- 检查与机器人的连通性

**使用方法:**

```bash
python check_connect.py --server 192.168.10.1:50051
```

---

## robot_control.py

**功能说明:**

- `emergency_stop()`: 紧急停止，在紧急情况下调用（请勿频繁调用）
- `recover_emergency_stop()`: 从紧急停止状态恢复
- `homing()`: 执行机器人回零操作

脚本会先将机器人工作模式设置为 SDK，再执行指定操作。

**使用方法:**

```bash
# 紧急停止
python3 robot_control.py --action stop --server 192.168.10.1:50051

# 从紧急停止恢复
python3 robot_control.py --action recover --server 192.168.10.1:50051

# 回零（默认操作）
python3 robot_control.py --action homing --server 192.168.10.1:50051
```

---

## data_collection/collection_config.py

传感器数据采集配置

定义所有可采集的传感器数据流及其配置选项

---

## data_collection/data_collector.py

gRPC 实时数据采集器 - 支持所有传感器，保存为通用 JSON 格式

使用方法:

```python
from x2robot import connect
from x2robot.action_data_collection import DataCollector
from x2robot.collection_config import CollectionConfigPresets

robot = connect("x2://192.168.10.1:50051")

# 使用预设配置
collector = DataCollector(
    robot,
    output_dir="./collected_data",
    target_hz=30,
    collection_config=CollectionConfigPresets.full_manipulation()
)

collector.start_recording(task="pick and place")
# ... 执行任务 ...
collector.stop_recording()
```

---

## data_collection_example.py

数据采集示例

该脚本演示如何使用 DataCollector 采集机器人数据，采集的数据保存在 ./collected_data 目录下

**功能说明:**

- `create_collection_config_for_quanta_x1()`
- `create_collection_config_for_quanta_x2()`
- `create_collection_config_for_desktop()`

**使用方法:**

```bash
python3 data_collection_example.py  # 服务器地址，如 localhost:50051
```

---

## data_replay_example.py

数据回放示例

该脚本演示如何读取采集的数据并进行回放
支持两种回放模式：

1. 关节位置回放 (joint_position)
2. 末端位姿回放 (end_pose)

**功能说明:**

- `load_episode_data()`: 加载 episode 数据
- `filter_nan_values()`: 过滤 NaN 值，保留有效关节位置
- `quaternion_to_yaw()`: 将四元数转换为 yaw 角（绕 Z 轴旋转）
- `replay_by_joint_positions()`: 按关节位置回放所有关节和底盘
- `replay_by_end_pose()`: 按末端位姿回放（包含底盘、夹爪、腰部/升降台、头部）

**使用方法:**

```bash
python3 data_replay_example.py ./collected_data/episode_0000/ --server 192.168.10.1:50051 --mode end_pose
```

---

## depth_points.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `parse_point_cloud2()`: 解析 PointCloud2 消息，提取所有点的坐标
- `display_depth_points()`: 显示深度点云数据

**使用方法:**

```bash
# 单次读取
python3 depth_points.py --action single --server 192.168.10.1:50051
# 流式读取
python3 depth_points.py --action stream --server 192.168.10.1:50051
```

---

## head_control.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `move_head()`: 控制操作
- `stream_head_joint_states()`: 数据流

**使用方法:**

```bash
# 获取头部关节电机状态数据流
python3 head_control.py --action stream --server 192.168.10.1:50051
# 控制头部移动
python3 head_control.py --action move --server 192.168.10.1:50051
```

---

## imu.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `quaternion_to_euler()`: 将四元数转换为欧拉角（roll、pitch、yaw，单位：弧度）
- `display_imu_data()`: 显示简化的 IMU 传感器数据
- `read_and_display_imu()`: 读取并显示 IMU 传感器数据

**使用方法:**

```bash
# 获取单次数据
python3 imu.py --action single --server 192.168.10.1:50051
# 持续流式获取
python3 imu.py --action stream --server 192.168.10.1:50051
```

---

## quanta_x1/arm_control.py

Quanta X1 机械臂控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_arm_joints_toppra()`: 控制操作
- `move_by_joint_positions()`: 控制操作
- `move_arm_endpose_toppra()`: 控制操作
- `move_by_end_pose()`: 控制操作
- `stream_arm_joint_states()`: 数据流

**使用方法:**

```bash
# 通过末端位姿模式控制左臂
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm left
# 通过关节角度控制右臂
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm right

# 获取左臂关节状态流
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# 获取右臂末端位姿流
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## quanta_x1/gripper_control.py

量子1号 夹爪控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_gripper()`: 控制夹爪位置
- `stream_gripper_data()`: 获取夹爪关节状态流

**使用方法:**

```bash
# 获取左夹爪关节状态
python3 quanta_x1/gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# 控制左夹爪
python3 quanta_x1/gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---

## quanta_x1/lift_control.py

量子1号 升降台控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_by_lift_position()`: 控制操作
- `stream_lift_joint_states()`: 数据流

**使用方法:**

```bash
# 上升 0.05m
python3 quanta_x1/lift_control.py --server 192.168.10.1:50051 --action move --direction up
# 下降 0.1m
python3 quanta_x1/lift_control.py --server 192.168.10.1:50051 --action move --direction down --distance 0.1

# 获取关节状态流
python3 quanta_x1/lift_control.py --server 192.168.10.1:50051 --action stream
```

---

## quanta_x2/arm_control.py

Quanta X2 机械臂控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_arm_joints_toppra()`: 控制操作
- `move_by_joint_positions()`: 控制操作
- `move_arm_endpose_toppra()`: 控制操作
- `move_by_end_pose()`: 控制操作
- `stream_arm_joint_states()`: 数据流

**使用方法:**

```bash
# 通过末端位姿模式控制左臂
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm left
# 通过关节角度控制右臂
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm right

# 获取左臂关节状态流
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# 获取右臂末端位姿流
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## quanta_x2/gripper_control.py

量子2号 夹爪控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_gripper()`: 控制夹爪位置
- `stream_gripper_data()`: 获取夹爪关节位置流

**使用方法:**

```bash
# 获取左夹爪关节状态
python3 quanta_x2/gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# 控制左夹爪
python3 quanta_x2/gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---

## quanta_x2/waist_control.py

Quanta X2 腰部控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_joints_toppra()`: 控制操作
- `stream_waist_data()`: 演示腰部控制器流式接口 - 实时关节状态监听

**使用方法:**

```bash
# 腰部不支持末端位姿控制
python3 quanta_x2/waist_control.py --server 192.168.10.1:50051 --mode joint_pos
```

---

## radar.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `display_scan_data()`

**使用方法:**

```bash
# 单次读取
python3 radar.py --server 192.168.10.1:50051 --action single
# 持续流式读取
python3 radar.py --server 192.168.10.1:50051 --action stream
```

---

## system.py

系统信息示例（版本、CPU 占用、内存占用等）

- robot.system

**使用方法:**

```bash
python3 system.py --server 192.168.10.1:50051
```

---

## tof.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `display_tof_data()`
- `read_all_sensors()`

**使用方法:**

```bash
# 单次读取单个传感器
python3 tof.py --action sensor-1 --server 192.168.10.1:50051

# 单次读取全部传感器
python3 tof.py --action both --server 192.168.10.1:50051

# 流式读取单个传感器
python3 tof.py --action sensor-1-stream --server 192.168.10.1:50051

# 流式读取全部传感器
python3 tof.py --action both-stream --server 192.168.10.1:50051
```

---

## ultrasonic.py

适用于 量子1号 和 量子2号，不支持 桌面主从 机型

**功能说明:**

- `display_ultrasonic_data()`
- `read_all_sensors()`

**使用方法:**

```bash
# 单次读取单个传感器
python3 ultrasonic.py --action sensor-1 --server 192.168.10.1:50051

# 单次读取全部传感器
python3 ultrasonic.py --action all --server 192.168.10.1:50051

# 流式读取单个传感器
python3 ultrasonic.py --action sensor-1-stream --server 192.168.10.1:50051

# 流式读取全部传感器
python3 ultrasonic.py --action all-stream --server 192.168.10.1:50051
```

---

## desktop/arm_control.py

桌面主从 机械臂控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_arm_joints_toppra()`: 控制操作
- `move_by_joint_positions()`: 控制操作
- `move_arm_endpose_toppra()`: 控制操作
- `move_by_end_pose()`: 控制操作
- `stream_arm_joint_states()`: 数据流

**使用方法:**

```bash
# 通过末端位姿模式控制左臂
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm left
# 通过关节角度控制右臂
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm right

# 获取左臂关节状态流
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# 获取右臂末端位姿流
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## desktop/gripper_control.py

桌面主从 夹爪控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_gripper()`: 控制夹爪位置
- `stream_gripper_data()`: 获取夹爪关节状态流

**使用方法:**

```bash
# 获取左夹爪关节状态
python3 desktop/gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# 控制左夹爪
python3 desktop/gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---
