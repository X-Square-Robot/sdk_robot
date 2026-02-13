# 示例

---

## camera.py

**功能说明:**

- 获取头部相机RGB图像和深度图数据
- 获取左右手臂相机的RGB图像数据

**使用方法:**

```bash
# 头部rgb图片
python3 camera.py head rgb-image --server 192.168.10.1:50051

# 头部相机深度图
python3 camera.py head depth-image --server 192.168.10.1:50051

# 头部流图片获取
python3 camera.py head rgb-stream --server 192.168.10.1:50051

# 头部深度流图片获取
python3 camera.py head depth-stream --server 192.168.10.1:50051

# 左臂单张rgb图片
python3 camera.py left-arm raw-image --server 192.168.10.1:50051

# 左臂rgb流图片获取：
python3 camera.py left-arm stream --server 192.168.10.1:50051
```

---

## chassis_control.py

**功能说明:**

- `move_to_global_position()`: 移动到全局位置
- `move_to_relative_position()`: 移动到相对位置
- `move_by_velocity()`: 速度控制
- `get_chassis_odometry()`: 获取里程计数据
- `move_by_map()`: 基于地图的导航
- `move_by_keyboard()`: 通过键盘控制，基于速度控制的封装

**使用方法:**

```bash
# 通过键盘控制
python3 chassis_control.py --server 192.168.10.1:50051 --control_mode keyboard
# 通过地图控制，会自动转圈建图
python3 chassis_control.py --server 192.168.10.1:50051 --control_mode map
```

---

## check_connect.py

**功能说明:**

- 检查和机器人的连通性

**使用方法:**

```bash
python check_connect.py  --server 192.168.10.1:50051
```

---

## data_collection/collection_config.py

传感器数据采集配置

定义所有可采集的传感器数据流及其配置选项

---

## data_collection/data_collector.py

gRPC 实时数据采集器 - 支持所有传感器，保存为通用JSON格式

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

这个脚本展示了如何使用DataCollector采集机器人数据，采集的数据保存在当前目录的collected_data目录下

**功能说明:**

- `create_collection_config_for_quanta_x1()`
- `create_collection_config_for_quanta_x2()`
- `create_collection_config_for_desktop()`

**使用方法:**

```bash
python3 data_collection_example.py  # server address, e.g., localhost:50051
```

---

## data_replay_example.py

数据回放示例

这个脚本展示了如何读取采集的数据并进行回放
支持两种回放模式：

1. 关节位置回放 (joint_position)
2. 末端位姿回放 (end_pose)

**功能说明:**

- `load_episode_data()`: 加载episode数据
- `filter_nan_values()`: 过滤掉NaN值，保留有效的关节位置
- `quaternion_to_yaw()`: 将四元数转换为yaw角（绕Z轴旋转）
- `replay_by_joint_positions()`: 按关节位置回放所有关节和底盘
- `replay_by_end_pose()`: 按末端位姿回放（包含底盘、夹爪、腰部/升降台、头部）

**使用方法:**

```bash
python3 data_replay_example.py ./collected_data/episode_0000/ --server 192.168.10.1:50051 --mode end_pose
```

---

## depth_points.py

深度点云获取示例

**功能说明:**

- `parse_point_cloud2()`: Parses a PointCloud2 message to extract the coordinates of all points.
- `display_depth_points()`: Displays the depth point cloud data.

**使用方法:**

```bash
#  单次读取
python3 depth_points.py --action single --server 192.168.10.1:50051
# 流读取
python3 depth_points.py --action stream --server 192.168.10.1:50051
```

---

---

## head_control.py

头部控制示例

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

IMU传感器数据获取示例

**功能说明:**

- `quaternion_to_euler()`: Converts a quaternion to Euler angles (roll, pitch, yaw) in radians.
- `display_imu_data()`: Displays simplified IMU sensor data.
- `read_and_display_imu()`: Reads and displays data from the IMU sensor.

**使用方法:**

```bash
# 获取单次数据
python3 imu.py --action single --server 192.168.10.1:50051
# 持续获取
python3 imu.py --action stream --server 192.168.10.1:50051
```

---

## navigation.py

导航接口使用示例

**使用方法:**

```bash
python3 navigation.py --server 192.168.10.1:50051
```

---

## quanta_x1/arm_control.py

量子1号手臂控制示例，其他机型请勿使用该示例

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

# 获取左臂的关节状态流
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# 获取右臂的末端位姿流
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## quanta_x1/gripper_control.py

量子1号夹爪控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_gripper()`: 控制夹爪位置
- `stream_gripper_data()`: 获取夹爪关节状态流

**使用方法:**

```bash
# 获取左边夹爪的关节状态
python3 gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# 控制左边夹爪
python3 gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---

## quanta_x1/lift_control.py

量子1号腰部使用示例，其他机型请勿使用该示例

**功能说明:**

- `move_by_lift_position()`: 控制操作
- `stream_lift_joint_states()`: 数据流

**使用方法:**

```bash
# 往上移动0.05m
python3 quanta_x1/lift_control.py  --server 192.168.10.1:50051 --action move --direction up
# 往下移动0.1m
python3 quanta_x1/lift_control.py  --server 192.168.10.1:50051 --action move --direction down --distance 0.1

# 获取关节状态流
python3 quanta_x1/lift_control.py  --server 192.168.10.1:50051 --action stream
```

---

## quanta_x2/arm_control.py

量子2号手臂控制示例，其他机型请勿使用该示例

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

# 获取左臂的关节状态流
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# 获取右臂的末端位姿流
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## quanta_x2/gripper_control.py

量子2号夹爪控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_gripper()`: 控制夹爪位置
- `stream_gripper_data()`: 获取夹爪关节位置流

**使用方法:**

```bash
# 获取左边夹爪的关节状态
python3 gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# 控制左边夹爪
python3 gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---

## quanta_x2/waist_control.py

量子2号腰部控制示例，其他机型请勿使用该示例

**功能说明:**

- `move_joints_toppra()`: 控制操作
- `stream_waist_data()`: 演示腰部控制器的流式接口 - 实时监听关节状态

**使用方法:**

```bash
# 腰部不支持末端位姿控制
python3 quanta_x2/waist_control.py  --server 192.168.10.1:50051 --mode joint_pos
```

---

## radar.py

雷达数据获取示例

**功能说明:**

- `display_scan_data()`

**使用方法:**

```bash
# 单次获取
python3 radar.py --server 192.168.10.1:50051 --action single
# 持续获取
python3 radar.py --server 192.168.10.1:50051 --action stream
```

---

## system.py

获取系统信息（版本，CPU占用，内存占用等等）示例

- robot.system

**使用方法:**

```bash
python3 system.py  --server 192.168.10.1:50051
```

---

## tof.py

红外传感器数据获取示例

**功能说明:**

- `display_tof_data()`
- `read_all_sensors()`

**使用方法:**

```bash
# 单次读取单个传感器
python3 tof.py --action sensor-1 --server 192.168.10.1:50051

# 单次读取全部传感器
python3 tof.py --action both --server 192.168.10.1:50051

# 流读取单个传感器
python3 tof.py --action sensor-1-stream --server 192.168.10.1:50051

# 流读取全部传感器
python3 tof.py --action both-stream --server 192.168.10.1:50051
```

---

## ultrasonic.py

超声波传感器数据获取示例

**功能说明:**

- `display_ultrasonic_data()`
- `read_all_sensors()`

**使用方法:**

```bash
# 单次读取单个传感器
python3 ultrasonic.py --action sensor-1 --server 192.168.10.1:50051

# 单次读取全部传感器
python3 ultrasonic.py --action all --server 192.168.10.1:50051

# 流读取单个传感器
python3 ultrasonic.py --action sensor-1-stream --server 192.168.10.1:50051

# 流读取全部传感器
python3 ultrasonic.py --action all-stream --server 192.168.10.1:50051
```

---
