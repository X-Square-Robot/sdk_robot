# Examples

---

## camera.py

**Functions:**

- Get head camera RGB images and depth map data
- Get left and right arm camera RGB image data

**Usage:**

```bash
# Head RGB image
python3 camera.py head rgb-image --server 192.168.10.1:50051

# Head camera depth map
python3 camera.py head depth-image --server 192.168.10.1:50051

# Head RGB stream
python3 camera.py head rgb-stream --server 192.168.10.1:50051

# Head depth stream
python3 camera.py head depth-stream --server 192.168.10.1:50051

# Left arm single RGB image
python3 camera.py left-arm raw-image --server 192.168.10.1:50051

# Left arm RGB stream
python3 camera.py left-arm stream --server 192.168.10.1:50051
```

---

## chassis_control.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `move_to_global_position()`: Move to global position
- `move_to_relative_position()`: Move to relative position
- `move_by_velocity()`: Velocity control
- `get_chassis_odometry()`: Get odometry data
- `move_by_map()`: Map-based navigation
- `move_by_keyboard()`: Keyboard control, wrapper based on velocity control

**Usage:**

```bash
# Keyboard control
python3 chassis_control.py --server 192.168.10.1:50051 --control_mode keyboard
# Map control, will automatically circle to build map
python3 chassis_control.py --server 192.168.10.1:50051 --control_mode map
```

---

## check_connect.py

**Functions:**

- Check connectivity with the robot

**Usage:**

```bash
python check_connect.py --server 192.168.10.1:50051
```

---

## robot_control.py

**Functions:**

- `emergency_stop()`: Emergency stop, call when emergency situation occurs (do not call frequently)
- `recover_emergency_stop()`: Recover from emergency stop state
- `homing()`: Execute robot homing operation

The script first sets the robot work mode to SDK, then executes the specified action.

**Usage:**

```bash
# Emergency stop
python3 robot_control.py --action stop --server 192.168.10.1:50051

# Recover from emergency stop
python3 robot_control.py --action recover --server 192.168.10.1:50051

# Homing (default action)
python3 robot_control.py --action homing --server 192.168.10.1:50051
```

---

## data_collection/collection_config.py

Sensor Data Collection Configuration

Defines all collectible sensor data streams and their configuration options

---

## data_collection/data_collector.py

gRPC Real-time Data Collector - Supports all sensors, saves in universal JSON format

Usage:

```python
from x2robot import connect
from x2robot.action_data_collection import DataCollector
from x2robot.collection_config import CollectionConfigPresets

robot = connect("x2://192.168.10.1:50051")

# Use preset configuration
collector = DataCollector(
    robot,
    output_dir="./collected_data",
    target_hz=30,
    collection_config=CollectionConfigPresets.full_manipulation()
)

collector.start_recording(task="pick and place")
# ... Execute task ...
collector.stop_recording()
```

---

## data_collection_example.py

Data Collection Example

This script demonstrates how to use DataCollector to collect robot data, colleted data is saved in ./collected_data directory.

**Functions:**

- `create_collection_config_for_quanta_x1()`
- `create_collection_config_for_quanta_x2()`
- `create_collection_config_for_desktop()`

**Usage:**

```bash
python3 data_collection_example.py  # server address, e.g., localhost:50051
```

---

## data_replay_example.py

Data Replay Example

This script demonstrates how to read collected data and replay it
Supports two replay modes:

1. Joint position replay (joint_position)
2. End pose replay (end_pose)

**Functions:**

- `load_episode_data()`: Load episode data
- `filter_nan_values()`: Filter out NaN values, keep valid joint positions
- `quaternion_to_yaw()`: Convert quaternion to yaw angle (rotation around Z-axis)
- `replay_by_joint_positions()`: Replay all joints and chassis by joint positions
- `replay_by_end_pose()`: Replay by end pose (includes chassis, gripper, waist/lift, head)

**Usage:**

```bash
python3 data_replay_example.py ./collected_data/episode_0000/ --server 192.168.10.1:50051 --mode end_pose
```

---

## depth_points.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `parse_point_cloud2()`: Parses a PointCloud2 message to extract the coordinates of all points.
- `display_depth_points()`: Displays the depth point cloud data.

**Usage:**

```bash
# Single read
python3 depth_points.py --action single --server 192.168.10.1:50051
# Stream read
python3 depth_points.py --action stream --server 192.168.10.1:50051
```

---

## head_control.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `move_head()`: Control operation
- `stream_head_joint_states()`: Data stream

**Usage:**

```bash
# Get head joint motor state data stream
python3 head_control.py --action stream --server 192.168.10.1:50051
# Control head movement
python3 head_control.py --action move --server 192.168.10.1:50051
```

---

## imu.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `quaternion_to_euler()`: Converts a quaternion to Euler angles (roll, pitch, yaw) in radians.
- `display_imu_data()`: Displays simplified IMU sensor data.
- `read_and_display_imu()`: Reads and displays data from the IMU sensor.

**Usage:**

```bash
# Get single data
python3 imu.py --action single --server 192.168.10.1:50051
# Continuous stream
python3 imu.py --action stream --server 192.168.10.1:50051
```

---

## quanta_x1/arm_control.py

Quanta X1 Arm Control Example, do not use this example for other models

**Functions:**

- `move_arm_joints_toppra()`: Control operation
- `move_by_joint_positions()`: Control operation
- `move_arm_endpose_toppra()`: Control operation
- `move_by_end_pose()`: Control operation
- `stream_arm_joint_states()`: Data stream

**Usage:**

```bash
# Control left arm by end pose mode
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm left
# Control right arm by joint angles
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm right

# Get left arm joint state stream
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# Get right arm end pose stream
python3 quanta_x1/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## quanta_x1/gripper_control.py

Quanta_X1 Gripper Control Example, Do not use this example in other model

**Functions:**

- `move_gripper()`: Control gripper position
- `stream_gripper_data()`: Get gripper joint state stream

**Usage:**

```bash
# Get left gripper joint states
python3 quanta_x1/gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# Control left gripper
python3 quanta_x1/gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---

## quanta_x1/lift_control.py

Quanta_X1 Lift Control Example, do not use this example for other models

**Functions:**

- `move_by_lift_position()`: Control operation
- `stream_lift_joint_states()`: Data stream

**Usage:**

```bash
# Move up 0.05m
python3 quanta_x1/lift_control.py --server 192.168.10.1:50051 --action move --direction up
# Move down 0.1m
python3 quanta_x1/lift_control.py --server 192.168.10.1:50051 --action move --direction down --distance 0.1

# Get joint state stream
python3 quanta_x1/lift_control.py --server 192.168.10.1:50051 --action stream
```

---

## quanta_x2/arm_control.py

Quanta X2 Arm Control Example, do not use this example for other models

**Functions:**

- `move_arm_joints_toppra()`: Control operation
- `move_by_joint_positions()`: Control operation
- `move_arm_endpose_toppra()`: Control operation
- `move_by_end_pose()`: Control operation
- `stream_arm_joint_states()`: Data stream

**Usage:**

```bash
# Control left arm by end pose mode
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm left
# Control right arm by joint angles
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm right

# Get left arm joint state stream
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# Get right arm end pose stream
python3 quanta_x2/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## quanta_x2/gripper_control.py

Quanta_X2 Gripper Control Example, Do not use this example in other model

**Functions:**

- `move_gripper()`: Control gripper position
- `stream_gripper_data()`: Get gripper joint position stream

**Usage:**

```bash
# Get left gripper joint states
python3 gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# Control left gripper
python3 gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---

## quanta_x2/waist_control.py

Quanta X2 Waist Control Example, do not use this example for other models

**Functions:**

- `move_joints_toppra()`: Control operation
- `stream_waist_data()`: Demonstrates waist controller streaming interface - real-time joint state monitoring

**Usage:**

```bash
# Waist does not support end pose control
python3 quanta_x2/waist_control.py --server 192.168.10.1:50051 --mode joint_pos
```

---

## radar.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `display_scan_data()`

**Usage:**

```bash
# Single read
python3 radar.py --server 192.168.10.1:50051 --action single
# Continuous stream
python3 radar.py --server 192.168.10.1:50051 --action stream
```

---

## system.py

System Information Example (version, CPU usage, memory usage, etc.)

- robot.system

**Usage:**

```bash
python3 system.py --server 192.168.10.1:50051
```

---

## tof.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `display_tof_data()`
- `read_all_sensors()`

**Usage:**

```bash
# Single read single sensor
python3 tof.py --action sensor-1 --server 192.168.10.1:50051

# Single read all sensors
python3 tof.py --action both --server 192.168.10.1:50051

# Stream read single sensor
python3 tof.py --action sensor-1-stream --server 192.168.10.1:50051

# Stream read all sensors
python3 tof.py --action both-stream --server 192.168.10.1:50051
```

---

## ultrasonic.py

For Quanta_X1 and Quanta_X2, Do not support DeskTop

**Functions:**

- `display_ultrasonic_data()`
- `read_all_sensors()`

**Usage:**

```bash
# Single read single sensor
python3 ultrasonic.py --action sensor-1 --server 192.168.10.1:50051

# Single read all sensors
python3 ultrasonic.py --action all --server 192.168.10.1:50051

# Stream read single sensor
python3 ultrasonic.py --action sensor-1-stream --server 192.168.10.1:50051

# Stream read all sensors
python3 ultrasonic.py --action all-stream --server 192.168.10.1:50051
```

---

## desktop/arm_control.py

Desktop Arm Control Example, do not use this example for other models

**Functions:**

- `move_arm_joints_toppra()`: Control operation
- `move_by_joint_positions()`: Control operation
- `move_arm_endpose_toppra()`: Control operation
- `move_by_end_pose()`: Control operation
- `stream_arm_joint_states()`: Data stream

**Usage:**

```bash
# Control left arm by end pose mode
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm left
# Control right arm by joint angles
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm right

# Get left arm joint state stream
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode joint_pos --arm left --action stream
# Get right arm end pose stream
python3 desktop/arm_control.py --server 192.168.10.1:50051 --mode end_pose --arm right --action stream
```

---

## desktop/gripper_control.py

Desktop Gripper Control Example, do not use this example for other models

**Functions:**

- `move_gripper()`: Control gripper position
- `stream_gripper_data()`: Get gripper joint state stream

**Usage:**

```bash
# Get left gripper joint states
python3 desktop/gripper_control.py --action stream --server 192.168.10.1:50051 --gripper left
# Control left gripper
python3 desktop/gripper_control.py --action move --server 192.168.10.1:50051 --gripper left
```

---
