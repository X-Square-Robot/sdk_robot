# API 文档 - Quanta_X2

## 目录

### 服务

* [ChassisController](#chassiscontroller)
   * [set_control_mode](#chassiscontroller-set_control_mode)
   * [get_control_mode](#chassiscontroller-get_control_mode)
   * [move_to_global_position](#chassiscontroller-move_to_global_position)
   * [move_to_relative_position](#chassiscontroller-move_to_relative_position)
   * [set_velocity](#chassiscontroller-set_velocity)
   * [set_virtual_zero_point](#chassiscontroller-set_virtual_zero_point)
   * [get_virtual_zero_point](#chassiscontroller-get_virtual_zero_point)
   * [get_global_position](#chassiscontroller-get_global_position)
   * [get_relative_position](#chassiscontroller-get_relative_position)
   * [get_odometry](#chassiscontroller-get_odometry)
   * [get_odometry_stream](#chassiscontroller-get_odometry_stream)
   * [get_pose_stream](#chassiscontroller-get_pose_stream)
   * [send_relative_pose_to_navigation](#chassiscontroller-send_relative_pose_to_navigation)
   * [reset_navigation_chunk_id](#chassiscontroller-reset_navigation_chunk_id)
   * [set_trajectory_coord_system_mode](#chassiscontroller-set_trajectory_coord_system_mode)
* [DepthPoints](#depthpoints)
   * [get_chassis_depth_points](#depthpoints-get_chassis_depth_points)
   * [get_chassis_depth_points_stream](#depthpoints-get_chassis_depth_points_stream)
* [HeadCamera](#headcamera)
   * [get_rgb_image](#headcamera-get_rgb_image)
   * [get_depth_image](#headcamera-get_depth_image)
   * [get_rgb_video_stream](#headcamera-get_rgb_video_stream)
   * [get_depth_video_stream](#headcamera-get_depth_video_stream)
* [HeadController](#headcontroller)
   * [set_pose](#headcontroller-set_pose)
   * [get_pose](#headcontroller-get_pose)
   * [reset](#headcontroller-reset)
   * [get_joint_states_stream](#headcontroller-get_joint_states_stream)
* [Imu](#imu)
   * [get_chassis_imu](#imu-get_chassis_imu)
   * [get_chassis_imu_stream](#imu-get_chassis_imu_stream)
* [LeftArmCamera](#leftarmcamera)
   * [get_raw_image](#leftarmcamera-get_raw_image)
   * [get_video_stream](#leftarmcamera-get_video_stream)
* [LeftArmController](#leftarmcontroller)
   * [set_joint_positions](#leftarmcontroller-set_joint_positions)
   * [set_end_pose](#leftarmcontroller-set_end_pose)
   * [get_joint_states](#leftarmcontroller-get_joint_states)
   * [get_end_pose](#leftarmcontroller-get_end_pose)
   * [reset](#leftarmcontroller-reset)
   * [get_wrench_ext_world](#leftarmcontroller-get_wrench_ext_world)
   * [get_wrench_ext_local](#leftarmcontroller-get_wrench_ext_local)
   * [get_joint_states_stream](#leftarmcontroller-get_joint_states_stream)
   * [get_end_pose_stream](#leftarmcontroller-get_end_pose_stream)
* [LeftGripperController](#leftgrippercontroller)
   * [set_position](#leftgrippercontroller-set_position)
   * [get_position](#leftgrippercontroller-get_position)
   * [get_position_stream](#leftgrippercontroller-get_position_stream)
* [LeftGripperTactile](#leftgrippertactile)
   * [get_tactile_sensor_data](#leftgrippertactile-get_tactile_sensor_data)
   * [get_tactile_sensor_data_stream](#leftgrippertactile-get_tactile_sensor_data_stream)
* [Navigation](#navigation)
   * [start_mapping](#navigation-start_mapping)
   * [stop_mapping](#navigation-stop_mapping)
   * [set_navigation_mode](#navigation-set_navigation_mode)
   * [start_localization](#navigation-start_localization)
   * [stop_localization](#navigation-stop_localization)
* [RadarService](#radarservice)
   * [get_laser_scan](#radarservice-get_laser_scan)
   * [get_laser_scan_stream](#radarservice-get_laser_scan_stream)
* [RightArmCamera](#rightarmcamera)
   * [get_raw_image](#rightarmcamera-get_raw_image)
   * [get_video_stream](#rightarmcamera-get_video_stream)
* [RightArmController](#rightarmcontroller)
   * [set_joint_positions](#rightarmcontroller-set_joint_positions)
   * [set_end_pose](#rightarmcontroller-set_end_pose)
   * [get_joint_states](#rightarmcontroller-get_joint_states)
   * [get_end_pose](#rightarmcontroller-get_end_pose)
   * [reset](#rightarmcontroller-reset)
   * [get_wrench_ext_world](#rightarmcontroller-get_wrench_ext_world)
   * [get_wrench_ext_local](#rightarmcontroller-get_wrench_ext_local)
   * [get_joint_states_stream](#rightarmcontroller-get_joint_states_stream)
   * [get_end_pose_stream](#rightarmcontroller-get_end_pose_stream)
* [RightGripperController](#rightgrippercontroller)
   * [set_position](#rightgrippercontroller-set_position)
   * [get_position](#rightgrippercontroller-get_position)
   * [get_position_stream](#rightgrippercontroller-get_position_stream)
* [RightGripperTactile](#rightgrippertactile)
   * [get_tactile_sensor_data](#rightgrippertactile-get_tactile_sensor_data)
   * [get_tactile_sensor_data_stream](#rightgrippertactile-get_tactile_sensor_data_stream)
* [RobotControl](#robotcontrol)
   * [set_manipulator_control_mode](#robotcontrol-set_manipulator_control_mode)
   * [get_manipulator_control_mode](#robotcontrol-get_manipulator_control_mode)
   * [homing](#robotcontrol-homing)
   * [emergency_stop](#robotcontrol-emergency_stop)
   * [recover_emergency_stop](#robotcontrol-recover_emergency_stop)
* [System](#system)
   * [set_work_mode](#system-set_work_mode)
   * [get_static_info](#system-get_static_info)
   * [get_dynamic_info](#system-get_dynamic_info)
* [Tof](#tof)
   * [get_chassis_tof1](#tof-get_chassis_tof1)
   * [get_chassis_tof2](#tof-get_chassis_tof2)
   * [get_chassis_tof1_stream](#tof-get_chassis_tof1_stream)
   * [get_chassis_tof2_stream](#tof-get_chassis_tof2_stream)
* [Ultrasonic](#ultrasonic)
   * [get_chassis_ultrasonic1](#ultrasonic-get_chassis_ultrasonic1)
   * [get_chassis_ultrasonic2](#ultrasonic-get_chassis_ultrasonic2)
   * [get_chassis_ultrasonic3](#ultrasonic-get_chassis_ultrasonic3)
   * [get_chassis_ultrasonic4](#ultrasonic-get_chassis_ultrasonic4)
   * [get_chassis_ultrasonic1_stream](#ultrasonic-get_chassis_ultrasonic1_stream)
   * [get_chassis_ultrasonic2_stream](#ultrasonic-get_chassis_ultrasonic2_stream)
   * [get_chassis_ultrasonic3_stream](#ultrasonic-get_chassis_ultrasonic3_stream)
   * [get_chassis_ultrasonic4_stream](#ultrasonic-get_chassis_ultrasonic4_stream)
* [WaistController](#waistcontroller)
   * [set_joint_positions](#waistcontroller-set_joint_positions)
   * [get_joint_states](#waistcontroller-get_joint_states)
   * [get_joint_states_stream](#waistcontroller-get_joint_states_stream)
   * [get_end_pose](#waistcontroller-get_end_pose)
   * [get_end_pose_stream](#waistcontroller-get_end_pose_stream)

### 消息类型列表

* [Time](#message-builtin_interfacestime)
* [Point](#message-geometry_msgspoint)
* [Pose](#message-geometry_msgspose)
* [PoseStamped](#message-geometry_msgsposestamped)
* [PoseWithCovariance](#message-geometry_msgsposewithcovariance)
* [Quaternion](#message-geometry_msgsquaternion)
* [Twist](#message-geometry_msgstwist)
* [TwistWithCovariance](#message-geometry_msgstwistwithcovariance)
* [Vector3](#message-geometry_msgsvector3)
* [HeadPanTiltControl](#message-halheadpantiltcontrol)
* [Odometry](#message-nav_msgsodometry)
* [CompressedImage](#message-sensor_msgscompressedimage)
* [Imu](#message-sensor_msgsimu)
* [JointState](#message-sensor_msgsjointstate)
* [LaserScan](#message-sensor_msgslaserscan)
* [PointCloud2](#message-sensor_msgspointcloud2)
* [PointField](#message-sensor_msgspointfield)
* [Range](#message-sensor_msgsrange)
* [Float32](#message-std_msgsfloat32)
* [Float64MultiArray](#message-std_msgsfloat64multiarray)
* [Header](#message-std_msgsheader)
* [MultiArrayDimension](#message-std_msgsmultiarraydimension)
* [MultiArrayLayout](#message-std_msgsmultiarraylayout)
* [String](#message-std_msgsstring)
* [ChassisControlModeParam](#message-xrsdkchassiscontrolmodeparam)
* [ChassisPosition](#message-xrsdkchassisposition)
* [ChassisPositionList](#message-xrsdkchassispositionlist)
* [ChassisVelocity](#message-xrsdkchassisvelocity)
* [CoordinateSystemModeParam](#message-xrsdkcoordinatesystemmodeparam)
* [ExecutionResult](#message-xrsdkexecutionresult)
* [GripperPosition](#message-xrsdkgripperposition)
* [HeadPose](#message-xrsdkheadpose)
* [JointPositions](#message-xrsdkjointpositions)
* [LiftPosition](#message-xrsdkliftposition)
* [ManipulatorControlModeParam](#message-xrsdkmanipulatorcontrolmodeparam)
* [NavigationModeParam](#message-xrsdknavigationmodeparam)
* [PingRequest](#message-xrsdkpingrequest)
* [PongResponse](#message-xrsdkpongresponse)
* [PowerStatus](#message-xrsdkpowerstatus)
* [RobotDynamicInfo](#message-xrsdkrobotdynamicinfo)
* [RobotModeParam](#message-xrsdkrobotmodeparam)
* [RobotRuntimeInfo](#message-xrsdkrobotruntimeinfo)
* [RobotStaticInfo](#message-xrsdkrobotstaticinfo)
* [SaveMapParam](#message-xrsdksavemapparam)
* [TactileSensorData](#message-xrsdktactilesensordata)

### 枚举类型列表

* [PointFieldConstants](#enum-sensor_msgspointfieldconstants)
* [ChassisControlMode](#enum-xrsdkchassiscontrolmode)
* [CoordinateSystemMode](#enum-xrsdkcoordinatesystemmode)
* [ManipulatorControlMode](#enum-xrsdkmanipulatorcontrolmode)
* [NavigationMode](#enum-xrsdknavigationmode)
* [RobotModelType](#enum-xrsdkrobotmodeltype)
* [RobotWorkMode](#enum-xrsdkrobotworkmode)

---

## API 服务

<h3 id="chassiscontroller">ChassisController</h3>

<h4 id="chassiscontroller-set_control_mode">set_control_mode</h4>

```python
def set_control_mode(chassis_control_mode_param: ChassisControlModeParam, timeout) -> ExecutionResult
```

设置控制模式：全局位置、相对位置或速度控制

**参数:**

* `chassis_control_mode_param` ([`ChassisControlModeParam`](#message-xrsdkchassiscontrolmodeparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-get_control_mode">get_control_mode</h4>

```python
def get_control_mode(timeout) -> ChassisControlModeParam
```

获取当前控制模式

**参数:**

* 无参数

**返回:**

* [`ChassisControlModeParam`](#message-xrsdkchassiscontrolmodeparam)

---

<h4 id="chassiscontroller-move_to_global_position">move_to_global_position</h4>

```python
def move_to_global_position(chassis_position: ChassisPosition, timeout) -> ExecutionResult
```

移动到全局位置（必须先设置GLOBAL模式）

**参数:**

* `chassis_position` ([`ChassisPosition`](#message-xrsdkchassisposition))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-move_to_relative_position">move_to_relative_position</h4>

```python
def move_to_relative_position(chassis_position: ChassisPosition, timeout) -> ExecutionResult
```

移动到相对位置（必须先设置RELATIVE模式和虚拟零点）

**参数:**

* `chassis_position` ([`ChassisPosition`](#message-xrsdkchassisposition))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-set_velocity">set_velocity</h4>

```python
def set_velocity(chassis_velocity: ChassisVelocity, timeout) -> ExecutionResult
```

设置速度控制（必须先设置VELOCITY模式）

**参数:**

* `chassis_velocity` ([`ChassisVelocity`](#message-xrsdkchassisvelocity))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-set_virtual_zero_point">set_virtual_zero_point</h4>

```python
def set_virtual_zero_point(chassis_position: ChassisPosition, timeout) -> ExecutionResult
```

设置虚拟零点（相对运动的原点）

**参数:**

* `chassis_position` ([`ChassisPosition`](#message-xrsdkchassisposition))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-get_virtual_zero_point">get_virtual_zero_point</h4>

```python
def get_virtual_zero_point(timeout) -> ChassisPosition
```

获取当前虚拟零点设置

**参数:**

* 无参数

**返回:**

* [`ChassisPosition`](#message-xrsdkchassisposition)

---

<h4 id="chassiscontroller-get_global_position">get_global_position</h4>

```python
def get_global_position(timeout) -> ChassisPosition
```

获取当前全局位置

**参数:**

* 无参数

**返回:**

* [`ChassisPosition`](#message-xrsdkchassisposition)

---

<h4 id="chassiscontroller-get_relative_position">get_relative_position</h4>

```python
def get_relative_position(timeout) -> ChassisPosition
```

获取当前相对位置（相对于虚拟零点）

**参数:**

* 无参数

**返回:**

* [`ChassisPosition`](#message-xrsdkchassisposition)

---

<h4 id="chassiscontroller-get_odometry">get_odometry</h4>

```python
def get_odometry(timeout) -> _nav_msgs__.Odometry
```

获取当前里程计

**参数:**

* 无参数

**返回:**

* [`Odometry`](#message-nav_msgsodometry)
   * `header` ([`Header`](#message-std_msgsheader))
   * `child_frame_id` (`string`)
   * `pose` ([`PoseWithCovariance`](#message-geometry_msgsposewithcovariance))
   * `twist` ([`TwistWithCovariance`](#message-geometry_msgstwistwithcovariance))

---

<h4 id="chassiscontroller-get_odometry_stream">get_odometry_stream</h4>

```python
def get_odometry_stream(timeout) -> Iterator[_nav_msgs__.Odometry]
```

获取里程计流

**参数:**

* 无参数

**返回:**

* [`Odometry`](#message-nav_msgsodometry)
   * `header` ([`Header`](#message-std_msgsheader))
   * `child_frame_id` (`string`)
   * `pose` ([`PoseWithCovariance`](#message-geometry_msgsposewithcovariance))
   * `twist` ([`TwistWithCovariance`](#message-geometry_msgstwistwithcovariance))

---

<h4 id="chassiscontroller-get_pose_stream">get_pose_stream</h4>

```python
def get_pose_stream(timeout) -> Iterator[_geometry_msgs__.PoseStamped]
```

获取位姿流

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="chassiscontroller-send_relative_pose_to_navigation">send_relative_pose_to_navigation</h4>

```python
def send_relative_pose_to_navigation(chassis_position_list: ChassisPositionList, timeout) -> ExecutionResult
```

向导航系统发送多个相对位置，用于数据回放

**参数:**

* `chassis_position_list` ([`ChassisPositionList`](#message-xrsdkchassispositionlist))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-reset_navigation_chunk_id">reset_navigation_chunk_id</h4>

```python
def reset_navigation_chunk_id(timeout) -> ExecutionResult
```

将导航chunk_id重置为0

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-set_trajectory_coord_system_mode">set_trajectory_coord_system_mode</h4>

```python
def set_trajectory_coord_system_mode(coordinate_system_mode_param: CoordinateSystemModeParam, timeout) -> ExecutionResult
```

设置轨迹坐标系模式，默认为地图坐标系，可设置为里程计坐标系

**参数:**

* `coordinate_system_mode_param` ([`CoordinateSystemModeParam`](#message-xrsdkcoordinatesystemmodeparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h3 id="depthpoints">DepthPoints</h3>

深度点云服务

<h4 id="depthpoints-get_chassis_depth_points">get_chassis_depth_points</h4>

```python
def get_chassis_depth_points(timeout) -> _sensor_msgs__.PointCloud2
```

获取底盘深度点云数据

**参数:**

* 无参数

**返回:**

* [`PointCloud2`](#message-sensor_msgspointcloud2)
   * `header` ([`Header`](#message-std_msgsheader))
   * `height` (`uint32`)
   * `width` (`uint32`)
   * `fields` (List[[`PointField`](#message-sensor_msgspointfield)])
   * `is_bigendian` (`bool`)
   * `point_step` (`uint32`)
   * `row_step` (`uint32`)
   * `data` (List[`uint32`])
   * `is_dense` (`bool`)

---

<h4 id="depthpoints-get_chassis_depth_points_stream">get_chassis_depth_points_stream</h4>

```python
def get_chassis_depth_points_stream(timeout) -> Iterator[_sensor_msgs__.PointCloud2]
```

**参数:**

* 无参数

**返回:**

* [`PointCloud2`](#message-sensor_msgspointcloud2)
   * `header` ([`Header`](#message-std_msgsheader))
   * `height` (`uint32`)
   * `width` (`uint32`)
   * `fields` (List[[`PointField`](#message-sensor_msgspointfield)])
   * `is_bigendian` (`bool`)
   * `point_step` (`uint32`)
   * `row_step` (`uint32`)
   * `data` (List[`uint32`])
   * `is_dense` (`bool`)

---

<h3 id="headcamera">HeadCamera</h3>

<h4 id="headcamera-get_rgb_image">get_rgb_image</h4>

```python
def get_rgb_image(timeout) -> _sensor_msgs__.CompressedImage
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="headcamera-get_depth_image">get_depth_image</h4>

```python
def get_depth_image(timeout) -> _sensor_msgs__.CompressedImage
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="headcamera-get_rgb_video_stream">get_rgb_video_stream</h4>

```python
def get_rgb_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="headcamera-get_depth_video_stream">get_depth_video_stream</h4>

```python
def get_depth_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h3 id="headcontroller">HeadController</h3>

<h4 id="headcontroller-set_pose">set_pose</h4>

```python
def set_pose(head_pose: HeadPose, timeout) -> ExecutionResult
```

控制头部位姿（偏航角和俯仰角）

适用于量子2号机型

* `上限: [0.87, 1.57]`
* `下限: [-0.52, -1.57]`

适用于量子1号机型

* `上限: [0.9, 1.20]`
* `下限: [-0.06, -1.20]`

**参数:**

* `head_pose` ([`HeadPose`](#message-xrsdkheadpose))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="headcontroller-get_pose">get_pose</h4>

```python
def get_pose(timeout) -> HeadPose
```

获取当前头部位姿

**参数:**

* 无参数

**返回:**

* [`HeadPose`](#message-xrsdkheadpose)

---

<h4 id="headcontroller-reset">reset</h4>

```python
def reset(timeout) -> ExecutionResult
```

重置头部到中心位置

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="headcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

获取关节状态流

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="imu">Imu</h3>

IMU服务

<h4 id="imu-get_chassis_imu">get_chassis_imu</h4>

```python
def get_chassis_imu(timeout) -> _sensor_msgs__.Imu
```

获取IMU数据

**参数:**

* 无参数

**返回:**

* [`Imu`](#message-sensor_msgsimu)
   * `header` ([`Header`](#message-std_msgsheader))
   * `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))
   * `orientation_covariance` (List[`double`])
   * `angular_velocity` ([`Vector3`](#message-geometry_msgsvector3))
   * `angular_velocity_covariance` (List[`double`])
   * `linear_acceleration` ([`Vector3`](#message-geometry_msgsvector3))
   * `linear_acceleration_covariance` (List[`double`])

---

<h4 id="imu-get_chassis_imu_stream">get_chassis_imu_stream</h4>

```python
def get_chassis_imu_stream(timeout) -> Iterator[_sensor_msgs__.Imu]
```

**参数:**

* 无参数

**返回:**

* [`Imu`](#message-sensor_msgsimu)
   * `header` ([`Header`](#message-std_msgsheader))
   * `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))
   * `orientation_covariance` (List[`double`])
   * `angular_velocity` ([`Vector3`](#message-geometry_msgsvector3))
   * `angular_velocity_covariance` (List[`double`])
   * `linear_acceleration` ([`Vector3`](#message-geometry_msgsvector3))
   * `linear_acceleration_covariance` (List[`double`])

---

<h3 id="leftarmcamera">LeftArmCamera</h3>

<h4 id="leftarmcamera-get_raw_image">get_raw_image</h4>

```python
def get_raw_image(timeout) -> _sensor_msgs__.CompressedImage
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="leftarmcamera-get_video_stream">get_video_stream</h4>

```python
def get_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h3 id="leftarmcontroller">LeftArmController</h3>

左臂控制器服务

<h4 id="leftarmcontroller-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

控制关节角度（必须先设置JOINT_POSITIONS模式）

适用于量子2号机型

* `上限: [3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708]`
* `下限: [-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708]`

适用于量子1号和桌面主从机型

* `上限: [2.792, 3.44, 3.14, 1.57, 1.4, 1.745]`
* `下限: [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]`

**参数:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftarmcontroller-set_end_pose">set_end_pose</h4>

```python
def set_end_pose(_geometry_msgs__: _geometry_msgs__.Pose, timeout) -> ExecutionResult
```

控制末端执行器位姿（必须先设置END_POSE模式）

适用于量子2号机型

* `上限: [5.0, 5.0, 5.0, 3.14, 3.14, 3.14, 3.14]`
* `下限: [-5.0, -5.0, -5.0, -3.14, -3.14, -3.14, -3.14]`

适用于量子1号和桌面主从机型

* `上限: [5.0, 5.0, 5.0, 3.14, 3.14, 3.14, 3.14]`
* `下限: [-5.0, -5.0, -5.0, -3.14, -3.14, -3.14, -3.14]`

**参数:**

* `position` ([`Point`](#message-geometry_msgspoint))
* `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftarmcontroller-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

获取关节状态（位置、速度、力矩）

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="leftarmcontroller-get_end_pose">get_end_pose</h4>

```python
def get_end_pose(timeout) -> _geometry_msgs__.PoseStamped
```

获取末端执行器位姿

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="leftarmcontroller-reset">reset</h4>

```python
def reset(timeout) -> ExecutionResult
```

重置机械臂到初始位置

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftarmcontroller-get_wrench_ext_world">get_wrench_ext_world</h4>

```python
def get_wrench_ext_world(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext world

**参数:**

* 无参数

**返回:**

* `WrenchStamped`
   * `header` ([`Header`](#message-std_msgsheader))
   * `wrench` (`Wrench`)

---

<h4 id="leftarmcontroller-get_wrench_ext_local">get_wrench_ext_local</h4>

```python
def get_wrench_ext_local(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext local

**参数:**

* 无参数

**返回:**

* `WrenchStamped`
   * `header` ([`Header`](#message-std_msgsheader))
   * `wrench` (`Wrench`)

---

<h4 id="leftarmcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

获取关节状态流

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="leftarmcontroller-get_end_pose_stream">get_end_pose_stream</h4>

```python
def get_end_pose_stream(timeout) -> Iterator[_geometry_msgs__.PoseStamped]
```

获取末端位姿流

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h3 id="leftgrippercontroller">LeftGripperController</h3>

<h4 id="leftgrippercontroller-set_position">set_position</h4>

```python
def set_position(gripper_position: GripperPosition, timeout) -> ExecutionResult
```

控制夹爪开合程度

适用于量子2号机型

* `上限: 25.2`
* `下限: 0.0`

适用于量子1号机型

* `上限: 4.5`
* `下限: 0.0`

**参数:**

* `gripper_position` ([`GripperPosition`](#message-xrsdkgripperposition))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftgrippercontroller-get_position">get_position</h4>

```python
def get_position(timeout) -> GripperPosition
```

获取当前夹爪状态

**参数:**

* 无参数

**返回:**

* [`GripperPosition`](#message-xrsdkgripperposition)

---

<h4 id="leftgrippercontroller-get_position_stream">get_position_stream</h4>

```python
def get_position_stream(timeout) -> Iterator[GripperPosition]
```

Get position stream

**参数:**

* 无参数

**返回:**

* `Iterator[[`GripperPosition`](#message-xrsdkgripperposition)]`: GripperPosition 流

---

<h3 id="leftgrippertactile">LeftGripperTactile</h3>

<h4 id="leftgrippertactile-get_tactile_sensor_data">get_tactile_sensor_data</h4>

```python
def get_tactile_sensor_data(timeout) -> TactileSensorData
```

**参数:**

* 无参数

**返回:**

* [`TactileSensorData`](#message-xrsdktactilesensordata)

---

<h4 id="leftgrippertactile-get_tactile_sensor_data_stream">get_tactile_sensor_data_stream</h4>

```python
def get_tactile_sensor_data_stream(timeout) -> Iterator[TactileSensorData]
```

**参数:**

* 无参数

**返回:**

* `Iterator[[`TactileSensorData`](#message-xrsdktactilesensordata)]`: TactileSensorData 流

---

<h3 id="navigation">Navigation</h3>

<h4 id="navigation-start_mapping">start_mapping</h4>

```python
def start_mapping(timeout) -> ExecutionResult
```

开始建图

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-stop_mapping">stop_mapping</h4>

```python
def stop_mapping(save_map_param: SaveMapParam, timeout) -> ExecutionResult
```

停止并保存建图

**参数:**

* `save_map_param` ([`SaveMapParam`](#message-xrsdksavemapparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-set_navigation_mode">set_navigation_mode</h4>

```python
def set_navigation_mode(navigation_mode_param: NavigationModeParam, timeout) -> ExecutionResult
```

设置导航模式（启用/禁用内置导航算法）

**参数:**

* `navigation_mode_param` ([`NavigationModeParam`](#message-xrsdknavigationmodeparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-start_localization">start_localization</h4>

```python
def start_localization(save_map_param: SaveMapParam, timeout) -> ExecutionResult
```

开始定位

**参数:**

* `save_map_param` ([`SaveMapParam`](#message-xrsdksavemapparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-stop_localization">stop_localization</h4>

```python
def stop_localization(timeout) -> ExecutionResult
```

停止定位

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h3 id="radarservice">RadarService</h3>

雷达服务

<h4 id="radarservice-get_laser_scan">get_laser_scan</h4>

```python
def get_laser_scan(timeout) -> _sensor_msgs__.LaserScan
```

获取雷达扫描数据

**参数:**

* 无参数

**返回:**

* [`LaserScan`](#message-sensor_msgslaserscan)
   * `header` ([`Header`](#message-std_msgsheader))
   * `angle_min` (`float`)
   * `angle_max` (`float`)
   * `angle_increment` (`float`)
   * `time_increment` (`float`)
   * `scan_time` (`float`)
   * `range_min` (`float`)
   * `range_max` (`float`)
   * `ranges` (List[`float`])
   * `intensities` (List[`float`])

---

<h4 id="radarservice-get_laser_scan_stream">get_laser_scan_stream</h4>

```python
def get_laser_scan_stream(timeout) -> Iterator[_sensor_msgs__.LaserScan]
```

**参数:**

* 无参数

**返回:**

* [`LaserScan`](#message-sensor_msgslaserscan)
   * `header` ([`Header`](#message-std_msgsheader))
   * `angle_min` (`float`)
   * `angle_max` (`float`)
   * `angle_increment` (`float`)
   * `time_increment` (`float`)
   * `scan_time` (`float`)
   * `range_min` (`float`)
   * `range_max` (`float`)
   * `ranges` (List[`float`])
   * `intensities` (List[`float`])

---

<h3 id="rightarmcamera">RightArmCamera</h3>

<h4 id="rightarmcamera-get_raw_image">get_raw_image</h4>

```python
def get_raw_image(timeout) -> _sensor_msgs__.CompressedImage
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="rightarmcamera-get_video_stream">get_video_stream</h4>

```python
def get_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**参数:**

* 无参数

**返回:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h3 id="rightarmcontroller">RightArmController</h3>

右臂控制器服务

<h4 id="rightarmcontroller-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

控制关节角度（必须先设置JOINT_POSITIONS模式）

适用于量子2号机型

* `上限: [3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708]`
* `下限: [-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708]`

适用于量子1号和桌面主从机型

* `上限: [2.792, 3.44, 3.14, 1.57, 1.4, 1.745]`
* `下限: [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]`

**参数:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightarmcontroller-set_end_pose">set_end_pose</h4>

```python
def set_end_pose(_geometry_msgs__: _geometry_msgs__.Pose, timeout) -> ExecutionResult
```

控制末端执行器位姿（必须先设置END_POSE模式）

支持位置和姿态控制

**参数:**

* `position` ([`Point`](#message-geometry_msgspoint))
* `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightarmcontroller-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

获取关节状态（位置、速度、力矩）

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="rightarmcontroller-get_end_pose">get_end_pose</h4>

```python
def get_end_pose(timeout) -> _geometry_msgs__.PoseStamped
```

获取末端执行器位姿

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="rightarmcontroller-reset">reset</h4>

```python
def reset(timeout) -> ExecutionResult
```

重置机械臂到初始位置

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightarmcontroller-get_wrench_ext_world">get_wrench_ext_world</h4>

```python
def get_wrench_ext_world(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext world

**参数:**

* 无参数

**返回:**

* `WrenchStamped`
   * `header` ([`Header`](#message-std_msgsheader))
   * `wrench` (`Wrench`)

---

<h4 id="rightarmcontroller-get_wrench_ext_local">get_wrench_ext_local</h4>

```python
def get_wrench_ext_local(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext local

**参数:**

* 无参数

**返回:**

* `WrenchStamped`
   * `header` ([`Header`](#message-std_msgsheader))
   * `wrench` (`Wrench`)

---

<h4 id="rightarmcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

获取关节状态流

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="rightarmcontroller-get_end_pose_stream">get_end_pose_stream</h4>

```python
def get_end_pose_stream(timeout) -> Iterator[_geometry_msgs__.PoseStamped]
```

获取末端位姿流

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h3 id="rightgrippercontroller">RightGripperController</h3>

<h4 id="rightgrippercontroller-set_position">set_position</h4>

```python
def set_position(gripper_position: GripperPosition, timeout) -> ExecutionResult
```

控制夹爪开合程度

适用于量子2号机型

* `上限: 25.2`
* `下限: 0.0`

适用于量子1号机型

* `上限: 4.5`
* `下限: 0.0`

**参数:**

* `gripper_position` ([`GripperPosition`](#message-xrsdkgripperposition))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightgrippercontroller-get_position">get_position</h4>

```python
def get_position(timeout) -> GripperPosition
```

获取当前夹爪状态

**参数:**

* 无参数

**返回:**

* [`GripperPosition`](#message-xrsdkgripperposition)

---

<h4 id="rightgrippercontroller-get_position_stream">get_position_stream</h4>

```python
def get_position_stream(timeout) -> Iterator[GripperPosition]
```

Get position stream

**参数:**

* 无参数

**返回:**

* `Iterator[[`GripperPosition`](#message-xrsdkgripperposition)]`: GripperPosition 流

---

<h3 id="rightgrippertactile">RightGripperTactile</h3>

<h4 id="rightgrippertactile-get_tactile_sensor_data">get_tactile_sensor_data</h4>

```python
def get_tactile_sensor_data(timeout) -> TactileSensorData
```

**参数:**

* 无参数

**返回:**

* [`TactileSensorData`](#message-xrsdktactilesensordata)

---

<h4 id="rightgrippertactile-get_tactile_sensor_data_stream">get_tactile_sensor_data_stream</h4>

```python
def get_tactile_sensor_data_stream(timeout) -> Iterator[TactileSensorData]
```

**参数:**

* 无参数

**返回:**

* `Iterator[[`TactileSensorData`](#message-xrsdktactilesensordata)]`: TactileSensorData 流

---

<h3 id="robotcontrol">RobotControl</h3>

<h4 id="robotcontrol-set_manipulator_control_mode">set_manipulator_control_mode</h4>

```python
def set_manipulator_control_mode(manipulator_control_mode_param: ManipulatorControlModeParam, timeout) -> ExecutionResult
```

设置机械臂（手臂和腰部）的控制模式：关节位置控制或末端位姿控制

**参数:**

* `manipulator_control_mode_param` ([`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="robotcontrol-get_manipulator_control_mode">get_manipulator_control_mode</h4>

```python
def get_manipulator_control_mode(timeout) -> ManipulatorControlModeParam
```

获取机械臂的当前控制模式

**参数:**

* 无参数

**返回:**

* [`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam)

---

<h4 id="robotcontrol-homing">homing</h4>

```python
def homing(timeout) -> ExecutionResult
```

机器人归位，所有关节将归位到初始位置

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="robotcontrol-emergency_stop">emergency_stop</h4>

```python
def emergency_stop(timeout) -> ExecutionResult
```

紧急停止，请谨慎调用，仅在必要时使用

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="robotcontrol-recover_emergency_stop">recover_emergency_stop</h4>

```python
def recover_emergency_stop(timeout) -> ExecutionResult
```

从紧急停止中恢复，仅在紧急停止被调用时使用

**参数:**

* 无参数

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h3 id="system">System</h3>

<h4 id="system-set_work_mode">set_work_mode</h4>

```python
def set_work_mode(robot_mode_param: RobotModeParam, timeout) -> ExecutionResult
```

设置机器人工作模式：IDLE, INFERE, COLLECT, SDK, 目前仅支持SDK模式。

**参数:**

* `robot_mode_param` ([`RobotModeParam`](#message-xrsdkrobotmodeparam))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="system-get_static_info">get_static_info</h4>

```python
def get_static_info(timeout) -> RobotStaticInfo
```

获取机器人系统信息

**参数:**

* 无参数

**返回:**

* [`RobotStaticInfo`](#message-xrsdkrobotstaticinfo)

---

<h4 id="system-get_dynamic_info">get_dynamic_info</h4>

```python
def get_dynamic_info(timeout) -> RobotDynamicInfo
```

获取机器人运行时信息

**参数:**

* 无参数

**返回:**

* [`RobotDynamicInfo`](#message-xrsdkrobotdynamicinfo)

---

<h3 id="tof">Tof</h3>

<h4 id="tof-get_chassis_tof1">get_chassis_tof1</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="tof-get_chassis_tof2">get_chassis_tof2</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="tof-get_chassis_tof1_stream">get_chassis_tof1_stream</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="tof-get_chassis_tof2_stream">get_chassis_tof2_stream</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h3 id="ultrasonic">Ultrasonic</h3>

<h4 id="ultrasonic-get_chassis_ultrasonic1">get_chassis_ultrasonic1</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic2">get_chassis_ultrasonic2</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic3">get_chassis_ultrasonic3</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic4">get_chassis_ultrasonic4</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic1_stream">get_chassis_ultrasonic1_stream</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic2_stream">get_chassis_ultrasonic2_stream</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic3_stream">get_chassis_ultrasonic3_stream</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h4 id="ultrasonic-get_chassis_ultrasonic4_stream">get_chassis_ultrasonic4_stream</h4>

**参数:**

* 无参数

**返回:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

---

<h3 id="waistcontroller">WaistController</h3>

<h4 id="waistcontroller-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

Control joint angles,

* `上限: [0.87266, 1.5708, 1.4486, 1.7453]`
* `下限: [-2.2689, -1.0472, -2.0944, -1.7453]`

**参数:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**返回:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="waistcontroller-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

获取关节状态

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="waistcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

获取关节状态流

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="waistcontroller-get_end_pose">get_end_pose</h4>

```python
def get_end_pose(timeout) -> _geometry_msgs__.PoseStamped
```

获取末端位姿

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="waistcontroller-get_end_pose_stream">get_end_pose_stream</h4>

```python
def get_end_pose_stream(timeout) -> Iterator[_geometry_msgs__.PoseStamped]
```

获取末端位姿流

**参数:**

* 无参数

**返回:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

## 类型

### 消息类型

<a id="message-builtin_interfacestime"></a>
#### Time

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `sec` | `int32` |  |
| `nanosec` | `uint32` |  |

---

<a id="message-geometry_msgspoint"></a>
#### Point

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |

---

<a id="message-geometry_msgspose"></a>
#### Pose

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `position` | [`Point`](#message-geometry_msgspoint) |  |
| `orientation` | [`Quaternion`](#message-geometry_msgsquaternion) |  |

---

<a id="message-geometry_msgsposestamped"></a>
#### PoseStamped

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `pose` | [`Pose`](#message-geometry_msgspose) |  |

---

<a id="message-geometry_msgsposewithcovariance"></a>
#### PoseWithCovariance

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `pose` | [`Pose`](#message-geometry_msgspose) |  |
| `covariance` | List[`double`] |  |

---

<a id="message-geometry_msgsquaternion"></a>
#### Quaternion

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |
| `w` | `double` |  |

---

<a id="message-geometry_msgstwist"></a>
#### Twist

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `linear` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `angular` | [`Vector3`](#message-geometry_msgsvector3) |  |

---

<a id="message-geometry_msgstwistwithcovariance"></a>
#### TwistWithCovariance

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `twist` | [`Twist`](#message-geometry_msgstwist) |  |
| `covariance` | List[`double`] |  |

---

<a id="message-geometry_msgsvector3"></a>
#### Vector3

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |

---

<a id="message-halheadpantiltcontrol"></a>
#### HeadPanTiltControl

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `yaw_position` | `float` |  |
| `yaw_velocity` | `float` |  |
| `pitch_position` | `float` |  |
| `pitch_velocity` | `float` |  |

---

<a id="message-nav_msgsodometry"></a>
#### Odometry

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `child_frame_id` | `string` |  |
| `pose` | [`PoseWithCovariance`](#message-geometry_msgsposewithcovariance) |  |
| `twist` | [`TwistWithCovariance`](#message-geometry_msgstwistwithcovariance) |  |

---

<a id="message-sensor_msgscompressedimage"></a>
#### CompressedImage

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `format` | `string` |  |
| `data` | `bytes` |  |

---

<a id="message-sensor_msgsimu"></a>
#### Imu

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `orientation` | [`Quaternion`](#message-geometry_msgsquaternion) |  |
| `orientation_covariance` | List[`double`] |  |
| `angular_velocity` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `angular_velocity_covariance` | List[`double`] |  |
| `linear_acceleration` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `linear_acceleration_covariance` | List[`double`] |  |

---

<a id="message-sensor_msgsjointstate"></a>
#### JointState

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `name` | List[`string`] |  |
| `position` | List[`double`] |  |
| `velocity` | List[`double`] |  |
| `effort` | List[`double`] |  |

---

<a id="message-sensor_msgslaserscan"></a>
#### LaserScan

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `angle_min` | `float` |  |
| `angle_max` | `float` |  |
| `angle_increment` | `float` |  |
| `time_increment` | `float` |  |
| `scan_time` | `float` |  |
| `range_min` | `float` |  |
| `range_max` | `float` |  |
| `ranges` | List[`float`] |  |
| `intensities` | List[`float`] |  |

---

<a id="message-sensor_msgspointcloud2"></a>
#### PointCloud2

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `height` | `uint32` |  |
| `width` | `uint32` |  |
| `fields` | List[[`PointField`](#message-sensor_msgspointfield)] |  |
| `is_bigendian` | `bool` |  |
| `point_step` | `uint32` |  |
| `row_step` | `uint32` |  |
| `data` | List[`uint32`] |  |
| `is_dense` | `bool` |  |

---

<a id="message-sensor_msgspointfield"></a>
#### PointField

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `name` | `string` |  |
| `offset` | `uint32` |  |
| `datatype` | `uint32` |  |
| `count` | `uint32` |  |

---

<a id="message-sensor_msgsrange"></a>
#### Range

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `radiation_type` | `uint32` |  |
| `field_of_view` | `float` |  |
| `min_range` | `float` |  |
| `max_range` | `float` |  |
| `range` | `float` |  |
| `variance` | `float` |  |

---

<a id="message-std_msgsfloat32"></a>
#### Float32

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `data` | `float` |  |

---

<a id="message-std_msgsfloat64multiarray"></a>
#### Float64MultiArray

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `layout` | [`MultiArrayLayout`](#message-std_msgsmultiarraylayout) |  |
| `data` | List[`double`] |  |

---

<a id="message-std_msgsheader"></a>
#### Header

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `stamp` | [`Time`](#message-builtin_interfacestime) |  |
| `frame_id` | `string` |  |

---

<a id="message-std_msgsmultiarraydimension"></a>
#### MultiArrayDimension

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `label` | `string` |  |
| `size` | `uint32` |  |
| `stride` | `uint32` |  |

---

<a id="message-std_msgsmultiarraylayout"></a>
#### MultiArrayLayout

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `dim` | List[[`MultiArrayDimension`](#message-std_msgsmultiarraydimension)] |  |
| `data_offset` | `uint32` |  |

---

<a id="message-std_msgsstring"></a>
#### String

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `data` | `string` |  |

---

<a id="message-xrsdkchassiscontrolmodeparam"></a>
#### ChassisControlModeParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `mode` | [`ChassisControlMode`](#enum-xrsdkchassiscontrolmode) |  |

---

<a id="message-xrsdkchassisposition"></a>
#### ChassisPosition

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `x` | `double` | x方向位置（单位：米） |
| `y` | `double` | y方向位置（单位：米） |
| `yaw` | `double` | 偏航角（单位：弧度） |

---

<a id="message-xrsdkchassispositionlist"></a>
#### ChassisPositionList

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `chunk_id` | `int32` (optional) | chunk_id用于标识位置块。可选，如果未设置，使用自增id。<br>导航系统将chunk_id 0作为起始位置。 |
| `positions` | List[[`ChassisPosition`](#message-xrsdkchassisposition)] |  |

---

<a id="message-xrsdkchassisvelocity"></a>
#### ChassisVelocity

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `vel_x` | `double` | x方向速度（单位：米/秒） |
| `vel_y` | `double` | 未使用，设置为0 |
| `vel_yaw` | `double` | 偏航角速度（单位：弧度/秒） |

---

<a id="message-xrsdkcoordinatesystemmodeparam"></a>
#### CoordinateSystemModeParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `coordinate_system_mode` | [`CoordinateSystemMode`](#enum-xrsdkcoordinatesystemmode) |  |

---

<a id="message-xrsdkexecutionresult"></a>
#### ExecutionResult

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `is_success` | `bool` |  |
| `error_message` | `string` |  |
| `error_code` | `ErrorCode` | 详细错误分类 |

---

<a id="message-xrsdkgripperposition"></a>
#### GripperPosition

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `position` | `float` |  |

---

<a id="message-xrsdkheadpose"></a>
#### HeadPose

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `pitch` | `float` | 俯仰角（单位：度），正值表示向上倾斜 |
| `yaw` | `float` | 偏航角（单位：度），正值表示向右转 |

---

<a id="message-xrsdkjointpositions"></a>
#### JointPositions

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `positions` | List[`double`] |  |

---

<a id="message-xrsdkliftposition"></a>
#### LiftPosition

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `position` | `float` | 升降台位置（单位：米），正值表示上升，负值表示下降 |

---

<a id="message-xrsdkmanipulatorcontrolmodeparam"></a>
#### ManipulatorControlModeParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `mode` | [`ManipulatorControlMode`](#enum-xrsdkmanipulatorcontrolmode) |  |

---

<a id="message-xrsdknavigationmodeparam"></a>
#### NavigationModeParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `mode` | [`NavigationMode`](#enum-xrsdknavigationmode) |  |

---

<a id="message-xrsdkpingrequest"></a>
#### PingRequest

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `payload` | `string` |  |

---

<a id="message-xrsdkpongresponse"></a>
#### PongResponse

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `payload` | `string` |  |

---

<a id="message-xrsdkpowerstatus"></a>
#### PowerStatus

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `is_charging` | `bool` | 机器人是否正在充电 |
| `value` | `float` | 电池电量 |

---

<a id="message-xrsdkrobotdynamicinfo"></a>
#### RobotDynamicInfo

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `power_status` | [`PowerStatus`](#message-xrsdkpowerstatus) |  |
| `runtime_info` | [`RobotRuntimeInfo`](#message-xrsdkrobotruntimeinfo) |  |

---

<a id="message-xrsdkrobotmodeparam"></a>
#### RobotModeParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `mode` | [`RobotWorkMode`](#enum-xrsdkrobotworkmode) |  |

---

<a id="message-xrsdkrobotruntimeinfo"></a>
#### RobotRuntimeInfo

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `cpu_load_percent` | `float` |  |
| `gpu_load_percent` | `float` |  |
| `memory_usage_mb` | `float` |  |
| `core_temp_celsius` | `float` |  |

---

<a id="message-xrsdkrobotstaticinfo"></a>
#### RobotStaticInfo

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `model_type` | [`RobotModelType`](#enum-xrsdkrobotmodeltype) |  |
| `model` | `string` |  |
| `robot_id` | `uint32` |  |
| `device_sn` | `string` |  |
| `device_name` | `string` |  |
| `software_version` | `string` |  |
| `hardware_version` | `string` | 保留供将来使用 |
| `device_ip` | List[`string`] |  |

---

<a id="message-xrsdksavemapparam"></a>
#### SaveMapParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `map_name` | `string` |  |

---

<a id="message-xrsdktactilesensordata"></a>
#### TactileSensorData

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `stamp` | [`Time`](#message-builtin_interfacestime) |  |
| `sensor_names` | List[`string`] |  |
| `frame_ids` | List[`string`] |  |
| `normal_forces` | List[`float`] |  |
| `tangential_forces` | List[`float`] |  |
| `directions` | List[`int32`] |  |
| `capacitances` | List[`uint32`] |  |
| `error_codes` | List[`uint32`] |  |

---

### 枚举类型

<a id="enum-sensor_msgspointfieldconstants"></a>
#### PointFieldConstants

| 值 | 说明 |
|------|--------|
| `INVALID0` (0) |  |
| `INT8` (1) |  |
| `UINT8` (2) |  |
| `INT16` (3) |  |
| `UINT16` (4) |  |
| `INT32` (5) |  |
| `UINT32` (6) |  |
| `FLOAT32` (7) |  |
| `FLOAT64` (8) |  |

---

<a id="enum-xrsdkchassiscontrolmode"></a>
#### ChassisControlMode

| 值 | 说明 |
|------|--------|
| `GLOBAL` (0) | 全局绝对位置控制，相对于地图坐标系。地图坐标系因场景而异，实际使用中很少使用。 |
| `RELATIVE` (1) | 相对位置控制（推荐！！！），相对于必须通过API设置的虚拟零点。 |
| `VELOCITY` (2) | 直接速度控制，需要提前禁用底盘位置规划器。 |

---

<a id="enum-xrsdkcoordinatesystemmode"></a>
#### CoordinateSystemMode

地图坐标系用于基于地图的导航。需在导航前设置此模式。

| 值 | 说明 |
|------|--------|
| `COORDINATE_SYSTEM_MODE_MAP` (0) | 地图坐标系用于基于地图的导航。需在导航前设置此模式。 |
| `COORDINATE_SYSTEM_MODE_ODOMETRY` (1) | 里程计坐标系用于未建图时的数据回放。需在数据回放前设置此模式。 |

---

<a id="enum-xrsdkmanipulatorcontrolmode"></a>
#### ManipulatorControlMode

| 值 | 说明 |
|------|--------|
| `MANIPULATOR_END_POSE` (0) | 机械臂（手臂和腰部）的末端位姿控制模式 |
| `MANIPULATOR_JOINT_POSITIONS` (1) | 机械臂（手臂和腰部）的关节位置控制模式 |

---

<a id="enum-xrsdknavigationmode"></a>
#### NavigationMode

| 值 | 说明 |
|------|--------|
| `BUILT_IN_NAVIGATION` (0) | 启用内置导航 |
| `USER_CUSTOM_NAVIGATION` (1) | 禁用内置导航 |

---

<a id="enum-xrsdkrobotmodeltype"></a>
#### RobotModelType

| 值 | 说明 |
|------|--------|
| `CX001` (0) |  |
| `CX002` (1) |  |
| `EX001` (2) |  |
| `DESKTOP` (3) |  |
| `INVALID_MODEL` (255) |  |

---

<a id="enum-xrsdkrobotworkmode"></a>
#### RobotWorkMode

| 值 | 说明 |
|------|--------|
| `IDLE` (0) | 机器人处于空闲状态，目前不支持此模式。 |
| `INFERE` (1) | 机器人处于推理模式，目前不支持此模式。 |
| `COLLECT` (2) | 机器人处于采集模式，目前不支持此模式。 |
| `SDK` (3) | 机器人处于SDK模式，目前仅支持此模式。 |

---
