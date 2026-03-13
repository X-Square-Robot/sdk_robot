# API 文档 - Desktop

## 目录

### 服务

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
   * [get_wrench_ext_world_stream](#leftarmcontroller-get_wrench_ext_world_stream)
   * [get_wrench_ext_local_stream](#leftarmcontroller-get_wrench_ext_local_stream)
* [LeftGripperController](#leftgrippercontroller)
   * [set_position](#leftgrippercontroller-set_position)
   * [get_position](#leftgrippercontroller-get_position)
   * [get_position_stream](#leftgrippercontroller-get_position_stream)
   * [get_joint_states_stream](#leftgrippercontroller-get_joint_states_stream)
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
   * [get_wrench_ext_world_stream](#rightarmcontroller-get_wrench_ext_world_stream)
   * [get_wrench_ext_local_stream](#rightarmcontroller-get_wrench_ext_local_stream)
* [RightGripperController](#rightgrippercontroller)
   * [set_position](#rightgrippercontroller-set_position)
   * [get_position](#rightgrippercontroller-get_position)
   * [get_position_stream](#rightgrippercontroller-get_position_stream)
   * [get_joint_states_stream](#rightgrippercontroller-get_joint_states_stream)
* [System](#system)
   * [set_work_mode](#system-set_work_mode)
   * [get_static_info](#system-get_static_info)
   * [get_dynamic_info](#system-get_dynamic_info)

### 消息类型列表

* [Time](#message-builtin_interfacestime)
* [Point](#message-geometry_msgspoint)
* [Pose](#message-geometry_msgspose)
* [PoseStamped](#message-geometry_msgsposestamped)
* [PoseWithCovariance](#message-geometry_msgsposewithcovariance)
* [Quaternion](#message-geometry_msgsquaternion)
* [Vector3](#message-geometry_msgsvector3)
* [Wrench](#message-geometry_msgswrench)
* [WrenchStamped](#message-geometry_msgswrenchstamped)
* [CompressedImage](#message-sensor_msgscompressedimage)
* [JointState](#message-sensor_msgsjointstate)
* [ExecutionResult](#message-xrsdkexecutionresult)
* [GripperPosition](#message-xrsdkgripperposition)
* [JointPositions](#message-xrsdkjointpositions)
* [ManipulatorControlModeParam](#message-xrsdkmanipulatorcontrolmodeparam)
* [PingRequest](#message-xrsdkpingrequest)
* [PongResponse](#message-xrsdkpongresponse)
* [PowerStatus](#message-xrsdkpowerstatus)
* [RobotDynamicInfo](#message-xrsdkrobotdynamicinfo)
* [RobotModeParam](#message-xrsdkrobotmodeparam)
* [RobotRuntimeInfo](#message-xrsdkrobotruntimeinfo)
* [RobotStaticInfo](#message-xrsdkrobotstaticinfo)

### 枚举类型列表

* [RobotModelType](#enum-xrsdkrobotmodeltype)
* [RobotWorkMode](#enum-xrsdkrobotworkmode)

---

## API 服务

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
   * `header` (`Header`)
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
   * `header` (`Header`)
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

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="leftarmcontroller-get_wrench_ext_local">get_wrench_ext_local</h4>

```python
def get_wrench_ext_local(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext local

**参数:**

* 无参数

**返回:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

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
   * `header` (`Header`)
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
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="leftarmcontroller-get_wrench_ext_world_stream">get_wrench_ext_world_stream</h4>

```python
def get_wrench_ext_world_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

获取外力世界坐标系流

**参数:**

* 无参数

**返回:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="leftarmcontroller-get_wrench_ext_local_stream">get_wrench_ext_local_stream</h4>

```python
def get_wrench_ext_local_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

获取外力局部坐标系流

**参数:**

* 无参数

**返回:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

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

<h4 id="leftgrippercontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

获取关节状态流

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

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
   * `header` (`Header`)
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
   * `header` (`Header`)
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

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="rightarmcontroller-get_wrench_ext_local">get_wrench_ext_local</h4>

```python
def get_wrench_ext_local(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext local

**参数:**

* 无参数

**返回:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

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
   * `header` (`Header`)
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
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="rightarmcontroller-get_wrench_ext_world_stream">get_wrench_ext_world_stream</h4>

```python
def get_wrench_ext_world_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

获取外力世界坐标系流

**参数:**

* 无参数

**返回:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="rightarmcontroller-get_wrench_ext_local_stream">get_wrench_ext_local_stream</h4>

```python
def get_wrench_ext_local_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

获取外力局部坐标系流

**参数:**

* 无参数

**返回:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

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

<h4 id="rightgrippercontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

获取关节状态流

**参数:**

* 无参数

**返回:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

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
| `header` | `Header` |  |
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

<a id="message-geometry_msgsvector3"></a>
#### Vector3

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |

---

<a id="message-geometry_msgswrench"></a>
#### Wrench

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `force` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `torque` | [`Vector3`](#message-geometry_msgsvector3) |  |

---

<a id="message-geometry_msgswrenchstamped"></a>
#### WrenchStamped

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | `Header` |  |
| `wrench` | [`Wrench`](#message-geometry_msgswrench) |  |

---

<a id="message-sensor_msgscompressedimage"></a>
#### CompressedImage

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | `Header` |  |
| `format` | `string` |  |
| `data` | `bytes` |  |

---

<a id="message-sensor_msgsjointstate"></a>
#### JointState

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `header` | `Header` |  |
| `name` | List[`string`] |  |
| `position` | List[`double`] |  |
| `velocity` | List[`double`] |  |
| `effort` | List[`double`] |  |

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

<a id="message-xrsdkjointpositions"></a>
#### JointPositions

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `positions` | List[`double`] |  |

---

<a id="message-xrsdkmanipulatorcontrolmodeparam"></a>
#### ManipulatorControlModeParam

**字段:**

| 字段 | 类型 | 说明 |
|------|------|--------|
| `mode` | `ManipulatorControlMode` |  |

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

### 枚举类型

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
