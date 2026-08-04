# API Documentation - Desktop

## Table of Contents

### Services

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
* [MasterLeftArm](#masterleftarm)
   * [get_control_mode](#masterleftarm-get_control_mode)
   * [set_control_mode](#masterleftarm-set_control_mode)
   * [get_joint_states](#masterleftarm-get_joint_states)
   * [get_end_pose](#masterleftarm-get_end_pose)
   * [get_gripper_position](#masterleftarm-get_gripper_position)
   * [get_joint_states_stream](#masterleftarm-get_joint_states_stream)
   * [get_end_pose_stream](#masterleftarm-get_end_pose_stream)
   * [get_gripper_state_stream](#masterleftarm-get_gripper_state_stream)
   * [set_joint_positions](#masterleftarm-set_joint_positions)
   * [set_end_pose](#masterleftarm-set_end_pose)
   * [get_gripper_joint_states_stream](#masterleftarm-get_gripper_joint_states_stream)
* [MasterRightArm](#masterrightarm)
   * [get_control_mode](#masterrightarm-get_control_mode)
   * [set_control_mode](#masterrightarm-set_control_mode)
   * [get_joint_states](#masterrightarm-get_joint_states)
   * [get_end_pose](#masterrightarm-get_end_pose)
   * [get_gripper_position](#masterrightarm-get_gripper_position)
   * [get_joint_states_stream](#masterrightarm-get_joint_states_stream)
   * [get_end_pose_stream](#masterrightarm-get_end_pose_stream)
   * [get_gripper_state_stream](#masterrightarm-get_gripper_state_stream)
   * [set_joint_positions](#masterrightarm-set_joint_positions)
   * [set_end_pose](#masterrightarm-set_end_pose)
   * [get_gripper_joint_states_stream](#masterrightarm-get_gripper_joint_states_stream)
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
* [RobotStatus](#robotstatus)
   * [get_robot_status](#robotstatus-get_robot_status)
* [System](#system)
   * [set_work_mode](#system-set_work_mode)
   * [get_static_info](#system-get_static_info)
   * [get_dynamic_info](#system-get_dynamic_info)
   * [get_model_type](#system-get_model_type)

### Message Types

* [Time](#message-builtin_interfacestime)
* [Point](#message-geometry_msgspoint)
* [Pose](#message-geometry_msgspose)
* [PoseStamped](#message-geometry_msgsposestamped)
* [PoseWithCovariance](#message-geometry_msgsposewithcovariance)
* [Quaternion](#message-geometry_msgsquaternion)
* [Transform](#message-geometry_msgstransform)
* [TransformStamped](#message-geometry_msgstransformstamped)
* [Vector3](#message-geometry_msgsvector3)
* [Wrench](#message-geometry_msgswrench)
* [WrenchStamped](#message-geometry_msgswrenchstamped)
* [CompressedImage](#message-sensor_msgscompressedimage)
* [JointState](#message-sensor_msgsjointstate)
* [TFMessage](#message-tf2_msgstfmessage)
* [AudioData](#message-xrsdkaudiodata)
* [AudioDataStamped](#message-xrsdkaudiodatastamped)
* [AudioInfo](#message-xrsdkaudioinfo)
* [DownloadMeta](#message-xrsdkdownloadmeta)
* [DownloadRequest](#message-xrsdkdownloadrequest)
* [DownloadResponse](#message-xrsdkdownloadresponse)
* [ExecutionResult](#message-xrsdkexecutionresult)
* [FileTransferMeta](#message-xrsdkfiletransfermeta)
* [GetMapListResponse](#message-xrsdkgetmaplistresponse)
* [GetRobotStatusReply](#message-xrsdkgetrobotstatusreply)
* [GetRobotStatusRequest](#message-xrsdkgetrobotstatusrequest)
* [GripperPosition](#message-xrsdkgripperposition)
* [JointPositions](#message-xrsdkjointpositions)
* [ManipulatorControlModeParam](#message-xrsdkmanipulatorcontrolmodeparam)
* [ModelTypeResult](#message-xrsdkmodeltyperesult)
* [PingRequest](#message-xrsdkpingrequest)
* [PlayAudioResponse](#message-xrsdkplayaudioresponse)
* [PongResponse](#message-xrsdkpongresponse)
* [PowerStatus](#message-xrsdkpowerstatus)
* [RobotDynamicInfo](#message-xrsdkrobotdynamicinfo)
* [RobotModeParam](#message-xrsdkrobotmodeparam)
* [RobotRuntimeInfo](#message-xrsdkrobotruntimeinfo)
* [RobotStaticInfo](#message-xrsdkrobotstaticinfo)
* [StartLocalizationParam](#message-xrsdkstartlocalizationparam)
* [StopAudioResponse](#message-xrsdkstopaudioresponse)
* [UploadRequest](#message-xrsdkuploadrequest)

### Enum Types

* [RobotModelType](#enum-xrsdkrobotmodeltype)
* [RobotWorkMode](#enum-xrsdkrobotworkmode)
* [VoicePromptPriority](#enum-xrsdkvoicepromptpriority)

---

## API Services

<h3 id="leftarmcontroller">LeftArmController</h3>

Left arm controller service

<h4 id="leftarmcontroller-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

Control joint angles (must set JOINT_POSITIONS mode first)

For Quanta_X2

* `upper limit: [3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708]`
* `lower limit: [-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708]`

For Quanta_X1 and Desktop

* `upper limit: [2.792, 3.44, 3.14, 1.57, 1.4, 1.745]`
* `lower limit: [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]`

**Parameters:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftarmcontroller-set_end_pose">set_end_pose</h4>

```python
def set_end_pose(_geometry_msgs__: _geometry_msgs__.Pose, timeout) -> ExecutionResult
```

Control end effector pose (must set END_POSE mode first)

For Quanta_X2

* `upper limit: [5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0]`
* `lower limit: [-5.0, -5.0, -5.0, -1.0, -1.0, -1.0, -1.0]`

For Quanta_X1 and Desktop

* `upper limit: [5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0]`
* `lower limit: [-5.0, -5.0, -5.0, -1.0, -1.0, -1.0, -1.0]`

**Parameters:**

* `position` ([`Point`](#message-geometry_msgspoint))
* `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftarmcontroller-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

Get joint states (positions, velocities, efforts)

**Parameters:**

* No parameters

**Returns:**

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

Get end effector pose

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="leftarmcontroller-reset">reset</h4>

```python
def reset(timeout) -> ExecutionResult
```

Reset arm to home position

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftarmcontroller-get_wrench_ext_world">get_wrench_ext_world</h4>

```python
def get_wrench_ext_world(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext world

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="leftarmcontroller-get_wrench_ext_local">get_wrench_ext_local</h4>

```python
def get_wrench_ext_local(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext local

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="leftarmcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get joint states stream

**Parameters:**

* No parameters

**Returns:**

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

Get end pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="leftarmcontroller-get_wrench_ext_world_stream">get_wrench_ext_world_stream</h4>

```python
def get_wrench_ext_world_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

Get wrench ext world stream

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="leftarmcontroller-get_wrench_ext_local_stream">get_wrench_ext_local_stream</h4>

```python
def get_wrench_ext_local_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

Get wrench ext local stream

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h3 id="leftgrippercontroller">LeftGripperController</h3>

<h4 id="leftgrippercontroller-set_position">set_position</h4>

```python
def set_position(gripper_position: GripperPosition, timeout) -> ExecutionResult
```

Control gripper opening/closing degree

For Quanta_X2

* `upper limit: 25.2`
* `lower limit: 0.0`

For Quanta_X1

* `upper limit: 4.5`
* `lower limit: 0.0`

**Parameters:**

* `gripper_position` ([`GripperPosition`](#message-xrsdkgripperposition))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="leftgrippercontroller-get_position">get_position</h4>

```python
def get_position(timeout) -> GripperPosition
```

Get current gripper state

**Parameters:**

* No parameters

**Returns:**

* [`GripperPosition`](#message-xrsdkgripperposition)

---

<h4 id="leftgrippercontroller-get_position_stream">get_position_stream</h4>

```python
def get_position_stream(timeout) -> Iterator[GripperPosition]
```

Get position stream

**Parameters:**

* No parameters

**Returns:**

* `Iterator[[`GripperPosition`](#message-xrsdkgripperposition)]`: Stream of GripperPosition

---

<h4 id="leftgrippercontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get Joint state stream

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="masterleftarm">MasterLeftArm</h3>

============================================================================

<h4 id="masterleftarm-get_control_mode">get_control_mode</h4>

```python
def get_control_mode(timeout) -> ManipulatorControlModeParam
```

**Parameters:**

* No parameters

**Returns:**

* [`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam)

---

<h4 id="masterleftarm-set_control_mode">set_control_mode</h4>

```python
def set_control_mode(manipulator_control_mode_param: ManipulatorControlModeParam, timeout) -> ExecutionResult
```

**Parameters:**

* `manipulator_control_mode_param` ([`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="masterleftarm-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

Get joint states

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="masterleftarm-get_end_pose">get_end_pose</h4>

```python
def get_end_pose(timeout) -> _geometry_msgs__.PoseStamped
```

Get End Pose

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="masterleftarm-get_gripper_position">get_gripper_position</h4>

```python
def get_gripper_position(timeout) -> GripperPosition
```

**Parameters:**

* No parameters

**Returns:**

* [`GripperPosition`](#message-xrsdkgripperposition)

---

<h4 id="masterleftarm-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get joint state stream

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="masterleftarm-get_end_pose_stream">get_end_pose_stream</h4>

```python
def get_end_pose_stream(timeout) -> Iterator[_geometry_msgs__.PoseStamped]
```

Get End Pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="masterleftarm-get_gripper_state_stream">get_gripper_state_stream</h4>

```python
def get_gripper_state_stream(timeout) -> Iterator[GripperPosition]
```

**Parameters:**

* No parameters

**Returns:**

* `Iterator[[`GripperPosition`](#message-xrsdkgripperposition)]`: Stream of GripperPosition

---

<h4 id="masterleftarm-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

Placeholder methods, backend is not supported yet.

**Parameters:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="masterleftarm-set_end_pose">set_end_pose</h4>

```python
def set_end_pose(_geometry_msgs__: _geometry_msgs__.Pose, timeout) -> ExecutionResult
```

Control end effector pose (must set END_POSE mode first)
position: [x, y, z] in meters, range: [-5.0, 5.0]
orientation: [qx, qy, qz, qw] quaternion, range: [-1.0, 1.0]

**Parameters:**

* `position` ([`Point`](#message-geometry_msgspoint))
* `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="masterleftarm-get_gripper_joint_states_stream">get_gripper_joint_states_stream</h4>

```python
def get_gripper_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Backward compatibility for old API shape.

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="masterrightarm">MasterRightArm</h3>

<h4 id="masterrightarm-get_control_mode">get_control_mode</h4>

```python
def get_control_mode(timeout) -> ManipulatorControlModeParam
```

**Parameters:**

* No parameters

**Returns:**

* [`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam)

---

<h4 id="masterrightarm-set_control_mode">set_control_mode</h4>

```python
def set_control_mode(manipulator_control_mode_param: ManipulatorControlModeParam, timeout) -> ExecutionResult
```

**Parameters:**

* `manipulator_control_mode_param` ([`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="masterrightarm-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

Get joint states

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="masterrightarm-get_end_pose">get_end_pose</h4>

```python
def get_end_pose(timeout) -> _geometry_msgs__.PoseStamped
```

Get End Pose

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="masterrightarm-get_gripper_position">get_gripper_position</h4>

```python
def get_gripper_position(timeout) -> GripperPosition
```

**Parameters:**

* No parameters

**Returns:**

* [`GripperPosition`](#message-xrsdkgripperposition)

---

<h4 id="masterrightarm-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get joint state stream

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h4 id="masterrightarm-get_end_pose_stream">get_end_pose_stream</h4>

```python
def get_end_pose_stream(timeout) -> Iterator[_geometry_msgs__.PoseStamped]
```

Get End Pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="masterrightarm-get_gripper_state_stream">get_gripper_state_stream</h4>

```python
def get_gripper_state_stream(timeout) -> Iterator[GripperPosition]
```

**Parameters:**

* No parameters

**Returns:**

* `Iterator[[`GripperPosition`](#message-xrsdkgripperposition)]`: Stream of GripperPosition

---

<h4 id="masterrightarm-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

Placeholder methods, backend is not supported yet.

**Parameters:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="masterrightarm-set_end_pose">set_end_pose</h4>

```python
def set_end_pose(_geometry_msgs__: _geometry_msgs__.Pose, timeout) -> ExecutionResult
```

Control end effector pose (must set END_POSE mode first)
position: [x, y, z] in meters, range: [-5.0, 5.0]
orientation: [qx, qy, qz, qw] quaternion, range: [-1.0, 1.0]

**Parameters:**

* `position` ([`Point`](#message-geometry_msgspoint))
* `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="masterrightarm-get_gripper_joint_states_stream">get_gripper_joint_states_stream</h4>

```python
def get_gripper_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Backward compatibility for old API shape.

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="rightarmcontroller">RightArmController</h3>

Right arm controller service

<h4 id="rightarmcontroller-set_joint_positions">set_joint_positions</h4>

```python
def set_joint_positions(joint_positions: JointPositions, timeout) -> ExecutionResult
```

Control joint angles (must set JOINT_POSITIONS mode first)

For Quanta_X2

* `upper limit: [3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708]`
* `lower limit: [-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708]`

For Quanta_X1 and Desktop

* `upper limit: [2.792, 3.44, 3.14, 1.57, 1.4, 1.745]`
* `lower limit: [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]`

**Parameters:**

* `joint_positions` ([`JointPositions`](#message-xrsdkjointpositions))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightarmcontroller-set_end_pose">set_end_pose</h4>

```python
def set_end_pose(_geometry_msgs__: _geometry_msgs__.Pose, timeout) -> ExecutionResult
```

Control end effector pose (must set END_POSE mode first)

Supports position and orientation control

**Parameters:**

* `position` ([`Point`](#message-geometry_msgspoint))
* `orientation` ([`Quaternion`](#message-geometry_msgsquaternion))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightarmcontroller-get_joint_states">get_joint_states</h4>

```python
def get_joint_states(timeout) -> _sensor_msgs__.JointState
```

Get joint states (positions, velocities, efforts)

**Parameters:**

* No parameters

**Returns:**

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

Get end effector pose

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="rightarmcontroller-reset">reset</h4>

```python
def reset(timeout) -> ExecutionResult
```

Reset arm to home position

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightarmcontroller-get_wrench_ext_world">get_wrench_ext_world</h4>

```python
def get_wrench_ext_world(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext world

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="rightarmcontroller-get_wrench_ext_local">get_wrench_ext_local</h4>

```python
def get_wrench_ext_local(timeout) -> _geometry_msgs__.WrenchStamped
```

Get wrench ext local

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="rightarmcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get joint states stream

**Parameters:**

* No parameters

**Returns:**

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

Get end pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` (`Header`)
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="rightarmcontroller-get_wrench_ext_world_stream">get_wrench_ext_world_stream</h4>

```python
def get_wrench_ext_world_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

Get wrench ext world stream

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h4 id="rightarmcontroller-get_wrench_ext_local_stream">get_wrench_ext_local_stream</h4>

```python
def get_wrench_ext_local_stream(timeout) -> Iterator[_geometry_msgs__.WrenchStamped]
```

Get wrench ext local stream

**Parameters:**

* No parameters

**Returns:**

* [`WrenchStamped`](#message-geometry_msgswrenchstamped)
   * `header` (`Header`)
   * `wrench` ([`Wrench`](#message-geometry_msgswrench))

---

<h3 id="rightgrippercontroller">RightGripperController</h3>

<h4 id="rightgrippercontroller-set_position">set_position</h4>

```python
def set_position(gripper_position: GripperPosition, timeout) -> ExecutionResult
```

Control gripper opening/closing degree

For Quanta_X2

* `upper limit: 25.2`
* `lower limit: 0.0`

For Quanta_X1

* `upper limit: 4.5`
* `lower limit: 0.0`

**Parameters:**

* `gripper_position` ([`GripperPosition`](#message-xrsdkgripperposition))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="rightgrippercontroller-get_position">get_position</h4>

```python
def get_position(timeout) -> GripperPosition
```

Get current gripper state

**Parameters:**

* No parameters

**Returns:**

* [`GripperPosition`](#message-xrsdkgripperposition)

---

<h4 id="rightgrippercontroller-get_position_stream">get_position_stream</h4>

```python
def get_position_stream(timeout) -> Iterator[GripperPosition]
```

Get position stream

**Parameters:**

* No parameters

**Returns:**

* `Iterator[[`GripperPosition`](#message-xrsdkgripperposition)]`: Stream of GripperPosition

---

<h4 id="rightgrippercontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get Joint state stream

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` (`Header`)
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="robotstatus">RobotStatus</h3>

Real-time whole-robot status query service; client wraps JSON payload in SdkResult.

<h4 id="robotstatus-get_robot_status">get_robot_status</h4>

```python
def get_robot_status(fields = None, request_id = None) -> SdkResult
```

Query the robot's real-time status.

**Parameters:**

* `fields` - Filter list. None or [] returns everything; supports two-level paths, e.g. ["energy"] (whole category) or ["energy.battery_level"] (single field).
* `request_id` - Per-call trace id; auto-generated as uuid4 when omitted, echoed back in SdkResult.request_id for log correlation.

**Returns:**

* `SdkResult`: on success, `data` is a status dict (energy/motion/execution/safety/health); on failure, `error` is a standard ErrorCode

---

<h3 id="system">System</h3>

<h4 id="system-set_work_mode">set_work_mode</h4>

```python
def set_work_mode(robot_mode_param: RobotModeParam, timeout) -> ExecutionResult
```

Set Robot work mode: IDLE, INFERE, COLLECT, SDK

**Parameters:**

* `robot_mode_param` ([`RobotModeParam`](#message-xrsdkrobotmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="system-get_static_info">get_static_info</h4>

```python
def get_static_info(timeout) -> RobotStaticInfo
```

Get Robot system info

**Parameters:**

* No parameters

**Returns:**

* [`RobotStaticInfo`](#message-xrsdkrobotstaticinfo)

---

<h4 id="system-get_dynamic_info">get_dynamic_info</h4>

```python
def get_dynamic_info(timeout) -> RobotDynamicInfo
```

Get Robot runtime info

**Parameters:**

* No parameters

**Returns:**

* [`RobotDynamicInfo`](#message-xrsdkrobotdynamicinfo)

---

<h4 id="system-get_model_type">get_model_type</h4>

```python
def get_model_type(timeout) -> ModelTypeResult
```

Get robot model type (works on all models, does not depend on application node)

**Parameters:**

* No parameters

**Returns:**

* [`ModelTypeResult`](#message-xrsdkmodeltyperesult)

---

## Types

### Messages

<a id="message-builtin_interfacestime"></a>
#### Time

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `sec` | `int32` |  |
| `nanosec` | `uint32` |  |

---

<a id="message-geometry_msgspoint"></a>
#### Point

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |

---

<a id="message-geometry_msgspose"></a>
#### Pose

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `position` | [`Point`](#message-geometry_msgspoint) |  |
| `orientation` | [`Quaternion`](#message-geometry_msgsquaternion) |  |

---

<a id="message-geometry_msgsposestamped"></a>
#### PoseStamped

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `Header` |  |
| `pose` | [`Pose`](#message-geometry_msgspose) |  |

---

<a id="message-geometry_msgsposewithcovariance"></a>
#### PoseWithCovariance

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `pose` | [`Pose`](#message-geometry_msgspose) |  |
| `covariance` | List[`double`] |  |

---

<a id="message-geometry_msgsquaternion"></a>
#### Quaternion

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |
| `w` | `double` |  |

---

<a id="message-geometry_msgstransform"></a>
#### Transform

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `translation` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `rotation` | [`Quaternion`](#message-geometry_msgsquaternion) |  |

---

<a id="message-geometry_msgstransformstamped"></a>
#### TransformStamped

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `Header` |  |
| `child_frame_id` | `string` |  |
| `transform` | [`Transform`](#message-geometry_msgstransform) |  |

---

<a id="message-geometry_msgsvector3"></a>
#### Vector3

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `x` | `double` |  |
| `y` | `double` |  |
| `z` | `double` |  |

---

<a id="message-geometry_msgswrench"></a>
#### Wrench

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `force` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `torque` | [`Vector3`](#message-geometry_msgsvector3) |  |

---

<a id="message-geometry_msgswrenchstamped"></a>
#### WrenchStamped

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `Header` |  |
| `wrench` | [`Wrench`](#message-geometry_msgswrench) |  |

---

<a id="message-sensor_msgscompressedimage"></a>
#### CompressedImage

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `Header` |  |
| `format` | `string` |  |
| `data` | `bytes` |  |

---

<a id="message-sensor_msgsjointstate"></a>
#### JointState

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `Header` |  |
| `name` | List[`string`] |  |
| `position` | List[`double`] |  |
| `velocity` | List[`double`] |  |
| `effort` | List[`double`] |  |

---

<a id="message-tf2_msgstfmessage"></a>
#### TFMessage

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `transforms` | List[[`TransformStamped`](#message-geometry_msgstransformstamped)] |  |

---

<a id="message-xrsdkaudiodata"></a>
#### AudioData

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `data` | `bytes` |  |

---

<a id="message-xrsdkaudiodatastamped"></a>
#### AudioDataStamped

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `audio_info` | [`AudioInfo`](#message-xrsdkaudioinfo) |  |
| `audio_data` | [`AudioData`](#message-xrsdkaudiodata) |  |

---

<a id="message-xrsdkaudioinfo"></a>
#### AudioInfo

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `channels` | `uint32` |  |
| `sample_rate` | `uint32` |  |
| `sample_format` | `string` |  |
| `bitrate` | `uint32` |  |
| `coding_format` | `string` |  |
| `bit_depth` | `uint32` |  |

---

<a id="message-xrsdkdownloadmeta"></a>
#### DownloadMeta

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `total_size` | `uint64` |  |

---

<a id="message-xrsdkdownloadrequest"></a>
#### DownloadRequest

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `identifier` | `string` |  |

---

<a id="message-xrsdkdownloadresponse"></a>
#### DownloadResponse

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `meta` | [`DownloadMeta`](#message-xrsdkdownloadmeta) |  |
| `chunk` | `bytes` |  |

---

<a id="message-xrsdkexecutionresult"></a>
#### ExecutionResult

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `is_success` | `bool` |  |
| `error_message` | `string` |  |
| `error_code` | `ErrorCode` | Detailed error classification |

---

<a id="message-xrsdkfiletransfermeta"></a>
#### FileTransferMeta

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `identifier` | `string` |  |
| `total_size` | `uint64` |  |

---

<a id="message-xrsdkgetmaplistresponse"></a>
#### GetMapListResponse

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | [`ExecutionResult`](#message-xrsdkexecutionresult) | Call status: is_success / error_code / error_message |
| `map_list` | List[`string`] | List of saved map names |

---

<a id="message-xrsdkgetrobotstatusreply"></a>
#### GetRobotStatusReply

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `json` | `string` | Payload (JSON string) grouped into energy/motion/execution/safety/health.<br>Unavailable or uncaptured fields are null. |

---

<a id="message-xrsdkgetrobotstatusrequest"></a>
#### GetRobotStatusRequest

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `fields` | List[`string`] | Filter field list:<br>empty -> all fields;<br>"energy" -> whole category;<br>"energy.battery_level" -> single field (two-level path). |

---

<a id="message-xrsdkgripperposition"></a>
#### GripperPosition

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | `Header` |  |
| `position` | `float` |  |

---

<a id="message-xrsdkjointpositions"></a>
#### JointPositions

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `positions` | List[`double`] |  |

---

<a id="message-xrsdkmanipulatorcontrolmodeparam"></a>
#### ManipulatorControlModeParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | `ManipulatorControlMode` |  |

---

<a id="message-xrsdkmodeltyperesult"></a>
#### ModelTypeResult

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `model_type` | [`RobotModelType`](#enum-xrsdkrobotmodeltype) |  |

---

<a id="message-xrsdkpingrequest"></a>
#### PingRequest

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `payload` | `string` |  |

---

<a id="message-xrsdkplayaudioresponse"></a>
#### PlayAudioResponse

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` |  |
| `message` | `string` |  |
| `play_id` | `uint64` | equals ROS request_id; reserved 0 for "stop all" |
| `resource_id` | `string` | empty when cache=false |

---

<a id="message-xrsdkpongresponse"></a>
#### PongResponse

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `payload` | `string` |  |

---

<a id="message-xrsdkpowerstatus"></a>
#### PowerStatus

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `is_charging` | `bool` | Whether the robot is charging |
| `value` | `float` | Battery level |

---

<a id="message-xrsdkrobotdynamicinfo"></a>
#### RobotDynamicInfo

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `power_status` | [`PowerStatus`](#message-xrsdkpowerstatus) |  |
| `runtime_info` | [`RobotRuntimeInfo`](#message-xrsdkrobotruntimeinfo) |  |

---

<a id="message-xrsdkrobotmodeparam"></a>
#### RobotModeParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | [`RobotWorkMode`](#enum-xrsdkrobotworkmode) |  |

---

<a id="message-xrsdkrobotruntimeinfo"></a>
#### RobotRuntimeInfo

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `cpu_load_percent` | `float` |  |
| `gpu_load_percent` | `float` |  |
| `memory_usage_mb` | `float` |  |
| `core_temp_celsius` | `float` |  |

---

<a id="message-xrsdkrobotstaticinfo"></a>
#### RobotStaticInfo

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `model_type` | [`RobotModelType`](#enum-xrsdkrobotmodeltype) |  |
| `model` | `string` |  |
| `robot_id` | `uint32` |  |
| `device_sn` | `string` |  |
| `device_name` | `string` |  |
| `software_version` | `string` |  |
| `hardware_version` | `string` | reserved for future use |
| `device_ip` | List[`string`] |  |

---

<a id="message-xrsdkstartlocalizationparam"></a>
#### StartLocalizationParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `map_name` | `string` | Map name used for localization |
| `use_init_pose` | `bool` | Whether to use initial pose for localization |
| `init_pose` | [`Pose`](#message-geometry_msgspose) | Initial pose in map coordinate system, valid only when use_init_pose is true |

---

<a id="message-xrsdkstopaudioresponse"></a>
#### StopAudioResponse

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` |  |
| `message` | `string` |  |

---

<a id="message-xrsdkuploadrequest"></a>
#### UploadRequest

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `meta` | [`FileTransferMeta`](#message-xrsdkfiletransfermeta) |  |
| `chunk` | `bytes` |  |

---

### Enums

<a id="enum-xrsdkrobotmodeltype"></a>
#### RobotModelType

| Value | Description |
|-------|-------------|
| `CX001` (0) |  |
| `CX002` (1) |  |
| `EX001` (2) |  |
| `DESKTOP` (3) |  |
| `EX001_MASTER` (4) |  |
| `EX002` (5) |  |
| `INVALID_MODEL` (255) |  |

---

<a id="enum-xrsdkrobotworkmode"></a>
#### RobotWorkMode

| Value | Description |
|-------|-------------|
| `IDLE` (0) | Robot is idle |
| `INFERE` (1) | Robot is in inference mode |
| `COLLECT` (2) | Robot is in collect mode |
| `SDK` (3) | Robot is in SDK mode |

---

<a id="enum-xrsdkvoicepromptpriority"></a>
#### VoicePromptPriority

| Value | Description |
|-------|-------------|
| `PRIORITY_URGENT` (0) |  |
| `PRIORITY_HIGH` (1) |  |
| `PRIORITY_NORMAL` (2) |  |

---
