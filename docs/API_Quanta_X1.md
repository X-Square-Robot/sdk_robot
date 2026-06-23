# API Documentation - Quanta_X1

## Table of Contents

### Services

* [Audio](#audio)
   * [play_audio](#audio-play_audio)
   * [replay](#audio-replay)
   * [stop_audio](#audio-stop_audio)
   * [get_audio_stream](#audio-get_audio_stream)
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
   * [get_wrench_ext_world_stream](#leftarmcontroller-get_wrench_ext_world_stream)
   * [get_wrench_ext_local_stream](#leftarmcontroller-get_wrench_ext_local_stream)
* [LeftGripperController](#leftgrippercontroller)
   * [set_position](#leftgrippercontroller-set_position)
   * [get_position](#leftgrippercontroller-get_position)
   * [get_position_stream](#leftgrippercontroller-get_position_stream)
   * [get_joint_states_stream](#leftgrippercontroller-get_joint_states_stream)
* [LiftController](#liftcontroller)
   * [set_lift_position](#liftcontroller-set_lift_position)
   * [get_lift_position](#liftcontroller-get_lift_position)
   * [get_joint_states_stream](#liftcontroller-get_joint_states_stream)
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
* [Navigation](#navigation)
   * [start_mapping](#navigation-start_mapping)
   * [stop_mapping](#navigation-stop_mapping)
   * [set_navigation_mode](#navigation-set_navigation_mode)
   * [start_localization](#navigation-start_localization)
   * [stop_localization](#navigation-stop_localization)
   * [cancel_navigation](#navigation-cancel_navigation)
   * [load_map](#navigation-load_map)
   * [export_map](#navigation-export_map)
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
   * [get_wrench_ext_world_stream](#rightarmcontroller-get_wrench_ext_world_stream)
   * [get_wrench_ext_local_stream](#rightarmcontroller-get_wrench_ext_local_stream)
* [RightGripperController](#rightgrippercontroller)
   * [set_position](#rightgrippercontroller-set_position)
   * [get_position](#rightgrippercontroller-get_position)
   * [get_position_stream](#rightgrippercontroller-get_position_stream)
   * [get_joint_states_stream](#rightgrippercontroller-get_joint_states_stream)
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
   * [get_model_type](#system-get_model_type)
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

### Message Types

* [Time](#message-builtin_interfacestime)
* [Point](#message-geometry_msgspoint)
* [Pose](#message-geometry_msgspose)
* [PoseStamped](#message-geometry_msgsposestamped)
* [PoseWithCovariance](#message-geometry_msgsposewithcovariance)
* [Quaternion](#message-geometry_msgsquaternion)
* [Twist](#message-geometry_msgstwist)
* [TwistWithCovariance](#message-geometry_msgstwistwithcovariance)
* [Vector3](#message-geometry_msgsvector3)
* [Wrench](#message-geometry_msgswrench)
* [WrenchStamped](#message-geometry_msgswrenchstamped)
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
* [AudioData](#message-xrsdkaudiodata)
* [AudioDataStamped](#message-xrsdkaudiodatastamped)
* [AudioInfo](#message-xrsdkaudioinfo)
* [ChassisControlModeParam](#message-xrsdkchassiscontrolmodeparam)
* [ChassisPosition](#message-xrsdkchassisposition)
* [ChassisPositionList](#message-xrsdkchassispositionlist)
* [ChassisVelocity](#message-xrsdkchassisvelocity)
* [CoordinateSystemModeParam](#message-xrsdkcoordinatesystemmodeparam)
* [DownloadMeta](#message-xrsdkdownloadmeta)
* [DownloadRequest](#message-xrsdkdownloadrequest)
* [DownloadResponse](#message-xrsdkdownloadresponse)
* [ExecutionResult](#message-xrsdkexecutionresult)
* [FileTransferMeta](#message-xrsdkfiletransfermeta)
* [GripperPosition](#message-xrsdkgripperposition)
* [HeadPose](#message-xrsdkheadpose)
* [JointPositions](#message-xrsdkjointpositions)
* [LiftPosition](#message-xrsdkliftposition)
* [ManipulatorControlModeParam](#message-xrsdkmanipulatorcontrolmodeparam)
* [ModelTypeResult](#message-xrsdkmodeltyperesult)
* [NavigationModeParam](#message-xrsdknavigationmodeparam)
* [PingRequest](#message-xrsdkpingrequest)
* [PlayAudioResponse](#message-xrsdkplayaudioresponse)
* [PongResponse](#message-xrsdkpongresponse)
* [PowerStatus](#message-xrsdkpowerstatus)
* [RobotDynamicInfo](#message-xrsdkrobotdynamicinfo)
* [RobotModeParam](#message-xrsdkrobotmodeparam)
* [RobotRuntimeInfo](#message-xrsdkrobotruntimeinfo)
* [RobotStaticInfo](#message-xrsdkrobotstaticinfo)
* [SaveMapParam](#message-xrsdksavemapparam)
* [StartLocalizationParam](#message-xrsdkstartlocalizationparam)
* [StopAudioResponse](#message-xrsdkstopaudioresponse)
* [TactileSensorData](#message-xrsdktactilesensordata)
* [UploadRequest](#message-xrsdkuploadrequest)

### Enum Types

* [PointFieldConstants](#enum-sensor_msgspointfieldconstants)
* [ChassisControlMode](#enum-xrsdkchassiscontrolmode)
* [CoordinateSystemMode](#enum-xrsdkcoordinatesystemmode)
* [ManipulatorControlMode](#enum-xrsdkmanipulatorcontrolmode)
* [NavigationMode](#enum-xrsdknavigationmode)
* [RobotModelType](#enum-xrsdkrobotmodeltype)
* [RobotWorkMode](#enum-xrsdkrobotworkmode)
* [VoicePromptPriority](#enum-xrsdkvoicepromptpriority)

---

## API Services

<h3 id="audio">Audio</h3>

Audio service

<h4 id="audio-play_audio">play_audio</h4>

```python
def play_audio(source: str, volume: int = 80, block: bool = False, priority: int = 2, cache: bool = True, resource_id: Optional[str] = None) -> AudioResult
```

Upload audio from a local file or URL and play it.

Audio format is auto-detected from content (supports wav, mp3).

Max single upload is 60 MB; client rejects locally if the size is exceeded.

**Parameters:**

* `source` (str) - Local file path or http(s):// URL.
* `volume` (int) - 1-100.
* `block` (bool) - Wait until playback finishes.
* `priority` (int) - 0=urgent, 1=high, 2=normal.
* `cache` (bool) - Whether the server should cache the resource for replay.
* `resource_id` (Optional[str]) - Optional cache-key hint (overrides server-computed sha).

**Returns:**

* [`AudioResult`](#message-xrsdkaudioresult)

---

<h4 id="audio-replay">replay</h4>

```python
def replay(resource_id: str, volume: int = 80, block: bool = False, priority: int = 2) -> AudioResult
```

Replay a previously cached resource by id.

**Parameters:**

* `resource_id` (str) - Cache key returned from a previous play_audio call.
* `volume` (int) - 1-100.
* `block` (bool) - Wait until playback finishes.
* `priority` (int) - 0=urgent, 1=high, 2=normal.

**Returns:**

* [`AudioResult`](#message-xrsdkaudioresult)

---

<h4 id="audio-stop_audio">stop_audio</h4>

```python
def stop_audio(play_id: int = 0) -> StopAudioResult
```

Stop a specific playback task or all tasks.

**Parameters:**

* `play_id` (int) - The play_id to stop. 0 means stop all queued/playing tasks.

**Returns:**

* [`StopAudioResult`](#message-xrsdkstopaudioresult)

---

<h4 id="audio-get_audio_stream">get_audio_stream</h4>

```python
def get_audio_stream() -> Iterator[AudioDataStamped]
```

Stream microphone audio frames. Sample rate is determined by the server.

**Parameters:**

* No parameters

**Returns:**

* `Iterator[`[`AudioDataStamped`](#message-xrsdkaudiodatastamped)`]`

---

<h3 id="chassiscontroller">ChassisController</h3>

<h4 id="chassiscontroller-set_control_mode">set_control_mode</h4>

```python
def set_control_mode(manipulator_control_mode_param: ManipulatorControlModeParam, timeout) -> ExecutionResult
```

Set control mode: global position, relative position, or velocity control

**Parameters:**

* `chassis_control_mode_param` ([`ChassisControlModeParam`](#message-xrsdkchassiscontrolmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-get_control_mode">get_control_mode</h4>

```python
def get_control_mode(timeout) -> ManipulatorControlModeParam
```

Get current control mode

**Parameters:**

* No parameters

**Returns:**

* [`ChassisControlModeParam`](#message-xrsdkchassiscontrolmodeparam)

---

<h4 id="chassiscontroller-move_to_global_position">move_to_global_position</h4>

```python
def move_to_global_position(chassis_position: ChassisPosition, timeout) -> ExecutionResult
```

Move to global position (must set GLOBAL mode first)

**Parameters:**

* `chassis_position` ([`ChassisPosition`](#message-xrsdkchassisposition))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-move_to_relative_position">move_to_relative_position</h4>

```python
def move_to_relative_position(chassis_position: ChassisPosition, timeout) -> ExecutionResult
```

Move to relative position (must set RELATIVE mode and virtual zero point first)

**Parameters:**

* `chassis_position` ([`ChassisPosition`](#message-xrsdkchassisposition))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-set_velocity">set_velocity</h4>

```python
def set_velocity(chassis_velocity: ChassisVelocity, timeout) -> ExecutionResult
```

Set velocity control (must set VELOCITY mode first)

**Parameters:**

* `chassis_velocity` ([`ChassisVelocity`](#message-xrsdkchassisvelocity))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-set_virtual_zero_point">set_virtual_zero_point</h4>

```python
def set_virtual_zero_point(chassis_position: ChassisPosition, timeout) -> ExecutionResult
```

Set virtual zero point (origin for relative movement)

**Parameters:**

* `chassis_position` ([`ChassisPosition`](#message-xrsdkchassisposition))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-get_virtual_zero_point">get_virtual_zero_point</h4>

```python
def get_virtual_zero_point(timeout) -> ChassisPosition
```

Get current virtual zero point setting

**Parameters:**

* No parameters

**Returns:**

* [`ChassisPosition`](#message-xrsdkchassisposition)

---

<h4 id="chassiscontroller-get_global_position">get_global_position</h4>

```python
def get_global_position(timeout) -> ChassisPosition
```

Get current global position

**Parameters:**

* No parameters

**Returns:**

* [`ChassisPosition`](#message-xrsdkchassisposition)

---

<h4 id="chassiscontroller-get_relative_position">get_relative_position</h4>

```python
def get_relative_position(timeout) -> ChassisPosition
```

Get current relative position (relative to virtual zero point)

**Parameters:**

* No parameters

**Returns:**

* [`ChassisPosition`](#message-xrsdkchassisposition)

---

<h4 id="chassiscontroller-get_odometry">get_odometry</h4>

```python
def get_odometry(timeout) -> _nav_msgs__.Odometry
```

Get current odometry

**Parameters:**

* No parameters

**Returns:**

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

Get odometry stream

**Parameters:**

* No parameters

**Returns:**

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

Get pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
   * `pose` ([`Pose`](#message-geometry_msgspose))

---

<h4 id="chassiscontroller-send_relative_pose_to_navigation">send_relative_pose_to_navigation</h4>

```python
def send_relative_pose_to_navigation(chassis_position_list: ChassisPositionList, timeout) -> ExecutionResult
```

Send multiple relative positions to navigation system, used for data replay

**Parameters:**

* `chassis_position_list` ([`ChassisPositionList`](#message-xrsdkchassispositionlist))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-reset_navigation_chunk_id">reset_navigation_chunk_id</h4>

```python
def reset_navigation_chunk_id(timeout) -> ExecutionResult
```

reset_navigation_chunk_id to 0

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="chassiscontroller-set_trajectory_coord_system_mode">set_trajectory_coord_system_mode</h4>

```python
def set_trajectory_coord_system_mode(coordinate_system_mode_param: CoordinateSystemModeParam, timeout) -> ExecutionResult
```

Set trajectory coordinate system mode, default is map coordinate system, can be set to odometry coordinate system

**Parameters:**

* `coordinate_system_mode_param` ([`CoordinateSystemModeParam`](#message-xrsdkcoordinatesystemmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h3 id="depthpoints">DepthPoints</h3>

Depth points service

<h4 id="depthpoints-get_chassis_depth_points">get_chassis_depth_points</h4>

```python
def get_chassis_depth_points(timeout) -> _sensor_msgs__.PointCloud2
```

Get chassis depth points data

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="headcamera-get_depth_image">get_depth_image</h4>

```python
def get_depth_image(timeout) -> _sensor_msgs__.CompressedImage
```

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="headcamera-get_rgb_video_stream">get_rgb_video_stream</h4>

```python
def get_rgb_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="headcamera-get_depth_video_stream">get_depth_video_stream</h4>

```python
def get_depth_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**Parameters:**

* No parameters

**Returns:**

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

Control head pose (yaw and pitch angles)

For Quanta_X2

* `upper limit: [0.87, 1.57]`
* `lower limit: [-0.52, -1.57]`

For Quanta_X1

* `upper limit: [0.9, 1.20]`
* `lower limit: [-0.06, -1.20]`

**Parameters:**

* `head_pose` ([`HeadPose`](#message-xrsdkheadpose))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="headcontroller-get_pose">get_pose</h4>

```python
def get_pose(timeout) -> HeadPose
```

Get current head pose

**Parameters:**

* No parameters

**Returns:**

* [`HeadPose`](#message-xrsdkheadpose)

---

<h4 id="headcontroller-reset">reset</h4>

```python
def reset(timeout) -> ExecutionResult
```

Reset head to center position

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="headcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get Joint state stream

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="imu">Imu</h3>

Imu service

<h4 id="imu-get_chassis_imu">get_chassis_imu</h4>

```python
def get_chassis_imu(timeout) -> _sensor_msgs__.Imu
```

Get imu data

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="leftarmcamera-get_video_stream">get_video_stream</h4>

```python
def get_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

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

* `upper limit: [5.0, 5.0, 5.0, 3.14, 3.14, 3.14, 3.14]`
* `lower limit: [-5.0, -5.0, -5.0, -3.14, -3.14, -3.14, -3.14]`

For Quanta_X1 and Desktop

* `upper limit: [5.0, 5.0, 5.0, 3.14, 3.14, 3.14, 3.14]`
* `lower limit: [-5.0, -5.0, -5.0, -3.14, -3.14, -3.14, -3.14]`

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

Get end effector pose

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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

Get end pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="liftcontroller">LiftController</h3>

Lift controller service ( quanta_x1 only)

<h4 id="liftcontroller-set_lift_position">set_lift_position</h4>

```python
def set_lift_position(lift_position: LiftPosition, timeout) -> ExecutionResult
```

Set lift position

* `upper limit: 0.78`
* `lower limit: 0.0`

**Parameters:**

* `lift_position` ([`LiftPosition`](#message-xrsdkliftposition))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="liftcontroller-get_lift_position">get_lift_position</h4>

```python
def get_lift_position(timeout) -> LiftPosition
```

Get lift position

**Parameters:**

* No parameters

**Returns:**

* [`LiftPosition`](#message-xrsdkliftposition)

---

<h4 id="liftcontroller-get_joint_states_stream">get_joint_states_stream</h4>

```python
def get_joint_states_stream(timeout) -> Iterator[_sensor_msgs__.JointState]
```

Get Joint state stream

**Parameters:**

* No parameters

**Returns:**

* [`JointState`](#message-sensor_msgsjointstate)
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
orientation: [qx, qy, qz, qw] quaternion, range: [-3.14, 3.14]

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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
orientation: [qx, qy, qz, qw] quaternion, range: [-3.14, 3.14]

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
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="navigation">Navigation</h3>

<h4 id="navigation-start_mapping">start_mapping</h4>

```python
def start_mapping(timeout) -> ExecutionResult
```

Start mapping

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-stop_mapping">stop_mapping</h4>

```python
def stop_mapping(save_map_param: SaveMapParam, timeout) -> ExecutionResult
```

Stop and save mapping

**Parameters:**

* `save_map_param` ([`SaveMapParam`](#message-xrsdksavemapparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-set_navigation_mode">set_navigation_mode</h4>

```python
def set_navigation_mode(navigation_mode_param: NavigationModeParam, timeout) -> ExecutionResult
```

Set navigation mode (enable/disable built-in navigation algorithm)

**Parameters:**

* `navigation_mode_param` ([`NavigationModeParam`](#message-xrsdknavigationmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-start_localization">start_localization</h4>

```python
def start_localization(start_localization_param: StartLocalizationParam, timeout) -> ExecutionResult
```

Start localization

**Parameters:**

* `start_localization_param` ([`StartLocalizationParam`](#message-xrsdkstartlocalizationparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-stop_localization">stop_localization</h4>

```python
def stop_localization(timeout) -> ExecutionResult
```

Stop localization

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-cancel_navigation">cancel_navigation</h4>

```python
def cancel_navigation(timeout) -> ExecutionResult
```

Cancel current navigation task

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="navigation-load_map">load_map</h4>

```python
def load_map(messages: Iterable[UploadRequest], timeout) -> ExecutionResult
```

Load a map package (tar.gz of a directory) into the robot.

**Parameters:**

* `upload_request` ([`UploadRequest`](#message-xrsdkuploadrequest))

**Returns:**

* `Iterator[[`ExecutionResult`](#message-xrsdkexecutionresult)]`: Stream of ExecutionResult

---

<h4 id="navigation-export_map">export_map</h4>

```python
def export_map(download_request: DownloadRequest, timeout) -> Iterator[DownloadResponse]
```

Export a map from the robot as a tar.gz stream of a directory.

**Parameters:**

* `download_request` ([`DownloadRequest`](#message-xrsdkdownloadrequest))

**Returns:**

* `Iterator[[`DownloadResponse`](#message-xrsdkdownloadresponse)]`: Stream of DownloadResponse

---

<h3 id="radarservice">RadarService</h3>

Radar service

<h4 id="radarservice-get_laser_scan">get_laser_scan</h4>

```python
def get_laser_scan(timeout) -> _sensor_msgs__.LaserScan
```

Get radar scan data

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

---

<h4 id="rightarmcamera-get_video_stream">get_video_stream</h4>

```python
def get_video_stream(timeout) -> Iterator[_sensor_msgs__.CompressedImage]
```

**Parameters:**

* No parameters

**Returns:**

* [`CompressedImage`](#message-sensor_msgscompressedimage)
   * `header` ([`Header`](#message-std_msgsheader))
   * `format` (`string`)
   * `data` (`bytes`)

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

Get end effector pose

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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

Get end pose stream

**Parameters:**

* No parameters

**Returns:**

* [`PoseStamped`](#message-geometry_msgsposestamped)
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
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
   * `header` ([`Header`](#message-std_msgsheader))
   * `name` (List[`string`])
   * `position` (List[`double`])
   * `velocity` (List[`double`])
   * `effort` (List[`double`])

---

<h3 id="robotcontrol">RobotControl</h3>

<h4 id="robotcontrol-set_manipulator_control_mode">set_manipulator_control_mode</h4>

```python
def set_manipulator_control_mode(manipulator_control_mode_param: ManipulatorControlModeParam, timeout) -> ExecutionResult
```

Set control mode for Manipulator (Arm and Waist): joint positions control or end pose control

**Parameters:**

* `manipulator_control_mode_param` ([`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam))

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="robotcontrol-get_manipulator_control_mode">get_manipulator_control_mode</h4>

```python
def get_manipulator_control_mode(timeout) -> ManipulatorControlModeParam
```

Get current control mode for Manipulator

**Parameters:**

* No parameters

**Returns:**

* [`ManipulatorControlModeParam`](#message-xrsdkmanipulatorcontrolmodeparam)

---

<h4 id="robotcontrol-homing">homing</h4>

```python
def homing(timeout) -> ExecutionResult
```

Homing the robot, all joints will be homing to the home position

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="robotcontrol-emergency_stop">emergency_stop</h4>

```python
def emergency_stop(timeout) -> ExecutionResult
```

Emergency stop, call carefully and only when necessary

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

---

<h4 id="robotcontrol-recover_emergency_stop">recover_emergency_stop</h4>

```python
def recover_emergency_stop(timeout) -> ExecutionResult
```

Recover from emergency stop, only when emergency stop is called

**Parameters:**

* No parameters

**Returns:**

* [`ExecutionResult`](#message-xrsdkexecutionresult)

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

<h3 id="tof">Tof</h3>

<h4 id="tof-get_chassis_tof1">get_chassis_tof1</h4>

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

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

**Parameters:**

* No parameters

**Returns:**

* [`Range`](#message-sensor_msgsrange)
   * `header` ([`Header`](#message-std_msgsheader))
   * `radiation_type` (`uint32`)
   * `field_of_view` (`float`)
   * `min_range` (`float`)
   * `max_range` (`float`)
   * `range` (`float`)
   * `variance` (`float`)

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
| `header` | [`Header`](#message-std_msgsheader) |  |
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

<a id="message-geometry_msgstwist"></a>
#### Twist

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `linear` | [`Vector3`](#message-geometry_msgsvector3) |  |
| `angular` | [`Vector3`](#message-geometry_msgsvector3) |  |

---

<a id="message-geometry_msgstwistwithcovariance"></a>
#### TwistWithCovariance

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `twist` | [`Twist`](#message-geometry_msgstwist) |  |
| `covariance` | List[`double`] |  |

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
| `header` | [`Header`](#message-std_msgsheader) |  |
| `wrench` | [`Wrench`](#message-geometry_msgswrench) |  |

---

<a id="message-halheadpantiltcontrol"></a>
#### HeadPanTiltControl

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `yaw_position` | `float` |  |
| `yaw_velocity` | `float` |  |
| `pitch_position` | `float` |  |
| `pitch_velocity` | `float` |  |

---

<a id="message-nav_msgsodometry"></a>
#### Odometry

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `child_frame_id` | `string` |  |
| `pose` | [`PoseWithCovariance`](#message-geometry_msgsposewithcovariance) |  |
| `twist` | [`TwistWithCovariance`](#message-geometry_msgstwistwithcovariance) |  |

---

<a id="message-sensor_msgscompressedimage"></a>
#### CompressedImage

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `format` | `string` |  |
| `data` | `bytes` |  |

---

<a id="message-sensor_msgsimu"></a>
#### Imu

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
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

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | [`Header`](#message-std_msgsheader) |  |
| `name` | List[`string`] |  |
| `position` | List[`double`] |  |
| `velocity` | List[`double`] |  |
| `effort` | List[`double`] |  |

---

<a id="message-sensor_msgslaserscan"></a>
#### LaserScan

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
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

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
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

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `name` | `string` |  |
| `offset` | `uint32` |  |
| `datatype` | `uint32` |  |
| `count` | `uint32` |  |

---

<a id="message-sensor_msgsrange"></a>
#### Range

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
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

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `data` | `float` |  |

---

<a id="message-std_msgsfloat64multiarray"></a>
#### Float64MultiArray

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `layout` | [`MultiArrayLayout`](#message-std_msgsmultiarraylayout) |  |
| `data` | List[`double`] |  |

---

<a id="message-std_msgsheader"></a>
#### Header

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | [`Time`](#message-builtin_interfacestime) |  |
| `frame_id` | `string` |  |

---

<a id="message-std_msgsmultiarraydimension"></a>
#### MultiArrayDimension

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `label` | `string` |  |
| `size` | `uint32` |  |
| `stride` | `uint32` |  |

---

<a id="message-std_msgsmultiarraylayout"></a>
#### MultiArrayLayout

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `dim` | List[[`MultiArrayDimension`](#message-std_msgsmultiarraydimension)] |  |
| `data_offset` | `uint32` |  |

---

<a id="message-std_msgsstring"></a>
#### String

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `data` | `string` |  |

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

<a id="message-xrsdkchassiscontrolmodeparam"></a>
#### ChassisControlModeParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | [`ChassisControlMode`](#enum-xrsdkchassiscontrolmode) |  |

---

<a id="message-xrsdkchassisposition"></a>
#### ChassisPosition

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `x` | `double` | position in x direction in m |
| `y` | `double` | position in y direction in m |
| `yaw` | `double` | yaw angle in rad |

---

<a id="message-xrsdkchassispositionlist"></a>
#### ChassisPositionList

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | `int32` (optional) | chunk_id is used to identify the chunk of the positions. optional, if not set, use self-incremented id.<br>navigation system will use chunk_id 0 as the start position. |
| `positions` | List[[`ChassisPosition`](#message-xrsdkchassisposition)] |  |

---

<a id="message-xrsdkchassisvelocity"></a>
#### ChassisVelocity

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `vel_x` | `double` | velocity in x direction in m/s |
| `vel_y` | `double` | not used, set to 0 |
| `vel_yaw` | `double` | yaw velocity in rad/s |

---

<a id="message-xrsdkcoordinatesystemmodeparam"></a>
#### CoordinateSystemModeParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `coordinate_system_mode` | [`CoordinateSystemMode`](#enum-xrsdkcoordinatesystemmode) |  |

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

<a id="message-xrsdkgripperposition"></a>
#### GripperPosition

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `position` | `float` |  |

---

<a id="message-xrsdkheadpose"></a>
#### HeadPose

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `pitch` | `float` | Pitch angle in degrees, positive values tilt up |
| `yaw` | `float` | Yaw angle in degrees, positive values turn right |

---

<a id="message-xrsdkjointpositions"></a>
#### JointPositions

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `positions` | List[`double`] |  |

---

<a id="message-xrsdkliftposition"></a>
#### LiftPosition

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `position` | `float` | lift position in meters, positive value means lift up, negative value means lift down |

---

<a id="message-xrsdkmanipulatorcontrolmodeparam"></a>
#### ManipulatorControlModeParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | [`ManipulatorControlMode`](#enum-xrsdkmanipulatorcontrolmode) |  |

---

<a id="message-xrsdkmodeltyperesult"></a>
#### ModelTypeResult

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `model_type` | [`RobotModelType`](#enum-xrsdkrobotmodeltype) |  |

---

<a id="message-xrsdknavigationmodeparam"></a>
#### NavigationModeParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | [`NavigationMode`](#enum-xrsdknavigationmode) |  |

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

<a id="message-xrsdksavemapparam"></a>
#### SaveMapParam

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `map_name` | `string` |  |

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

<a id="message-xrsdktactilesensordata"></a>
#### TactileSensorData

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `stamp` | [`Time`](#message-builtin_interfacestime) |  |
| `sensor_names` | List[`string`] |  |
| `frame_ids` | List[`string`] |  |
| `normal_forces` | List[`float`] |  |
| `tangential_forces` | List[`float`] |  |
| `directions` | List[`int32`] |  |
| `capacitances` | List[`uint32`] |  |
| `error_codes` | List[`uint32`] |  |

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

<a id="enum-sensor_msgspointfieldconstants"></a>
#### PointFieldConstants

| Value | Description |
|-------|-------------|
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

| Value | Description |
|-------|-------------|
| `GLOBAL` (0) | Global absolute position control relative to map coordinate system. Map coordinate system varies by scenario, rarely used in practice. |
| `RELATIVE` (1) | Relative position control (RECOMMENDED!!!), relative to a virtual zero point that must be set via API. |
| `VELOCITY` (2) | Direct velocity control, requires disabling chassis position planner in advance. |

---

<a id="enum-xrsdkcoordinatesystemmode"></a>
#### CoordinateSystemMode

map coordinate system is used for map based navigation. need to set this mode before navigation.

| Value | Description |
|-------|-------------|
| `COORDINATE_SYSTEM_MODE_MAP` (0) | map coordinate system is used for map based navigation. need to set this mode before navigation. |
| `COORDINATE_SYSTEM_MODE_ODOMETRY` (1) | odometry coordinate system is used for data replay when map is not built. need to set this mode before data replay. |

---

<a id="enum-xrsdkmanipulatorcontrolmode"></a>
#### ManipulatorControlMode

| Value | Description |
|-------|-------------|
| `MANIPULATOR_END_POSE` (0) | End pose control mode for manipulator(Arm and Waist) |
| `MANIPULATOR_JOINT_POSITIONS` (1) | Joint positions control mode for manipulator(Arm and Waist) |

---

<a id="enum-xrsdknavigationmode"></a>
#### NavigationMode

| Value | Description |
|-------|-------------|
| `BUILT_IN_NAVIGATION` (0) | Enable built-in navigation |
| `USER_CUSTOM_NAVIGATION` (1) | Disable built-in navigation |

---

<a id="enum-xrsdkrobotmodeltype"></a>
#### RobotModelType

| Value | Description |
|-------|-------------|
| `CX001` (0) |  |
| `CX002` (1) |  |
| `EX001` (2) |  |
| `DESKTOP` (3) |  |
| `EX001_MASTER` (4) |  |
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
