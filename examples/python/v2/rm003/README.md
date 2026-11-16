# 三指外骨骼与 VR 读取示例

安装与设备软件配套的 `x2robot-sdk` wheel，激活对应的 Python 环境后，
在本目录运行以下命令。将 `192.168.1.10:55051` 替换为 MiniPC 的实际地址和端口。
四个脚本也可以复制到同一工作目录后直接运行。

这些示例使用原生 `x2robot_sdk`，只读取数据，不需要进入控制模式或发送控制心跳。
每次读取和流中的每一项均为 `SdkResult`：先检查 `is_success`，成功后访问 `data`，
失败时查看 `error` 和 `request_id`。

## 安装与连接自检

```bash
python check_sdk.py 192.168.1.10:55051
```

脚本显示 SDK 版本与导入路径，并检查关节、相机和 VR 的 12 个客户端方法。
它根据接口能力检查设备，不依赖设备的内部型号字符串。
自检成功不代表设备已提供真实数据，还需分别运行下列读取示例。

## 左右外骨骼关节

```bash
python read_exoskeleton.py 192.168.1.10:55051 --side left
python read_exoskeleton.py 192.168.1.10:55051 --side right
python read_exoskeleton.py 192.168.1.10:55051 --side left --stream --count 20
python read_exoskeleton.py 192.168.1.10:55051 --side right --stream --count 20
```

`robot.exo_hand.get_left_joint_state(timeout=1)` 返回单次关节数据；右侧将 `left`
换为 `right`，连续读取使用 `get_left_joint_state_stream(timeout=5)` 或右侧同名接口。
数据包含时间戳、`frame_id`、`name`、`position`、`velocity`、`effort`。
SDK 保留设备提供的单位、顺序和数组长度，`velocity`、`effort` 可以为空。
脚本的关节流总时限为 5 秒。

## 左右外骨骼相机

```bash
python read_exoskeleton_camera.py 192.168.1.10:55051 \
  --side left --count 5 --timeout 10 --output frames/left
python read_exoskeleton_camera.py 192.168.1.10:55051 \
  --side right --count 5 --timeout 10 --output frames/right
```

左右接口分别为 `get_left_camera_stream()` 和 `get_right_camera_stream()`。
每帧保留 `header`、`format` 和 `data`，其中 `data` 是已编码的压缩图像字节。
示例直接保存 JPEG/PNG，并为每帧生成记录时间戳、`frame_id`、编码格式和字节数的 JSON。
保存压缩图像不需要安装 OpenCV 或 Pillow；显示或处理图像时再选择对应的解码器。

## VR world 与 related

| 读取部位 | world 接口 | related 接口 |
| --- | --- | --- |
| 左手 | `get_left_arm_pose_stream()` | `get_left_arm_pose_related_stream()` |
| 右手 | `get_right_arm_pose_stream()` | `get_right_arm_pose_related_stream()` |
| 头部 | `get_head_pose_stream()` | `get_head_pose_related_stream()` |

`world` 是 VR 跟踪系统世界参考系中的绝对位姿，该参考系不自动等同于机器人底座或地图。
`related` 是相对于设备定义的参考姿态或头手关系处理后的相对位姿；具体参考系和重置行为
取决于设备标定与工作模式，不能一概当作“相对头部”或用 world 的位置相减代替。
SDK 分别读取两类数据，不自动转换，也不会在 related 无数据时使用 world 替代。

```bash
python read_vr.py 192.168.1.10:55051 \
  --part head --frame world --count 10 --timeout 10
python read_vr.py 192.168.1.10:55051 \
  --part left --frame world --count 10 --timeout 10
python read_vr.py 192.168.1.10:55051 \
  --part right --frame world --count 10 --timeout 10
python read_vr.py 192.168.1.10:55051 \
  --part head --frame related --count 10 --timeout 15
python read_vr.py 192.168.1.10:55051 \
  --part left --frame related --count 10 --timeout 15
python read_vr.py 192.168.1.10:55051 \
  --part right --frame related --count 10 --timeout 15
```

related 数据可能需要完成相应的设备操作后才可读取，具体以设备操作说明为准。
打开 SDK 流接口本身不会触发设备提供数据；world 有数据不保证 related 同时有数据。
脚本打印原始时间戳、`frame_id`、位置 xyz 和四元数 xyzw；六路位姿独立读取，
联合使用时应按时间戳匹配。

## 超时与退出

Python `timeout` 的单位是秒，对流接口表示整条 RPC 的总时限，不是每帧等待时长。
VR 和相机示例读取指定数量后主动取消流，并在错误或提前结束时报告失败。
所有流均在 `finally` 中取消，连接由 `with` 管理并关闭。
