"""
完全自定义数据采集示例

这个脚本展示了如何完全手动控制数据采集，不使用任何封装层。
用户直接指定要采集的数据类型，完全控制采集过程。

主要特点:
- 完全手动控制采集的数据类型
- 不依赖任何配置类
- 直接调用机器人接口
- 自定义数据格式和存储逻辑
- 灵活的采集控制

使用方法:
1. 基本使用：
   python custom_data_collection_example.py

2. 指定配置：
   python custom_data_collection_example.py --config minimal    # 只采集关节状态
   python custom_data_collection_example.py --config vision     # 只采集视觉数据
   python custom_data_collection_example.py --config full       # 采集所有数据

3. 自定义数据源：
   python custom_data_collection_example.py --config "joint_states,head_rgb,left_arm_rgb"

4. 指定输出目录：
   python custom_data_collection_example.py --output-dir ./my_data

可用数据源:
- joint_states: 关节状态
- head_rgb: 头部RGB相机
- left_arm_rgb: 左臂RGB相机
- right_arm_rgb: 右臂RGB相机
- head_depth: 头部深度相机
- imu: IMU传感器
- odometry: 里程计
- left_arm_end_pose: 左臂末端位姿
- right_arm_end_pose: 右臂末端位姿
"""

import time
import json
import threading
import signal
import sys
from pathlib import Path
from typing import Dict, List, Any, Optional, Set
from dataclasses import dataclass
from enum import Enum
import numpy as np
from PIL import Image
import io

from x2robot import connect


@dataclass
class CustomDataFrame:
    """自定义数据帧结构"""
    timestamp: float
    frame_id: int

    # 关节状态数据
    joint_positions: Optional[np.ndarray] = None
    joint_velocities: Optional[np.ndarray] = None
    joint_efforts: Optional[np.ndarray] = None

    # 图像数据
    images: Dict[str, Image.Image] = None

    # 传感器数据
    imu_data: Optional[Dict[str, Any]] = None
    odometry: Optional[Dict[str, Any]] = None
    left_arm_end_pose: Optional[Dict[str, Any]] = None
    right_arm_end_pose: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式用于JSON序列化"""
        data = {
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
        }

        if self.joint_positions is not None:
            data["joint_positions"] = self.joint_positions.tolist()

        if self.joint_velocities is not None:
            data["joint_velocities"] = self.joint_velocities.tolist()

        if self.joint_efforts is not None:
            data["joint_efforts"] = self.joint_efforts.tolist()

        if self.images:
            data["images"] = {}
            for cam_name, img in self.images.items():
                data["images"][cam_name] = f"frame_{self.frame_id:04d}_{cam_name}.jpg"

        if self.imu_data:
            data["imu"] = self.imu_data

        if self.odometry:
            data["odometry"] = self.odometry

        if self.left_arm_end_pose:
            data["left_arm_end_pose"] = self.left_arm_end_pose

        if self.right_arm_end_pose:
            data["right_arm_end_pose"] = self.right_arm_end_pose

        return data


class DataSource(Enum):
    """数据源枚举"""
    JOINT_STATES = "joint_states"
    HEAD_RGB_CAMERA = "head_rgb"
    LEFT_ARM_RGB_CAMERA = "left_arm_rgb"
    RIGHT_ARM_RGB_CAMERA = "right_arm_rgb"
    HEAD_DEPTH_CAMERA = "head_depth"
    CHASSIS_IMU = "imu"
    ODOMETRY = "odometry"
    LEFT_ARM_END_POSE = "left_arm_end_pose"
    RIGHT_ARM_END_POSE = "right_arm_end_pose"


class CustomDataCollector:
    """完全自定义数据采集器"""

    def __init__(self, robot, output_dir: str = "./custom_collected_data",
                 data_sources: Set[DataSource] = None, target_hz: float = 30.0):
        """
        初始化自定义数据采集器

        Args:
            robot: 机器人实例
            output_dir: 输出目录
            data_sources: 要采集的数据源集合，如果为None则采集所有数据
            target_hz: 目标采集频率
        """
        self.robot = robot
        self.output_dir = Path(output_dir)
        self.data_sources = data_sources or set(DataSource)
        self.target_hz = target_hz
        self.target_period = 1.0 / target_hz

        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # 采集状态
        self.is_collecting = False
        self.is_recording = False
        self.threads = []
        self.current_episode_data = []

        # 关节名称映射
        self.joint_names = None
        self.joint_name_mapping = None

        # 数据缓冲区 - 动态创建
        self.data_buffer = {}
        self._init_data_buffer()

        # 线程锁
        self.buffer_lock = threading.Lock()

    def _init_data_buffer(self):
        """根据数据源初始化数据缓冲区"""
        # 关节状态
        if DataSource.JOINT_STATES in self.data_sources:
            self.data_buffer['joint_states'] = []

        # 图像数据
        self.data_buffer['images'] = {}
        camera_sources = [DataSource.HEAD_RGB_CAMERA, DataSource.LEFT_ARM_RGB_CAMERA,
                         DataSource.RIGHT_ARM_RGB_CAMERA, DataSource.HEAD_DEPTH_CAMERA]
        for cam_source in camera_sources:
            if cam_source in self.data_sources:
                self.data_buffer['images'][cam_source.value] = []

        # 传感器数据
        sensor_sources = [DataSource.CHASSIS_IMU, DataSource.ODOMETRY,
                         DataSource.LEFT_ARM_END_POSE, DataSource.RIGHT_ARM_END_POSE]
        for sensor_source in sensor_sources:
            if sensor_source in self.data_sources:
                self.data_buffer[sensor_source.value] = []

    def _extract_timestamp_from_header(self, msg):
        """从消息的header中提取时间戳"""
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            return float(sec) + float(nanosec) / 1e9
        else:
            return time.time()

    def _collect_joint_states(self):
        """采集关节状态数据"""
        print("启动关节状态采集...")
        try:
            stream = self.robot.state.get_all_joint_states_stream(timeout=None)

            for state_msg in stream:
                if not self.is_collecting:
                    break

                try:
                    # 建立关节名称映射
                    if self.joint_name_mapping is None and hasattr(state_msg, 'name') and state_msg.name:
                        self.joint_names = list(state_msg.name)
                        self.joint_name_mapping = {name: idx for idx, name in enumerate(state_msg.name)}
                        print(f"关节名称: {self.joint_names}")

                    # 提取关节数据
                    joint_positions = np.array(state_msg.position, dtype=np.float32)
                    joint_velocities = np.array(state_msg.velocity, dtype=np.float32) if hasattr(state_msg, 'velocity') and state_msg.velocity else None
                    joint_efforts = np.array(state_msg.effort, dtype=np.float32) if hasattr(state_msg, 'effort') and state_msg.effort else None

                    timestamp = self._extract_timestamp_from_header(state_msg)

                    with self.buffer_lock:
                        self.data_buffer['joint_states'].append({
                            'timestamp': timestamp,
                            'positions': joint_positions,
                            'velocities': joint_velocities,
                            'efforts': joint_efforts
                        })

                    # 简单的限流，避免数据积累过快
                    time.sleep(0.01)  # 100Hz采集

                except Exception as e:
                    print(f"关节状态处理错误: {e}")
                    continue

        except Exception as e:
            print(f"关节状态流错误: {e}")

    def _collect_camera_stream(self, camera_name, stream_func):
        """采集相机流"""
        print(f"启动 {camera_name} 采集...")

        try:
            stream = stream_func(timeout=None)

            for frame_msg in stream:
                if not self.is_collecting:
                    break

                try:
                    if not frame_msg or not frame_msg.data:
                        continue

                    # 解码图像
                    img_bytes = bytes(frame_msg.data)
                    img = Image.open(io.BytesIO(img_bytes))

                    if img.mode != 'RGB':
                        img = img.convert('RGB')

                    timestamp = self._extract_timestamp_from_header(frame_msg)

                    with self.buffer_lock:
                        if camera_name not in self.data_buffer['images']:
                            self.data_buffer['images'][camera_name] = []
                        self.data_buffer['images'][camera_name].append({
                            'timestamp': timestamp,
                            'image': img
                        })

                    # 限流
                    time.sleep(0.033)  # ~30Hz

                except Exception as e:
                    print(f"{camera_name} 处理错误: {e}")
                    continue

        except Exception as e:
            print(f"{camera_name} 流错误: {e}")

    def _collect_imu(self):
        """采集IMU数据"""
        print("启动IMU采集...")
        try:
            stream = self.robot.imu.get_chassis_imu_stream(timeout=None)

            for imu_msg in stream:
                if not self.is_collecting:
                    break

                try:
                    # 检查消息是否有效
                    if not imu_msg:
                        continue

                    imu_data = {
                        'orientation': [
                            float(imu_msg.orientation.x) if imu_msg.orientation else 0,
                            float(imu_msg.orientation.y) if imu_msg.orientation else 0,
                            float(imu_msg.orientation.z) if imu_msg.orientation else 0,
                            float(imu_msg.orientation.w) if imu_msg.orientation else 1
                        ],
                        'angular_velocity': [
                            float(imu_msg.angular_velocity.x) if imu_msg.angular_velocity else 0,
                            float(imu_msg.angular_velocity.y) if imu_msg.angular_velocity else 0,
                            float(imu_msg.angular_velocity.z) if imu_msg.angular_velocity else 0
                        ],
                        'linear_acceleration': [
                            float(imu_msg.linear_acceleration.x) if imu_msg.linear_acceleration else 0,
                            float(imu_msg.linear_acceleration.y) if imu_msg.linear_acceleration else 0,
                            float(imu_msg.linear_acceleration.z) if imu_msg.linear_acceleration else 0
                        ]
                    }

                    timestamp = self._extract_timestamp_from_header(imu_msg)

                    with self.buffer_lock:
                        self.data_buffer['imu'].append({
                            'timestamp': timestamp,
                            'data': imu_data
                        })

                    time.sleep(0.1)  # 10Hz

                except Exception as e:
                    print(f"IMU处理错误: {e}")
                    continue

        except Exception as e:
            print(f"IMU流错误: {e}")

    def _collect_odometry(self):
        """采集里程计数据"""
        print("启动里程计采集...")
        try:
            stream = self.robot.chassis.get_odometry_stream(timeout=None)

            for odom_msg in stream:
                if not self.is_collecting:
                    break

                try:
                    # 检查消息是否有效
                    if not odom_msg:
                        continue

                    odometry_data = {
                        'pose': {
                            'position': {
                                'x': odom_msg.pose.pose.position.x if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.position else 0,
                                'y': odom_msg.pose.pose.position.y if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.position else 0,
                                'z': odom_msg.pose.pose.position.z if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.position else 0,
                            },
                            'orientation': {
                                'x': odom_msg.pose.pose.orientation.x if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.orientation else 0,
                                'y': odom_msg.pose.pose.orientation.y if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.orientation else 0,
                                'z': odom_msg.pose.pose.orientation.z if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.orientation else 0,
                                'w': odom_msg.pose.pose.orientation.w if odom_msg.pose and odom_msg.pose.pose and odom_msg.pose.pose.orientation else 1,
                            }
                        },
                        'twist': {
                            'linear': {
                                'x': odom_msg.twist.twist.linear.x if odom_msg.twist and odom_msg.twist.twist and odom_msg.twist.twist.linear else 0,
                                'y': odom_msg.twist.twist.linear.y if odom_msg.twist and odom_msg.twist.twist and odom_msg.twist.twist.linear else 0,
                                'z': odom_msg.twist.twist.linear.z if odom_msg.twist and odom_msg.twist.twist and odom_msg.twist.twist.linear else 0,
                            },
                            'angular': {
                                'x': odom_msg.twist.twist.angular.x if odom_msg.twist and odom_msg.twist.twist and odom_msg.twist.twist.angular else 0,
                                'y': odom_msg.twist.twist.angular.y if odom_msg.twist and odom_msg.twist.twist and odom_msg.twist.twist.angular else 0,
                                'z': odom_msg.twist.twist.angular.z if odom_msg.twist and odom_msg.twist.twist and odom_msg.twist.twist.angular else 0,
                            }
                        }
                    }

                    timestamp = self._extract_timestamp_from_header(odom_msg)

                    with self.buffer_lock:
                        self.data_buffer['odometry'].append({
                            'timestamp': timestamp,
                            'data': odometry_data
                        })

                    time.sleep(0.1)  # 10Hz

                except Exception as e:
                    print(f"里程计处理错误: {e}")
                    continue

        except Exception as e:
            print(f"里程计流错误: {e}")

    def _collect_arm_end_pose(self, arm_name):
        """采集手臂末端位姿"""
        print(f"启动{arm_name}末端位姿采集...")

        # 手臂对象名称映射
        arm_attr_name = f"{arm_name}_arm"
        stream_func = getattr(self.robot, arm_attr_name).get_end_pose_stream
        buffer_key = f"{arm_name}_arm_end_pose"

        try:
            stream = stream_func(timeout=None)

            for pose_msg in stream:
                if not self.is_collecting:
                    break

                try:
                    # 检查消息是否有效
                    if not pose_msg:
                        continue

                    pose_data = {
                        'position': {
                            'x': pose_msg.pose.position.x if pose_msg.pose and pose_msg.pose.position else 0,
                            'y': pose_msg.pose.position.y if pose_msg.pose and pose_msg.pose.position else 0,
                            'z': pose_msg.pose.position.z if pose_msg.pose and pose_msg.pose.position else 0,
                        },
                        'orientation': {
                            'x': pose_msg.pose.orientation.x if pose_msg.pose and pose_msg.pose.orientation else 0,
                            'y': pose_msg.pose.orientation.y if pose_msg.pose and pose_msg.pose.orientation else 0,
                            'z': pose_msg.pose.orientation.z if pose_msg.pose and pose_msg.pose.orientation else 0,
                            'w': pose_msg.pose.orientation.w if pose_msg.pose and pose_msg.pose.orientation else 1,
                        }
                    }

                    timestamp = self._extract_timestamp_from_header(pose_msg)

                    with self.buffer_lock:
                        self.data_buffer[buffer_key].append({
                            'timestamp': timestamp,
                            'data': pose_data
                        })

                    time.sleep(0.033)  # ~30Hz

                except Exception as e:
                    print(f"{arm_name}末端位姿处理错误: {e}")
                    continue

        except Exception as e:
            print(f"{arm_name}末端位姿流错误: {e}")

    def start_collecting(self):
        """启动数据采集"""
        if self.is_collecting:
            print("已在采集中")
            return

        self.is_collecting = True

        # 清空缓冲区
        with self.buffer_lock:
            for key in self.data_buffer:
                if isinstance(self.data_buffer[key], list):
                    self.data_buffer[key].clear()
                elif isinstance(self.data_buffer[key], dict):
                    for sub_key in self.data_buffer[key]:
                        self.data_buffer[key][sub_key].clear()

        # 启动采集线程
        threads = []

        # 关节状态
        if DataSource.JOINT_STATES in self.data_sources:
            threads.append(threading.Thread(target=self._collect_joint_states, daemon=True))

        # 相机流
        camera_configs = [
            (DataSource.HEAD_RGB_CAMERA, 'head_rgb', self.robot.head_camera.get_rgb_video_stream),
            (DataSource.LEFT_ARM_RGB_CAMERA, 'left_arm_rgb', self.robot.left_arm_camera.get_video_stream),
            (DataSource.RIGHT_ARM_RGB_CAMERA, 'right_arm_rgb', self.robot.right_arm_camera.get_video_stream),
            (DataSource.HEAD_DEPTH_CAMERA, 'head_depth', self.robot.head_camera.get_depth_video_stream),
        ]

        for data_source, cam_name, stream_func in camera_configs:
            if data_source in self.data_sources:
                threads.append(threading.Thread(
                    target=self._collect_camera_stream,
                    args=(cam_name, stream_func),
                    daemon=True
                ))

        # 传感器
        if DataSource.CHASSIS_IMU in self.data_sources:
            threads.append(threading.Thread(target=self._collect_imu, daemon=True))

        if DataSource.ODOMETRY in self.data_sources:
            threads.append(threading.Thread(target=self._collect_odometry, daemon=True))

        # 末端位姿
        if DataSource.LEFT_ARM_END_POSE in self.data_sources:
            threads.append(threading.Thread(
                target=self._collect_arm_end_pose,
                args=('left',),
                daemon=True
            ))

        if DataSource.RIGHT_ARM_END_POSE in self.data_sources:
            threads.append(threading.Thread(
                target=self._collect_arm_end_pose,
                args=('right',),
                daemon=True
            ))

        # 启动所有线程
        for t in threads:
            t.start()
            self.threads.append(t)

        # 显示启用的数据源
        enabled_sources = [src.value for src in self.data_sources]
        print(f"✓ 完全自定义数据采集已启动 (目标频率: {self.target_hz} Hz)")
        print(f"  输出目录: {self.output_dir}")
        print(f"  启用的数据源: {enabled_sources}")

    def stop_collecting(self):
        """停止数据采集"""
        self.is_collecting = False

        # 等待线程结束
        for t in self.threads:
            t.join(timeout=2.0)

        self.threads.clear()
        print("✓ 自定义数据采集已停止")

    def start_recording(self, task: str = "custom_task"):
        """开始录制"""
        if not self.is_collecting:
            raise RuntimeError("请先调用 start_collecting()")

        self.is_recording = True
        self.current_episode_data = []
        self.recording_start_time = time.time()
        self.frame_id = 0

        print(f"✓ 开始录制任务: {task}")

    def record_frame(self):
        """手动记录一帧数据"""
        if not self.is_recording:
            return None

        # 获取当前时间戳
        current_time = time.time()

        # 从缓冲区获取最近的数据
        with self.buffer_lock:
            frame_data = CustomDataFrame(
                timestamp=current_time,
                frame_id=self.frame_id
            )

            # 获取关节状态（最近的）
            if DataSource.JOINT_STATES in self.data_sources and self.data_buffer['joint_states']:
                joint_data = self.data_buffer['joint_states'][-1]  # 最新的关节状态
                frame_data.joint_positions = joint_data['positions']
                frame_data.joint_velocities = joint_data['velocities']
                frame_data.joint_efforts = joint_data['efforts']

            # 获取图像（最近的）
            for cam_name, cam_buffer in self.data_buffer['images'].items():
                if cam_buffer:
                    frame_data.images = frame_data.images or {}
                    frame_data.images[cam_name] = cam_buffer[-1]['image']  # 最新的图像

            # 获取传感器数据（最近的）
            if DataSource.CHASSIS_IMU in self.data_sources and self.data_buffer.get('imu'):
                frame_data.imu_data = self.data_buffer['imu'][-1]['data']

            if DataSource.ODOMETRY in self.data_sources and self.data_buffer.get('odometry'):
                frame_data.odometry = self.data_buffer['odometry'][-1]['data']

            if DataSource.LEFT_ARM_END_POSE in self.data_sources and self.data_buffer.get('left_arm_end_pose'):
                frame_data.left_arm_end_pose = self.data_buffer['left_arm_end_pose'][-1]['data']

            if DataSource.RIGHT_ARM_END_POSE in self.data_sources and self.data_buffer.get('right_arm_end_pose'):
                frame_data.right_arm_end_pose = self.data_buffer['right_arm_end_pose'][-1]['data']

        self.current_episode_data.append(frame_data)
        self.frame_id += 1

        return frame_data

    def stop_recording(self, task: str = "custom_task") -> Dict[str, Any]:
        """停止录制并保存数据"""
        if not self.is_recording:
            return None

        self.is_recording = False
        recording_duration = time.time() - self.recording_start_time

        # 创建episode目录
        episode_id = len(list(self.output_dir.glob("episode_*")))
        episode_dir = self.output_dir / f"episode_{episode_id:04d}"
        episode_dir.mkdir(exist_ok=True)

        # 准备episode数据
        episode_data = {
            "episode_id": episode_id,
            "task": task,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration": recording_duration,
            "num_frames": len(self.current_episode_data),
            "joint_names": self.joint_names or [],
            "frames": []
        }

        # 保存每一帧
        for frame in self.current_episode_data:
            frame_dict = frame.to_dict()

            # 保存图像文件
            if frame.images:
                for cam_name, img in frame.images.items():
                    img_filename = f"frame_{frame.frame_id:04d}_{cam_name}.jpg"
                    img_path = episode_dir / img_filename
                    img.save(img_path, 'JPEG', quality=95)

            episode_data["frames"].append(frame_dict)

        # 保存JSON文件
        json_path = episode_dir / "episode.json"
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(episode_data, f, indent=2, ensure_ascii=False)

        print(f"✓ Episode {episode_id} 已保存")
        print(f"  帧数: {len(self.current_episode_data)}")
        print(f"  时长: {recording_duration:.2f}s")
        print(f"  保存路径: {episode_dir}")

        # 清空当前episode数据
        self.current_episode_data = []

        return {
            "episode_id": episode_id,
            "episode_dir": str(episode_dir),
            "num_frames": len(episode_data["frames"]),
            "duration": recording_duration
        }


def create_minimal_data_sources() -> Set[DataSource]:
    """创建最小化数据采集配置（只采集关节状态）"""
    return {DataSource.JOINT_STATES}


def create_full_data_sources() -> Set[DataSource]:
    """创建完整数据采集配置"""
    return {
        DataSource.JOINT_STATES,
        DataSource.HEAD_RGB_CAMERA,
        DataSource.LEFT_ARM_RGB_CAMERA,
        DataSource.RIGHT_ARM_RGB_CAMERA,
        DataSource.CHASSIS_IMU,
        DataSource.ODOMETRY,
        DataSource.LEFT_ARM_END_POSE,
        DataSource.RIGHT_ARM_END_POSE,
    }


def create_vision_only_sources() -> Set[DataSource]:
    """创建仅视觉数据采集配置"""
    return {
        DataSource.HEAD_RGB_CAMERA,
        DataSource.LEFT_ARM_RGB_CAMERA,
        DataSource.RIGHT_ARM_RGB_CAMERA,
        DataSource.HEAD_DEPTH_CAMERA,
    }


def main(
    server: str = "localhost:50051",
    config: str = "full",  # minimal, full, vision
    output_dir: str = "./custom_collected_data"
):
    """
    主函数

    Args:
        server: 服务器地址
        config: 配置类型 (minimal, full, vision)
        output_dir: 输出目录
    """
    # 注册信号处理器
    def signal_handler(sig, frame):
        print("\n\n收到中断信号，正在停止...")
        if 'collector' in globals():
            collector.stop_collecting()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # 根据配置选择数据源
    if config == "minimal":
        data_sources = create_minimal_data_sources()
    elif config == "vision":
        data_sources = create_vision_only_sources()
    elif config == "full":
        data_sources = create_full_data_sources()
    else:
        # 自定义配置：直接指定数据源
        data_sources = set()
        config_parts = config.split(',')
        for part in config_parts:
            part = part.strip()
            try:
                data_sources.add(DataSource(part))
            except ValueError:
                print(f"警告: 未知的数据源 '{part}'，已忽略")
                continue

    print("将要采集的数据源:")
    for source in sorted(data_sources, key=lambda x: x.value):
        print(f"  - {source.value}")
    print()

    # 连接机器人
    print(f"正在连接机器人 {server}...")
    robot = connect(f"x2://{server}")
    print("✓ 机器人连接成功")

    # 创建自定义采集器
    collector = CustomDataCollector(
        robot=robot,
        output_dir=output_dir,
        data_sources=data_sources,
        target_hz=30.0
    )

    try:
        # 启动数据采集
        collector.start_collecting()

        # 录制多个episodes
        for episode_idx in range(3):
            print(f"\n{'='*50}")
            print(f"准备录制 Episode {episode_idx}")
            print(f"{'='*50}")

            # 开始录制
            task_name = input("请输入任务名称: ").strip() or f"task_{episode_idx}"
            collector.start_recording(task=task_name)

            print("\n开始手动录制帧...")
            print("按Enter键记录一帧，按 'q' + Enter 结束录制")

            frame_count = 0
            while True:
                user_input = input("录制命令 (Enter=记录帧, q=结束): ").strip().lower()

                if user_input == 'q':
                    break

                # 记录一帧
                frame = collector.record_frame()
                if frame:
                    frame_count += 1
                    print(f"✓ 已记录帧 {frame_count} (ID: {frame.frame_id})")

                    # 显示数据信息
                    if DataSource.JOINT_STATES in data_sources and frame.joint_positions is not None:
                        print(f"  关节位置: {len(frame.joint_positions)} 个关节")

                    if frame.images:
                        print(f"  图像: {list(frame.images.keys())}")

                    if DataSource.CHASSIS_IMU in data_sources and frame.imu_data:
                        print("  IMU数据: ✓")

            # 停止录制
            episode_info = collector.stop_recording(task_name)

            if episode_info:
                print(f"\n✓ Episode {episode_idx} 录制完成!")
                print(f"  - Episode ID: {episode_info['episode_id']}")
                print(f"  - 帧数: {episode_info['num_frames']}")
                print(f"  - 时长: {episode_info['duration']:.2f}s")
                print(f"  - 保存路径: {episode_info['episode_dir']}")

            # 询问是否继续
            if episode_idx < 2:
                continue_recording = input("\n是否继续录制下一个episode? (y/n): ").strip().lower()
                if continue_recording != 'y':
                    break
    except KeyboardInterrupt:
        print("\n用户中断录制")
    finally:
        # 停止数据采集
        collector.stop_collecting()
        print("\n完全自定义数据采集已完成!")
        print(f"数据保存在: {collector.output_dir}")


if __name__ == "__main__":
    import typer
    typer.run(main)
