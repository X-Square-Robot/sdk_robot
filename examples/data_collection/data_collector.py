"""
gRPC 实时数据采集器 - 支持所有传感器，保存为通用JSON格式

使用方法:
    from x2robot import connect
    from x2robot.action_data_collection import DataCollector
    from x2robot.collection_config import CollectionConfigPresets
    
    robot = connect("x2://192.168.1.100:50051")
    
    # 使用预设配置
    collector = DataCollector(
        robot, 
        output_dir="./data", 
        target_hz=30,
        collection_config=CollectionConfigPresets.full_manipulation()
    )
    
    collector.start_recording(task="pick and place")
    # ... 执行任务 ...
    collector.stop_recording()
"""

import json
from pathlib import Path
import time
import threading
import queue
from collections import defaultdict
import tempfile
import pickle
import os
import sys
import signal
import struct
import subprocess
from datetime import datetime
from typing import Optional, List, Dict, Any
import io
import atexit

import numpy as np
from PIL import Image
import cv2

from x2robot import Robot
from .collection_config import CollectionConfig


class DataCollector:
    """实时数据采集器 - 保存为通用JSON格式

    采集的数据格式:
    {
        "metadata": {
            "fps": 30,
            "joint_names": ["left_arm_joint1", ...],
            "camera_names": ["head_camera", ...],
            "robot_type": "x2_robot",
            "created_at": "2026-01-12T10:30:00"
        },
        "episodes": [
            {
                "episode_id": 0,
                "task": "pick and place",
                "timestamp": "2026-01-12T10:30:00",
                "duration": 10.5,
                "num_frames": 315,
                "joint_names": ["joint1", "joint2", ...],  // 关节名称列表
                "frames": [
                    {
                        "frame_id": 0,
                        "timestamp": 1234567890.123,
                        "observation": {
                            "joint_positions": [0.1, 0.2, ...],
                            "joint_velocities": [0.0, 0.0, ...],
                            "joint_efforts": [0.0, 0.0, ...]
                        },
                        "action": {
                            "joint_positions": [0.1, 0.2, ...]
                        },
                        "images": {
                            "head_camera": "episode_0/frame_0000_head_camera.jpg",
                            ...
                        }
                    },
                    ...
                ]
            }
        ]
    }
    """
    
    def __init__(
        self,
        robot: Robot,
        output_dir: str = "./collected_data",
        target_hz: float = 30.0,
        collection_config: Optional[CollectionConfig] = None,
        image_quality: int = 95,
        downsample_joint_states: bool = True,
        use_video_storage: bool = False,
        video_codec: str = 'XVID',  # 默认使用XVID，兼容性好
    ):
        """初始化数据采集器

        Args:
            robot: Robot实例
            output_dir: 数据保存目录
            target_hz: 目标采集频率（用于降采样）
            collection_config: 传感器配置对象，使用SensorConfigPresets中的预设
                          如果为None，使用basic_manipulation预设
            image_quality: JPEG图像质量 (1-100)，仅在use_video_storage=False时使用
            downsample_joint_states: 是否对关节状态进行降采样
                - True: 降采样到target_hz（节省内存，适合训练）
                - False: 保持原始500Hz（高精度，内存占用大）
            use_video_storage: 是否使用视频格式存储图像
                - True: 保存为MP4视频（节省空间，加快加载速度）
                - False: 保存为JPG图像（默认，兼容性好）
            video_codec: 视频编码器 (默认: 'XVID')
                推荐: 'XVID' (兼容性好), 'MJPG' (Motion JPEG, 兼容性最好)
                可选: 'mp4v' (MPEG-4), 'avc1' (H.264, 需要硬件支持)
        """
        self.robot = robot
        self.output_dir = Path(output_dir)
        self.target_hz = target_hz
        self.target_period = 1.0 / target_hz
        self.image_quality = image_quality
        self.downsample_joint_states = downsample_joint_states
        self.use_video_storage = use_video_storage
        self.video_codec = video_codec

        # 使用提供的配置或默认配置
        self.collection_config = collection_config or CollectionConfigPresets.basic_manipulation()
        self.camera_names = self.collection_config.get_camera_names()
        
        # 创建输出目录
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Episode计数
        self.episode_count = 0
        
        # 数据缓冲队列 - 只在需要时创建
        # 队列大小计算: 期望录制时长(秒) × 采集频率(Hz)
        # 关节状态: 可能是高频(500Hz)
        # 其他传感器: 通常较低频率(10-100Hz)
        self.queues = {}  # 统一管理所有数据队列
        
        # 计算关节状态队列大小（用于显示，即使不启用关节状态）
        if downsample_joint_states:
            # 支持10分钟采集时间，留有足够缓冲
            self.queue_size = int(target_hz * 900)  # 900秒 = 15分钟
        else:
            # 高频模式：支持更长时间或更大缓冲
            self.queue_size = int(500 * 900)  # 450,000，支持15分钟 @ 500Hz

        # 传感器数据文件锁（不再使用队列）
        sensor_names = []

        if collection_config.enable_left_arm_end_pose:
            sensor_names.append('left_arm_end_pose')
        if collection_config.enable_right_arm_end_pose:
            sensor_names.append('right_arm_end_pose')

        if collection_config.enable_odometry:
            sensor_names.append('odometry')
        if collection_config.enable_pose:
            sensor_names.append('pose')
        if collection_config.enable_chassis_imu:
            sensor_names.append('chassis_imu')

        if collection_config.enable_depth_points:
            sensor_names.append('depth_points')
        if collection_config.enable_laser_scan:
            sensor_names.append('laser_scan')

        if collection_config.enable_left_gripper_tactile:
            sensor_names.append('left_gripper_tactile')
        if collection_config.enable_right_gripper_tactile:
            sensor_names.append('right_gripper_tactile')
        if collection_config.enable_left_hand_tactile:
            sensor_names.append('left_hand_tactile')
        if collection_config.enable_right_hand_tactile:
            sensor_names.append('right_hand_tactile')

        if collection_config.enable_tof_sensors:
            sensor_names.extend(['tof_1', 'tof_2'])
        if collection_config.enable_ultrasonic_sensors:
            sensor_names.extend([f'ultrasonic_{i}' for i in range(1, 5)])

        # 关节状态使用临时文件存储（高频采集，避免内存问题）
        # 根据配置的 slave_joint_names 和 slave_action_names 来设置
        self.slave_joint_names = []
        self.slave_action_names = []
        
        if collection_config.slave_joint_names:
            self.slave_joint_names = collection_config.slave_joint_names
            if downsample_joint_states:
                print(f"关节状态降采样模式: 500Hz → {target_hz}Hz")
            else:
                print(f"关节状态原始频率模式: 保持500Hz")
            print(f"配置的关节状态: {self.slave_joint_names}")
            
            # 根据 slave_action_names 配置动作名称
            if collection_config.slave_action_names is not None:
                # 如果明确指定了 action_names，使用指定的
                self.slave_action_names = collection_config.slave_action_names
            else:
                # 如果为 None，自动根据 joint_names 生成 action_names
                self.slave_action_names = [name.replace('_joint_states', '_actions') for name in self.slave_joint_names]
            
            if self.slave_action_names:
                print(f"配置的动作: {self.slave_action_names}")
            
            # 添加各部位的关节状态和动作到传感器名称列表
            sensor_names.extend(self.slave_joint_names)
            sensor_names.extend(self.slave_action_names)

        if collection_config.enable_master_arm_data:
            sensor_names.append('master_left_arm_joint_state')
            sensor_names.append('master_right_arm_joint_state')
            sensor_names.append('master_left_arm_end_pose')
            sensor_names.append('master_right_arm_end_pose')
            sensor_names.append('master_left_gripper_joint_state')
            sensor_names.append('master_right_gripper_joint_state')

        if collection_config.enable_wrench_ext_world:
            sensor_names.append('left_arm_wrench_ext_world')
            sensor_names.append('right_arm_wrench_ext_world')
        if collection_config.enable_wrench_ext_local:
            sensor_names.append('left_arm_wrench_ext_local')
            sensor_names.append('right_arm_wrench_ext_local')

        # 所有传感器数据使用临时文件存储，避免队列内存溢出
        self.sensor_temp_files = {}  # {sensor_name: file_handle}
        self.sensor_temp_paths = {}  # {sensor_name: file_path}
        self.sensor_file_locks = {}  # {sensor_name: lock}

        # 为每个传感器创建文件锁
        for sensor_name in sensor_names:
            self.sensor_file_locks[sensor_name] = threading.Lock()
        
        # 图像数据使用临时文件存储，避免内存溢出
        self.image_temp_files = {}  # {camera_name: file_handle}
        self.image_temp_paths = {}  # {camera_name: file_path}
        self.image_file_locks = {cam: threading.Lock() for cam in self.camera_names}
        
        # 控制标志
        self.is_recording = False
        # 注意：不再使用 is_collecting，只使用 is_recording
        self.threads = []
        # 存储线程信息，用于重新启动线程
        self.thread_info = []  # [(thread_name, target_func, args), ...]
        
        # 当前episode信息
        self.current_episode_task = None
        self.current_episode_start_time = None
        
        # 数据采集统计 - 动态创建每个启用传感器的计数器
        self.stats = {
            'start_time': None,
            'last_update': None
        }
        # 为每个启用的队列创建计数器
        for queue_name in self.queues.keys():
            self.stats[f'{queue_name}_count'] = 0
        # 为每个相机创建计数器
        for cam in self.camera_names:
            self.stats[f'{cam}_count'] = 0
        self.stats_lock = threading.Lock()
        
        # 关节名称映射
        self.joint_names = None
        self.joint_name_mapping = None
        
        # 错误时间记录（用于网络错误处理）
        self.last_error_time = 0
        
        # 相机数据检测 - 记录每个相机最后一次收到数据的时间
        self.camera_last_data_time = {cam: None for cam in self.camera_names}
        self.camera_data_check_interval = 1.0  # 检查间隔（秒）
        self.camera_data_timeout = 5.0  # 超时时间（秒）- 如果5秒内没有数据就报错
        self.camera_monitor_thread = None
        
        # 数据集元数据文件
        self.metadata_file = self.output_dir / "dataset_metadata.json"
        self._load_or_create_metadata()
        
        # 注册退出时的清理函数，确保临时文件被清理
        # 使用lambda包装以确保能访问self
        atexit.register(lambda: self._cleanup_before_exit("程序退出，正在清理资源..."))
        
        # 注册信号处理器，确保异常退出时也清理临时文件
        def signal_handler(sig, frame):
            """处理中断信号，确保清理临时文件"""
            err_msg = "\n收到中断信号，正在清理资源..."
            try:
                self._cleanup_before_exit(err_msg)
            except Exception as e:
                print(f"清理资源时出错: {e}")
            sys.exit(0)
        
        # 注册 SIGINT (Ctrl+C) 和 SIGTERM 信号处理器
        # 注意：信号处理器可能会被示例文件中的处理器覆盖，但至少这里会尝试清理
        try:
            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)
        except (ValueError, OSError):
            # 在某些环境中（如Windows或某些测试环境）可能不支持某些信号
            pass
    
    def _load_or_create_metadata(self):
        """加载或创建数据集元数据"""
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r', encoding='utf-8') as f:
                self.dataset_metadata = json.load(f)
            self.episode_count = len(self.dataset_metadata.get('episodes', []))
            print(f"加载已有数据集，当前有 {self.episode_count} 个episodes")
        else:
            self.dataset_metadata = {
                "fps": self.target_hz,
                "camera_names": self.camera_names,
                "robot_type": "x2_robot",
                "created_at": datetime.now().isoformat(),
                "episodes": []
            }
            self._save_metadata()
    
    def _save_metadata(self):
        """保存数据集元数据"""
        with open(self.metadata_file, 'w', encoding='utf-8') as f:
            json.dump(self.dataset_metadata, f, indent=2, ensure_ascii=False)
        
    def start_recording(self, task: str = "default_task"):
        """开始录制episode（启动所有数据采集线程）
        
        Args:
            task: 任务描述
        """
        if self.is_recording:
            print("⚠️  警告: 已在录制中")
            return
        
        # 启动各个数据采集线程
        threads = []
        
        # 关节状态采集 - 根据配置的 slave_joint_names 启动对应的采集线程
        if self.slave_joint_names:
            # 创建 joint_name 到采集方法的映射
            joint_collector_map = {
                'left_arm_joint_states': (self._collect_left_arm_joint_states, "LeftArmJointStateCollector"),
                'right_arm_joint_states': (self._collect_right_arm_joint_states, "RightArmJointStateCollector"),
                'lift_joint_states': (self._collect_lift_joint_states, "LiftJointStateCollector"),
                'waist_joint_states': (self._collect_waist_joint_states, "WaistJointStateCollector"),
                'left_gripper_joint_states': (self._collect_left_gripper_joint_states, "LeftGripperJointStateCollector"),
                'right_gripper_joint_states': (self._collect_right_gripper_joint_states, "RightGripperJointStateCollector"),
                'head_joint_states': (self._collect_head_joint_states, "HeadJointStateCollector"),
            }
            
            # 根据配置的 joint_names 启动对应的线程
            for joint_name in self.slave_joint_names:
                if joint_name in joint_collector_map:
                    collector_func, thread_name = joint_collector_map[joint_name]
                    threads.append(threading.Thread(target=collector_func, daemon=True, name=thread_name))
                else:
                    raise ValueError(f"未知的关节状态名称: {joint_name}。支持的名称: {list(joint_collector_map.keys())}")

        # 图像流采集
        if self.collection_config.enable_head_rgb_stream:
            threads.append(threading.Thread(target=self._collect_head_rgb_stream, daemon=True, name="HeadRgbStreamCollector"))

        if self.collection_config.enable_head_depth_stream:
            threads.append(threading.Thread(target=self._collect_head_depth_stream, daemon=True, name="HeadDepthStreamCollector"))

        if self.collection_config.enable_left_arm_rgb_stream:
            threads.append(threading.Thread(target=self._collect_left_arm_rgb_stream, daemon=True, name="LeftArmRgbStreamCollector"))

        if self.collection_config.enable_right_arm_rgb_stream:
            threads.append(threading.Thread(target=self._collect_right_arm_rgb_stream, daemon=True, name="RightArmRgbStreamCollector"))

        # 传感器采集
        if self.collection_config.enable_chassis_imu:
            threads.append(threading.Thread(target=self._collect_imu, daemon=True, name="ImuCollector"))
        
        if self.collection_config.enable_depth_points:
            threads.append(threading.Thread(target=self._collect_depth, daemon=True, name="DepthCollector"))
        
        # 末端位姿采集
        if self.collection_config.enable_left_arm_end_pose:
            threads.append(threading.Thread(target=self._collect_left_arm_end_pose, daemon=True, name="LeftArmEndPoseCollector"))

        if self.collection_config.enable_right_arm_end_pose:
            threads.append(threading.Thread(target=self._collect_right_arm_end_pose, daemon=True, name="RightArmEndPoseCollector"))

        # 底盘传感器采集
        if self.collection_config.enable_odometry:
            threads.append(threading.Thread(target=self._collect_odometry, daemon=True, name="OdometryCollector"))

        if self.collection_config.enable_pose:
            threads.append(threading.Thread(target=self._collect_pose, daemon=True, name="PoseCollector"))

        # 深度和激光传感器采集
        if self.collection_config.enable_head_depth_video:
            threads.append(threading.Thread(target=self._collect_head_depth_video, daemon=True, name="HeadDepthVideoCollector"))

        if self.collection_config.enable_laser_scan:
            threads.append(threading.Thread(target=self._collect_laser_scan, daemon=True, name="LaserScanCollector"))

        # 触觉传感器采集
        if self.collection_config.enable_left_gripper_tactile:
            threads.append(threading.Thread(target=self._collect_left_gripper_tactile, daemon=True, name="LeftGripperTactileCollector"))

        if self.collection_config.enable_right_gripper_tactile:
            threads.append(threading.Thread(target=self._collect_right_gripper_tactile, daemon=True, name="RightGripperTactileCollector"))

        if self.collection_config.enable_left_hand_tactile:
            threads.append(threading.Thread(target=self._collect_left_hand_tactile, daemon=True, name="LeftHandTactileCollector"))

        if self.collection_config.enable_right_hand_tactile:
            threads.append(threading.Thread(target=self._collect_right_hand_tactile, daemon=True, name="RightHandTactileCollector"))

        # 距离传感器采集
        if self.collection_config.enable_tof_sensors:
            threads.append(threading.Thread(target=self._collect_tof_sensors, daemon=True, name="ToFSensorsCollector"))

        if self.collection_config.enable_ultrasonic_sensors:
            threads.append(threading.Thread(target=self._collect_ultrasonic_sensors, daemon=True, name="UltrasonicSensorsCollector"))

        if self.collection_config.enable_master_arm_data:
            threads.append(threading.Thread(target=self._collect_master_left_arm_end_pose, daemon=True, name="MasterLeftArmEndPoseCollector"))
            threads.append(threading.Thread(target=self._collect_master_left_arm_joint_state, daemon=True, name="MasterLeftArmJointStateCollector"))
            threads.append(threading.Thread(target=self._collect_master_right_arm_end_pose, daemon=True, name="MasterRightArmEndPoseCollector"))
            threads.append(threading.Thread(target=self._collect_master_right_arm_joint_state, daemon=True, name="MasterRightArmJointStateCollector"))
            threads.append(threading.Thread(target=self._collect_master_left_gripper_joint_state, daemon=True, name="MasterLeftGripperJointStateCollector"))
            threads.append(threading.Thread(target=self._collect_master_right_gripper_joint_state, daemon=True, name="MasterRightGripperJointStateCollector"))

        if self.collection_config.enable_wrench_ext_world:
            threads.append(threading.Thread(target=self._collect_left_arm_wrench_ext_world, daemon=True, name="LeftArmWrenchExtWorldCollector"))
            threads.append(threading.Thread(target=self._collect_right_arm_wrench_ext_world, daemon=True, name="RightArmWrenchExtWorldCollector"))
        if self.collection_config.enable_wrench_ext_local:
            threads.append(threading.Thread(target=self._collect_left_arm_wrench_ext_local, daemon=True, name="LeftArmWrenchExtLocalCollector"))
            threads.append(threading.Thread(target=self._collect_right_arm_wrench_ext_local, daemon=True, name="RightArmWrenchExtLocalCollector"))

        # 检查是否至少启用了一种数据采集
        if not threads:
            print("错误: 没有启用任何数据采集，请至少启用一种数据类型")
            return
        
        # 初始化统计
        with self.stats_lock:
            self.stats['start_time'] = time.time()
            self.stats['last_update'] = time.time()
        
        # 保存线程信息
        self.thread_info = []
        for t in threads:
            self.thread_info.append((t.name, t._target))
            self.threads.append(t)
        
        # 清空队列
        self._clear_queues()
        
        # 创建临时文件
        self._create_temp_files()
        
        self.current_episode_task = task
        self.current_episode_start_time = time.time()
        self.is_recording = True
        
        # 启动所有线程
        for t in threads:
            t.start()
        
        print(f"✓ 开始录制 Episode {self.episode_count} (任务: {task})")
        print(f"  - 输出目录: {self.output_dir}")
        print(f"  - 图像存储: {'MP4视频' if self.use_video_storage else 'JPG图像'}")
        print(f"  - 目标频率: {self.target_hz} Hz")
        
        # 显示启用的数据流
        sensor_count = len(self.sensor_file_locks)
        image_count = len(self.camera_names)

        if sensor_count > 0:
            print(f"  - 传感器数据流: ✓ ({sensor_count}个传感器)")
        else:
            print(f"  - 传感器数据流: ✗ (未启用)")

        if image_count > 0:
            print(f"  - 图像流: ✓ ({image_count}个相机)")
        else:
            print(f"  - 图像流: ✗ (未启用)")
        
        # 重置相机数据时间跟踪
        with self.stats_lock:
            for cam in self.camera_names:
                self.camera_last_data_time[cam] = None
        
        # 启动相机数据监控线程
        self._start_camera_monitor()
        
        # 重置统计 - 重置所有计数统计项
        with self.stats_lock:
            # 重置所有以 _count 结尾的统计项
            for key in list(self.stats.keys()):
                if key.endswith('_count'):
                    if isinstance(self.stats[key], dict):
                        # 如果是字典类型（如 image_count），重置为 defaultdict(int)
                        self.stats[key] = defaultdict(int)
                    else:
                        # 如果是数字类型，重置为 0
                        self.stats[key] = 0
            # 确保这些固定统计项存在
            if 'state_count' not in self.stats:
                self.stats['state_count'] = 0
            if 'action_count' not in self.stats:
                self.stats['action_count'] = 0
            if 'image_count' not in self.stats:
                self.stats['image_count'] = defaultdict(int)
            if 'imu_count' not in self.stats:
                self.stats['imu_count'] = 0
            if 'depth_count' not in self.stats:
                self.stats['depth_count'] = 0
        
        print(f"✓ 开始录制 Episode {self.episode_count} (任务: {task})")
    
    def stop_recording(self) -> Optional[Dict[str, Any]]:
        """停止录制并保存episode（停止所有数据采集线程）
        
        Returns:
            episode_info: Episode信息字典，包含保存路径等
        """
        if not self.is_recording:
            print("⚠️  警告: 未在录制中")
            return None
        
        self.is_recording = False
        print(f"停止录制 Episode {self.episode_count}...")
        
        try:
            # 停止相机数据监控线程
            self._stop_camera_monitor()
            
            # 等待线程退出循环，停止从stream读取数据
            # 这样可以避免在处理数据过程中stream出错导致进程退出
            print("  等待采集线程停止...")
            time.sleep(0.5)  # 给线程足够时间退出循环
            
            # 收集数据（在收集过程中验证数据）
            episode_data = self._collect_episode_data()
            
            if episode_data is None:
                print("错误: Episode数据收集失败，无法获取数据")
                sys.exit(1)
            
            # 验证数据（如果验证失败会抛出RuntimeError并退出）
            # 注意：验证应该在数据收集阶段进行，检查临时文件中是否有实际数据
            if not self._validate_episode_data(episode_data):
                print("错误: Episode数据验证失败")
                sys.exit(1)
            
            # 保存episode
            episode_info = self._save_episode(episode_data, self.current_episode_task)
            
            self.episode_count += 1
            self.current_episode_task = None
            self.current_episode_start_time = None
            
            return episode_info
        finally:
            # 确保临时文件被清理（即使出错也要清理）
            # 注意：这里不需要退出，只是清理资源
            # 先停止线程，再清理文件，避免往已关闭的文件写入
            try:
                # 停止所有线程（通过设置is_recording=False，线程会自动退出）
                if hasattr(self, 'threads') and self.threads:
                    alive_threads = [t for t in self.threads if t.is_alive()]
                    if alive_threads:
                        # 给线程一个很短的时间退出（0.1秒），然后继续
                        # 线程是daemon线程，即使没有完全退出也不会阻塞主进程
                        for thread in alive_threads:
                            thread.join(timeout=0.1)
                        self.threads = []
                
                # 清理临时文件
                self._cleanup_temp_files()
            except Exception as e:
                print(f"  ⚠️  清理资源时出错: {e}")
            
            print("✓ 录制已停止")
            print(f"✓ 总共采集了 {self.episode_count} 个episodes")
            print(f"✓ 数据保存在: {self.output_dir}")
    
    def _collect_joint_states(self):
        """采集机器人全量关节状态数据"""
        print("启动全量关节状态流...")
        
        try:
            stream = self.robot.state.get_all_joint_states_stream(timeout=None)
            
            for state_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break
                
                try:
                    # 第一次收到消息时，建立关节名称映射
                    if self.joint_name_mapping is None:
                        print(f"  🔍 分析关节状态消息结构...")
                        print(f"    消息类型: {type(state_msg)}")

                        # 检查name字段
                        has_name = hasattr(state_msg, 'name')
                        print(f"    name字段: {'✓' if has_name else '✗'}")
                        if has_name:
                            name_val = getattr(state_msg, 'name', None)
                            if name_val and len(name_val) > 0:
                                print(f"    ✓ 关节名称: {len(name_val)} 个 ({name_val[0]}...{name_val[-1]})")
                            else:
                                print(f"    ⚠️  name字段为空或None")

                        # 检查position字段
                        has_position = hasattr(state_msg, 'position')
                        print(f"    position字段: {'✓' if has_position else '✗'}")
                        if has_position:
                            pos_val = getattr(state_msg, 'position', None)
                            if pos_val and len(pos_val) > 0:
                                print(f"    ✓ 关节位置: {len(pos_val)} 个")
                            else:
                                print(f"    ⚠️  position字段为空或None (可能是消息未初始化)")

                        joint_names_obtained = False

                        # 优先使用消息中的关节名称
                        if hasattr(state_msg, 'name') and state_msg.name and len(state_msg.name) > 0:
                            self.joint_names = list(state_msg.name)
                            self.joint_name_mapping = {}
                            for idx, name in enumerate(state_msg.name):
                                self.joint_name_mapping[name] = idx

                            print(f"  ✅ 关节名称设置成功: {len(self.joint_name_mapping)} 个关节")
                            print(f"     关节列表: {self.joint_names}")
                            joint_names_obtained = True

                        # 如果没有name字段，尝试从position长度推断
                        elif hasattr(state_msg, 'position') and state_msg.position and len(state_msg.position) > 0:
                            num_joints = len(state_msg.position)
                            self.joint_names = [f"joint_{i+1}" for i in range(num_joints)]
                            self.joint_name_mapping = {name: idx for idx, name in enumerate(self.joint_names)}

                            print(f"  ⚠️  使用默认关节名称: {len(self.joint_name_mapping)} 个关节")
                            print(f"     默认列表: {self.joint_names}")
                            joint_names_obtained = True

                        else:
                            print(f"  ⏳ 等待有效关节数据... (可能是首次消息未初始化)")

                        # 更新元数据
                        if joint_names_obtained and self.joint_names:
                            self.dataset_metadata['joint_names'] = self.joint_names
                            self._save_metadata()
                            print(f"  ✅ 关节映射配置完成")
                        elif not joint_names_obtained:
                            print(f"  ⏳ 关节名称设置延迟...")
                    
                    # 提取所有关节数据
                    joint_positions = np.array(state_msg.position, dtype=np.float32).flatten()
                    joint_velocities = np.array(state_msg.velocity, dtype=np.float32).flatten() if hasattr(state_msg, 'velocity') and state_msg.velocity else None
                    joint_efforts = np.array(state_msg.effort, dtype=np.float32).flatten() if hasattr(state_msg, 'effort') and state_msg.effort else None

                    timestamp = self._extract_timestamp_from_header(state_msg)
                    
                    if self.is_recording:
                        # 确保临时文件已创建（start_recording后才可用）
                        if hasattr(self, 'sensor_temp_files') and 'joint_states' in self.sensor_temp_files:
                            # 将关节状态数据写入临时文件（避免队列内存问题）
                            joint_data = (timestamp, joint_positions, joint_velocities, joint_efforts)
                            with self.sensor_file_locks['joint_states']:
                                if 'joint_states' in self.sensor_temp_files:
                                    pickle.dump(joint_data, self.sensor_temp_files['joint_states'])
                                    self.sensor_temp_files['joint_states'].flush()

                            # action数据也写入文件（与state相同，用于模仿学习）
                            action_data = (timestamp, joint_positions)
                            with self.sensor_file_locks['actions']:
                                if 'actions' in self.sensor_temp_files:
                                    pickle.dump(action_data, self.sensor_temp_files['actions'])
                                    self.sensor_temp_files['actions'].flush()
                        
                        with self.stats_lock:
                            self.stats['state_count'] += 1
                            self.stats['action_count'] += 1
                
                except Exception as e:
                    print(f"关节状态数据处理错误: {e}")
                    continue
        
        except Exception as e:
            print(f"关节状态流错误: {e}")
            raise
    
    def _collect_left_arm_joint_states(self):
        """采集左臂关节状态数据"""
        print("启动左臂关节状态流...")
        self._collect_joint_state_stream('left_arm_joint_states', self.robot.left_arm.get_joint_states_stream)
    
    def _collect_right_arm_joint_states(self):
        """采集右臂关节状态数据"""
        print("启动右臂关节状态流...")
        self._collect_joint_state_stream('right_arm_joint_states', self.robot.right_arm.get_joint_states_stream)
    
    def _collect_lift_joint_states(self):
        """采集腰部关节状态数据"""
        print("启动腰部关节状态流...")
        self._collect_joint_state_stream('lift_joint_states', self.robot.lift.get_joint_states_stream)
    
    def _collect_left_gripper_joint_states(self):
        """采集左夹爪关节状态数据"""
        print("启动左夹爪关节状态流...")
        self._collect_joint_state_stream('left_gripper_joint_states', self.robot.left_gripper.get_joint_states_stream)
    
    def _collect_right_gripper_joint_states(self):
        """采集右夹爪关节状态数据"""
        print("启动右夹爪关节状态流...")
        self._collect_joint_state_stream('right_gripper_joint_states', self.robot.right_gripper.get_joint_states_stream)
    
    def _collect_head_joint_states(self):
        """采集头部关节状态数据"""
        print("启动头部关节状态流...")
        self._collect_joint_state_stream('head_joint_states', self.robot.head.get_joint_states_stream)

    
    def _collect_waist_joint_states(self):
        """采集腰部关节状态数据"""
        print("启动腰部关节状态流...")
        self._collect_joint_state_stream('waist_joint_states', self.robot.waist.get_joint_states_stream)
    
    
    # ============ 图像流采集方法 ============

    def _collect_head_rgb_stream(self):
        """采集头部RGB视频流"""
        print("启动头部RGB视频流...")
        self._collect_camera_stream('head_rgb_stream', self.robot.head_camera.get_rgb_video_stream)

    def _collect_head_depth_stream(self):
        """采集头部深度视频流"""
        print("启动头部深度视频流...")
        self._collect_depth_stream('head_depth_stream', self.robot.head_camera.get_depth_video_stream)

    def _collect_left_arm_rgb_stream(self):
        """采集左臂RGB视频流"""
        print("启动左臂RGB视频流...")
        self._collect_camera_stream('left_arm_rgb_stream', self.robot.left_arm_camera.get_video_stream)

    def _collect_right_arm_rgb_stream(self):
        """采集右臂RGB视频流"""
        print("启动右臂RGB视频流...")
        self._collect_camera_stream('right_arm_rgb_stream', self.robot.right_arm_camera.get_video_stream)
    
    def _collect_camera_stream(self, camera_name, stream_func):
        """采集单个相机的视频流"""
        print(f"启动 {camera_name} 流...")
        
        try:
            stream = stream_func(timeout=None)
            
            for frame_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break
                
                try:
                    # 检查数据是否为空
                    if not frame_msg or not frame_msg.data:
                        continue

                    # 解码图像 - 需要转换为bytes
                    img_bytes = bytes(frame_msg.data)
                    img = Image.open(io.BytesIO(img_bytes))

                    # 转换为RGB
                    if img.mode != 'RGB':
                        img = img.convert('RGB')

                    timestamp = self._extract_timestamp_from_header(frame_msg)

                    if self.is_recording:
                        # 写入临时文件（压缩为JPEG格式以减少存储空间）
                        try:
                            with self.image_file_locks[camera_name]:
                                if camera_name in self.image_temp_files:
                                    # 将图像压缩为JPEG格式（bytes）以减少存储空间
                                    # 720P未压缩约2.64MB，JPEG压缩后约200-500KB，可减少80-90%空间
                                    img_bytes_compressed = io.BytesIO()
                                    img.save(img_bytes_compressed, 'JPEG', quality=self.image_quality)
                                    img_bytes_compressed = img_bytes_compressed.getvalue()
                                    
                                    # 存储压缩后的图像数据和时间戳
                                    pickle.dump((timestamp, img_bytes_compressed), self.image_temp_files[camera_name])
                                    self.image_temp_files[camera_name].flush()

                                    with self.stats_lock:
                                        self.stats['image_count'][camera_name] += 1
                                        # 更新相机最后收到数据的时间
                                        self.camera_last_data_time[camera_name] = time.time()
                        except Exception as e:
                            print(f"❌ {camera_name} 数据处理错误: {e}")
                            print(f"   数据类型: {type(frame_msg.data) if hasattr(frame_msg, 'data') else '无data属性'}")
                            print(f"   数据内容: {repr(frame_msg.data) if hasattr(frame_msg, 'data') else '无data属性'}")
                            raise RuntimeError(f"{camera_name} 数据处理失败: {e}")

                except Exception as e:
                    if self._is_grpc_connection_error(e):
                        self._handle_grpc_error_and_exit(camera_name, e)
                    print(f"❌ {camera_name} 流处理错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
                    raise RuntimeError(f"{camera_name} 流处理失败: {e}")

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(camera_name, e)
            print(f"❌ {camera_name} 流处理错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
            raise RuntimeError(f"{camera_name} 流处理失败: {e}")

    def _collect_depth_stream(self, camera_name, stream_func):
        """采集深度视频流"""
        print(f"启动 {camera_name} 流...")

        try:
            stream = stream_func(timeout=None)

            for frame_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break

                try:
                    # 检查数据是否为空
                    if not frame_msg or not frame_msg.data:
                        continue
        
                    depth_bytes = bytes(frame_msg.data)
                    timestamp = self._extract_timestamp_from_header(frame_msg)

                    if self.is_recording:
                        # 尝试多种方式处理深度数据

                        depth_data = None

                        # 方法1: 尝试作为压缩图像处理
                        try:
                            depth_img = Image.open(io.BytesIO(depth_bytes))
                            # 转换为numpy数组
                            depth_data = np.array(depth_img, dtype=np.float32)
                            print(f"{camera_name} 深度数据解析为图像: {depth_data.shape}")
                        except Exception:
                            # 方法2: 尝试作为原始float32数组处理
                            try:
                                # 检查数据长度是否是4的倍数
                                if len(depth_bytes) % 4 == 0:
                                    num_pixels = len(depth_bytes) // 4
                                    depth_values = struct.unpack(f'{num_pixels}f', depth_bytes)
                                    depth_data = np.array(depth_values, dtype=np.float32)

                                    # 尝试常见的深度图分辨率
                                    if len(depth_data) == 640 * 480:
                                        depth_data = depth_data.reshape(480, 640)
                                    elif len(depth_data) == 320 * 240:
                                        depth_data = depth_data.reshape(240, 320)
                                    elif len(depth_data) == 1280 * 720:
                                        depth_data = depth_data.reshape(720, 1280)
                                    elif len(depth_data) == 640 * 360:
                                        depth_data = depth_data.reshape(360, 640)
                                    # 如果不是标准分辨率，保持为一维数组
                                    print(f"{camera_name} 深度数据解析为float32数组: {depth_data.shape}")
                                else:
                                    raise ValueError("数据长度不是4的倍数")
                            except Exception:
                                # 方法3: 保存原始字节数据
                                depth_data = depth_bytes
                                print(f"{camera_name} 保存原始深度数据: {len(depth_bytes)} bytes")

                        # 保存处理后的深度数据
                        with self.image_file_locks[camera_name]:
                            if camera_name in self.image_temp_files:
                                pickle.dump((timestamp, depth_data), self.image_temp_files[camera_name])
                                self.image_temp_files[camera_name].flush()

                        with self.stats_lock:
                            self.stats['image_count'][camera_name] += 1
                            # 更新相机最后收到数据的时间
                            self.camera_last_data_time[camera_name] = time.time()

                except Exception as e:
                    if self._is_grpc_connection_error(e):
                        self._handle_grpc_error_and_exit(camera_name, e)
                    print(f"❌ {camera_name} 数据处理错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
                    print(f"   数据类型: {type(frame_msg.data) if hasattr(frame_msg, 'data') else '无data属性'}")
                    print(f"   数据内容: {repr(frame_msg.data) if hasattr(frame_msg, 'data') else '无data属性'}")
                    raise RuntimeError(f"{camera_name} 数据处理失败: {e}")

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(camera_name, e)
            # 其他类型的错误，重新抛出
            print(f"❌ {camera_name} 流错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
            raise
    
    def _collect_imu(self):
        """采集IMU数据"""
        print("启动IMU数据流...")
        
        try:
            stream = self.robot.imu.get_chassis_imu_stream(timeout=None)
            
            for imu_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break
                
                try:
                    imu_data = {
                        'orientation': [
                            float(imu_msg.orientation.x),
                            float(imu_msg.orientation.y),
                            float(imu_msg.orientation.z),
                            float(imu_msg.orientation.w)
                        ],
                        'angular_velocity': [
                            float(imu_msg.angular_velocity.x),
                            float(imu_msg.angular_velocity.y),
                            float(imu_msg.angular_velocity.z)
                        ],
                        'linear_acceleration': [
                            float(imu_msg.linear_acceleration.x),
                            float(imu_msg.linear_acceleration.y),
                            float(imu_msg.linear_acceleration.z)
                        ]
                    }

                    timestamp = self._extract_timestamp_from_header(imu_msg)
                    
                    if self.is_recording:
                        # 确保临时文件已创建（start_recording后才可用）
                        if hasattr(self, 'sensor_temp_files') and 'chassis_imu' in self.sensor_temp_files:
                            imu_data_tuple = (timestamp, imu_data)
                            with self.sensor_file_locks['chassis_imu']:
                                if 'chassis_imu' in self.sensor_temp_files:
                                    pickle.dump(imu_data_tuple, self.sensor_temp_files['chassis_imu'])
                                    self.sensor_temp_files['chassis_imu'].flush()
                        with self.stats_lock:
                            self.stats['imu_count'] += 1
                
                except Exception as e:
                    if self._is_grpc_connection_error(e):
                        self._handle_grpc_error_and_exit('chassis_imu', e)
                    print(f"IMU数据处理错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                    continue
        
        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit('chassis_imu', e)
            print(f"IMU流错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            raise
    
    def _collect_depth(self):
        """采集深度数据"""
        print("启动深度数据流...")
        
        try:
            stream = self.robot.depth_points.get_chassis_depth_points_stream(timeout=None)
            
            for depth_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break
                
                try:
                    timestamp = self._extract_timestamp_from_header(depth_msg)
                    
                    if self.is_recording:
                        depth_info = {
                            'timestamp': timestamp,
                            'width': depth_msg.width,
                            'height': depth_msg.height,
                            'point_count': len(depth_msg.data) if hasattr(depth_msg, 'data') else 0
                        }
                        # 确保临时文件已创建（start_recording后才可用）
                        if hasattr(self, 'sensor_temp_files') and 'depth_points' in self.sensor_temp_files:
                            depth_data_tuple = (timestamp, depth_info)
                            with self.sensor_file_locks['depth_points']:
                                if 'depth_points' in self.sensor_temp_files:
                                    pickle.dump(depth_data_tuple, self.sensor_temp_files['depth_points'])
                                    self.sensor_temp_files['depth_points'].flush()
                        with self.stats_lock:
                            self.stats['depth_count'] += 1
                
                except Exception as e:
                    print(f"深度数据处理错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                    continue
        
        except Exception as e:
            print(f"深度流错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            raise

    # ============ 末端位姿采集方法 ============
    
    def _collect_left_arm_end_pose(self):
        """采集左臂末端位姿"""
        print("启动左臂末端位姿流...")
        self._collect_pose_stream('left_arm_end_pose', self.robot.left_arm.get_end_pose_stream)

    def _collect_right_arm_end_pose(self):
        """采集右臂末端位姿"""
        print("启动右臂末端位姿流...")
        self._collect_pose_stream('right_arm_end_pose', self.robot.right_arm.get_end_pose_stream)
                
    # ============ 底盘传感器采集方法 ============

    def _collect_odometry(self):
        """采集底盘里程计数据"""
        print("启动里程计流...")
        self._collect_generic_stream('odometry', self.robot.chassis.get_odometry_stream)

    def _collect_pose(self):
        """采集机器人定位数据"""
        print("启动定位数据流...")
        self._collect_generic_stream('pose', self.robot.chassis.get_pose_stream)

    # ============ 深度和激光传感器采集方法 ============

    def _collect_head_depth_video(self):
        """采集头部深度视频流"""
        print("启动头部深度视频流...")
        self._collect_camera_stream('head_depth_video', self.robot.head_camera.get_depth_video_stream)

    def _collect_laser_scan(self):
        """采集激光雷达扫描数据"""
        print("启动激光雷达流...")
        self._collect_generic_stream('laser_scan', self.robot.radar.get_laser_scan_stream)

    # ============ 触觉传感器采集方法 ============

    def _collect_left_gripper_tactile(self):
        """采集左夹爪触觉数据"""
        print("启动左夹爪触觉传感器...")
        if hasattr(self.robot, 'left_gripper_tactile'):
            self._collect_generic_stream('left_gripper_tactile', self.robot.left_gripper_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("左夹爪触觉传感器不可用 (仅CX002型号支持)")

    def _collect_right_gripper_tactile(self):
        """采集右夹爪触觉数据"""
        print("启动右夹爪触觉传感器...")
        if hasattr(self.robot, 'right_gripper_tactile'):
            self._collect_generic_stream('right_gripper_tactile', self.robot.right_gripper_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("右夹爪触觉传感器不可用 (仅CX002型号支持)")

    def _collect_left_hand_tactile(self):
        """采集左灵巧手触觉数据"""
        print("启动左灵巧手触觉传感器...")
        if hasattr(self.robot, 'left_hand_tactile'):
            self._collect_generic_stream('left_hand_tactile', self.robot.left_hand_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("左灵巧手触觉传感器不可用 (仅CX002型号支持)")

    def _collect_right_hand_tactile(self):
        """采集右灵巧手触觉数据"""
        print("启动右灵巧手触觉传感器...")
        if hasattr(self.robot, 'right_hand_tactile'):
            self._collect_generic_stream('right_hand_tactile', self.robot.right_hand_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("右灵巧手触觉传感器不可用 (仅CX002型号支持)")

    # ============ Action专用采集方法 ============

    # ============ 独立的控制命令流采集方法 ============

    def _collect_master_left_arm_end_pose(self):
        """采集主臂左臂位姿控制命令"""
        print("启动主臂左臂位姿控制命令采集...")
        try:
            self._collect_pose_stream('master_left_arm_end_pose', self.robot.master_left_arm.get_end_pose_stream)  # 占位符
        except Exception as e:
            print(f"主臂左臂位姿控制命令采集失败: {e}")
            raise

    def _collect_master_right_arm_end_pose(self):
        """采集主臂右臂位姿控制命令"""
        print("启动主臂右臂位姿控制命令采集...")
        try:
            self._collect_pose_stream('master_right_arm_end_pose', self.robot.master_right_arm.get_end_pose_stream)  # 占位符
        except Exception as e:
            print(f"主臂右臂位姿控制命令采集失败: {e}")
            raise
    
    def _collect_master_left_arm_joint_state(self):
        """采集主臂左臂关节控制命令"""
        print("启动主臂左臂关节控制命令采集...")
        try:
            self._collect_joint_state_stream('master_left_arm_joint_state', self.robot.master_left_arm.get_joint_states_stream)  # 占位符
        except Exception as e:
            print(f"主臂左臂关节控制命令采集失败: {e}")
            raise

    def _collect_master_right_arm_joint_state(self):
        """采集主臂右臂关节控制命令"""
        print("启动主臂右臂关节控制命令采集...")
        try:
            self._collect_joint_state_stream('master_right_arm_joint_state', self.robot.master_right_arm.get_joint_states_stream)  # 占位符
        except Exception as e:
            print(f"主臂右臂关节控制命令采集失败: {e}")
            raise

    def _collect_master_left_gripper_joint_state(self):
        """采集主臂左夹爪关节控制命令"""
        print("启动主臂左夹爪关节控制命令采集...")
        try:
            self._collect_joint_state_stream('master_left_gripper_joint_state', self.robot.master_left_arm.get_gripper_joint_states_stream)  # 占位符
        except Exception as e:
            print(f"主臂左夹爪关节控制命令采集失败: {e}")
            raise

    def _collect_master_right_gripper_joint_state(self):
        """采集主臂右夹爪关节控制命令"""
        print("启动主臂右夹爪关节控制命令采集...")
        try:
            self._collect_joint_state_stream('master_right_gripper_joint_state', self.robot.master_right_arm.get_gripper_joint_states_stream)  # 占位符
        except Exception as e:
            print(f"主臂右夹爪关节控制命令采集失败: {e}")
            raise

    def _collect_left_arm_wrench_ext_world(self):
        """采集主臂左夹爪关节控制命令"""
        print("启动手腕外力采集...")
        try:
            self._collect_generic_stream('left_arm_wrench_ext_world', self.robot.left_arm.get_wrench_ext_world_stream)  # 占位符
        except Exception as e:
            print(f"手腕外力采集失败: {e}")
            raise

    def _collect_left_arm_wrench_ext_local(self):
        """采集主臂右夹爪关节控制命令"""
        print("启动手腕本地力采集...")
        try:
            self._collect_generic_stream('left_arm_wrench_ext_local', self.robot.left_arm.get_wrench_ext_local_stream)  # 占位符
        except Exception as e:
            print(f"手腕本地力采集失败: {e}")
            raise

    def _collect_right_arm_wrench_ext_world(self):
        """采集主臂左夹爪关节控制命令"""
        print("启动手腕外力采集...")
        try:
            self._collect_generic_stream('right_arm_wrench_ext_world', self.robot.right_arm.get_wrench_ext_world_stream)  # 占位符
        except Exception as e:
            print(f"手腕外力采集失败: {e}")
            raise

    def _collect_right_arm_wrench_ext_local(self):
        """采集主臂右夹爪关节控制命令"""
        print("启动手腕本地力采集...")
        try:
            self._collect_generic_stream('right_arm_wrench_ext_local', self.robot.right_arm.get_wrench_ext_local_stream)  # 占位符
        except Exception as e:
            print(f"手腕本地力采集失败: {e}")
            raise

    def _collect_vr_left_arm_pose_commands(self):
        """采集VR左臂位姿控制命令"""
        print("启动VR左臂位姿控制命令采集...")
        try:
            self._collect_generic_stream('vr_left_arm_pose_commands', self.robot.action_data_collection.get_vr_left_arm_pose_commands)  # 占位符
        except Exception as e:
            print(f"VR左臂位姿控制命令采集失败: {e}")
            raise

    def _collect_vr_right_arm_pose_commands(self):
        """采集VR右臂位姿控制命令"""
        print("启动VR右臂位姿控制命令采集...")
        try:
            self._collect_generic_stream('vr_right_arm_pose_commands', self.robot.action_data_collection.get_vr_right_arm_pose_commands)  # 占位符
        except Exception as e:
            print(f"VR右臂位姿控制命令采集失败: {e}")
            raise

    def _collect_vr_left_gripper_joint_commands(self):
        """采集VR左夹爪关节控制命令"""
        print("启动VR左夹爪关节控制命令采集...")
        try:
            self._collect_generic_stream('vr_left_gripper_joint_commands', self.robot.action_data_collection.get_vr_left_gripper_joint_commands)  # 占位符
        except Exception as e:
            print(f"VR左夹爪关节控制命令采集失败: {e}")
            raise

    def _collect_vr_right_gripper_joint_commands(self):
        """采集VR右夹爪关节控制命令"""
        print("启动VR右臂位姿控制命令采集...")
        try:
            self._collect_generic_stream('vr_right_arm_pose_commands', self.robot.action_data_collection.get_vr_right_gripper_joint_commands)  # 占位符
        except Exception as e:
            print(f"VR右夹爪关节控制命令采集失败: {e}")
            raise


    # ============ 距离传感器采集方法 ============

    def _collect_tof_sensors(self):
        """采集ToF传感器数据"""
        print("启动ToF传感器...")
        # 同时采集两个ToF传感器
        tof_threads = []
        for i in [1, 2]:
            queue_name = f'tof_{i}'
            stream_func = getattr(self.robot.tof, f'get_chassis_tof_{i}_stream')
            thread = threading.Thread(
                target=self._collect_generic_stream,
                args=(queue_name, stream_func),
                daemon=True
            )
            thread.start()
            tof_threads.append(thread)
            # 保存线程信息以便重新启动（使用 lambda 包装参数）
            if hasattr(self, 'thread_info'):
                def _tof_collector_wrapper():
                    self._collect_generic_stream(queue_name, stream_func)
                self.thread_info.append((f'ToF_{i}_Collector', _tof_collector_wrapper))
                self.threads.append(thread)

        # 不调用 join()，让线程在后台运行
        # 当 is_recording = False 时，线程会退出循环，方法也会结束
        # 当重新启动时，会重新创建这些线程

    def _collect_ultrasonic_sensors(self):
        """采集超声波传感器数据"""
        print("启动超声波传感器...")
        # 同时采集4个超声波传感器
        ultrasonic_threads = []
        for i in range(1, 5):
            queue_name = f'ultrasonic_{i}'
            stream_func = getattr(self.robot.ultrasonic, f'get_chassis_ultrasonic_{i}_stream')
            thread = threading.Thread(
                target=self._collect_generic_stream,
                args=(queue_name, stream_func),
                daemon=True
            )
            thread.start()
            ultrasonic_threads.append(thread)
            # 保存线程信息以便重新启动（使用 lambda 包装参数）
            if hasattr(self, 'thread_info'):
                def _ultrasonic_collector_wrapper():
                    self._collect_generic_stream(queue_name, stream_func)
                self.thread_info.append((f'Ultrasonic_{i}_Collector', _ultrasonic_collector_wrapper))
                self.threads.append(thread)

        # 不调用 join()，让线程在后台运行
        # 当 is_recording = False 时，线程会退出循环，方法也会结束
        # 当重新启动时，会重新创建这些线程

    # ============ 通用辅助方法 ============

    def _start_camera_monitor(self):
        """启动相机数据监控线程"""
        if self.camera_monitor_thread is not None and self.camera_monitor_thread.is_alive():
            return
        
        self.camera_monitor_thread = threading.Thread(
            target=self._monitor_camera_data,
            daemon=True,
            name="CameraDataMonitor"
        )
        self.camera_monitor_thread.start()
    
    def _stop_camera_monitor(self):
        """停止相机数据监控线程"""
        # 监控线程会在检查到 is_recording = False 时自动退出
        pass
    
    def _stop_all_threads(self):
        """停止所有数据采集线程"""
        # 设置标志让所有线程退出循环
        self.is_recording = False
        
        # 停止相机数据监控线程
        self._stop_camera_monitor()
        
        # 等待线程退出循环，停止从stream读取数据
        # 这样可以避免在处理数据过程中stream出错导致进程退出
        if hasattr(self, 'threads') and self.threads:
            print("  等待采集线程停止...")
            time.sleep(0.5)  # 给线程足够时间退出循环
            
            # 快速检查线程状态，但不阻塞太久
            # 线程会在检查到is_recording=False时退出循环
            alive_threads = [t for t in self.threads if t.is_alive()]
            if alive_threads:
                # 给线程一个很短的时间退出（0.1秒），然后继续
                # 线程是daemon线程，即使没有完全退出也不会阻塞主进程
                for thread in alive_threads:
                    thread.join(timeout=0.1)
            
            # 清空线程列表
            self.threads = []
    
    def _cleanup_before_exit(self, message="正在清理资源..."):
        """统一的清理函数：先停止线程，再清理临时文件（用于退出前的清理）
        
        注意：此函数只负责清理，不负责退出，退出由调用者决定
        
        Args:
            message: 清理前的提示信息
        """
        
        # 第一步：停止所有线程（避免往已关闭的文件写入）
        try:
            self._stop_all_threads()
        except Exception as e:
            print(f"  ⚠️  停止线程时出错: {e}")

        # 第二步：清理临时文件
        try:
            if message:
                print(message)

            self._cleanup_temp_files()
            print("  ✓ 临时文件已清理")
        except Exception as e:
            print(f"  ⚠️  清理临时文件时出错: {e}")
    
    def _get_enabled_cameras(self):
        """获取实际启用的相机列表（根据配置）"""
        enabled_cameras = []
        if self.collection_config.enable_head_rgb_stream:
            enabled_cameras.append('head_rgb_stream')
        if self.collection_config.enable_head_depth_stream:
            enabled_cameras.append('head_depth_stream')
        if self.collection_config.enable_left_arm_rgb_stream:
            enabled_cameras.append('left_arm_rgb_stream')
        if self.collection_config.enable_right_arm_rgb_stream:
            enabled_cameras.append('right_arm_rgb_stream')
        return enabled_cameras
    
    def _monitor_camera_data(self):
        """监控相机数据，如果某个相机没有数据就报错退出"""
        recording_start_time = time.time()
        
        # 获取实际启用的相机列表
        enabled_cameras = self._get_enabled_cameras()
        
        # 如果没有启用的相机，不需要监控
        if not enabled_cameras:
            return
        
        while self.is_recording:
            current_time = time.time()
            elapsed_time = current_time - recording_start_time
            
            # 只在录制开始后检查（给相机一些时间开始采集）
            # 至少要等待超时时间才能判断相机是否真的没有数据
            if elapsed_time < self.camera_data_timeout:
                time.sleep(self.camera_data_check_interval)
                continue
            
            # 只检查实际启用的相机
            cameras_without_data = []
            with self.stats_lock:
                for cam in enabled_cameras:
                    # 确保相机在跟踪列表中
                    if cam not in self.camera_last_data_time:
                        continue
                    
                    last_data_time = self.camera_last_data_time.get(cam)
                    if last_data_time is None:
                        # 从未收到过数据，但只有在录制时间超过超时时间后才报错
                        if elapsed_time >= self.camera_data_timeout:
                            cameras_without_data.append(cam)
                    else:
                        # 检查是否超时
                        time_since_last_data = current_time - last_data_time
                        if time_since_last_data > self.camera_data_timeout:
                            cameras_without_data.append(cam)
            
            if cameras_without_data:
                error_msg = f"❌ 检测到以下相机没有数据（超时 {self.camera_data_timeout} 秒）:\n"
                for cam in cameras_without_data:
                    last_time = self.camera_last_data_time.get(cam)
                    if last_time is None:
                        error_msg += f"  - {cam}: 从未收到数据\n"
                    else:
                        time_since = current_time - last_time
                        error_msg += f"  - {cam}: 最后数据时间 {time_since:.2f} 秒前\n"
                error_msg += "  进程即将退出..."
                # 清理临时文件后再退出
                self._cleanup_before_exit(error_msg)
                os._exit(1)
            
            time.sleep(self.camera_data_check_interval)
    
    def _restart_stopped_threads(self):
        """重新启动已停止的线程"""
        if not hasattr(self, 'thread_info') or not self.thread_info:
            return
        
        # 检查哪些线程已经停止
        stopped_threads = []
        for i, thread in enumerate(self.threads):
            if not thread.is_alive():
                stopped_threads.append(i)
        
        if stopped_threads:
            print(f"  检测到 {len(stopped_threads)} 个线程已停止，正在重新启动...")
            for i in stopped_threads:
                thread_name, target_func = self.thread_info[i]
                # 创建新线程
                new_thread = threading.Thread(target=target_func, daemon=True, name=thread_name)
                new_thread.start()
                self.threads[i] = new_thread
                print(f"    ✓ 重新启动线程: {thread_name}")

    def _is_grpc_connection_error(self, error):
        """检测是否是 gRPC 连接错误"""
        error_str = str(error).lower()
        error_type = type(error).__name__
        
        # 检查错误类型
        if 'MultiThreadedRendezvous' in error_type or 'grpc' in error_type.lower():
            return True
        
        # 检查错误消息
        if any(keyword in error_str for keyword in [
            'connection reset', 
            'unavailable', 
            'peer', 
            'network',
            'grpc_status',
            'statuscode.unavailable',
            'recvmsg:connection reset'
        ]):
            return True
        
        return False
    
    def _handle_grpc_error_and_exit(self, queue_name, error):
        """处理 gRPC 连接错误并退出进程"""
        err_msg = f"❌ {queue_name} 网络连接中断: {error}\n"
        err_msg += f"   时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}\n"
        err_msg += f"   请检查网络连接后重启数据采集"
        # 清理资源后退出
        self._cleanup_before_exit(err_msg)
        print(f"   进程即将退出...")
        # 直接退出进程（使用os._exit避免触发其他清理逻辑，但我们已经手动清理了）
        os._exit(1)

    def _collect_joint_state_stream(self, queue_name, stream_func):
        """通用关节状态流采集方法"""
        # 存储每个部位的关节名称（第一次收到消息时保存）
        joint_names_cache = {}
        
        try:
            stream = stream_func(timeout=None)
            message_count = 0
            
            for joint_state_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                try:
                    timestamp = self._extract_timestamp_from_header(joint_state_msg)
                    message_count += 1
                    
                    # 第一次收到消息时，保存关节名称
                    if queue_name not in joint_names_cache:
                        if hasattr(joint_state_msg, 'name') and joint_state_msg.name and len(joint_state_msg.name) > 0:
                            joint_names_cache[queue_name] = list(joint_state_msg.name)
                            print(f"  ✓ {queue_name} 关节名称: {joint_names_cache[queue_name]}")
                            # 保存到实例变量中，供后续使用
                            if not hasattr(self, '_joint_names_by_part'):
                                self._joint_names_by_part = {}
                            self._joint_names_by_part[queue_name] = joint_names_cache[queue_name]
                    
                    if self.is_recording:
                        # 提取关节数据
                        positions = np.array(joint_state_msg.position, dtype=np.float32).flatten() if joint_state_msg.position else np.array([], dtype=np.float32)
                        velocities = np.array(joint_state_msg.velocity, dtype=np.float32).flatten() if hasattr(joint_state_msg, 'velocity') and joint_state_msg.velocity else None
                        efforts = np.array(joint_state_msg.effort, dtype=np.float32).flatten() if hasattr(joint_state_msg, 'effort') and joint_state_msg.effort else None
                        
                        # 存储关节状态数据到临时文件
                        if queue_name in self.sensor_file_locks and hasattr(self, 'sensor_temp_files') and queue_name in self.sensor_temp_files:
                            joint_state_data_tuple = (timestamp, positions, velocities, efforts)
                            with self.sensor_file_locks[queue_name]:
                                if queue_name in self.sensor_temp_files:
                                    pickle.dump(joint_state_data_tuple, self.sensor_temp_files[queue_name])
                                    self.sensor_temp_files[queue_name].flush()
                            with self.stats_lock:
                                self.stats[f'{queue_name}_count'] = self.stats.get(f'{queue_name}_count', 0) + 1
                        else:
                            if message_count == 1:  # 只在第一次时打印，避免刷屏
                                print(f"  ⚠️  警告: {queue_name} 临时文件不存在，无法保存数据")
                        
                        # 注意：action数据不需要单独保存，在保存episode时会使用下一帧的state作为action
                except Exception as e:
                    print(f"{queue_name} 处理错误: {e}")
                    import traceback
                    traceback.print_exc()
                    continue
            
            # 如果线程退出但没有收到任何消息，打印警告
            if message_count == 0:
                print(f"  ⚠️  警告: {queue_name} 线程退出，但没有收到任何数据消息")
        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(queue_name, e)
            print(f"{queue_name} 流错误: {e}")
            import traceback
            traceback.print_exc()
            raise

    def _collect_pose_stream(self, queue_name, stream_func):
        """通用末端位姿流采集方法"""
        try:
            stream = stream_func(timeout=None)
            for pose_msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break

                try:
                    timestamp = self._extract_timestamp_from_header(pose_msg)
                    if self.is_recording:
                        pose_data = {
                            'position': {
                                'x': pose_msg.pose.position.x,
                                'y': pose_msg.pose.position.y,
                                'z': pose_msg.pose.position.z,
                            },
                            'orientation': {
                                'x': pose_msg.pose.orientation.x,
                                'y': pose_msg.pose.orientation.y,
                                'z': pose_msg.pose.orientation.z,
                                'w': pose_msg.pose.orientation.w,
                            }
                        }
                        # 存储到对应临时文件
                        if queue_name in self.sensor_file_locks and hasattr(self, 'sensor_temp_files') and queue_name in self.sensor_temp_files:
                            pose_data_tuple = (timestamp, pose_data)
                            with self.sensor_file_locks[queue_name]:
                                if queue_name in self.sensor_temp_files:
                                    pickle.dump(pose_data_tuple, self.sensor_temp_files[queue_name])
                                    self.sensor_temp_files[queue_name].flush()
                            with self.stats_lock:
                                self.stats[f'{queue_name}_count'] = self.stats.get(f'{queue_name}_count', 0) + 1

                except Exception as e:
                    print(f"{queue_name} 处理错误: {e}")
                    continue

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(queue_name, e)
            print(f"{queue_name} 流错误: {e}")
            raise
    
    def _convert_ros_msg_to_dict(self, msg):
        """递归地将ROS消息对象转换为可JSON序列化的字典
        
        Args:
            msg: ROS消息对象
            
        Returns:
            dict或基本类型
        """
        # 如果是基本类型，直接返回
        if isinstance(msg, (int, float, str, bool, type(None))):
            return msg
        
        # 如果是NumPy数组，转换为列表
        if isinstance(msg, np.ndarray):
            return msg.tolist()
        
        # 如果是列表或元组，递归处理每个元素
        if isinstance(msg, (list, tuple)):
            return [self._convert_ros_msg_to_dict(item) for item in msg]
        
        # 如果是字典，递归处理每个值
        if isinstance(msg, dict):
            return {k: self._convert_ros_msg_to_dict(v) for k, v in msg.items()}
        
        # 如果是ROS消息对象（有__slots__属性）
        if hasattr(msg, '__slots__'):
            result = {}
            for slot in msg.__slots__:
                if hasattr(msg, slot):
                    value = getattr(msg, slot)
                    result[slot] = self._convert_ros_msg_to_dict(value)
            return result
        
        # 如果有__dict__属性
        if hasattr(msg, '__dict__'):
            return {k: self._convert_ros_msg_to_dict(v) for k, v in msg.__dict__.items()}
        
        # 其他情况，尝试转换为字符串
        try:
            return str(msg)
        except:
            return None
    
    def _collect_generic_stream(self, queue_name, stream_func):
        """通用传感器流采集方法"""
        try:
            stream = stream_func(timeout=None)
            for msg in stream:
                # 如果不在录制中，退出循环
                if not self.is_recording:
                    break
                
                # 如果不在录制中，退出循环（线程会停止）
                if not self.is_recording:
                    break

                try:
                    timestamp = self._extract_timestamp_from_header(msg)
                    if self.is_recording:
                        # 存储原始消息数据
                        data = {
                            'timestamp': timestamp,
                            'data': msg
                        }
                        # 存储到对应临时文件
                        if queue_name in self.sensor_file_locks and hasattr(self, 'sensor_temp_files') and queue_name in self.sensor_temp_files:
                            sensor_data_tuple = (timestamp, data)
                            with self.sensor_file_locks[queue_name]:
                                if queue_name in self.sensor_temp_files:
                                    pickle.dump(sensor_data_tuple, self.sensor_temp_files[queue_name])
                                    self.sensor_temp_files[queue_name].flush()
                            with self.stats_lock:
                                self.stats[f'{queue_name}_count'] = self.stats.get(f'{queue_name}_count', 0) + 1

                except Exception as e:
                    print(f"{queue_name} 处理错误: {e}")
                    continue

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(queue_name, e)
            # 其他类型的错误，重新抛出
            print(f"❌ {queue_name} 流错误: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
            raise


    def get_stats(self):
        """获取当前统计信息"""
        with self.stats_lock:
            stats = self.stats.copy()
            if self.current_episode_start_time:
                stats['recording_duration'] = time.time() - self.current_episode_start_time
            return stats
    
    def print_stats(self):
        """打印当前统计信息"""
        stats = self.get_stats()
        print(f"\n=== 采集统计 ===")
        print(f"状态帧数: {stats.get('state_count', 0)}")
        print(f"动作帧数: {stats.get('action_count', 0)}")
        for cam, count in stats.get('image_count', {}).items():
            print(f"{cam}: {count} 帧")
        # 显示所有传感器统计
        for stat_key, count in sorted(stats.items()):
            if stat_key.endswith('_count') and stat_key not in ['state_count', 'action_count', 'image_count']:
                # 只对整数类型的count进行 > 0 检查
                if isinstance(count, int) and count > 0:
                    sensor_name = stat_key.replace('_count', '')
                    if sensor_name in ['head_camera', 'left_arm_camera', 'right_arm_camera',
                                       'head_depth_video', 'head_rgb_video']:
                        print(f"{sensor_name}: {count} 帧")
                    else:
                        print(f"{sensor_name}: {count}")

        # 兼容旧的显示方式
        if self.collection_config.enable_chassis_imu:
            print(f"IMU: {stats.get('chassis_imu_count', 0)} 帧")
        if self.collection_config.enable_depth_points:
            print(f"深度: {stats.get('depth_points_count', 0)} 帧")
        if 'recording_duration' in stats:
            print(f"录制时长: {stats['recording_duration']:.2f}s")
        
        # 显示文件存储状态
        sensor_files = len([s for s in self.sensor_file_locks.keys() if hasattr(self, 'sensor_temp_files') and s in self.sensor_temp_files])
        hf_files = len([h for h in ['joint_states', 'actions'] if hasattr(self, 'sensor_temp_files') and h in self.sensor_temp_files])

        if sensor_files > 0 or hf_files > 0:
            print(f"数据存储状态: ✓ {sensor_files}个传感器文件, ✓ {hf_files}个高频文件")
            print(f"存储策略: 全部文件写入 (非阻塞，无内存限制)")

            # 检查是否有覆盖发生
            if hasattr(self, '_queue_overwrites'):
                overwrites = self._queue_overwrites
                if overwrites > 0:
                    print(f"⚠️  注意: 某些数据可能因处理延迟而被覆盖")
        else:
            print(f"数据存储: ✗ 无文件存储")
        
        print("================\n")
    
    def _extract_timestamp_from_header(self, msg):
        """从消息的header中提取时间戳"""
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            return float(sec) + float(nanosec) / 1e9
        else:
            # 如果没有header，使用本地时间戳（兼容旧格式）
            return time.time()
    
    def _get_enabled_data_mapping(self):
        """获取启用的配置项到数据键名的映射
        
        Returns:
            dict: {配置项名称: [数据键名列表]}
        """
        mapping = {}
        
        # 关节状态采集
        if self.slave_joint_names:
            mapping['slave_joint_names'] = self.slave_joint_names
        
        # 图像流采集
        if self.collection_config.enable_head_rgb_stream:
            mapping['enable_head_rgb_stream'] = ['head_rgb_stream']
        if self.collection_config.enable_head_depth_stream:
            mapping['enable_head_depth_stream'] = ['head_depth_stream']
        if self.collection_config.enable_left_arm_rgb_stream:
            mapping['enable_left_arm_rgb_stream'] = ['left_arm_rgb_stream']
        if self.collection_config.enable_right_arm_rgb_stream:
            mapping['enable_right_arm_rgb_stream'] = ['right_arm_rgb_stream']
        
        # 传感器采集
        if self.collection_config.enable_left_arm_end_pose:
            mapping['enable_left_arm_end_pose'] = ['left_arm_end_pose']
        if self.collection_config.enable_right_arm_end_pose:
            mapping['enable_right_arm_end_pose'] = ['right_arm_end_pose']
        if self.collection_config.enable_odometry:
            mapping['enable_odometry'] = ['odometry']
        if self.collection_config.enable_pose:
            mapping['enable_pose'] = ['pose']
        if self.collection_config.enable_chassis_imu:
            mapping['enable_chassis_imu'] = ['chassis_imu']
        if self.collection_config.enable_depth_points:
            mapping['enable_depth_points'] = ['depth_points']
        if self.collection_config.enable_head_depth_video:
            mapping['enable_head_depth_video'] = ['head_depth_video']
        if self.collection_config.enable_laser_scan:
            mapping['enable_laser_scan'] = ['laser_scan']
        
        # 触觉传感器
        if self.collection_config.enable_left_gripper_tactile:
            mapping['enable_left_gripper_tactile'] = ['left_gripper_tactile']
        if self.collection_config.enable_right_gripper_tactile:
            mapping['enable_right_gripper_tactile'] = ['right_gripper_tactile']
        if self.collection_config.enable_left_hand_tactile:
            mapping['enable_left_hand_tactile'] = ['left_hand_tactile']
        if self.collection_config.enable_right_hand_tactile:
            mapping['enable_right_hand_tactile'] = ['right_hand_tactile']
        
        # 距离传感器
        if self.collection_config.enable_tof_sensors:
            mapping['enable_tof_sensors'] = ['tof_1', 'tof_2']
        if self.collection_config.enable_ultrasonic_sensors:
            mapping['enable_ultrasonic_sensors'] = [f'ultrasonic_{i}' for i in range(1, 5)]
        
        # 主臂数据
        if self.collection_config.enable_master_arm_data:
            mapping['enable_master_arm_data'] = [
                'master_left_arm_joint_state',
                'master_right_arm_joint_state',
                'master_left_arm_end_pose',
                'master_right_arm_end_pose',
                'master_left_gripper_joint_state',
                'master_right_gripper_joint_state'
            ]
        
        # 力传感器
        if self.collection_config.enable_wrench_ext_world:
            mapping['enable_wrench_ext_world'] = [
                'left_arm_wrench_ext_world',
                'right_arm_wrench_ext_world'
            ]
        if self.collection_config.enable_wrench_ext_local:
            mapping['enable_wrench_ext_local'] = [
                'left_arm_wrench_ext_local',
                'right_arm_wrench_ext_local'
            ]
        
        return mapping
    
    def _validate_collected_data(self, sensor_data: Dict[str, Any], images: Dict[str, Any]) -> bool:
        """验证收集到的原始数据（在对齐之前）
        
        检查所有启用的数据项是否都有实际数据（从临时文件中读取的）
        
        Args:
            sensor_data: 从临时文件收集的原始传感器数据
            images: 从临时文件收集的原始图像数据
            
        Returns:
            True if all enabled data items have data, False otherwise
        """
        # 获取启用的配置项映射
        enabled_mapping = self._get_enabled_data_mapping()
        
        # 检查所有启用的数据项是否都有数据
        missing_data_items = []
        
        # 用于记录主臂数据的缺失情况（仅告警，不报错，排除action数据）
        master_arm_missing_items = []
        
        # 相机名称映射：内部流名称 -> 用户友好名称
        camera_name_mapping = {
            'head_rgb_stream': 'head_camera',
            'head_depth_stream': 'head_depth_camera',
            'left_arm_rgb_stream': 'left_arm_camera',
            'right_arm_rgb_stream': 'right_arm_camera'
        }
        
        for config_name, data_keys in enabled_mapping.items():
            # 主臂数据的检查（仅告警，不强制要求，排除action数据）
            if config_name == 'enable_master_arm_data':
                # 只检查主臂的 joint_state 和 end_pose，不检查 action
                for key in data_keys:
                    # 跳过 action 相关的数据
                    if 'action' in key.lower():
                        continue
                    
                    # 检查传感器数据
                    if key in sensor_data:
                        if len(sensor_data[key]) == 0:
                            master_arm_missing_items.append((key, f"数据为空 ({len(sensor_data[key])} 条)"))
                    else:
                        available_keys = list(sensor_data.keys())[:10]  # 只显示前10个
                        master_arm_missing_items.append((key, f"数据不存在 (sensor_data中有: {available_keys})"))
                continue
            
            # 检查图像数据
            if any(key.endswith('_rgb_stream') or key.endswith('_depth_stream') or key.endswith('_depth_video') 
                   for key in data_keys):
                # 图像数据在images字典中（使用原始内部键名）
                for internal_key in data_keys:
                    if internal_key in images:
                        if len(images[internal_key]) == 0:
                            friendly_name = camera_name_mapping.get(internal_key, internal_key)
                            missing_data_items.append((config_name, friendly_name, "图像数据为空"))
                    else:
                        friendly_name = camera_name_mapping.get(internal_key, internal_key)
                        missing_data_items.append((config_name, friendly_name, f"图像数据不存在 (内部键: {internal_key})"))
            else:
                # 传感器数据在sensor_data字典中
                for key in data_keys:
                    if key in sensor_data:
                        if len(sensor_data[key]) == 0:
                            missing_data_items.append((config_name, key, f"数据为空 ({len(sensor_data[key])} 条)"))
                    else:
                        # 提供更详细的错误信息
                        available_keys = list(sensor_data.keys())[:10]  # 只显示前10个
                        missing_data_items.append((config_name, key, f"数据不存在 (sensor_data中有: {available_keys})"))
        
        # 如果有缺失的数据项，报错退出
        if missing_data_items:
            error_parts = ["错误: 以下启用的数据项没有采集到数据:\n"]
            
            # 按配置项分组
            by_config = {}
            for config_name, data_key, reason in missing_data_items:
                if config_name not in by_config:
                    by_config[config_name] = []
                by_config[config_name].append((data_key, reason))
            
            for config_name, items in by_config.items():
                error_parts.append(f"  - {config_name}:")
                for data_key, reason in items:
                    error_parts.append(f"    • {data_key}: {reason}")
            
            error_parts.append("\n请检查机器人连接和配置，确保所有启用的数据项都能正常采集。")
            
            error_msg = "\n".join(error_parts)
            print(f"  {error_msg}")
            return False
        
        # 主臂数据缺失告警（不报错退出）
        if master_arm_missing_items:
            print(f"\n  ⚠️  警告: enable_master_arm_data 已启用，但以下主臂数据项缺失（不影响数据采集）:")
            for data_key, reason in master_arm_missing_items:
                print(f"    • {data_key}: {reason}")
        
        return True
    
    def _validate_episode_data(self, episode_data):
        """验证episode数据质量"""
        if episode_data is None:
            return False
        
        # 从对齐后的数据结构中获取数据
        sensor_data = episode_data.get('sensor_data', {})
        action_data = episode_data.get('action_data', {})
        images = episode_data.get('images', {})
        
        # 获取启用的配置项映射
        enabled_mapping = self._get_enabled_data_mapping()
        
        # 检查所有启用的数据项是否都有数据
        missing_data_items = []
        
        # 相机名称映射：内部流名称 -> 用户友好名称
        camera_name_mapping = episode_data.get('camera_name_mapping', {
            'head_rgb_stream': 'head_camera',
            'head_depth_stream': 'head_depth_camera',
            'left_arm_rgb_stream': 'left_arm_camera',
            'right_arm_rgb_stream': 'right_arm_camera'
        })
        
        # 用于记录主臂数据的缺失情况（仅告警，不报错）
        master_arm_missing_items = []
        
        for config_name, data_keys in enabled_mapping.items():
            # 主臂数据的检查（仅告警，不强制要求）
            if config_name == 'enable_master_arm_data':
                # 检查图像数据
                if any(key.endswith('_rgb_stream') or key.endswith('_depth_stream') or key.endswith('_depth_video') 
                       for key in data_keys):
                    # 图像数据在images字典中（使用映射后的友好名称）
                    if self.use_video_storage and 'image_index_mapping' in episode_data:
                        # 视频存储模式：检查image_index_mapping
                        image_index_mapping = episode_data.get('image_index_mapping', {})
                        for internal_key in data_keys:
                            friendly_name = camera_name_mapping.get(internal_key, internal_key)
                            if friendly_name in image_index_mapping:
                                if len(image_index_mapping[friendly_name]) == 0:
                                    master_arm_missing_items.append((friendly_name, "图像数据为空"))
                            else:
                                master_arm_missing_items.append((friendly_name, f"图像数据不存在 (内部键: {internal_key})"))
                    else:
                        # 图像存储模式：检查aligned_images（在images字典中）
                        for internal_key in data_keys:
                            friendly_name = camera_name_mapping.get(internal_key, internal_key)
                            if friendly_name in images:
                                if len(images[friendly_name]) == 0:
                                    master_arm_missing_items.append((friendly_name, "图像数据为空"))
                            else:
                                # 也检查原始内部键名（在数据对齐之前）
                                if internal_key in images:
                                    if len(images[internal_key]) == 0:
                                        master_arm_missing_items.append((friendly_name, "图像数据为空"))
                                else:
                                    master_arm_missing_items.append((friendly_name, f"图像数据不存在 (内部键: {internal_key})"))
                else:
                    # 传感器数据在sensor_data或action_data字典中
                    # 只检查主臂的 joint_state 和 end_pose，不检查 action
                    for key in data_keys:
                        # 跳过 action 相关的数据
                        if 'action' in key.lower():
                            continue
                        
                        found = False
                        
                        # 先检查sensor_data
                        if key in sensor_data:
                            if len(sensor_data[key]) > 0:
                                found = True
                            else:
                                master_arm_missing_items.append((key, f"数据为空 ({len(sensor_data[key])} 条)"))
                                continue
                        
                        # 如果都没找到，记录告警
                        if not found:
                            available_sensor_keys = list(sensor_data.keys())[:10]  # 只显示前10个
                            error_info = f"数据不存在 (sensor_data中有: {available_sensor_keys})"
                            master_arm_missing_items.append((key, error_info))
                continue
            
            # 检查图像数据
            if any(key.endswith('_rgb_stream') or key.endswith('_depth_stream') or key.endswith('_depth_video') 
                   for key in data_keys):
                # 图像数据在images字典中（使用映射后的友好名称）
                if self.use_video_storage and 'image_index_mapping' in episode_data:
                    # 视频存储模式：检查image_index_mapping
                    image_index_mapping = episode_data.get('image_index_mapping', {})
                    for internal_key in data_keys:
                        friendly_name = camera_name_mapping.get(internal_key, internal_key)
                        if friendly_name in image_index_mapping:
                            if len(image_index_mapping[friendly_name]) == 0:
                                missing_data_items.append((config_name, friendly_name, "图像数据为空"))
                        else:
                            missing_data_items.append((config_name, friendly_name, f"图像数据不存在 (内部键: {internal_key})"))
                else:
                    # 图像存储模式：检查aligned_images（在images字典中）
                    for internal_key in data_keys:
                        friendly_name = camera_name_mapping.get(internal_key, internal_key)
                        if friendly_name in images:
                            if len(images[friendly_name]) == 0:
                                missing_data_items.append((config_name, friendly_name, "图像数据为空"))
                        else:
                            # 也检查原始内部键名（在数据对齐之前）
                            if internal_key in images:
                                if len(images[internal_key]) == 0:
                                    missing_data_items.append((config_name, friendly_name, "图像数据为空"))
                            else:
                                missing_data_items.append((config_name, friendly_name, f"图像数据不存在 (内部键: {internal_key})"))
            else:
                # 传感器数据在sensor_data或action_data字典中
                for key in data_keys:
                    found = False
                    data_source = None
                    
                    # 先检查sensor_data
                    if key in sensor_data:
                        if len(sensor_data[key]) > 0:
                            found = True
                            data_source = "sensor_data"
                        else:
                            missing_data_items.append((config_name, key, f"数据为空 ({len(sensor_data[key])} 条)"))
                            continue
                    
                    # 如果不在sensor_data中，检查action_data（某些数据可能在那里）
                    if not found and key in action_data:
                        if len(action_data[key]) > 0:
                            found = True
                            data_source = "action_data"
                        else:
                            missing_data_items.append((config_name, key, f"数据为空 ({len(action_data[key])} 条)"))
                            continue
                    
                    # 如果都没找到，报错
                    if not found:
                        # 提供更详细的错误信息
                        available_sensor_keys = list(sensor_data.keys())[:10]  # 只显示前10个
                        available_action_keys = list(action_data.keys())[:10]
                        error_info = f"数据不存在 (sensor_data中有: {available_sensor_keys}, action_data中有: {available_action_keys})"
                        missing_data_items.append((config_name, key, error_info))
        
        # 如果有缺失的数据项，报错退出
        if missing_data_items:
            error_parts = ["错误: 以下启用的数据项没有采集到数据:\n"]
            
            # 按配置项分组
            by_config = {}
            for config_name, data_key, reason in missing_data_items:
                if config_name not in by_config:
                    by_config[config_name] = []
                by_config[config_name].append((data_key, reason))
            
            for config_name, items in by_config.items():
                error_parts.append(f"  - {config_name}:")
                for data_key, reason in items:
                    error_parts.append(f"    • {data_key}: {reason}")
            
            error_parts.append("\n请检查机器人连接和配置，确保所有启用的数据项都能正常采集。")
            
            error_msg = "\n".join(error_parts)
            # 清理临时文件后再退出
            self._cleanup_before_exit(error_msg)
            sys.exit(1)
        
        # 主臂数据缺失告警（不报错退出）
        # 注意：告警只在 _validate_collected_data 中打印，这里不再重复打印，避免重复
        
        # 使用成员变量中的关节状态名称（用于后续统计）
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        
        # 检查是否有任何关节状态数据（兼容旧格式）
        states = sensor_data.get('joint_states', [])  # 兼容旧格式
        actions = sensor_data.get('actions', [])  # 兼容旧格式
        
        # 检查各部位的关节状态数据
        joint_states_by_part = {}
        for joint_state_name in joint_state_names:
            if joint_state_name in sensor_data and len(sensor_data[joint_state_name]) > 0:
                joint_states_by_part[joint_state_name] = sensor_data[joint_state_name]
        
        # 检查是否有任何数据
        has_joint_data = len(states) > 0 or len(actions) > 0 or len(joint_states_by_part) > 0
        has_image_data = any(len(imgs) > 0 for imgs in images.values())
        
        if not has_joint_data and not has_image_data:
            print("  验证失败: 没有任何数据")
            return False

        # 获取状态帧数（优先使用各部位的数据）
        if joint_states_by_part:
            # 使用各部位的数据
            state_frame_count = max(len(data) for data in joint_states_by_part.values())
        else:
            # 兼容旧格式
            state_frame_count = len(states) if states else 0

        # 如果启用了图像采集，检查图像数据
        if len(self.camera_names) > 0:
            # 视频存储模式：使用 image_index_mapping 验证
            if self.use_video_storage and 'image_index_mapping' in episode_data:
                image_index_mapping = episode_data['image_index_mapping']
                for cam_name, indices in image_index_mapping.items():
                    if len(indices) == 0:
                        print(f"  ⚠️  警告: {cam_name} 没有图像数据")
                    elif has_joint_data and state_frame_count > 0:
                        if abs(len(indices) - state_frame_count) > max(state_frame_count * 0.1, 1):
                            print(f"  ⚠️  警告: {cam_name} 图像数({len(indices)})与状态数({state_frame_count})差异较大")
            # 图像存储模式：检查 aligned_images
            else:
                for cam_name, cam_images in images.items():
                    if len(cam_images) == 0:
                        print(f"  ⚠️  警告: {cam_name} 没有图像数据")
                    elif has_joint_data and state_frame_count > 0:
                        if abs(len(cam_images) - state_frame_count) > max(state_frame_count * 0.1, 1):
                            print(f"  ⚠️  警告: {cam_name} 图像数({len(cam_images)})与状态数({state_frame_count})差异较大")
        
        # 检查数据量
        max_frames = max(state_frame_count, max((len(imgs) for imgs in images.values()), default=0))
        min_frames = int(self.target_hz * 0.5)  # 至少0.5秒的数据
        if max_frames < min_frames:
            print(f"  ⚠️  警告: 数据量较少 ({max_frames} 帧 < {min_frames} 帧)")
        
        # 打印各部位的数据统计
        if joint_states_by_part:
            print(f"  ✓ 数据验证完成: {len(joint_states_by_part)} 个部位的关节状态, {state_frame_count} 状态帧, {len(images)} 个相机")
            for part_name, data in joint_states_by_part.items():
                print(f"    - {part_name}: {len(data)} 帧")
        else:
            print(f"  ✓ 数据验证完成: {state_frame_count} 状态帧, {len(images)} 个相机")
        return True
    
    def _clear_queues(self):
        """清空所有队列"""
        # 只清空实际存在的队列
        for queue_name, queue in self.queues.items():
            if queue is not None:
                try:
                    while not queue.empty():
                        queue.get_nowait()  # 非阻塞获取，避免阻塞
                except:
                    pass  # 队列可能在其他地方被修改，忽略错误
    
    def _create_temp_files(self):
        """为相机和高频数据创建临时文件"""
        # 确保所有临时文件字典都已初始化
        if not hasattr(self, 'image_temp_files'):
            self.image_temp_files = {}
        if not hasattr(self, 'image_temp_paths'):
            self.image_temp_paths = {}
        if not hasattr(self, 'sensor_temp_files'):
            self.sensor_temp_files = {}
        if not hasattr(self, 'sensor_temp_paths'):
            self.sensor_temp_paths = {}

        self._cleanup_temp_files()
        
        # 相机临时文件
        for camera_name in self.camera_names:
            temp_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{camera_name}.pkl')
            self.image_temp_files[camera_name] = temp_file
            self.image_temp_paths[camera_name] = temp_file.name

        # 传感器数据临时文件
        for sensor_name in self.sensor_file_locks.keys():
            temp_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{sensor_name}.pkl')
            self.sensor_temp_files[sensor_name] = temp_file
            self.sensor_temp_paths[sensor_name] = temp_file.name

        # 关节状态和动作数据临时文件 - 根据配置的 joint_names 创建
        if self.slave_joint_names:
            for joint_state_name in self.slave_joint_names:
                joint_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{joint_state_name}.pkl')
                self.sensor_temp_files[joint_state_name] = joint_file
                self.sensor_temp_paths[joint_state_name] = joint_file.name
                
                # 同时创建对应的action文件（用于future state作为action）
                if self.slave_action_names:
                    # 找到对应的 action_name
                    action_name = joint_state_name.replace('_joint_states', '_actions')
                    if action_name in self.slave_action_names:
                        action_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{action_name}.pkl')
                        self.sensor_temp_files[action_name] = action_file
                        self.sensor_temp_paths[action_name] = action_file.name
    
    def _cleanup_temp_files(self):
        """清理临时文件"""
        cleaned_count = 0
        
        # 清理相机临时文件
        if hasattr(self, 'image_temp_files'):
            for camera_name in list(self.image_temp_files.keys()):
                if camera_name in self.image_temp_files:
                    try:
                        if not self.image_temp_files[camera_name].closed:
                            self.image_temp_files[camera_name].close()
                    except Exception:
                        pass  # 忽略关闭错误
                    try:
                        if hasattr(self, 'image_temp_paths') and camera_name in self.image_temp_paths:
                            temp_path = self.image_temp_paths[camera_name]
                            if temp_path and os.path.exists(temp_path):
                                os.unlink(temp_path)
                                cleaned_count += 1
                    except Exception as e:
                        print(f"清理相机临时文件错误 ({camera_name}): {e}")

        # 清理传感器数据临时文件（现在包括关节状态和动作）
        if hasattr(self, 'sensor_temp_files'):
            for sensor_name in list(self.sensor_temp_files.keys()):
                if sensor_name in self.sensor_temp_files:
                    try:
                        if not self.sensor_temp_files[sensor_name].closed:
                            self.sensor_temp_files[sensor_name].close()
                    except Exception:
                        pass  # 忽略关闭错误
                    try:
                        if hasattr(self, 'sensor_temp_paths') and sensor_name in self.sensor_temp_paths:
                            temp_path = self.sensor_temp_paths[sensor_name]
                            if temp_path and os.path.exists(temp_path):
                                os.unlink(temp_path)
                                cleaned_count += 1
                    except Exception as e:
                        print(f"清理传感器临时文件错误 ({sensor_name}): {e}")

        # 清空字典（如果存在）
        if hasattr(self, 'image_temp_files'):
            self.image_temp_files.clear()
        if hasattr(self, 'image_temp_paths'):
            self.image_temp_paths.clear()
        if hasattr(self, 'sensor_temp_files'):
            self.sensor_temp_files.clear()
        if hasattr(self, 'sensor_temp_paths'):
            self.sensor_temp_paths.clear()
        
        # 额外清理：扫描临时目录中可能遗留的临时文件（以我们的后缀结尾的）
        # 这可以清理程序异常退出时遗留的文件
        try:
            temp_dir = tempfile.gettempdir()
            if os.path.isdir(temp_dir):
                # 查找所有以我们的后缀结尾的pkl文件
                suffixes_to_clean = []
                if hasattr(self, 'camera_names'):
                    suffixes_to_clean.extend([f'_{cam}.pkl' for cam in self.camera_names])
                if hasattr(self, 'sensor_file_locks'):
                    suffixes_to_clean.extend([f'_{sensor}.pkl' for sensor in self.sensor_file_locks.keys()])
                if hasattr(self, 'slave_joint_names') and self.slave_joint_names:
                    suffixes_to_clean.extend([f'_{name}.pkl' for name in self.slave_joint_names])
                    if hasattr(self, 'slave_action_names') and self.slave_action_names:
                        suffixes_to_clean.extend([f'_{name}.pkl' for name in self.slave_action_names])
                
                for filename in os.listdir(temp_dir):
                    if filename.startswith('tmp') and filename.endswith('.pkl'):
                        # 检查是否匹配我们的后缀
                        for suffix in suffixes_to_clean:
                            if filename.endswith(suffix):
                                temp_file_path = os.path.join(temp_dir, filename)
                                try:
                                    # 检查文件是否很旧（超过1小时）或者是0字节
                                    file_stat = os.stat(temp_file_path)
                                    file_age = time.time() - file_stat.st_mtime
                                    if file_age > 3600 or file_stat.st_size == 0:
                                        os.unlink(temp_file_path)
                                        cleaned_count += 1
                                except Exception:
                                    pass  # 忽略删除错误
                                break
        except Exception as e:
            # 忽略扫描错误，不影响主流程
            pass
        
        if cleaned_count > 0:
            print(f"✓ 已清理 {cleaned_count} 个临时文件")
    
    def _collect_episode_data(self):
        """从队列和临时文件中收集episode数据"""
        # 从传感器临时文件中收集数据
        sensor_data = {}
        episode_data = {}

        for sensor_name in self.sensor_file_locks.keys():
            if sensor_name not in self.sensor_temp_files:
                continue

            # 关闭写入句柄
            with self.sensor_file_locks[sensor_name]:
                self.sensor_temp_files[sensor_name].close()

            # 重新打开文件进行读取
            temp_path = self.sensor_temp_paths[sensor_name]
            if not os.path.exists(temp_path):
                print(f"⚠️  警告: 传感器临时文件不存在 {sensor_name}: {temp_path}")
                continue

            data_list = []
            try:
                with open(temp_path, 'rb') as f:
                    while True:
                        try:
                            data = pickle.load(f)
                            data_list.append(data)
                        except EOFError:
                            break  # 文件结束
                        except Exception as e:
                            print(f"读取{sensor_name}数据错误: {e}")
                            break

                if data_list:
                    sensor_data[sensor_name] = data_list
                    print(f"  {sensor_name}: 收集 {len(data_list)} 条数据")

            except Exception as e:
                print(f"读取{sensor_name}临时文件错误: {e}")
        
        # 从临时文件读取各部位的关节状态和动作数据
        print("正在从临时文件读取各部位的关节状态和动作数据...")

        # 使用成员变量中的关节状态和动作名称
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        action_names = self.slave_action_names if self.slave_action_names else []

        # 读取各部位的关节状态数据
        for data_type in joint_state_names:
            if not hasattr(self, 'sensor_temp_files') or data_type not in self.sensor_temp_files:
                continue

            # 关闭写入句柄
            if data_type in self.sensor_file_locks:
                with self.sensor_file_locks[data_type]:
                    if data_type in self.sensor_temp_files:
                        self.sensor_temp_files[data_type].close()
            else:
                if data_type in self.sensor_temp_files:
                    self.sensor_temp_files[data_type].close()

            # 重新打开文件进行读取
            temp_path = self.sensor_temp_paths.get(data_type)
            if not temp_path:
                print(f"⚠️  警告: {data_type} 的临时文件路径不存在")
                continue
            if not os.path.exists(temp_path):
                print(f"⚠️  警告: 关节状态临时文件不存在 {data_type}: {temp_path}")
                continue

            data_list = []
            try:
                with open(temp_path, 'rb') as f:
                    while True:
                        try:
                            data = pickle.load(f)
                            data_list.append(data)
                        except EOFError:
                            break  # 文件结束
                        except Exception as e:
                            print(f"读取{data_type}数据错误: {e}")
                            break

                if data_list:
                    sensor_data[data_type] = data_list
                    print(f"  {data_type}: 收集 {len(data_list)} 条数据")
                else:
                    print(f"  ⚠️  警告: {data_type} 临时文件存在但数据为空")

            except Exception as e:
                print(f"读取{data_type}临时文件错误: {e}")
                import traceback
                traceback.print_exc()
        
        # 读取各部位的动作数据
        for data_type in action_names:
            if not hasattr(self, 'sensor_temp_files') or data_type not in self.sensor_temp_files:
                continue

            # 关闭写入句柄
            if data_type in self.sensor_file_locks:
                with self.sensor_file_locks[data_type]:
                    if data_type in self.sensor_temp_files:
                        self.sensor_temp_files[data_type].close()
            else:
                if data_type in self.sensor_temp_files:
                    self.sensor_temp_files[data_type].close()

            # 重新打开文件进行读取
            temp_path = self.sensor_temp_paths[data_type]
            if not os.path.exists(temp_path):
                print(f"⚠️  警告: 动作数据临时文件不存在 {data_type}: {temp_path}")
                continue

            data_list = []
            try:
                with open(temp_path, 'rb') as f:
                    while True:
                        try:
                            data = pickle.load(f)
                            data_list.append(data)
                        except EOFError:
                            break  # 文件结束
                        except Exception as e:
                            print(f"读取{data_type}数据错误: {e}")
                            break

                if data_list:
                    sensor_data[data_type] = data_list
                    print(f"  {data_type}: 收集 {len(data_list)} 条数据")
                else:
                    # 跳过 action 数据的警告（action数据不是必需的）
                    if 'action' not in data_type.lower():
                        print(f"  ⚠️  {data_type}: 临时文件存在但数据为空")

            except Exception as e:
                print(f"读取{data_type}临时文件错误: {e}")
                import traceback
                traceback.print_exc()
        
        # 优化图像数据读取：使用视频存储时不加载所有图像到内存
        if self.use_video_storage:
            # 视频模式：只读取时间戳信息，不加载图像数据
            images = defaultdict(list)
            print("正在从临时文件读取图像时间戳...")
            
            for camera_name in self.camera_names:
                if camera_name not in self.image_temp_files:
                    continue
                
                # 关闭写入句柄
                with self.image_file_locks[camera_name]:
                    self.image_temp_files[camera_name].close()
                
                # 重新打开文件进行读取，只读取时间戳
                temp_path = self.image_temp_paths[camera_name]
                if not os.path.exists(temp_path):
                    print(f"⚠️  警告: 临时文件不存在 {camera_name}: {temp_path}")
                    continue
                
                try:
                    with open(temp_path, 'rb') as f:
                        while True:
                            try:
                                timestamp, img_data = pickle.load(f)
                                # 只保存时间戳，不保存图像数据以节省内存
                                # img_data 现在是压缩后的JPEG bytes，不需要加载
                                images[camera_name].append((timestamp, None))
                            except EOFError:
                                break
                    
                    print(f"  {camera_name}: 读取 {len(images[camera_name])} 帧时间戳")
                
                except Exception as e:
                    print(f"读取临时文件错误 ({camera_name}): {e}")
        else:
            # 图像模式：正常加载所有图像
            images = defaultdict(list)
            print("正在从临时文件读取图像数据...")
            
            for camera_name in self.camera_names:
                if camera_name not in self.image_temp_files:
                    continue
                
                # 关闭写入句柄
                with self.image_file_locks[camera_name]:
                    self.image_temp_files[camera_name].close()
                
                # 重新打开文件进行读取
                temp_path = self.image_temp_paths[camera_name]
                if not os.path.exists(temp_path):
                    print(f"⚠️  警告: 临时文件不存在 {camera_name}: {temp_path}")
                    continue
                
                try:
                    with open(temp_path, 'rb') as f:
                        while True:
                            try:
                                timestamp, img_data = pickle.load(f)
                                # img_data 是压缩后的JPEG bytes，需要转换回PIL Image
                                if isinstance(img_data, bytes):
                                    # 压缩的JPEG数据，需要解码
                                    img = Image.open(io.BytesIO(img_data))
                                    if img.mode != 'RGB':
                                        img = img.convert('RGB')
                                else:
                                    # 兼容旧格式：如果已经是PIL Image对象（向后兼容）
                                    img = img_data
                                images[camera_name].append((timestamp, img))
                            except EOFError:
                                break
                    
                    print(f"  {camera_name}: 读取 {len(images[camera_name])} 帧")
                
                except Exception as e:
                    print(f"读取临时文件错误 ({camera_name}): {e}")
        
        # 在数据对齐前验证：检查是否所有启用的数据项都有实际数据
        # 这样可以避免在对齐过程中丢失数据导致的误报
        validation_result = self._validate_collected_data(sensor_data, images)
        if not validation_result:
            print("错误: 数据收集验证失败，某些启用的数据项没有实际数据")
            return None
        
        # 基于时间戳的插值对齐
        print("正在进行基于时间戳的数据对齐...")
        episode_data = self._align_data_by_timestamp(sensor_data, images)
        
        if episode_data is None:
            print("⚠️  警告: 数据对齐失败，返回None")
            return None
        
        # 注意：使用视频存储时，不在这里清理临时文件，在保存完视频后再清理
        if not self.use_video_storage:
            self._cleanup_temp_files()
        
        return episode_data
    
    def _align_data_by_timestamp(self, sensor_data, images):
        """基于时间戳对齐所有数据到统一的时间网格"""

        # 相机名称映射：内部流名称 -> 用户友好名称
        camera_name_mapping = {
            'head_rgb_stream': 'head_camera',
            'head_depth_stream': 'head_depth_camera',
            'left_arm_rgb_stream': 'left_arm_camera',
            'right_arm_rgb_stream': 'right_arm_camera'
        }

        # 1. 以主摄像头（头部摄像头）的时间戳为时间线对齐图像数据帧
        # 首先找到主摄像头的内部键名
        head_camera_internal_name = None
        for internal_name, friendly_name in camera_name_mapping.items():
            if friendly_name == 'head_camera' and internal_name in images and images[internal_name]:
                head_camera_internal_name = internal_name
                break

        if not head_camera_internal_name:
            print("⚠️  警告: 主摄像头（head_camera）无数据，无法进行时间对齐")
            print(f"   可用图像流: {list(images.keys())}")
            return None

        # 使用主摄像头的时间戳作为基准时间线
        target_timestamps = [t for t, _ in images[head_camera_internal_name]]
        num_frames = len(target_timestamps)

        print(f"  使用主摄像头（{head_camera_internal_name}）时间戳作为基准: {num_frames} 帧")

        # 统计各传感器数据量
        sensor_stats = [(name, len(data_list)) for name, data_list in sensor_data.items() if data_list]
        hf_stats = [(name, len(data_list)) for name, data_list in sensor_data.items() if data_list and name in ['joint_states', 'actions']]
        image_stats = [(cam, len(imgs)) for cam, imgs in images.items() if imgs]
        print(f"  传感器数据: {sensor_stats}")
        print(f"  高频数据: {hf_stats}")
        print(f"  图像数据: {image_stats}")

        # 2. 对每种传感器数据进行时间对齐
        aligned_sensor_data = {}
        aligned_action_data = {}

        # 使用成员变量中的关节状态和动作名称
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        action_names = self.slave_action_names if self.slave_action_names else []
        
        for sensor_name, data_list in sensor_data.items():
            if data_list:
                # 降采样处理时，关节状态和末端位姿需要线性插值
                if sensor_name in joint_state_names + action_names and self.downsample_joint_states:
                    print(f"  对 {sensor_name} 使用线性插值进行时间对齐")
                    if sensor_name in action_names:
                        aligned_action_data[sensor_name] = self._interpolate_linear(data_list, target_timestamps)
                    else:
                        aligned_sensor_data[sensor_name] = self._interpolate_linear(data_list, target_timestamps)
                else:
                    # 其他传感器数据使用最近邻插值
                    if sensor_name in action_names:
                        aligned_action_data[sensor_name] = self._interpolate_nearest(data_list, target_timestamps)
                    else:
                        aligned_sensor_data[sensor_name] = self._interpolate_nearest(data_list, target_timestamps)

        # 对图像数据进行时间对齐（使用最近邻插值，因为图像数据已经是基准）
        aligned_images = {}
        image_index_mapping = {}  # 用于视频模式的帧索引映射
        
        if self.use_video_storage:
            # 视频模式：只创建时间戳到帧索引的映射，不加载图像
            for cam_name, cam_imgs in images.items():
                friendly_name = camera_name_mapping.get(cam_name, cam_name)
                if cam_name == head_camera_internal_name:
                    # 主摄像头：创建1:1映射
                    image_index_mapping[friendly_name] = list(range(len(cam_imgs)))
                    aligned_images[friendly_name] = []  # 空列表
                else:
                    # 其他摄像头：创建时间戳对齐的索引映射
                    cam_timestamps = [t for t, _ in cam_imgs]
                    indices = self._interpolate_nearest_indices(cam_timestamps, target_timestamps)
                    image_index_mapping[friendly_name] = indices
                    aligned_images[friendly_name] = []  # 空列表
        else:
            # 图像模式：正常对齐图像数据
            for cam_name, cam_imgs in images.items():
                friendly_name = camera_name_mapping.get(cam_name, cam_name)
                if cam_name == head_camera_internal_name:
                    # 主摄像头直接使用，不需要插值
                    aligned_images[friendly_name] = [img for _, img in cam_imgs]
                else:
                    # 其他摄像头使用最近邻插值对齐到主摄像头时间戳
                    aligned_images[friendly_name] = self._interpolate_nearest(cam_imgs, target_timestamps)

        print(f"  对齐后帧数: {len(target_timestamps)}")

        # 确保joint_names不为None - 从各部位的关节状态数据推断
        joint_names = self.joint_names
        if not joint_names or not isinstance(joint_names, dict):
            # 优先使用保存的真实关节名称
            if hasattr(self, '_joint_names_by_part') and self._joint_names_by_part:
                joint_names = {}
                for joint_state_name, names in self._joint_names_by_part.items():
                    part_name = joint_state_name.replace('_joint_states', '')
                    joint_names[part_name] = names
                print(f"✓ 使用保存的关节名称: {len(joint_names)} 个部位")
                for part_name, names in joint_names.items():
                    print(f"    - {part_name}: {names}")
            else:
                # 从各部位的关节状态数据推断关节名称（按部位分开）
                joint_names = {}
                for joint_state_name in joint_state_names:
                    if joint_state_name in aligned_sensor_data and len(aligned_sensor_data[joint_state_name]) > 0:
                        # 从第一帧数据推断关节数量
                        first_state = aligned_sensor_data[joint_state_name][0]
                        if isinstance(first_state, tuple) and len(first_state) > 0:
                            num_joints = len(first_state[0])
                        else:
                            num_joints = len(first_state) if hasattr(first_state, '__len__') else 0
                        
                        # 为每个部位生成默认关节名称
                        part_name = joint_state_name.replace('_joint_states', '')
                        part_joint_names = []
                        for i in range(num_joints):
                            part_joint_names.append(f"{part_name}_joint{i+1}")
                        joint_names[part_name] = part_joint_names
                
                if joint_names:
                    total_joints = sum(len(names) for names in joint_names.values())
                    print(f"⚠️  joint_names为空，从数据推断创建默认名称: {len(joint_names)} 个部位, 共 {total_joints} 个关节")
                    for part_name, names in joint_names.items():
                        print(f"    - {part_name}: {len(names)} 个关节")

        episode_data = {
            'timestamps': target_timestamps,
            'sensor_data': aligned_sensor_data,  # 普通传感器数据
            'action_data': aligned_action_data,  # Action数据
            'images': aligned_images,  # 图像数据
            'joint_names': joint_names,
        }
        
        # 视频模式：添加图像索引映射和原始摄像头名称映射
        if self.use_video_storage:
            episode_data['image_index_mapping'] = image_index_mapping
            episode_data['camera_name_mapping'] = camera_name_mapping

        print("✓ 时间对齐完成")
        return episode_data
    
    def _interpolate_linear(self, data_with_timestamps, target_timestamps):
        """使用线性插值将数据对齐到目标时间戳"""
        import numpy as np

        if not data_with_timestamps:
            return []

        sorted_data = sorted(data_with_timestamps, key=lambda x: x[0])
        aligned_data = []

        # 检查数据格式：关节状态是4元素元组，其他是2元素元组
        first_item = sorted_data[0]
        is_joint_state = len(first_item) == 4  # 关节状态格式: (timestamp, positions, velocities, efforts)

        # 提取时间戳和数据
        timestamps = np.array([item[0] for item in sorted_data])

        if is_joint_state:
            # 关节状态数据：positions, velocities, efforts
            positions = np.array([item[1] for item in sorted_data])
            velocities = np.array([item[2] for item in sorted_data])
            efforts = np.array([item[3] for item in sorted_data])
        else:
            # 其他数据格式
            data_values = np.array([item[1] for item in sorted_data])

        for target_t in target_timestamps:
            if target_t <= timestamps[0]:
                # 使用第一个数据点
                if is_joint_state:
                    aligned_data.append((sorted_data[0][1], sorted_data[0][2], sorted_data[0][3]))
                else:
                    aligned_data.append(sorted_data[0][1])
            elif target_t >= timestamps[-1]:
                # 使用最后一个数据点
                if is_joint_state:
                    aligned_data.append((sorted_data[-1][1], sorted_data[-1][2], sorted_data[-1][3]))
                else:
                    aligned_data.append(sorted_data[-1][1])
            else:
                # 线性插值
                # 找到target_t所在的区间
                idx = np.searchsorted(timestamps, target_t) - 1
                if idx < 0:
                    idx = 0

                t1, t2 = timestamps[idx], timestamps[idx + 1]
                ratio = (target_t - t1) / (t2 - t1) if t2 != t1 else 0

                if is_joint_state:
                    # 对positions, velocities, efforts分别进行线性插值
                    pos1, pos2 = positions[idx], positions[idx + 1]
                    vel1, vel2 = velocities[idx], velocities[idx + 1]
                    eff1, eff2 = efforts[idx], efforts[idx + 1]

                    interpolated_pos = pos1 + ratio * (pos2 - pos1)
                    interpolated_vel = vel1 + ratio * (vel2 - vel1)
                    interpolated_eff = eff1 + ratio * (eff2 - eff1)

                    aligned_data.append((interpolated_pos, interpolated_vel, interpolated_eff))
                else:
                    # 对单个数值或数组进行线性插值
                    val1, val2 = data_values[idx], data_values[idx + 1]
                    interpolated_val = val1 + ratio * (val2 - val1)
                    aligned_data.append(interpolated_val)

        return aligned_data

    def _interpolate_nearest(self, data_with_timestamps, target_timestamps):
        """使用最近邻插值将数据对齐到目标时间戳"""
        if not data_with_timestamps:
            return []

        sorted_data = sorted(data_with_timestamps, key=lambda x: x[0])
        aligned_data = []
        data_idx = 0

        for target_t in target_timestamps:
            min_diff = float('inf')
            # 检查数据格式：关节状态是4元素元组，其他是2元素元组
            first_item = sorted_data[0]
            if len(first_item) == 4:  # 关节状态格式: (timestamp, positions, velocities, efforts)
                closest_data = (first_item[1], first_item[2], first_item[3])  # positions, velocities, efforts
            else:  # 其他格式: (timestamp, data)
                closest_data = first_item[1]

            for i in range(data_idx, len(sorted_data)):
                item = sorted_data[i]
                t = item[0]
                diff = abs(t - target_t)

                if diff < min_diff:
                    min_diff = diff
                    if len(item) == 4:  # 关节状态格式
                        closest_data = (item[1], item[2], item[3])  # positions, velocities, efforts
                    else:  # 其他格式
                        closest_data = item[1]
                    data_idx = i
                else:
                    break

            aligned_data.append(closest_data)

        return aligned_data
    
    def _interpolate_nearest_indices(self, timestamps, target_timestamps):
        """使用最近邻插值返回索引映射（用于视频模式）"""
        if not timestamps:
            return []
        
        indices = []
        data_idx = 0
        
        for target_t in target_timestamps:
            min_diff = float('inf')
            closest_idx = 0
            
            for i in range(data_idx, len(timestamps)):
                t = timestamps[i]
                diff = abs(t - target_t)
                
                if diff < min_diff:
                    min_diff = diff
                    closest_idx = i
                    data_idx = i
                else:
                    break
            
            indices.append(closest_idx)
        
        return indices
    
    def _create_video_with_ffmpeg(self, temp_path: str, indices: List[int], output_path: str, fps: float, expected_frames: int) -> bool:
        """使用 ffmpeg 创建视频（流式处理，节省内存）
        
        Args:
            temp_path: 临时图像文件路径
            indices: 索引映射列表，指定每一帧应该使用哪个原始图像
            output_path: 输出视频路径
            fps: 帧率
            expected_frames: 期望的帧数
            
        Returns:
            成功返回 True，失败返回 False
        """
        if not indices or not os.path.exists(temp_path):
            print(f"  ⚠️  警告: 没有图像数据或临时文件不存在，无法创建视频")
            return False
        
        try:
            # 构建 ffmpeg 命令
            ffmpeg_cmd = [
                "ffmpeg",
                "-r", str(fps),
                "-f", "image2pipe",
                "-vcodec", "mjpeg",
                "-pix_fmt", "yuvj420p",
                "-i", "-",
                "-c:v", "libx264",
                "-pix_fmt", "yuv420p",
                "-preset", "medium",
                "-crf", "23",
                "-y",
                output_path
            ]
            
            # 启动 ffmpeg 进程
            process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 首先加载所有原始图像到内存（避免重复读取文件）
            # 但这是按摄像头分开的，所以内存压力小很多
            all_images = []
            with open(temp_path, 'rb') as f:
                while True:
                    try:
                        _, img_data = pickle.load(f)
                        # img_data 可能是压缩的JPEG bytes或PIL Image对象（向后兼容）
                        if isinstance(img_data, bytes):
                            # 压缩的JPEG数据，需要解码为PIL Image
                            img = Image.open(io.BytesIO(img_data))
                            if img.mode != 'RGB':
                                img = img.convert('RGB')
                        elif isinstance(img_data, Image.Image):
                            # 已经是PIL Image对象（向后兼容旧格式）
                            img = img_data
                            if img.mode != 'RGB':
                                img = img.convert('RGB')
                        else:
                            # 其他格式（如numpy数组）
                            img = img_data
                        all_images.append(img)
                    except EOFError:
                        break
            
            print(f"    加载了 {len(all_images)} 帧原始图像，需要对齐到 {expected_frames} 帧")
            
            # 获取第一帧的尺寸（用于创建黑色帧）
            first_img = all_images[0] if all_images else None
            if first_img is None:
                print(f"  ⚠️  警告: 没有有效的第一帧")
                process.stdin.close()
                process.kill()
                return False
            
            if isinstance(first_img, Image.Image):
                width, height = first_img.size
            elif isinstance(first_img, np.ndarray):
                height, width = first_img.shape[:2]
            else:
                print(f"  ⚠️  警告: 无法确定图像尺寸 (类型: {type(first_img)})")
                process.stdin.close()
                process.kill()
                return False
            
            # 创建黑色帧（用于错误帧）
            black_frame = Image.new('RGB', (width, height), (0, 0, 0))
            
            # 流式处理：逐帧读取、转换并写入
            frames_written = 0
            for frame_idx, img_idx in enumerate(indices):
                try:
                    pil_img = None
                    
                    # 获取对应的原始图像
                    if img_idx >= len(all_images):
                        print(f"  ⚠️  警告: 索引超出范围 {img_idx}/{len(all_images)}，使用黑色帧")
                        pil_img = black_frame
                    else:
                        img = all_images[img_idx]
                        
                        if img is None:
                            print(f"  ⚠️  警告: 帧{frame_idx} 图像为空，使用黑色帧")
                            pil_img = black_frame
                        else:
                            # 转换为 PIL Image
                            # img 应该已经是PIL Image（在加载时已处理）
                            if isinstance(img, Image.Image):
                                pil_img = img
                            elif isinstance(img, np.ndarray):
                                pil_img = Image.fromarray(img)
                            elif isinstance(img, bytes):
                                # 如果是bytes（不应该发生，因为在加载时已处理），尝试解码
                                pil_img = Image.open(io.BytesIO(img))
                                if pil_img.mode != 'RGB':
                                    pil_img = pil_img.convert('RGB')
                            else:
                                print(f"  ⚠️  警告: 帧{frame_idx} 格式不支持: {type(img)}，使用黑色帧")
                                pil_img = black_frame
                    
                    # 确保是 RGB 模式
                    if pil_img and pil_img.mode != 'RGB':
                        pil_img = pil_img.convert('RGB')
                    
                    # 保存为 JPEG 并写入管道
                    if pil_img:
                        buffer = io.BytesIO()
                        pil_img.save(buffer, format='JPEG', quality=95)
                        process.stdin.write(buffer.getvalue())
                        frames_written += 1
                        del buffer
                    
                    # 定期释放已处理的图像（如果不再需要）
                    if frame_idx % 100 == 99:
                        # 找出后续帧不会再用到的图像索引
                        remaining_indices = set(indices[frame_idx+1:])
                        for i in range(len(all_images)):
                            if i not in remaining_indices and i <= img_idx:
                                all_images[i] = None
                        
                        import gc
                        gc.collect()
                    
                except Exception as e:
                    print(f"  ⚠️  错误: 处理帧{frame_idx}失败: {e}")
                    # 即使出错也要写入黑色帧，保证帧数一致
                    try:
                        buffer = io.BytesIO()
                        black_frame.save(buffer, format='JPEG', quality=95)
                        process.stdin.write(buffer.getvalue())
                        frames_written += 1
                        del buffer
                    except:
                        pass
            
            print(f"    成功写入 {frames_written}/{expected_frames} 帧")
            
            # 清理
            del all_images
            
            # 关闭输入并等待完成
            try:
                # flush 并关闭 stdin
                if process.stdin and not process.stdin.closed:
                    process.stdin.flush()
                    process.stdin.close()
                
                # 使用 wait() 而不是 communicate()，因为我们已经写完了数据
                # communicate() 会尝试 flush 已关闭的 stdin，导致错误
                returncode = process.wait(timeout=60)
                
                # 读取 stderr 以获取错误信息
                if process.stderr:
                    stderr = process.stderr.read()
                else:
                    stderr = b''
                    
            except subprocess.TimeoutExpired:
                print(f"  ⚠️  ffmpeg 处理超时")
                try:
                    process.kill()
                    process.wait()
                except:
                    pass
                return False
            except Exception as e:
                print(f"  ⚠️  处理错误: {e}")
                try:
                    process.kill()
                    process.wait()
                except:
                    pass
                return False
            
            if returncode == 0:
                return True
            else:
                error_msg = stderr.decode('utf-8', errors='ignore') if stderr else 'Unknown error'
                print(f"  ⚠️  ffmpeg 错误 (返回码: {returncode}): {error_msg[:200]}")
                return False
                
        except FileNotFoundError:
            print(f"  ⚠️  ffmpeg 未安装，请安装: sudo apt-get install ffmpeg")
            return False
        except Exception as e:
            print(f"  ⚠️  创建视频失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _save_episode(self, episode_data: Dict[str, Any], task: str) -> Dict[str, Any]:
        """保存episode数据到JSON和图像文件或视频文件"""
        episode_id = self.episode_count
        episode_dir = self.output_dir / f"episode_{episode_id:04d}"
        episode_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"正在保存 Episode {episode_id} 到 {episode_dir}...")
        
        timestamps = episode_data['timestamps']
        sensor_data = episode_data['sensor_data']
        action_data = episode_data.get('action_data', {})
        images = episode_data['images']
        
        # 使用成员变量中的关节状态和动作名称
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        action_names = self.slave_action_names if self.slave_action_names else []
        
        # 收集各部位的关节状态数据
        joint_states_by_part = {}
        for joint_state_name in joint_state_names:
            if joint_state_name in sensor_data:
                joint_states_by_part[joint_state_name] = sensor_data[joint_state_name]
        
        # 收集各部位的动作数据（用于future state作为action）
        actions_by_part = {}
        for action_name in action_names:
            if action_name in action_data:
                actions_by_part[action_name] = action_data[action_name]
        
        # 构建关节名称字典（按部位分开，使用真实的关节名称）
        joint_names = {}
        for joint_state_name in joint_state_names:
            if joint_state_name in joint_states_by_part and len(joint_states_by_part[joint_state_name]) > 0:
                part_name = joint_state_name.replace('_joint_states', '')
                
                # 优先使用保存的真实关节名称
                if hasattr(self, '_joint_names_by_part') and joint_state_name in self._joint_names_by_part:
                    joint_names[part_name] = self._joint_names_by_part[joint_state_name]
                else:
                    # 如果没有保存的关节名称，从第一帧数据推断关节数量并生成默认名称
                    first_state = joint_states_by_part[joint_state_name][0]
                    if isinstance(first_state, tuple) and len(first_state) > 0:
                        num_joints = len(first_state[0])
                    else:
                        num_joints = len(first_state) if hasattr(first_state, '__len__') else 0
                    
                    # 为每个部位生成默认关节名称
                    part_joint_names = []
                    for i in range(num_joints):
                        part_joint_names.append(f"{part_name}_joint{i+1}")
                    joint_names[part_name] = part_joint_names
        
        if not joint_names:
            joint_names = {}
        
        num_frames = len(timestamps)
        
        # 构建episode JSON数据
        episode_json = {
            "episode_id": episode_id,
            "task": task,
            "timestamp": datetime.now().isoformat(),
            "duration": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0,
            "num_frames": num_frames,
            "joint_names": joint_names if joint_names else {},  # 按部位分开的字典
            "storage_format": "video" if self.use_video_storage else "images",
            "frames": []
        }
        
        # 如果使用视频存储，使用 ffmpeg 创建视频
        video_files = {}
        
        if self.use_video_storage:
            # 从episode_data中获取索引映射和摄像头名称映射
            image_index_mapping = episode_data.get('image_index_mapping', {})
            camera_name_mapping = episode_data.get('camera_name_mapping', {})
            
            # 反向映射：友好名称 -> 内部名称
            reverse_mapping = {v: k for k, v in camera_name_mapping.items()}
            
            # 使用 ffmpeg 创建视频（流式处理，节省内存）
            print(f"  开始创建视频文件...")
            for friendly_name in image_index_mapping.keys():
                internal_name = reverse_mapping.get(friendly_name, friendly_name)
                if internal_name not in self.image_temp_paths:
                    print(f"  ⚠️  警告: 找不到 {friendly_name} 的临时文件")
                    continue
                    
                temp_path = self.image_temp_paths[internal_name]
                if not os.path.exists(temp_path):
                    print(f"  ⚠️  警告: 临时文件不存在: {temp_path}")
                    continue
                
                # 获取索引映射
                indices = image_index_mapping[friendly_name]
                
                # 使用 ffmpeg 创建视频
                video_path = episode_dir / f"{friendly_name}.mp4"
                print(f"  正在使用 ffmpeg 创建视频: {friendly_name}.mp4...")
                
                try:
                    success = self._create_video_with_ffmpeg(
                        temp_path,
                        indices,
                        str(video_path),
                        self.target_hz,
                        num_frames
                    )
                    
                    if success:
                        video_files[friendly_name] = f"{friendly_name}.mp4"
                        print(f"  ✓ 视频创建成功: {friendly_name}.mp4 ({num_frames} 帧)")
                    else:
                        print(f"  ⚠️  视频创建失败: {friendly_name}")
                    
                except Exception as e:
                    print(f"  ⚠️  警告: 创建视频时发生错误 {friendly_name}: {e}")
                    import traceback
                    traceback.print_exc()
            
            # 更新 episode_json
            if video_files:
                episode_json["video_files"] = video_files
        
        # 保存每一帧
        import gc
        for i in range(num_frames):
            frame_data = {
                "frame_id": i,
                "timestamp": timestamps[i],
                "images": {}
            }

            # 添加各部位的关节状态数据
            if "observation" not in frame_data:
                frame_data["observation"] = {}
            
            for joint_state_name in joint_state_names:
                if joint_state_name in joint_states_by_part and i < len(joint_states_by_part[joint_state_name]):
                    # 提取state数据（positions, velocities, efforts）
                    state_data = joint_states_by_part[joint_state_name][i]
                    if isinstance(state_data, tuple):
                        positions, velocities, efforts = state_data
                    else:
                        positions = state_data
                        velocities = None
                        efforts = None
                    
                    # 转换为列表格式
                    positions_list = positions.tolist() if isinstance(positions, np.ndarray) else list(positions)
                    
                    # 保存各部位的关节状态
                    frame_data["observation"][joint_state_name] = {
                        "positions": positions_list
                    }
                    
                    # 添加可选的velocity和effort
                    if velocities is not None:
                        frame_data["observation"][joint_state_name]["velocities"] = velocities.tolist() if isinstance(velocities, np.ndarray) else list(velocities)
                    if efforts is not None:
                        frame_data["observation"][joint_state_name]["efforts"] = efforts.tolist() if isinstance(efforts, np.ndarray) else list(efforts)
            
            # 添加action数据（使用下一帧的state作为action）
            if "action" not in frame_data:
                frame_data["action"] = {}
            
            for joint_state_name in joint_state_names:
                part_name = joint_state_name.replace('_joint_states', '')
                action_name = f"{part_name}_actions"
                
                # 使用下一帧的state作为action
                if joint_state_name in joint_states_by_part:
                    if i + 1 < len(joint_states_by_part[joint_state_name]):
                        # 有下一帧，使用下一帧的state作为action
                        next_state_data = joint_states_by_part[joint_state_name][i + 1]
                        if isinstance(next_state_data, tuple):
                            next_positions, _, _ = next_state_data
                        else:
                            next_positions = next_state_data
                        
                        next_positions_list = next_positions.tolist() if isinstance(next_positions, np.ndarray) else list(next_positions)
                        frame_data["action"][action_name] = {
                            "positions": next_positions_list
                        }
                    elif i < len(joint_states_by_part[joint_state_name]):
                        # 最后一帧，使用当前帧的state作为action
                        current_state_data = joint_states_by_part[joint_state_name][i]
                        if isinstance(current_state_data, tuple):
                            current_positions, _, _ = current_state_data
                        else:
                            current_positions = current_state_data
                        
                        current_positions_list = current_positions.tolist() if isinstance(current_positions, np.ndarray) else list(current_positions)
                        frame_data["action"][action_name] = {
                            "positions": current_positions_list
                        }

            # 添加其他传感器数据
            for sensor_name, sensor_data_list in sensor_data.items():
                # 跳过已经处理过的关节状态数据
                if sensor_name in joint_state_names:
                    continue
                
                # 跳过空数据列表
                if not sensor_data_list:
                    continue

                if i < len(sensor_data_list):
                    sensor_value = sensor_data_list[i]

                    # 如果还没有observation字典，创建一个
                    if "observation" not in frame_data:
                        frame_data["observation"] = {}

                    # 根据传感器类型处理数据
                    if sensor_name.endswith('_end_pose'):
                        # 末端位姿数据
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            pose_data = sensor_value['data']
                            frame_data["observation"][sensor_name] = {
                                'position': {
                                    'x': pose_data.position.x if hasattr(pose_data, 'position') else 0,
                                    'y': pose_data.position.y if hasattr(pose_data, 'position') else 0,
                                    'z': pose_data.position.z if hasattr(pose_data, 'position') else 0,
                                },
                                'orientation': {
                                    'x': pose_data.orientation.x if hasattr(pose_data, 'orientation') else 0,
                                    'y': pose_data.orientation.y if hasattr(pose_data, 'orientation') else 0,
                                    'z': pose_data.orientation.z if hasattr(pose_data, 'orientation') else 0,
                                    'w': pose_data.orientation.w if hasattr(pose_data, 'orientation') else 1,
                                }
                            }
                        else:
                            # 如果数据格式不同，直接保存
                            frame_data["observation"][sensor_name] = sensor_value
                        
                        # 添加end_pose的action（使用下一帧的数据）
                        if i + 1 < len(sensor_data_list):
                            next_sensor_value = sensor_data_list[i + 1]
                            if isinstance(next_sensor_value, dict) and 'data' in next_sensor_value:
                                next_pose_data = next_sensor_value['data']
                                action_name = sensor_name.replace('_end_pose', '_end_pose_action')
                                frame_data["action"][action_name] = {
                                    'position': {
                                        'x': next_pose_data.position.x if hasattr(next_pose_data, 'position') else 0,
                                        'y': next_pose_data.position.y if hasattr(next_pose_data, 'position') else 0,
                                        'z': next_pose_data.position.z if hasattr(next_pose_data, 'position') else 0,
                                    },
                                    'orientation': {
                                        'x': next_pose_data.orientation.x if hasattr(next_pose_data, 'orientation') else 0,
                                        'y': next_pose_data.orientation.y if hasattr(next_pose_data, 'orientation') else 0,
                                        'z': next_pose_data.orientation.z if hasattr(next_pose_data, 'orientation') else 0,
                                        'w': next_pose_data.orientation.w if hasattr(next_pose_data, 'orientation') else 1,
                                    }
                                }
                            else:
                                # 如果没有下一帧或格式不同，使用当前帧作为action
                                action_name = sensor_name.replace('_end_pose', '_end_pose_action')
                                frame_data["action"][action_name] = frame_data["observation"][sensor_name]
                        else:
                            # 最后一帧，使用当前帧作为action
                            action_name = sensor_name.replace('_end_pose', '_end_pose_action')
                            frame_data["action"][action_name] = frame_data["observation"][sensor_name]

                    elif sensor_name == 'odometry':
                        # 里程计数据
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            odom_data = sensor_value['data']
                            frame_data["observation"][sensor_name] = {
                                'pose': {
                                    'position': {
                                        'x': odom_data.pose.pose.position.x if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 0,
                                        'y': odom_data.pose.pose.position.y if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 0,
                                        'z': odom_data.pose.pose.position.z if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 0,
                                    },
                                    'orientation': {
                                        'x': odom_data.pose.pose.orientation.x if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 0,
                                        'y': odom_data.pose.pose.orientation.y if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 0,
                                        'z': odom_data.pose.pose.orientation.z if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 0,
                                        'w': odom_data.pose.pose.orientation.w if hasattr(odom_data, 'pose') and hasattr(odom_data.pose, 'pose') else 1,
                                    }
                                },
                                'twist': {
                                    'linear': {
                                        'x': odom_data.twist.twist.linear.x if hasattr(odom_data, 'twist') and hasattr(odom_data.twist, 'twist') else 0,
                                        'y': odom_data.twist.twist.linear.y if hasattr(odom_data, 'twist') and hasattr(odom_data.twist, 'twist') else 0,
                                        'z': odom_data.twist.twist.linear.z if hasattr(odom_data, 'twist') and hasattr(odom_data.twist, 'twist') else 0,
                                    },
                                    'angular': {
                                        'x': odom_data.twist.twist.angular.x if hasattr(odom_data, 'twist') and hasattr(odom_data.twist, 'twist') else 0,
                                        'y': odom_data.twist.twist.angular.y if hasattr(odom_data, 'twist') and hasattr(odom_data.twist, 'twist') else 0,
                                        'z': odom_data.twist.twist.angular.z if hasattr(odom_data, 'twist') and hasattr(odom_data.twist, 'twist') else 0,
                                    }
                                }
                            }
                        else:
                            frame_data["observation"][sensor_name] = sensor_value
                        
                        # 添加odometry的action（使用下一帧的数据）
                        if i + 1 < len(sensor_data_list):
                            next_sensor_value = sensor_data_list[i + 1]
                            if isinstance(next_sensor_value, dict) and 'data' in next_sensor_value:
                                next_odom_data = next_sensor_value['data']
                                frame_data["action"]["odometry_action"] = {
                                    'pose': {
                                        'position': {
                                            'x': next_odom_data.pose.pose.position.x if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 0,
                                            'y': next_odom_data.pose.pose.position.y if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 0,
                                            'z': next_odom_data.pose.pose.position.z if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 0,
                                        },
                                        'orientation': {
                                            'x': next_odom_data.pose.pose.orientation.x if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 0,
                                            'y': next_odom_data.pose.pose.orientation.y if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 0,
                                            'z': next_odom_data.pose.pose.orientation.z if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 0,
                                            'w': next_odom_data.pose.pose.orientation.w if hasattr(next_odom_data, 'pose') and hasattr(next_odom_data.pose, 'pose') else 1,
                                        }
                                    },
                                    'twist': {
                                        'linear': {
                                            'x': next_odom_data.twist.twist.linear.x if hasattr(next_odom_data, 'twist') and hasattr(next_odom_data.twist, 'twist') else 0,
                                            'y': next_odom_data.twist.twist.linear.y if hasattr(next_odom_data, 'twist') and hasattr(next_odom_data.twist, 'twist') else 0,
                                            'z': next_odom_data.twist.twist.linear.z if hasattr(next_odom_data, 'twist') and hasattr(next_odom_data.twist, 'twist') else 0,
                                        },
                                        'angular': {
                                            'x': next_odom_data.twist.twist.angular.x if hasattr(next_odom_data, 'twist') and hasattr(next_odom_data.twist, 'twist') else 0,
                                            'y': next_odom_data.twist.twist.angular.y if hasattr(next_odom_data, 'twist') and hasattr(next_odom_data.twist, 'twist') else 0,
                                            'z': next_odom_data.twist.twist.angular.z if hasattr(next_odom_data, 'twist') and hasattr(next_odom_data.twist, 'twist') else 0,
                                        }
                                    }
                                }
                            else:
                                # 如果没有下一帧或格式不同，使用当前帧作为action
                                frame_data["action"]["odometry_action"] = frame_data["observation"][sensor_name]
                        else:
                            # 最后一帧，使用当前帧作为action
                            frame_data["action"]["odometry_action"] = frame_data["observation"][sensor_name]

                    elif sensor_name.endswith('_wrench_ext_world') or sensor_name.endswith('_wrench_ext_local'):
                        # 触觉传感器数据（WrenchStamped）
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            wrench_data = sensor_value['data']
                            frame_data["observation"][sensor_name] = {
                                    'force': {
                                        'x': wrench_data.wrench.force.x if hasattr(wrench_data.wrench, 'force') else 0,
                                        'y': wrench_data.wrench.force.y if hasattr(wrench_data.wrench, 'force') else 0,
                                        'z': wrench_data.wrench.force.z if hasattr(wrench_data.wrench, 'force') else 0,
                                    },
                                    'torque': {
                                        'x': wrench_data.wrench.torque.x if hasattr(wrench_data.wrench, 'torque') else 0,
                                        'y': wrench_data.wrench.torque.y if hasattr(wrench_data.wrench, 'torque') else 0,
                                        'z': wrench_data.wrench.torque.z if hasattr(wrench_data.wrench, 'torque') else 0,
                                    }
                                }
                        else:
                            frame_data["observation"][sensor_name] = sensor_value
                    
                    elif sensor_name == 'chassis_imu':
                        # IMU数据
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            imu_data = sensor_value['data']
                            frame_data["observation"][sensor_name] = {
                                'orientation': {
                                    'x': imu_data.orientation.x if hasattr(imu_data, 'orientation') else 0,
                                    'y': imu_data.orientation.y if hasattr(imu_data, 'orientation') else 0,
                                    'z': imu_data.orientation.z if hasattr(imu_data, 'orientation') else 0,
                                    'w': imu_data.orientation.w if hasattr(imu_data, 'orientation') else 1,
                                },
                                'angular_velocity': {
                                    'x': imu_data.angular_velocity.x if hasattr(imu_data, 'angular_velocity') else 0,
                                    'y': imu_data.angular_velocity.y if hasattr(imu_data, 'angular_velocity') else 0,
                                    'z': imu_data.angular_velocity.z if hasattr(imu_data, 'angular_velocity') else 0,
                                },
                                'linear_acceleration': {
                                    'x': imu_data.linear_acceleration.x if hasattr(imu_data, 'linear_acceleration') else 0,
                                    'y': imu_data.linear_acceleration.y if hasattr(imu_data, 'linear_acceleration') else 0,
                                    'z': imu_data.linear_acceleration.z if hasattr(imu_data, 'linear_acceleration') else 0,
                                }
                            }
                        else:
                            frame_data["observation"][sensor_name] = sensor_value
                    
                    elif sensor_name == 'pose':
                        # 定位数据
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            pose_data = sensor_value['data']
                            frame_data["observation"][sensor_name] = {
                                'position': {
                                    'x': pose_data.position.x if hasattr(pose_data, 'position') else 0,
                                    'y': pose_data.position.y if hasattr(pose_data, 'position') else 0,
                                    'z': pose_data.position.z if hasattr(pose_data, 'position') else 0,
                                },
                                'orientation': {
                                    'x': pose_data.orientation.x if hasattr(pose_data, 'orientation') else 0,
                                    'y': pose_data.orientation.y if hasattr(pose_data, 'orientation') else 0,
                                    'z': pose_data.orientation.z if hasattr(pose_data, 'orientation') else 0,
                                    'w': pose_data.orientation.w if hasattr(pose_data, 'orientation') else 1,
                                }
                            }
                        else:
                            frame_data["observation"][sensor_name] = sensor_value
                    
                    else:
                        # 其他传感器数据
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            # 尝试将消息对象转换为字典
                            data_obj = sensor_value['data']
                            # 检查是否是ROS消息对象
                            if hasattr(data_obj, '__dict__'):
                                # 使用递归方法转换
                                frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(data_obj)
                            else:
                                frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(data_obj)
                        elif isinstance(sensor_value, tuple):
                            # 处理joint state格式的元组
                            # 插值后的格式: (positions, velocities, efforts) - 3元素
                            # 或者原始格式: (timestamp, positions, velocities, efforts) - 4元素
                            # 或者 (timestamp, data) - 2元素
                            if sensor_name.endswith('_joint_state') and len(sensor_value) == 3:
                                # Master joint state格式（插值后）: (positions, velocities, efforts)
                                positions = sensor_value[0]
                                velocities = sensor_value[1] if len(sensor_value) > 1 else None
                                efforts = sensor_value[2] if len(sensor_value) > 2 else None
                                
                                joint_state_dict = {}
                                if positions is not None:
                                    joint_state_dict['positions'] = self._convert_ros_msg_to_dict(positions)
                                if velocities is not None:
                                    joint_state_dict['velocities'] = self._convert_ros_msg_to_dict(velocities)
                                if efforts is not None:
                                    joint_state_dict['efforts'] = self._convert_ros_msg_to_dict(efforts)
                                
                                frame_data["observation"][sensor_name] = joint_state_dict
                            else:
                                # 其他元组格式，递归转换（会处理NumPy数组）
                                frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(sensor_value)
                        else:
                            # 其他格式，使用递归转换确保NumPy数组被转换
                            frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(sensor_value)
            
            # 保存图像
            if self.use_video_storage:
                # 视频存储模式：视频已经创建完成，只需记录帧号到JSON
                for cam_name in video_files.keys():
                    frame_data["images"][cam_name] = i
            else:
                # 图像存储模式：从images字典读取并保存为文件
                for cam_name, cam_images in images.items():
                    if i < len(cam_images):
                        img = cam_images[i]
                        # 图像存储模式：保存为单独的文件
                        img_filename = f"frame_{i:04d}_{cam_name}.jpg"
                        img_path = episode_dir / img_filename
                        
                        # 根据图像类型选择保存格式
                        if 'depth' in cam_name:
                            # 深度图像：使用PNG格式（支持浮点数）或保存为numpy数组
                            if isinstance(img, Image.Image):
                                # 深度图像通常是浮点数模式，需要特殊处理
                                if img.mode == 'F':
                                    # 将浮点数深度图转换为可视化图像保存
                                    # 归一化到0-255范围用于可视化
                                    depth_normalized = (img - img.min()) / (img.max() - img.min()) * 255
                                    depth_vis = Image.fromarray(depth_normalized.astype(np.uint8), mode='L')
                                    img_path = img_path.with_suffix('.png')
                                    depth_vis.save(img_path, 'PNG')
                                    img_filename = f"frame_{i:04d}_{cam_name}.png"
                                else:
                                    img_path = img_path.with_suffix('.png')
                                    img.save(img_path, 'PNG')
                                    img_filename = f"frame_{i:04d}_{cam_name}.png"
                            elif isinstance(img, np.ndarray):
                                if img.dtype == np.float32 or img.dtype == np.float64:
                                    # 深度数据：保存为numpy数组文件
                                    img_path = img_path.with_suffix('.npz')
                                    np.savez_compressed(img_path, depth=img)
                                    img_filename = f"frame_{i:04d}_{cam_name}.npz"
                                else:
                                    # 其他numpy数组：转换为图像保存
                                    img_path = img_path.with_suffix('.png')
                                    img_pil = Image.fromarray(img.astype(np.uint8))
                                    img_pil.save(img_path, 'PNG')
                                    img_filename = f"frame_{i:04d}_{cam_name}.png"
                            elif isinstance(img, bytes):
                                # 原始字节数据：保存为二进制文件
                                img_path = img_path.with_suffix('.bin')
                                with open(img_path, 'wb') as f:
                                    f.write(img)
                                img_filename = f"frame_{i:04d}_{cam_name}.bin"
                            else:
                                # 其他格式：尝试保存为pickle
                                img_path = img_path.with_suffix('.pkl')
                                with open(img_path, 'wb') as f:
                                    pickle.dump(img, f)
                                img_filename = f"frame_{i:04d}_{cam_name}.pkl"
                        else:
                            # 普通RGB图像：使用JPEG格式
                            if isinstance(img, Image.Image):
                                img.save(img_path, 'JPEG', quality=self.image_quality)
                            else:
                                # 如果是numpy数组，转换为PIL Image
                                if isinstance(img, np.ndarray):
                                    img = Image.fromarray(img)
                                    img.save(img_path, 'JPEG', quality=self.image_quality)
                        
                        frame_data["images"][cam_name] = str(img_filename)
            
            episode_json["frames"].append(frame_data)
        
        # 视频存储模式：视频已经通过 ffmpeg 创建完成
        if self.use_video_storage:
            print(f"  ✓ 视频文件已保存: {len(video_files)} 个")
            
            # 验证视频文件
            for cam_name, video_file in video_files.items():
                video_path = episode_dir / video_file
                if video_path.exists():
                    file_size = video_path.stat().st_size
                    print(f"    {cam_name}: {file_size / 1024 / 1024:.2f} MB")
                    if file_size < 1024:  # 小于1KB
                        print(f"    ⚠️  警告: {cam_name} 文件过小，可能损坏")
            
            # 清理图像数据
            if 'aligned_video_images' in locals():
                aligned_video_images.clear()
                del aligned_video_images
            
            # 清理临时文件（确保即使出错也清理）
            try:
                self._cleanup_temp_files()
            except Exception as e:
                print(f"  ⚠️  警告: 清理临时文件时出错: {e}")
            
            # 强制垃圾回收以释放内存
            import gc
            gc.collect()
        
        # 保存episode JSON文件（使用临时文件确保原子性写入）
        episode_json_path = episode_dir / "episode.json"
        temp_json_path = episode_dir / "episode.json.tmp"
        
        try:
            # 先写入临时文件
            with open(temp_json_path, 'w', encoding='utf-8') as f:
                json.dump(episode_json, f, indent=2, ensure_ascii=False)
                # 确保文件完整写入磁盘
                f.flush()
                os.fsync(f.fileno())
            
            # 原子性重命名（替换现有文件）
            if episode_json_path.exists():
                episode_json_path.unlink()
            temp_json_path.rename(episode_json_path)
            
        except Exception as e:
            print(f"  ⚠️  保存JSON文件时出错: {e}")
            import traceback
            traceback.print_exc()
            # 尝试直接保存（如果临时文件方式失败）
            try:
                with open(episode_json_path, 'w', encoding='utf-8') as f:
                    json.dump(episode_json, f, indent=2, ensure_ascii=False)
                    f.flush()
                    os.fsync(f.fileno())
            except Exception as e2:
                print(f"  ✗ JSON文件保存失败: {e2}")
                import traceback
                traceback.print_exc()
                raise
            finally:
                # 清理临时文件（如果存在）
                if temp_json_path.exists():
                    try:
                        temp_json_path.unlink()
                    except:
                        pass
        
        # 更新数据集元数据
        self.dataset_metadata['episodes'].append({
            "episode_id": episode_id,
            "task": task,
            "timestamp": episode_json["timestamp"],
            "duration": episode_json["duration"],
            "num_frames": num_frames,
            "path": str(episode_dir.relative_to(self.output_dir))
        })
        self._save_metadata()
        
        print(f"✓ Episode {episode_id} 已保存")
        print(f"  - 帧数: {num_frames}")
        print(f"  - 时长: {episode_json['duration']:.2f}s")
        print(f"  - JSON: {episode_json_path}")
        if self.use_video_storage:
            total_video_frames = num_frames * len(video_files)
            print(f"  - 视频: {len(video_files)} 个MP4文件 (每个{num_frames}帧)")
        else:
            total_image_count = sum(len(imgs) for imgs in images.values())
            print(f"  - 图像: {total_image_count} 张JPG文件")
        
        return {
            "episode_id": episode_id,
            "episode_dir": str(episode_dir),
            "episode_json": str(episode_json_path),
            "num_frames": num_frames,
            "duration": episode_json["duration"]
        }
