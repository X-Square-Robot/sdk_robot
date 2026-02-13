"""
gRPC Real-time Data Collector - Supports all sensors, saves as universal JSON format

Usage:
    from x2robot import connect
    from x2robot.action_data_collection import DataCollector
    from x2robot.collection_config import CollectionConfigPresets
    
    robot = connect("x2://192.168.1.100:50051")
    
    # Use preset configuration
    collector = DataCollector(
        robot, 
        output_dir="./data", 
        target_hz=30,
        collection_config=CollectionConfigPresets.full_manipulation()
    )
    
    collector.start_recording(task="pick and place")
    # ... execute task ...
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
    """Real-time data collector - saves as universal JSON format

    Collected data format:
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
                "joint_names": ["joint1", "joint2", ...],  // Joint name list
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
        video_codec: str = 'XVID',  # Default to XVID, good compatibility
    ):
        """Initialize data collector

        Args:
            robot: Robot instance
            output_dir: Data save directory
            target_hz: Target collection frequency (for downsampling)
            collection_config: Sensor configuration object, use presets from SensorConfigPresets
                          If None, use basic_manipulation preset
            image_quality: JPEG image quality (1-100), only used when use_video_storage=False
            downsample_joint_states: Whether to downsample joint states
                - True: Downsample to target_hz (saves memory, suitable for training)
                - False: Keep original 500Hz (high precision, large memory usage)
            use_video_storage: Whether to use video format for image storage
                - True: Save as MP4 video (saves space, faster loading)
                - False: Save as JPG images (default, good compatibility)
            video_codec: Video codec (default: 'XVID')
                Recommended: 'XVID' (good compatibility), 'MJPG' (Motion JPEG, best compatibility)
                Optional: 'mp4v' (MPEG-4), 'avc1' (H.264, requires hardware support)
        """
        self.robot = robot
        self.output_dir = Path(output_dir)
        self.target_hz = target_hz
        self.target_period = 1.0 / target_hz
        self.image_quality = image_quality
        self.downsample_joint_states = downsample_joint_states
        self.use_video_storage = use_video_storage
        self.video_codec = video_codec

        # Use provided configuration or default configuration
        self.collection_config = collection_config or CollectionConfig()
        self.camera_names = self.collection_config.get_camera_names()
        
        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Episode counter
        self.episode_count = 0
        
        # Data buffer queues - only created when needed
        # Queue size calculation: expected recording duration (seconds) × collection frequency (Hz)
        # Joint states: may be high frequency (500Hz)
        # Other sensors: usually lower frequency (10-100Hz)
        self.queues = {}  # Unified management of all data queues
        
        # Calculate joint state queue size (for display, even if joint states are not enabled)
        if downsample_joint_states:
            # Support 10 minutes of collection time with sufficient buffer
            self.queue_size = int(target_hz * 900)  # 900 seconds = 15 minutes
        else:
            # High frequency mode: support longer duration or larger buffer
            self.queue_size = int(500 * 900)  # 450,000, support 15 minutes @ 500Hz

        # Sensor data file locks (no longer using queues)
        sensor_names = []

        if collection_config.enable_left_arm_end_pose:
            sensor_names.append('left_arm_end_pose')
        if collection_config.enable_right_arm_end_pose:
            sensor_names.append('right_arm_end_pose')
        if collection_config.enable_waist_end_pose:
            sensor_names.append('waist_end_pose')

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

        # Joint states use temporary file storage (high frequency collection, avoid memory issues)
        # Set according to configured slave_joint_names and slave_action_names
        self.slave_joint_names = []
        self.slave_action_names = []
        
        if collection_config.slave_joint_names:
            self.slave_joint_names = collection_config.slave_joint_names
            if downsample_joint_states:
                print(f"Joint state downsampling mode: 500Hz → {target_hz}Hz")
            else:
                print(f"Joint state original frequency mode: keeping 500Hz")
            print(f"Configured joint states: {self.slave_joint_names}")
            
            # Configure action names according to slave_action_names
            if collection_config.slave_action_names is not None:
                # If action_names are explicitly specified, use them
                self.slave_action_names = collection_config.slave_action_names
            else:
                # If None, automatically generate action_names from joint_names
                self.slave_action_names = [name.replace('_joint_states', '_actions') for name in self.slave_joint_names]
            
            if self.slave_action_names:
                print(f"Configured actions: {self.slave_action_names}")
            
            # Add joint states and actions for each part to sensor name list
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

        if collection_config.enable_left_gripper_position:
            sensor_names.append('left_gripper_position')
        if collection_config.enable_right_gripper_position:
            sensor_names.append('right_gripper_position')

        # All sensor data uses temporary file storage to avoid queue memory overflow
        self.sensor_temp_files = {}  # {sensor_name: file_handle}
        self.sensor_temp_paths = {}  # {sensor_name: file_path}
        self.sensor_file_locks = {}  # {sensor_name: lock}

        # Create file lock for each sensor
        for sensor_name in sensor_names:
            self.sensor_file_locks[sensor_name] = threading.Lock()
        
        # Image data uses temporary file storage to avoid memory overflow
        self.image_temp_files = {}  # {camera_name: file_handle}
        self.image_temp_paths = {}  # {camera_name: file_path}
        self.image_file_locks = {cam: threading.Lock() for cam in self.camera_names}
        
        # Control flags
        self.is_recording = False
        # Note: is_collecting is no longer used, only is_recording
        self.threads = []
        # Store thread information for restarting threads
        self.thread_info = []  # [(thread_name, target_func, args), ...]
        
        # Current episode information
        self.current_episode_task = None
        self.current_episode_start_time = None
        
        # Data collection statistics - dynamically create counters for each enabled sensor
        self.stats = {
            'start_time': None,
            'last_update': None
        }
        # Create counters for each enabled queue
        for queue_name in self.queues.keys():
            self.stats[f'{queue_name}_count'] = 0
        # Create counters for each camera
        for cam in self.camera_names:
            self.stats[f'{cam}_count'] = 0
        self.stats_lock = threading.Lock()
        
        # Joint name mapping
        self.joint_names = None
        self.joint_name_mapping = None
        
        # Error time record (for network error handling)
        self.last_error_time = 0
        
        # Camera data detection - record the last time each camera received data
        self.camera_last_data_time = {cam: None for cam in self.camera_names}
        self.camera_data_check_interval = 1.0  # Check interval (seconds)
        self.camera_data_timeout = 5.0  # Timeout (seconds) - error if no data within 5 seconds
        self.camera_monitor_thread = None
        
        # Dataset metadata file
        self.metadata_file = self.output_dir / "dataset_metadata.json"
        self._load_or_create_metadata()
        
        # Register cleanup function on exit to ensure temporary files are cleaned up
        # Use lambda wrapper to ensure access to self
        atexit.register(lambda: self._cleanup_before_exit("Program exiting, cleaning up resources..."))
        
        # Register signal handler to ensure temporary files are cleaned up even on abnormal exit
        def signal_handler(sig, frame):
            """Handle interrupt signal, ensure temporary files are cleaned up"""
            err_msg = "\nReceived interrupt signal, cleaning up resources..."
            try:
                self._cleanup_before_exit(err_msg)
            except Exception as e:
                print(f"Error cleaning up resources: {e}")
            sys.exit(0)
        
        # Register SIGINT (Ctrl+C) and SIGTERM signal handlers
        # Note: Signal handlers may be overridden by handlers in example files, but at least try to clean up here
        try:
            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)
        except (ValueError, OSError):
            # Some signals may not be supported in certain environments (e.g., Windows or some test environments)
            pass
    
    def _load_or_create_metadata(self):
        """Load or create dataset metadata"""
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r', encoding='utf-8') as f:
                self.dataset_metadata = json.load(f)
            self.episode_count = len(self.dataset_metadata.get('episodes', []))
            print(f"Loaded existing dataset, currently has {self.episode_count} episodes")
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
        """Save dataset metadata"""
        with open(self.metadata_file, 'w', encoding='utf-8') as f:
            json.dump(self.dataset_metadata, f, indent=2, ensure_ascii=False)
        
    def start_recording(self, task: str = "default_task"):
        """Start recording episode (start all data collection threads)
        
        Args:
            task: Task description
        """
        if self.is_recording:
            print("⚠️  Warning: Already recording")
            return
        
        # Start various data collection threads
        threads = []
        
        # Joint state collection - start corresponding collection threads according to configured slave_joint_names
        if self.slave_joint_names:
            # Create mapping from joint_name to collection method
            joint_collector_map = {
                'left_arm_joint_states': (self._collect_left_arm_joint_states, "LeftArmJointStateCollector"),
                'right_arm_joint_states': (self._collect_right_arm_joint_states, "RightArmJointStateCollector"),
                'lift_joint_states': (self._collect_lift_joint_states, "LiftJointStateCollector"),
                'waist_joint_states': (self._collect_waist_joint_states, "WaistJointStateCollector"),
                'left_gripper_joint_states': (self._collect_left_gripper_joint_states, "LeftGripperJointStateCollector"),
                'right_gripper_joint_states': (self._collect_right_gripper_joint_states, "RightGripperJointStateCollector"),
                'head_joint_states': (self._collect_head_joint_states, "HeadJointStateCollector"),
            }
            
            # Start corresponding threads according to configured joint_names
            for joint_name in self.slave_joint_names:
                if joint_name in joint_collector_map:
                    collector_func, thread_name = joint_collector_map[joint_name]
                    threads.append(threading.Thread(target=collector_func, daemon=True, name=thread_name))
                else:
                    raise ValueError(f"Unknown joint state name: {joint_name}. Supported names: {list(joint_collector_map.keys())}")

        # Image stream collection
        if self.collection_config.enable_head_rgb_stream:
            threads.append(threading.Thread(target=self._collect_head_rgb_stream, daemon=True, name="HeadRgbStreamCollector"))

        if self.collection_config.enable_head_depth_stream:
            threads.append(threading.Thread(target=self._collect_head_depth_stream, daemon=True, name="HeadDepthStreamCollector"))

        if self.collection_config.enable_left_arm_rgb_stream:
            threads.append(threading.Thread(target=self._collect_left_arm_rgb_stream, daemon=True, name="LeftArmRgbStreamCollector"))

        if self.collection_config.enable_right_arm_rgb_stream:
            threads.append(threading.Thread(target=self._collect_right_arm_rgb_stream, daemon=True, name="RightArmRgbStreamCollector"))

        # Sensor collection
        if self.collection_config.enable_chassis_imu:
            threads.append(threading.Thread(target=self._collect_imu, daemon=True, name="ImuCollector"))
        
        if self.collection_config.enable_depth_points:
            threads.append(threading.Thread(target=self._collect_depth, daemon=True, name="DepthCollector"))
        
        # End effector pose collection
        if self.collection_config.enable_left_arm_end_pose:
            threads.append(threading.Thread(target=self._collect_left_arm_end_pose, daemon=True, name="LeftArmEndPoseCollector"))

        if self.collection_config.enable_right_arm_end_pose:
            threads.append(threading.Thread(target=self._collect_right_arm_end_pose, daemon=True, name="RightArmEndPoseCollector"))

        if self.collection_config.enable_waist_end_pose:
            threads.append(threading.Thread(target=self._collect_waist_end_pose, daemon=True, name="WaistEndPoseCollector"))

        # Chassis sensor collection
        if self.collection_config.enable_odometry:
            threads.append(threading.Thread(target=self._collect_odometry, daemon=True, name="OdometryCollector"))

        if self.collection_config.enable_pose:
            threads.append(threading.Thread(target=self._collect_pose, daemon=True, name="PoseCollector"))

        # Depth and laser sensor collection
        if self.collection_config.enable_head_depth_video:
            threads.append(threading.Thread(target=self._collect_head_depth_video, daemon=True, name="HeadDepthVideoCollector"))

        if self.collection_config.enable_laser_scan:
            threads.append(threading.Thread(target=self._collect_laser_scan, daemon=True, name="LaserScanCollector"))

        # Tactile sensor collection
        if self.collection_config.enable_left_gripper_tactile:
            threads.append(threading.Thread(target=self._collect_left_gripper_tactile, daemon=True, name="LeftGripperTactileCollector"))

        if self.collection_config.enable_right_gripper_tactile:
            threads.append(threading.Thread(target=self._collect_right_gripper_tactile, daemon=True, name="RightGripperTactileCollector"))

        if self.collection_config.enable_left_hand_tactile:
            threads.append(threading.Thread(target=self._collect_left_hand_tactile, daemon=True, name="LeftHandTactileCollector"))

        if self.collection_config.enable_right_hand_tactile:
            threads.append(threading.Thread(target=self._collect_right_hand_tactile, daemon=True, name="RightHandTactileCollector"))

        # Distance sensor collection
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

        if self.collection_config.enable_left_gripper_position:
            threads.append(threading.Thread(target=self._collect_left_gripper_position, daemon=True, name="LeftGripperPositionCollector"))
        if self.collection_config.enable_right_gripper_position:
            threads.append(threading.Thread(target=self._collect_right_gripper_position, daemon=True, name="RightGripperPositionCollector"))

        # Check if at least one data collection is enabled
        if not threads:
            print("Error: No data collection enabled, please enable at least one data type")
            return
        
        # Initialize statistics
        with self.stats_lock:
            self.stats['start_time'] = time.time()
            self.stats['last_update'] = time.time()
        
        # Save thread information
        self.thread_info = []
        for t in threads:
            self.thread_info.append((t.name, t._target))
            self.threads.append(t)
        
        # Clear queues
        self._clear_queues()
        
        # Create temporary files
        self._create_temp_files()
        
        self.current_episode_task = task
        self.current_episode_start_time = time.time()
        self.is_recording = True
        
        # Start all threads
        for t in threads:
            t.start()
        
        print(f"✓ Started recording Episode {self.episode_count} (Task: {task})")
        print(f"  - Output directory: {self.output_dir}")
        print(f"  - Image storage: {'MP4 video' if self.use_video_storage else 'JPG images'}")
        print(f"  - Target frequency: {self.target_hz} Hz")
        
        # Display enabled data streams
        sensor_count = len(self.sensor_file_locks)
        image_count = len(self.camera_names)

        if sensor_count > 0:
            print(f"  - Sensor data streams: ✓ ({sensor_count} sensors)")
        else:
            print(f"  - Sensor data streams: ✗ (not enabled)")

        if image_count > 0:
            print(f"  - Image streams: ✓ ({image_count} cameras)")
        else:
            print(f"  - Image streams: ✗ (not enabled)")
        
        # Reset camera data time tracking
        with self.stats_lock:
            for cam in self.camera_names:
                self.camera_last_data_time[cam] = None
        
        # Start camera data monitoring thread
        self._start_camera_monitor()
        
        # Reset statistics - reset all count statistics
        with self.stats_lock:
            # Reset all statistics ending with _count
            for key in list(self.stats.keys()):
                if key.endswith('_count'):
                    if isinstance(self.stats[key], dict):
                        # If dictionary type (e.g., image_count), reset to defaultdict(int)
                        self.stats[key] = defaultdict(int)
                    else:
                        # If numeric type, reset to 0
                        self.stats[key] = 0
            # Ensure these fixed statistics exist
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
    
    def stop_recording(self) -> Optional[Dict[str, Any]]:
        """Stop recording and save episode (stop all data collection threads)
        
        Returns:
            episode_info: Episode information dictionary, including save path, etc.
        """
        if not self.is_recording:
            print("⚠️  Warning: Not recording")
            return None
        
        self.is_recording = False
        print(f"Stopping recording Episode {self.episode_count}...")
        
        try:
            # Stop camera data monitoring thread
            self._stop_camera_monitor()
            
            # Wait for threads to exit loop, stop reading from stream
            # This avoids process exit due to stream errors during data processing
            print("  Waiting for collection threads to stop...")
            time.sleep(0.5)  # Give threads enough time to exit loop
            
            # Collect data (validate data during collection)
            episode_data = self._collect_episode_data()
            
            if episode_data is None:
                print("Error: Episode data collection failed, unable to get data")
                sys.exit(1)
            
            # Validate data (throws RuntimeError and exits if validation fails)
            # Note: Validation should be done during data collection phase, check if there is actual data in temporary files
            if not self._validate_episode_data(episode_data):
                print("Error: Episode data validation failed")
                sys.exit(1)
            
            # Save episode
            episode_info = self._save_episode(episode_data, self.current_episode_task)
            
            self.episode_count += 1
            self.current_episode_task = None
            self.current_episode_start_time = None
            
            return episode_info
        finally:
            # Ensure temporary files are cleaned up (even if error occurs)
            # Note: No need to exit here, just clean up resources
            # Stop threads first, then clean up files, avoid writing to closed files
            try:
                # Stop all threads (threads will automatically exit by setting is_recording=False)
                if hasattr(self, 'threads') and self.threads:
                    alive_threads = [t for t in self.threads if t.is_alive()]
                    if alive_threads:
                        # Give threads a very short time to exit (0.1 seconds), then continue
                        # Threads are daemon threads, even if not fully exited, won't block main process
                        for thread in alive_threads:
                            thread.join(timeout=0.1)
                        self.threads = []
                
                # Clean up temporary files
                self._cleanup_temp_files()
            except Exception as e:
                print(f"  ⚠️  Error cleaning up resources: {e}")
            
            print("✓ Recording stopped")
            print(f"✓ Collected {self.episode_count} episodes in total")
            print(f"✓ Data saved in: {self.output_dir}")
    
    def _collect_joint_states(self):
        """Collect full robot joint state data"""
        print("Starting full joint state stream...")
        
        try:
            stream = self.robot.state.get_all_joint_states_stream(timeout=None)
            
            for state_msg in stream:
                # If not recording, exit loop
                if not self.is_recording:
                    break
                
                # If not recording, exit loop (thread will stop)
                if not self.is_recording:
                    break
                
                try:
                    # Establish joint name mapping when receiving first message
                    if self.joint_name_mapping is None:
                        print(f"  🔍 Analyzing joint state message structure...")
                        print(f"    Message type: {type(state_msg)}")

                        # Check name field
                        has_name = hasattr(state_msg, 'name')
                        print(f"    name field: {'✓' if has_name else '✗'}")
                        if has_name:
                            name_val = getattr(state_msg, 'name', None)
                            if name_val and len(name_val) > 0:
                                print(f"    ✓ Joint names: {len(name_val)} ({name_val[0]}...{name_val[-1]})")
                            else:
                                print(f"    ⚠️  name field is empty or None")

                        # Check position field
                        has_position = hasattr(state_msg, 'position')
                        print(f"    position field: {'✓' if has_position else '✗'}")
                        if has_position:
                            pos_val = getattr(state_msg, 'position', None)
                            if pos_val and len(pos_val) > 0:
                                print(f"    ✓ Joint positions: {len(pos_val)}")
                            else:
                                print(f"    ⚠️  position field is empty or None (message may not be initialized)")

                        joint_names_obtained = False

                        # Prefer joint names from message
                        if hasattr(state_msg, 'name') and state_msg.name and len(state_msg.name) > 0:
                            self.joint_names = list(state_msg.name)
                            self.joint_name_mapping = {}
                            for idx, name in enumerate(state_msg.name):
                                self.joint_name_mapping[name] = idx

                            print(f"  ✅ Joint names set successfully: {len(self.joint_name_mapping)} joints")
                            print(f"     Joint list: {self.joint_names}")
                            joint_names_obtained = True

                        # If no name field, try to infer from position length
                        elif hasattr(state_msg, 'position') and state_msg.position and len(state_msg.position) > 0:
                            num_joints = len(state_msg.position)
                            self.joint_names = [f"joint_{i+1}" for i in range(num_joints)]
                            self.joint_name_mapping = {name: idx for idx, name in enumerate(self.joint_names)}

                            print(f"  ⚠️  Using default joint names: {len(self.joint_name_mapping)} joints")
                            print(f"     Default list: {self.joint_names}")
                            joint_names_obtained = True

                        else:
                            print(f"  ⏳ Waiting for valid joint data... (first message may not be initialized)")

                        # Update metadata
                        if joint_names_obtained and self.joint_names:
                            self.dataset_metadata['joint_names'] = self.joint_names
                            self._save_metadata()
                            print(f"  ✅ Joint mapping configuration completed")
                        elif not joint_names_obtained:
                            print(f"  ⏳ Joint name setting delayed...")
                    
                    # Extract all joint data
                    joint_positions = np.array(state_msg.position, dtype=np.float32).flatten()
                    joint_velocities = np.array(state_msg.velocity, dtype=np.float32).flatten() if hasattr(state_msg, 'velocity') and state_msg.velocity else None
                    joint_efforts = np.array(state_msg.effort, dtype=np.float32).flatten() if hasattr(state_msg, 'effort') and state_msg.effort else None

                    timestamp = self._extract_timestamp_from_header(state_msg)
                    
                    if self.is_recording:
                        # Ensure temporary file is created (only available after start_recording)
                        if hasattr(self, 'sensor_temp_files') and 'joint_states' in self.sensor_temp_files:
                            # Write joint state data to temporary file (avoid queue memory issues)
                            joint_data = (timestamp, joint_positions, joint_velocities, joint_efforts)
                            with self.sensor_file_locks['joint_states']:
                                if 'joint_states' in self.sensor_temp_files:
                                    pickle.dump(joint_data, self.sensor_temp_files['joint_states'])
                                    self.sensor_temp_files['joint_states'].flush()

                            # Also write action data to file (same as state, for imitation learning)
                            action_data = (timestamp, joint_positions)
                            with self.sensor_file_locks['actions']:
                                if 'actions' in self.sensor_temp_files:
                                    pickle.dump(action_data, self.sensor_temp_files['actions'])
                                    self.sensor_temp_files['actions'].flush()
                        
                        with self.stats_lock:
                            self.stats['state_count'] += 1
                            self.stats['action_count'] += 1
                
                except Exception as e:
                    print(f"Joint state data processing error: {e}")
                    continue
        
        except Exception as e:
            print(f"Joint state stream error: {e}")
            raise
    
    def _collect_left_arm_joint_states(self):
        """Collect left arm joint state data"""
        print("Starting left arm joint state stream...")
        self._collect_joint_state_stream('left_arm_joint_states', self.robot.left_arm.get_joint_states_stream)
    
    def _collect_right_arm_joint_states(self):
        """Collect right arm joint state data"""
        print("Starting right arm joint state stream...")
        self._collect_joint_state_stream('right_arm_joint_states', self.robot.right_arm.get_joint_states_stream)
    
    def _collect_lift_joint_states(self):
        """Collect waist joint state data"""
        print("Starting waist joint state stream...")
        self._collect_joint_state_stream('lift_joint_states', self.robot.lift.get_joint_states_stream)
    
    def _collect_left_gripper_joint_states(self):
        """Collect left gripper joint state data"""
        print("Starting left gripper joint state stream...")
        self._collect_joint_state_stream('left_gripper_joint_states', self.robot.left_gripper.get_joint_states_stream)
    
    def _collect_right_gripper_joint_states(self):
        """Collect right gripper joint state data"""
        print("Starting right gripper joint state stream...")
        self._collect_joint_state_stream('right_gripper_joint_states', self.robot.right_gripper.get_joint_states_stream)
    
    def _collect_head_joint_states(self):
        """Collect head joint state data"""
        print("Starting head joint state stream...")
        self._collect_joint_state_stream('head_joint_states', self.robot.head.get_joint_states_stream)

    
    def _collect_waist_joint_states(self):
        """Collect waist joint state data"""
        print("Starting waist joint state stream...")
        self._collect_joint_state_stream('waist_joint_states', self.robot.waist.get_joint_states_stream)
    
    
    # ============ Image Stream Collection Methods ============

    def _collect_head_rgb_stream(self):
        """Collect head RGB video stream"""
        print("Starting head RGB video stream...")
        self._collect_camera_stream('head_rgb_stream', self.robot.head_camera.get_rgb_video_stream)

    def _collect_head_depth_stream(self):
        """Collect head depth video stream"""
        print("Starting head depth video stream...")
        self._collect_depth_stream('head_depth_stream', self.robot.head_camera.get_depth_video_stream)

    def _collect_left_arm_rgb_stream(self):
        """Collect left arm RGB video stream"""
        print("Starting left arm RGB video stream...")
        self._collect_camera_stream('left_arm_rgb_stream', self.robot.left_arm_camera.get_video_stream)

    def _collect_right_arm_rgb_stream(self):
        """Collect right arm RGB video stream"""
        print("Starting right arm RGB video stream...")
        self._collect_camera_stream('right_arm_rgb_stream', self.robot.right_arm_camera.get_video_stream)
    
    def _collect_camera_stream(self, camera_name, stream_func):
        """Collect video stream from a single camera"""
        print(f"Starting {camera_name} stream...")
        
        try:
            stream = stream_func(timeout=None)
            
            for frame_msg in stream:
                # If not recording, exit loop
                if not self.is_recording:
                    break
                
                # If not recording, exit loop (thread will stop)
                if not self.is_recording:
                    break
                
                try:
                    # Check if data is empty
                    if not frame_msg or not frame_msg.data:
                        continue

                    # Decode image - need to convert to bytes
                    img_bytes = bytes(frame_msg.data)
                    img = Image.open(io.BytesIO(img_bytes))

                    # Convert to RGB
                    if img.mode != 'RGB':
                        img = img.convert('RGB')

                    timestamp = self._extract_timestamp_from_header(frame_msg)

                    if self.is_recording:
                        # Write to temporary file (compressed as JPEG format to reduce storage space)
                        try:
                            with self.image_file_locks[camera_name]:
                                if camera_name in self.image_temp_files:
                                    # Compress image to JPEG format (bytes) to reduce storage space
                                    # 720P uncompressed ~2.64MB, JPEG compressed ~200-500KB, can reduce 80-90% space
                                    img_bytes_compressed = io.BytesIO()
                                    img.save(img_bytes_compressed, 'JPEG', quality=self.image_quality)
                                    img_bytes_compressed = img_bytes_compressed.getvalue()
                                    
                                    # Store compressed image data and timestamp
                                    pickle.dump((timestamp, img_bytes_compressed), self.image_temp_files[camera_name])
                                    self.image_temp_files[camera_name].flush()

                                    with self.stats_lock:
                                        self.stats['image_count'][camera_name] += 1
                                        # Update last time camera received data
                                        self.camera_last_data_time[camera_name] = time.time()
                        except Exception as e:
                            print(f"❌ {camera_name} data processing error: {e}")
                            print(f"   Data type: {type(frame_msg.data) if hasattr(frame_msg, 'data') else 'No data attribute'}")
                            print(f"   Data content: {repr(frame_msg.data) if hasattr(frame_msg, 'data') else 'No data attribute'}")
                            raise RuntimeError(f"{camera_name} data processing failed: {e}")

                except Exception as e:
                    if self._is_grpc_connection_error(e):
                        self._handle_grpc_error_and_exit(camera_name, e)
                    print(f"❌ {camera_name} stream processing error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
                    raise RuntimeError(f"{camera_name} stream processing failed: {e}")

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(camera_name, e)
            print(f"❌ {camera_name} stream processing error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
            raise RuntimeError(f"{camera_name} stream processing failed: {e}")

    def _collect_depth_stream(self, camera_name, stream_func):
        """Collect depth video stream"""
        print(f"Starting {camera_name} stream...")

        try:
            stream = stream_func(timeout=None)

            for frame_msg in stream:
                # If not recording, exit loop
                if not self.is_recording:
                    break
                
                # If not recording, exit loop (thread will stop)
                if not self.is_recording:
                    break

                try:
                    # Check if data is empty
                    if not frame_msg or not frame_msg.data:
                        continue
        
                    depth_bytes = bytes(frame_msg.data)
                    timestamp = self._extract_timestamp_from_header(frame_msg)

                    if self.is_recording:
                        # Try multiple methods to process depth data

                        depth_data = None

                        # Method 1: Try to process as compressed image
                        try:
                            depth_img = Image.open(io.BytesIO(depth_bytes))
                            # Convert to numpy array
                            depth_data = np.array(depth_img, dtype=np.float32)
                            print(f"{camera_name} depth data parsed as image: {depth_data.shape}")
                        except Exception:
                            # Method 2: Try to process as raw float32 array
                            try:
                                # Check if data length is a multiple of 4
                                if len(depth_bytes) % 4 == 0:
                                    num_pixels = len(depth_bytes) // 4
                                    depth_values = struct.unpack(f'{num_pixels}f', depth_bytes)
                                    depth_data = np.array(depth_values, dtype=np.float32)

                                    # Try common depth map resolutions
                                    if len(depth_data) == 640 * 480:
                                        depth_data = depth_data.reshape(480, 640)
                                    elif len(depth_data) == 320 * 240:
                                        depth_data = depth_data.reshape(240, 320)
                                    elif len(depth_data) == 1280 * 720:
                                        depth_data = depth_data.reshape(720, 1280)
                                    elif len(depth_data) == 640 * 360:
                                        depth_data = depth_data.reshape(360, 640)
                                    # If not standard resolution, keep as 1D array
                                    print(f"{camera_name} depth data parsed as float32 array: {depth_data.shape}")
                                else:
                                    raise ValueError("Data length is not a multiple of 4")
                            except Exception:
                                # Method 3: Save raw byte data
                                depth_data = depth_bytes
                                print(f"{camera_name} saving raw depth data: {len(depth_bytes)} bytes")

                        # Save processed depth data
                        with self.image_file_locks[camera_name]:
                            if camera_name in self.image_temp_files:
                                pickle.dump((timestamp, depth_data), self.image_temp_files[camera_name])
                                self.image_temp_files[camera_name].flush()

                        with self.stats_lock:
                            self.stats['image_count'][camera_name] += 1
                            # Update last time camera received data
                            self.camera_last_data_time[camera_name] = time.time()

                except Exception as e:
                    if self._is_grpc_connection_error(e):
                        self._handle_grpc_error_and_exit(camera_name, e)
                    print(f"❌ {camera_name} data processing error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
                    print(f"   Data type: {type(frame_msg.data) if hasattr(frame_msg, 'data') else 'No data attribute'}")
                    print(f"   Data content: {repr(frame_msg.data) if hasattr(frame_msg, 'data') else 'No data attribute'}")
                    raise RuntimeError(f"{camera_name} data processing failed: {e}")

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(camera_name, e)
            # Other types of errors, re-raise
            print(f"❌ {camera_name} stream error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
            raise
    
    def _collect_imu(self):
        """Collect IMU data"""
        print("Starting IMU data stream...")
        
        try:
            stream = self.robot.imu.get_chassis_imu_stream(timeout=None)
            
            for imu_msg in stream:
                # If not recording, exit loop
                if not self.is_recording:
                    break
                
                # If not recording, exit loop (thread will stop)
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
                        # Ensure temporary file is created (available after start_recording)
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
                    print(f"IMU data processing error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                    continue
        
        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit('chassis_imu', e)
            print(f"IMU stream error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            raise
    
    def _collect_depth(self):
        """Collect depth data"""
        print("Starting depth data stream...")
        
        try:
            stream = self.robot.depth_points.get_chassis_depth_points_stream(timeout=None)
            
            for depth_msg in stream:
                # If not recording, exit loop
                if not self.is_recording:
                    break
                
                # If not recording, exit loop (thread will stop)
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
                        # Ensure temporary file is created (available after start_recording)
                        if hasattr(self, 'sensor_temp_files') and 'depth_points' in self.sensor_temp_files:
                            depth_data_tuple = (timestamp, depth_info)
                            with self.sensor_file_locks['depth_points']:
                                if 'depth_points' in self.sensor_temp_files:
                                    pickle.dump(depth_data_tuple, self.sensor_temp_files['depth_points'])
                                    self.sensor_temp_files['depth_points'].flush()
                        with self.stats_lock:
                            self.stats['depth_count'] += 1
                
                except Exception as e:
                    print(f"Depth data processing error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                    continue
        
        except Exception as e:
            print(f"Depth stream error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            raise

    # ============ End effector pose collection methods ============
    
    def _collect_left_arm_end_pose(self):
        """Collect left arm end effector pose"""
        print("Starting left arm end effector pose stream...")
        self._collect_pose_stream('left_arm_end_pose', self.robot.left_arm.get_end_pose_stream)

    def _collect_right_arm_end_pose(self):
        """Collect right arm end effector pose"""
        print("Starting right arm end effector pose stream...")
        self._collect_pose_stream('right_arm_end_pose', self.robot.right_arm.get_end_pose_stream)

    def _collect_waist_end_pose(self):
        """Collect waist end effector pose"""
        print("Starting waist end effector pose stream...")
        self._collect_pose_stream('waist_end_pose', self.robot.waist.get_end_pose_stream)

    # ============ Chassis sensor collection methods ============

    def _collect_odometry(self):
        """Collect chassis odometry data"""
        print("Starting odometry stream...")
        self._collect_generic_stream('odometry', self.robot.chassis.get_odometry_stream)

    def _collect_pose(self):
        """Collect robot pose data"""
        print("Starting pose data stream...")
        self._collect_generic_stream('pose', self.robot.chassis.get_pose_stream)

    # ============ Depth and LiDAR sensor collection methods ============

    def _collect_head_depth_video(self):
        """Collect head depth video stream"""
        print("Starting head depth video stream...")
        self._collect_camera_stream('head_depth_video', self.robot.head_camera.get_depth_video_stream)

    def _collect_laser_scan(self):
        """Collect LiDAR scan data"""
        print("Starting LiDAR stream...")
        self._collect_generic_stream('laser_scan', self.robot.radar.get_laser_scan_stream)

    # ============ Tactile sensor collection methods ============

    def _collect_left_gripper_tactile(self):
        """Collect left gripper tactile data"""
        print("Starting left gripper tactile sensor...")
        if hasattr(self.robot, 'left_gripper_tactile'):
            self._collect_generic_stream('left_gripper_tactile', self.robot.left_gripper_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("Left gripper tactile sensor not available (only quanta_x2 model supported)")

    def _collect_right_gripper_tactile(self):
        """Collect right gripper tactile data"""
        print("Starting right gripper tactile sensor...")
        if hasattr(self.robot, 'right_gripper_tactile'):
            self._collect_generic_stream('right_gripper_tactile', self.robot.right_gripper_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("Right gripper tactile sensor not available (only quanta_x2 model supported)")

    def _collect_left_hand_tactile(self):
        """Collect left dexterous hand tactile data"""
        print("Starting left dexterous hand tactile sensor...")
        if hasattr(self.robot, 'left_hand_tactile'):
            self._collect_generic_stream('left_hand_tactile', self.robot.left_hand_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("Left dexterous hand tactile sensor not available (only quanta_x2 model supported)")

    def _collect_right_hand_tactile(self):
        """Collect right dexterous hand tactile data"""
        print("Starting right dexterous hand tactile sensor...")
        if hasattr(self.robot, 'right_hand_tactile'):
            self._collect_generic_stream('right_hand_tactile', self.robot.right_hand_tactile.get_tactile_sensor_data_stream)
        else:
            raise RuntimeError("Right dexterous hand tactile sensor not available (only quanta_x2 model supported)")

    def _collect_master_left_arm_end_pose(self):
        """Collect master left arm end pose control command"""
        print("Starting master left arm end pose control command collection...")
        try:
            self._collect_pose_stream('master_left_arm_end_pose', self.robot.master_left_arm.get_end_pose_stream)  # Placeholder
        except Exception as e:
            print(f"Master left arm end pose control command collection failed: {e}")
            raise

    def _collect_master_right_arm_end_pose(self):
        """Collect master right arm end pose control command"""
        print("Starting master right arm end pose control command collection...")
        try:
            self._collect_pose_stream('master_right_arm_end_pose', self.robot.master_right_arm.get_end_pose_stream)  # Placeholder
        except Exception as e:
            print(f"Master right arm end pose control command collection failed: {e}")
            raise
    
    def _collect_master_left_arm_joint_state(self):
        """Collect master left arm joint control command"""
        print("Starting master left arm joint control command collection...")
        try:
            self._collect_joint_state_stream('master_left_arm_joint_state', self.robot.master_left_arm.get_joint_states_stream)  # Placeholder
        except Exception as e:
            print(f"Master left arm joint control command collection failed: {e}")
            raise

    def _collect_master_right_arm_joint_state(self):
        """Collect master right arm joint control command"""
        print("Starting master right arm joint control command collection...")
        try:
            self._collect_joint_state_stream('master_right_arm_joint_state', self.robot.master_right_arm.get_joint_states_stream)  # Placeholder
        except Exception as e:
            print(f"Master right arm joint control command collection failed: {e}")
            raise

    def _collect_master_left_gripper_joint_state(self):
        """Collect master left gripper joint control command"""
        print("Starting master left gripper joint control command collection...")
        try:
            self._collect_joint_state_stream('master_left_gripper_joint_state', self.robot.master_left_arm.get_gripper_joint_states_stream)  # Placeholder
        except Exception as e:
            print(f"Master left gripper joint control command collection failed: {e}")
            raise

    def _collect_master_right_gripper_joint_state(self):
        """Collect master right gripper joint control command"""
        print("Starting master right gripper joint control command collection...")
        try:
            self._collect_joint_state_stream('master_right_gripper_joint_state', self.robot.master_right_arm.get_gripper_joint_states_stream)  # Placeholder
        except Exception as e:
            print(f"Master right gripper joint control command collection failed: {e}")
            raise

    def _collect_left_arm_wrench_ext_world(self):
        """Collect master left arm wrench ext world control command"""
        print("Starting master left arm wrench ext world control command collection...")
        try:
            self._collect_generic_stream('left_arm_wrench_ext_world', self.robot.left_arm.get_wrench_ext_world_stream)  # Placeholder
        except Exception as e:
            print(f"Master left arm wrench ext world control command collection failed: {e}")
            raise

    def _collect_left_arm_wrench_ext_local(self):
        """Collect master left arm wrench ext local control command"""
        print("Starting master left arm wrench ext local control command collection...")
        try:
            self._collect_generic_stream('left_arm_wrench_ext_local', self.robot.left_arm.get_wrench_ext_local_stream)  # Placeholder
        except Exception as e:
            print(f"Master left arm wrench ext local control command collection failed: {e}")
            raise

    def _collect_right_arm_wrench_ext_world(self):
        """Collect master right arm wrench ext world control command"""
        print("Starting master right arm wrench ext world control command collection...")
        try:
            self._collect_generic_stream('right_arm_wrench_ext_world', self.robot.right_arm.get_wrench_ext_world_stream)  # Placeholder
        except Exception as e:
            print(f"Master right arm wrench ext world control command collection failed: {e}")
            raise

    def _collect_right_arm_wrench_ext_local(self):
        """Collect master right arm wrench ext local control command"""
        print("Starting master right arm wrench ext local control command collection...")
        try:
            self._collect_generic_stream('right_arm_wrench_ext_local', self.robot.right_arm.get_wrench_ext_local_stream)  # Placeholder
        except Exception as e:
            print(f"Master right arm wrench ext local control command collection failed: {e}")
            raise

    def _collect_left_gripper_position(self):
        """Collect left gripper position"""
        print("Starting left gripper position collection...")
        try:
            self._collect_generic_stream('left_gripper_position', self.robot.left_gripper.get_position_stream)  # Placeholder
        except Exception as e:
            print(f"Left gripper position collection failed: {e}")
            raise

    def _collect_right_gripper_position(self):
        """Collect right gripper position"""
        print("Starting right gripper position collection...")
        try:
            self._collect_generic_stream('right_gripper_position', self.robot.right_gripper.get_position_stream)  # Placeholder
        except Exception as e:
            print(f"Right gripper position collection failed: {e}")
            raise

    def _collect_vr_left_arm_pose_commands(self):
        """Collect VR left arm pose control command"""
        print("Starting VR left arm pose control command collection...")
        try:
            self._collect_generic_stream('vr_left_arm_pose_commands', self.robot.action_data_collection.get_vr_left_arm_pose_commands)  # Placeholder
        except Exception as e:
            print(f"VR left arm pose control command collection failed: {e}")
            raise

    def _collect_vr_right_arm_pose_commands(self):
        """Collect VR right arm pose control command"""
        print("Starting VR right arm pose control command collection...")
        try:
            self._collect_generic_stream('vr_right_arm_pose_commands', self.robot.action_data_collection.get_vr_right_arm_pose_commands)  # Placeholder
        except Exception as e:
            print(f"VR right arm pose control command collection failed: {e}")
            raise

    def _collect_vr_left_gripper_joint_commands(self):
        """Collect VR left gripper joint control command"""
        print("Starting VR left gripper joint control command collection...")
        try:
            self._collect_generic_stream('vr_left_gripper_joint_commands', self.robot.action_data_collection.get_vr_left_gripper_joint_commands)  # Placeholder
        except Exception as e:
            print(f"VR left gripper joint control command collection failed: {e}")
            raise

    def _collect_vr_right_gripper_joint_commands(self):
        """Collect VR right gripper joint control command"""
        print("Starting VR right gripper joint control command collection...")
        try:
            self._collect_generic_stream('vr_right_arm_pose_commands', self.robot.action_data_collection.get_vr_right_gripper_joint_commands)  # Placeholder
        except Exception as e:
            print(f"VR right gripper joint control command collection failed: {e}")
            raise


    # ============ Distance sensor collection methods ============

    def _collect_tof_sensors(self):
        """Collect ToF sensor data"""
        print("Starting ToF sensor collection...")
        # Collect two ToF sensors
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
            # Save thread information for restart (using lambda to wrap parameters)
            if hasattr(self, 'thread_info'):
                def _tof_collector_wrapper():
                    self._collect_generic_stream(queue_name, stream_func)
                self.thread_info.append((f'ToF_{i}_Collector', _tof_collector_wrapper))
                self.threads.append(thread)

        # Do not call join(), let the thread run in the background
        # When is_recording = False, the thread will exit the loop, and the method will end
        # When restarting, these threads will be recreated

    def _collect_ultrasonic_sensors(self):
        """Collect ultrasonic sensor data"""
        print("Starting ultrasonic sensor collection...")
        # Collect four ultrasonic sensors
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
            # Save thread information for restart (using lambda to wrap parameters)
            if hasattr(self, 'thread_info'):
                def _ultrasonic_collector_wrapper():
                    self._collect_generic_stream(queue_name, stream_func)
                self.thread_info.append((f'Ultrasonic_{i}_Collector', _ultrasonic_collector_wrapper))
                self.threads.append(thread)

        # Do not call join(), let the thread run in the background
        # When is_recording = False, the thread will exit the loop, and the method will end
        # When restarting, these threads will be recreated

    # ============ General auxiliary methods ============

    def _start_camera_monitor(self):
        """Start camera data monitoring thread"""
        if self.camera_monitor_thread is not None and self.camera_monitor_thread.is_alive():
            return
        
        self.camera_monitor_thread = threading.Thread(
            target=self._monitor_camera_data,
            daemon=True,
            name="CameraDataMonitor"
        )
        self.camera_monitor_thread.start()
    
    def _stop_camera_monitor(self):
        """Stop camera data monitoring thread"""
        # The monitoring thread will exit automatically when is_recording = False
        pass
    
    def _stop_all_threads(self):
        """Stop all data collection threads"""
        # Set flag to let all threads exit the loop
        self.is_recording = False
        
        # Stop camera data monitoring thread
        self._stop_camera_monitor()
        
        # Wait for threads to exit the loop, stop reading data from stream
        # This can avoid the process exiting due to stream errors during data processing
        if hasattr(self, 'threads') and self.threads:
            print("Waiting for collection threads to stop...")
            time.sleep(0.5)  # Give threads enough time to exit the loop
            
            # Fast check thread status, but not block too long
            # Thread will exit the loop when is_recording=False
            alive_threads = [t for t in self.threads if t.is_alive()]
            if alive_threads:
                # Give threads a very short time to exit (0.1 seconds), then continue
                # Thread is daemon thread, even if it doesn't fully exit, it won't block the main process
                for thread in alive_threads:
                    thread.join(timeout=0.1)
            
            # Clear thread list
            self.threads = []
    
    def _cleanup_before_exit(self, message="Cleaning up resources..."):
        """Uniform cleanup function: stop threads first, then clean up temporary files (for cleanup before exit)
        
        Note: This function only handles cleanup, not exit, exit is decided by caller
        
        Args:
            message: Prompt message before cleanup
        """
        
        # First step: stop all threads (to avoid writing to closed files)
        try:
            self._stop_all_threads()
        except Exception as e:
            print(f"Error stopping threads: {e}")

        # Second step: clean up temporary files
        try:
            if message:
                print(message)

            self._cleanup_temp_files()
            print("Temporary files cleaned up")
        except Exception as e:
            print(f"  ⚠️  Clean up temporary files failed: {e}")
    
    def _get_enabled_cameras(self):
        """Get the list of actually enabled cameras (according to configuration)"""
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
        """Monitor camera data, if a camera has no data, report an error and exit"""
        recording_start_time = time.time()
        
        # Get the list of actually enabled cameras
        enabled_cameras = self._get_enabled_cameras()
        
        # If there are no enabled cameras, no need to monitor
        if not enabled_cameras:
            return
        
        while self.is_recording:
            current_time = time.time()
            elapsed_time = current_time - recording_start_time
            
            # Only check after recording starts (give cameras some time to start collecting)
            # At least wait for the timeout time to determine if the camera has no data
            if elapsed_time < self.camera_data_timeout:
                time.sleep(self.camera_data_check_interval)
                continue
            
            # Only check actually enabled cameras
            cameras_without_data = []
            with self.stats_lock:
                for cam in enabled_cameras:
                    # Ensure camera is in tracking list
                    if cam not in self.camera_last_data_time:
                        continue
                    
                    last_data_time = self.camera_last_data_time.get(cam)
                    if last_data_time is None:
                        # Never received data, but only report error after recording time exceeds timeout
                        if elapsed_time >= self.camera_data_timeout:
                            cameras_without_data.append(cam)
                    else:
                        # Check if timeout
                        time_since_last_data = current_time - last_data_time
                        if time_since_last_data > self.camera_data_timeout:
                            cameras_without_data.append(cam)
            
            if cameras_without_data:
                error_msg = f"❌ Detected the following cameras have no data (timeout {self.camera_data_timeout} seconds):\n"
                for cam in cameras_without_data:
                    last_time = self.camera_last_data_time.get(cam)
                    if last_time is None:
                        error_msg += f"  - {cam}: Never received data\n"
                    else:
                        time_since = current_time - last_time
                        error_msg += f"  - {cam}: Last data time {time_since:.2f} seconds ago\n"
                error_msg += "Process will exit..."
                # Clean up temporary files before exiting
                self._cleanup_before_exit(error_msg)
                os._exit(1)
            
            time.sleep(self.camera_data_check_interval)
    
    def _restart_stopped_threads(self):
        """Restart stopped threads"""
        if not hasattr(self, 'thread_info') or not self.thread_info:
            return
        
        # Check which threads have stopped
        stopped_threads = []
        for i, thread in enumerate(self.threads):
            if not thread.is_alive():
                stopped_threads.append(i)
        
        if stopped_threads:
            print(f"Detected {len(stopped_threads)} threads have stopped, restarting...")
            for i in stopped_threads:
                thread_name, target_func = self.thread_info[i]
                # Create new thread
                new_thread = threading.Thread(target=target_func, daemon=True, name=thread_name)
                new_thread.start()
                self.threads[i] = new_thread
                print(f"    ✓ Restart thread: {thread_name}")

    def _is_grpc_connection_error(self, error):
        """Check if it is a gRPC connection error"""
        error_str = str(error).lower()
        error_type = type(error).__name__
        
        # Check error type
        if 'MultiThreadedRendezvous' in error_type or 'grpc' in error_type.lower():
            return True
        
        # Check error message
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
        """Handle gRPC connection error and exit process"""
        err_msg = f"❌ {queue_name} Network connection interrupted: {error}\n"
        err_msg += f"   Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}\n"
        err_msg += f"   Please check the network connection and restart data collection"
        # Clean up resources before exiting
        self._cleanup_before_exit(err_msg)
        print(f"Process will exit...")
        # Exit process directly (using os._exit to avoid triggering other cleanup logic, but we have already manually cleaned up)
        os._exit(1)

    def _collect_joint_state_stream(self, queue_name, stream_func):
        """General joint state stream collection method"""
        # Store the joint names of each part (save the first time the message is received)
        joint_names_cache = {}
        
        try:
            stream = stream_func(timeout=None)
            message_count = 0
            
            for joint_state_msg in stream:
                # If not in recording, exit the loop
                if not self.is_recording:
                    break
                
                try:
                    timestamp = self._extract_timestamp_from_header(joint_state_msg)
                    message_count += 1
                    
                    # The first time the message is received, save the joint names
                    if queue_name not in joint_names_cache:
                        if hasattr(joint_state_msg, 'name') and joint_state_msg.name and len(joint_state_msg.name) > 0:
                            joint_names_cache[queue_name] = list(joint_state_msg.name)
                            print(f"  ✓ {queue_name} joint names: {joint_names_cache[queue_name]}")
                            # Save to instance variable for subsequent use
                            if not hasattr(self, '_joint_names_by_part'):
                                self._joint_names_by_part = {}
                            self._joint_names_by_part[queue_name] = joint_names_cache[queue_name]
                    
                    if self.is_recording:
                        # Extract joint data
                        positions = np.array(joint_state_msg.position, dtype=np.float32).flatten() if joint_state_msg.position else np.array([], dtype=np.float32)
                        velocities = np.array(joint_state_msg.velocity, dtype=np.float32).flatten() if hasattr(joint_state_msg, 'velocity') and joint_state_msg.velocity else None
                        efforts = np.array(joint_state_msg.effort, dtype=np.float32).flatten() if hasattr(joint_state_msg, 'effort') and joint_state_msg.effort else None
                        
                        # Store joint state data to temporary file
                        if queue_name in self.sensor_file_locks and hasattr(self, 'sensor_temp_files') and queue_name in self.sensor_temp_files:
                            joint_state_data_tuple = (timestamp, positions, velocities, efforts)
                            with self.sensor_file_locks[queue_name]:
                                if queue_name in self.sensor_temp_files:
                                    pickle.dump(joint_state_data_tuple, self.sensor_temp_files[queue_name])
                                    self.sensor_temp_files[queue_name].flush()
                            with self.stats_lock:
                                self.stats[f'{queue_name}_count'] = self.stats.get(f'{queue_name}_count', 0) + 1
                        else:
                            if message_count == 1:  # Only print once, to avoid spamming
                                print(f"  ⚠️  Warning: {queue_name} temporary file does not exist, cannot save data")
                        
                        # Note: action data does not need to be saved separately, the next frame's state will be used as action when saving episode
                except Exception as e:
                    print(f"{queue_name} Error processing: {e}")
                    import traceback
                    traceback.print_exc()
                    continue
            
            # If the thread exits but no messages are received, print a warning
            if message_count == 0:
                print(f"  ⚠️  Warning: {queue_name} thread exited, but no data messages were received")
        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(queue_name, e)
            print(f"{queue_name} Stream error: {e}")
            import traceback
            traceback.print_exc()
            raise

    def _collect_pose_stream(self, queue_name, stream_func):
        """General end-effector pose stream collection method"""
        try:
            stream = stream_func(timeout=None)
            for pose_msg in stream:
                # If not in recording, exit the loop
                if not self.is_recording:
                    break
                
                # If not in recording, exit the loop (thread will stop)
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
                        # Store to corresponding temporary file
                        if queue_name in self.sensor_file_locks and hasattr(self, 'sensor_temp_files') and queue_name in self.sensor_temp_files:
                            pose_data_tuple = (timestamp, pose_data)
                            with self.sensor_file_locks[queue_name]:
                                if queue_name in self.sensor_temp_files:
                                    pickle.dump(pose_data_tuple, self.sensor_temp_files[queue_name])
                                    self.sensor_temp_files[queue_name].flush()
                            with self.stats_lock:
                                self.stats[f'{queue_name}_count'] = self.stats.get(f'{queue_name}_count', 0) + 1

                except Exception as e:
                    print(f"{queue_name} Error processing: {e}")
                    continue

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(queue_name, e)
            print(f"{queue_name} Stream error: {e}")
            raise
    
    def _convert_ros_msg_to_dict(self, msg, exclude_fields=None):
        """Recursively convert protobuf message object to a JSON serializable dictionary
        
        Args:
            msg: protobuf message object
            exclude_fields: Collection of field names to exclude (optional)
            
        Returns:
            dict or basic type
        """
        # Default excluded fields: timestamp (aligned) and private fields
        if exclude_fields is None:
            exclude_fields = {'stamp', 'header'}  # stamp for sensor timestamp, header for ROS message header
        
        # If it is a basic type, return directly
        if isinstance(msg, (int, float, str, bool, type(None))):
            return msg
        
        # If it is a NumPy array, convert to list
        if isinstance(msg, np.ndarray):
            return msg.tolist()
        
        # If it is a list or tuple, recursively process each element
        if isinstance(msg, (list, tuple)):
            return [self._convert_ros_msg_to_dict(item, exclude_fields) for item in msg]
        
        # If it is a dictionary, recursively process each value
        if isinstance(msg, dict):
            return {k: self._convert_ros_msg_to_dict(v, exclude_fields) for k, v in msg.items() if k not in exclude_fields}
        
        # If it is a protobuf message object (has DESCRIPTOR attribute)
        if hasattr(msg, 'DESCRIPTOR'):
            result = {}
            # Use ListFields() to get all set fields
            if hasattr(msg, 'ListFields'):
                for field_descriptor, value in msg.ListFields():
                    field_name = field_descriptor.name
                    if field_name not in exclude_fields:
                        result[field_name] = self._convert_ros_msg_to_dict(value, exclude_fields)
            # If ListFields() is empty, try to get all fields from DESCRIPTOR
            elif hasattr(msg, 'DESCRIPTOR'):
                for field in msg.DESCRIPTOR.fields:
                    field_name = field.name
                    if field_name not in exclude_fields and hasattr(msg, field_name):
                        value = getattr(msg, field_name)
                        # Only add fields with non-default values
                        if value is not None and value != field.default_value:
                            result[field_name] = self._convert_ros_msg_to_dict(value, exclude_fields)
            return result
        
        # If it has __dict__ attribute (preferred over __slots__, because dataclass may have empty __slots__)
        if hasattr(msg, '__dict__'):
            msg_dict = msg.__dict__
            # If __dict__ is not empty, use it (filter out private fields starting with underscore and excluded fields)
            if msg_dict:
                return {k: self._convert_ros_msg_to_dict(v, exclude_fields) 
                       for k, v in msg_dict.items() 
                       if not k.startswith('_') and k not in exclude_fields}
        
        # If it is a ROS message object (has __slots__ attribute and __slots__ is not empty)
        if hasattr(msg, '__slots__') and msg.__slots__:
            result = {}
            for slot in msg.__slots__:
                if slot not in exclude_fields and hasattr(msg, slot):
                    value = getattr(msg, slot)
                    result[slot] = self._convert_ros_msg_to_dict(value, exclude_fields)
            return result
        
        # Other cases, try to convert to string
        try:
            return str(msg)
        except:
            return None
    
    def _collect_generic_stream(self, queue_name, stream_func):
        """General sensor stream collection method"""
        try:
            stream = stream_func(timeout=None)
            for msg in stream:
                # If not in recording, exit the loop
                if not self.is_recording:
                    break
                
                # If not in recording, exit the loop (thread will stop)
                if not self.is_recording:
                    break

                try:
                    timestamp = self._extract_timestamp_from_header(msg)
                    if self.is_recording:
                        # Store original message data
                        data = {
                            'timestamp': timestamp,
                            'data': msg
                        }
                        # Store to corresponding temporary file
                        if queue_name in self.sensor_file_locks and hasattr(self, 'sensor_temp_files') and queue_name in self.sensor_temp_files:
                            sensor_data_tuple = (timestamp, data)
                            with self.sensor_file_locks[queue_name]:
                                if queue_name in self.sensor_temp_files:
                                    pickle.dump(sensor_data_tuple, self.sensor_temp_files[queue_name])
                                    self.sensor_temp_files[queue_name].flush()
                            with self.stats_lock:
                                self.stats[f'{queue_name}_count'] = self.stats.get(f'{queue_name}_count', 0) + 1

                except Exception as e:
                    print(f"{queue_name} Error processing: {e}")
                    continue

        except Exception as e:
            if self._is_grpc_connection_error(e):
                self._handle_grpc_error_and_exit(queue_name, e)
            # Other types of errors, rethrow
            print(f"❌ {queue_name} Stream error: {e}, time:{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')}")
            raise


    def get_stats(self):
        """Get current statistics"""
        with self.stats_lock:
            stats = self.stats.copy()
            if self.current_episode_start_time:
                stats['recording_duration'] = time.time() - self.current_episode_start_time
            return stats
    
    def print_stats(self):
        """Print current statistics"""
        stats = self.get_stats()
        print(f"\n=== Collection Statistics ===")
        if 'recording_duration' in stats:
            print(f"Recording duration: {stats['recording_duration']:.2f}s")
        print(f"State frame count: {stats.get('state_count', 0)}")
        print(f"Action frame count: {stats.get('action_count', 0)}")
        for cam, count in stats.get('image_count', {}).items():
            print(f"{cam}: {count} frames")
        # Show all sensor statistics
        for stat_key, count in sorted(stats.items()):
            if stat_key.endswith('_count') and stat_key not in ['state_count', 'action_count', 'image_count']:
                # Only check for integer type count > 0
                if isinstance(count, int) and count > 0:
                    sensor_name = stat_key.replace('_count', '')
                    if sensor_name in ['head_camera', 'left_arm_camera', 'right_arm_camera',
                                       'head_depth_video', 'head_rgb_video']:
                        print(f"{sensor_name}: {count} frame")
                    else:
                        print(f"{sensor_name}: {count}")

        # Compatibility with old display method
        if self.collection_config.enable_chassis_imu:
            print(f"IMU: {stats.get('chassis_imu_count', 0)} frames")
        if self.collection_config.enable_depth_points:
            print(f"Depth: {stats.get('depth_points_count', 0)} frames")
        
        # Show file storage status
        sensor_files = len([s for s in self.sensor_file_locks.keys() if hasattr(self, 'sensor_temp_files') and s in self.sensor_temp_files])
        hf_files = len([h for h in ['joint_states', 'actions'] if hasattr(self, 'sensor_temp_files') and h in self.sensor_temp_files])

        if sensor_files > 0 or hf_files > 0:
            print(f"Data storage status: ✓ {sensor_files} sensor files, ✓ {hf_files} high-frequency files")
            print(f"Storage strategy: All files written (non-blocking, no memory limit)")

            # Check if there is any overwrite
            if hasattr(self, '_queue_overwrites'):
                overwrites = self._queue_overwrites
                if overwrites > 0:
                    print(f"⚠️  Warning: Some data may be overwritten due to processing delay")
        else:
            print(f"Data storage: ✗ No file storage")
        
        print("================\n")
    
    def _extract_timestamp_from_header(self, msg):
        """Extract timestamp from the header of the message"""
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            sec = msg.header.stamp.sec
            nanosec = msg.header.stamp.nanosec
            return float(sec) + float(nanosec) / 1e9
        else:
            # If there is no header, use local timestamp (compatible with old format)
            return time.time()
    
    def _get_enabled_data_mapping(self):
        """Get the mapping of enabled configuration items to data key names
        
        Returns:
            dict: {configuration item name: [data key name list]}
        """
        mapping = {}
        
        # Joint state collection
        if self.slave_joint_names:
            mapping['slave_joint_names'] = self.slave_joint_names
        
        # Image stream collection
        if self.collection_config.enable_head_rgb_stream:
            mapping['enable_head_rgb_stream'] = ['head_rgb_stream']
        if self.collection_config.enable_head_depth_stream:
            mapping['enable_head_depth_stream'] = ['head_depth_stream']
        if self.collection_config.enable_left_arm_rgb_stream:
            mapping['enable_left_arm_rgb_stream'] = ['left_arm_rgb_stream']
        if self.collection_config.enable_right_arm_rgb_stream:
            mapping['enable_right_arm_rgb_stream'] = ['right_arm_rgb_stream']
        
        # Sensor collection
        if self.collection_config.enable_left_arm_end_pose:
            mapping['enable_left_arm_end_pose'] = ['left_arm_end_pose']
        if self.collection_config.enable_right_arm_end_pose:
            mapping['enable_right_arm_end_pose'] = ['right_arm_end_pose']
        if self.collection_config.enable_waist_end_pose:
            mapping['enable_waist_end_pose'] = ['waist_end_pose']
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
        
        # Tactile sensors
        if self.collection_config.enable_left_gripper_tactile:
            mapping['enable_left_gripper_tactile'] = ['left_gripper_tactile']
        if self.collection_config.enable_right_gripper_tactile:
            mapping['enable_right_gripper_tactile'] = ['right_gripper_tactile']
        if self.collection_config.enable_left_hand_tactile:
            mapping['enable_left_hand_tactile'] = ['left_hand_tactile']
        if self.collection_config.enable_right_hand_tactile:
            mapping['enable_right_hand_tactile'] = ['right_hand_tactile']
        
        # Distance sensors
        if self.collection_config.enable_tof_sensors:
            mapping['enable_tof_sensors'] = ['tof_1', 'tof_2']
        if self.collection_config.enable_ultrasonic_sensors:
            mapping['enable_ultrasonic_sensors'] = [f'ultrasonic_{i}' for i in range(1, 5)]
        
        # Master arm data
        if self.collection_config.enable_master_arm_data:
            mapping['enable_master_arm_data'] = [
                'master_left_arm_joint_state',
                'master_right_arm_joint_state',
                'master_left_arm_end_pose',
                'master_right_arm_end_pose',
                'master_left_gripper_joint_state',
                'master_right_gripper_joint_state'
            ]
        
        # Force sensors
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

        if self.collection_config.enable_left_gripper_position:
            mapping['enable_left_gripper_position'] = ['left_gripper_position']
        if self.collection_config.enable_right_gripper_position:
            mapping['enable_right_gripper_position'] = ['right_gripper_position']
        
        return mapping
    
    def _validate_collected_data(self, sensor_data: Dict[str, Any], images: Dict[str, Any]) -> bool:
        """Validate collected raw data (before alignment)
        
        Check if all enabled data items have actual data (read from temporary files)
        
        Args:
            sensor_data: Raw sensor data collected from temporary files
            images: Raw image data collected from temporary files
            
        Returns:
            True if all enabled data items have data, False otherwise
        """
        # Get the mapping of enabled configuration items
        enabled_mapping = self._get_enabled_data_mapping()
        
        # Check if all enabled data items have data
        missing_data_items = []
        
        # For recording the missing cases of master arm data (only warning, not error, exclude action data)
        master_arm_missing_items = []
        
        # Camera name mapping: internal stream name -> user-friendly name
        camera_name_mapping = {
            'head_rgb_stream': 'head_camera',
            'head_depth_stream': 'head_depth_camera',
            'left_arm_rgb_stream': 'left_arm_camera',
            'right_arm_rgb_stream': 'right_arm_camera'
        }
        
        for config_name, data_keys in enabled_mapping.items():
            # Check the master arm data (only warning, not required, exclude action data)
            if config_name == 'enable_master_arm_data':
                # Only check the joint_state and end_pose of the master arm, not check action
                for key in data_keys:
                    # Skip data related to action
                    if 'action' in key.lower():
                        continue
                    
                    # Check sensor data
                    if key in sensor_data:
                        if len(sensor_data[key]) == 0:
                            master_arm_missing_items.append((key, f"Data is empty ({len(sensor_data[key])} items)"))
                    else:
                        available_keys = list(sensor_data.keys())[:10]  # Only show the first 10 keys
                        master_arm_missing_items.append((key, f"Data does not exist (sensor_data has: {available_keys})"))
                continue
            
            # Check image data
            if any(key.endswith('_rgb_stream') or key.endswith('_depth_stream') or key.endswith('_depth_video') 
                   for key in data_keys):
                # Image data in images dictionary (using original internal key names)
                for internal_key in data_keys:
                    if internal_key in images:
                        if len(images[internal_key]) == 0:
                            friendly_name = camera_name_mapping.get(internal_key, internal_key)
                            missing_data_items.append((config_name, friendly_name, "Image data is empty"))
                    else:
                        friendly_name = camera_name_mapping.get(internal_key, internal_key)
                        missing_data_items.append((config_name, friendly_name, f"Image data does not exist (internal key: {internal_key})"))
            else:
                # Sensor data in sensor_data dictionary
                for key in data_keys:
                    if key in sensor_data:
                        if len(sensor_data[key]) == 0:
                            missing_data_items.append((config_name, key, f"Data is empty ({len(sensor_data[key])} items)"))
                    else:
                        # Provide more detailed error information
                        available_keys = list(sensor_data.keys())[:10]  # Only show the first 10 keys
                        missing_data_items.append((config_name, key, f"Data does not exist (sensor_data has: {available_keys})"))
        
        # If there are missing data items, exit with error
        if missing_data_items:
            error_parts = ["Error: The following enabled data items have no data collected:\n"]
            
            # Group by configuration item
            by_config = {}
            for config_name, data_key, reason in missing_data_items:
                if config_name not in by_config:
                    by_config[config_name] = []
                by_config[config_name].append((data_key, reason))
            
            for config_name, items in by_config.items():
                error_parts.append(f"  - {config_name}:")
                for data_key, reason in items:
                    error_parts.append(f"    • {data_key}: {reason}")
            
            error_parts.append("\nPlease check the robot connection and configuration, ensure that all enabled data items can be collected normally.")
            
            error_msg = "\n".join(error_parts)
            print(f"  {error_msg}")
            return False
        
        # Master arm data missing warning (not error, exit)
        if master_arm_missing_items:
            print(f"\n  ⚠️  Warning: enable_master_arm_data is enabled, but the following master arm data items are missing (does not affect data collection):")
            for data_key, reason in master_arm_missing_items:
                print(f"    • {data_key}: {reason}")
        
        return True
    
    def _validate_episode_data(self, episode_data):
        """Validate episode data quality"""
        if episode_data is None:
            return False
        
        # Get data from the aligned data structure
        sensor_data = episode_data.get('sensor_data', {})
        action_data = episode_data.get('action_data', {})
        images = episode_data.get('images', {})
        
        # Get the mapping of enabled configuration items
        enabled_mapping = self._get_enabled_data_mapping()
        
        # Check if all enabled data items have data
        missing_data_items = []
        
        # Camera name mapping: internal stream name -> user-friendly name
        camera_name_mapping = episode_data.get('camera_name_mapping', {
            'head_rgb_stream': 'head_camera',
            'head_depth_stream': 'head_depth_camera',
            'left_arm_rgb_stream': 'left_arm_camera',
            'right_arm_rgb_stream': 'right_arm_camera'
        })
        
        # For recording the missing cases of master arm data (only warning, not error)
        master_arm_missing_items = []
        
        for config_name, data_keys in enabled_mapping.items():
            # Check the master arm data (only warning, not required)
            if config_name == 'enable_master_arm_data':
                # Check image data
                if any(key.endswith('_rgb_stream') or key.endswith('_depth_stream') or key.endswith('_depth_video') 
                       for key in data_keys):
                    # Image data in images dictionary (using mapped friendly name)
                    if self.use_video_storage and 'image_index_mapping' in episode_data:
                        # Video storage mode: check image_index_mapping
                        image_index_mapping = episode_data.get('image_index_mapping', {})
                        for internal_key in data_keys:
                            friendly_name = camera_name_mapping.get(internal_key, internal_key)
                            if friendly_name in image_index_mapping:
                                if len(image_index_mapping[friendly_name]) == 0:
                                    master_arm_missing_items.append((friendly_name, "Image data is empty"))
                            else:
                                master_arm_missing_items.append((friendly_name, f"Image data does not exist (internal key: {internal_key})"))
                    else:
                        # Image storage mode: check aligned_images (in images dictionary)
                        for internal_key in data_keys:
                            friendly_name = camera_name_mapping.get(internal_key, internal_key)
                            if friendly_name in images:
                                if len(images[friendly_name]) == 0:
                                    master_arm_missing_items.append((friendly_name, "Image data is empty"))
                            else:
                                # Also check the original internal key names (before data alignment)
                                if internal_key in images:
                                    if len(images[internal_key]) == 0:
                                        master_arm_missing_items.append((friendly_name, "Image data is empty"))
                                else:
                                    master_arm_missing_items.append((friendly_name, f"Image data does not exist (internal key: {internal_key})"))
                else:
                    # Sensor data in sensor_data or action_data dictionary
                    # Only check the joint_state and end_pose of the master arm, not check action
                    for key in data_keys:
                        # Skip data related to action
                        if 'action' in key.lower():
                            continue
                        
                        found = False
                        
                        # Check sensor_data first
                        if key in sensor_data:
                            if len(sensor_data[key]) > 0:
                                found = True
                            else:
                                master_arm_missing_items.append((key, f"Data is empty ({len(sensor_data[key])} items)"))
                                continue
                        
                        # If not found, record warning
                        if not found:
                            available_sensor_keys = list(sensor_data.keys())[:10]  # Only show the first 10 keys
                            error_info = f"Data does not exist (sensor_data has: {available_sensor_keys})"
                            master_arm_missing_items.append((key, error_info))
                continue
            
            # Check image data
            if any(key.endswith('_rgb_stream') or key.endswith('_depth_stream') or key.endswith('_depth_video') 
                   for key in data_keys):
                # Image data in images dictionary (using mapped friendly name)
                if self.use_video_storage and 'image_index_mapping' in episode_data:
                    # Video storage mode: check image_index_mapping
                    image_index_mapping = episode_data.get('image_index_mapping', {})
                    for internal_key in data_keys:
                        friendly_name = camera_name_mapping.get(internal_key, internal_key)
                        if friendly_name in image_index_mapping:
                            if len(image_index_mapping[friendly_name]) == 0:
                                missing_data_items.append((config_name, friendly_name, "Image data is empty"))
                        else:
                            missing_data_items.append((config_name, friendly_name, f"Image data does not exist (internal key: {internal_key})"))
                else:
                    # Image storage mode: check aligned_images (in images dictionary)
                    for internal_key in data_keys:
                        friendly_name = camera_name_mapping.get(internal_key, internal_key)
                        if friendly_name in images:
                            if len(images[friendly_name]) == 0:
                                missing_data_items.append((config_name, friendly_name, "Image data is empty"))
                        else:
                            # Also check the original internal key names (before data alignment)
                            if internal_key in images:
                                if len(images[internal_key]) == 0:
                                    missing_data_items.append((config_name, friendly_name, "Image data is empty"))
                            else:
                                missing_data_items.append((config_name, friendly_name, f"Image data does not exist (internal key: {internal_key})"))
            else:
                # Sensor data in sensor_data or action_data dictionary
                for key in data_keys:
                    found = False
                    data_source = None
                    
                    # Check sensor_data first
                    if key in sensor_data:
                        if len(sensor_data[key]) > 0:
                            found = True
                            data_source = "sensor_data"
                        else:
                            missing_data_items.append((config_name, key, f"Data is empty ({len(sensor_data[key])} items)"))
                            continue
                    
                    # If not in sensor_data, check action_data (some data may be there)
                    if not found and key in action_data:
                        if len(action_data[key]) > 0:
                            found = True
                            data_source = "action_data"
                        else:
                            missing_data_items.append((config_name, key, f"Data is empty ({len(action_data[key])} items)"))
                            continue
                    
                    # If not found, exit with error
                    if not found:
                        # Provide more detailed error information
                        available_sensor_keys = list(sensor_data.keys())[:10]  # Only show the first 10 keys
                        available_action_keys = list(action_data.keys())[:10]
                        error_info = f"Data does not exist (sensor_data has: {available_sensor_keys}, action_data has: {available_action_keys})"
                        missing_data_items.append((config_name, key, error_info))
        
        # If there are missing data items, exit with error
        if missing_data_items:
            error_parts = ["Error: The following enabled data items have no data collected:\n"]
            
            # Group by configuration item
            by_config = {}
            for config_name, data_key, reason in missing_data_items:
                if config_name not in by_config:
                    by_config[config_name] = []
                by_config[config_name].append((data_key, reason))
            
            for config_name, items in by_config.items():
                error_parts.append(f"  - {config_name}:")
                for data_key, reason in items:
                    error_parts.append(f"    • {data_key}: {reason}")
            
            error_parts.append("\nPlease check the robot connection and configuration, ensure that all enabled data items can be collected normally.")
            
            error_msg = "\n".join(error_parts)
            # Clean up temporary files before exiting
            self._cleanup_before_exit(error_msg)
            sys.exit(1)
        
        # Master arm data missing warning (not error, exit)
        # Note: Warning is only printed in _validate_collected_data, here is not repeated to avoid repetition
        
        # Use the joint state names in the member variable (for subsequent statistics)
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        
        # Check if there is any joint state data (compatible with old format)
        states = sensor_data.get('joint_states', [])  # Compatible with old format
        actions = sensor_data.get('actions', [])  # Compatible with old format
        
        # Check if there is any joint state data (compatible with old format)
        joint_states_by_part = {}
        for joint_state_name in joint_state_names:
            if joint_state_name in sensor_data and len(sensor_data[joint_state_name]) > 0:
                joint_states_by_part[joint_state_name] = sensor_data[joint_state_name]
        
        # Check if there is any data
        has_joint_data = len(states) > 0 or len(actions) > 0 or len(joint_states_by_part) > 0
        has_image_data = any(len(imgs) > 0 for imgs in images.values())
        
        if not has_joint_data and not has_image_data:
            print("  Validation failed: no data")
            return False

        # Get the number of state frames (use the data of each part first)
        if joint_states_by_part:
            # Use the data of each part
            state_frame_count = max(len(data) for data in joint_states_by_part.values())
        else:
            # Compatible with old format
            state_frame_count = len(states) if states else 0

        # If image collection is enabled, check the image data
        if len(self.camera_names) > 0:
            # Video storage mode: use image_index_mapping to validate
            if self.use_video_storage and 'image_index_mapping' in episode_data:
                image_index_mapping = episode_data['image_index_mapping']
                for cam_name, indices in image_index_mapping.items():
                    if len(indices) == 0:
                        print(f"  ⚠️  Warning: {cam_name} has no image data")
                    elif has_joint_data and state_frame_count > 0:
                        if abs(len(indices) - state_frame_count) > max(state_frame_count * 0.1, 1):
                            print(f"  ⚠️  Warning: the number of images ({len(indices)}) of {cam_name} is significantly different from the number of states ({state_frame_count})")
            # Image storage mode: check aligned_images
            else:
                for cam_name, cam_images in images.items():
                    if len(cam_images) == 0:
                        print(f"  ⚠️  Warning: {cam_name} has no image data")
                    elif has_joint_data and state_frame_count > 0:
                        if abs(len(cam_images) - state_frame_count) > max(state_frame_count * 0.1, 1):
                            print(f"  ⚠️  Warning: the number of images ({len(cam_images)}) of {cam_name} is significantly different from the number of states ({state_frame_count})")
        
        # Check the data amount
        max_frames = max(state_frame_count, max((len(imgs) for imgs in images.values()), default=0))
        min_frames = int(self.target_hz * 0.5)  # At least 0.5 seconds of data
        if max_frames < min_frames:
            print(f"  ⚠️  Warning: the data amount is too little ({max_frames} frames < {min_frames} frames)")
        
        # Print the data statistics of each part
        if joint_states_by_part:
            print(f"  ✓ Data validation completed: {len(joint_states_by_part)} parts of joint states, {state_frame_count} state frames, {len(images)} cameras")
            for part_name, data in joint_states_by_part.items():
                print(f"    - {part_name}: {len(data)} frames")
        else:
            print(f"  ✓ Data validation completed: {state_frame_count} state frames, {len(images)} cameras")
        return True
    
    def _clear_queues(self):
        """Clear all queues"""
        # Only clear the queues that actually exist
        for queue_name, queue in self.queues.items():
            if queue is not None:
                try:
                    while not queue.empty():
                        queue.get_nowait()  # Non-blocking get, avoid blocking
                except:
                    pass  # The queue may have been modified elsewhere, ignore the error
    
    def _create_temp_files(self):
        """Create temporary files for cameras and high-frequency data"""
        # Ensure that all temporary file dictionaries are initialized
        if not hasattr(self, 'image_temp_files'):
            self.image_temp_files = {}
        if not hasattr(self, 'image_temp_paths'):
            self.image_temp_paths = {}
        if not hasattr(self, 'sensor_temp_files'):
            self.sensor_temp_files = {}
        if not hasattr(self, 'sensor_temp_paths'):
            self.sensor_temp_paths = {}

        self._cleanup_temp_files()
        
        # Camera temporary files
        for camera_name in self.camera_names:
            temp_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{camera_name}.pkl')
            self.image_temp_files[camera_name] = temp_file
            self.image_temp_paths[camera_name] = temp_file.name

        # Sensor data temporary files
        for sensor_name in self.sensor_file_locks.keys():
            temp_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{sensor_name}.pkl')
            self.sensor_temp_files[sensor_name] = temp_file
            self.sensor_temp_paths[sensor_name] = temp_file.name

        # Joint state and action data temporary files - created based on the configured joint_names
        if self.slave_joint_names:
            for joint_state_name in self.slave_joint_names:
                joint_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{joint_state_name}.pkl')
                self.sensor_temp_files[joint_state_name] = joint_file
                self.sensor_temp_paths[joint_state_name] = joint_file.name
                
                # Create the corresponding action file (for future state as action)
                if self.slave_action_names:
                    # Find the corresponding action_name
                    action_name = joint_state_name.replace('_joint_states', '_actions')
                    if action_name in self.slave_action_names:
                        action_file = tempfile.NamedTemporaryFile(mode='wb', delete=False, suffix=f'_{action_name}.pkl')
                        self.sensor_temp_files[action_name] = action_file
                        self.sensor_temp_paths[action_name] = action_file.name
    
    def _cleanup_temp_files(self):
        """Clean up temporary files"""
        cleaned_count = 0
        
        # Clean up camera temporary files
        if hasattr(self, 'image_temp_files'):
            for camera_name in list(self.image_temp_files.keys()):
                if camera_name in self.image_temp_files:
                    try:
                        if not self.image_temp_files[camera_name].closed:
                            self.image_temp_files[camera_name].close()
                    except Exception:
                        pass  # Ignore the error of closing
                    try:
                        if hasattr(self, 'image_temp_paths') and camera_name in self.image_temp_paths:
                            temp_path = self.image_temp_paths[camera_name]
                            if temp_path and os.path.exists(temp_path):
                                os.unlink(temp_path)
                                cleaned_count += 1
                    except Exception as e:
                        print(f"Error cleaning up camera temporary file ({camera_name}): {e}")

        # Clean up sensor data temporary files (now including joint state and action)
        if hasattr(self, 'sensor_temp_files'):
            for sensor_name in list(self.sensor_temp_files.keys()):
                if sensor_name in self.sensor_temp_files:
                    try:
                        if not self.sensor_temp_files[sensor_name].closed:
                            self.sensor_temp_files[sensor_name].close()
                    except Exception:
                        pass  # Ignore the error of closing
                    try:
                        if hasattr(self, 'sensor_temp_paths') and sensor_name in self.sensor_temp_paths:
                            temp_path = self.sensor_temp_paths[sensor_name]
                            if temp_path and os.path.exists(temp_path):
                                os.unlink(temp_path)
                                cleaned_count += 1
                    except Exception as e:
                        print(f"Error cleaning up sensor temporary file ({sensor_name}): {e}")

        # Clear the dictionaries (if they exist)
        if hasattr(self, 'image_temp_files'):
            self.image_temp_files.clear()
        if hasattr(self, 'image_temp_paths'):
            self.image_temp_paths.clear()
        if hasattr(self, 'sensor_temp_files'):
            self.sensor_temp_files.clear()
        if hasattr(self, 'sensor_temp_paths'):
            self.sensor_temp_paths.clear()
        
        # Additional cleanup: scan the temporary directory for possible leftover temporary files (with our suffix)
        # This can clean up files left over when the program exits abnormally
        try:
            temp_dir = tempfile.gettempdir()
            if os.path.isdir(temp_dir):
                # Find all pkl files with our suffix
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
                        # Check if it matches our suffix
                        for suffix in suffixes_to_clean:
                            if filename.endswith(suffix):
                                temp_file_path = os.path.join(temp_dir, filename)
                                try:
                                    # Check if the file is very old (more than 1 hour) or is 0 bytes
                                    file_stat = os.stat(temp_file_path)
                                    file_age = time.time() - file_stat.st_mtime
                                    if file_age > 3600 or file_stat.st_size == 0:
                                        os.unlink(temp_file_path)
                                        cleaned_count += 1
                                except Exception:
                                    pass  # Ignore the error of deleting
                                break
        except Exception as e:
            # Ignore the error of scanning, does not affect main flow
            pass
        
        if cleaned_count > 0:
            print(f"✓ Cleaned up {cleaned_count} temporary files")
    
    def _collect_episode_data(self):
        """Collect episode data from queues and temporary files"""
        # Collect data from sensor temporary files
        sensor_data = {}
        episode_data = {}

        for sensor_name in self.sensor_file_locks.keys():
            if sensor_name not in self.sensor_temp_files:
                continue

            # Close the write handle
            with self.sensor_file_locks[sensor_name]:
                self.sensor_temp_files[sensor_name].close()

            # Reopen the file for reading
            temp_path = self.sensor_temp_paths[sensor_name]
            if not os.path.exists(temp_path):
                print(f"⚠️  Warning: the sensor temporary file does not exist {sensor_name}: {temp_path}")
                continue

            data_list = []
            try:
                with open(temp_path, 'rb') as f:
                    while True:
                        try:
                            data = pickle.load(f)
                            data_list.append(data)
                        except EOFError:
                            break  # File end
                        except Exception as e:
                            print(f"Error reading {sensor_name} data: {e}")
                            break

                if data_list:
                    sensor_data[sensor_name] = data_list
                    print(f"  {sensor_name}: collected {len(data_list)} data")

            except Exception as e:
                print(f"Error reading {sensor_name} temporary file: {e}")
        
        # Read the joint state and action data from the temporary files
        print("Reading joint state and action data from temporary files...")

        # Use the joint state and action names in the member variable
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        action_names = self.slave_action_names if self.slave_action_names else []

        # Read the joint state data of each part
        for data_type in joint_state_names:
            if not hasattr(self, 'sensor_temp_files') or data_type not in self.sensor_temp_files:
                continue

            # Close the write handle
            if data_type in self.sensor_file_locks:
                with self.sensor_file_locks[data_type]:
                    if data_type in self.sensor_temp_files:
                        self.sensor_temp_files[data_type].close()
            else:
                if data_type in self.sensor_temp_files:
                    self.sensor_temp_files[data_type].close()

            # Reopen the file for reading
            temp_path = self.sensor_temp_paths.get(data_type)
            if not temp_path:
                print(f"⚠️  Warning: the temporary file path does not exist for {data_type}")
                continue
            if not os.path.exists(temp_path):
                print(f"⚠️  Warning: the joint state temporary file does not exist {data_type}: {temp_path}")
                continue

            data_list = []
            try:
                with open(temp_path, 'rb') as f:
                    while True:
                        try:
                            data = pickle.load(f)
                            data_list.append(data)
                        except EOFError:
                            break  # File end
                        except Exception as e:
                            print(f"Error reading {data_type} data: {e}")
                            break

                if data_list:
                    sensor_data[data_type] = data_list
                    print(f"  {data_type}: collected {len(data_list)} data")
                else:
                    print(f"  ⚠️  Warning: the temporary file exists for {data_type} but the data is empty")

            except Exception as e:
                print(f"Error reading {data_type} temporary file: {e}")
                import traceback
                traceback.print_exc()
        
        # Read the action data of each part
        for data_type in action_names:
            if not hasattr(self, 'sensor_temp_files') or data_type not in self.sensor_temp_files:
                continue

            # Close the write handle
            if data_type in self.sensor_file_locks:
                with self.sensor_file_locks[data_type]:
                    if data_type in self.sensor_temp_files:
                        self.sensor_temp_files[data_type].close()
            else:
                if data_type in self.sensor_temp_files:
                    self.sensor_temp_files[data_type].close()

            # Reopen the file for reading
            temp_path = self.sensor_temp_paths[data_type]
            if not os.path.exists(temp_path):
                print(f"⚠️  Warning: the action data temporary file does not exist {data_type}: {temp_path}")
                continue

            data_list = []
            try:
                with open(temp_path, 'rb') as f:
                    while True:
                        try:
                            data = pickle.load(f)
                            data_list.append(data)
                        except EOFError:
                            break  # File end
                        except Exception as e:
                            print(f"Error reading {data_type} data: {e}")
                            break

                if data_list:
                    sensor_data[data_type] = data_list
                    print(f"  {data_type}: collected {len(data_list)} data")
                else:
                    # Skip the warning for action data (action data is not required)
                    if 'action' not in data_type.lower():
                        print(f"  ⚠️  {data_type}: the temporary file exists but the data is empty")

            except Exception as e:
                print(f"Error reading {data_type} temporary file: {e}")
                import traceback
                traceback.print_exc()
        
        # use video storage to read image data
        if self.use_video_storage:
            # Video mode: only read the timestamp information, not load the image data
            images = defaultdict(list)
            print("Reading image timestamp from temporary files...")
            
            for camera_name in self.camera_names:
                if camera_name not in self.image_temp_files:
                    continue
                
                # Close the write handle
                with self.image_file_locks[camera_name]:
                    self.image_temp_files[camera_name].close()
                
                # Reopen the file for reading, only read the timestamp
                temp_path = self.image_temp_paths[camera_name]
                if not os.path.exists(temp_path):
                    print(f"⚠️  Warning: the temporary file does not exist {camera_name}: {temp_path}")
                    continue
                
                try:
                    with open(temp_path, 'rb') as f:
                        while True:
                            try:
                                timestamp, img_data = pickle.load(f)
                                # Only save the timestamp, not save the image data to save memory
                                # img_data is now compressed JPEG bytes, no need to load
                                images[camera_name].append((timestamp, None))
                            except EOFError:
                                break
                    
                    print(f"  {camera_name}: read {len(images[camera_name])} timestamp frames")
                
                except Exception as e:
                    print(f"Error reading temporary file ({camera_name}): {e}")
        else:
            # Image mode: load all images normally
            images = defaultdict(list)
            print("Reading image data from temporary files...")
            
            for camera_name in self.camera_names:
                if camera_name not in self.image_temp_files:
                    continue
                
                # Close the write handle
                with self.image_file_locks[camera_name]:
                    self.image_temp_files[camera_name].close()
                
                # Reopen the file for reading
                temp_path = self.image_temp_paths[camera_name]
                if not os.path.exists(temp_path):
                    print(f"⚠️  Warning: the temporary file does not exist {camera_name}: {temp_path}")
                    continue
                
                try:
                    with open(temp_path, 'rb') as f:
                        while True:
                            try:
                                timestamp, img_data = pickle.load(f)
                                # img_data is compressed JPEG bytes, need to convert back to PIL Image
                                if isinstance(img_data, bytes):
                                    # Compressed JPEG data, need to decode
                                    img = Image.open(io.BytesIO(img_data))
                                    if img.mode != 'RGB':
                                        img = img.convert('RGB')
                                else:
                                    # Compatible with old format: if it is already a PIL Image object (backward compatible)
                                    img = img_data
                                images[camera_name].append((timestamp, img))
                            except EOFError:
                                break
                    
                    print(f"  {camera_name}: read {len(images[camera_name])} frames")
                
                except Exception as e:
                    print(f"Error reading temporary file ({camera_name}): {e}")
        
        # Validate before data alignment: check if all enabled data items have actual data
        # This can avoid the error of missing data during alignment
        validation_result = self._validate_collected_data(sensor_data, images)
        if not validation_result:
            print("Error: data collection validation failed, some enabled data items have no actual data")
            return None
        
        # Align data by timestamp interpolation
        print("Aligning data by timestamp interpolation...")
        episode_data = self._align_data_by_timestamp(sensor_data, images)
        
        if episode_data is None:
            print("⚠️  Warning: data alignment failed, return None")
            return None
        
        # Note: when using video storage, do not clean up temporary files here, clean up after saving the video
        if not self.use_video_storage:
            self._cleanup_temp_files()
        
        return episode_data
    
    def _align_data_by_timestamp(self, sensor_data, images):
        """Align all data to a uniform time grid based on timestamp"""

        # Camera name mapping: internal stream name -> user friendly name
        camera_name_mapping = {
            'head_rgb_stream': 'head_camera',
            'head_depth_stream': 'head_depth_camera',
            'left_arm_rgb_stream': 'left_arm_camera',
            'right_arm_rgb_stream': 'right_arm_camera'
        }

        # 1. Align image data frames based on the timestamp of the main camera (head camera)
        # First find the internal key name of the main camera
        head_camera_internal_name = None
        for internal_name, friendly_name in camera_name_mapping.items():
            if friendly_name == 'head_camera' and internal_name in images and images[internal_name]:
                head_camera_internal_name = internal_name
                break

        if not head_camera_internal_name:
            print("⚠️  Warning: the main camera (head_camera) has no data, cannot align the data")
            print(f"   Available image streams: {list(images.keys())}")
            return None

        # Use the timestamp of the main camera as the baseline time line
        target_timestamps = [t for t, _ in images[head_camera_internal_name]]
        num_frames = len(target_timestamps)

        print(f"   Use the timestamp of the main camera ({head_camera_internal_name}) as the baseline: {num_frames} frames")

        # Count the data of each sensor
        sensor_stats = [(name, len(data_list)) for name, data_list in sensor_data.items() if data_list]
        hf_stats = [(name, len(data_list)) for name, data_list in sensor_data.items() if data_list and name in ['joint_states', 'actions']]
        image_stats = [(cam, len(imgs)) for cam, imgs in images.items() if imgs]
        print(f"   Sensor data: {sensor_stats}")
        print(f"   High frequency data: {hf_stats}")
        print(f"   Image data: {image_stats}")

        # 2. Align the data of each sensor by timestamp
        aligned_sensor_data = {}
        aligned_action_data = {}

        # Use the joint state and action names in the member variable
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        action_names = self.slave_action_names if self.slave_action_names else []
        
        for sensor_name, data_list in sensor_data.items():
            if data_list:
                # When downsampling, the joint state and end pose need linear interpolation
                if sensor_name in joint_state_names + action_names and self.downsample_joint_states:
                    print(f"   Use linear interpolation to align the data of {sensor_name}")
                    if sensor_name in action_names:
                        aligned_action_data[sensor_name] = self._interpolate_linear(data_list, target_timestamps)
                    else:
                        aligned_sensor_data[sensor_name] = self._interpolate_linear(data_list, target_timestamps)
                else:
                    # Other sensor data use nearest neighbor interpolation
                    if sensor_name in action_names:
                        aligned_action_data[sensor_name] = self._interpolate_nearest(data_list, target_timestamps)
                    else:
                        aligned_sensor_data[sensor_name] = self._interpolate_nearest(data_list, target_timestamps)

        # Align the image data by timestamp (use nearest neighbor interpolation, because the image data is already the baseline)
        aligned_images = {}
        image_index_mapping = {}  # Used for frame index mapping in video mode
        
        if self.use_video_storage:
            # Video mode: only create the mapping from timestamp to frame index, not load the image
            for cam_name, cam_imgs in images.items():
                friendly_name = camera_name_mapping.get(cam_name, cam_name)
                if cam_name == head_camera_internal_name:
                    # Main camera: create 1:1 mapping
                    image_index_mapping[friendly_name] = list(range(len(cam_imgs)))
                    aligned_images[friendly_name] = []  # Empty list
                else:
                    # Other cameras: create the mapping from timestamp to frame index
                    cam_timestamps = [t for t, _ in cam_imgs]
                    indices = self._interpolate_nearest_indices(cam_timestamps, target_timestamps)
                    image_index_mapping[friendly_name] = indices
                    aligned_images[friendly_name] = []  # Empty list
        else:
            # Image mode: align the image data normally
            for cam_name, cam_imgs in images.items():
                friendly_name = camera_name_mapping.get(cam_name, cam_name)
                if cam_name == head_camera_internal_name:
                    # Main camera: use directly, no interpolation
                    aligned_images[friendly_name] = [img for _, img in cam_imgs]
                else:
                    # Other cameras use nearest neighbor interpolation to align to the timestamp of the main camera
                    aligned_images[friendly_name] = self._interpolate_nearest(cam_imgs, target_timestamps)

        print(f"   Aligned frames: {len(target_timestamps)}")

        # Ensure joint_names is not None - infer the joint names from the joint state data of each part
        joint_names = self.joint_names
        if not joint_names or not isinstance(joint_names, dict):
            # Use the saved real joint names first
            if hasattr(self, '_joint_names_by_part') and self._joint_names_by_part:
                joint_names = {}
                for joint_state_name, names in self._joint_names_by_part.items():
                    part_name = joint_state_name.replace('_joint_states', '')
                    joint_names[part_name] = names
                print(f"✓ Use the saved joint names: {len(joint_names)} parts")
                for part_name, names in joint_names.items():
                    print(f"    - {part_name}: {names}")
            else:
                # Infer the joint names from the joint state data of each part (separated by parts)
                joint_names = {}
                for joint_state_name in joint_state_names:
                    if joint_state_name in aligned_sensor_data and len(aligned_sensor_data[joint_state_name]) > 0:
                        # Infer the number of joints from the first frame data
                        first_state = aligned_sensor_data[joint_state_name][0]
                        if isinstance(first_state, tuple) and len(first_state) > 0:
                            num_joints = len(first_state[0])
                        else:
                            num_joints = len(first_state) if hasattr(first_state, '__len__') else 0
                        
                        # Generate default joint names for each part
                        part_name = joint_state_name.replace('_joint_states', '')
                        part_joint_names = []
                        for i in range(num_joints):
                            part_joint_names.append(f"{part_name}_joint{i+1}")
                        joint_names[part_name] = part_joint_names
                
                if joint_names:
                    total_joints = sum(len(names) for names in joint_names.values())
                    print(f"⚠️  joint_names is empty, infer the default names from the data: {len(joint_names)} parts, total {total_joints} joints")
                    for part_name, names in joint_names.items():
                        print(f"    - {part_name}: {len(names)} joints")

        episode_data = {
            'timestamps': target_timestamps,
            'sensor_data': aligned_sensor_data,  # Normal sensor data
            'action_data': aligned_action_data,  # Action data
            'images': aligned_images,  # Image data
            'joint_names': joint_names,
        }
        
        # Video mode: add the image index mapping and the original camera name mapping
        if self.use_video_storage:
            episode_data['image_index_mapping'] = image_index_mapping
            episode_data['camera_name_mapping'] = camera_name_mapping

        print("✓ Timestamp alignment completed")
        return episode_data
    
    def _interpolate_linear(self, data_with_timestamps, target_timestamps):
        """Use linear interpolation to align the data to the target timestamps"""
        import numpy as np

        if not data_with_timestamps:
            return []

        sorted_data = sorted(data_with_timestamps, key=lambda x: x[0])
        aligned_data = []

        # Check the data format: the joint state is a 4 element tuple, other is a 2 element tuple
        first_item = sorted_data[0]
        is_joint_state = len(first_item) == 4  # The joint state format: (timestamp, positions, velocities, efforts)

        # Extract the timestamp and data
        timestamps = np.array([item[0] for item in sorted_data])

        if is_joint_state:
            # The joint state data: positions, velocities, efforts
            positions = np.array([item[1] for item in sorted_data])
            velocities = np.array([item[2] for item in sorted_data])
            efforts = np.array([item[3] for item in sorted_data])
        else:
            # Other data format
            data_values = np.array([item[1] for item in sorted_data])

        for target_t in target_timestamps:
            if target_t <= timestamps[0]:
                # Use the first data point
                if is_joint_state:
                    aligned_data.append((sorted_data[0][1], sorted_data[0][2], sorted_data[0][3]))
                else:
                    aligned_data.append(sorted_data[0][1])
            elif target_t >= timestamps[-1]:
                # Use the last data point
                if is_joint_state:
                    aligned_data.append((sorted_data[-1][1], sorted_data[-1][2], sorted_data[-1][3]))
                else:
                    aligned_data.append(sorted_data[-1][1])
            else:
                # Linear interpolation
                # Find the interval where target_t is located
                idx = np.searchsorted(timestamps, target_t) - 1
                if idx < 0:
                    idx = 0

                t1, t2 = timestamps[idx], timestamps[idx + 1]
                ratio = (target_t - t1) / (t2 - t1) if t2 != t1 else 0

                if is_joint_state:
                    # Linear interpolation for positions, velocities, efforts separately
                    pos1, pos2 = positions[idx], positions[idx + 1]
                    vel1, vel2 = velocities[idx], velocities[idx + 1]
                    eff1, eff2 = efforts[idx], efforts[idx + 1]

                    interpolated_pos = pos1 + ratio * (pos2 - pos1)
                    interpolated_vel = vel1 + ratio * (vel2 - vel1)
                    interpolated_eff = eff1 + ratio * (eff2 - eff1)

                    aligned_data.append((interpolated_pos, interpolated_vel, interpolated_eff))
                else:
                    # Linear interpolation for single value or array
                    val1, val2 = data_values[idx], data_values[idx + 1]
                    interpolated_val = val1 + ratio * (val2 - val1)
                    aligned_data.append(interpolated_val)

        return aligned_data

    def _interpolate_nearest(self, data_with_timestamps, target_timestamps):
        """Use nearest neighbor interpolation to align the data to the target timestamps"""
        if not data_with_timestamps:
            return []

        sorted_data = sorted(data_with_timestamps, key=lambda x: x[0])
        aligned_data = []
        data_idx = 0

        for target_t in target_timestamps:
            min_diff = float('inf')
            # Check the data format: the joint state is a 4 element tuple, other is a 2 element tuple
            first_item = sorted_data[0]
            if len(first_item) == 4:  # The joint state format: (timestamp, positions, velocities, efforts)
                closest_data = (first_item[1], first_item[2], first_item[3])  # positions, velocities, efforts
            else:  # Other format: (timestamp, data)
                closest_data = first_item[1]

            for i in range(data_idx, len(sorted_data)):
                item = sorted_data[i]
                t = item[0]
                diff = abs(t - target_t)

                if diff < min_diff:
                    min_diff = diff
                    if len(item) == 4:  # The joint state format
                        closest_data = (item[1], item[2], item[3])  # positions, velocities, efforts
                    else:  # Other format
                        closest_data = item[1]
                    data_idx = i
                else:
                    break

            aligned_data.append(closest_data)

        return aligned_data
    
    def _interpolate_nearest_indices(self, timestamps, target_timestamps):
        """Use nearest neighbor interpolation to return the index mapping (for video mode)"""
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
        """Use ffmpeg to create a video (streaming processing, saving memory)
        
        Args:
            temp_path: temporary image file path
            indices: index mapping list, specify which original image should be used for each frame
            output_path: output video path
            fps: frame rate
            expected_frames: expected number of frames
            
        Returns:
            True if successful, False if failed
        """
        if not indices or not os.path.exists(temp_path):
            print(f"  ⚠️  Warning: no image data or temporary file does not exist, cannot create video")
            return False
        
        try:
            # Build ffmpeg command
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
            
            # Start ffmpeg process
            process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # First load all original images into memory (to avoid repeated reading of files)
            # But this is separated by cameras, so the memory pressure is much smaller
            all_images = []
            with open(temp_path, 'rb') as f:
                while True:
                    try:
                        _, img_data = pickle.load(f)
                        # img_data may be compressed JPEG bytes or PIL Image object (backward compatibility)
                        if isinstance(img_data, bytes):
                            # Compressed JPEG data, need to decode to PIL Image
                            img = Image.open(io.BytesIO(img_data))
                            if img.mode != 'RGB':
                                img = img.convert('RGB')
                        elif isinstance(img_data, Image.Image):
                            # Already PIL Image object (backward compatibility for old format)
                            img = img_data
                            if img.mode != 'RGB':
                                img = img.convert('RGB')
                        else:
                            # Other format (e.g. numpy array)
                            img = img_data
                        all_images.append(img)
                    except EOFError:
                        break
            
            print(f"    Loaded {len(all_images)} original images, need to align to {expected_frames} frames")
            
            # Get the size of the first frame (for creating black frame)
            first_img = all_images[0] if all_images else None
            if first_img is None:
                print(f"  ⚠️  Warning: no valid first frame")
                process.stdin.close()
                process.kill()
                return False
            
            if isinstance(first_img, Image.Image):
                width, height = first_img.size
            elif isinstance(first_img, np.ndarray):
                height, width = first_img.shape[:2]
            else:
                print(f"  ⚠️  Warning: cannot determine the image size (type: {type(first_img)})")
                process.stdin.close()
                process.kill()
                return False
            
            # Create black frame (for error frame)
            black_frame = Image.new('RGB', (width, height), (0, 0, 0))
            
            # Streaming processing: read, convert and write frame by frame
            frames_written = 0
            for frame_idx, img_idx in enumerate(indices):
                try:
                    pil_img = None
                    
                    # Get the corresponding original image
                    if img_idx >= len(all_images):
                        print(f"  ⚠️  Warning: index out of range {img_idx}/{len(all_images)}, using black frame")
                        pil_img = black_frame
                    else:
                        img = all_images[img_idx]
                        
                        if img is None:
                            print(f"  ⚠️  Warning: frame {frame_idx} image is empty, using black frame")
                            pil_img = black_frame
                        else:
                            # Convert to PIL Image
                            # img should already be PIL Image (processed when loaded)
                            if isinstance(img, Image.Image):
                                pil_img = img
                            elif isinstance(img, np.ndarray):
                                pil_img = Image.fromarray(img)
                            elif isinstance(img, bytes):
                                # If it is bytes (should not happen, because it is processed when loaded), try to decode
                                pil_img = Image.open(io.BytesIO(img))
                                if pil_img.mode != 'RGB':
                                    pil_img = pil_img.convert('RGB')
                            else:
                                print(f"  ⚠️  Warning: frame {frame_idx} format not supported: {type(img)}, using black frame")
                                pil_img = black_frame
                    
                    # Ensure it is in RGB mode
                    if pil_img and pil_img.mode != 'RGB':
                        pil_img = pil_img.convert('RGB')
                    
                    # Save as JPEG and write to pipe
                    if pil_img:
                        buffer = io.BytesIO()
                        pil_img.save(buffer, format='JPEG', quality=95)
                        process.stdin.write(buffer.getvalue())
                        frames_written += 1
                        del buffer
                    
                    # Periodically release the processed images (if not needed anymore)
                    if frame_idx % 100 == 99:
                        # Find the indices of the images that will not be used in the subsequent frames
                        remaining_indices = set(indices[frame_idx+1:])
                        for i in range(len(all_images)):
                            if i not in remaining_indices and i <= img_idx:
                                all_images[i] = None
                        
                        import gc
                        gc.collect()
                    
                except Exception as e:
                    print(f"  ⚠️  Error: failed to process frame {frame_idx}: {e}")
                    # Even if it fails, write the black frame to ensure the number of frames is consistent
                    try:
                        buffer = io.BytesIO()
                        black_frame.save(buffer, format='JPEG', quality=95)
                        process.stdin.write(buffer.getvalue())
                        frames_written += 1
                        del buffer
                    except:
                        pass
            
            print(f"    Successfully written {frames_written}/{expected_frames} frames")
            
            # Clean up
            del all_images
            
            # Close input and wait for completion
            try:
                # flush and close stdin
                if process.stdin and not process.stdin.closed:
                    process.stdin.flush()
                    process.stdin.close()
                
                # Use wait() instead of communicate(), because we have written the data
                # communicate() will try to flush the closed stdin, causing an error
                returncode = process.wait(timeout=60)
                
                # Read stderr to get error information
                if process.stderr:
                    stderr = process.stderr.read()
                else:
                    stderr = b''
                    
            except subprocess.TimeoutExpired:
                print(f"  ⚠️  ffmpeg processing timeout")
                try:
                    process.kill()
                    process.wait()
                except:
                    pass
                return False
            except Exception as e:
                print(f"  ⚠️  Processing error: {e}")
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
                print(f"  ⚠️  ffmpeg error (return code: {returncode}): {error_msg[:200]}")
                return False
                
        except FileNotFoundError:
            print(f"  ⚠️  ffmpeg not installed, please install: sudo apt-get install ffmpeg")
            return False
        except Exception as e:
            print(f"  ⚠️  Failed to create video: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _save_episode(self, episode_data: Dict[str, Any], task: str) -> Dict[str, Any]:
        """Save episode data to JSON and image or video file"""
        episode_id = self.episode_count
        episode_dir = self.output_dir / f"episode_{episode_id:04d}"
        episode_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"Saving Episode {episode_id} to {episode_dir}...")
        
        timestamps = episode_data['timestamps']
        sensor_data = episode_data['sensor_data']
        action_data = episode_data.get('action_data', {})
        images = episode_data['images']
        
        # Use the joint state and action names in the member variables
        joint_state_names = self.slave_joint_names if self.slave_joint_names else []
        action_names = self.slave_action_names if self.slave_action_names else []
        
        # Collect the joint state data for each part
        joint_states_by_part = {}
        for joint_state_name in joint_state_names:
            if joint_state_name in sensor_data:
                joint_states_by_part[joint_state_name] = sensor_data[joint_state_name]
        
        # Collect the action data for each part (for future state as action)
        actions_by_part = {}
        for action_name in action_names:
            if action_name in action_data:
                actions_by_part[action_name] = action_data[action_name]
        
        # Build the joint name dictionary (separated by parts, using the real joint names)
        joint_names = {}
        for joint_state_name in joint_state_names:
            if joint_state_name in joint_states_by_part and len(joint_states_by_part[joint_state_name]) > 0:
                part_name = joint_state_name.replace('_joint_states', '')
                
                # Use the saved real joint names first
                if hasattr(self, '_joint_names_by_part') and joint_state_name in self._joint_names_by_part:
                    joint_names[part_name] = self._joint_names_by_part[joint_state_name]
                else:
                    # If there are no saved joint names, infer the number of joints from the first frame data and generate default names
                    first_state = joint_states_by_part[joint_state_name][0]
                    if isinstance(first_state, tuple) and len(first_state) > 0:
                        num_joints = len(first_state[0])
                    else:
                        num_joints = len(first_state) if hasattr(first_state, '__len__') else 0
                    
                    # Generate default joint names for each part
                    part_joint_names = []
                    for i in range(num_joints):
                        part_joint_names.append(f"{part_name}_joint{i+1}")
                    joint_names[part_name] = part_joint_names
        
        if not joint_names:
            joint_names = {}
        
        num_frames = len(timestamps)
        
        # Get the robot model and map it to the product name
        robot_model = self.robot.get_robot_model()
        # Build the episode JSON data
        episode_json = {
            "episode_id": episode_id,
            "model": robot_model,
            "task": task,
            "timestamp": datetime.now().isoformat(),
            "duration": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0,
            "num_frames": num_frames,
            "joint_names": joint_names if joint_names else {},  # Dictionary separated by parts
            "storage_format": "video" if self.use_video_storage else "images",
            "frames": []
        }
        
        # If using video storage, use ffmpeg to create video
        video_files = {}
        
        if self.use_video_storage:
            # Get the index mapping and camera name mapping from episode_data
            image_index_mapping = episode_data.get('image_index_mapping', {})
            camera_name_mapping = episode_data.get('camera_name_mapping', {})
            
            # Reverse mapping: friendly name -> internal name
            reverse_mapping = {v: k for k, v in camera_name_mapping.items()}
            
            # Use ffmpeg to create video (streaming processing, saving memory)
            print(f"  Starting to create video files...")
            for friendly_name in image_index_mapping.keys():
                internal_name = reverse_mapping.get(friendly_name, friendly_name)
                if internal_name not in self.image_temp_paths:
                    print(f"  ⚠️  Warning: temporary file not found for {friendly_name}")
                    continue
                    
                temp_path = self.image_temp_paths[internal_name]
                if not os.path.exists(temp_path):
                    print(f"  ⚠️  Warning: temporary file does not exist: {temp_path}")
                    continue
                
                # Get the index mapping
                indices = image_index_mapping[friendly_name]
                
                # Use ffmpeg to create video
                video_path = episode_dir / f"{friendly_name}.mp4"
                print(f"  Using ffmpeg to create video: {friendly_name}.mp4...")
                
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
                        print(f"  ✓ Video created successfully: {friendly_name}.mp4 ({num_frames} frames)")
                    else:
                        print(f"  ⚠️  Video creation failed: {friendly_name}")
                    
                except Exception as e:
                    print(f"  ⚠️  Warning: error occurred when creating video for {friendly_name}: {e}")
                    import traceback
                    traceback.print_exc()
            
            # Update episode_json
            if video_files:
                episode_json["video_files"] = video_files
        
        # Save each frame
        import gc
        for i in range(num_frames):
            frame_data = {
                "frame_id": i,
                "timestamp": timestamps[i],
                "images": {}
            }

            # Add the joint state data for each part
            if "observation" not in frame_data:
                frame_data["observation"] = {}
            
            for joint_state_name in joint_state_names:
                if joint_state_name in joint_states_by_part and i < len(joint_states_by_part[joint_state_name]):
                    # Extract the state data (positions, velocities, efforts)
                    state_data = joint_states_by_part[joint_state_name][i]
                    if isinstance(state_data, tuple):
                        positions, velocities, efforts = state_data
                    else:
                        positions = state_data
                        velocities = None
                        efforts = None
                    
                    # Convert to list format
                    positions_list = positions.tolist() if isinstance(positions, np.ndarray) else list(positions)
                    
                    # Save the joint state for each part
                    frame_data["observation"][joint_state_name] = {
                        "positions": positions_list
                    }
                    
                    # Add optional velocity and effort
                    if velocities is not None:
                        frame_data["observation"][joint_state_name]["velocities"] = velocities.tolist() if isinstance(velocities, np.ndarray) else list(velocities)
                    if efforts is not None:
                        frame_data["observation"][joint_state_name]["efforts"] = efforts.tolist() if isinstance(efforts, np.ndarray) else list(efforts)
            
            # Add action data (using the state of the next frame as action)
            if "action" not in frame_data:
                frame_data["action"] = {}
            
            for joint_state_name in joint_state_names:
                part_name = joint_state_name.replace('_joint_states', '')
                action_name = f"{part_name}_actions"
                
                # Use the state of the next frame as action
                if joint_state_name in joint_states_by_part:
                    if i + 1 < len(joint_states_by_part[joint_state_name]):
                        # There is a next frame, use the state of the next frame as action
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
                        # The last frame, use the state of the current frame as action
                        current_state_data = joint_states_by_part[joint_state_name][i]
                        if isinstance(current_state_data, tuple):
                            current_positions, _, _ = current_state_data
                        else:
                            current_positions = current_state_data
                        
                        current_positions_list = current_positions.tolist() if isinstance(current_positions, np.ndarray) else list(current_positions)
                        frame_data["action"][action_name] = {
                            "positions": current_positions_list
                        }

            # Add other sensor data
            for sensor_name, sensor_data_list in sensor_data.items():
                # Skip the joint state data that has already been processed
                if sensor_name in joint_state_names:
                    continue
                
                # Skip the empty data list
                if not sensor_data_list:
                    continue

                if i < len(sensor_data_list):
                    sensor_value = sensor_data_list[i]

                    # If there is no observation dictionary, create one
                    if "observation" not in frame_data:
                        frame_data["observation"] = {}

                    # Process the data based on the sensor type
                    if sensor_name.endswith('_end_pose'):
                        # End pose data
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
                            # If the data format is different, save it directly
                            frame_data["observation"][sensor_name] = sensor_value
                        
                        # Add the action of end_pose (using the data of the next frame)
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
                                # If there is no next frame or the format is different, use the current frame as action
                                action_name = sensor_name.replace('_end_pose', '_end_pose_action')
                                frame_data["action"][action_name] = frame_data["observation"][sensor_name]
                        else:
                            # The last frame, use the current frame as action
                            action_name = sensor_name.replace('_end_pose', '_end_pose_action')
                            frame_data["action"][action_name] = frame_data["observation"][sensor_name]

                    elif sensor_name == 'odometry':
                        # Odometry data
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
                        
                        # Add the action of odometry (using the data of the next frame)
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
                                # If there is no next frame or the format is different, use the current frame as action
                                frame_data["action"]["odometry_action"] = frame_data["observation"][sensor_name]
                        else:
                            # The last frame, use the current frame as action
                            frame_data["action"]["odometry_action"] = frame_data["observation"][sensor_name]

                    elif sensor_name.endswith('_wrench_ext_world') or sensor_name.endswith('_wrench_ext_local'):
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
                        # IMU data
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
                        # Pose data
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
                    
                    elif sensor_name.endswith('_gripper_position'):
                        # Gripper position data
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            # Try to convert the message object to a dictionary
                            data_obj = sensor_value['data']
                            # Use the recursive method to convert
                            frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(data_obj)
                            
                            # Add the action of gripper_position (using the data of the next frame)
                            if i + 1 < len(sensor_data_list):
                                next_sensor_value = sensor_data_list[i + 1]
                                if isinstance(next_sensor_value, dict) and 'data' in next_sensor_value:
                                    next_data_obj = next_sensor_value['data']
                                    action_name = sensor_name.replace('_gripper_position', '_gripper_position_action')
                                    frame_data["action"][action_name] = self._convert_ros_msg_to_dict(next_data_obj)
                                else:
                                    # If there is no next frame or the format is different, use the current frame as action
                                    action_name = sensor_name.replace('_gripper_position', '_gripper_position_action')
                                    frame_data["action"][action_name] = frame_data["observation"][sensor_name]
                            else:
                                # The last frame, use the current frame as action
                                action_name = sensor_name.replace('_gripper_position', '_gripper_position_action')
                                frame_data["action"][action_name] = frame_data["observation"][sensor_name]
                        else:
                            # If the data format is different, save it directly
                            frame_data["observation"][sensor_name] = sensor_value
                            # Add the action (using the data of the next frame)
                            if i + 1 < len(sensor_data_list):
                                action_name = sensor_name.replace('_gripper_position', '_gripper_position_action')
                                frame_data["action"][action_name] = sensor_data_list[i + 1]
                            else:
                                # The last frame, use the current frame as action
                                action_name = sensor_name.replace('_gripper_position', '_gripper_position_action')
                                frame_data["action"][action_name] = sensor_value
                    
                    else:
                        # Other sensor data
                        if isinstance(sensor_value, dict) and 'data' in sensor_value:
                            # Try to convert the message object to a dictionary
                            data_obj = sensor_value['data']
                            # Use the recursive method to convert
                            frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(data_obj)
                        elif isinstance(sensor_value, tuple):
                            # Process the joint state tuple format
                            # Interpolated format: (positions, velocities, efforts) - 3 elements
                            # Or the original format: (timestamp, positions, velocities, efforts) - 4 elements
                            # Or (timestamp, data) - 2 elements
                            if sensor_name.endswith('_joint_state') and len(sensor_value) == 3:
                                # Master joint state format (interpolated): (positions, velocities, efforts)
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
                                # Other tuple format, recursive conversion (will handle NumPy arrays)
                                frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(sensor_value)
                        else:
                            # Other format, use recursive conversion to ensure NumPy arrays are converted
                            frame_data["observation"][sensor_name] = self._convert_ros_msg_to_dict(sensor_value)
            
            # Save images
            if self.use_video_storage:
                # Video storage mode: video has already been created, only record the frame number to JSON
                for cam_name in video_files.keys():
                    frame_data["images"][cam_name] = i
            else:
                # Image storage mode: read from the images dictionary and save as files
                for cam_name, cam_images in images.items():
                    if i < len(cam_images):
                        img = cam_images[i]
                        # Image storage mode: save as separate files
                        img_filename = f"frame_{i:04d}_{cam_name}.jpg"
                        img_path = episode_dir / img_filename
                        
                        # Select the save format based on the image type
                        if 'depth' in cam_name:
                            # Depth image: use PNG format (supports floating point) or save as numpy array
                            if isinstance(img, Image.Image):
                                # Depth image is usually in floating point mode, needs special handling
                                if img.mode == 'F':
                                    # Convert the floating point depth image to a visual image and save
                                    # Normalize to 0-255 range for visualization
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
                                    # Depth data: save as numpy array file
                                    img_path = img_path.with_suffix('.npz')
                                    np.savez_compressed(img_path, depth=img)
                                    img_filename = f"frame_{i:04d}_{cam_name}.npz"
                                else:
                                    # Other numpy arrays: convert to image and save
                                    img_path = img_path.with_suffix('.png')
                                    img_pil = Image.fromarray(img.astype(np.uint8))
                                    img_pil.save(img_path, 'PNG')
                                    img_filename = f"frame_{i:04d}_{cam_name}.png"
                            elif isinstance(img, bytes):
                                # Original byte data: save as binary file
                                img_path = img_path.with_suffix('.bin')
                                with open(img_path, 'wb') as f:
                                    f.write(img)
                                img_filename = f"frame_{i:04d}_{cam_name}.bin"
                            else:
                                # Other format: try to save as pickle
                                img_path = img_path.with_suffix('.pkl')
                                with open(img_path, 'wb') as f:
                                    pickle.dump(img, f)
                                img_filename = f"frame_{i:04d}_{cam_name}.pkl"
                        else:
                            # Ordinary RGB image: use JPEG format
                            if isinstance(img, Image.Image):
                                img.save(img_path, 'JPEG', quality=self.image_quality)
                            else:
                                # If it is a numpy array, convert to PIL Image
                                if isinstance(img, np.ndarray):
                                    img = Image.fromarray(img)
                                    img.save(img_path, 'JPEG', quality=self.image_quality)
                        
                        frame_data["images"][cam_name] = str(img_filename)
            
            episode_json["frames"].append(frame_data)
        
        # Video storage mode: video has already been created through ffmpeg
        if self.use_video_storage:
            print(f"  ✓ Video files have been saved: {len(video_files)}")
            
            # Verify the video files
            for cam_name, video_file in video_files.items():
                video_path = episode_dir / video_file
                if video_path.exists():
                    file_size = video_path.stat().st_size
                    print(f"    {cam_name}: {file_size / 1024 / 1024:.2f} MB")
                    if file_size < 1024:  # Less than 1KB
                        print(f"    ⚠️  Warning: {cam_name} file is too small, may be corrupted")
            
            # Clean up the image data
            if 'aligned_video_images' in locals():
                aligned_video_images.clear()
                del aligned_video_images
            
            # Clean up the temporary files (ensure cleanup even if there is an error)
            try:
                self._cleanup_temp_files()
            except Exception as e:
                print(f"  ⚠️  Warning: error cleaning up temporary files: {e}")
            
            # Force garbage collection to release memory
            import gc
            gc.collect()
        
        # Save episode JSON file (using temporary file to ensure atomic write)
        episode_json_path = episode_dir / "episode.json"
        temp_json_path = episode_dir / "episode.json.tmp"
        
        try:
            # Write to temporary file first
            with open(temp_json_path, 'w', encoding='utf-8') as f:
                json.dump(episode_json, f, indent=2, ensure_ascii=False)
                # Ensure the file is fully written to disk
                f.flush()
                os.fsync(f.fileno())
            
            # Atomic rename (replace existing file)
            if episode_json_path.exists():
                episode_json_path.unlink()
            temp_json_path.rename(episode_json_path)
            
        except Exception as e:
            print(f"  ⚠️  Error saving JSON file: {e}")
            import traceback
            traceback.print_exc()
            # Try to save directly (if the temporary file method fails)
            try:
                with open(episode_json_path, 'w', encoding='utf-8') as f:
                    json.dump(episode_json, f, indent=2, ensure_ascii=False)
                    f.flush()
                    os.fsync(f.fileno())
            except Exception as e2:
                print(f"  ✗ Error saving JSON file: {e2}")
                import traceback
                traceback.print_exc()
                raise
            finally:
                # Clean up the temporary files (if they exist)
                if temp_json_path.exists():
                    try:
                        temp_json_path.unlink()
                    except:
                        pass
        
        # Update the dataset metadata
        self.dataset_metadata['episodes'].append({
            "episode_id": episode_id,
            "model": robot_model,
            "task": task,
            "timestamp": episode_json["timestamp"],
            "duration": episode_json["duration"],
            "num_frames": num_frames,
            "path": str(episode_dir.relative_to(self.output_dir))
        })
        self._save_metadata()
        
        print(f"✓ Episode {episode_id} has been saved")
        print(f"  - Number of frames: {num_frames}")
        print(f"  - Duration: {episode_json['duration']:.2f}s")
        print(f"  - JSON: {episode_json_path}")
        if self.use_video_storage:
            total_video_frames = num_frames * len(video_files)
            print(f"  - Video: {len(video_files)} MP4 files (each {num_frames} frames)")
        else:
            total_image_count = sum(len(imgs) for imgs in images.values())
            print(f"  - Images: {total_image_count} JPG files")
        
        return {
            "episode_id": episode_id,
            "episode_dir": str(episode_dir),
            "episode_json": str(episode_json_path),
            "num_frames": num_frames,
            "duration": episode_json["duration"]
        }
