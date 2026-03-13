"""
convert collected data to LeRobot dataset format

Support the latest data collection format, including:
- Joint states (joint_states)
- End pose
- Actions
- Multi-mode action data (action): support master_arm_control, vr_control, etc.
- Image data (images)
- Additional sensor data: odometry, chassis_imu, pose, tactile etc.

Dependencies:
    pip install lerobot torch numpy pillow

Usage:
    python convert_to_lerobot.py \
        --input-dir ./collected_data \
        --output-dir ./lerobot_data \
        --repo-id "my_robot/dataset" \
        --robot-type "quanta_x1"

Optional parameters:
    --fps: Target frame rate (default: 30)
    --resize-width: Image width (default: 640)
    --resize-height: Image height (default: 480)
    --delta-action: Use delta action (default: False)
    --relative-start: Relative to start point (default: False)
    --select-joints: Select specific joints (e.g.: "left_arm_joint1,left_arm_joint2,...")
    --episodes: Select specific episodes (e.g.: "0,1,2" or "0-5")
    --include-sensors: Include additional sensor data (e.g.: "odometry,chassis_imu,left_arm_end_pose")

Important notes:
    - Action data format: The new collection system supports multiple control modes, each mode produces a different action vector
      The script will automatically detect and combine all available control commands

    - task field: In LeRobot, task is a special field that should not be defined in the features dictionary.
      Each frame must contain the task field (string type), but it does not need to be defined in features.
      LeRobot will automatically process the task when adding a frame, and store it in the episode_buffer.

    - Sensor data: New sensor data will be saved as additional observation fields
      For example: observation.odometry, observation.left_arm_end_pose etc.
"""

import argparse
import json
from pathlib import Path
from typing import List, Optional, Dict, Any
import numpy as np
import torch
from PIL import Image
import cv2
import math
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# Data configuration for different machine types (contains action and state extraction configuration)
# Note: state_order and action_order should be consistent, ensure the dimension matching of state and action
# If you need to support other machine types, you need to:
# 1. Add the corresponding machine type configuration in this dictionary
# 2. Ensure the order and dimension consistency of state_order and action_order
ROBOT_DATA_CONFIG = {
    "quanta_x1": {
        "action_order": [
            "master_left_ee_cartesian_pos",  # 3 dimensions - Master left arm end effector position
            "master_left_ee_rotation",        # 3 dimensions - Master left arm end effector rotation angle (quaternion to euler angle)
            "follow_left_gripper",            # 1 dimension - Follow left gripper position
            "master_right_ee_cartesian_pos",  # 3 dimensions - Master right arm end effector position
            "master_right_ee_rotation",       # 3 dimensions - Master right arm end effector rotation angle (quaternion to euler angle)
            "follow_right_gripper",          # 1 dimension - Follow right gripper position
            "head_rotation",                  # 2 dimensions - Head motor angle (pitch, yaw)
            "height",                         # 1 dimension - Lift height
            "velocity_decomposed_odom",       # 3 dimensions - Odometry information (x, y, yaw)
        ],
        "state_order": [
            "master_left_ee_cartesian_pos",  # 3 dimensions - Master left arm end effector position (from master_left_arm_end_pose)
            "master_left_ee_rotation",        # 3 dimensions - Master left arm end effector rotation angle (from master_left_arm_end_pose)
            "follow_left_gripper",            # 1 dimension - Follow left gripper position (from left_gripper_joint_states)
            "master_right_ee_cartesian_pos",  # 3 dimensions - Master right arm end effector position (from master_right_arm_end_pose)
            "master_right_ee_rotation",       # 3 dimensions - Master right arm end effector rotation angle (from master_right_arm_end_pose)
            "follow_right_gripper",          # 1 dimension - Follow right gripper position (from right_gripper_joint_states)
            "head_rotation",                  # 2 dimensions - Head motor angle (from head_joint_states)
            "height",                         # 1 dimension - Lift height (from lift_joint_states)
            "velocity_decomposed_odom",       # 3 dimensions - Odometry information (from odometry x, y, yaw)
        ],
        # Define the data source mapping for each state field
        "state_source_mapping": {
            "master_left_ee_cartesian_pos": {
                "source": "master_left_arm_end_pose",
                "type": "position",  # position, orientation, joint_positions
                "dim": 3
            },
            "master_left_ee_rotation": {
                "source": "master_left_arm_end_pose",
                "type": "orientation_euler",  # Quaternion to Euler angle
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0  # Get the first joint
            },
            "master_right_ee_cartesian_pos": {
                "source": "master_right_arm_end_pose",
                "type": "position",
                "dim": 3
            },
            "master_right_ee_rotation": {
                "source": "master_right_arm_end_pose",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_right_gripper": {
                "source": "right_gripper_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "head_rotation": {
                "source": "head_joint_states",
                "type": "joint_positions",
                "dim": 2,
                "index": [0, 1]  # Get the first two joints
            },
            "height": {
                "source": "lift_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "velocity_decomposed_odom": {
                "source": "odometry",
                "type": "odom_xy_yaw",  # Extract x, y, yaw
                "dim": 3
            }
        },
        # Define the data source mapping for each action field
        "action_source_mapping": {
            "master_left_ee_cartesian_pos": {
                "source": "master_left_arm_end_pose_action",
                "type": "position",  # position, orientation_euler, joint_positions, odom_xy_yaw
                "dim": 3
            },
            "master_left_ee_rotation": {
                "source": "master_left_arm_end_pose_action",
                "type": "orientation_euler",  # Quaternion to Euler angle
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0  # Get the first joint
            },
            "master_right_ee_cartesian_pos": {
                "source": "master_right_arm_end_pose_action",
                "type": "position",
                "dim": 3
            },
            "master_right_ee_rotation": {
                "source": "master_right_arm_end_pose_action",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_right_gripper": {
                "source": "right_gripper_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "head_rotation": {
                "source": "head_actions",
                "type": "joint_positions",
                "dim": 2,
                "index": [0, 1]  # Get the first two joints
            },
            "height": {
                "source": "lift_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "velocity_decomposed_odom": {
                "source": "odometry_action",
                "type": "odom_xy_yaw",  # Extract x, y, yaw
                "dim": 3
            }
        }
    },
    "quanta_x2": {
        # quanta_x2 无 master 数据，使用 left/right 命名
        "action_order": [
            "left_ee_cartesian_pos",   # 3 dimensions - Left arm end effector position
            "left_ee_rotation",        # 3 dimensions - Left arm end effector rotation (quaternion to euler)
            "follow_left_gripper",    # 1 dimension - Left gripper position
            "right_ee_cartesian_pos",  # 3 dimensions - Right arm end effector position
            "right_ee_rotation",      # 3 dimensions - Right arm end effector rotation (quaternion to euler)
            "follow_right_gripper",  # 1 dimension - Right gripper position
            "head_rotation",          # 2 dimensions - Head motor angle (pitch, yaw)
            "waist",                  # 4 dimensions - Waist joint positions (no lift)
            "velocity_decomposed_odom",  # 3 dimensions - Odometry (x, y, yaw)
        ],
        "state_order": [
            "left_ee_cartesian_pos",
            "left_ee_rotation",
            "follow_left_gripper",
            "right_ee_cartesian_pos",
            "right_ee_rotation",
            "follow_right_gripper",
            "head_rotation",
            "waist",                   # 4 dimensions - Waist joint states
            "velocity_decomposed_odom",
        ],
        "state_source_mapping": {
            "left_ee_cartesian_pos": {
                "source": "left_arm_end_pose",
                "type": "position",
                "dim": 3
            },
            "left_ee_rotation": {
                "source": "left_arm_end_pose",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_position",
                "type": "gripper_position",
                "dim": 1
            },
            "right_ee_cartesian_pos": {
                "source": "right_arm_end_pose",
                "type": "position",
                "dim": 3
            },
            "right_ee_rotation": {
                "source": "right_arm_end_pose",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_right_gripper": {
                "source": "right_gripper_position",
                "type": "gripper_position",
                "dim": 1
            },
            "head_rotation": {
                "source": "head_joint_states",
                "type": "joint_positions",
                "dim": 2,
                "index": [0, 1]
            },
            "waist": {
                "source": "waist_joint_states",
                "type": "joint_positions",
                "dim": 4,
                "index": [0, 1, 2, 3]
            },
            "velocity_decomposed_odom": {
                "source": "odometry",
                "type": "odom_xy_yaw",
                "dim": 3
            }
        },
        "action_source_mapping": {
            "left_ee_cartesian_pos": {
                "source": "left_arm_end_pose_action",
                "type": "position",
                "dim": 3
            },
            "left_ee_rotation": {
                "source": "left_arm_end_pose_action",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_position_action",
                "type": "gripper_position",
                "dim": 1
            },
            "right_ee_cartesian_pos": {
                "source": "right_arm_end_pose_action",
                "type": "position",
                "dim": 3
            },
            "right_ee_rotation": {
                "source": "right_arm_end_pose_action",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_right_gripper": {
                "source": "right_gripper_position_action",
                "type": "gripper_position",
                "dim": 1
            },
            "head_rotation": {
                "source": "head_actions",
                "type": "joint_positions",
                "dim": 2,
                "index": [0, 1]
            },
            "waist": {
                "source": "waist_actions",
                "type": "joint_positions",
                "dim": 4,
                "index": [0, 1, 2, 3]
            },
            "velocity_decomposed_odom": {
                "source": "odometry_action",
                "type": "odom_xy_yaw",
                "dim": 3
            }
        }
    },
    "desktop": {
        # Desktop: arm + gripper only, no waist, head, or chassis/odometry
        "action_order": [
            "left_ee_cartesian_pos",   # 3 dimensions - Left arm end effector position
            "left_ee_rotation",        # 3 dimensions - Left arm end effector rotation (quaternion to euler)
            "follow_left_gripper",     # 1 dimension - Left gripper position
            "right_ee_cartesian_pos",  # 3 dimensions - Right arm end effector position
            "right_ee_rotation",       # 3 dimensions - Right arm end effector rotation (quaternion to euler)
            "follow_right_gripper",    # 1 dimension - Right gripper position
        ],
        "state_order": [
            "left_ee_cartesian_pos",
            "left_ee_rotation",
            "follow_left_gripper",
            "right_ee_cartesian_pos",
            "right_ee_rotation",
            "follow_right_gripper",
        ],
        "state_source_mapping": {
            "left_ee_cartesian_pos": {
                "source": "left_arm_end_pose",
                "type": "position",
                "dim": 3
            },
            "left_ee_rotation": {
                "source": "left_arm_end_pose",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "right_ee_cartesian_pos": {
                "source": "right_arm_end_pose",
                "type": "position",
                "dim": 3
            },
            "right_ee_rotation": {
                "source": "right_arm_end_pose",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_right_gripper": {
                "source": "right_gripper_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
        },
        "action_source_mapping": {
            "left_ee_cartesian_pos": {
                "source": "left_arm_end_pose_action",
                "type": "position",
                "dim": 3
            },
            "left_ee_rotation": {
                "source": "left_arm_end_pose_action",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "right_ee_cartesian_pos": {
                "source": "right_arm_end_pose_action",
                "type": "position",
                "dim": 3
            },
            "right_ee_rotation": {
                "source": "right_arm_end_pose_action",
                "type": "orientation_euler",
                "dim": 3
            },
            "follow_right_gripper": {
                "source": "right_gripper_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
        }
    },
    # Can add the configuration for other machine types here
    # "other_model": {
    #     "action_order": [...],  # action field order list
    #     "state_order": [...],   # state field order list (should be consistent with action_order)
    #     "state_source_mapping": {...},  # state field data source mapping
    #     "action_source_mapping": {...}   # action field data source mapping
    # }
}

# Default configuration using quanta_x1
DEFAULT_ROBOT_TYPE = "quanta_x1"


def get_robot_config(robot_type: str) -> Dict[str, Any]:
    """
    Get the configuration for the specified machine type (contains action and state extraction configuration)
    
    Args:
        robot_type: Machine type string
        
    Returns:
        Machine configuration dictionary, if not found, return the default configuration
    """
    # Try to match robot_type (supports partial matching)
    robot_type_lower = robot_type.lower()
    for key in ROBOT_DATA_CONFIG.keys():
        if key.lower() in robot_type_lower or robot_type_lower in key.lower():
            return ROBOT_DATA_CONFIG[key]
    
    # If no match is found, return the default configuration
    print(f"  ⚠️  Warning: No configuration found for machine type '{robot_type}', using default configuration (quanta_x1)")
    return ROBOT_DATA_CONFIG[DEFAULT_ROBOT_TYPE]


def get_robot_config_key(robot_type: str) -> str:
    """Get the canonical config key for robot_type (for consistency check)."""
    robot_type_lower = robot_type.lower()
    for key in ROBOT_DATA_CONFIG.keys():
        if key.lower() in robot_type_lower or robot_type_lower in key.lower():
            return key
    return DEFAULT_ROBOT_TYPE


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """
    Convert quaternion to Euler angle (roll, pitch, yaw), unit: radians
    
    Args:
        x, y, z, w: Four components of the quaternion
        
    Returns:
        (roll, pitch, yaw): Euler angle, unit: radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range (90 degrees is the maximum pitch angle)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def extract_action_v3(frame: Dict[str, Any], robot_type: str = DEFAULT_ROBOT_TYPE) -> Optional[np.ndarray]:
    """
    Extract action from the latest format frame, dynamically extract according to the configuration order and dimension
    
    Args:
        frame: Frame data dictionary
        robot_type: Machine type, for getting configuration
        
    Returns:
        action vector or None
    """
    # Get the machine configuration
    config = get_robot_config(robot_type)
    action_order = config.get("action_order", [])
    action_source_mapping = config.get("action_source_mapping", {})
    
    if 'action' not in frame:
        return None
    
    action_dict = frame['action']
    action_parts = []
    
    # Extract action according to the action_order order
    for action_field in action_order:
        if action_field not in action_source_mapping:
            # If no configuration is found, use default value to fill
            dim = 0
            if action_field in ("master_left_ee_cartesian_pos", "master_right_ee_cartesian_pos",
                               "left_ee_cartesian_pos", "right_ee_cartesian_pos"):
                dim = 3
            elif action_field in ("master_left_ee_rotation", "master_right_ee_rotation",
                                  "left_ee_rotation", "right_ee_rotation"):
                dim = 3
            elif action_field == "follow_left_gripper" or action_field == "follow_right_gripper" or action_field == "height":
                dim = 1
            elif action_field == "waist":
                dim = 4
            elif action_field == "head_rotation":
                dim = 2
            elif action_field == "velocity_decomposed_odom":
                dim = 3
            action_parts.extend([0.0] * dim)
            continue
        
        source_config = action_source_mapping[action_field]
        source_name = source_config["source"]
        source_type = source_config["type"]
        dim = source_config.get("dim", 0)
        
        # Extract data according to the source_type
        if source_type == "position":
            # Extract position from end_pose_action
            if source_name in action_dict:
                pose = action_dict[source_name]
                pos = pose.get('position', {})
                action_parts.extend([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0),
                    pos.get('z', 0.0)
                ])
            else:
                action_parts.extend([0.0, 0.0, 0.0])
        
        elif source_type == "orientation_euler":
            # Extract rotation from end_pose_action (quaternion to Euler angle)
            if source_name in action_dict:
                pose = action_dict[source_name]
                ori = pose.get('orientation', {})
                roll, pitch, yaw = quaternion_to_euler(
                    ori.get('x', 0.0),
                    ori.get('y', 0.0),
                    ori.get('z', 0.0),
                    ori.get('w', 1.0)
                )
                action_parts.extend([roll, pitch, yaw])
            else:
                action_parts.extend([0.0, 0.0, 0.0])
        
        elif source_type == "gripper_position":
            # Extract scalar position from gripper_position_action {position: value}
            if source_name in action_dict:
                data = action_dict[source_name]
                pos = data.get('position', 0.0)
                if isinstance(pos, (int, float)):
                    action_parts.append(float(pos))
                else:
                    action_parts.append(0.0)
            else:
                action_parts.append(0.0)
        
        elif source_type == "joint_positions":
            # Extract joint positions from actions
            if source_name in action_dict:
                actions = action_dict[source_name]
                positions = actions.get('positions', [])
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        # Multiple indices
                        for idx in index:
                            if idx < len(positions):
                                action_parts.append(positions[idx])
                            else:
                                action_parts.append(0.0)
                    else:
                        # Single index
                        if index < len(positions):
                            action_parts.append(positions[index])
                        else:
                            action_parts.append(0.0)
                else:
                    # No specified index, use all positions
                    action_parts.extend(positions[:dim] if len(positions) >= dim else positions + [0.0] * (dim - len(positions)))
            else:
                # Data does not exist, fill with zero
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        action_parts.extend([0.0] * len(index))
                    else:
                        action_parts.append(0.0)
                else:
                    action_parts.extend([0.0] * dim)
        
        elif source_type == "odom_xy_yaw":
            # Extract x, y, yaw from odometry_action
            if source_name in action_dict:
                odom = action_dict[source_name]
                pose = odom.get('pose', {})
                pos = pose.get('position', {})
                ori = pose.get('orientation', {})
                # Position x, y
                action_parts.extend([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0)
                ])
                # Quaternion to yaw angle
                _, _, yaw = quaternion_to_euler(
                    ori.get('x', 0.0),
                    ori.get('y', 0.0),
                    ori.get('z', 0.0),
                    ori.get('w', 1.0)
                )
                action_parts.append(yaw)
            else:
                action_parts.extend([0.0, 0.0, 0.0])
        
        else:
            # Unknown type, fill with zero
            action_parts.extend([0.0] * dim)
    
    return np.array(action_parts, dtype=np.float32)


def extract_joint_names_from_v3_episode(episode_data: Dict[str, Any]) -> List[str]:
    """
    Extract joint names list from V3 format episode
    
    Args:
        episode_data: Episode data dictionary, contains joint_names field
        
    Returns:
        Flattened joint names list, order consistent with the extraction order in extract_state_v3
    """
    joint_names_dict = episode_data.get('joint_names', {})
    if not isinstance(joint_names_dict, dict):
        return []
    
    joint_names_list = []
    
    # Extract joint names according to the order in extract_state_v3
    # left_arm (if exists)
    if 'left_arm' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['left_arm'])
    
    # right_arm
    if 'right_arm' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['right_arm'])
    
    # lift (quanta_x1)
    if 'lift' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['lift'])
    
    # waist (quanta_x2)
    if 'waist' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['waist'])
    
    # left_gripper
    if 'left_gripper' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['left_gripper'])
    
    # right_gripper
    if 'right_gripper' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['right_gripper'])
    
    # head
    if 'head' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['head'])
    
    return joint_names_list


def extract_state_v3(frame: Dict[str, Any], joint_indices: Optional[List[int]] = None, robot_type: str = DEFAULT_ROBOT_TYPE) -> Optional[np.ndarray]:
    """
    Extract state from the latest format frame, extract according to the configuration order and dimension, consistent with action
    
    Extract state from observation according to the state_order configuration order:
    1. master_left_ee_cartesian_pos: Master left arm end effector position
    2. master_left_ee_rotation: Master left arm end effector rotation angle
    3. follow_left_gripper: Follow left gripper position
    4. master_right_ee_cartesian_pos: Master right arm end effector position
    5. master_right_ee_rotation: Master right arm end effector rotation angle
    6. follow_right_gripper: Follow right gripper position
    7. head_rotation: Head motor angle
    8. height: Lift height
    9. velocity_decomposed_odom: Odometry information
    
    Args:
        frame: Frame data dictionary
        joint_indices: Joint indices to select (if None, select all joints, note: this parameter is no longer used in the current implementation, because state no longer contains original joint data)
        robot_type: Machine type, for getting configuration
        
    Returns:
        state vector or None
    """
    if 'observation' not in frame:
        return None
    
    obs = frame['observation']
    
    # Get the machine configuration
    config = get_robot_config(robot_type)
    state_order = config.get("state_order", config.get("action_order", []))
    state_source_mapping = config.get("state_source_mapping", {})
    
    state_parts = []
    
    # Extract state according to the state_order order
    for state_field in state_order:
        if state_field not in state_source_mapping:
            # If no configuration is found, use default value to fill
            dim = 0
            if state_field in ("master_left_ee_cartesian_pos", "master_right_ee_cartesian_pos",
                              "left_ee_cartesian_pos", "right_ee_cartesian_pos"):
                dim = 3
            elif state_field in ("master_left_ee_rotation", "master_right_ee_rotation",
                                 "left_ee_rotation", "right_ee_rotation"):
                dim = 3
            elif state_field == "follow_left_gripper" or state_field == "follow_right_gripper" or state_field == "height":
                dim = 1
            elif state_field == "head_rotation":
                dim = 2
            elif state_field == "velocity_decomposed_odom":
                dim = 3
            state_parts.extend([0.0] * dim)
            continue
        
        source_config = state_source_mapping[state_field]
        source_name = source_config["source"]
        source_type = source_config["type"]
        dim = source_config.get("dim", 0)
        
        # Extract data according to the source_type
        if source_type == "position":
            # Extract position from end_pose
            if source_name in obs:
                pose = obs[source_name]
                pos = pose.get('position', {})
                state_parts.extend([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0),
                    pos.get('z', 0.0)
                ])
            else:
                state_parts.extend([0.0, 0.0, 0.0])
        
        elif source_type == "orientation_euler":
            # Extract rotation from end_pose (quaternion to Euler angle)
            if source_name in obs:
                pose = obs[source_name]
                ori = pose.get('orientation', {})
                roll, pitch, yaw = quaternion_to_euler(
                    ori.get('x', 0.0),
                    ori.get('y', 0.0),
                    ori.get('z', 0.0),
                    ori.get('w', 1.0)
                )
                state_parts.extend([roll, pitch, yaw])
            else:
                state_parts.extend([0.0, 0.0, 0.0])
        
        elif source_type == "gripper_position":
            # Extract scalar position from gripper_position {position: value}
            if source_name in obs:
                data = obs[source_name]
                pos = data.get('position', 0.0)
                if isinstance(pos, (int, float)):
                    state_parts.append(float(pos))
                else:
                    state_parts.append(0.0)
            else:
                state_parts.append(0.0)
        
        elif source_type == "joint_positions":
            # Extract joint positions from joint_states
            if source_name in obs:
                joint_states = obs[source_name]
                positions = joint_states.get('positions', [])
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        # Multiple indices
                        for idx in index:
                            if idx < len(positions):
                                state_parts.append(positions[idx])
                            else:
                                state_parts.append(0.0)
                    else:
                        # Single index
                        if index < len(positions):
                            state_parts.append(positions[index])
                        else:
                            state_parts.append(0.0)
                else:
                    # No specified index, use all positions
                    state_parts.extend(positions[:dim] if len(positions) >= dim else positions + [0.0] * (dim - len(positions)))
            else:
                # Data does not exist, fill with zero
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        state_parts.extend([0.0] * len(index))
                    else:
                        state_parts.append(0.0)
                else:
                    state_parts.extend([0.0] * dim)
        
        elif source_type == "odom_xy_yaw":
            # Extract x, y, yaw from odometry
            if source_name in obs:
                odom = obs[source_name]
                pose = odom.get('pose', {})
                pos = pose.get('position', {})
                ori = pose.get('orientation', {})
                # Position x, y
                state_parts.extend([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0)
                ])
                # Quaternion to yaw angle
                _, _, yaw = quaternion_to_euler(
                    ori.get('x', 0.0),
                    ori.get('y', 0.0),
                    ori.get('z', 0.0),
                    ori.get('w', 1.0)
                )
                state_parts.append(yaw)
            else:
                state_parts.extend([0.0, 0.0, 0.0])
        
        else:
            # Unknown type, fill with zero
            state_parts.extend([0.0] * dim)
    
    if not state_parts:
        return None
    
    return np.array(state_parts, dtype=np.float32)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert collected JSON data to LeRobot dataset format\n\n"
                    "Automatically detect data format, support all sensor data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Use examples:

  Basic conversion:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --robot-type "quanta_x1"

  Select specific episodes:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --episodes "0,1,2" \\
        --robot-type "quanta_x1"

  Select specific joints:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --select-joints "left_arm_joint1,left_arm_joint2,right_arm_joint1" \\
        --robot-type "quanta_x1"

  Include extra sensor data (IMU, odometry, etc.):
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --include-sensors "odometry,chassis_imu,pose,left_arm_end_pose,right_arm_end_pose" \\
        --robot-type "quanta_x1"

  Use video format:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --use-videos \\
        --video-backend pyav \\
        --robot-type "quanta_x1"

Important notes:
  - task field: In LeRobot, task is a special field that should not be defined in the features dictionary.
    Each frame must contain the task field (string type), but it does not need to be defined in the features.
    LeRobot will automatically process task when adding a frame, and store it in the episode_buffer.
  
  - Sensor data: New sensor data will be saved as additional observation fields
    For example: observation.odometry, observation.left_arm_end_pose, etc.
        """
    )
    parser.add_argument("--input-dir", type=str, required=True, 
                       help="Input data directory (contains dataset_metadata.json and episode_* directory)")
    parser.add_argument("--output-dir", type=str, required=True, 
                       help="Output LeRobot dataset directory")
    parser.add_argument("--repo-id", type=str, required=True, 
                       help="Dataset repo ID (e.g.: 'my_robot/dataset')")
    parser.add_argument("--robot-type", type=str, default="quanta_x1", 
                       help="Machine type (default: quanta_x1, currently supported machines: quanta_x1, quanta_x2)")
    parser.add_argument("--fps", type=int, default=30, 
                       help="Target frame rate (default: 30)")
    parser.add_argument("--resize-width", type=int, default=640, 
                       help="Image width (default: 640)")
    parser.add_argument("--resize-height", type=int, default=480, 
                       help="Image height (default: 480)")
    parser.add_argument("--delta-action", action="store_true", 
                       help="Use delta action (action = next state - current state)")
    parser.add_argument("--relative-start", action="store_true", 
                       help="Relative to start point (state and action are subtracted by the value of the first frame)")
    parser.add_argument("--select-joints", type=str, default=None, 
                       help="Select specific joints (comma separated, e.g.: 'left_arm_joint1,left_arm_joint2')")
    parser.add_argument("--episodes", type=str, default=None, 
                       help="Select specific episodes (e.g.: '0,1,2' or '0-5', default: all)")
    parser.add_argument("--use-videos", action="store_true", 
                       help="Use video format instead of image (can reduce storage space)")
    parser.add_argument("--video-backend", type=str, default=None, 
                       choices=["pyav", "opencv"], 
                       help="Video backend (only valid when using --use-videos, optional: pyav, opencv)")
    parser.add_argument("--include-sensors", type=str, default=None, 
                       help="Include extra sensor data (comma separated, e.g.: 'odometry,chassis_imu,left_arm_end_pose')")
    parser.add_argument("--future-action-chunk-size", type=int, default=32,
                       help="Number of frames when using future state as action (default: 32)")
    return parser.parse_args()


def load_dataset_metadata(input_dir: Path) -> Dict[str, Any]:
    """Load dataset metadata"""
    metadata_file = input_dir / "dataset_metadata.json"
    if not metadata_file.exists():
        raise FileNotFoundError(f"Metadata file not found: {metadata_file}")
    
    with open(metadata_file, 'r', encoding='utf-8') as f:
        return json.load(f)


def load_episode(input_dir: Path, episode_id: int) -> Dict[str, Any]:
    """Load single episode data - only load metadata, not all frames"""
    episode_dir = input_dir / f"episode_{episode_id:04d}"
    episode_json = episode_dir / "episode.json"
    
    if not episode_json.exists():
        raise FileNotFoundError(f"Episode file not found: {episode_json}")
    
    with open(episode_json, 'r', encoding='utf-8') as f:
        episode_data = json.load(f)
    
    # Do not preload image paths, but record episode directory
    episode_data['_episode_dir'] = episode_dir
    
    # Detect storage format and load
    storage_format = episode_data.get('storage_format', 'images')
    episode_data['_storage_format'] = storage_format
    
    if storage_format == 'video' and 'video_files' in episode_data:
        # Open all video files
        video_captures = {}
        for cam_name, video_file in episode_data['video_files'].items():
            video_path = episode_dir / video_file
            if video_path.exists():
                cap = cv2.VideoCapture(str(video_path))
                if cap.isOpened():
                    video_captures[cam_name] = cap
                    # Get video frame count to verify
                    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                    print(f"  Load video: {cam_name}.mp4 ({frame_count} frames)")
                else:
                    print(f"  ⚠️  Warning: Unable to open video file: {video_path}")
        
        episode_data['_video_captures'] = video_captures
    
    return episode_data


def load_episode_metadata(input_dir: Path, episode_id: int) -> Dict[str, Any]:
    """Load episode metadata only, not all frames"""
    episode_dir = input_dir / f"episode_{episode_id:04d}"
    episode_json = episode_dir / "episode.json"
    
    if not episode_json.exists():
        raise FileNotFoundError(f"Episode file not found: {episode_json}")
    
    with open(episode_json, 'r', encoding='utf-8') as f:
        # Only read metadata
        episode_data = json.load(f)
    
    # Return lightweight metadata
    metadata = {
        'episode_id': episode_data.get('episode_id', episode_id),
        'task': episode_data.get('task', 'unknown'),
        'num_frames': episode_data.get('num_frames', len(episode_data.get('frames', []))),
        'duration': episode_data.get('duration', 0),
        'joint_names': episode_data.get('joint_names', []),
        '_episode_dir': episode_dir,
        '_has_frames': 'frames' in episode_data and len(episode_data['frames']) > 0
    }
    
    # Detect data format and get first frame information
    if metadata['_has_frames']:
        first_frame = episode_data['frames'][0]
        metadata['_first_frame'] = first_frame
    
    return metadata


def parse_episode_selection(episodes_str: str, total_episodes: int) -> List[int]:
    """Parse episode selection string"""
    if episodes_str is None:
        return list(range(total_episodes))
    
    selected = []
    for part in episodes_str.split(','):
        if '-' in part:
            start, end = map(int, part.split('-'))
            selected.extend(range(start, end + 1))
        else:
            selected.append(int(part))
    
    return sorted(set(selected))


def select_joints(joint_names: List[str], select_str: Optional[str]) -> List[int]:
    """Select specific joint indices"""
    if select_str is None:
        return list(range(len(joint_names)))
    
    selected_names = [name.strip() for name in select_str.split(',')]
    indices = []
    
    for name in selected_names:
        if name in joint_names:
            indices.append(joint_names.index(name))
        else:
            print(f"Warning: Joint '{name}' does not exist, skip")
    
    return indices


def _detect_data_format(episode_data: Dict[str, Any]) -> bool:
    """Detect data format (new format with more sensor data)"""
    if not episode_data.get('frames'):
        return False
    first_frame = episode_data['frames'][0]
    # V3 format: In observation, there are right_arm_joint_states, etc. fields, and in action, there are master_left_arm_end_pose_action, etc.
    if 'observation' in first_frame and 'right_arm_joint_states' in first_frame['observation']:
        return True
    # V2 format: In frames, there are joint_states fields directly
    if 'joint_states' in first_frame:
        return True
    # Old format detection
    return 'collection_config' in episode_data


def _has_action_field(episode_data: Dict[str, Any]) -> bool:
    """Detect if episode contains action field"""
    if not episode_data.get('frames'):
        return False
    
    # Check if there is action field in the first few frames
    for frame in episode_data['frames'][:5]:
        if 'action' in frame:
            # V3 format: Check if there are master_left_arm_end_pose_action, etc. fields
            if isinstance(frame['action'], dict):
                if 'master_left_arm_end_pose_action' in frame['action']:
                    return True
            # V2 format or other format
            return True
    
    return False


def _is_v3_format(frame: Dict[str, Any]) -> bool:
    """Detect if it is V3 format (latest format).
    Supports both quanta_x1 (master_* fields) and quanta_x2 (no master prefix).
    """
    if 'observation' not in frame or 'action' not in frame or not isinstance(frame['action'], dict):
        return False
    obs = frame['observation']
    act = frame['action']
    # quanta_x1: right_arm_joint_states + master_left_arm_end_pose_action
    if 'right_arm_joint_states' in obs and 'master_left_arm_end_pose_action' in act:
        return True
    # quanta_x2: left_arm_end_pose + left_arm_end_pose_action (no master)
    if 'left_arm_end_pose' in obs and 'left_arm_end_pose_action' in act:
        return True
    return False


def _extract_sensor_data(frame: Dict[str, Any], sensor_name: str) -> Optional[np.ndarray]:
    """Extract sensor data from V2 format frame and convert to numpy array"""
    if sensor_name not in frame:
        return None

    sensor_data = frame[sensor_name]

    # Process different types of sensor data
    if sensor_name == 'joint_states':
        # joint_states is now an array: [positions, velocities, efforts]
        if isinstance(sensor_data, list) and len(sensor_data) >= 1:
            return np.array(sensor_data[0], dtype=np.float32)  # positions
        return np.array(sensor_data.get('positions', []), dtype=np.float32)

    elif sensor_name == 'action':
        # New action format: contains multiple command streams in a dictionary
        if isinstance(sensor_data, dict):
            # Combine all command streams into a single action vector
            action_parts = []
            # Sort by priority, ensure consistent action vector order
            command_keys = sorted(sensor_data.keys())
            for key in command_keys:
                cmd_data = sensor_data[key]
                if isinstance(cmd_data, (list, tuple)):
                    action_parts.extend(cmd_data)
                elif isinstance(cmd_data, dict):
                    # Process complex commands (e.g. pose)
                    if 'positions' in cmd_data:
                        action_parts.extend(cmd_data['positions'])
                    elif 'position' in cmd_data and 'orientation' in cmd_data:
                        # Pose command
                        pos = cmd_data['position']
                        ori = cmd_data['orientation']
                        action_parts.extend([pos['x'], pos['y'], pos['z'],
                                           ori['x'], ori['y'], ori['z'], ori['w']])
                    else:
                        # Other dictionary format, flatten all values
                        for v in cmd_data.values():
                            if isinstance(v, (int, float)):
                                action_parts.append(v)
            return np.array(action_parts, dtype=np.float32)
        # Backward compatible with old format
        return np.array(sensor_data.get('positions', []), dtype=np.float32)

    elif sensor_name.endswith('_end_pose'):
        # End pose: position(3) + orientation(4) = 7 dimensions
        pos = sensor_data.get('position', {})
        ori = sensor_data.get('orientation', {})
        return np.array([
            pos.get('x', 0), pos.get('y', 0), pos.get('z', 0),
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)
        ], dtype=np.float32)

    elif sensor_name == 'odometry':
        # Odometry: position(3) + orientation(4) + linear_vel(3) + angular_vel(3) = 13 dimensions
        pose = sensor_data.get('pose', {})
        pos = pose.get('position', {})
        ori = pose.get('orientation', {})
        twist = sensor_data.get('twist', {})
        lin = twist.get('linear', {})
        ang = twist.get('angular', {})
        return np.array([
            pos.get('x', 0), pos.get('y', 0), pos.get('z', 0),
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1),
            lin.get('x', 0), lin.get('y', 0), lin.get('z', 0),
            ang.get('x', 0), ang.get('y', 0), ang.get('z', 0)
        ], dtype=np.float32)

    elif sensor_name == 'pose':
        # Position data: position(3) + orientation(4) = 7 dimensions
        pos = sensor_data.get('position', {})
        ori = sensor_data.get('orientation', {})
        return np.array([
            pos.get('x', 0), pos.get('y', 0), pos.get('z', 0),
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)
        ], dtype=np.float32)

    elif sensor_name == 'chassis_imu':
        # IMU: orientation(4) + angular_vel(3) + linear_acc(3) = 10 dimensions
        ori = sensor_data.get('orientation', {})
        ang = sensor_data.get('angular_velocity', {})
        lin = sensor_data.get('linear_acceleration', {})
        return np.array([
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1),
            ang.get('x', 0), ang.get('y', 0), ang.get('z', 0),
            lin.get('x', 0), lin.get('y', 0), lin.get('z', 0)
        ], dtype=np.float32)

    elif sensor_name.endswith('_tactile'):
        # Tactile sensor: return force array
        return np.array(sensor_data.get('force', []), dtype=np.float32)

    else:
        # Other sensors, try to convert to array
        if isinstance(sensor_data, (list, tuple)):
            return np.array(sensor_data, dtype=np.float32)
        return None


def process_single_frame(
    frame: Dict[str, Any],
    episode_dir: Path,
    joint_indices: List[int],
    resize_width: int,
    resize_height: int,
    is_new_format: bool,
    camera_name_mapping: Optional[Dict[str, str]] = None,
    include_sensors: Optional[List[str]] = None,
    storage_format: str = 'images',
    video_captures: Optional[Dict[str, cv2.VideoCapture]] = None,
    override_action: Optional[np.ndarray] = None,
    robot_type: str = DEFAULT_ROBOT_TYPE
) -> Dict[str, Any]:
    """Process single frame, memory efficient
    
    Args:
        camera_name_mapping: Map camera names in frame to camera names in metadata
                            例如: {'head_camera': 'head_rgb_stream'}
        storage_format: Storage format ('images' or 'video')
        video_captures: Video capture object dictionary (when storage_format='video')
        override_action: If provided, use this action instead of extracting from frame
    """
    result = {}
    
    # Detect if it is V3 format (latest format)
    is_v3_format = _is_v3_format(frame)
    
    # Process joint states
    if is_v3_format:
        # V3 format: Extract joint states from observation
        state_data = extract_state_v3(frame, joint_indices, robot_type=robot_type)
        if state_data is not None:
            result['state'] = state_data
        
        # Actions (V3 format)
        if override_action is not None:
            # Use external provided action
            result['action'] = override_action
        elif 'action' in frame:
            action_data = extract_action_v3(frame, robot_type=robot_type)
            if action_data is not None:
                result['action'] = action_data
            else:
                result['action'] = None
        else:
            # If there is no action, mark as needing to use future state
            result['action'] = None
    
    elif is_new_format and 'joint_states' in frame:
        # V2 format: joint_states field
        joint_data = _extract_sensor_data(frame, 'joint_states')
        if joint_data is not None:
            if joint_indices:
                joint_data = joint_data[joint_indices]
            result['state'] = joint_data
        
        # Actions (V2 format)
        if override_action is not None:
            # Use external provided action
            result['action'] = override_action
        elif 'action' in frame:
            action_data = _extract_sensor_data(frame, 'action')
            if action_data is not None:
                result['action'] = action_data
        else:
            # If there is no action, mark as needing to use future state
            result['action'] = None
    
    # Old format
    elif 'observation' in frame and 'joint_positions' in frame['observation']:
        joint_pos = np.array(frame['observation']['joint_positions'], dtype=np.float32)
        if joint_indices:
            joint_pos = joint_pos[joint_indices]
        result['state'] = joint_pos
        
        if override_action is not None:
            # Use external provided action
            result['action'] = override_action
        elif 'action' in frame and 'joint_positions' in frame['action']:
            action_pos = np.array(frame['action']['joint_positions'], dtype=np.float32)
            if joint_indices:
                action_pos = action_pos[joint_indices]
            result['action'] = action_pos
        else:
            # If there is no action field, mark as needing to use future state
            result['action'] = None
    
    # Process extra sensors
    if is_new_format and include_sensors:
        result['sensors'] = {}
        for sensor_name in include_sensors:
            sensor_data = _extract_sensor_data(frame, sensor_name)
            if sensor_data is not None:
                result['sensors'][sensor_name] = sensor_data
    
    # Process images
    result['images'] = {}
    
    if storage_format == 'video' and video_captures:
        # Read frames from video
        for cam_name, frame_index in frame.get('images', {}).items():
            if cam_name in video_captures:
                cap = video_captures[cam_name]
                
                # Set video position to specified frame
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                
                # Read frame
                ret, img_bgr = cap.read()
                if ret:
                    # BGR -> RGB
                    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                    
                    # Resize
                    img_resized = cv2.resize(
                        img_rgb, 
                        (resize_width, resize_height),
                        interpolation=cv2.INTER_LANCZOS4
                    )
                    
                    # Convert to (C, H, W) format
                    img_array = img_resized.transpose(2, 0, 1)
                    
                    # Use mapped camera name
                    mapped_cam_name = camera_name_mapping.get(cam_name, cam_name) if camera_name_mapping else cam_name
                    result['images'][mapped_cam_name] = img_array
                else:
                    print(f"  ⚠️  Warning: Unable to read frame {frame_index} ({cam_name}) from video")
            else:
                print(f"  ⚠️  Warning: No video capture object found for camera {cam_name}")
    else:
        # Read frames from image files
        for cam_name, img_filename in frame.get('images', {}).items():
            img_path = episode_dir / img_filename
            img = Image.open(img_path)
            if img.mode != 'RGB':
                img = img.convert('RGB')
            try:
                resample = Image.Resampling.LANCZOS
            except AttributeError:
                resample = Image.LANCZOS
            img = img.resize((resize_width, resize_height), resample)
            # Convert to (C, H, W) format
            img_array = np.array(img).transpose(2, 0, 1)
            
            # Use mapped camera name (if there is mapping)
            mapped_cam_name = camera_name_mapping.get(cam_name, cam_name) if camera_name_mapping else cam_name
            result['images'][mapped_cam_name] = img_array
    
    return result


def extract_all_states(
    frames: List[Dict[str, Any]],
    joint_indices: List[int],
    is_new_format: bool,
    robot_type: str = DEFAULT_ROBOT_TYPE
) -> List[Optional[np.ndarray]]:
    """Extract all frame state data
    
    Returns:
        List of state arrays or None, each element is a numpy array or None
    """
    states = []
    for frame in frames:
        state = None
        # Detect if it is V3 format
        if _is_v3_format(frame):
            state = extract_state_v3(frame, joint_indices, robot_type=robot_type)
        elif is_new_format and 'joint_states' in frame:
            joint_data = _extract_sensor_data(frame, 'joint_states')
            if joint_data is not None:
                if joint_indices:
                    joint_data = joint_data[joint_indices]
                state = joint_data
        elif 'observation' in frame and 'joint_positions' in frame['observation']:
            joint_pos = np.array(frame['observation']['joint_positions'], dtype=np.float32)
            if joint_indices:
                joint_pos = joint_pos[joint_indices]
            state = joint_pos
        states.append(state)
    return states


def generate_future_actions(
    frames: List[Dict[str, Any]],
    joint_indices: List[int],
    is_new_format: bool,
    chunk_size: int = 32,
    robot_type: str = DEFAULT_ROBOT_TYPE
) -> List[Optional[np.ndarray]]:
    """Generate future state as action for data without action field
    
    Strategy:
    - For the i-th frame, action = state of the (i+1)-th to (i+chunk_size)-th frames
    - For the last chunk_size-1 frames: Use remaining future states, fill with the last frame if not enough
    - For the last frame: Return None (will be discarded)
    
    Args:
        frames: All frame data
        joint_indices: Joint indices
        is_new_format: Whether it is new format
        chunk_size: Number of future state frames
    
    Returns:
        List of action arrays or None, each element is a numpy array or None (last frame)
    """
    total_frames = len(frames)
    
    # First extract all states
    all_states = extract_all_states(frames, joint_indices, is_new_format, robot_type=robot_type)
    
    # Check if there are valid states
    valid_states = [s for s in all_states if s is not None]
    if not valid_states:
        return [None] * total_frames
    
    actions = []
    
    for i in range(total_frames):
        current_state = all_states[i]
        
        if current_state is None:
            actions.append(None)
            continue
        
        # Last frame: Return None (will be discarded)
        if i == total_frames - 1:
            actions.append(None)
            continue
        
        # Collect future states
        future_states = []
        for j in range(1, chunk_size + 1):
            future_idx = i + j
            if future_idx < total_frames and all_states[future_idx] is not None:
                future_states.append(all_states[future_idx])
            else:
                # Out of range or no state, use the last valid state to fill
                if future_states:
                    # Use the last collected future state
                    future_states.append(future_states[-1].copy())
                elif all_states[i] is not None:
                    # No future state, use the current state
                    future_states.append(all_states[i].copy())
        
        # Concatenate future states into action
        if future_states:
            action = np.concatenate(future_states)
            actions.append(action)
        else:
            actions.append(None)
    
    return actions


def process_episode_data(
    episode_data: Dict[str, Any],
    joint_indices: List[int],
    resize_width: int,
    resize_height: int,
    delta_action: bool = False,
    relative_start: bool = False,
    include_sensors: Optional[List[str]] = None,
    robot_type: str = DEFAULT_ROBOT_TYPE
) -> Dict[str, Any]:
    """Process episode data, automatically detect format - for analyzing action dimension"""
    frames = episode_data['frames']
    
    # Only process the first few frames to determine dimension
    sample_frames = frames[:min(10, len(frames))]
    
    # Detect data format
    is_new_format = _detect_data_format(episode_data)
    episode_dir = episode_data.get('_episode_dir')
    
    # Extract data
    states = []
    actions = []
    
    for frame in sample_frames:
        # Detect if it is V3 format
        is_v3_format = _is_v3_format(frame)
        
        if is_v3_format:
            # V3 format: Use extract_state_v3 and extract_action_v3
            state_data = extract_state_v3(frame, joint_indices, robot_type=robot_type)
            if state_data is not None:
                states.append(state_data)
            
            # Process action
            if 'action' in frame:
                action_data = extract_action_v3(frame, robot_type=robot_type)
                if action_data is not None:
                    actions.append(action_data)
                else:
                    # No action, use state as action
                    if state_data is not None:
                        actions.append(state_data.copy())
            else:
                # No action field, use state as action
                if state_data is not None:
                    actions.append(state_data.copy())
        
        # V2 format: joint_states field
        elif is_new_format and 'joint_states' in frame:
            joint_data = _extract_sensor_data(frame, 'joint_states')
            if joint_data is not None:
                if joint_indices:
                    joint_data = joint_data[joint_indices]
                states.append(joint_data)
                
                # Process action
                if 'action' in frame:
                    action_data = _extract_sensor_data(frame, 'action')
                    if action_data is not None:
                        actions.append(action_data)
                else:
                    # No action, use state as action
                    actions.append(joint_data.copy())
        
        # Old format
        elif 'observation' in frame and 'joint_positions' in frame['observation']:
            joint_pos = np.array(frame['observation']['joint_positions'], dtype=np.float32)
            if joint_indices:
                joint_pos = joint_pos[joint_indices]
            states.append(joint_pos)
            
            # Process action
            if 'action' in frame and 'joint_positions' in frame['action']:
                action_pos = np.array(frame['action']['joint_positions'], dtype=np.float32)
                if joint_indices:
                    action_pos = action_pos[joint_indices]
                actions.append(action_pos)
            else:
                # No action, use observation as action
                actions.append(joint_pos.copy())
    
    # Convert to numpy array
    states = np.array(states) if states else np.array([]).reshape(0, 0)
    actions = np.array(actions) if actions else np.array([]).reshape(0, 0)
    
    return {
        'states': states,
        'actions': actions,
        'images': {},  # Do not load images
    }


def create_lerobot_dataset(
    output_dir: str,
    repo_id: str,
    robot_type: str,
    fps: int,
    joint_names: List[str],
    camera_names: List[str],
    resize_width: int,
    resize_height: int,
    use_videos: bool = False,
    video_backend: Optional[str] = None,
    action_dim: Optional[int] = None,
    robot_type_for_action: Optional[str] = None,
    state_dim: Optional[int] = None,
    state_names: Optional[List[str]] = None
) -> LeRobotDataset:
    """Create LeRobot dataset
    
    Args:
        robot_type_for_action: Robot type for getting action names (default uses robot_type)
        state_dim: Actual dimension of state (if provided, use this value instead of calculating from state_names)
        state_names: Full list of state names (includes joints + end pose + odometry etc.)
    """

    features = {}

    # State features - using provided state_dim and state_names
    if state_names and len(state_names) > 0:
        actual_state_dim = state_dim if state_dim is not None else len(state_names)
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (actual_state_dim,),
            "names": state_names
        }
    elif joint_names and len(joint_names) > 0:
        # If state_names are not provided, only use joint_names (backwards compatibility)
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (len(joint_names),),
            "names": joint_names
        }

    # Action features - dynamically determine dimension and add names
    if action_dim and action_dim > 0:
        # Get action names
        action_names = []
        robot_type_for_names = robot_type_for_action or robot_type
        config = get_robot_config(robot_type_for_names)
        action_order = config["action_order"]
        
        # Build names according to action_order
        for action_name in action_order:
            if action_name == "master_left_ee_cartesian_pos":
                action_names.extend(["master_left_ee_pos_x", "master_left_ee_pos_y", "master_left_ee_pos_z"])
            elif action_name == "master_left_ee_rotation":
                action_names.extend(["master_left_ee_roll", "master_left_ee_pitch", "master_left_ee_yaw"])
            elif action_name == "left_ee_cartesian_pos":
                action_names.extend(["left_ee_pos_x", "left_ee_pos_y", "left_ee_pos_z"])
            elif action_name == "left_ee_rotation":
                action_names.extend(["left_ee_roll", "left_ee_pitch", "left_ee_yaw"])
            elif action_name == "follow_left_gripper":
                action_names.append("follow_left_gripper")
            elif action_name == "master_right_ee_cartesian_pos":
                action_names.extend(["master_right_ee_pos_x", "master_right_ee_pos_y", "master_right_ee_pos_z"])
            elif action_name == "master_right_ee_rotation":
                action_names.extend(["master_right_ee_roll", "master_right_ee_pitch", "master_right_ee_yaw"])
            elif action_name == "right_ee_cartesian_pos":
                action_names.extend(["right_ee_pos_x", "right_ee_pos_y", "right_ee_pos_z"])
            elif action_name == "right_ee_rotation":
                action_names.extend(["right_ee_roll", "right_ee_pitch", "right_ee_yaw"])
            elif action_name == "follow_right_gripper":
                action_names.append("follow_right_gripper")
            elif action_name == "head_rotation":
                action_names.extend(["head_pitch", "head_yaw"])
            elif action_name == "height":
                action_names.append("height")
            elif action_name == "waist":
                action_names.extend(["waist_joint_0", "waist_joint_1", "waist_joint_2", "waist_joint_3"])
            elif action_name == "velocity_decomposed_odom":
                action_names.extend(["odom_x", "odom_y", "odom_yaw"])
        
        # Ensure number of names matches action_dim
        if len(action_names) != action_dim:
            print(f"  ⚠️  警告: action names数量({len(action_names)})与action维度({action_dim})不匹配")
            # If names are not enough, fill with placeholders
            while len(action_names) < action_dim:
                action_names.append(f"action_{len(action_names)}")
            # If names are too many, truncate
            action_names = action_names[:action_dim]
        
        features["action"] = {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": action_names
        }
    else:
        # Default action dimension (for compatibility)
        features["action"] = {
            "dtype": "float32",
            "shape": (len(joint_names),) if joint_names else (1,),
            "names": joint_names if joint_names else None
        }

    # Add camera features
    for cam in camera_names:
        features[f"observation.images.{cam}"] = {
            "dtype": "video" if use_videos else "image",
            "shape": (3, resize_height, resize_width),
            "names": ["channels", "height", "width"],
        }

    # Note: task should not be defined in features!
    # task is a special field in LeRobot, it will be popped from frame in add_frame
    # Each frame must contain task field, but task should not be defined in features dictionary

    return LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        robot_type=robot_type,
        features=features,
        use_videos=use_videos,
        video_backend=video_backend,
        root=output_dir,
    )


def convert_dataset(args):
    """Convert dataset"""
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    
    print(f"Converting dataset...")
    print(f"  Input directory: {input_dir}")
    print(f"  Output directory: {output_dir}")
    
    # Load metadata
    metadata = load_dataset_metadata(input_dir)
    print(f"\nDataset information:")
    print(f"  Robot type: {args.robot_type}")
    print(f"  Original FPS: {metadata['fps']}")
    print(f"  Joint number: {len(metadata.get('joint_names', []))}")
    print(f"  Camera number: {len(metadata['camera_names'])}")
    print(f"  Episodes: {len(metadata['episodes'])}")
    
    # Select episodes (first parse, then extract joint_names from selected episodes)
    selected_episodes = parse_episode_selection(args.episodes, len(metadata['episodes']))
    
    # Check robot_type consistency with episode model (exact match, case-insensitive)
    for ep_idx in selected_episodes:
        if ep_idx >= len(metadata['episodes']):
            continue
        ep_model = metadata['episodes'][ep_idx].get('model', '')
        if ep_model.lower() != args.robot_type.lower():
            print(f"\nError: robot_type mismatch!")
            print(f"  --robot-type: {args.robot_type}")
            print(f"  Episode {ep_idx} model: {ep_model}")
            print(f"  Please use --robot-type \"{ep_model}\" to match the episode data.")
            return 1
    
    # Select joints
    # Extract complete joint_names from episodes, rather than relying on metadata (metadata may be incomplete)
    joint_names = []
    
    try:
        # Extract joint_names from selected episodes, if not selected, extract from all episodes
        episode_indices_to_check = selected_episodes[:3] if selected_episodes else list(range(min(3, len(metadata['episodes']))))
        print(f"  Checking episodes to extract joint names: {episode_indices_to_check}")
        
        for ep_idx in episode_indices_to_check:
            try:
                print(f"    Checking episode {ep_idx}...")
                full_episode = load_episode(input_dir, ep_idx)
                if 'joint_names' in full_episode:
                    joint_names_dict = full_episode.get('joint_names', {})
                    print(f"      Joint names structure of episode {ep_idx}: {list(joint_names_dict.keys())}")
                    extracted_names = extract_joint_names_from_v3_episode(full_episode)
                    if extracted_names:
                        joint_names = extracted_names
                        print(f"  ✓ Extracted joint names from episode {ep_idx}: {len(joint_names)} joints")
                        print(f"    Joint list: {joint_names}")
                        break
                    else:
                        print(f"      ⚠️  Joint names of episode {ep_idx} are empty")
                else:
                    print(f"      ⚠️  Episode {ep_idx} does not have joint_names field")
            except Exception as e:
                print(f"  ⚠️  Warning: Cannot extract joint names from episode {ep_idx}: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # If extraction from episode fails, try to extract from metadata (as a fallback)
        if not joint_names:
            print(f"  ⚠️  Warning: Cannot extract joint names from episode, trying to extract from metadata")
            metadata_joint_names = metadata.get('joint_names', [])
            if isinstance(metadata_joint_names, dict):
                extracted_names = extract_joint_names_from_v3_episode({'joint_names': metadata_joint_names})
                if extracted_names:
                    joint_names = extracted_names
                    print(f"  Extracted joint names from metadata: {len(joint_names)} joints")
                    print(f"    Joint list: {joint_names}")
            elif isinstance(metadata_joint_names, list) and len(metadata_joint_names) > 0:
                joint_names = metadata_joint_names
                print(f"  Extracted joint names from metadata: {len(joint_names)} joints")
                print(f"    Joint list: {joint_names}")
                print(f"  ⚠️  Warning: Joint names in metadata may be incomplete, suggest checking episode data")
    except Exception as e:
        print(f"  ⚠️  Warning: Cannot extract joint names: {e}")
        import traceback
        traceback.print_exc()
    
    # Ensure joint_names is a list
    if not isinstance(joint_names, list):
        joint_names = []
    
    # Validate joint_names (should have at least multiple joints, if only 1-2 may be incorrect)
    if len(joint_names) <= 2:
        print(f"  ⚠️  Warning: Extracted joint number is less ({len(joint_names)}), may be incomplete")
        print(f"    If data contains left and right arms, there should be more joints")
    
    joint_indices = select_joints(joint_names, args.select_joints)
    selected_joint_names = [joint_names[i] for i in joint_indices] if joint_indices else joint_names
    
    # Detect actual camera names (use selected episodes first, if not specified, use first 3)
    actual_camera_names = metadata['camera_names']  # Default using metadata
    try:
        # Use selected episodes first, if not specified, use first 3
        episodes_to_check = selected_episodes[:3] if selected_episodes else list(range(min(3, len(metadata['episodes']))))
        for ep_idx in episodes_to_check:
            if ep_idx >= len(metadata['episodes']):
                continue
            test_episode = load_episode_metadata(input_dir, ep_idx)
            if test_episode.get('_has_frames') and '_first_frame' in test_episode:
                first_frame = test_episode['_first_frame']
                if 'images' in first_frame:
                    actual_camera_names = list(first_frame['images'].keys())
                    if actual_camera_names != metadata['camera_names']:
                        print(f"\nWarning: Detected actual camera names are different from metadata")
                        print(f"  Metadata: {metadata['camera_names']}")
                        print(f"  Actual data: {actual_camera_names}")
                    break
    except Exception as e:
        print(f"  ⚠️  Warning: Cannot automatically detect camera names, using metadata: {e}")
    
    print(f"\nConversion configuration:")
    print(f"  Selected joint number: {len(selected_joint_names)}")
    if len(selected_joint_names) == 0:
        print(f"  ⚠️  Warning: No joint data, will only convert image data")
        print(f"  This may be because enable_joint_states=False was set during collection")
    elif args.select_joints:
        print(f"  Joint list: {selected_joint_names}")
    print(f"  Camera names: {actual_camera_names}")
    print(f"  Target FPS: {args.fps}")
    print(f"  Image size: {args.resize_width}x{args.resize_height}")
    print(f"  Delta action: {args.delta_action}")
    print(f"  Relative start: {args.relative_start}")
    print(f"  Use videos: {args.use_videos}")
    print(f"  Selected episodes: {selected_episodes}")
    
    # Analyze all selected episodes to determine action dimension
    action_dim = None
    print(f"\nAnalyzing episodes to determine data dimension...")

    for ep_idx in selected_episodes[:3]:  # Check first 3 episodes
        if ep_idx >= len(metadata['episodes']):
            continue

        try:
            # Use lightweight loading
            episode_data = load_episode_metadata(input_dir, ep_idx)
            
            # Only check first frame to determine dimension
            if episode_data.get('_has_frames') and '_first_frame' in episode_data:
                first_frame = episode_data['_first_frame']
                
                # Detect format
                is_v3_format = _is_v3_format(first_frame)
                is_new_format = 'joint_states' in first_frame or is_v3_format
                
                # Extract action dimension
                if is_v3_format:
                    # V3 format: use extract_action_v3 to extract action (master arm end pose format)
                    if 'action' in first_frame:
                        action_data = extract_action_v3(first_frame, robot_type=args.robot_type)
                        if action_data is not None:
                            current_action_dim = len(action_data)
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ Determined action dimension from episode {ep_idx} (V3 format): {action_dim}")
                            elif action_dim != current_action_dim:
                                print(f"  ⚠️  Warning: Action dimension of episode {ep_idx} ({current_action_dim}) is different from previous ({action_dim})")
                                action_dim = max(action_dim, current_action_dim)
                                print(f"  ✓ Updated action dimension to: {action_dim}")
                            break
                    else:
                        # No action field, use future state as action
                        state_data = extract_state_v3(first_frame, joint_indices, robot_type=args.robot_type)
                        if state_data is not None:
                            # action dimension = state dimension × chunk size
                            current_action_dim = len(state_data) * args.future_action_chunk_size
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ Determined action dimension from episode {ep_idx} (V3 format, using future state {args.future_action_chunk_size} frames): {action_dim}")
                            break
                
                elif is_new_format and 'joint_states' in first_frame:
                    # V2 format: use action first, if not then use joint_states
                    if 'action' in first_frame:
                        action_data = _extract_sensor_data(first_frame, 'action')
                        if action_data is not None:
                            current_action_dim = len(action_data)
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ Determined action dimension from episode {ep_idx}: {action_dim}")
                            elif action_dim != current_action_dim:
                                print(f"  ⚠️  Warning: Action dimension of episode {ep_idx} ({current_action_dim}) is different from previous ({action_dim})")
                                action_dim = max(action_dim, current_action_dim)
                                print(f"  ✓ Updated action dimension to: {action_dim}")
                            break
                    else:
                        # No action field, use future state as action
                        joint_data = _extract_sensor_data(first_frame, 'joint_states')
                        if joint_data is not None:
                            if joint_indices:
                                joint_data = joint_data[joint_indices]
                            # action dimension = state dimension × chunk size
                            current_action_dim = len(joint_data) * args.future_action_chunk_size
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ Determined action dimension from episode {ep_idx} (using future state {args.future_action_chunk_size} frames): {action_dim}")
                            break
                elif 'observation' in first_frame and 'joint_positions' in first_frame['observation']:
                    # Old format: use action first, if not then use observation
                    if 'action' in first_frame and 'joint_positions' in first_frame['action']:
                        action_pos = first_frame['action']['joint_positions']
                        if joint_indices:
                            current_action_dim = len(joint_indices)
                        else:
                            current_action_dim = len(action_pos)
                        
                        if action_dim is None:
                            action_dim = current_action_dim
                            print(f"  ✓ Determined action dimension from episode {ep_idx}: {action_dim}")
                        break
                    else:
                        # No action field, use future state as action
                        joint_pos = first_frame['observation']['joint_positions']
                        if joint_indices:
                            state_dim = len(joint_indices)
                        else:
                            state_dim = len(joint_pos)
                        
                        # action dimension = state dimension × chunk size
                        current_action_dim = state_dim * args.future_action_chunk_size
                        if action_dim is None:
                            action_dim = current_action_dim
                            print(f"  ✓ Determined action dimension from episode {ep_idx} (using future state {args.future_action_chunk_size} frames): {action_dim}")
                        break

        except Exception as e:
            print(f"  ⚠️  Error analyzing episode {ep_idx}: {e}")
            import traceback
            traceback.print_exc()
            continue

    if action_dim is None:
        # Default action dimension for V3 format is 20
        print(f"  ⚠️  No valid action data found, trying to use V3 format default dimension: 20")
        action_dim = 20

    # Detect actual state dimension and names (from first frame of first selected episode)
    state_dim_detected = None
    state_names_list = []
    try:
        for ep_idx in selected_episodes[:1]:  # Only check first selected episode
            test_episode = load_episode_metadata(input_dir, ep_idx)
            if test_episode.get('_has_frames') and '_first_frame' in test_episode:
                first_frame = test_episode['_first_frame']
                if _is_v3_format(first_frame):
                    test_state = extract_state_v3(first_frame, joint_indices, robot_type=args.robot_type)
                    if test_state is not None:
                        state_dim_detected = len(test_state)
                        print(f"  ✓ Detected state dimension from episode {ep_idx}: {state_dim_detected}")
                        
                        # Build state names, according to the order of state_order configuration, consistent with action names
                        config = get_robot_config(args.robot_type)
                        state_order = config.get("state_order", config.get("action_order", []))
                        
                        for state_field in state_order:
                            if state_field == "master_left_ee_cartesian_pos":
                                state_names_list.extend(["master_left_ee_pos_x", "master_left_ee_pos_y", "master_left_ee_pos_z"])
                            elif state_field == "master_left_ee_rotation":
                                state_names_list.extend(["master_left_ee_roll", "master_left_ee_pitch", "master_left_ee_yaw"])
                            elif state_field == "left_ee_cartesian_pos":
                                state_names_list.extend(["left_ee_pos_x", "left_ee_pos_y", "left_ee_pos_z"])
                            elif state_field == "left_ee_rotation":
                                state_names_list.extend(["left_ee_roll", "left_ee_pitch", "left_ee_yaw"])
                            elif state_field == "follow_left_gripper":
                                state_names_list.append("follow_left_gripper")
                            elif state_field == "master_right_ee_cartesian_pos":
                                state_names_list.extend(["master_right_ee_pos_x", "master_right_ee_pos_y", "master_right_ee_pos_z"])
                            elif state_field == "master_right_ee_rotation":
                                state_names_list.extend(["master_right_ee_roll", "master_right_ee_pitch", "master_right_ee_yaw"])
                            elif state_field == "right_ee_cartesian_pos":
                                state_names_list.extend(["right_ee_pos_x", "right_ee_pos_y", "right_ee_pos_z"])
                            elif state_field == "right_ee_rotation":
                                state_names_list.extend(["right_ee_roll", "right_ee_pitch", "right_ee_yaw"])
                            elif state_field == "follow_right_gripper":
                                state_names_list.append("follow_right_gripper")
                            elif state_field == "head_rotation":
                                state_names_list.extend(["head_pitch", "head_yaw"])
                            elif state_field == "height":
                                state_names_list.append("height")
                            elif state_field == "waist":
                                state_names_list.extend(["waist_joint_0", "waist_joint_1", "waist_joint_2", "waist_joint_3"])
                            elif state_field == "velocity_decomposed_odom":
                                state_names_list.extend(["odom_x", "odom_y", "odom_yaw"])
                        
                        print(f"    State names number: {len(state_names_list)}")
                        print(f"    State names: {state_names_list}")
                        break
    except Exception as e:
        print(f"  ⚠️  Warning: Cannot detect state dimension: {e}")
        import traceback
        traceback.print_exc()
    
    # If detection fails, use default values configured
    if state_dim_detected is None or not state_names_list:
        # Use configuration to generate default state names
        config = get_robot_config(args.robot_type)
        state_order = config.get("state_order", config.get("action_order", []))
        state_names_list = []
        for state_field in state_order:
            if state_field == "master_left_ee_cartesian_pos":
                state_names_list.extend(["master_left_ee_pos_x", "master_left_ee_pos_y", "master_left_ee_pos_z"])
            elif state_field == "master_left_ee_rotation":
                state_names_list.extend(["master_left_ee_roll", "master_left_ee_pitch", "master_left_ee_yaw"])
            elif state_field == "left_ee_cartesian_pos":
                state_names_list.extend(["left_ee_pos_x", "left_ee_pos_y", "left_ee_pos_z"])
            elif state_field == "left_ee_rotation":
                state_names_list.extend(["left_ee_roll", "left_ee_pitch", "left_ee_yaw"])
            elif state_field == "follow_left_gripper":
                state_names_list.append("follow_left_gripper")
            elif state_field == "master_right_ee_cartesian_pos":
                state_names_list.extend(["master_right_ee_pos_x", "master_right_ee_pos_y", "master_right_ee_pos_z"])
            elif state_field == "master_right_ee_rotation":
                state_names_list.extend(["master_right_ee_roll", "master_right_ee_pitch", "master_right_ee_yaw"])
            elif state_field == "right_ee_cartesian_pos":
                state_names_list.extend(["right_ee_pos_x", "right_ee_pos_y", "right_ee_pos_z"])
            elif state_field == "right_ee_rotation":
                state_names_list.extend(["right_ee_roll", "right_ee_pitch", "right_ee_yaw"])
            elif state_field == "follow_right_gripper":
                state_names_list.append("follow_right_gripper")
            elif state_field == "head_rotation":
                state_names_list.extend(["head_pitch", "head_yaw"])
            elif state_field == "height":
                state_names_list.append("height")
            elif state_field == "waist":
                state_names_list.extend(["waist_joint_0", "waist_joint_1", "waist_joint_2", "waist_joint_3"])
            elif state_field == "velocity_decomposed_odom":
                state_names_list.extend(["odom_x", "odom_y", "odom_yaw"])
        state_dim_detected = len(state_names_list)
        print(f"  ⚠️  Warning: Cannot detect state dimension, using default values: {state_dim_detected}")
    
    # Create LeRobot dataset
    print(f"\nCreating LeRobot dataset...")
    dataset = create_lerobot_dataset(
        output_dir=str(output_dir),
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        fps=args.fps,
        joint_names=selected_joint_names,
        camera_names=actual_camera_names,  # Use actual detected camera names
        resize_width=args.resize_width,
        resize_height=args.resize_height,
        use_videos=args.use_videos,
        video_backend=args.video_backend,
        action_dim=action_dim,
        robot_type_for_action=args.robot_type,
        state_dim=state_dim_detected,
        state_names=state_names_list
    )
    
    # Convert each episode
    print(f"\nStarting to convert episodes...")
    for ep_idx in selected_episodes:
        if ep_idx >= len(metadata['episodes']):
            print(f"Warning: Episode {ep_idx} does not exist, skipping")
            continue
        
        ep_meta = metadata['episodes'][ep_idx]
        print(f"\nProcessing Episode {ep_idx}: {ep_meta['task']}")
        print(f"  Frame number: {ep_meta['num_frames']}")
        print(f"  Duration: {ep_meta['duration']:.2f}s")
        
        # Load episode data
        episode_data = load_episode(input_dir, ep_idx)
        frames = episode_data['frames']
        episode_dir = episode_data['_episode_dir']
        
        if not frames:
            print(f"  ⚠️  Warning: Episode {ep_idx} has no data, skipping")
            continue
        
        # Detect data format
        is_new_format = _detect_data_format(episode_data)
        
        # Detect if it is V3 format
        is_v3_format = False
        if episode_data.get('frames'):
            is_v3_format = _is_v3_format(episode_data['frames'][0])
        
        # Get storage format and video capture objects
        storage_format = episode_data.get('_storage_format', 'images')
        video_captures = episode_data.get('_video_captures', {})
        
        if storage_format == 'video':
            print(f"  Storage format: MP4 video ({len(video_captures)} files)")
        else:
            print(f"  Storage format: JPG images")
        
        # task string
        task_str = ep_meta['task']
        
        # Detect if there is an action field
        has_action = _has_action_field(episode_data)
        
        # If there is no action field, generate future state actions
        future_actions = None
        if not has_action:
            print(f"  No action field detected, using future state (chunk_size={args.future_action_chunk_size}) as action")
            future_actions = generate_future_actions(
                frames,
                joint_indices,
                is_new_format,
                args.future_action_chunk_size,
                robot_type=args.robot_type
            )
            print(f"  ✓ Generated {len([a for a in future_actions if a is not None])} future actions")
        
        # For V3 format, if action field is detected, ensure using extract_action_v3
        if is_v3_format and has_action:
            print(f"  Detected V3 format, using new action extraction method")
        
        # Baseline for relative start point
        first_state = None
        first_action = None
        
        # Process each frame and add to dataset
        print(f"  Starting to process each frame...")
        total_frames = len(frames)
        frames_added = 0
        
        for i in range(total_frames):
            # If using future actions and current frame's action is None (last frame), skip
            if future_actions is not None and future_actions[i] is None:
                print(f"    Skipping last frame (frame {i})")
                continue
            
            # Get current frame and clean up references after processing
            frame_data = frames[i]
            
            # Get current frame's action (if using future actions)
            override_action = future_actions[i] if future_actions is not None else None
            
            # Process single frame
            processed_frame = process_single_frame(
                frame_data,
                episode_dir,
                joint_indices,
                args.resize_width,
                args.resize_height,
                is_new_format,
                include_sensors=None,
                storage_format=storage_format,
                video_captures=video_captures,
                override_action=override_action,
                robot_type=args.robot_type
            )
            
            # Build dataset frame
            frame = {"task": task_str}
            
            # Process state (only add when dataset.features defines observation.state)
            if "observation.state" in dataset.features:
                if 'state' in processed_frame:
                    state = processed_frame['state']
                    
                    # Record first frame for relative start point
                    if args.relative_start and first_state is None:
                        first_state = state.copy()
                    
                    # Apply relative start point
                    if args.relative_start and first_state is not None:
                        state = state - first_state
                    
                    frame["observation.state"] = torch.from_numpy(state).type(torch.float32)
                else:
                    # No state data, fill with zero
                    state_shape = dataset.features["observation.state"]["shape"]
                    if isinstance(state_shape, tuple) and len(state_shape) == 1:
                        state_dim = state_shape[0]
                        frame["observation.state"] = torch.zeros(state_dim, dtype=torch.float32)
            
            # Process action
            if 'action' in processed_frame and processed_frame['action'] is not None:
                action = processed_frame['action']
                
                # Record first frame for relative start point
                if args.relative_start and first_action is None:
                    first_action = action.copy()
                
                # Apply relative start point
                if args.relative_start and first_action is not None:
                    action = action - first_action
                
                # Delta action processing (disabled when using future actions)
                if args.delta_action and 'state' in processed_frame and future_actions is None:
                    action = action - processed_frame['state']
                
                # Verify and adjust action dimension
                if action_dim is not None:
                    actual_action_dim = len(action)
                    if actual_action_dim != action_dim:
                        if actual_action_dim < action_dim:
                            # Fill with zeros
                            padding = np.zeros(action_dim - actual_action_dim, dtype=np.float32)
                            action = np.concatenate([action, padding])
                        else:
                            # Truncate
                            action = action[:action_dim]
                
                frame["action"] = torch.from_numpy(action).type(torch.float32)
            else:
                # No action data
                action_shape = dataset.features["action"]["shape"]
                if isinstance(action_shape, tuple) and len(action_shape) == 1:
                    frame["action"] = torch.zeros(action_shape[0], dtype=torch.float32)
            
            # Add images
            for cam_name, img_array in processed_frame.get('images', {}).items():
                frame[f"observation.images.{cam_name}"] = img_array
            
            # Add frame to dataset
            dataset.add_frame(frame)
            frames_added += 1
            
            # Clean up processed frame data to release memory
            frames[i] = None
            del processed_frame
            
            # Print progress periodically
            if frames_added % 100 == 0:
                print(f"    Processed {frames_added}/{total_frames} frames...")
        
        # Clean up entire frames list
        del frames
        
        # Release video resources
        if storage_format == 'video' and video_captures:
            for cam_name, cap in video_captures.items():
                cap.release()
            print(f"  ✓ Released video resources ({len(video_captures)} files)")
        
        del episode_data
        
        # Save episode
        dataset.save_episode()
        if future_actions is not None:
            print(f"  ✓ Episode {ep_idx} saved to LeRobot dataset ({frames_added}/{total_frames} frames, last frame discarded)")
        else:
            print(f"  ✓ Episode {ep_idx} saved to LeRobot dataset ({frames_added} frames)")
    
    print(f"\n✓ Conversion completed!")
    print(f"  LeRobot dataset saved in: {output_dir}")
    print(f"  Total episodes: {len(selected_episodes)}")
    print(f"\nUsage:")
    print(f"  from lerobot.common.datasets.lerobot_dataset import LeRobotDataset")
    print(f"  dataset = LeRobotDataset('{args.repo_id}', root='{output_dir}')")


def main():
    args = parse_args()
    
    try:
        result = convert_dataset(args)
        if result is not None and result != 0:
            return result
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())

