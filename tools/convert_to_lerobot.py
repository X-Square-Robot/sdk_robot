"""
将采集的JSON数据转换为LeRobot数据集格式

支持最新的数据采集格式，包括：
- 关节状态 (joint_states)
- 多模式动作数据 (action): 支持master_arm_control, vr_control, sdk_control等模式
- 图像数据 (images)
- 额外传感器数据: odometry, chassis_imu, pose, tactile等

依赖:
    pip install lerobot torch numpy pillow

使用方法:
    python convert_to_lerobot.py \
        --input-dir ./collected_data \
        --output-dir ./lerobot_data \
        --repo-id "my_robot/dataset" \
        --robot-type "quanta_x1"

可选参数:
    --fps: 目标帧率 (默认: 30)
    --resize-width: 图像宽度 (默认: 640)
    --resize-height: 图像高度 (默认: 480)
    --delta-action: 使用delta action (默认: False)
    --relative-start: 相对于起始点 (默认: False)
    --select-joints: 选择特定关节 (例如: "left_arm_joint1,left_arm_joint2,...")
    --episodes: 选择特定episodes (例如: "0,1,2" 或 "0-5")
    --include-sensors: 包含额外传感器数据 (例如: "odometry,chassis_imu,left_arm_end_pose")

重要说明:
    - Action数据格式: 新的采集系统支持多种控制模式，每种模式产生不同的action向量
      脚本会自动检测并组合所有可用的控制命令

    - task字段: 在LeRobot中，task是一个特殊字段，不应在features字典中定义
      每个frame必须包含task字段（字符串类型），但features中不需要定义它
      LeRobot会在add_frame时自动处理task，将其存储在episode_buffer中

    - 传感器数据: 新的传感器数据会被保存为额外的observation字段
      例如: observation.odometry, observation.left_arm_end_pose等
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

# 不同机型的数据配置（包含action和state的提取配置）
# 注意：state_order和action_order应该保持一致，确保state和action的维度匹配
# 如需支持其他机型，需要：
# 1. 在此字典中添加对应机型的配置
# 2. 确保state_order和action_order的顺序和维度一致
ROBOT_DATA_CONFIG = {
    "quanta_x1": {
        "action_order": [
            "master_left_ee_cartesian_pos",  # 3维 - 主左臂末端执行器位置
            "master_left_ee_rotation",        # 3维 - 主左臂末端执行器旋转角度（四元数转欧拉角）
            "follow_left_gripper",            # 1维 - 从动左夹爪位置
            "master_right_ee_cartesian_pos",  # 3维 - 主右臂末端执行器位置
            "master_right_ee_rotation",       # 3维 - 主右臂末端执行器旋转角度（四元数转欧拉角）
            "follow_right_gripper",          # 1维 - 从动右夹爪位置
            "head_rotation",                  # 2维 - 头部电机角度 (pitch, yaw)
            "height",                         # 1维 - 腰部升降高度
            "velocity_decomposed_odom",       # 3维 - 底盘里程计信息 (x, y, yaw)
        ],
        "state_order": [
            "master_left_ee_cartesian_pos",  # 3维 - 主左臂末端执行器位置（从master_left_arm_end_pose提取）
            "master_left_ee_rotation",        # 3维 - 主左臂末端执行器旋转角度（从master_left_arm_end_pose提取）
            "follow_left_gripper",            # 1维 - 从动左夹爪位置（从left_gripper_joint_states提取）
            "master_right_ee_cartesian_pos",  # 3维 - 主右臂末端执行器位置（从master_right_arm_end_pose提取）
            "master_right_ee_rotation",       # 3维 - 主右臂末端执行器旋转角度（从master_right_arm_end_pose提取）
            "follow_right_gripper",          # 1维 - 从动右夹爪位置（从right_gripper_joint_states提取）
            "head_rotation",                  # 2维 - 头部电机角度（从head_joint_states提取）
            "height",                         # 1维 - 腰部升降高度（从lift_joint_states提取）
            "velocity_decomposed_odom",       # 3维 - 底盘里程计信息（从odometry提取 x, y, yaw）
        ],
        # 定义每个state字段的数据源映射
        "state_source_mapping": {
            "master_left_ee_cartesian_pos": {
                "source": "master_left_arm_end_pose",
                "type": "position",  # position, orientation, joint_positions
                "dim": 3
            },
            "master_left_ee_rotation": {
                "source": "master_left_arm_end_pose",
                "type": "orientation_euler",  # 四元数转欧拉角
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0  # 取第一个关节
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
                "index": [0, 1]  # 取前两个关节
            },
            "height": {
                "source": "lift_joint_states",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "velocity_decomposed_odom": {
                "source": "odometry",
                "type": "odom_xy_yaw",  # 提取x, y, yaw
                "dim": 3
            }
        },
        # 定义每个action字段的数据源映射
        "action_source_mapping": {
            "master_left_ee_cartesian_pos": {
                "source": "master_left_arm_end_pose_action",
                "type": "position",  # position, orientation_euler, joint_positions, odom_xy_yaw
                "dim": 3
            },
            "master_left_ee_rotation": {
                "source": "master_left_arm_end_pose_action",
                "type": "orientation_euler",  # 四元数转欧拉角
                "dim": 3
            },
            "follow_left_gripper": {
                "source": "left_gripper_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0  # 取第一个关节
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
                "index": [0, 1]  # 取前两个关节
            },
            "height": {
                "source": "lift_actions",
                "type": "joint_positions",
                "dim": 1,
                "index": 0
            },
            "velocity_decomposed_odom": {
                "source": "odometry_action",
                "type": "odom_xy_yaw",  # 提取x, y, yaw
                "dim": 3
            }
        }
    },
    # 可以在这里添加其他机型的配置
    # "other_model": {
    #     "action_order": [...],  # action字段的顺序列表
    #     "state_order": [...],   # state字段的顺序列表（应与action_order一致）
    #     "state_source_mapping": {...},  # state字段的数据源映射
    #     "action_source_mapping": {...}   # action字段的数据源映射
    # }
}

# 默认使用quanta_x1的配置
DEFAULT_ROBOT_TYPE = "quanta_x1"


def get_robot_config(robot_type: str) -> Dict[str, Any]:
    """
    获取指定机器人类型的配置（包含action和state的提取配置）
    
    Args:
        robot_type: 机器人类型字符串
        
    Returns:
        机器人配置字典，如果不存在则返回默认配置
    """
    # 尝试匹配robot_type（支持部分匹配）
    robot_type_lower = robot_type.lower()
    for key in ROBOT_DATA_CONFIG.keys():
        if key.lower() in robot_type_lower or robot_type_lower in key.lower():
            return ROBOT_DATA_CONFIG[key]
    
    # 如果没有匹配到，返回默认配置
    print(f"  ⚠️  警告: 未找到机器人类型 '{robot_type}' 的配置，使用默认配置 (quanta_x1)")
    return ROBOT_DATA_CONFIG[DEFAULT_ROBOT_TYPE]


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)，单位：弧度
    
    Args:
        x, y, z, w: 四元数的四个分量
        
    Returns:
        (roll, pitch, yaw): 欧拉角，单位：弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def extract_action_v3(frame: Dict[str, Any], robot_type: str = DEFAULT_ROBOT_TYPE) -> Optional[np.ndarray]:
    """
    从最新格式的frame中提取action，按照配置的顺序和维度动态提取
    
    Args:
        frame: 帧数据字典
        robot_type: 机器人类型，用于获取配置
        
    Returns:
        action向量或 None
    """
    # 获取机器人配置
    config = get_robot_config(robot_type)
    action_order = config.get("action_order", [])
    action_source_mapping = config.get("action_source_mapping", {})
    
    if 'action' not in frame:
        return None
    
    action_dict = frame['action']
    action_parts = []
    
    # 按照action_order的顺序提取action
    for action_field in action_order:
        if action_field not in action_source_mapping:
            # 如果没有配置，使用默认值填充
            dim = 0
            if action_field == "master_left_ee_cartesian_pos" or action_field == "master_right_ee_cartesian_pos":
                dim = 3
            elif action_field == "master_left_ee_rotation" or action_field == "master_right_ee_rotation":
                dim = 3
            elif action_field == "follow_left_gripper" or action_field == "follow_right_gripper" or action_field == "height":
                dim = 1
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
        
        # 根据source_type提取数据
        if source_type == "position":
            # 从end_pose_action提取位置
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
            # 从end_pose_action提取旋转（四元数转欧拉角）
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
        
        elif source_type == "joint_positions":
            # 从actions提取关节位置
            if source_name in action_dict:
                actions = action_dict[source_name]
                positions = actions.get('positions', [])
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        # 多个索引
                        for idx in index:
                            if idx < len(positions):
                                action_parts.append(positions[idx])
                            else:
                                action_parts.append(0.0)
                    else:
                        # 单个索引
                        if index < len(positions):
                            action_parts.append(positions[index])
                        else:
                            action_parts.append(0.0)
                else:
                    # 没有指定索引，使用所有位置
                    action_parts.extend(positions[:dim] if len(positions) >= dim else positions + [0.0] * (dim - len(positions)))
            else:
                # 数据不存在，填充零
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        action_parts.extend([0.0] * len(index))
                    else:
                        action_parts.append(0.0)
                else:
                    action_parts.extend([0.0] * dim)
        
        elif source_type == "odom_xy_yaw":
            # 从odometry_action提取x, y, yaw
            if source_name in action_dict:
                odom = action_dict[source_name]
                pose = odom.get('pose', {})
                pos = pose.get('position', {})
                ori = pose.get('orientation', {})
                # 位置 x, y
                action_parts.extend([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0)
                ])
                # 四元数转yaw角度
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
            # 未知类型，填充零
            action_parts.extend([0.0] * dim)
    
    return np.array(action_parts, dtype=np.float32)


def extract_joint_names_from_v3_episode(episode_data: Dict[str, Any]) -> List[str]:
    """
    从V3格式的episode中提取关节名称列表
    
    Args:
        episode_data: episode数据字典，包含joint_names字段
        
    Returns:
        展平后的关节名称列表，顺序与extract_state_v3中的提取顺序一致
    """
    joint_names_dict = episode_data.get('joint_names', {})
    if not isinstance(joint_names_dict, dict):
        return []
    
    joint_names_list = []
    
    # 按照extract_state_v3中的顺序提取关节名称
    # left_arm (如果存在)
    if 'left_arm' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['left_arm'])
    
    # right_arm
    if 'right_arm' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['right_arm'])
    
    # lift
    if 'lift' in joint_names_dict:
        joint_names_list.extend(joint_names_dict['lift'])
    
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
    从最新格式的frame中提取state，按照配置的顺序和维度提取，与action保持一致
    
    从observation中按照state_order配置的顺序提取：
    1. master_left_ee_cartesian_pos: 主左臂末端执行器位置
    2. master_left_ee_rotation: 主左臂末端执行器旋转角度
    3. follow_left_gripper: 从动左夹爪位置
    4. master_right_ee_cartesian_pos: 主右臂末端执行器位置
    5. master_right_ee_rotation: 主右臂末端执行器旋转角度
    6. follow_right_gripper: 从动右夹爪位置
    7. head_rotation: 头部电机角度
    8. height: 腰部升降高度
    9. velocity_decomposed_odom: 底盘里程计信息
    
    Args:
        frame: 帧数据字典
        joint_indices: 要选择的关节索引（如果为None，则选择所有关节，注意：此参数在当前实现中不再使用，因为state不再包含原始关节数据）
        robot_type: 机器人类型，用于获取配置
        
    Returns:
        state向量或None
    """
    if 'observation' not in frame:
        return None
    
    obs = frame['observation']
    
    # 获取机器人配置
    config = get_robot_config(robot_type)
    state_order = config.get("state_order", config.get("action_order", []))
    state_source_mapping = config.get("state_source_mapping", {})
    
    state_parts = []
    
    # 按照state_order的顺序提取state
    for state_field in state_order:
        if state_field not in state_source_mapping:
            # 如果没有配置，使用默认值填充
            dim = 0
            if state_field == "master_left_ee_cartesian_pos" or state_field == "master_right_ee_cartesian_pos":
                dim = 3
            elif state_field == "master_left_ee_rotation" or state_field == "master_right_ee_rotation":
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
        
        # 根据source_type提取数据
        if source_type == "position":
            # 从end_pose提取位置
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
            # 从end_pose提取旋转（四元数转欧拉角）
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
        
        elif source_type == "joint_positions":
            # 从joint_states提取关节位置
            if source_name in obs:
                joint_states = obs[source_name]
                positions = joint_states.get('positions', [])
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        # 多个索引
                        for idx in index:
                            if idx < len(positions):
                                state_parts.append(positions[idx])
                            else:
                                state_parts.append(0.0)
                    else:
                        # 单个索引
                        if index < len(positions):
                            state_parts.append(positions[index])
                        else:
                            state_parts.append(0.0)
                else:
                    # 没有指定索引，使用所有位置
                    state_parts.extend(positions[:dim] if len(positions) >= dim else positions + [0.0] * (dim - len(positions)))
            else:
                # 数据不存在，填充零
                if "index" in source_config:
                    index = source_config["index"]
                    if isinstance(index, list):
                        state_parts.extend([0.0] * len(index))
                    else:
                        state_parts.append(0.0)
                else:
                    state_parts.extend([0.0] * dim)
        
        elif source_type == "odom_xy_yaw":
            # 从odometry提取x, y, yaw
            if source_name in obs:
                odom = obs[source_name]
                pose = odom.get('pose', {})
                pos = pose.get('position', {})
                ori = pose.get('orientation', {})
                # 位置 x, y
                state_parts.extend([
                    pos.get('x', 0.0),
                    pos.get('y', 0.0)
                ])
                # 四元数转yaw角度
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
            # 未知类型，填充零
            state_parts.extend([0.0] * dim)
    
    if not state_parts:
        return None
    
    return np.array(state_parts, dtype=np.float32)


def parse_args():
    parser = argparse.ArgumentParser(
        description="将采集的JSON数据转换为LeRobot数据集格式\n\n"
                    "自动检测数据格式，支持所有传感器数据。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:

  基础转换:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --robot-type "quanta_x1"

  选择特定episodes:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --episodes "0,1,2" \\
        --robot-type "quanta_x1"

  选择特定关节:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --select-joints "left_arm_joint1,left_arm_joint2,right_arm_joint1" \\
        --robot-type "quanta_x1"

  包含额外传感器数据 (IMU, 里程计等):
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --include-sensors "odometry,chassis_imu,pose,left_arm_end_pose,right_arm_end_pose" \\
        --robot-type "quanta_x1"

  使用视频格式:
    python convert_to_lerobot.py \\
        --input-dir ./collected_data \\
        --output-dir ./lerobot_data \\
        --repo-id "my_robot/dataset" \\
        --use-videos \\
        --video-backend pyav \\
        --robot-type "quanta_x1"

依赖:
    pip install lerobot torch numpy pillow

重要说明:
  - task字段: 在LeRobot中，task是一个特殊字段，不应在features字典中定义
    每个frame必须包含task字段（字符串类型），但features中不需要定义它
    LeRobot会在add_frame时自动处理task，将其存储在episode_buffer中
  
  - 传感器数据: 新的传感器数据会被保存为额外的observation字段
    例如: observation.odometry, observation.left_arm_end_pose等
        """
    )
    parser.add_argument("--input-dir", type=str, required=True, 
                       help="输入数据目录（包含dataset_metadata.json和episode_*目录）")
    parser.add_argument("--output-dir", type=str, required=True, 
                       help="输出LeRobot数据集目录")
    parser.add_argument("--repo-id", type=str, required=True, 
                       help="数据集repo ID（例如: 'my_robot/dataset'）")
    parser.add_argument("--robot-type", type=str, default="quanta_x1", 
                       help="机器人类型（默认: quanta_x1，当前支持的机型: quanta_x1, quanta_x2）")
    parser.add_argument("--fps", type=int, default=30, 
                       help="目标帧率（默认: 30）")
    parser.add_argument("--resize-width", type=int, default=640, 
                       help="图像宽度（默认: 640）")
    parser.add_argument("--resize-height", type=int, default=480, 
                       help="图像高度（默认: 480）")
    parser.add_argument("--delta-action", action="store_true", 
                       help="使用delta action（动作 = 下一状态 - 当前状态）")
    parser.add_argument("--relative-start", action="store_true", 
                       help="相对于起始点（状态和动作都减去第一帧的值）")
    parser.add_argument("--select-joints", type=str, default=None, 
                       help="选择特定关节（逗号分隔，例如: 'left_arm_joint1,left_arm_joint2'）")
    parser.add_argument("--episodes", type=str, default=None, 
                       help="选择特定episodes（例如: '0,1,2' 或 '0-5'，默认: 全部）")
    parser.add_argument("--use-videos", action="store_true", 
                       help="使用视频格式而非图像（可减少存储空间）")
    parser.add_argument("--video-backend", type=str, default=None, 
                       choices=["pyav", "opencv"], 
                       help="视频后端（仅在使用--use-videos时有效，可选: pyav, opencv）")
    parser.add_argument("--include-sensors", type=str, default=None, 
                       help="包含额外传感器数据（逗号分隔，例如: 'odometry,chassis_imu,left_arm_end_pose'）")
    parser.add_argument("--future-action-chunk-size", type=int, default=32,
                       help="使用future state作为action时的帧数（默认: 32）")
    return parser.parse_args()


def load_dataset_metadata(input_dir: Path) -> Dict[str, Any]:
    """加载数据集元数据"""
    metadata_file = input_dir / "dataset_metadata.json"
    if not metadata_file.exists():
        raise FileNotFoundError(f"找不到元数据文件: {metadata_file}")
    
    with open(metadata_file, 'r', encoding='utf-8') as f:
        return json.load(f)


def load_episode(input_dir: Path, episode_id: int) -> Dict[str, Any]:
    """加载单个episode数据 - 只加载元数据，不加载所有帧"""
    episode_dir = input_dir / f"episode_{episode_id:04d}"
    episode_json = episode_dir / "episode.json"
    
    if not episode_json.exists():
        raise FileNotFoundError(f"找不到episode文件: {episode_json}")
    
    with open(episode_json, 'r', encoding='utf-8') as f:
        episode_data = json.load(f)
    
    # 不预加载图像路径，而是记录episode目录
    episode_data['_episode_dir'] = episode_dir
    
    # 检测存储格式并加载视频文件（如果是视频格式）
    storage_format = episode_data.get('storage_format', 'images')
    episode_data['_storage_format'] = storage_format
    
    if storage_format == 'video' and 'video_files' in episode_data:
        # 打开所有视频文件
        video_captures = {}
        for cam_name, video_file in episode_data['video_files'].items():
            video_path = episode_dir / video_file
            if video_path.exists():
                cap = cv2.VideoCapture(str(video_path))
                if cap.isOpened():
                    video_captures[cam_name] = cap
                    # 获取视频帧数以验证
                    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                    print(f"  加载视频: {cam_name}.mp4 ({frame_count} 帧)")
                else:
                    print(f"  ⚠️  警告: 无法打开视频文件: {video_path}")
        
        episode_data['_video_captures'] = video_captures
    
    return episode_data


def load_episode_metadata(input_dir: Path, episode_id: int) -> Dict[str, Any]:
    """只加载episode的元数据，不加载帧数据"""
    episode_dir = input_dir / f"episode_{episode_id:04d}"
    episode_json = episode_dir / "episode.json"
    
    if not episode_json.exists():
        raise FileNotFoundError(f"找不到episode文件: {episode_json}")
    
    with open(episode_json, 'r', encoding='utf-8') as f:
        # 只读取元数据
        episode_data = json.load(f)
    
    # 返回轻量级元数据
    metadata = {
        'episode_id': episode_data.get('episode_id', episode_id),
        'task': episode_data.get('task', 'unknown'),
        'num_frames': episode_data.get('num_frames', len(episode_data.get('frames', []))),
        'duration': episode_data.get('duration', 0),
        'joint_names': episode_data.get('joint_names', []),
        '_episode_dir': episode_dir,
        '_has_frames': 'frames' in episode_data and len(episode_data['frames']) > 0
    }
    
    # 检测数据格式并获取第一帧信息
    if metadata['_has_frames']:
        first_frame = episode_data['frames'][0]
        metadata['_first_frame'] = first_frame
    
    return metadata


def parse_episode_selection(episodes_str: str, total_episodes: int) -> List[int]:
    """解析episode选择字符串"""
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
    """选择特定关节的索引"""
    if select_str is None:
        return list(range(len(joint_names)))
    
    selected_names = [name.strip() for name in select_str.split(',')]
    indices = []
    
    for name in selected_names:
        if name in joint_names:
            indices.append(joint_names.index(name))
        else:
            print(f"警告: 关节 '{name}' 不存在，跳过")
    
    return indices


def _detect_data_format(episode_data: Dict[str, Any]) -> bool:
    """检测数据格式（新格式带有更多传感器数据）"""
    if not episode_data.get('frames'):
        return False
    first_frame = episode_data['frames'][0]
    # V3格式：在observation中有right_arm_joint_states等字段，action中有master_left_arm_end_pose_action等
    if 'observation' in first_frame and 'right_arm_joint_states' in first_frame['observation']:
        return True
    # V2格式：在frames中直接有joint_states字段
    if 'joint_states' in first_frame:
        return True
    # 旧格式检测
    return 'collection_config' in episode_data


def _has_action_field(episode_data: Dict[str, Any]) -> bool:
    """检测episode是否包含action字段"""
    if not episode_data.get('frames'):
        return False
    
    # 检查前几帧是否有action
    for frame in episode_data['frames'][:5]:
        if 'action' in frame:
            # V3格式：检查是否有master_left_arm_end_pose_action等字段
            if isinstance(frame['action'], dict):
                if 'master_left_arm_end_pose_action' in frame['action']:
                    return True
            # V2格式或其他格式
            return True
    
    return False


def _is_v3_format(frame: Dict[str, Any]) -> bool:
    """检测是否为V3格式（最新格式）"""
    if 'observation' in frame and 'right_arm_joint_states' in frame['observation']:
        if 'action' in frame and isinstance(frame['action'], dict):
            if 'master_left_arm_end_pose_action' in frame['action']:
                return True
    return False


def _extract_sensor_data(frame: Dict[str, Any], sensor_name: str) -> Optional[np.ndarray]:
    """从V2格式frame中提取传感器数据并转换为numpy数组"""
    if sensor_name not in frame:
        return None

    sensor_data = frame[sensor_name]

    # 处理不同类型的传感器数据
    if sensor_name == 'joint_states':
        # joint_states现在是一个数组: [positions, velocities, efforts]
        if isinstance(sensor_data, list) and len(sensor_data) >= 1:
            return np.array(sensor_data[0], dtype=np.float32)  # positions
        return np.array(sensor_data.get('positions', []), dtype=np.float32)

    elif sensor_name == 'action':
        # 新的action格式：包含多个命令流的字典
        if isinstance(sensor_data, dict):
            # 将所有命令流组合成一个action向量
            action_parts = []
            # 按优先级排序，确保一致的action向量顺序
            command_keys = sorted(sensor_data.keys())
            for key in command_keys:
                cmd_data = sensor_data[key]
                if isinstance(cmd_data, (list, tuple)):
                    action_parts.extend(cmd_data)
                elif isinstance(cmd_data, dict):
                    # 处理复杂命令（如位姿）
                    if 'positions' in cmd_data:
                        action_parts.extend(cmd_data['positions'])
                    elif 'position' in cmd_data and 'orientation' in cmd_data:
                        # 位姿命令
                        pos = cmd_data['position']
                        ori = cmd_data['orientation']
                        action_parts.extend([pos['x'], pos['y'], pos['z'],
                                           ori['x'], ori['y'], ori['z'], ori['w']])
                    else:
                        # 其他字典格式，展平所有数值
                        for v in cmd_data.values():
                            if isinstance(v, (int, float)):
                                action_parts.append(v)
            return np.array(action_parts, dtype=np.float32)
        # 向后兼容旧格式
        return np.array(sensor_data.get('positions', []), dtype=np.float32)

    elif sensor_name.endswith('_end_pose'):
        # 末端位姿: position(3) + orientation(4) = 7维
        pos = sensor_data.get('position', {})
        ori = sensor_data.get('orientation', {})
        return np.array([
            pos.get('x', 0), pos.get('y', 0), pos.get('z', 0),
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)
        ], dtype=np.float32)

    elif sensor_name == 'odometry':
        # 里程计: position(3) + orientation(4) + linear_vel(3) + angular_vel(3) = 13维
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
        # 定位数据: position(3) + orientation(4) = 7维
        pos = sensor_data.get('position', {})
        ori = sensor_data.get('orientation', {})
        return np.array([
            pos.get('x', 0), pos.get('y', 0), pos.get('z', 0),
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1)
        ], dtype=np.float32)

    elif sensor_name == 'chassis_imu':
        # IMU: orientation(4) + angular_vel(3) + linear_acc(3) = 10维
        ori = sensor_data.get('orientation', {})
        ang = sensor_data.get('angular_velocity', {})
        lin = sensor_data.get('linear_acceleration', {})
        return np.array([
            ori.get('x', 0), ori.get('y', 0), ori.get('z', 0), ori.get('w', 1),
            ang.get('x', 0), ang.get('y', 0), ang.get('z', 0),
            lin.get('x', 0), lin.get('y', 0), lin.get('z', 0)
        ], dtype=np.float32)

    elif sensor_name.endswith('_tactile'):
        # 触觉传感器: 返回force数组
        return np.array(sensor_data.get('force', []), dtype=np.float32)

    else:
        # 其他传感器，尝试转换为数组
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
    """处理单个frame，内存高效
    
    Args:
        camera_name_mapping: 映射frame中的相机名到metadata中的相机名
                            例如: {'head_camera': 'head_rgb_stream'}
        storage_format: 存储格式 ('images' 或 'video')
        video_captures: 视频捕获对象字典（当storage_format='video'时使用）
        override_action: 如果提供，则使用此action而不是从frame中提取
    """
    result = {}
    
    # 检测是否为V3格式（最新格式）
    is_v3_format = _is_v3_format(frame)
    
    # 处理关节状态
    if is_v3_format:
        # V3格式：从observation中提取关节状态
        state_data = extract_state_v3(frame, joint_indices, robot_type=robot_type)
        if state_data is not None:
            result['state'] = state_data
        
        # Actions (V3格式)
        if override_action is not None:
            # 使用外部提供的action
            result['action'] = override_action
        elif 'action' in frame:
            action_data = extract_action_v3(frame, robot_type=robot_type)
            if action_data is not None:
                result['action'] = action_data
            else:
                result['action'] = None
        else:
            # 如果没有action，标记为需要使用future state
            result['action'] = None
    
    elif is_new_format and 'joint_states' in frame:
        # V2格式：joint_states字段
        joint_data = _extract_sensor_data(frame, 'joint_states')
        if joint_data is not None:
            if joint_indices:
                joint_data = joint_data[joint_indices]
            result['state'] = joint_data
        
        # Actions (V2格式)
        if override_action is not None:
            # 使用外部提供的action
            result['action'] = override_action
        elif 'action' in frame:
            action_data = _extract_sensor_data(frame, 'action')
            if action_data is not None:
                result['action'] = action_data
        else:
            # 如果没有action，标记为需要使用future state
            result['action'] = None
    
    # 旧格式
    elif 'observation' in frame and 'joint_positions' in frame['observation']:
        joint_pos = np.array(frame['observation']['joint_positions'], dtype=np.float32)
        if joint_indices:
            joint_pos = joint_pos[joint_indices]
        result['state'] = joint_pos
        
        if override_action is not None:
            # 使用外部提供的action
            result['action'] = override_action
        elif 'action' in frame and 'joint_positions' in frame['action']:
            action_pos = np.array(frame['action']['joint_positions'], dtype=np.float32)
            if joint_indices:
                action_pos = action_pos[joint_indices]
            result['action'] = action_pos
        else:
            # 如果没有action字段，标记为需要使用future state
            result['action'] = None
    
    # 处理额外传感器
    if is_new_format and include_sensors:
        result['sensors'] = {}
        for sensor_name in include_sensors:
            sensor_data = _extract_sensor_data(frame, sensor_name)
            if sensor_data is not None:
                result['sensors'][sensor_name] = sensor_data
    
    # 处理图像
    result['images'] = {}
    
    if storage_format == 'video' and video_captures:
        # 从视频读取帧
        for cam_name, frame_index in frame.get('images', {}).items():
            if cam_name in video_captures:
                cap = video_captures[cam_name]
                
                # 设置视频位置到指定帧
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                
                # 读取帧
                ret, img_bgr = cap.read()
                if ret:
                    # BGR -> RGB
                    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                    
                    # 调整大小
                    img_resized = cv2.resize(
                        img_rgb, 
                        (resize_width, resize_height),
                        interpolation=cv2.INTER_LANCZOS4
                    )
                    
                    # 转换为 (C, H, W) 格式
                    img_array = img_resized.transpose(2, 0, 1)
                    
                    # 使用映射后的相机名
                    mapped_cam_name = camera_name_mapping.get(cam_name, cam_name) if camera_name_mapping else cam_name
                    result['images'][mapped_cam_name] = img_array
                else:
                    print(f"  ⚠️  警告: 无法从视频读取帧 {frame_index} ({cam_name})")
            else:
                print(f"  ⚠️  警告: 没有找到相机 {cam_name} 的视频捕获对象")
    else:
        # 从图像文件读取
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
            # 转换为 (C, H, W) 格式
            img_array = np.array(img).transpose(2, 0, 1)
            
            # 使用映射后的相机名（如果有映射的话）
            mapped_cam_name = camera_name_mapping.get(cam_name, cam_name) if camera_name_mapping else cam_name
            result['images'][mapped_cam_name] = img_array
    
    return result


def extract_all_states(
    frames: List[Dict[str, Any]],
    joint_indices: List[int],
    is_new_format: bool,
    robot_type: str = DEFAULT_ROBOT_TYPE
) -> List[Optional[np.ndarray]]:
    """提取所有帧的state数据
    
    Returns:
        state列表，每个元素是一个numpy数组或None
    """
    states = []
    for frame in frames:
        state = None
        # 检测是否为V3格式
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
    """为没有action字段的数据生成future state作为action
    
    策略：
    - 对于第i帧，action = 第(i+1)到第(i+chunk_size)帧的state
    - 对于最后chunk_size-1帧：使用剩余的future state，不足的用最后一帧补齐
    - 最后一帧：返回None（会被丢弃）
    
    Args:
        frames: 所有帧数据
        joint_indices: 关节索引
        is_new_format: 是否为新格式
        chunk_size: future state的帧数
    
    Returns:
        action列表，每个元素是一个numpy数组或None（最后一帧）
    """
    total_frames = len(frames)
    
    # 首先提取所有state
    all_states = extract_all_states(frames, joint_indices, is_new_format, robot_type=robot_type)
    
    # 检查是否有有效的state
    valid_states = [s for s in all_states if s is not None]
    if not valid_states:
        return [None] * total_frames
    
    actions = []
    
    for i in range(total_frames):
        current_state = all_states[i]
        
        if current_state is None:
            actions.append(None)
            continue
        
        # 最后一帧：返回None（会被丢弃）
        if i == total_frames - 1:
            actions.append(None)
            continue
        
        # 收集future states
        future_states = []
        for j in range(1, chunk_size + 1):
            future_idx = i + j
            if future_idx < total_frames and all_states[future_idx] is not None:
                future_states.append(all_states[future_idx])
            else:
                # 超出范围或没有state，使用最后一个有效state补齐
                if future_states:
                    # 使用已收集的最后一个future state
                    future_states.append(future_states[-1].copy())
                elif all_states[i] is not None:
                    # 没有任何future state，使用当前state
                    future_states.append(all_states[i].copy())
        
        # 拼接future states成为action
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
    """处理episode数据，自动检测格式 - 用于分析action维度"""
    frames = episode_data['frames']
    
    # 只处理前几帧来确定维度
    sample_frames = frames[:min(10, len(frames))]
    
    # 检测数据格式
    is_new_format = _detect_data_format(episode_data)
    episode_dir = episode_data.get('_episode_dir')
    
    # 提取数据
    states = []
    actions = []
    
    for frame in sample_frames:
        # 检测是否为V3格式
        is_v3_format = _is_v3_format(frame)
        
        if is_v3_format:
            # V3格式：使用extract_state_v3和extract_action_v3
            state_data = extract_state_v3(frame, joint_indices, robot_type=robot_type)
            if state_data is not None:
                states.append(state_data)
            
            # 处理action
            if 'action' in frame:
                action_data = extract_action_v3(frame, robot_type=robot_type)
                if action_data is not None:
                    actions.append(action_data)
                else:
                    # 没有action，使用state作为action
                    if state_data is not None:
                        actions.append(state_data.copy())
            else:
                # 没有action字段，使用state作为action
                if state_data is not None:
                    actions.append(state_data.copy())
        
        # V2格式: joint_states字段
        elif is_new_format and 'joint_states' in frame:
            joint_data = _extract_sensor_data(frame, 'joint_states')
            if joint_data is not None:
                if joint_indices:
                    joint_data = joint_data[joint_indices]
                states.append(joint_data)
                
                # 处理action
                if 'action' in frame:
                    action_data = _extract_sensor_data(frame, 'action')
                    if action_data is not None:
                        actions.append(action_data)
                else:
                    # 没有action，使用state作为action
                    actions.append(joint_data.copy())
        
        # 旧格式
        elif 'observation' in frame and 'joint_positions' in frame['observation']:
            joint_pos = np.array(frame['observation']['joint_positions'], dtype=np.float32)
            if joint_indices:
                joint_pos = joint_pos[joint_indices]
            states.append(joint_pos)
            
            # 处理action
            if 'action' in frame and 'joint_positions' in frame['action']:
                action_pos = np.array(frame['action']['joint_positions'], dtype=np.float32)
                if joint_indices:
                    action_pos = action_pos[joint_indices]
                actions.append(action_pos)
            else:
                # 没有action，使用observation作为action
                actions.append(joint_pos.copy())
    
    # 转换为numpy数组
    states = np.array(states) if states else np.array([]).reshape(0, 0)
    actions = np.array(actions) if actions else np.array([]).reshape(0, 0)
    
    return {
        'states': states,
        'actions': actions,
        'images': {},  # 不加载图像
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
    """创建LeRobot数据集
    
    Args:
        robot_type_for_action: 用于获取action names的机器人类型（默认使用robot_type）
        state_dim: state的实际维度（如果提供，将使用此值而不是从state_names计算）
        state_names: state的完整名称列表（包含关节 + 末端位姿 + 里程计等）
    """

    features = {}

    # State特征 - 使用提供的state_dim和state_names
    if state_names and len(state_names) > 0:
        actual_state_dim = state_dim if state_dim is not None else len(state_names)
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (actual_state_dim,),
            "names": state_names
        }
    elif joint_names and len(joint_names) > 0:
        # 如果没有提供state_names，只使用joint_names（向后兼容）
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (len(joint_names),),
            "names": joint_names
        }

    # Action特征 - 动态确定维度，并添加names
    if action_dim and action_dim > 0:
        # 获取action names
        action_names = []
        robot_type_for_names = robot_type_for_action or robot_type
        config = get_robot_config(robot_type_for_names)
        action_order = config["action_order"]
        
        # 按照action_order构建names
        for action_name in action_order:
            if action_name == "master_left_ee_cartesian_pos":
                action_names.extend(["master_left_ee_pos_x", "master_left_ee_pos_y", "master_left_ee_pos_z"])
            elif action_name == "master_left_ee_rotation":
                action_names.extend(["master_left_ee_roll", "master_left_ee_pitch", "master_left_ee_yaw"])
            elif action_name == "follow_left_gripper":
                action_names.append("follow_left_gripper")
            elif action_name == "master_right_ee_cartesian_pos":
                action_names.extend(["master_right_ee_pos_x", "master_right_ee_pos_y", "master_right_ee_pos_z"])
            elif action_name == "master_right_ee_rotation":
                action_names.extend(["master_right_ee_roll", "master_right_ee_pitch", "master_right_ee_yaw"])
            elif action_name == "follow_right_gripper":
                action_names.append("follow_right_gripper")
            elif action_name == "head_rotation":
                action_names.extend(["head_pitch", "head_yaw"])
            elif action_name == "height":
                action_names.append("height")
            elif action_name == "velocity_decomposed_odom":
                action_names.extend(["odom_x", "odom_y", "odom_yaw"])
        
        # 确保names数量与action_dim匹配
        if len(action_names) != action_dim:
            print(f"  ⚠️  警告: action names数量({len(action_names)})与action维度({action_dim})不匹配")
            # 如果names不足，用占位符补齐
            while len(action_names) < action_dim:
                action_names.append(f"action_{len(action_names)}")
            # 如果names过多，截断
            action_names = action_names[:action_dim]
        
        features["action"] = {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": action_names
        }
    else:
        # 默认action维度（用于兼容性）
        features["action"] = {
            "dtype": "float32",
            "shape": (len(joint_names),) if joint_names else (1,),
            "names": joint_names if joint_names else None
        }

    # 添加相机特征
    for cam in camera_names:
        features[f"observation.images.{cam}"] = {
            "dtype": "video" if use_videos else "image",
            "shape": (3, resize_height, resize_width),
            "names": ["channels", "height", "width"],
        }

    # 注意：task不应该在features中定义！
    # task是LeRobot的特殊字段，会在add_frame时从frame中pop出来
    # 每个frame必须包含task字段，但features字典中不需要定义它

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
    """转换数据集"""
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    
    print(f"正在转换数据集...")
    print(f"  输入目录: {input_dir}")
    print(f"  输出目录: {output_dir}")
    
    # 加载元数据
    metadata = load_dataset_metadata(input_dir)
    print(f"\n数据集信息:")
    print(f"  机器人类型: {metadata['robot_type']}")
    print(f"  原始FPS: {metadata['fps']}")
    print(f"  关节数: {len(metadata.get('joint_names', []))}")
    print(f"  相机数: {len(metadata['camera_names'])}")
    print(f"  Episodes: {len(metadata['episodes'])}")
    
    # 选择episodes（先解析，用于优先从选中的episodes中提取joint_names）
    selected_episodes = parse_episode_selection(args.episodes, len(metadata['episodes']))
    
    # 选择关节
    # 优先从episode中提取完整的joint_names，而不是依赖metadata（metadata可能不完整）
    joint_names = []
    
    try:
        # 优先从用户选择的episodes中提取joint_names，如果没有选择则从所有episodes中提取
        episode_indices_to_check = selected_episodes[:3] if selected_episodes else list(range(min(3, len(metadata['episodes']))))
        print(f"  检查episodes以提取关节名称: {episode_indices_to_check}")
        
        for ep_idx in episode_indices_to_check:
            try:
                print(f"    正在检查episode {ep_idx}...")
                full_episode = load_episode(input_dir, ep_idx)
                if 'joint_names' in full_episode:
                    joint_names_dict = full_episode.get('joint_names', {})
                    print(f"      episode {ep_idx}的joint_names结构: {list(joint_names_dict.keys())}")
                    extracted_names = extract_joint_names_from_v3_episode(full_episode)
                    if extracted_names:
                        joint_names = extracted_names
                        print(f"  ✓ 从episode {ep_idx}提取到关节名称: {len(joint_names)}个关节")
                        print(f"    关节列表: {joint_names}")
                        break
                    else:
                        print(f"      ⚠️  episode {ep_idx}提取到的关节名称为空")
                else:
                    print(f"      ⚠️  episode {ep_idx}没有joint_names字段")
            except Exception as e:
                print(f"  ⚠️  警告: 无法从episode {ep_idx}提取关节名称: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # 如果从episode提取失败，尝试从metadata中提取（作为后备方案）
        if not joint_names:
            print(f"  ⚠️  警告: 无法从episode提取关节名称，尝试从metadata提取")
            metadata_joint_names = metadata.get('joint_names', [])
            if isinstance(metadata_joint_names, dict):
                extracted_names = extract_joint_names_from_v3_episode({'joint_names': metadata_joint_names})
                if extracted_names:
                    joint_names = extracted_names
                    print(f"  从metadata提取到关节名称: {len(joint_names)}个关节")
                    print(f"    关节列表: {joint_names}")
            elif isinstance(metadata_joint_names, list) and len(metadata_joint_names) > 0:
                joint_names = metadata_joint_names
                print(f"  从metadata提取到关节名称: {len(joint_names)}个关节")
                print(f"    关节列表: {joint_names}")
                print(f"  ⚠️  警告: metadata中的关节名称可能不完整，建议检查episode数据")
    except Exception as e:
        print(f"  ⚠️  警告: 无法提取关节名称: {e}")
        import traceback
        traceback.print_exc()
    
    # 确保joint_names是列表
    if not isinstance(joint_names, list):
        joint_names = []
    
    # 验证joint_names是否合理（至少应该有多个关节，如果只有1-2个可能是错误的）
    if len(joint_names) <= 2:
        print(f"  ⚠️  警告: 提取到的关节数量较少 ({len(joint_names)}个)，可能不完整")
        print(f"    如果数据中包含左右臂，应该有更多关节")
    
    joint_indices = select_joints(joint_names, args.select_joints)
    selected_joint_names = [joint_names[i] for i in joint_indices] if joint_indices else joint_names
    
    # 检测实际的相机名称（优先使用选定的episode，如果没有指定则使用前3个）
    actual_camera_names = metadata['camera_names']  # 默认使用元数据中的
    try:
        # 优先使用选定的episode，如果没有指定则使用前3个
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
                        print(f"\n注意: 检测到实际相机名称与元数据不同")
                        print(f"  元数据中: {metadata['camera_names']}")
                        print(f"  实际数据: {actual_camera_names}")
                    break
    except Exception as e:
        print(f"  ⚠️  警告: 无法自动检测相机名称，使用元数据: {e}")
    
    print(f"\n转换配置:")
    print(f"  选择的关节数: {len(selected_joint_names)}")
    if len(selected_joint_names) == 0:
        print(f"  ⚠️  警告: 没有关节数据，将只转换图像数据")
        print(f"  提示: 这可能是因为采集时设置了 enable_joint_states=False")
    elif args.select_joints:
        print(f"  关节列表: {selected_joint_names}")
    print(f"  相机: {actual_camera_names}")
    print(f"  目标FPS: {args.fps}")
    print(f"  图像大小: {args.resize_width}x{args.resize_height}")
    print(f"  Delta action: {args.delta_action}")
    print(f"  Relative start: {args.relative_start}")
    print(f"  使用视频: {args.use_videos}")
    print(f"  选择的episodes: {selected_episodes}")
    
    # 分析所有选中的episodes来确定action维度
    action_dim = None
    print(f"\n分析episodes以确定数据维度...")

    for ep_idx in selected_episodes[:3]:  # 检查前3个episodes
        if ep_idx >= len(metadata['episodes']):
            continue

        try:
            # 使用轻量级加载
            episode_data = load_episode_metadata(input_dir, ep_idx)
            
            # 只检查第一帧来确定维度
            if episode_data.get('_has_frames') and '_first_frame' in episode_data:
                first_frame = episode_data['_first_frame']
                
                # 检测格式
                is_v3_format = _is_v3_format(first_frame)
                is_new_format = 'joint_states' in first_frame or is_v3_format
                
                # 提取action维度
                if is_v3_format:
                    # V3格式：使用extract_action_v3提取action（主臂end_pose格式）
                    if 'action' in first_frame:
                        action_data = extract_action_v3(first_frame, robot_type=args.robot_type)
                        if action_data is not None:
                            current_action_dim = len(action_data)
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ 从episode {ep_idx}确定action维度(V3格式): {action_dim}")
                            elif action_dim != current_action_dim:
                                print(f"  ⚠️  警告: episode {ep_idx}的action维度({current_action_dim})与之前({action_dim})不一致")
                                action_dim = max(action_dim, current_action_dim)
                                print(f"  ✓ 更新action维度为: {action_dim}")
                            break
                    else:
                        # 没有action字段，使用future state作为action
                        state_data = extract_state_v3(first_frame, joint_indices, robot_type=args.robot_type)
                        if state_data is not None:
                            # action维度 = state维度 × chunk_size
                            current_action_dim = len(state_data) * args.future_action_chunk_size
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ 从episode {ep_idx}确定action维度(V3格式，使用future state {args.future_action_chunk_size}帧): {action_dim}")
                            break
                
                elif is_new_format and 'joint_states' in first_frame:
                    # V2格式：优先使用action，如果没有则使用joint_states
                    if 'action' in first_frame:
                        action_data = _extract_sensor_data(first_frame, 'action')
                        if action_data is not None:
                            current_action_dim = len(action_data)
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ 从episode {ep_idx}确定action维度: {action_dim}")
                            elif action_dim != current_action_dim:
                                print(f"  ⚠️  警告: episode {ep_idx}的action维度({current_action_dim})与之前({action_dim})不一致")
                                action_dim = max(action_dim, current_action_dim)
                                print(f"  ✓ 更新action维度为: {action_dim}")
                            break
                    else:
                        # 没有action字段，使用future state作为action
                        joint_data = _extract_sensor_data(first_frame, 'joint_states')
                        if joint_data is not None:
                            if joint_indices:
                                joint_data = joint_data[joint_indices]
                            # action维度 = state维度 × chunk_size
                            current_action_dim = len(joint_data) * args.future_action_chunk_size
                            if action_dim is None:
                                action_dim = current_action_dim
                                print(f"  ✓ 从episode {ep_idx}确定action维度(使用future state {args.future_action_chunk_size}帧): {action_dim}")
                            break
                elif 'observation' in first_frame and 'joint_positions' in first_frame['observation']:
                    # 旧格式：优先使用action，如果没有则使用observation
                    if 'action' in first_frame and 'joint_positions' in first_frame['action']:
                        action_pos = first_frame['action']['joint_positions']
                        if joint_indices:
                            current_action_dim = len(joint_indices)
                        else:
                            current_action_dim = len(action_pos)
                        
                        if action_dim is None:
                            action_dim = current_action_dim
                            print(f"  ✓ 从episode {ep_idx}确定action维度: {action_dim}")
                        break
                    else:
                        # 没有action字段，使用future state作为action
                        joint_pos = first_frame['observation']['joint_positions']
                        if joint_indices:
                            state_dim = len(joint_indices)
                        else:
                            state_dim = len(joint_pos)
                        
                        # action维度 = state维度 × chunk_size
                        current_action_dim = state_dim * args.future_action_chunk_size
                        if action_dim is None:
                            action_dim = current_action_dim
                            print(f"  ✓ 从episode {ep_idx}确定action维度(使用future state {args.future_action_chunk_size}帧): {action_dim}")
                        break

        except Exception as e:
            print(f"  ⚠️  分析episode {ep_idx}时出错: {e}")
            import traceback
            traceback.print_exc()
            continue

    if action_dim is None:
        # V3格式的默认action维度是20
        print(f"  ⚠️  没有找到有效的action数据，尝试使用V3格式默认维度: 20")
        action_dim = 20

    # 检测state的实际维度和names（从第一个episode的第一帧）
    state_dim_detected = None
    state_names_list = []
    try:
        for ep_idx in selected_episodes[:1]:  # 只检查第一个选中的episode
            test_episode = load_episode_metadata(input_dir, ep_idx)
            if test_episode.get('_has_frames') and '_first_frame' in test_episode:
                first_frame = test_episode['_first_frame']
                if _is_v3_format(first_frame):
                    test_state = extract_state_v3(first_frame, joint_indices, robot_type=args.robot_type)
                    if test_state is not None:
                        state_dim_detected = len(test_state)
                        print(f"  从episode {ep_idx}检测到state维度: {state_dim_detected}")
                        
                        # 构建state names，按照state_order配置的顺序，与action names保持一致
                        config = get_robot_config(args.robot_type)
                        state_order = config.get("state_order", config.get("action_order", []))
                        
                        for state_field in state_order:
                            if state_field == "master_left_ee_cartesian_pos":
                                state_names_list.extend(["master_left_ee_pos_x", "master_left_ee_pos_y", "master_left_ee_pos_z"])
                            elif state_field == "master_left_ee_rotation":
                                state_names_list.extend(["master_left_ee_roll", "master_left_ee_pitch", "master_left_ee_yaw"])
                            elif state_field == "follow_left_gripper":
                                state_names_list.append("follow_left_gripper")
                            elif state_field == "master_right_ee_cartesian_pos":
                                state_names_list.extend(["master_right_ee_pos_x", "master_right_ee_pos_y", "master_right_ee_pos_z"])
                            elif state_field == "master_right_ee_rotation":
                                state_names_list.extend(["master_right_ee_roll", "master_right_ee_pitch", "master_right_ee_yaw"])
                            elif state_field == "follow_right_gripper":
                                state_names_list.append("follow_right_gripper")
                            elif state_field == "head_rotation":
                                state_names_list.extend(["head_pitch", "head_yaw"])
                            elif state_field == "height":
                                state_names_list.append("height")
                            elif state_field == "velocity_decomposed_odom":
                                state_names_list.extend(["odom_x", "odom_y", "odom_yaw"])
                        
                        print(f"    State names数量: {len(state_names_list)}")
                        print(f"    State names: {state_names_list}")
                        break
    except Exception as e:
        print(f"  ⚠️  警告: 无法检测state维度: {e}")
        import traceback
        traceback.print_exc()
    
    # 如果检测失败，使用配置的默认值
    if state_dim_detected is None or not state_names_list:
        # 使用配置生成默认state names
        config = get_robot_config(args.robot_type)
        state_order = config.get("state_order", config.get("action_order", []))
        state_names_list = []
        for state_field in state_order:
            if state_field == "master_left_ee_cartesian_pos":
                state_names_list.extend(["master_left_ee_pos_x", "master_left_ee_pos_y", "master_left_ee_pos_z"])
            elif state_field == "master_left_ee_rotation":
                state_names_list.extend(["master_left_ee_roll", "master_left_ee_pitch", "master_left_ee_yaw"])
            elif state_field == "follow_left_gripper":
                state_names_list.append("follow_left_gripper")
            elif state_field == "master_right_ee_cartesian_pos":
                state_names_list.extend(["master_right_ee_pos_x", "master_right_ee_pos_y", "master_right_ee_pos_z"])
            elif state_field == "master_right_ee_rotation":
                state_names_list.extend(["master_right_ee_roll", "master_right_ee_pitch", "master_right_ee_yaw"])
            elif state_field == "follow_right_gripper":
                state_names_list.append("follow_right_gripper")
            elif state_field == "head_rotation":
                state_names_list.extend(["head_pitch", "head_yaw"])
            elif state_field == "height":
                state_names_list.append("height")
            elif state_field == "velocity_decomposed_odom":
                state_names_list.extend(["odom_x", "odom_y", "odom_yaw"])
        state_dim_detected = len(state_names_list)
        print(f"  ⚠️  警告: 无法检测state维度，使用配置默认值: {state_dim_detected}")
    
    # 创建LeRobot数据集
    print(f"\n创建LeRobot数据集...")
    dataset = create_lerobot_dataset(
        output_dir=str(output_dir),
        repo_id=args.repo_id,
        robot_type=args.robot_type,
        fps=args.fps,
        joint_names=selected_joint_names,
        camera_names=actual_camera_names,  # 使用实际检测到的相机名称
        resize_width=args.resize_width,
        resize_height=args.resize_height,
        use_videos=args.use_videos,
        video_backend=args.video_backend,
        action_dim=action_dim,
        robot_type_for_action=args.robot_type,
        state_dim=state_dim_detected,
        state_names=state_names_list
    )
    
    # 转换每个episode
    print(f"\n开始转换episodes...")
    for ep_idx in selected_episodes:
        if ep_idx >= len(metadata['episodes']):
            print(f"警告: Episode {ep_idx} 不存在，跳过")
            continue
        
        ep_meta = metadata['episodes'][ep_idx]
        print(f"\n处理 Episode {ep_idx}: {ep_meta['task']}")
        print(f"  帧数: {ep_meta['num_frames']}")
        print(f"  时长: {ep_meta['duration']:.2f}s")
        
        # 加载episode数据
        episode_data = load_episode(input_dir, ep_idx)
        frames = episode_data['frames']
        episode_dir = episode_data['_episode_dir']
        
        if not frames:
            print(f"  ⚠️  警告: Episode {ep_idx} 没有数据，跳过")
            continue
        
        # 检测数据格式
        is_new_format = _detect_data_format(episode_data)
        
        # 检测是否为V3格式
        is_v3_format = False
        if episode_data.get('frames'):
            is_v3_format = _is_v3_format(episode_data['frames'][0])
        
        # 获取存储格式和视频捕获对象
        storage_format = episode_data.get('_storage_format', 'images')
        video_captures = episode_data.get('_video_captures', {})
        
        if storage_format == 'video':
            print(f"  存储格式: MP4视频 ({len(video_captures)} 个文件)")
        else:
            print(f"  存储格式: JPG图像")
        
        # task字符串
        task_str = ep_meta['task']
        
        # 检测是否有action字段
        has_action = _has_action_field(episode_data)
        
        # 如果没有action字段，生成future state actions
        future_actions = None
        if not has_action:
            print(f"  未检测到action字段，使用future state (chunk_size={args.future_action_chunk_size}) 作为action")
            future_actions = generate_future_actions(
                frames,
                joint_indices,
                is_new_format,
                args.future_action_chunk_size,
                robot_type=args.robot_type
            )
            print(f"  ✓ 生成了 {len([a for a in future_actions if a is not None])} 个future actions")
        
        # 对于V3格式，如果检测到action字段，确保使用extract_action_v3
        if is_v3_format and has_action:
            print(f"  检测到V3格式，使用新的action提取方法")
        
        # 用于相对起始点的基准值
        first_state = None
        first_action = None
        
        # 逐帧处理并添加到数据集
        print(f"  开始逐帧处理...")
        total_frames = len(frames)
        frames_added = 0
        
        for i in range(total_frames):
            # 如果使用future actions且当前帧的action为None（最后一帧），跳过
            if future_actions is not None and future_actions[i] is None:
                print(f"    跳过最后一帧 (帧 {i})")
                continue
            
            # 获取当前帧并在处理后清理引用
            frame_data = frames[i]
            
            # 获取当前帧的action（如果使用future actions）
            override_action = future_actions[i] if future_actions is not None else None
            
            # 处理单个frame
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
            
            # 构建dataset frame
            frame = {"task": task_str}
            
            # 处理state（只有当dataset.features中定义了observation.state时才添加）
            if "observation.state" in dataset.features:
                if 'state' in processed_frame:
                    state = processed_frame['state']
                    
                    # 记录第一帧用于相对起始点
                    if args.relative_start and first_state is None:
                        first_state = state.copy()
                    
                    # 应用相对起始点
                    if args.relative_start and first_state is not None:
                        state = state - first_state
                    
                    frame["observation.state"] = torch.from_numpy(state).type(torch.float32)
                else:
                    # 没有state数据，填充零
                    state_shape = dataset.features["observation.state"]["shape"]
                    if isinstance(state_shape, tuple) and len(state_shape) == 1:
                        state_dim = state_shape[0]
                        frame["observation.state"] = torch.zeros(state_dim, dtype=torch.float32)
            
            # 处理action
            if 'action' in processed_frame and processed_frame['action'] is not None:
                action = processed_frame['action']
                
                # 记录第一帧用于相对起始点
                if args.relative_start and first_action is None:
                    first_action = action.copy()
                
                # 应用相对起始点
                if args.relative_start and first_action is not None:
                    action = action - first_action
                
                # Delta action处理（仅当使用future actions时禁用）
                if args.delta_action and 'state' in processed_frame and future_actions is None:
                    action = action - processed_frame['state']
                
                # 验证和调整action维度
                if action_dim is not None:
                    actual_action_dim = len(action)
                    if actual_action_dim != action_dim:
                        if actual_action_dim < action_dim:
                            # 填充
                            padding = np.zeros(action_dim - actual_action_dim, dtype=np.float32)
                            action = np.concatenate([action, padding])
                        else:
                            # 截断
                            action = action[:action_dim]
                
                frame["action"] = torch.from_numpy(action).type(torch.float32)
            else:
                # 没有action数据
                action_shape = dataset.features["action"]["shape"]
                if isinstance(action_shape, tuple) and len(action_shape) == 1:
                    frame["action"] = torch.zeros(action_shape[0], dtype=torch.float32)
            
            # 添加图像
            for cam_name, img_array in processed_frame.get('images', {}).items():
                frame[f"observation.images.{cam_name}"] = img_array
            
            # 添加frame到数据集
            dataset.add_frame(frame)
            frames_added += 1
            
            # 清理已处理的frame数据以释放内存
            frames[i] = None
            del processed_frame
            
            # 定期打印进度
            if frames_added % 100 == 0:
                print(f"    已处理 {frames_added}/{total_frames} 帧...")
        
        # 清理整个frames列表
        del frames
        
        # 释放视频资源
        if storage_format == 'video' and video_captures:
            for cam_name, cap in video_captures.items():
                cap.release()
            print(f"  ✓ 释放视频资源 ({len(video_captures)} 个)")
        
        del episode_data
        
        # 保存episode
        dataset.save_episode()
        if future_actions is not None:
            print(f"  ✓ Episode {ep_idx} 已保存到LeRobot数据集 ({frames_added}/{total_frames} 帧，丢弃了最后1帧)")
        else:
            print(f"  ✓ Episode {ep_idx} 已保存到LeRobot数据集 ({frames_added} 帧)")
    
    print(f"\n✓ 转换完成!")
    print(f"  LeRobot数据集保存在: {output_dir}")
    print(f"  总episodes: {len(selected_episodes)}")
    print(f"\n使用方法:")
    print(f"  from lerobot.common.datasets.lerobot_dataset import LeRobotDataset")
    print(f"  dataset = LeRobotDataset('{args.repo_id}', root='{output_dir}')")


def main():
    args = parse_args()
    
    try:
        convert_dataset(args)
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())

