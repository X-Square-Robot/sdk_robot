"""
传感器数据采集配置

定义所有可采集的传感器数据流及其配置选项
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class CollectionConfig:
    """传感器采集配置"""

    # 关节状态（从臂关节状态）
    slave_joint_names: Optional[List[str]] = None
    """从臂关节状态名称列表，例如: ['left_arm_joint_states', 'right_arm_joint_states']
    如果为 None 或空列表，则不采集从臂关节状态
    支持的关节状态名称：
    - left_arm_joint_states: 左臂关节状态
    - right_arm_joint_states: 右臂关节状态
    - lift_joint_states: 升降关节状态（EX001）
    - waist_joint_states: 腰部关节状态（CX002）
    - left_gripper_joint_states: 左夹爪关节状态
    - right_gripper_joint_states: 右夹爪关节状态
    - head_joint_states: 头部关节状态"""
    
    slave_action_names: Optional[List[str]] = None
    """从臂动作名称列表，例如: ['left_arm_actions', 'right_arm_actions']
    如果为 None，将根据 slave_joint_names 自动生成（将 '_joint_states' 替换为 '_actions'）
    如果为 []，则不采集动作数据"""
    
    # 图像传感器（4种固定类型）
    enable_head_rgb_stream: bool = False
    """启用头部RGB视频流"""

    enable_head_depth_stream: bool = False
    """启用头部深度视频流"""

    enable_left_arm_rgb_stream: bool = False
    """启用左臂RGB视频流"""

    enable_right_arm_rgb_stream: bool = False
    """启用右臂RGB视频流"""
    
    # 末端位姿
    enable_left_arm_end_pose: bool = False
    """启用左臂末端位姿"""
    
    enable_right_arm_end_pose: bool = False
    """启用右臂末端位姿"""

    enable_wrench_ext_world: bool = False
    """启用手腕外力"""

    enable_wrench_ext_local: bool = False
    """启用手腕本地力"""

    # 底盘传感器
    enable_odometry: bool = False
    """启用底盘里程计（odom）"""
    
    enable_pose: bool = False
    """启用机器人定位数据（tracked_pose）"""
    
    enable_chassis_imu: bool = False
    """启用底盘IMU数据"""
    
    # 深度传感器
    enable_depth_points: bool = False
    """启用底盘深度点云"""
    
    enable_head_depth_video: bool = False
    """启用头部深度视频流"""
    
    # 激光雷达
    enable_laser_scan: bool = False
    """启用激光雷达扫描"""
    
    # 触觉传感器
    enable_left_gripper_tactile: bool = False
    """启用左夹爪触觉传感器"""
    
    enable_right_gripper_tactile: bool = False
    """启用右夹爪触觉传感器"""
    
    enable_left_hand_tactile: bool = False
    """启用左灵巧手触觉传感器"""
    
    enable_right_hand_tactile: bool = False
    """启用右灵巧手触觉传感器"""
    
    # 距离传感器
    enable_tof_sensors: bool = False
    """启用ToF传感器（2个）"""
    
    enable_ultrasonic_sensors: bool = False
    """启用超声波传感器（4个）"""

    enable_master_arm_data: bool = False
    """启用主臂状态, 关节和夹爪关节状态"""

    
    def get_enabled_sensors(self) -> List[str]:
        """获取所有启用的传感器列表"""
        enabled = []
        for field_name, field_value in self.__dict__.items():
            if field_name.startswith('enable_') and field_value:
                sensor_name = field_name.replace('enable_', '')
                enabled.append(sensor_name)
        return enabled
    
    def get_camera_names(self) -> List[str]:
        """获取启用的相机名称列表"""
        cameras = []
        if self.enable_head_rgb_stream:
            cameras.append('head_rgb_stream')
        if self.enable_head_depth_stream:
            cameras.append('head_depth_stream')
        if self.enable_left_arm_rgb_stream:
            cameras.append('left_arm_rgb_stream')
        if self.enable_right_arm_rgb_stream:
            cameras.append('right_arm_rgb_stream')
        return cameras

