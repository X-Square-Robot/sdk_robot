"""
数据回放示例

这个脚本展示了如何读取采集的数据并进行回放
支持两种回放模式：
1. 关节位置回放 (joint_position)
2. 末端位姿回放 (end_pose)
"""

import json
import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Annotated, Literal

import numpy as np
import typer
from x2robot import connect, geometry_msgs
from x2robot.sdk import (
    ChassisControlMode,
    ChassisControlModeParam,
    ChassisVelocity,
    GripperPosition,
    HeadPose,
    JointPositions,
    LiftPosition,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)


def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    print("\n\n收到中断信号，正在停止...")
    sys.exit(0)


def load_episode_data(episode_path: str) -> dict:
    """加载episode数据"""
    episode_file = Path(episode_path) / "episode.json"

    if not episode_file.exists():
        raise FileNotFoundError(f"找不到文件: {episode_file}")

    print(f"正在加载数据: {episode_file}")
    with open(episode_file, "r") as f:
        data = json.load(f)

    print(f"✓ 数据加载成功")
    print(f"  - Episode ID: {data['episode_id']}")
    print(f"  - 任务: {data['task']}")
    print(f"  - 总帧数: {data['num_frames']}")
    print(f"  - 时长: {data['duration']:.2f}s")

    return data


def filter_nan_values(joint_positions: list) -> list:
    """过滤掉NaN值，保留有效的关节位置"""
    filtered = []
    for pos in joint_positions:
        if pos is None or (isinstance(pos, float) and math.isnan(pos)):
            continue
        filtered.append(pos)
    return filtered


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """将四元数转换为yaw角（绕Z轴旋转）"""
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ChassisController(threading.Thread):
    """底盘控制线程 - 高频持续控制底盘速度"""

    def __init__(self, robot, control_rate=100.0):
        """
        Args:
            robot: 机器人实例
            control_rate: 控制频率 (Hz)
        """
        super().__init__(daemon=True)
        self.robot = robot
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate

        self.target_odom = None
        self.lock = threading.Lock()
        self.running = False

        # 控制参数
        self.max_linear_vel = 0.3  # m/s
        self.max_angular_vel = 0.5  # rad/s
        self.kp_linear = 2.0
        self.kp_angular = 2.0

    def set_target_odom(self, target_odom):
        """设置目标odometry（线程安全）"""
        with self.lock:
            self.target_odom = target_odom

    def stop(self):
        """停止线程"""
        self.running = False
        # 停止底盘
        try:
            self.robot.chassis.set_velocity(
                ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0)
            )
        except:
            pass

    def run(self):
        """线程主循环 - 高频控制底盘"""
        self.running = True

        while self.running:
            try:
                loop_start = time.time()

                # 获取目标位置（线程安全）
                with self.lock:
                    target = self.target_odom

                if target is None:
                    # 没有目标，停止底盘
                    self.robot.chassis.set_velocity(
                        ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0)
                    )
                else:
                    # 获取当前位置
                    current_odom_msg = self.robot.chassis.get_odometry()

                    # 提取当前位置
                    curr_x = current_odom_msg.pose.pose.position.x
                    curr_y = current_odom_msg.pose.pose.position.y
                    curr_ori = current_odom_msg.pose.pose.orientation
                    curr_yaw = quaternion_to_yaw(
                        curr_ori.x, curr_ori.y, curr_ori.z, curr_ori.w
                    )

                    # 提取目标位置
                    target_x = target["pose"]["position"]["x"]
                    target_y = target["pose"]["position"]["y"]
                    target_ori = target["pose"]["orientation"]
                    target_yaw = quaternion_to_yaw(
                        target_ori["x"],
                        target_ori["y"],
                        target_ori["z"],
                        target_ori["w"],
                    )

                    # 计算位置差异
                    dx = target_x - curr_x
                    dy = target_y - curr_y
                    distance = math.sqrt(dx * dx + dy * dy)

                    # 计算角度差异
                    dyaw = target_yaw - curr_yaw
                    while dyaw > math.pi:
                        dyaw -= 2 * math.pi
                    while dyaw < -math.pi:
                        dyaw += 2 * math.pi

                    # 计算速度（P控制器）
                    if distance < 0.01 and abs(dyaw) < 0.05:  # 1cm, 3度
                        vel_x, vel_y, vel_yaw = 0, 0, 0
                    else:
                        angle_to_target = math.atan2(dy, dx)
                        angle_diff = angle_to_target - curr_yaw

                        while angle_diff > math.pi:
                            angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi:
                            angle_diff += 2 * math.pi

                        vel_x = self.kp_linear * distance * math.cos(angle_diff)
                        vel_y = self.kp_linear * distance * math.sin(angle_diff)
                        vel_yaw = self.kp_angular * dyaw

                        # 限制速度
                        vel_x = max(
                            -self.max_linear_vel, min(self.max_linear_vel, vel_x)
                        )
                        vel_y = max(
                            -self.max_linear_vel, min(self.max_linear_vel, vel_y)
                        )
                        vel_yaw = max(
                            -self.max_angular_vel, min(self.max_angular_vel, vel_yaw)
                        )

                    # 发送速度命令
                    chassis_vel = ChassisVelocity(
                        vel_x=vel_x, vel_y=vel_y, vel_yaw=vel_yaw
                    )
                    self.robot.chassis.set_velocity(chassis_vel)

                # 控制循环频率
                loop_time = time.time() - loop_start
                sleep_time = self.control_interval - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

            except Exception as e:
                # 出错时继续运行，避免线程崩溃
                pass


def replay_by_joint_positions(robot, episode_data: dict, playback_speed: float = 1.0):
    """按关节位置回放所有关节和底盘

    Args:
        robot: 机器人实例
        episode_data: episode数据
        playback_speed: 回放速度倍率（1.0为正常速度）

    Note:
        回放所有部件：
        - 底盘位置（使用速度控制）
        - 双臂关节（6个/臂）
        - 双手夹爪
        - 升降台 (EX001) / 腰部 (CX002)
        - 头部姿态（yaw + pitch）
    """
    print("\n" + "=" * 60)
    print("回放模式: 关节位置控制")
    print("=" * 60)

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    # 设置机器人控制模式为关节位置控制
    print("正在设置控制模式为关节位置控制...")
    mode_param = ManipulatorControlModeParam(
        mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS
    )
    result = robot.robot_control.set_manipulator_control_mode(mode_param)

    if not result.is_success:
        print(f"✗ 设置控制模式失败: {result.error_message}")
        return

    print("✓ 控制模式设置成功")

    # 设置底盘为速度控制模式
    print("正在设置底盘为速度控制模式...")
    chassis_mode = ChassisControlModeParam(mode=ChassisControlMode.VELOCITY)
    chassis_result = robot.chassis.set_control_mode(chassis_mode)

    if not chassis_result.is_success:
        print(f"警告: 底盘控制模式设置失败 - {chassis_result.error_message}")
    else:
        print("✓ 底盘速度控制模式设置成功")

    # 启动底盘控制线程（100Hz高频控制）
    print("正在启动底盘控制线程...")
    chassis_controller = ChassisController(robot)
    chassis_controller.start()
    print("✓ 底盘控制线程已启动 (50Hz)")

    frames = episode_data["frames"]
    total_frames = len(frames)

    print(f"\n开始回放 {total_frames} 帧数据...")
    print("按 Ctrl+C 停止回放\n")

    start_time = time.time()

    for i, frame in enumerate(frames):
        try:
            # 【第0步】更新底盘目标位置（线程会持续控制）
            target_odom = frame["observation"].get("odometry")
            if target_odom:
                chassis_controller.set_target_odom(target_odom)

            # 从新的数据格式中获取各部位的关节状态
            observation = frame.get("observation", {})

            # 获取左臂关节位置
            left_arm_joint_states = observation.get("left_arm_joint_states")
            if left_arm_joint_states and "positions" in left_arm_joint_states:
                left_arm_positions = filter_nan_values(
                    left_arm_joint_states["positions"]
                )
                if left_arm_positions:
                    left_joint_msg = JointPositions(positions=left_arm_positions)
                    left_result = robot.left_arm.set_joint_positions(left_joint_msg)
                    if not left_result.is_success:
                        print(f"警告: 左臂控制失败 - {left_result.error_message}")

            # 获取右臂关节位置
            right_arm_joint_states = observation.get("right_arm_joint_states")
            if right_arm_joint_states and "positions" in right_arm_joint_states:
                right_arm_positions = filter_nan_values(
                    right_arm_joint_states["positions"]
                )
                if right_arm_positions:
                    right_joint_msg = JointPositions(positions=right_arm_positions)
                    right_result = robot.right_arm.set_joint_positions(right_joint_msg)
                    if not right_result.is_success:
                        print(f"警告: 右臂控制失败 - {right_result.error_message}")

            # 获取左夹爪位置
            left_gripper_joint_states = observation.get("left_gripper_joint_states")
            if left_gripper_joint_states and "positions" in left_gripper_joint_states:
                left_gripper_positions = filter_nan_values(
                    left_gripper_joint_states["positions"]
                )
                if left_gripper_positions:
                    try:
                        robot.left_gripper.set_position(
                            GripperPosition(position=left_gripper_positions[0])
                        )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 左夹爪控制失败 - {e}")

            # 获取右夹爪位置
            right_gripper_joint_states = observation.get("right_gripper_joint_states")
            if right_gripper_joint_states and "positions" in right_gripper_joint_states:
                right_gripper_positions = filter_nan_values(
                    right_gripper_joint_states["positions"]
                )
                if right_gripper_positions:
                    try:
                        robot.right_gripper.set_position(
                            GripperPosition(position=right_gripper_positions[0])
                        )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 右夹爪控制失败 - {e}")

            # 获取升降台/腰部位置
            lift_joint_states = observation.get("lift_joint_states")
            if lift_joint_states and "positions" in lift_joint_states:
                lift_positions = filter_nan_values(lift_joint_states["positions"])
                if lift_positions:
                    try:
                        robot_model = robot.get_robot_model()
                        if robot_model == "EX001":
                            robot.lift.set_lift_position(
                                LiftPosition(position=lift_positions[0])
                            )
                        elif robot_model == "CX002":
                            robot.waist.set_joint_positions(
                                JointPositions(positions=[lift_positions[0]])
                            )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 升降台/腰部控制失败 - {e}")

            # 获取头部位置
            head_joint_states = observation.get("head_joint_states")
            if head_joint_states and "positions" in head_joint_states:
                head_positions = filter_nan_values(head_joint_states["positions"])
                if len(head_positions) >= 2:
                    try:
                        # 根据joint_names中的顺序确定yaw和pitch的顺序
                        # 如果joint_names是 ["head_pitch_joint", "head_yaw_joint"]，则顺序是 [pitch, yaw]
                        # 如果joint_names是 ["head_yaw_joint", "head_pitch_joint"]，则顺序是 [yaw, pitch]
                        # 默认假设顺序是 [pitch, yaw]
                        robot.head.set_pose(
                            HeadPose(pitch=head_positions[0], yaw=head_positions[1])
                        )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 头部控制失败 - {e}")
                elif len(head_positions) == 1:
                    # 只有一个关节的情况
                    try:
                        robot.head.set_pose(HeadPose(pitch=0, yaw=head_positions[0]))
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 头部控制失败 - {e}")

            # 计算进度
            progress = (i + 1) / total_frames * 100

            # 每10帧打印一次进度
            if i % 10 == 0:
                elapsed = time.time() - start_time
                print(
                    f"进度: {progress:.1f}% ({i+1}/{total_frames}) | "
                    f"用时: {elapsed:.2f}s",
                    end="\r",
                )

            # 控制回放速度（底盘由独立线程控制）
            if i < total_frames - 1:
                next_timestamp = frames[i + 1]["timestamp"]
                current_timestamp = frame["timestamp"]
                sleep_time = (next_timestamp - current_timestamp) / playback_speed

                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n回放被用户中断")
            break
        except Exception as e:
            print(f"\n警告: 帧 {i} 处理失败: {e}")
            continue

    # 停止底盘控制线程
    chassis_controller.stop()
    chassis_controller.join(timeout=1.0)

    elapsed = time.time() - start_time
    print(f"\n\n✓ 回放完成! 用时: {elapsed:.2f}s")


def replay_by_end_pose(robot, episode_data: dict, playback_speed: float = 1.0):
    """按末端位姿回放（包含底盘、夹爪、腰部/升降台、头部）

    Args:
        robot: 机器人实例
        episode_data: episode数据
        playback_speed: 回放速度倍率（1.0为正常速度）

    Note:
        执行顺序（重要）：
        0. 控制底盘移动到目标位置（使用速度控制）
        1. 控制腰部/升降台（调整工作空间）
        2. 控制双臂末端位姿（确保位姿可达）
        3. 控制夹爪和头部

        这个顺序很重要：必须先调整底盘和腰部/升降台位置，
        否则手臂末端位姿可能不可达。
    """
    print("\n" + "=" * 60)
    print("回放模式: 末端位姿控制")
    print("=" * 60)

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    # 设置机器人控制模式为末端位姿控制
    print("正在设置控制模式为末端位姿控制...")
    mode_param = ManipulatorControlModeParam(
        mode=ManipulatorControlMode.MANIPULATOR_END_POSE
    )
    result = robot.robot_control.set_manipulator_control_mode(mode_param)

    if not result.is_success:
        print(f"✗ 设置控制模式失败: {result.error_message}")
        return

    print("✓ 控制模式设置成功")

    # 设置底盘为速度控制模式
    print("正在设置底盘为速度控制模式...")
    chassis_mode = ChassisControlModeParam(mode=ChassisControlMode.VELOCITY)
    chassis_result = robot.chassis.set_control_mode(chassis_mode)

    if not chassis_result.is_success:
        print(f"警告: 底盘控制模式设置失败 - {chassis_result.error_message}")
    else:
        print("✓ 底盘速度控制模式设置成功")

    # 启动底盘控制线程（100Hz高频控制）
    print("正在启动底盘控制线程...")
    chassis_controller = ChassisController(robot, control_rate=100.0)
    chassis_controller.start()
    print("✓ 底盘控制线程已启动 (100Hz)")

    frames = episode_data["frames"]
    total_frames = len(frames)

    print(f"\n开始回放 {total_frames} 帧数据...")
    print("按 Ctrl+C 停止回放\n")

    start_time = time.time()
    valid_frames = 0

    for i, frame in enumerate(frames):
        try:
            # 【第0步】更新底盘目标位置（线程会持续控制）
            target_odom = frame["observation"].get("odometry")
            if target_odom:
                chassis_controller.set_target_odom(target_odom)

            # 【第1步】先控制腰部/升降台（调整工作空间，使末端位姿可达）
            observation = frame.get("observation", {})

            # 获取升降台/腰部位置
            lift_joint_states = observation.get("lift_joint_states")
            if lift_joint_states and "positions" in lift_joint_states:
                lift_positions = filter_nan_values(lift_joint_states["positions"])
                if lift_positions:
                    try:
                        robot_model = robot.get_robot_model()
                        if robot_model == "EX001":
                            robot.lift.set_lift_position(
                                LiftPosition(position=lift_positions[0])
                            )
                        elif robot_model == "CX002":
                            robot.waist.set_joint_positions(
                                JointPositions(positions=[lift_positions[0]])
                            )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 升降台/腰部控制失败 - {e}")

            # 获取末端位姿
            left_end_pose = frame["observation"].get("left_arm_end_pose")
            right_end_pose = frame["observation"].get("right_arm_end_pose")

            # 检查位姿是否有效（非零）
            def is_valid_pose(pose_data):
                if not pose_data:
                    return False
                pos = pose_data["position"]
                ori = pose_data["orientation"]
                # 检查是否全为零（无效数据）
                return not (
                    pos["x"] == 0
                    and pos["y"] == 0
                    and pos["z"] == 0
                    and ori["x"] == 0
                    and ori["y"] == 0
                    and ori["z"] == 0
                    and ori["w"] == 1
                )

            # 【第2步】发送左臂末端位姿
            if left_end_pose and is_valid_pose(left_end_pose):
                # 创建Position
                left_position = geometry_msgs.Point(
                    x=left_end_pose["position"]["x"],
                    y=left_end_pose["position"]["y"],
                    z=left_end_pose["position"]["z"],
                )

                # 创建Orientation
                left_orientation = geometry_msgs.Quaternion(
                    x=left_end_pose["orientation"]["x"],
                    y=left_end_pose["orientation"]["y"],
                    z=left_end_pose["orientation"]["z"],
                    w=left_end_pose["orientation"]["w"],
                )

                # 创建Pose
                left_pose_msg = geometry_msgs.Pose(
                    position=left_position, orientation=left_orientation
                )

                left_result = robot.left_arm.set_end_pose(left_pose_msg)

                if not left_result.is_success:
                    print(f"警告: 左臂位姿控制失败 - {left_result.error_message}")

            # 发送右臂末端位姿
            if right_end_pose and is_valid_pose(right_end_pose):
                # 创建Position
                right_position = geometry_msgs.Point(
                    x=right_end_pose["position"]["x"],
                    y=right_end_pose["position"]["y"],
                    z=right_end_pose["position"]["z"],
                )

                # 创建Orientation
                right_orientation = geometry_msgs.Quaternion(
                    x=right_end_pose["orientation"]["x"],
                    y=right_end_pose["orientation"]["y"],
                    z=right_end_pose["orientation"]["z"],
                    w=right_end_pose["orientation"]["w"],
                )

                # 创建Pose
                right_pose_msg = geometry_msgs.Pose(
                    position=right_position, orientation=right_orientation
                )

                right_result = robot.right_arm.set_end_pose(right_pose_msg)

                if not right_result.is_success:
                    print(f"警告: 右臂位姿控制失败 - {right_result.error_message}")

                valid_frames += 1

            # 【第3步】控制夹爪和头部
            # 获取左夹爪位置
            left_gripper_joint_states = observation.get("left_gripper_joint_states")
            if left_gripper_joint_states and "positions" in left_gripper_joint_states:
                left_gripper_positions = filter_nan_values(
                    left_gripper_joint_states["positions"]
                )
                if left_gripper_positions:
                    try:
                        robot.left_gripper.set_position(
                            GripperPosition(position=left_gripper_positions[0])
                        )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 左夹爪控制失败 - {e}")

            # 获取右夹爪位置
            right_gripper_joint_states = observation.get("right_gripper_joint_states")
            if right_gripper_joint_states and "positions" in right_gripper_joint_states:
                right_gripper_positions = filter_nan_values(
                    right_gripper_joint_states["positions"]
                )
                if right_gripper_positions:
                    try:
                        robot.right_gripper.set_position(
                            GripperPosition(position=right_gripper_positions[0])
                        )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 右夹爪控制失败 - {e}")

            # 获取头部位置
            head_joint_states = observation.get("head_joint_states")
            if head_joint_states and "positions" in head_joint_states:
                head_positions = filter_nan_values(head_joint_states["positions"])
                if len(head_positions) >= 2:
                    try:
                        # 根据joint_names中的顺序确定yaw和pitch的顺序
                        # 默认假设顺序是 [pitch, yaw]
                        robot.head.set_pose(
                            HeadPose(pitch=head_positions[0], yaw=head_positions[1])
                        )
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 头部控制失败 - {e}")
                elif len(head_positions) == 1:
                    try:
                        robot.head.set_pose(HeadPose(pitch=0, yaw=head_positions[0]))
                    except Exception as e:
                        if i == 0:
                            print(f"警告: 头部控制失败 - {e}")

            # 计算进度
            progress = (i + 1) / total_frames * 100

            # 每10帧打印一次进度
            if i % 10 == 0:
                elapsed = time.time() - start_time
                print(
                    f"进度: {progress:.1f}% ({i+1}/{total_frames}) | "
                    f"有效帧: {valid_frames} | 用时: {elapsed:.2f}s",
                    end="\r",
                )

            # 控制回放速度
            if i < total_frames - 1:
                next_timestamp = frames[i + 1]["timestamp"]
                current_timestamp = frame["timestamp"]
                sleep_time = (next_timestamp - current_timestamp) / playback_speed

                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n回放被用户中断")
            break
        except Exception as e:
            print(f"\n警告: 帧 {i} 处理失败: {e}")
            continue

    # 停止底盘控制线程
    chassis_controller.stop()
    chassis_controller.join(timeout=1.0)

    elapsed = time.time() - start_time
    print(f"\n\n✓ 回放完成! 用时: {elapsed:.2f}s")
    print(f"  有效帧数: {valid_frames}/{total_frames}")


def main(
    episode_dir: Annotated[
        str, typer.Argument(help="Episode目录路径，例如: ./collected_data/episode_0000")
    ],
    server: Annotated[str, typer.Option(help="机器人服务器地址")] = "localhost:50051",
    mode: Annotated[
        Literal["joint_pos", "end_pose"], typer.Option(help="回放模式")
    ] = "joint_pos",
    speed: Annotated[
        float, typer.Option(help="回放速度倍率 (例如: 1.0=正常, 0.5=慢速, 2.0=快速)")
    ] = 1.0,
):
    """
    数据回放脚本 - 自动回放所有关节和底盘

    两种回放模式（都会回放所有部件）：
    1. joint_pos: 关节位置回放（底盘+手臂+夹爪+升降台/腰部+头部）
    2. end_pose: 末端位姿回放（底盘+双臂末端位姿+夹爪+腰部/升降台+头部）

    底盘控制方式：
    - 使用独立线程以100Hz频率持续控制底盘
    - 根据当前odometry和目标odometry实时计算速度
    - 使用P控制器平滑跟踪目标位置

    示例:

    # 关节位置回放（正常速度）
    python data_replay_example.py ./collected_data/episode_0000 --mode joint_pos

    # 末端位姿回放
    python data_replay_example.py ./collected_data/episode_0000 --mode end_pose

    # 慢速回放（0.5倍速）
    python data_replay_example.py ./collected_data/episode_0000 --speed 0.5

    # 快速回放（2倍速）
    python data_replay_example.py ./collected_data/episode_0000 --speed 2.0
    """
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)

    # 加载数据
    try:
        episode_data = load_episode_data(episode_dir)
    except Exception as e:
        print(f"✗ 加载数据失败: {e}")
        sys.exit(1)

    # 连接机器人
    print(f"\n正在连接机器人 {server}...")
    try:
        robot = connect(f"x2://{server}")
        print("✓ 机器人连接成功")
        print(f"  机器人型号: {robot.get_robot_model()}")
    except Exception as e:
        print(f"✗ 连接失败: {e}")
        sys.exit(1)

    # 确认开始
    robot_model = robot.get_robot_model()
    print(f"\n回放配置:")
    print(f"  - 模式: {mode}")
    print(f"  - 速度: {speed}x")
    print(f"  - 帧数: {episode_data['num_frames']}")

    if mode == "joint_pos":
        waist_lift = "升降台" if robot_model == "EX001" else "腰部"
        print(f"  - 回放内容: 底盘 + 双臂关节 + 夹爪 + {waist_lift} + 头部")
    elif mode == "end_pose":
        waist_lift = "升降台" if robot_model == "EX001" else "腰部"
        print(f"  - 回放内容: 底盘 + 双臂末端位姿 + 夹爪 + {waist_lift} + 头部")

    input("\n按 Enter 开始回放...")

    try:
        if mode == "joint_pos":
            replay_by_joint_positions(robot, episode_data, speed)
        elif mode == "end_pose":
            replay_by_end_pose(robot, episode_data, speed)
        else:
            print(f"✗ 未知的回放模式: {mode}")
            sys.exit(1)
    except Exception as e:
        print(f"\n✗ 回放过程中出错: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)

    print("\n回放结束")


if __name__ == "__main__":
    typer.run(main)
