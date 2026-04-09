"""
Data Replay Example

This script demonstrates how to read collected data and replay it
Supports two replay modes:
1. Joint position replay
2. End pose replay
"""

import json
import time
import math
from typing import Annotated, Literal
from pathlib import Path
import typer
import signal
import sys
import threading
from x2robot import connect
from x2robot.sdk import ManipulatorControlMode, ManipulatorControlModeParam, JointPositions
from x2robot.sdk import GripperPosition, LiftPosition, HeadPose
from x2robot.sdk import ChassisVelocity, ChassisControlMode, ChassisControlModeParam
from x2robot import geometry_msgs
from x2robot.sdk import RobotModeParam, RobotWorkMode
import numpy as np


def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    print("\n\nReceived interrupt signal, stopping...")
    sys.exit(0)


def load_episode_data(episode_path: str) -> dict:
    """Load episode data"""
    episode_file = Path(episode_path) / "episode.json"
    
    if not episode_file.exists():
        raise FileNotFoundError(f"File not found: {episode_file}")
    
    print(f"Loading data: {episode_file}")
    with open(episode_file, 'r') as f:
        data = json.load(f)

    print(f"✓ Data loaded successfully")
    print(f"  - Episode ID: {data['episode_id']}")
    print(f"  - Task: {data['task']}")
    print(f"  - Total frames: {data['num_frames']}")
    print(f"  - Duration: {data['duration']:.2f}s")
    
    return data


def filter_nan_values(joint_positions: list) -> list:
    """Filter out NaN values, keep valid joint positions"""
    filtered = []
    for pos in joint_positions:
        if pos is None or (isinstance(pos, float) and math.isnan(pos)):
            continue
        filtered.append(pos)
    return filtered


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw angle (rotation around Z-axis)"""
    # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ChassisController(threading.Thread):
    """Chassis control thread - high-frequency continuous chassis velocity control"""
    
    def __init__(self, robot, control_rate=100.0):
        """
        Args:
            robot: Robot instance
            control_rate: Control frequency (Hz)
        """
        super().__init__(daemon=True)
        self.robot = robot
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        
        self.target_odom = None
        self.lock = threading.Lock()
        self.running = False
        
        # Control parameters
        self.max_linear_vel = 0.3  # m/s
        self.max_angular_vel = 0.5  # rad/s
        self.kp_linear = 2.0
        self.kp_angular = 2.0
    
    def set_target_odom(self, target_odom):
        """Set target odometry (thread-safe)"""
        with self.lock:
            self.target_odom = target_odom
    
    def stop(self):
        """Stop thread"""
        self.running = False
        # Stop chassis
        try:
            self.robot.chassis.set_velocity(ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0))
        except:
            pass
    
    def run(self):
        """Thread main loop - high-frequency chassis control"""
        self.running = True
        
        while self.running:
            try:
                loop_start = time.time()
                
                # Get target position (thread-safe)
                with self.lock:
                    target = self.target_odom

                if target is None:
                    # No target, stop chassis
                    self.robot.chassis.set_velocity(ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0))
                else:
                    # Get current position
                    current_odom_msg = self.robot.chassis.get_odometry()

                    # Extract current position
                    curr_x = current_odom_msg.pose.pose.position.x
                    curr_y = current_odom_msg.pose.pose.position.y
                    curr_ori = current_odom_msg.pose.pose.orientation
                    curr_yaw = quaternion_to_yaw(curr_ori.x, curr_ori.y, curr_ori.z, curr_ori.w)

                    # Extract target position
                    target_x = target['pose']['position']['x']
                    target_y = target['pose']['position']['y']
                    target_ori = target['pose']['orientation']
                    target_yaw = quaternion_to_yaw(target_ori['x'], target_ori['y'],
                                                   target_ori['z'], target_ori['w'])

                    # Calculate position difference
                    dx = target_x - curr_x
                    dy = target_y - curr_y
                    distance = math.sqrt(dx * dx + dy * dy)

                    # Calculate angle difference
                    dyaw = target_yaw - curr_yaw
                    while dyaw > math.pi:
                        dyaw -= 2 * math.pi
                    while dyaw < -math.pi:
                        dyaw += 2 * math.pi

                    # Calculate velocity (P controller)
                    if distance < 0.01 and abs(dyaw) < 0.05:  # 1cm, 3 degrees
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

                        # Limit velocity
                        vel_x = max(-self.max_linear_vel, min(self.max_linear_vel, vel_x))
                        vel_y = max(-self.max_linear_vel, min(self.max_linear_vel, vel_y))
                        vel_yaw = max(-self.max_angular_vel, min(self.max_angular_vel, vel_yaw))

                    # Send velocity command
                    chassis_vel = ChassisVelocity(vel_x=vel_x, vel_y=vel_y, vel_yaw=vel_yaw)
                    self.robot.chassis.set_velocity(chassis_vel)
                
                # Control loop frequency
                loop_time = time.time() - loop_start
                sleep_time = self.control_interval - loop_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                # Continue running on error to avoid thread crash
                pass


def replay_by_joint_positions(robot, episode_data: dict, playback_speed: float = 1.0):
    """Replay all joints and chassis by joint positions

    Args:
        robot: Robot instance
        episode_data: Episode data
        playback_speed: Playback speed multiplier (1.0 = normal speed)

    Note:
        Replay all components:
        - Chassis position (using velocity control)
        - Dual arm joints (6 joints per arm)
        - Both grippers
        - Lift (quanta_x1) / Waist (quanta_x2)
        - Head pose (yaw + pitch)
    """
    print("\n" + "="*60)
    print("Replay mode: Joint position control")
    print("="*60)

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    robot_model = robot.get_robot_model()

    # Set robot control mode to joint position control
    print("Setting control mode to joint position control...")
    mode_param = ManipulatorControlModeParam(
        mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS
    )
    result = robot.robot_control.set_manipulator_control_mode(mode_param)

    if not result.is_success:
        print(f"✗ Failed to set control mode: {result.error_message}")
        return

    print("✓ Control mode set successfully")

    # Desktop has no chassis
    chassis_controller = None
    if robot_model != "desktop":
        # Set chassis to velocity control mode
        print("Setting chassis to velocity control mode...")
        chassis_mode = ChassisControlModeParam(mode=ChassisControlMode.VELOCITY)
        chassis_result = robot.chassis.set_control_mode(chassis_mode)

        if not chassis_result.is_success:
            print(f"Warning: Chassis control mode setting failed - {chassis_result.error_message}")
        else:
            print("✓ Chassis velocity control mode set successfully")

        # Start chassis control thread (100Hz high-frequency control)
        print("Starting chassis control thread...")
        chassis_controller = ChassisController(robot)
        chassis_controller.start()
        print("✓ Chassis control thread started (50Hz)")
    
    frames = episode_data['frames']
    total_frames = len(frames)
    
    print(f"\nStarting replay of {total_frames} frames...")
    print("Press Ctrl+C to stop replay\n")
    
    start_time = time.time()
    
    for i, frame in enumerate(frames):
        try:
            # [Step 0] Update chassis target position (thread will continuously control)
            if chassis_controller is not None:
                target_odom = frame['observation'].get('odometry')
                if target_odom:
                    chassis_controller.set_target_odom(target_odom)

            # Get joint states of each part from the new data format
            observation = frame.get('observation', {})
            
            # Get left arm joint positions
            left_arm_joint_states = observation.get('left_arm_joint_states')
            if left_arm_joint_states and 'positions' in left_arm_joint_states:
                left_arm_positions = filter_nan_values(left_arm_joint_states['positions'])
                if left_arm_positions:
                    left_joint_msg = JointPositions(positions=left_arm_positions)
                    left_result = robot.left_arm.set_joint_positions(left_joint_msg)
                    if not left_result.is_success:
                        print(f"Warning: Left arm control failed - {left_result.error_message}")
            
            # Get right arm joint positions
            right_arm_joint_states = observation.get('right_arm_joint_states')
            if right_arm_joint_states and 'positions' in right_arm_joint_states:
                right_arm_positions = filter_nan_values(right_arm_joint_states['positions'])
                if right_arm_positions:
                    right_joint_msg = JointPositions(positions=right_arm_positions)
                    right_result = robot.right_arm.set_joint_positions(right_joint_msg)
                    if not right_result.is_success:
                        print(f"Warning: Right arm control failed - {right_result.error_message}")

            # [Step 3] Control grippers and head
            if (robot_model == "quanta_x2"):
                left_gripper_position = observation.get('left_gripper_position')
                right_gripper_position = observation.get('right_gripper_position')
                if left_gripper_position and right_gripper_position:
                    try:
                        position = left_gripper_position['position']
                        robot.left_gripper.set_position(GripperPosition(position=position))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Left gripper control failed - {e}")
                    try:
                        position = right_gripper_position['position']
                        robot.right_gripper.set_position(GripperPosition(position=position))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Right gripper control failed - {e}")
            else:
                # Get left gripper position
                left_gripper_joint_states = observation.get('left_gripper_joint_states')
                if left_gripper_joint_states and 'positions' in left_gripper_joint_states:
                    left_gripper_positions = filter_nan_values(left_gripper_joint_states['positions'])
                    if left_gripper_positions:
                        try:
                            robot.left_gripper.set_position(GripperPosition(position=left_gripper_positions[0]))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Left gripper control failed - {e}")
                
                # Get right gripper position
                right_gripper_joint_states = observation.get('right_gripper_joint_states')
                if right_gripper_joint_states and 'positions' in right_gripper_joint_states:
                    right_gripper_positions = filter_nan_values(right_gripper_joint_states['positions'])
                    if right_gripper_positions:
                        try:
                            robot.right_gripper.set_position(GripperPosition(position=right_gripper_positions[0]))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Right gripper control failed - {e}")

            # Get lift/waist position
            if robot_model == "quanta_x1":
                lift_joint_states = observation.get('lift_joint_states')
                if lift_joint_states and 'positions' in lift_joint_states:
                    lift_positions = filter_nan_values(lift_joint_states['positions'])
                    if lift_positions:
                        try:
                            robot.lift.set_lift_position(LiftPosition(position=lift_positions[0]))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Lift control failed - {e}")
            elif robot_model == "quanta_x2":
                waist_joint_states = observation.get('waist_joint_states')
                if waist_joint_states and 'positions' in waist_joint_states:
                    waist_positions = filter_nan_values(waist_joint_states['positions'])
                    if waist_positions:
                        try:
                            robot.waist.set_joint_positions(JointPositions(positions=waist_positions))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Waist control failed - {e}")
            elif robot_model == "desktop":
                pass  # Desktop has no lift/waist
            
            # Get head position
            head_joint_states = observation.get('head_joint_states')
            if head_joint_states and 'positions' in head_joint_states:
                head_positions = filter_nan_values(head_joint_states['positions'])
                if len(head_positions) >= 2:
                    try:
                        # Determine the order of yaw and pitch based on joint_names
                        # If joint_names is ["head_pitch_joint", "head_yaw_joint"], the order is [pitch, yaw]
                        # If joint_names is ["head_yaw_joint", "head_pitch_joint"], the order is [yaw, pitch]
                        # Default assumption is [pitch, yaw]
                        robot.head.set_pose(HeadPose(pitch=head_positions[0], yaw=head_positions[1]))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Head control failed - {e}")
                elif len(head_positions) == 1:
                    # Only one joint case
                    try:
                        robot.head.set_pose(HeadPose(pitch=0, yaw=head_positions[0]))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Head control failed - {e}")
            
            # Calculate progress
            progress = (i + 1) / total_frames * 100
            
            # Print progress every 10 frames
            if i % 10 == 0:
                elapsed = time.time() - start_time
                print(f"Progress: {progress:.1f}% ({i+1}/{total_frames}) | "
                      f"Time: {elapsed:.2f}s", end='\r')
            
            # Control playback speed (chassis by independent thread)
            if i < total_frames - 1:
                next_timestamp = frames[i + 1]['timestamp']
                current_timestamp = frame['timestamp']
                sleep_time = (next_timestamp - current_timestamp) / playback_speed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n\nReplay interrupted by user")
            break
        except Exception as e:
            print(f"\nWarning: Frame {i} processing failed: {e}")
            continue
    
    # Stop chassis control thread
    if chassis_controller is not None:
        chassis_controller.stop()
        chassis_controller.join(timeout=1.0)

    elapsed = time.time() - start_time
    print(f"\n\n✓ Replay completed! Time: {elapsed:.2f}s, Number of frames replayed: {i+1}")


def replay_by_end_pose(robot, episode_data: dict, playback_speed: float = 1.0):
    """Replay by end pose (including chassis, grippers, waist/lift, head)
    
    Args:
        robot: Robot instance
        episode_data: Episode data
        playback_speed: Playback speed multiplier (1.0 = normal speed)
    
    Note:
        Execution order (important):
        0. Control chassis to target position (using velocity control)
        1. Control waist/lift (adjust workspace)
        2. Control dual arm end pose (ensure pose reachable)
        3. Control grippers and head
        
        This order is important: Must first adjust chassis and waist/lift position,
        otherwise the arm end pose may not be reachable.
    """
    print("\n" + "="*60)
    print("Replay mode: End pose control")
    print("="*60)

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    robot_model = robot.get_robot_model()

    # Set robot control mode to end pose control
    print("Setting control mode to end pose control...")
    mode_param = ManipulatorControlModeParam(
        mode=ManipulatorControlMode.MANIPULATOR_END_POSE
    )
    result = robot.robot_control.set_manipulator_control_mode(mode_param)

    if not result.is_success:
        print(f"✗ Failed to set control mode: {result.error_message}")
        return

    print("✓ Control mode set successfully")

    # Desktop has no chassis
    chassis_controller = None
    if robot_model != "desktop":
        # Set chassis to velocity control mode
        print("Setting chassis to velocity control mode...")
        chassis_mode = ChassisControlModeParam(mode=ChassisControlMode.VELOCITY)
        chassis_result = robot.chassis.set_control_mode(chassis_mode)

        if not chassis_result.is_success:
            print(f"Warning: Chassis control mode setting failed - {chassis_result.error_message}")
        else:
            print("✓ Chassis velocity control mode set successfully")

        # Start chassis control thread (100Hz high frequency control)
        print("Starting chassis control thread...")
        chassis_controller = ChassisController(robot, control_rate=100.0)
        chassis_controller.start()
        print("✓ Chassis control thread started (100Hz)")
    
    frames = episode_data['frames']
    total_frames = len(frames)
    
    print(f"\nStarting replay of {total_frames} frames...")
    print("Press Ctrl+C to stop replay\n")

    # Check if pose is valid (non-zero)
    def is_valid_pose(pose_data):
        if not pose_data:
            return False
        pos = pose_data['position']
        ori = pose_data['orientation']
        # Check if all are zero (invalid data)
        return not (pos['x'] == 0 and pos['y'] == 0 and pos['z'] == 0 and
                   ori['x'] == 0 and ori['y'] == 0 and ori['z'] == 0 and ori['w'] == 1)
        
    start_time = time.time()
    valid_frames = 0
    
    for i, frame in enumerate(frames):
        try:
            # [Step 0] Update chassis target position (thread will continuously control)
            if chassis_controller is not None:
                target_odom = frame['observation'].get('odometry')
                if target_odom:
                    chassis_controller.set_target_odom(target_odom)

            # [Step 1] Control waist/lift (adjust workspace, make end pose reachable)
            observation = frame.get('observation', {})    

            # Get lift/waist position
            if robot_model == "quanta_x1":
                lift_joint_states = observation.get('lift_joint_states')
                if lift_joint_states and 'positions' in lift_joint_states:
                    lift_positions = filter_nan_values(lift_joint_states['positions'])
                    if lift_positions:
                        try:
                            robot.lift.set_lift_position(LiftPosition(position=lift_positions[0]))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Lift control failed - {e}")
            elif robot_model == "quanta_x2":
                waist_end_pose = observation.get('waist_end_pose')
                if waist_end_pose and is_valid_pose(waist_end_pose):
                    waist_position = geometry_msgs.Point(
                        x=waist_end_pose['position']['x'],
                        y=waist_end_pose['position']['y'],
                        z=waist_end_pose['position']['z']
                    )
                    waist_orientation = geometry_msgs.Quaternion(
                        x=waist_end_pose['orientation']['x'],
                        y=waist_end_pose['orientation']['y'],
                        z=waist_end_pose['orientation']['z'],
                        w=waist_end_pose['orientation']['w']
                    )
                    waist_pose_msg = geometry_msgs.Pose(
                        position=waist_position,
                        orientation=waist_orientation
                    )
                    waist_result = robot.waist.set_end_pose(waist_pose_msg)
                    if not waist_result.is_success:
                        if i == 0:
                            print(f"Warning: Waist end pose control failed - {waist_result.error_message}")
            elif robot_model == "desktop":
                pass  # Desktop has no lift/waist

            # Get end pose
            left_end_pose = frame['observation'].get('left_arm_end_pose')
            right_end_pose = frame['observation'].get('right_arm_end_pose')

            
            # [Step 2] Send left arm end pose
            if left_end_pose and is_valid_pose(left_end_pose):
                # Create Position
                left_position = geometry_msgs.Point(
                    x=left_end_pose['position']['x'],
                    y=left_end_pose['position']['y'],
                    z=left_end_pose['position']['z']
                )
                
                # Create Orientation
                left_orientation = geometry_msgs.Quaternion(
                    x=left_end_pose['orientation']['x'],
                    y=left_end_pose['orientation']['y'],
                    z=left_end_pose['orientation']['z'],
                    w=left_end_pose['orientation']['w']
                )
                
                # Create Pose
                left_pose_msg = geometry_msgs.Pose(
                    position=left_position,
                    orientation=left_orientation
                )
                
                left_result = robot.left_arm.set_end_pose(left_pose_msg)
                
                if not left_result.is_success:
                    print(f"Warning: Left arm end pose control failed - {left_result.error_message}")
            
            # Send right arm end pose
            if right_end_pose and is_valid_pose(right_end_pose):
                # Create Position
                right_position = geometry_msgs.Point(
                    x=right_end_pose['position']['x'],
                    y=right_end_pose['position']['y'],
                    z=right_end_pose['position']['z']
                )
                
                # Create Orientation
                right_orientation = geometry_msgs.Quaternion(
                    x=right_end_pose['orientation']['x'],
                    y=right_end_pose['orientation']['y'],
                    z=right_end_pose['orientation']['z'],
                    w=right_end_pose['orientation']['w']
                )
                
                # Create Pose
                right_pose_msg = geometry_msgs.Pose(
                    position=right_position,
                    orientation=right_orientation
                )
                
                right_result = robot.right_arm.set_end_pose(right_pose_msg)
                
                if not right_result.is_success:
                    print(f"Warning: Right arm end pose control failed - {right_result.error_message}")
                
                valid_frames += 1
            
            # [Step 3] Control grippers and head
            if (robot_model == "quanta_x2"):
                left_gripper_position = observation.get('left_gripper_position')
                right_gripper_position = observation.get('right_gripper_position')
                if left_gripper_position and right_gripper_position:
                    try:
                        position = left_gripper_position['position']
                        robot.left_gripper.set_position(GripperPosition(position=position))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Left gripper control failed - {e}")
                    try:
                        position = right_gripper_position['position']
                        robot.right_gripper.set_position(GripperPosition(position=position))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Right gripper control failed - {e}")
            else:
                # Get left gripper position
                left_gripper_joint_states = observation.get('left_gripper_joint_states')
                if left_gripper_joint_states and 'positions' in left_gripper_joint_states:
                    left_gripper_positions = filter_nan_values(left_gripper_joint_states['positions'])
                    if left_gripper_positions:
                        try:
                            robot.left_gripper.set_position(GripperPosition(position=left_gripper_positions[0]))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Left gripper control failed - {e}")
                
                # Get right gripper position
                right_gripper_joint_states = observation.get('right_gripper_joint_states')
                if right_gripper_joint_states and 'positions' in right_gripper_joint_states:
                    right_gripper_positions = filter_nan_values(right_gripper_joint_states['positions'])
                    if right_gripper_positions:
                        try:
                            robot.right_gripper.set_position(GripperPosition(position=right_gripper_positions[0]))
                        except Exception as e:
                            if i == 0:
                                print(f"Warning: Right gripper control failed - {e}")
                
            # Get head position
            head_joint_states = observation.get('head_joint_states')
            if head_joint_states and 'positions' in head_joint_states:
                head_positions = filter_nan_values(head_joint_states['positions'])
                if len(head_positions) >= 2:
                    try:
                        # Determine the order of yaw and pitch based on joint_names
                        # If joint_names is ["head_pitch_joint", "head_yaw_joint"], the order is [pitch, yaw]
                        # If joint_names is ["head_yaw_joint", "head_pitch_joint"], the order is [yaw, pitch]
                        # Default assumption is [pitch, yaw]
                        robot.head.set_pose(HeadPose(pitch=head_positions[0], yaw=head_positions[1]))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Head control failed - {e}")
                elif len(head_positions) == 1:
                    try:
                        robot.head.set_pose(HeadPose(pitch=0, yaw=head_positions[0]))
                    except Exception as e:
                        if i == 0:
                            print(f"Warning: Head control failed - {e}")
            
            # Calculate progress
            progress = (i + 1) / total_frames * 100
            
            # Print progress every 10 frames
            if i % 10 == 0:
                elapsed = time.time() - start_time
                print(f"Progress: {progress:.1f}% ({i+1}/{total_frames}) | "
                      f"Valid frames: {valid_frames} | Time: {elapsed:.2f}s", end='\r')
            
            # Control playback speed
            if i < total_frames - 1:
                next_timestamp = frames[i + 1]['timestamp']
                current_timestamp = frame['timestamp']
                sleep_time = (next_timestamp - current_timestamp) / playback_speed
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except KeyboardInterrupt:
            print("\n\nReplay interrupted by user")
            break
        except Exception as e:
            print(f"\nWarning: Frame {i} processing failed: {e}")
            continue
    
    # Stop chassis control thread
    if chassis_controller is not None:
        chassis_controller.stop()
        chassis_controller.join(timeout=1.0)

    elapsed = time.time() - start_time
    print(f"\n\n✓ Replay completed! Time: {elapsed:.2f}s, Number of frames replayed: {i+1}")


def main(
    episode_dir: Annotated[str, typer.Argument(help="Episode directory path, e.g., ./collected_data/episode_0000")],
    server: Annotated[str, typer.Option(help="Robot server address, e.g., localhost:50051")] = "localhost:50051",
    mode: Annotated[Literal["joint_pos", "end_pose"], typer.Option(help="Replay mode")] = "joint_pos",
    speed: Annotated[float, typer.Option(help="Replay speed multiplier (e.g., 1.0=normal, 0.5=slow, 2.0=fast)")] = 1.0,
):
    """
    Data replay script - automatically replay all joints and chassis
    
    Two replay modes (both will replay all components):
    1. joint_pos: Joint position replay (chassis + arms + grippers + lift/waist + head)
    2. end_pose: End pose replay (chassis + dual arm end pose + grippers + lift/waist + head)
    
    Chassis control mode:
    - Use independent thread to continuously control chassis at 100Hz frequency
    - Calculate speed based on current odometry and target odometry

    Examples:
    
    # Joint position replay (normal speed)
    python data_replay_example.py ./collected_data/episode_0000 --mode joint_pos
    
    # End pose replay
    python data_replay_example.py ./collected_data/episode_0000 --mode end_pose
    
    # Slow replay (0.5x speed)
    python data_replay_example.py ./collected_data/episode_0000 --speed 0.5
    
    # Fast replay (2x speed)
    python data_replay_example.py ./collected_data/episode_0000 --speed 2.0
    """
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Load data
    try:
        episode_data = load_episode_data(episode_dir)
    except Exception as e:
        print(f"✗ Failed to load data: {e}")
        sys.exit(1)
    
    # Connect to robot
    print(f"\nConnecting to robot {server}...")
    try:
        robot = connect(f"x2://{server}")
        print("✓ Robot connected successfully")
        print(f"  Robot model: {robot.get_robot_model()}")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        sys.exit(1)
    
    # Confirm start
    robot_model = robot.get_robot_model()
    print(f"\nReplay configuration:")
    print(f"  - Mode: {mode}")
    print(f"  - Speed: {speed}x")
    print(f"  - Number of frames: {episode_data['num_frames']}")

    record_data_model = episode_data['model']

    if (record_data_model != robot_model):
        print(f"✗ Record data model {record_data_model} does not match robot model {robot_model}")
        sys.exit(1)
    
    if robot_model == "desktop":
        if mode == "joint_pos":
            print(f"  - Replay content: Dual arms + Grippers + Head")
        elif mode == "end_pose":
            print(f"  - Replay content: Dual arms end pose + Grippers + Head")
    else:
        waist_lift = "Lift" if robot_model == "quanta_x1" else "Waist"
        if mode == "joint_pos":
            print(f"  - Replay content: Chassis + Dual arms + Grippers + {waist_lift} + Head")
        elif mode == "end_pose":
            print(f"  - Replay content: Chassis + Dual arms end pose + Grippers + {waist_lift} + Head")
    
    input("\nPress Enter to start replay...")
    
    try:
        if mode == "joint_pos":
            replay_by_joint_positions(robot, episode_data, speed)
        elif mode == "end_pose":
            replay_by_end_pose(robot, episode_data, speed)
        else:
            print(f"✗ Unknown replay mode: {mode}")
            sys.exit(1)
    except Exception as e:
        print(f"\n✗ Error during replay: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    
    print("\nReplay completed")


if __name__ == "__main__":
    typer.run(main)

