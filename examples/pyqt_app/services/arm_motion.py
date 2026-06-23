from __future__ import annotations

import time

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from toppra.constraint import JointAccelerationConstraint, JointVelocityConstraint

from x2robot import Robot
from x2robot.geometry_msgs import Point, Pose, Quaternion
from x2robot.sdk import JointPositions, ManipulatorControlMode, ManipulatorControlModeParam, RobotModeParam, RobotWorkMode

from .common import CancelFn, LogFn


def select_arm(robot: Robot, arm: str):
    if arm == "left":
        return robot.left_arm
    if arm == "right":
        return robot.right_arm
    raise ValueError("Invalid arm. Valid options: left, right")


def move_by_joint_positions(robot: Robot, arm: str, log: LogFn) -> None:
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    robot.robot_control.set_manipulator_control_mode(
        ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    )

    arm_client = select_arm(robot, arm)
    log("Starting joint reset.")
    move_arm_joints_toppra(arm_client, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], log)
    time.sleep(1)

    target_q = [-0.1486, 0.4707, -0.8101, 0.6350, 0.3164, 0.0]
    log(f"Moving to target joint angles: {target_q}")
    move_arm_joints_toppra(arm_client, target_q, log)
    time.sleep(2)
    log("Demo finished.")


def move_by_end_pose(
    robot: Robot,
    arm: str,
    log: LogFn,
    target_x: float = 0.0,
    target_y: float = 0.0,
    target_z: float = 0.0,
    target_qx: float = -0.0076,
    target_qy: float = 0.0868,
    target_qz: float = 0.0868,
    target_qw: float = 0.9924,
) -> None:
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    robot.robot_control.set_manipulator_control_mode(
        ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE)
    )

    arm_client = select_arm(robot, arm)
    log("Starting end pose reset.")
    zero_pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    move_arm_endpose_toppra(arm_client, zero_pose, log)
    time.sleep(2)

    target = Pose(
        position=Point(x=target_x, y=target_y, z=target_z),
        orientation=Quaternion(x=target_qx, y=target_qy, z=target_qz, w=target_qw),
    )
    log(
        "Moving to target pose: "
        f"pos=({target_x:.4f}, {target_y:.4f}, {target_z:.4f}) "
        f"ori=({target_qx:.4f}, {target_qy:.4f}, {target_qz:.4f}, {target_qw:.4f})"
    )
    move_arm_endpose_toppra(arm_client, target, log)


def stream_arm_joint_states(robot: Robot, arm: str, log: LogFn, is_cancelled: CancelFn) -> None:
    log("Starting arm joint-state streaming.")
    arm_client = select_arm(robot, arm)
    for joint_state in arm_client.get_joint_states_stream():
        if is_cancelled():
            log("Stop requested. Ending joint-state stream.")
            break
        log(f"joint_state: {joint_state}")
        time.sleep(0.1)


def stream_arm_end_pose(robot: Robot, arm: str, log: LogFn, is_cancelled: CancelFn) -> None:
    log("Starting arm end-pose streaming.")
    arm_client = select_arm(robot, arm)
    for end_pose in arm_client.get_end_pose_stream():
        if is_cancelled():
            log("Stop requested. Ending end-pose stream.")
            break
        log(f"end_pose: {end_pose}")
        time.sleep(0.1)


def move_arm_joints_toppra(arm_client, target_positions: list[float], log: LogFn) -> None:
    lower_limits = np.array([-2.792, 0.0, -3.14, -1.57, -1.4, -1.745])
    upper_limits = np.array([2.792, 3.44, 0.0, 1.57, 1.4, 1.745])

    current_state = arm_client.get_joint_states()
    q_start = np.array(current_state.position)
    q_end = np.array(target_positions)
    if np.any(q_end < lower_limits) or np.any(q_end > upper_limits):
        raise ValueError("Target position out of limits.")

    q_start = np.clip(q_start, lower_limits, upper_limits)
    path = ta.SplineInterpolator([0, 1], np.stack([q_start, q_end]))
    pc_vel = JointVelocityConstraint([1.0] * len(q_start))
    pc_acc = JointAccelerationConstraint([3.0] * len(q_start))
    traj = algo.TOPPRA([pc_vel, pc_acc], path).compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPP-RA planning failed.")

    duration = traj.duration
    dt = 0.002
    log(f"Executing smooth joint trajectory for {duration:.2f}s")
    for t in np.arange(0, duration, dt):
        q_t_safe = np.clip(traj(t), lower_limits, upper_limits)
        joint_cmd = JointPositions()
        joint_cmd.positions = q_t_safe.tolist()
        arm_client.set_joint_positions(joint_cmd)
        time.sleep(dt)

    final_cmd = JointPositions()
    final_cmd.positions = q_end.tolist()
    arm_client.set_joint_positions(final_cmd)
    log("Movement finished.")


def move_arm_endpose_toppra(arm_client, target_pose: Pose, log: LogFn) -> None:
    start_pose = arm_client.get_end_pose()
    p_start = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
    p_end = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    q_start = [
        start_pose.pose.orientation.x,
        start_pose.pose.orientation.y,
        start_pose.pose.orientation.z,
        start_pose.pose.orientation.w,
    ]
    q_end = [
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w,
    ]

    dist = np.linalg.norm(p_end - p_start)
    path_len = dist if dist > 1e-6 else 1.0
    path = ta.SplineInterpolator([0, 1], np.array([[0], [path_len]]))
    pc_vel = constraint.JointVelocityConstraint([2.2])
    pc_acc = constraint.JointAccelerationConstraint([0.3])
    traj = algo.TOPPRA([pc_vel, pc_acc], path).compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPP-RA planning failed.")

    slerp_func = Slerp([0, traj.duration], R.from_quat([q_start, q_end]))
    interval = 0.005
    log(f"Executing smooth end-pose trajectory for {traj.duration:.2f}s")
    for t in np.arange(0, traj.duration, interval):
        s_t = traj(t)[0]
        alpha = np.clip(s_t / path_len, 0, 1)

        pose = Pose()
        pose.position = Point(
            x=float(p_start[0] + (p_end[0] - p_start[0]) * alpha),
            y=float(p_start[1] + (p_end[1] - p_start[1]) * alpha),
            z=float(p_start[2] + (p_end[2] - p_start[2]) * alpha),
        )
        curr_q = slerp_func(t).as_quat()
        pose.orientation = Quaternion(
            x=float(curr_q[0]),
            y=float(curr_q[1]),
            z=float(curr_q[2]),
            w=float(curr_q[3]),
        )
        arm_client.set_end_pose(pose)
        time.sleep(interval)

    arm_client.set_end_pose(target_pose)
    time.sleep(0.1)
    log("Movement completed.")
