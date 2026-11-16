"""Desktop arm example: direct Set -> Get -> stream calls."""

from __future__ import annotations

import argparse
import time

import x2robot_sdk
from x2robot.sdk import (
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)

from x2robot_sdk import Pose
import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from toppra.constraint import JointAccelerationConstraint, JointVelocityConstraint


JOINT_LOWER_LIMITS = np.array([-2.792, 0.0, -3.14, -1.57, -1.4, -1.745])
JOINT_UPPER_LIMITS = np.array([2.792, 3.44, 0.0, 1.57, 1.4, 1.745])


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def confirm_motion(action, mode, arm_side, assume_yes=False):
    if assume_yes:
        return
    scope = "both arms" if action == "verify" or arm_side == "both" else f"{arm_side} arm"
    try:
        input(
            f"About to execute Desktop {scope} {mode} control; the robot will begin moving. "
            "Press Enter to continue or Ctrl+C to cancel (use --yes for automation): "
        )
    except EOFError as error:
        raise RuntimeError(
            "Motion confirmation required; press Enter in an interactive terminal "
            "or pass --yes for automation"
        ) from error


def set_mode(robot, mode):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=mode), timeout=5.0), "RobotControl.SetManipulatorControlMode")


def move_arm_joints_toppra(controller, target_positions, v_max=1.0, a_max=3.0):
    """Plan and execute a joint trajectory with the same limits as v1."""
    q_end = np.asarray(target_positions, dtype=float)
    if q_end.shape != JOINT_LOWER_LIMITS.shape or not np.all(np.isfinite(q_end)):
        raise ValueError("target must contain six finite joint positions")
    if np.any(q_end < JOINT_LOWER_LIMITS) or np.any(q_end > JOINT_UPPER_LIMITS):
        raise ValueError("target joint position is outside the arm limits")

    current = require(
        controller.get_joint_states(deadline_ms=1000), "Arm.GetJointStates"
    )
    q_start = np.asarray(list(current.position), dtype=float)
    if q_start.shape != q_end.shape or not np.all(np.isfinite(q_start)):
        raise RuntimeError("Arm.GetJointStates returned an invalid joint position")
    q_start = np.clip(q_start, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

    path = ta.SplineInterpolator([0, 1], np.stack([q_start, q_end]))
    constraints = [
        JointVelocityConstraint([v_max] * len(q_start)),
        JointAccelerationConstraint([a_max] * len(q_start)),
    ]
    trajectory = algo.TOPPRA(constraints, path).compute_trajectory(0, 0)
    if trajectory is None:
        raise RuntimeError("TOPP-RA joint planning failed")

    dt = 0.002  # 500 Hz, matching the v1 example.
    for timestamp in np.arange(0.0, trajectory.duration, dt):
        command = np.clip(
            np.asarray(trajectory(float(timestamp)), dtype=float),
            JOINT_LOWER_LIMITS,
            JOINT_UPPER_LIMITS,
        )
        require(
            controller.set_joint_positions(command.tolist(), deadline_ms=500),
            "Arm.SetJointPositions",
        )
        time.sleep(dt)

    require(
        controller.set_joint_positions(q_end.tolist(), deadline_ms=500),
        "Arm.SetJointPositions(final)",
    )


def move_arm_endpose_toppra(controller, target, v_max=2.2, a_max=0.3):
    """Plan a Cartesian line with TOPP-RA and interpolate orientation with SLERP."""
    current = require(controller.get_end_pose(deadline_ms=1000), "Arm.GetEndPose")
    p_start = np.array([current.x, current.y, current.z], dtype=float)
    p_end = np.array([target.x, target.y, target.z], dtype=float)
    q_start = np.array([current.qx, current.qy, current.qz, current.qw], dtype=float)
    q_end = np.array([target.qx, target.qy, target.qz, target.qw], dtype=float)
    if not all(np.all(np.isfinite(values)) for values in (p_start, p_end, q_start, q_end)):
        raise RuntimeError("Arm.GetEndPose or target contains non-finite values")
    if np.linalg.norm(q_start) < 1e-9 or np.linalg.norm(q_end) < 1e-9:
        raise ValueError("end-pose orientations must be non-zero quaternions")

    distance = np.linalg.norm(p_end - p_start)
    path_length = distance if distance > 1e-6 else 1.0
    path = ta.SplineInterpolator([0, 1], np.array([[0.0], [path_length]]))
    constraints = [
        constraint.JointVelocityConstraint([v_max]),
        constraint.JointAccelerationConstraint([a_max]),
    ]
    trajectory = algo.TOPPRA(constraints, path).compute_trajectory(0, 0)
    if trajectory is None:
        raise RuntimeError("TOPP-RA end-pose planning failed")

    slerp = Slerp([0, trajectory.duration], R.from_quat([q_start, q_end]))
    interval = 0.005  # 200 Hz, matching the v1 example.
    for timestamp in np.arange(0.0, trajectory.duration, interval):
        alpha = np.clip(
            float(trajectory(float(timestamp))[0]) / path_length, 0.0, 1.0
        )
        position = p_start + (p_end - p_start) * alpha
        orientation = slerp(float(timestamp)).as_quat()
        pose = Pose(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            qx=float(orientation[0]),
            qy=float(orientation[1]),
            qz=float(orientation[2]),
            qw=float(orientation[3]),
        )
        require(controller.set_end_pose(pose, deadline_ms=500), "Arm.SetEndPose")
        time.sleep(interval)

    require(controller.set_end_pose(target, deadline_ms=500), "Arm.SetEndPose(final)")
    time.sleep(0.1)


def move_arm(robot, side, mode):
    arm = controller(robot, side)
    if mode == "joint_pos":
        set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
        print("Starting joint reset...")
        move_arm_joints_toppra(arm, [0.0] * 6)
        time.sleep(1)
        target = [-0.1486, 0.4707, -0.8101, 0.6350, 0.3164, 0.0]
        print(f"Moving {side} arm in joint space: {target}")
        move_arm_joints_toppra(arm, target)
        time.sleep(2)
        print("Movement Finished.")
    else:
        set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
        print("Starting end pose reset...")
        move_arm_endpose_toppra(arm, Pose(x=0.0, y=0.0, z=0.0, qw=1.0))
        time.sleep(2)
        target = Pose(
            x=0.0,
            y=0.0,
            z=0.2,
            qx=-0.0076,
            qy=0.0868,
            qz=0.0868,
            qw=0.9924,
        )
        print(f"Moving {side} arm in end-pose space: {target}")
        move_arm_endpose_toppra(arm, target)
        print("Movement Finished.")


def stream_arm(robot, side, mode, seconds=None):
    arm = controller(robot, side)
    opened = require(
        arm.get_end_pose_stream(deadline_ms=3000) if mode == "end_pose"
        else arm.get_joint_states_stream(deadline_ms=3000), f"{side} arm stream"
    )
    end = None if seconds is None else time.monotonic() + seconds
    try:
        while end is None or time.monotonic() < end:
            print_arm(require(opened.read(deadline_ms=3000), f"{side} arm stream/read"), mode)
    except KeyboardInterrupt:
        print("\nStopping streaming...")
    finally:
        close_stream(opened)


def controller(robot, side):
    return robot.left_arm_controller if side == "left" else robot.right_arm_controller


def print_arm(value, mode):
    if mode == "end_pose":
        print(f"end_pose: position=(x={value.x:.6f}, y={value.y:.6f}, z={value.z:.6f}); "
              f"quaternion=(qx={value.qx:.6f}, qy={value.qy:.6f}, qz={value.qz:.6f}, qw={value.qw:.6f})")
    else:
        print(f"joint_states: name={list(value.name)}; position={list(value.position)}; "
              f"velocity={list(value.velocity)}; effort={list(value.effort)}")


def verify_one(robot, side):
    arm = controller(robot, side)
    joints = [0.08, 0.0, 0.0, 0.0, 0.0, 0.0]
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    require(arm.set_joint_positions(joints), f"{side} arm SetJointPositions")
    time.sleep(0.12)
    actual = require(arm.get_joint_states(), f"{side} arm GetJointStates")
    if list(actual.position) != joints:
        raise AssertionError(f"{side} arm joint readback {list(actual.position)!r}")
    pose = Pose(x=0.31, y=0.07, z=0.72, qw=1.0)
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
    require(arm.set_end_pose(pose), f"{side} arm SetEndPose")
    time.sleep(0.12)
    actual_pose = require(arm.get_end_pose(), f"{side} arm GetEndPose")
    for name in ("x", "y", "z"):
        if abs(getattr(actual_pose, name) - getattr(pose, name)) > 1e-5:
            raise AssertionError(f"{side} arm pose {name} mismatch")
    stream = require(arm.get_joint_states_stream(), f"{side} arm GetJointStatesStream")
    try:
        require(stream.read(), f"{side} arm GetJointStatesStream/read")
    finally:
        close_stream(stream)


def verify(robot, model="ex001"):
    if model not in ("desktop", "ex001", "auto"):
        raise RuntimeError("desktop arm example requires an EX001-compatible robot")
    verify_one(robot, "left")
    verify_one(robot, "right")


_sdk_mode_entered = False


def enter_sdk_mode(robot):
    """切到 SDK 模式并记录占用，退出时由 restore_idle_mode 交还应用。"""
    global _sdk_mode_entered
    result = robot.system.set_work_mode(
        RobotModeParam(mode=RobotWorkMode.SDK), timeout=5.0
    )
    _sdk_mode_entered = bool(getattr(result, "is_success", False))
    return result


def restore_idle_mode(robot):
    """只交还本示例占用的 SDK 模式：/application/task_info.type 回到 0。"""
    global _sdk_mode_entered
    if not _sdk_mode_entered:
        return
    _sdk_mode_entered = False
    try:
        robot.system.set_work_mode(
            RobotModeParam(mode=RobotWorkMode.IDLE), timeout=5.0
        )
    except Exception as error:  # 清理失败不能掩盖示例本身的错误
        print(f"WARN: failed to leave SDK mode: {error}")

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--action", choices=("read", "verify", "move", "stream"), default="read")
    parser.add_argument("--seconds", type=float, default=0.0, help="stream duration; 0 means until Ctrl+C")
    parser.add_argument("--yes", action="store_true", help="skip the motion confirmation prompt")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action in ("verify", "move"):
            confirm_motion(args.action, args.mode, args.arm, args.yes)
        if args.action == "verify":
            verify(robot)
        else:
            sides = ("left", "right") if args.arm == "both" else (args.arm,)
            for side in sides:
                if args.action == "read":
                    arm = controller(robot, side)
                    method = arm.get_end_pose if args.mode == "end_pose" else arm.get_joint_states
                    print_arm(require(method(deadline_ms=3000), f"{side} arm read"), args.mode)
                elif args.action == "move":
                    move_arm(robot, side, args.mode)
                else:
                    if args.arm == "both":
                        raise RuntimeError("stream does not support --arm both")
                    stream_arm(robot, side, args.mode, args.seconds or None)
        print("PASS: desktop arm control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
