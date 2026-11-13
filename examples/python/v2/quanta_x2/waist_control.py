"""Quanta X2 waist example: direct v2 API calls."""

from __future__ import annotations

import argparse
import math
import time

import x2robot_sdk
from x2robot.sdk import (
    JointPositions,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)

from x2robot_sdk import Pose
from x2robot.geometry_msgs import Point, Pose as RosPose, Quaternion


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def set_mode(robot, mode):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=mode), timeout=5.0), "RobotControl.SetManipulatorControlMode")


def remaining(deadline):
    seconds = deadline - time.monotonic()
    if seconds <= 0:
        raise TimeoutError("waist did not reach the target before the deadline")
    return min(0.5, seconds)


def pose_values(pose):
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("waist pose must contain finite values")
    norm = math.hypot(*values[3:])
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("waist pose must contain a non-zero quaternion")
    return values[:3], [value / norm for value in values[3:]]


def pose_reached(actual, target, position_tolerance=0.002, angle_tolerance=0.01):
    position, quaternion = pose_values(actual)
    target_position, target_quaternion = pose_values(target)
    dot = abs(sum(a * b for a, b in zip(quaternion, target_quaternion)))
    angle = 2 * math.acos(min(1.0, dot))
    return (all(abs(a - b) <= position_tolerance for a, b in zip(position, target_position))
            and angle <= angle_tolerance)


def hold_arm_pose(controller, deadline):
    current = require(controller.get_end_pose(deadline_ms=max(1, int(remaining(deadline) * 1000))),
                      "Arm.GetEndPose")
    values = [getattr(current, name) for name in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    if not all(math.isfinite(value) for value in values) or sum(v * v for v in values[3:]) < 1e-18:
        raise ValueError("arm hold pose is invalid")
    return Pose(**dict(zip(("x", "y", "z", "qx", "qy", "qz", "qw"), values)))


def move_end_pose_and_wait(robot, target, timeout=5.0):
    """Refresh both arm holds and waist together, then check actual PoseStamped feedback.

    SetEndPose acknowledges publication only. The 5 ms refresh interval is a
    scheduling target; synchronous RPC latency limits the achieved frequency.
    Position/angle tolerances match config/cx002/waist.yml (2 mm / 0.01 rad).
    """
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    pose_values(target)
    deadline = time.monotonic() + timeout
    left, right = robot.left_arm_controller, robot.right_arm_controller
    left_hold = hold_arm_pose(left, deadline)
    right_hold = hold_arm_pose(right, deadline)
    # Validate feedback before sending any target, including the ROS-shaped type.
    initial = require(robot.waist.get_end_pose(timeout=remaining(deadline)), "Waist.GetEndPose")
    pose_values(initial.pose)
    next_feedback = time.monotonic()
    while True:
        cycle = time.monotonic()
        for controller, hold, label in ((left, left_hold, "LeftArm"), (right, right_hold, "RightArm")):
            require(controller.set_end_pose(hold, deadline_ms=max(1, int(remaining(deadline) * 1000))),
                    label + ".SetEndPose(hold)")
        require(robot.waist.set_end_pose(target, timeout=remaining(deadline)), "Waist.SetEndPose")
        if time.monotonic() >= next_feedback:
            actual = require(robot.waist.get_end_pose(timeout=remaining(deadline)), "Waist.GetEndPose")
            remaining(deadline)
            if pose_reached(actual.pose, target):
                return actual
            next_feedback = time.monotonic() + 0.05
        time.sleep(min(max(0.0, cycle + 0.005 - time.monotonic()), remaining(deadline)))


def wait_joint_positions(robot, target, timeout=5.0):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    if len(target) != 4 or not all(math.isfinite(value) for value in target):
        raise ValueError("waist target must contain four finite joint positions")
    deadline = time.monotonic() + timeout
    while True:
        actual = require(robot.waist.get_joint_states(timeout=remaining(deadline)), "Waist.GetJointStates")
        remaining(deadline)
        values = list(actual.position)
        if len(values) != len(target) or not all(math.isfinite(value) for value in values):
            raise ValueError("waist joint feedback has invalid size or non-finite values")
        if all(abs(a - b) <= 0.01 for a, b in zip(values, target)):
            return actual
        time.sleep(min(0.05, remaining(deadline)))


def verify(robot, model="cx002"):
    if model not in ("cx002", "auto"):
        raise RuntimeError("this example is only for CX002/Quanta X2")
    values = [0.03, -0.02, 0.04, -0.01]
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    require(robot.waist.set_joint_positions(JointPositions(positions=values), timeout=3.0), "Waist.SetJointPositions")
    wait_joint_positions(robot, values)
    stream = robot.waist.get_joint_states_stream(timeout=4.0)
    try:
        require(next(iter(stream)), "Waist.GetJointStatesStream")
    finally:
        close_stream(stream)
    target = RosPose(position=Point(x=0.01, y=-0.02, z=0.61), orientation=Quaternion(w=1.0))
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
    move_end_pose_and_wait(robot, target)


def move_joints(robot):
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    current = require(robot.waist.get_joint_states(timeout=3.0), "Waist.GetJointStates")
    print(f"current_state: {list(current.position)}")
    for target in ([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.3]):
        print(f"moving waist to {target}")
        start = list(require(robot.waist.get_joint_states(timeout=3.0), "Waist.GetJointStates").position)
        steps = 200
        for index in range(1, steps + 1):
            alpha = index / steps
            values = [a + (b - a) * alpha for a, b in zip(start, target)]
            require(robot.waist.set_joint_positions(JointPositions(positions=values), timeout=3.0), "Waist.SetJointPositions")
            time.sleep(0.005)
        time.sleep(0.5)
        print(f"current_state: {require(robot.waist.get_joint_states(timeout=3.0), 'Waist.GetJointStates')}")


def stream_waist(robot, mode, seconds=None):
    stream = (robot.waist.get_end_pose_stream if mode == "end_pose" else robot.waist.get_joint_states_stream)(timeout=None if seconds is None else max(4.0, seconds + 1.0))
    end = None if seconds is None else time.monotonic() + seconds
    try:
        for result in stream:
            print(f"{'end_pose' if mode == 'end_pose' else 'joint_state'}: {require(result, 'Waist stream')}")
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(stream)


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
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="move")
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--seconds", type=float, default=0.0)
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "stream":
            if args.mode == "end_pose":
                set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
            stream_waist(robot, args.mode, args.seconds or None)
        elif args.action == "move":
            if args.mode == "end_pose":
                raise ValueError("end_pose mode is not supported for waist control")
            move_joints(robot)
        else:
            verify(robot)
        print("PASS: Quanta X2 waist control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
