"""Quanta X1 gripper example: direct v2 API calls."""

from __future__ import annotations

import argparse
import time

import x2robot_sdk
from x2robot.sdk import (
    GripperPosition,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def set_mode(robot):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS), timeout=5.0), "RobotControl.SetManipulatorControlMode")


def verify_one(robot, side):
    set_mode(robot)
    hand = robot.left_gripper if side == "left" else robot.right_gripper
    expected = 0.42
    require(hand.set_position(GripperPosition(position=expected), timeout=3.0), f"{side} gripper SetPosition")
    time.sleep(0.12)
    actual = require(hand.get_position(timeout=3.0), f"{side} gripper GetPosition")
    if abs(actual.position - expected) > 1e-5:
        raise AssertionError(f"{side} gripper readback mismatch")
    stream = hand.get_position_stream(timeout=4.0)
    try:
        streamed = require(next(iter(stream)), f"{side} gripper GetPositionStream")
        if abs(streamed.position - expected) > 1e-5:
            raise AssertionError(f"{side} gripper stream mismatch")
    finally:
        close_stream(stream)


def move_gripper(robot, side):
    set_mode(robot)
    hand = robot.left_gripper if side == "left" else robot.right_gripper
    print(f"current position: {require(hand.get_position(timeout=3.0), f'{side} gripper GetPosition')}")
    for target in (0.0, 0.3, 0.6):
        print(f"set {side} gripper position to {target}")
        require(hand.set_position(GripperPosition(position=target), timeout=3.0), f"{side} gripper SetPosition")
        time.sleep(1.0)
        print(f"current position: {require(hand.get_position(timeout=3.0), f'{side} gripper GetPosition')}")


def stream_gripper(robot, side, seconds=None):
    import time as _time
    hand = robot.left_gripper if side == "left" else robot.right_gripper
    stream = hand.get_joint_states_stream(timeout=None if seconds is None else max(4.0, seconds + 1.0))
    end = None if seconds is None else _time.monotonic() + seconds
    try:
        for result in stream:
            print(f"joint_state: {require(result, f'{side} gripper GetJointStatesStream')}")
            if end is not None and _time.monotonic() >= end:
                break
    finally:
        close_stream(stream)


def verify(robot, model="ex001"):
    if model not in ("ex001", "auto"):
        raise RuntimeError("this example is only for EX001/Quanta X1")
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
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--gripper", choices=("left", "right", "both"), default="left")
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="move")
    parser.add_argument("--gripper-type", choices=("g", "c"), default="g")
    parser.add_argument("--seconds", type=float, default=0.0, help="stream duration; 0 means until Ctrl+C")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot)
        else:
            for side in (("left", "right") if args.gripper == "both" else (args.gripper,)):
                if args.action == "move":
                    move_gripper(robot, side)
                else:
                    stream_gripper(robot, side, args.seconds or None)
        print("PASS: Quanta X1 gripper control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
