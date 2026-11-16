"""Quanta X3 gripper example: direct v2 API calls."""

from __future__ import annotations

import argparse
import math
import time

import x2robot_sdk
from x2robot.sdk import (
    GripperPosition,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)


MAX_POSITION = 1.526


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation} failed: {error.get('error_name', 'UNKNOWN')}: "
            f"{error.get('error_message', 'SDK call failed')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


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


def confirm_motion(action, side, assume_yes=False):
    if assume_yes:
        return
    try:
        input(
            f"About to execute {action} on the Quanta X3 {side} gripper; "
            "the robot will begin moving. Press Enter to continue or Ctrl+C "
            "to cancel (use --yes for automation): "
        )
    except EOFError as error:
        raise RuntimeError(
            "Motion confirmation required; pass --yes for automation"
        ) from error


def set_mode(robot):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(
        robot.robot_control.set_manipulator_control_mode(
            ManipulatorControlModeParam(
                mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS
            ),
            timeout=5.0,
        ),
        "RobotControl.SetManipulatorControlMode",
    )


def gripper(robot, side):
    return robot.left_gripper if side == "left" else robot.right_gripper


def set_position(hand, position, operation="Gripper.SetPosition"):
    if not 0.0 <= position <= MAX_POSITION:
        raise ValueError(f"gripper position must be within [0, {MAX_POSITION}]")
    require(hand.set_position(GripperPosition(position=position), timeout=3.0), operation)


def verify_one(robot, side):
    hand = gripper(robot, side)
    actual = require(hand.get_position(timeout=3.0), f"{side} gripper GetPosition")
    if not 0.0 <= actual.position <= MAX_POSITION:
        raise AssertionError(f"{side} gripper returned out-of-range feedback")
    stream = hand.get_position_stream(timeout=4.0)
    try:
        streamed = require(next(iter(stream)), f"{side} gripper GetPositionStream")
        if not 0.0 <= streamed.position <= MAX_POSITION:
            raise AssertionError(f"{side} gripper stream returned out-of-range feedback")
    finally:
        close_stream(stream)


def move_gripper(robot, side):
    set_mode(robot)
    hand = gripper(robot, side)
    current = require(hand.get_position(timeout=3.0), f"{side} gripper GetPosition")
    print(f"current position: {current.position}")
    # Keep the same small, monotonic demonstration as the other model examples.
    for target in (0.0, 0.3, 0.6):
        print(f"set {side} gripper position to {target}")
        set_position(hand, target)
        time.sleep(1.0)
        print(
            f"current position: "
            f"{require(hand.get_position(timeout=3.0), f'{side} gripper GetPosition').position}"
        )


def stream_gripper(robot, side, seconds=None):
    hand = gripper(robot, side)
    stream = hand.get_position_stream(
        timeout=None if seconds is None else max(4.0, seconds + 1.0)
    )
    end = None if seconds is None else time.monotonic() + seconds
    try:
        for result in stream:
            print(f"position: {require(result, f'{side} gripper GetPositionStream')}")
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(stream)


def verify(robot, model="quanta_x3"):
    if model not in ("quanta_x3", "auto"):
        raise RuntimeError("this example is only for Quanta X3")
    verify_one(robot, "left")
    verify_one(robot, "right")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--gripper", choices=("left", "right", "both"), default="left")
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="move")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stream duration; 0 means until Ctrl+C")
    parser.add_argument("--yes", action="store_true", help="skip the motion confirmation prompt")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and non-negative")
    if args.action == "stream" and args.gripper == "both":
        parser.error("stream does not support --gripper both")
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        sides = ("left", "right") if args.gripper == "both" else (args.gripper,)
        if args.action == "move":
            confirm_motion(args.action, args.gripper, args.yes)
        if args.action == "verify":
            verify(robot)
        else:
            for side in sides:
                if args.action == "move":
                    move_gripper(robot, side)
                else:
                    stream_gripper(robot, side, args.seconds or None)
        print("PASS: Quanta X3 gripper control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
