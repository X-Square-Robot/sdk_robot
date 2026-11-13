"""Quanta X1 lift example: direct Set -> Get -> stream calls."""

from __future__ import annotations

import argparse
import time

import x2robot_sdk
from x2robot.sdk import (
    LiftPosition,
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


def verify(robot, model="ex001"):
    if model not in ("ex001", "auto"):
        raise RuntimeError("this example is only for EX001/Quanta X1")
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    expected = 0.16
    require(robot.lift.set_lift_position(LiftPosition(position=expected), timeout=3.0), "Lift.SetLiftPosition")
    time.sleep(0.12)
    actual = require(robot.lift.get_lift_position(timeout=3.0), "Lift.GetLiftPosition")
    if abs(actual.position - expected) > 1e-5:
        raise AssertionError(f"lift readback mismatch: {actual.position}")
    stream = robot.lift.get_joint_states_stream(timeout=4.0)
    try:
        require(next(iter(stream)), "Lift.GetJointStatesStream")
    finally:
        close_stream(stream)


def move_by_lift_position(robot, direction, distance):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(
        robot.robot_control.set_manipulator_control_mode(
            ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS),
            timeout=5.0,
        ),
        "RobotControl.SetManipulatorControlMode",
    )
    current = require(robot.lift.get_lift_position(timeout=3.0), "Lift.GetLiftPosition")
    print(f"current_position: {current}")
    if direction not in ("up", "down"):
        raise ValueError("direction must be up or down")
    target = current.position + (distance if direction == "up" else -distance)
    require(robot.lift.set_lift_position(LiftPosition(position=target), timeout=3.0), "Lift.SetLiftPosition")
    time.sleep(0.5)
    print(f"current_position: {require(robot.lift.get_lift_position(timeout=3.0), 'Lift.GetLiftPosition')}")


def stream_lift(robot, seconds=None):
    import time as _time
    stream = robot.lift.get_joint_states_stream(timeout=None if seconds is None else max(4.0, seconds + 1.0))
    end = None if seconds is None else _time.monotonic() + seconds
    try:
        for result in stream:
            print(f"joint_state: {require(result, 'Lift.GetJointStatesStream')}")
            if end is not None and _time.monotonic() >= end:
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
    parser.add_argument("--direction", choices=("up", "down"), default="down")
    parser.add_argument("--distance", type=float, default=0.05)
    parser.add_argument("--seconds", type=float, default=0.0)
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "stream":
            stream_lift(robot, args.seconds or None)
        elif args.action == "move":
            move_by_lift_position(robot, args.direction, args.distance)
        else:
            verify(robot)
        print("PASS: Quanta X1 lift control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
