"""Homing, emergency-stop and recovery example."""

from __future__ import annotations

import argparse

import x2robot_sdk
from x2robot.sdk import RobotModeParam, RobotWorkMode


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def sdk_mode(robot):
    require(enter_sdk_mode(robot), "System.SetWorkMode")


def verify(robot, _model="auto"):
    sdk_mode(robot)
    require(robot.robot_control.homing(timeout=20.0), "RobotControl.Homing")
    require(robot.robot_control.emergency_stop(timeout=5.0), "RobotControl.EmergencyStop")
    require(robot.robot_control.recover_emergency_stop(timeout=20.0), "RobotControl.RecoverEmergencyStop")


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
    parser.add_argument("--action", choices=("verify", "stop", "recover", "homing"), default="homing")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot)
        elif args.action == "stop":
            print("Calling emergency_stop...")
            require(robot.robot_control.emergency_stop(timeout=5.0), "RobotControl.EmergencyStop")
            print("emergency_stop result: True")
        elif args.action == "recover":
            print("Calling recover_emergency_stop...")
            require(robot.robot_control.recover_emergency_stop(timeout=20.0), "RobotControl.RecoverEmergencyStop")
            print("recover_emergency_stop result: True")
        else:
            sdk_mode(robot)
            print("Calling homing...")
            require(robot.robot_control.homing(timeout=20.0), "RobotControl.Homing")
            print("homing result: True")
        print("PASS: robot control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
