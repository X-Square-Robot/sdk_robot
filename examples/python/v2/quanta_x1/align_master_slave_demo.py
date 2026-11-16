"""Copy the master arm joint positions to the EX001 slave arm."""

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


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def verify(robot, model="ex001", slave_arm="left"):
    if model not in ("ex001", "auto"):
        raise RuntimeError("master/slave alignment is only available on EX001")
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    if slave_arm not in ("left", "right"):
        raise ValueError("slave-arm must be left or right")
    master = robot.master_left_arm if slave_arm == "left" else robot.master_right_arm
    target = require(master.get_joint_states(timeout=3.0), f"Master{slave_arm.title()}Arm.GetJointStates").position
    if not target:
        raise AssertionError("master arm returned no joints")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS), timeout=5.0), "RobotControl.SetManipulatorControlMode")
    controller = robot.left_arm_controller if slave_arm == "left" else robot.right_arm_controller
    require(controller.set_joint_positions(list(target), deadline_ms=1000), f"{slave_arm.title()}ArmController.SetJointPositions")
    time.sleep(0.12)
    actual = require(controller.get_joint_states(deadline_ms=1000), f"{slave_arm.title()}ArmController.GetJointStates").position
    if any(abs(a - b) > 1e-5 for a, b in zip(actual, target)):
        raise AssertionError("master/slave joint readback mismatch")
    print(f"aligned {len(target)} joints")


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
    parser.add_argument("--slave-arm", choices=("left", "right"), default="left")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        verify(robot, slave_arm=args.slave_arm)
        print("PASS: Quanta X1 master/slave alignment")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
