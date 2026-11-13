"""System mode, static information, dynamic information and model example."""

from __future__ import annotations

import argparse

from x2robot.sdk import RobotModeParam, RobotWorkMode
from x2robot.sdk.v2 import RobotModelType

import x2robot_sdk


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def verify(robot, model="auto"):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    static = require(robot.system.get_static_info(timeout=3.0), "System.GetStaticInfo")
    dynamic = require(robot.system.get_dynamic_info(timeout=3.0), "System.GetDynamicInfo")
    model_info = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType")
    if not static.model:
        raise AssertionError("System.GetStaticInfo returned an empty model")
    expected = {"ex001": RobotModelType.QUANTA_X1_PRO, "cx002": RobotModelType.QUANTA_X2}.get(model)
    if expected is not None and model_info.model_type != expected:
        raise AssertionError(f"unexpected model type: {model_info.model_type}")
    print(f"model={static.model} type={model_info.model_type} cpu={dynamic.runtime_info.cpu_load_percent:.1f}%")


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
    # The default follows the v1 example: select SDK mode, then print both
    # static and dynamic information in one invocation.  Individual actions
    # remain available for scripts that only need one query.
    parser.add_argument("--action", choices=("verify", "all", "mode", "static", "dynamic", "model"), default="all")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot)
        else:
            if args.action in ("all", "mode"):
                print(require(enter_sdk_mode(robot), "System.SetWorkMode"))
            if args.action in ("all", "static"):
                print(f"static_info: {require(robot.system.get_static_info(timeout=3.0), 'System.GetStaticInfo')}")
            if args.action in ("all", "dynamic"):
                value = require(robot.system.get_dynamic_info(timeout=3.0), "System.GetDynamicInfo")
                power = value.power_status
                runtime = value.runtime_info
                print(
                    f"dynamic_info: is_charging: {power.is_charging if power else None}, "
                    f"battery_level: {power.value if power else None}% "
                    f"cpu_usage: {runtime.cpu_load_percent if runtime else None}% "
                    f"gpu_usage: {runtime.gpu_load_percent if runtime else None}% "
                    f"memory_usage: {runtime.memory_usage_mb if runtime else None}MB, "
                    f"core_temp: {runtime.core_temp_celsius if runtime else None}℃"
                )
            if args.action == "model":
                print(require(robot.system.get_model_type(timeout=3.0), "System.GetModelType"))
        print("PASS: system")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
