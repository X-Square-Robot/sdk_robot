from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import RobotModeParam, RobotWorkMode

# Example:
# python examples/robot_control.py --action stop
# python examples/robot_control.py --action recover
# python examples/robot_control.py --action homing


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


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="stop | recover | homing")] = "homing",
):
    robot = connect(f"x2://{server}")
    try:

        # Emergency stop/recovery do not require SDK mode. In particular, setting
        # SDK mode first is rejected while a soft emergency stop is active.
        if action == "stop":
            # call when emergency situation occurs, and do not call frequently
            print("Calling emergency_stop...")
            result = robot.robot_control.emergency_stop()
            print(f"emergency_stop result: {result.is_success}")
        elif action == "recover":
            print("Calling recover_emergency_stop...")
            result = robot.robot_control.recover_emergency_stop()
            print(f"recover_emergency_stop result: {result.is_success}")
        elif action == "homing":
            enter_sdk_mode(robot)
            print("Calling homing...")
            result = robot.robot_control.homing()
            print(f"homing result: {result.is_success}")
        else:
            print(f"Unknown action: {action}")
            return
    finally:
        restore_idle_mode(robot)

if __name__ == "__main__":
    typer.run(main)
