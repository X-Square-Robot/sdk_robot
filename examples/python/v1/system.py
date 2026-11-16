from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import RobotModeParam, RobotWorkMode


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
):
    robot = connect(f"x2://{server}")
    try:

        result = enter_sdk_mode(robot)
        print(result.is_success)

        result = robot.system.get_static_info()
        print(f"static_info: {result}")

        result = robot.system.get_dynamic_info()
        print(
            f"dynamic_info: is_charging: {result.power_status.is_charging}, "
            f"battery_level: {result.power_status.value}% "
            f"cpu_usage: {result.runtime_info.cpu_load_percent}% "
            f"gpu_usage: {result.runtime_info.gpu_load_percent}% "
            f"memory_usage: {result.runtime_info.memory_usage_mb}MB, "
            f"core_temp: {result.runtime_info.core_temp_celsius}℃"
        )
    finally:
        restore_idle_mode(robot)

if __name__ == "__main__":
    typer.run(main)
