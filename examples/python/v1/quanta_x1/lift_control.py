import signal
from typing import Annotated
import typer
from x2robot import Robot, connect
from x2robot.sdk import LiftPosition, RobotModeParam, RobotWorkMode, ManipulatorControlModeParam, ManipulatorControlMode
import time

# only for quanta_x1
def move_by_lift_position(robot: Robot, direction: str, distance: float):
    cur_position = robot.lift.get_lift_position()
    print(f"current_position: {cur_position}")
    if direction == "up":
        lift_position = LiftPosition(position=cur_position.position + distance)
    elif direction == "down":
        lift_position = LiftPosition(position=cur_position.position - distance)
    else:
        print(f"Unknown direction: {direction}")
        return
    robot.lift.set_lift_position(lift_position)
    time.sleep(0.5)
    cur_position = robot.lift.get_lift_position()
    print(f"current_position: {cur_position}")

def stream_lift_joint_states(robot: Robot):
    print("Starting lift joint states streaming...")
    print("Press Ctrl+C to stop streaming")

    try:
        for joint_state in robot.lift.get_joint_states_stream():
            print(f"joint_state: {joint_state}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping streaming...")


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
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
    direction: Annotated[str, typer.Option(help="direction: up, down")] = "down",
    distance: Annotated[float, typer.Option(help="distance: distance to move")] = 0.05,
):

    robot = connect(f"x2://{server}")
    try:

        if action == "move":
            enter_sdk_mode(robot)
            robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS))
            # quanta_x1, move by lift position
            move_by_lift_position(robot, direction, distance)
        elif action == "stream":
            stream_lift_joint_states(robot)
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: move, stream")
            return
    finally:
        restore_idle_mode(robot)

if __name__ == "__main__":
    typer.run(main)
