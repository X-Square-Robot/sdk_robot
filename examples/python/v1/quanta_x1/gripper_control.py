from typing import Annotated
from x2robot import Robot
import typer
from x2robot import connect
from x2robot.sdk import GripperPosition
from time import sleep
from x2robot.sdk import RobotModeParam, RobotWorkMode
from x2robot.sdk import ManipulatorControlModeParam, ManipulatorControlMode

def move_gripper(robot: Robot, gripper: str):
    enter_sdk_mode(robot)

    robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS))

    gripper_controller = robot.left_gripper if gripper == "left" else robot.right_gripper

    position = gripper_controller.get_position()
    print(position)

    print("set left gripper position to 0.0")
    gripper_controller.set_position(GripperPosition(position=0.0))

    sleep(1.5)

    position = gripper_controller.get_position()
    print(position)

    print("set left gripper position to 0.3")
    gripper_controller.set_position(GripperPosition(position=0.3))
    sleep(1.0)

    position = gripper_controller.get_position()
    print(position)

    print("set left gripper position to 0.6")
    gripper_controller.set_position(GripperPosition(position=0.6))
    sleep(1.0)

    position = gripper_controller.get_position()
    print(position)

def stream_gripper_data(robot: Robot, gripper: str):
    print("Starting gripper data streaming...")
    print("Press Ctrl+C to stop streaming")

    gripper_controller = robot.left_gripper if gripper == "left" else robot.right_gripper

    try:
        for joint_state in gripper_controller.get_joint_states_stream():
            print(f"joint_state: {joint_state}")
            sleep(0.1)
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
    gripper: Annotated[str, typer.Option(help="gripper: left, right")] = "left",
):
    robot = connect(f"x2://{server}")
    try:

        if action == "move":
            move_gripper(robot, gripper)
        elif action == "stream":
            stream_gripper_data(robot, gripper)
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: move, stream")
            return
    finally:
        restore_idle_mode(robot)

if __name__ == "__main__":
    typer.run(main)
