from typing import Annotated
from x2robot import Robot
import typer
from x2robot import connect
from x2robot.sdk import GripperPosition
from time import sleep
from x2robot.sdk import RobotModeParam, RobotWorkMode
from x2robot.sdk import ManipulatorControlModeParam, ManipulatorControlMode

def move_gripper(robot: Robot, gripper: str):
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
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

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
    gripper: Annotated[str, typer.Option(help="gripper: left, right")] = "left",
):
    robot = connect(f"x2://{server}")

    if action == "move":
        move_gripper(robot, gripper)
    elif action == "stream":
        stream_gripper_data(robot, gripper)
    else:
        print(f"Unknown action: {action}")
        print("Valid actions: move, stream")
        return

if __name__ == "__main__":
    typer.run(main)
