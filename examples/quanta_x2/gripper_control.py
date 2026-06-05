from typing import Annotated
from x2robot import Robot
import typer
from x2robot import connect
from x2robot.sdk import GripperPosition
from time import sleep
from x2robot.sdk import RobotModeParam, RobotWorkMode
from x2robot.sdk import ManipulatorControlModeParam, ManipulatorControlMode

GRIPPER_RANGES = {
    "g": (0.0, 1.89),
    "c": (0.0, 25.2),
}


def _resolve_range(gripper_type: str):
    gripper_type = gripper_type.lower()
    if gripper_type not in GRIPPER_RANGES:
        raise typer.BadParameter(
            f"unknown gripper-type: {gripper_type}, valid: {list(GRIPPER_RANGES.keys())}"
        )
    return GRIPPER_RANGES[gripper_type]


def move_gripper(robot: Robot, gripper: str, gripper_type: str):
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS))

    gripper_controller = robot.left_gripper if gripper == "left" else robot.right_gripper

    pos_min, pos_max = _resolve_range(gripper_type)
    pos_mid = (pos_min + pos_max) / 2.0

    print(f"gripper={gripper}, type={gripper_type}, range=[{pos_min}, {pos_max}]")
    print(f"current position: {gripper_controller.get_position()}")

    for target in (pos_min, pos_mid, pos_max):
        print(f"set {gripper} gripper position to {target:.3f}")
        gripper_controller.set_position(GripperPosition(position=target))
        sleep(1.5)
        print(f"current position: {gripper_controller.get_position()}")

def stream_gripper_data(robot: Robot, gripper: str):
    print("Starting gripper data streaming...")
    print("Press Ctrl+C to stop streaming")

    gripper_controller = robot.left_gripper if gripper == "left" else robot.right_gripper

    try:
        for position in gripper_controller.get_position_stream():
            print(f"position: {position}")
            sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping streaming...")

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
    gripper: Annotated[str, typer.Option(help="gripper: left, right")] = "left",
    gripper_type: Annotated[str, typer.Option("--gripper-type", help="gripper type: g (range 0.0~1.89) or c (range 0.0~25.2)")] = "g",
):
    robot = connect(f"x2://{server}")

    if action == "move":
        move_gripper(robot, gripper, gripper_type)
    elif action == "stream":
        stream_gripper_data(robot, gripper)
    else:
        print(f"Unknown action: {action}")
        print("Valid actions: move, stream")
        return

if __name__ == "__main__":
    typer.run(main)
