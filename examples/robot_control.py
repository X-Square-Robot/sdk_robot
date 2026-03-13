from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import RobotModeParam, RobotWorkMode

# Example:
# python examples/robot_control.py --action stop
# python examples/robot_control.py --action recover
# python examples/robot_control.py --action homing

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="stop | recover | homing")] = "homing",
):
    robot = connect(f"x2://{server}")

    result = robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    # call when emergency situation occurs, and do not call frequently
    if action == "stop":
        print("Calling emergency_stop...")
        result = robot.robot_control.emergency_stop()
        print(f"emergency_stop result: {result.is_success}")
    elif action == "recover":
        print("Calling recover_emergency_stop...")
        result = robot.robot_control.recover_emergency_stop()
        print(f"recover_emergency_stop result: {result.is_success}")
    elif action == "homing":
        print("Calling homing...")
        result = robot.robot_control.homing()
        print(f"homing result: {result.is_success}")
    else:
        print(f"Unknown action: {action}")
        return

if __name__ == "__main__":
    typer.run(main)
