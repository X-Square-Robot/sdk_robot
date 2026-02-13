import signal
from typing import Annotated
import typer
from x2robot import Robot, connect
from x2robot.sdk import LiftPosition, RobotModeParam, RobotWorkMode
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

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
    direction: Annotated[str, typer.Option(help="direction: up, down")] = "down",
    distance: Annotated[float, typer.Option(help="distance: distance to move")] = 0.05,
):

    robot = connect(f"x2://{server}")

    if action == "move":
        robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
        # quanta_x1, move by lift position
        move_by_lift_position(robot, direction, distance)
    elif action == "stream":
        stream_lift_joint_states(robot)
    else:
        print(f"Unknown action: {action}")
        print("Valid actions: move, stream")
        return

if __name__ == "__main__":
    typer.run(main)
