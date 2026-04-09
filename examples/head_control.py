import time
from typing import Annotated
import typer
from x2robot import Robot
from x2robot import connect
from x2robot.sdk import HeadPose
from x2robot.sdk import RobotModeParam, RobotWorkMode

def move_head(robot: Robot):
    # currently speed is fixed 0.1 rad/s
    print("set head pose to yaw=0.5")
    robot.head.set_pose(HeadPose(pitch=0.0, yaw=0.5))

    time.sleep(1.0)
    head_pose = robot.head.get_pose()
    print(f"cur_head_pose: {head_pose}")
    # reset head
    print("reset head")
    robot.head.reset()

    # wait 1s and get head pose
    time.sleep(1.0)

    head_pose = robot.head.get_pose()
    print(f"cur_head_pose: {head_pose}")

def stream_head_joint_states(robot: Robot):
    print("Starting head joint states streaming...")
    print("Press Ctrl+C to stop streaming")
    try:
        for joint_state in robot.head.get_joint_states_stream():
            print(f"joint_state: {joint_state}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping streaming...")

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    action: Annotated[str, typer.Option(help="action: move, stream")] = "move",
):
    robot = connect(f"x2://{server}")

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    if action == "move":
        move_head(robot)
    elif action == "stream":
        stream_head_joint_states(robot)
    else:
        print(f"Unknown action: {action}")
        print("Valid actions: move, stream")
        return

if __name__ == "__main__":
    typer.run(main)
