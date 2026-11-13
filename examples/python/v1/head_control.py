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
):
    robot = connect(f"x2://{server}")
    try:

        enter_sdk_mode(robot)

        if action == "move":
            move_head(robot)
        elif action == "stream":
            stream_head_joint_states(robot)
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: move, stream")
            return
    finally:
        restore_idle_mode(robot)

if __name__ == "__main__":
    typer.run(main)
