import time
from typing import Annotated

import grpc
import typer
from x2robot import connect
from arm_control import move_by_end_pose


def main(
    server: Annotated[str, typer.Option(help="server address, e.g. 192.168.10.1:50051")] = "localhost:50051",
    slave_arm: Annotated[str, typer.Option(help="slave arm to move: left or right")] = "left",
):
    print("This demo will move the slave arm to a preset position.")
    print(f"server: {server}")
    print("Please make sure there is enough free space around the arm.")
    if input("continue? (y/n): ").lower() != "y":
        return

    try:
        slave_robot = connect(f"x2://{server}")
    except grpc.RpcError as e:
        print(f"connect failed: {e.code().name} - {e.details()}")
        return

    try:
        print(f"moving slave {slave_arm} arm via move_by_end_pose...")
        move_by_end_pose(slave_robot, slave_arm)
        time.sleep(0.5)
    except Exception as e:
        print(f"failed to move slave arm: {e}")
        return

    print("Slave arm move completed.")


if __name__ == "__main__":
    typer.run(main)
