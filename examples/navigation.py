import time
from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import NavigationMode, NavigationModeParam, SaveMapParam
from x2robot.sdk import RobotModeParam, RobotWorkMode

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    result = robot.navigation.set_navigation_mode(NavigationModeParam(mode=NavigationMode.BUILT_IN_NAVIGATION))
    print(f"set built-in navigation mode success: {result.is_success}")

    result = robot.navigation.start_mapping();
    print(f"start mapping success: {result.is_success}")
    print(f"waiting 10 seconds for map to be ready...")

    time.sleep(10)

    result = robot.navigation.stop_mapping(SaveMapParam(map_name="test"))
    print(f"stop mapping success: {result.is_success}")

if __name__ == "__main__":
    typer.run(main)
