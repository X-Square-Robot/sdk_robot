from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import RobotModeParam, RobotWorkMode

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    result = robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    print(result.is_success)

    result = robot.system.get_static_info()
    print(f"static_info: {result}")

    result = robot.system.get_dynamic_info()
    print(f"dynamic_info: {result.power_status.is_charging}, {result.power_status.value}, {result.runtime_info.cpu_load_percent}, {result.runtime_info.gpu_load_percent}, {result.runtime_info.memory_usage_mb}, {result.runtime_info.core_temp_celsius}")

if __name__ == "__main__":
    typer.run(main)
