from typing import Annotated
import typer
from x2robot import connect

def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    result = robot.get_robot_status()
    if result.is_success:
        if not result.data["energy"]["is_charging"] is None:
            print(f"robot is charging: {result.data['energy']['is_charging']}")
        else:
            print(f"robot is not charging or unknown status")

        if not result.data["energy"]["battery_level"] is None:  
            print(f"robot battery level: {result.data['energy']['battery_level']}")
        else:
            print(f"robot battery level is unknown")

        if not result.data["safety"]["emergency_stop_active"] is None:
            print(f"robot emergency stop active: {result.data['safety']['emergency_stop_active']}")
        else:
            print(f"robot emergency stop active is unknown")

        if not result.data["health"]["cpu_usage"] is None:
            print(f"robot cpu usage: {result.data['health']['cpu_usage']}")
        else:
            print(f"robot cpu usage is unknown")

        if not result.data["execution"]["operation_mode"] is None:
            print(f"robot operation mode: {result.data['execution']['operation_mode']}")
        else:
            print(f"robot operation mode is unknown")
    else:
        print(f"get robot status failed: {result.error_code}, {result.error_message}")

if __name__ == "__main__":
    typer.run(main)
