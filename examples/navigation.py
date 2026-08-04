import typer
from typing_extensions import Annotated
from x2robot import connect


app = typer.Typer(help="A CLI to interact with the robot's map operations.")


def _print_result(result):
    status = "OK" if result.success else "FAIL"
    print(f"[{status}] message={result.message!r}")


@app.command()
def load(
    map_name: Annotated[str, typer.Argument(help="Logical map name on the robot")],
    map_path: Annotated[str, typer.Argument(help="Local directory containing the map files")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    """Pack a local map directory and upload it to the robot."""
    robot = connect(f"x2://{server}")
    print(f"Connected. Uploading {map_path} as {map_name!r}")
    result = robot.navigation.load_map(map_name=map_name, map_path=map_path)
    _print_result(result)


@app.command()
def export(
    map_name: Annotated[str, typer.Argument(help="Map name to fetch from the robot")],
    target_path: Annotated[str, typer.Argument(help="Local directory to extract into")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    """Download a map from the robot and extract it into a local directory."""
    robot = connect(f"x2://{server}")
    print(f"Connected. Downloading {map_name!r} into {target_path}")
    result = robot.navigation.export_map(map_name=map_name, target_path=target_path)
    _print_result(result)


@app.command(name="list")
def list_maps(
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    """List the saved map names on the robot."""
    robot = connect(f"x2://{server}")
    result = robot.navigation.get_map_list()
    status = "OK" if result.header.is_success else "FAIL"
    print(f"[{status}] error_code={result.header.error_code} message={result.header.error_message!r}")
    if result.header.is_success:
        if result.map_list:
            print(f"{len(result.map_list)} map(s):")
            for name in result.map_list:
                print(f"  - {name}")
        else:
            print("(no maps)")


if __name__ == "__main__":
    app()
