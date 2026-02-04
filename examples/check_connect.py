from typing import Annotated
import typer
from x2robot import connect
from x2robot.sdk import PingRequest


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    request = PingRequest(payload="Hello, X2Robot!")

    response = robot.benchmark.ping(request)
    if response.payload == "Pong to: Hello, X2Robot!":
        print("Connection to X2Robot SDK server successful!")
        print(f"Response payload:[{response.payload}]")
    else:
        print("Unexpected response:", response.payload)

if __name__ == "__main__":
    typer.run(main)
