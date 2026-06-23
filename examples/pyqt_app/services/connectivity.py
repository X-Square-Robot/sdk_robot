from __future__ import annotations

from x2robot import connect
from x2robot.sdk import PingRequest

from ..models.service import ServiceResult
from .common import LogFn


def run_check_connect(server: str, log: LogFn) -> ServiceResult:
    log(f"Connecting to x2://{server}")
    robot = connect(f"x2://{server}", model="quanta_x1")
    response = robot.benchmark.ping(PingRequest(payload="Hello, X2Robot!"))
    expected = "Pong to: Hello, X2Robot!"
    if response.payload != expected:
        log(f"Unexpected response payload: {response.payload}")
        return ServiceResult(False, "Connection check returned an unexpected response.", [response.payload])

    log("Connection to X2Robot SDK server successful!")
    log(f"Response payload: [{response.payload}]")
    return ServiceResult(True, "Connection check passed.", [response.payload])
