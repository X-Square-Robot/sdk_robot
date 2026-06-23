from __future__ import annotations

from x2robot import connect
from x2robot.sdk import RobotModeParam, RobotWorkMode

from ..models.service import ServiceResult
from .common import LogFn


def run_robot_control(server: str, action: str, log: LogFn) -> ServiceResult:
    log(f"Connecting to x2://{server}")
    robot = connect(f"x2://{server}", model="quanta_x1")
    log("Setting robot work mode to SDK")
    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    if action == "stop":
        log("Calling emergency_stop()")
        result = robot.robot_control.emergency_stop()
    elif action == "recover":
        log("Calling recover_emergency_stop()")
        result = robot.robot_control.recover_emergency_stop()
    elif action == "homing":
        log("Calling homing()")
        result = robot.robot_control.homing()
    else:
        raise ValueError(f"Unsupported action: {action}")

    log(f"{action} result: {result.is_success}")
    summary = f"Robot action '{action}' completed." if result.is_success else f"Robot action '{action}' reported failure."
    return ServiceResult(bool(result.is_success), summary, [f"is_success={result.is_success}"])
