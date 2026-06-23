from __future__ import annotations

from x2robot import connect

from ..models.requests import ArmControlRequest
from ..models.service import ServiceResult
from .arm_motion import move_by_end_pose, move_by_joint_positions, stream_arm_end_pose, stream_arm_joint_states
from .common import CancelFn, LogFn


def run_arm_control(request: ArmControlRequest, log: LogFn, is_cancelled: CancelFn) -> ServiceResult:
    if request.action == "move":
        log("Move mode will lift the arm; please ensure the workspace is clear.")

    log(f"Connecting to x2://{request.server}")
    robot = connect(f"x2://{request.server}", model="quanta_x1")

    if request.action == "move":
        if request.mode == "joint_pos":
            move_by_joint_positions(robot, request.arm, log)
        elif request.mode == "end_pose":
            move_by_end_pose(
                robot,
                request.arm,
                log,
                target_x=request.target_x,
                target_y=request.target_y,
                target_z=request.target_z,
                target_qx=request.target_qx,
                target_qy=request.target_qy,
                target_qz=request.target_qz,
                target_qw=request.target_qw,
            )
        else:
            raise ValueError(f"Unsupported mode: {request.mode}")
        return ServiceResult(True, f"Arm {request.arm} move completed in {request.mode} mode.")

    if request.action == "stream":
        if request.mode == "joint_pos":
            stream_arm_joint_states(robot, request.arm, log, is_cancelled)
        elif request.mode == "end_pose":
            stream_arm_end_pose(robot, request.arm, log, is_cancelled)
        else:
            raise ValueError(f"Unsupported mode: {request.mode}")
        return ServiceResult(True, f"Arm {request.arm} stream stopped for {request.mode} mode.")

    raise ValueError(f"Unsupported action: {request.action}")
