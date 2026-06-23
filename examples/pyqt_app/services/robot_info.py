from __future__ import annotations

import json

import grpc
from x2robot import Robot, connect
from x2robot.sdk import ManipulatorControlModeParam

from ..models.service import ServiceResult
from .common import CancelFn, LogFn


def _select_master_arm_stub(robot: Robot, arm: str):
    if arm == "left":
        return robot.master_left_arm
    if arm == "right":
        return robot.master_right_arm
    raise ValueError("Invalid arm. Valid options: left, right")


def _format_msg(msg) -> list[str]:
    return json.dumps(json.loads(msg.to_json()), indent=2, ensure_ascii=False).splitlines()


def _connect(server: str) -> Robot:
    return connect(f"x2://{server}", model="quanta_x1")


# ── unary queries ────────────────────────────────────────────────

_MASTER_UNARY_METHODS = {
    "GetControlMode": lambda stub: stub.get_control_mode(),
    "GetJointStates": lambda stub: stub.get_joint_states(),
    "GetEndPose": lambda stub: stub.get_end_pose(),
    "GetGripperPosition": lambda stub: stub.get_gripper_position(),
}


def run_unary_query(server: str, arm: str, method: str) -> ServiceResult:
    try:
        robot = _connect(server)
        stub = _select_master_arm_stub(robot, arm)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"connect failed: {exc.code().name} - {exc.details()}")

    try:
        result = _MASTER_UNARY_METHODS[method](stub)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"{method} RPC failed: {exc.code().name} - {exc.details()}")

    return ServiceResult(True, f"{method} OK", details=_format_msg(result))


# ── set control mode ─────────────────────────────────────────────

def run_set_control_mode(server: str, arm: str, mode: int) -> ServiceResult:
    try:
        robot = _connect(server)
        stub = _select_master_arm_stub(robot, arm)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"connect failed: {exc.code().name} - {exc.details()}")

    param = ManipulatorControlModeParam(mode=mode)
    try:
        result = stub.set_control_mode(param)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"SetControlMode RPC failed: {exc.code().name} - {exc.details()}")

    return ServiceResult(True, f"SetControlMode(mode={mode}) OK", details=_format_msg(result))


# ── stream queries ───────────────────────────────────────────────

_MASTER_STREAM_METHODS = {
    "GetJointStatesStream": lambda stub: stub.get_joint_states_stream(),
    "GetEndPoseStream": lambda stub: stub.get_end_pose_stream(),
    "GetGripperStateStream": lambda stub: stub.get_gripper_state_stream(),
}


def run_stream_query(server: str, arm: str, method: str, log: LogFn, is_cancelled: CancelFn) -> ServiceResult:
    try:
        robot = _connect(server)
        stub = _select_master_arm_stub(robot, arm)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"connect failed: {exc.code().name} - {exc.details()}")

    try:
        for msg in _MASTER_STREAM_METHODS[method](stub):
            if is_cancelled():
                return ServiceResult(True, f"{method} stream stopped.")
            for line in _format_msg(msg):
                log(line)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"{method} stream failed: {exc.code().name} - {exc.details()}")

    return ServiceResult(True, f"{method} stream ended.")


# ── system queries ───────────────────────────────────────────────

def run_get_model_type(server: str) -> ServiceResult:
    try:
        robot = _connect(server)
    except grpc.RpcError as exc:
        return ServiceResult(False, f"connect failed: {exc.code().name} - {exc.details()}")

    try:
        result = robot.system.get_model_type()
    except grpc.RpcError as exc:
        return ServiceResult(False, f"GetModelType RPC failed: {exc.code().name} - {exc.details()}")

    return ServiceResult(True, f"GetModelType OK: model_type={result.model_type}", details=_format_msg(result))
