from __future__ import annotations

import time

import grpc
from x2robot import connect

from ..models.service import ServiceResult
from .arm_motion import move_by_end_pose
from .common import LogFn


def run_move_slave_arm(
    slave_server: str,
    slave_arm: str,
    log: LogFn,
    target_x: float = 0.0,
    target_y: float = 0.0,
    target_z: float = 0.0,
    target_qx: float = -0.0076,
    target_qy: float = 0.0868,
    target_qz: float = 0.0868,
    target_qw: float = 0.9924,
) -> ServiceResult:
    log("Please make sure there is enough free space around the slave arm.")
    log(f"slave server: {slave_server}")
    log(f"Connecting to slave robot at x2://{slave_server}")
    try:
        slave_robot = connect(f"x2://{slave_server}", model="quanta_x1")
    except grpc.RpcError as exc:
        return ServiceResult(False, f"slave connect failed: {exc.code().name} - {exc.details()}")

    log(f"Moving slave {slave_arm} arm via move_by_end_pose...")
    try:
        move_by_end_pose(
            slave_robot,
            slave_arm,
            log,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_qx=target_qx,
            target_qy=target_qy,
            target_qz=target_qz,
            target_qw=target_qw,
        )
        time.sleep(0.5)
    except Exception as exc:
        return ServiceResult(False, f"failed to move slave arm: {exc}")

    slave_robot.channel_manager.close()
    log("Slave arm move completed. Channel closed — waiting for SDK auto-stop (~2s), then run Step 2.")
    return ServiceResult(True, "Slave arm move completed. Disconnected, waiting for SDK auto-stop.")

