#!/usr/bin/env python3
"""Guarded EX001 ManaEnv Head.Reset qualification through the typed SDK.

The script captures a stable live Head baseline, calls the ordinary typed
``HeadController.Reset`` RPC, proves fresh convergence to the configured
simulator initial pose, and always restores the captured baseline.  ROS graph
ownership and the visible simulator scene remain external prerequisites.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import x2robot_sdk
from x2robot.sdk import HeadPose, RobotModeParam, RobotWorkMode
from x2robot.sdk.v2 import RobotModelType


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}
ACKNOWLEDGEMENT = (
    "SIMULATOR_HEAD_RESET_TO_CONFIGURED_INITIAL_AND_RESTORE_BASELINE"
)
INITIAL_POSE = (0.0, 0.0)
STABILITY_TOLERANCE_RAD = 0.004
POSITION_TOLERANCE_RAD = 0.012
MIN_RESET_RESPONSE_RAD = 0.03


class RestoreUnconfirmedError(RuntimeError):
    """A Head write began but captured-baseline restoration was not proved."""


class TargetFailedRestoredError(RuntimeError):
    """Reset qualification failed, but the captured baseline was restored."""


def require(result, name: str):
    if not isinstance(result, x2robot_sdk.SdkResult):
        raise TypeError(f"{name}: expected SdkResult, got {type(result).__name__}")
    if not result.is_success:
        raise RuntimeError(f"{name}: {result.error}")
    return result.data


def read_pose(robot) -> tuple[float, float]:
    value = require(robot.head.get_pose(timeout=3.0), "HeadController.GetPose")
    pose = (float(value.pitch), float(value.yaw))
    if not all(math.isfinite(component) for component in pose):
        raise RuntimeError(f"Head feedback is non-finite: {pose!r}")
    return pose


def max_error(left: tuple[float, float], right: tuple[float, float]) -> float:
    return max(abs(a - b) for a, b in zip(left, right))


def capture_stable_pose(robot, label: str) -> tuple[float, float]:
    samples = []
    for _ in range(5):
        samples.append(read_pose(robot))
        time.sleep(0.12)
    pitch_span = max(value[0] for value in samples) - min(
        value[0] for value in samples
    )
    yaw_span = max(value[1] for value in samples) - min(
        value[1] for value in samples
    )
    if max(pitch_span, yaw_span) > STABILITY_TOLERANCE_RAD:
        raise RuntimeError(
            f"{label}: Head feedback is not stable; "
            f"pitch_span={pitch_span:.6f} yaw_span={yaw_span:.6f}"
        )
    latest = samples[-1]
    print(
        f"{label} PASS samples={len(samples)} "
        f"pitch={latest[0]:+.6f} yaw={latest[1]:+.6f}"
    )
    return latest


def wait_pose(
    robot,
    expected: tuple[float, float],
    label: str,
    timeout: float,
) -> tuple[float, float]:
    deadline = time.monotonic() + timeout
    latest = None
    consecutive = []
    while time.monotonic() < deadline:
        latest = read_pose(robot)
        if max_error(latest, expected) <= POSITION_TOLERANCE_RAD:
            consecutive.append(latest)
            if len(consecutive) == 3:
                pitch_span = max(value[0] for value in consecutive) - min(
                    value[0] for value in consecutive
                )
                yaw_span = max(value[1] for value in consecutive) - min(
                    value[1] for value in consecutive
                )
                if max(pitch_span, yaw_span) <= STABILITY_TOLERANCE_RAD:
                    return consecutive[-1]
                consecutive.pop(0)
        else:
            consecutive.clear()
        time.sleep(0.12)
    raise RuntimeError(
        f"{label}: Head feedback timeout; expected={expected!r}, latest={latest!r}"
    )


def enter_sdk_mode(robot) -> None:
    global _sdk_mode_entered
    request = RobotModeParam(mode=RobotWorkMode.SDK)
    for attempt in (1, 2):
        require(
            robot.system.set_work_mode(request, timeout=5.0),
            f"System.SetWorkMode(SDK) attempt {attempt}",
        )
        _sdk_mode_entered = True
        print(f"SDK_MODE PASS attempt={attempt}/2")


def validate_server(address: str) -> str:
    normalized = address.strip()
    host, separator, port_text = normalized.rpartition(":")
    if not separator or host.lower() not in LOOPBACK_HOSTS:
        raise SystemExit("--server must be a loopback host:port endpoint")
    try:
        port = int(port_text)
    except ValueError as error:
        raise SystemExit("--server port must be numeric") from error
    if not 1 <= port <= 65535:
        raise SystemExit("--server port must be in [1, 65535]")
    return normalized


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:15551")
    parser.add_argument("--settle-timeout", type=float, default=10.0)
    parser.add_argument("--allow-write", action="store_true")
    parser.add_argument("--confirm-simulator-only", action="store_true")
    parser.add_argument("--acknowledgement", default="")
    args = parser.parse_args()
    args.server = validate_server(args.server)
    if not args.allow_write or not args.confirm_simulator_only:
        raise SystemExit(
            "qualification requires --allow-write and --confirm-simulator-only"
        )
    if args.acknowledgement != ACKNOWLEDGEMENT:
        raise SystemExit("qualification requires --acknowledgement " + ACKNOWLEDGEMENT)
    if not 3.0 <= args.settle_timeout <= 30.0:
        raise SystemExit("--settle-timeout must be in [3.0, 30.0] seconds")
    return args


def exercise(robot, timeout: float) -> None:
    model = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType")
    if int(model.model_type) != int(RobotModelType.QUANTA_X1_PRO):
        raise RuntimeError(f"expected EX001/QUANTA_X1_PRO, got {model.model_type!r}")
    print(f"MODEL PASS model={model.model_type!r} transport={robot.transport}")
    baseline = capture_stable_pose(robot, "BASELINE")
    if not (-0.60 <= baseline[0] <= 0.90 and -1.20 <= baseline[1] <= 1.20):
        raise RuntimeError(f"captured baseline is outside simulator limits: {baseline!r}")
    enter_sdk_mode(robot)
    prewrite = capture_stable_pose(robot, "PREWRITE_BASELINE")
    if max_error(prewrite, baseline) > POSITION_TOLERANCE_RAD:
        raise RuntimeError(
            f"Head changed during mode gate: before={baseline!r}, after={prewrite!r}"
        )
    baseline = prewrite

    primary_error: BaseException | None = None
    write_attempted = False
    restoration_confirmed = False
    try:
        write_attempted = True
        require(robot.head.reset(timeout=3.0), "HeadController.Reset")
        reached = wait_pose(robot, INITIAL_POSE, "RESET_TARGET", timeout)
        response = max_error(reached, baseline)
        if max_error(baseline, INITIAL_POSE) >= MIN_RESET_RESPONSE_RAD and response < MIN_RESET_RESPONSE_RAD:
            raise RuntimeError(
                f"Reset response was too small: baseline={baseline!r}, reached={reached!r}"
            )
        print(
            "HEAD_RESET TARGET PASS "
            f"pitch={reached[0]:+.6f} yaw={reached[1]:+.6f} "
            f"response={response:.6f}rad"
        )
    except BaseException as error:
        primary_error = error
    finally:
        if write_attempted:
            try:
                require(
                    robot.head.set_pose(
                        HeadPose(pitch=baseline[0], yaw=baseline[1]), timeout=3.0
                    ),
                    "HeadController.SetPose(restore-baseline)",
                )
                restored = wait_pose(robot, baseline, "RESTORE", timeout)
                restoration_confirmed = True
                print(
                    "HEAD_RESET RESTORE PASS "
                    f"pitch_error={restored[0] - baseline[0]:+.6f} "
                    f"yaw_error={restored[1] - baseline[1]:+.6f}"
                )
            except BaseException as restore_error:
                raise RestoreUnconfirmedError(
                    f"Reset result={primary_error}; baseline restore={restore_error}"
                ) from restore_error

    if primary_error is not None:
        if restoration_confirmed:
            raise TargetFailedRestoredError(str(primary_error)) from primary_error
        raise RestoreUnconfirmedError(str(primary_error)) from primary_error
    print("HEAD_RESET_AND_RESTORE PASS")


_sdk_mode_entered = False


def restore_idle_mode(robot):
    """只交还本示例占用的 SDK 模式：/application/task_info.type 回到 0。"""
    global _sdk_mode_entered
    if not _sdk_mode_entered:
        return
    _sdk_mode_entered = False
    try:
        robot.system.set_work_mode(
            RobotModeParam(mode=RobotWorkMode.IDLE), timeout=5.0
        )
    except Exception as error:  # 清理失败不能掩盖示例本身的错误
        print(f"WARN: failed to leave SDK mode: {error}")


def main() -> int:
    args = parse_args()
    robot = require(
        x2robot_sdk.connect(args.server, force_grpc=True), "connect"
    )
    try:
        exercise(robot, args.settle_timeout)
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")
    return 0


def cli() -> int:
    try:
        return main()
    except RestoreUnconfirmedError as error:
        print(f"RESTORE_UNCONFIRMED: {error}", file=sys.stderr)
        return 2
    except TargetFailedRestoredError as error:
        print(f"TARGET_FAILED_RESTORED: {error}", file=sys.stderr)
        return 1
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(cli())
