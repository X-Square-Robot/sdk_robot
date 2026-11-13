#!/usr/bin/env python3
"""Continuously display real simulator state through the typed SDK2 Client.

This observer is strictly read-only: it sends no lifecycle, mode, heartbeat,
or Set RPC.  Run one target per terminal through the loopback SSH tunnel.
"""

from __future__ import annotations

import argparse
import math
import time

import x2robot_sdk


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}
EXPECTED_JOINTS = {
    "left-gripper": ("left_arm_gripper",),
    "right-gripper": ("right_arm_gripper",),
    "lift": ("lift_joint",),
    "head": ("head_pitch_joint", "head_yaw_joint"),
    "left-arm": tuple(f"left_arm_joint{index}" for index in range(1, 7)),
    "right-arm": tuple(f"right_arm_joint{index}" for index in range(1, 7)),
}
TARGETS = tuple(EXPECTED_JOINTS) + ("odometry",)


def require(result, operation: str):
    if not isinstance(result, x2robot_sdk.SdkResult):
        return result
    if not result.is_success:
        error = result.error or {}
        name = error.get("error_name", "UNKNOWN")
        message = error.get("error_message", "SDK call failed")
        raise RuntimeError(f"{operation}: {name}: {message}")
    return result.data


def validate_server_address(address: str) -> str:
    normalized = address.strip()
    for prefix in ("grpc://", "x2://"):
        if normalized.startswith(prefix):
            normalized = normalized[len(prefix) :]
            break

    if normalized.startswith("["):
        closing = normalized.find("]")
        if closing < 0 or normalized[closing + 1 : closing + 2] != ":":
            raise argparse.ArgumentTypeError("server must be loopback host:port")
        host = normalized[1:closing]
        port_text = normalized[closing + 2 :]
    else:
        host, separator, port_text = normalized.rpartition(":")
        if not separator:
            raise argparse.ArgumentTypeError("server must include a TCP port")

    if host.lower() not in LOOPBACK_HOSTS:
        raise argparse.ArgumentTypeError(
            f"server must use the loopback SSH tunnel, got {host!r}"
        )
    try:
        port = int(port_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("server port must be numeric") from error
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("server port must be in 1..65535")
    return normalized


def stamp_ns(value) -> int:
    direct = getattr(value, "stamp_ns", None)
    if direct is not None:
        result = int(direct)
        if result <= 0:
            raise RuntimeError("source timestamp is not positive")
        return result
    header = getattr(value, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise RuntimeError(f"invalid source timestamp: {sec}.{nanosec}")
    result = sec * 1_000_000_000 + nanosec
    if result <= 0:
        raise RuntimeError("source timestamp is not positive")
    return result


def joint_line(target: str, value) -> tuple[int, str]:
    names = tuple(str(name) for name in value.name)
    positions = tuple(float(item) for item in value.position)
    expected = EXPECTED_JOINTS[target]
    if names != expected or len(positions) != len(expected):
        raise RuntimeError(
            f"{target}: joint identity/order {names!r} != {expected!r}"
        )
    if not all(math.isfinite(item) for item in positions):
        raise RuntimeError(f"{target}: position contains non-finite data")
    timestamp = stamp_ns(value)
    values = ", ".join(
        f"{name}={position:.6f}" for name, position in zip(names, positions)
    )
    return timestamp, f"{target} {values}"


def odometry_line(value) -> tuple[int, str]:
    timestamp = stamp_ns(value)
    position = value.pose.pose.position
    linear = value.twist.twist.linear
    angular = value.twist.twist.angular
    numbers = (
        float(position.x),
        float(position.y),
        float(position.z),
        float(linear.x),
        float(linear.y),
        float(linear.z),
        float(angular.x),
        float(angular.y),
        float(angular.z),
    )
    if not all(math.isfinite(item) for item in numbers):
        raise RuntimeError("odometry contains non-finite data")
    return timestamp, (
        "odometry "
        f"xyz=({numbers[0]:.4f},{numbers[1]:.4f},{numbers[2]:.4f}) "
        f"linear=({numbers[3]:.4f},{numbers[4]:.4f},{numbers[5]:.4f}) "
        f"angular=({numbers[6]:.4f},{numbers[7]:.4f},{numbers[8]:.4f})"
    )


class NativeArmJointStream:
    """Adapt the native arm stream handle to the iterator used by this viewer."""

    def __init__(self, handle) -> None:
        self._handle = handle

    def __iter__(self):
        return self

    def __next__(self):
        if self._handle is None:
            raise StopIteration
        return require(
            self._handle.read(deadline_ms=30_000),
            "Arm.GetJointStatesStream.read",
        )

    def close(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            cancel = getattr(handle, "cancel", None) or getattr(
                handle, "close", None
            )
            if cancel is not None:
                cancel()


def open_stream(robot, target: str, timeout: float | None):
    if target == "left-gripper":
        return robot.left_gripper.get_joint_states_stream(timeout=timeout)
    if target == "right-gripper":
        return robot.right_gripper.get_joint_states_stream(timeout=timeout)
    if target == "lift":
        return robot.lift.get_joint_states_stream(timeout=timeout)
    if target == "head":
        return robot.head.get_joint_states_stream(timeout=timeout)
    if target == "left-arm":
        first_deadline_ms = int(min(timeout or 3.0, 30.0) * 1000)
        handle = require(
            robot.left_arm_controller.get_joint_states_stream(
                deadline_ms=first_deadline_ms
            ),
            "LeftArm.GetJointStatesStream",
        )
        return NativeArmJointStream(handle)
    if target == "right-arm":
        first_deadline_ms = int(min(timeout or 3.0, 30.0) * 1000)
        handle = require(
            robot.right_arm_controller.get_joint_states_stream(
                deadline_ms=first_deadline_ms
            ),
            "RightArm.GetJointStatesStream",
        )
        return NativeArmJointStream(handle)
    if target == "odometry":
        return robot.chassis.get_odometry_stream(timeout=timeout)
    raise RuntimeError(f"unsupported target {target!r}")


def close_stream(stream) -> None:
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close is not None:
        close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--server",
        default="127.0.0.1:15551",
        type=validate_server_address,
    )
    parser.add_argument("--target", choices=TARGETS, default="left-gripper")
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop after this many seconds; 0 means until Ctrl+C",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=5.0,
        help="maximum display rate while every received frame is validated",
    )
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0.0 <= args.seconds <= 3600.0:
        parser.error("--seconds must be finite and in 0..3600")
    if not math.isfinite(args.rate_hz) or not 0.2 <= args.rate_hz <= 30.0:
        parser.error("--rate-hz must be finite and in 0.2..30")
    return args


def main() -> int:
    args = parse_args()
    robot = require(
        x2robot_sdk.connect(args.server, force_grpc=True),
        "connect",
    )
    stream = None
    previous_stamp_ns = 0
    started = time.monotonic()
    next_print = started
    completed_requested_duration = False
    timeout = None if args.seconds == 0.0 else args.seconds + 2.0
    try:
        stream = open_stream(robot, args.target, timeout)
        for result in stream:
            value = require(result, f"{args.target}.stream")
            if args.target == "odometry":
                current_stamp_ns, line = odometry_line(value)
            else:
                current_stamp_ns, line = joint_line(args.target, value)
            if current_stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"{args.target}: source timestamp did not advance; "
                    f"previous={previous_stamp_ns}, current={current_stamp_ns}"
                )
            previous_stamp_ns = current_stamp_ns

            now = time.monotonic()
            if now >= next_print:
                print(f"stamp_ns={current_stamp_ns} {line}", flush=True)
                next_print = now + 1.0 / args.rate_hz
            if args.seconds > 0.0 and now - started >= args.seconds:
                completed_requested_duration = True
                break
        if not completed_requested_duration:
            duration = time.monotonic() - started
            requested = "until Ctrl+C" if args.seconds == 0.0 else (
                f"{args.seconds:.3f}s"
            )
            raise RuntimeError(
                f"{args.target}: stream ended unexpectedly after "
                f"{duration:.3f}s; requested={requested}"
            )
    except KeyboardInterrupt:
        print("observer stopped; writes_sent=0")
        return 0
    finally:
        if stream is not None:
            close_stream(stream)
        closed = robot.close()
        if isinstance(closed, x2robot_sdk.SdkResult) and not closed.is_success:
            raise RuntimeError(f"Robot.close failed: {closed.error}")
    print("observer complete; writes_sent=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
