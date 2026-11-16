#!/usr/bin/env python3
"""Guarded typed-SDK in-place yaw demo for the EX001 ManaEnv simulator.

The current ManaEnv scene turns at approximately twice the requested SDK yaw
rate.  The demonstration uses a deliberately small ``0.08 rad/s`` one-second
lease, then applies one symmetric reverse lease and verifies a fresh stop.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import x2robot_sdk
from x2robot.sdk import (
    ChassisControlMode,
    ChassisControlModeParam,
    ChassisVelocity,
    RobotModeParam,
    RobotWorkMode,
)
from x2robot.sdk.v2 import RobotModelType


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}
YAW_RADPS = 0.08
DURATION_SECONDS = 1.00
REFRESH_SECONDS = 0.08
MAX_REFRESH_SECONDS = 0.15
STOP_LINEAR_MPS = 0.02
STOP_ANGULAR_RADPS = 0.03
ACKNOWLEDGEMENT = (
    "SIMULATOR_CHASSIS_YAW_0.08RADPS_1.00S_REVERSE_RESTORE_AND_STOP"
)


@dataclass(frozen=True)
class Odom:
    x: float
    y: float
    yaw: float
    linear_x: float
    linear_y: float
    angular_z: float
    stamp_ns: int


class StopUnconfirmedError(RuntimeError):
    pass


class MotionFailedStoppedError(RuntimeError):
    pass


def require(result, operation):
    if not isinstance(result, x2robot_sdk.SdkResult):
        raise TypeError(f"{operation}: expected SdkResult")
    if not result.is_success:
        raise RuntimeError(f"{operation}: {result.error}")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close is not None:
        close()


def parse_odom(value):
    stamp = value.header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    position = value.pose.pose.position
    orientation = value.pose.pose.orientation
    twist = value.twist.twist
    values = (
        float(position.x),
        float(position.y),
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
        float(twist.linear.x),
        float(twist.linear.y),
        float(twist.angular.z),
    )
    if stamp_ns <= 0 or not all(math.isfinite(item) for item in values):
        raise RuntimeError("odometry timestamp/value contract failed")
    x, y, qx, qy, qz, qw, linear_x, linear_y, angular_z = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not 0.95 <= norm <= 1.05:
        raise RuntimeError(f"odometry quaternion norm is invalid: {norm}")
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return Odom(x, y, yaw, linear_x, linear_y, angular_z, stamp_ns)


def stopped(sample):
    return (
        math.hypot(sample.linear_x, sample.linear_y) <= STOP_LINEAR_MPS
        and abs(sample.angular_z) <= STOP_ANGULAR_RADPS
    )


def capture_stopped(robot, newer_than_ns, label, timeout=8.0):
    stream = robot.chassis.get_odometry_stream(timeout=timeout)
    samples = []
    previous = newer_than_ns
    try:
        for result in stream:
            sample = parse_odom(require(result, "Chassis.GetOdometryStream"))
            if sample.stamp_ns <= previous:
                raise RuntimeError(
                    f"{label}: timestamp did not advance: "
                    f"{previous} -> {sample.stamp_ns}"
                )
            previous = sample.stamp_ns
            if stopped(sample):
                samples.append(sample)
                if len(samples) == 3:
                    latest = samples[-1]
                    print(
                        f"{label} PASS stamp_ns={latest.stamp_ns} "
                        f"pose=({latest.x:.6f},{latest.y:.6f},"
                        f"{latest.yaw:.6f})"
                    )
                    return latest
            else:
                samples.clear()
    finally:
        close_stream(stream)
    raise RuntimeError(f"{label}: stream ended before three stopped samples")


def set_velocity(robot, yaw_radps, operation):
    if not math.isfinite(yaw_radps) or abs(yaw_radps) > 0.50:
        raise RuntimeError(f"{operation}: refusing yaw {yaw_radps!r}")
    require(
        robot.chassis.set_velocity(
            ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=yaw_radps),
            timeout=3.0,
        ),
        operation,
    )


def run_lease(robot, yaw_radps, label):
    start = time.monotonic()
    deadline = start + DURATION_SECONDS
    next_send = start
    previous_send = None
    sends = 0
    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        if now < next_send:
            time.sleep(next_send - now)
            now = time.monotonic()
        if now >= deadline:
            break
        if previous_send is not None and now - previous_send > MAX_REFRESH_SECONDS:
            raise RuntimeError(
                f"{label}: lease refresh missed: {now - previous_send:.6f}s"
            )
        set_velocity(robot, yaw_radps, f"Chassis.SetVelocity({label}#{sends + 1})")
        previous_send = now
        sends += 1
        next_send += REFRESH_SECONDS
    if sends < 2:
        raise RuntimeError(f"{label}: only {sends} lease writes")
    print(f"{label} LEASE PASS sends={sends} yaw={yaw_radps:+.2f}rad/s")


def yaw_delta(start, end):
    return math.atan2(math.sin(end.yaw - start.yaw), math.cos(end.yaw - start.yaw))


def distance(start, end):
    return math.hypot(end.x - start.x, end.y - start.y)


def enter_velocity_mode(robot):
    require(
        enter_sdk_mode(robot),
        "System.SetWorkMode(SDK)",
    )
    require(
        robot.chassis.set_control_mode(
            ChassisControlModeParam(mode=ChassisControlMode.VELOCITY),
            timeout=3.0,
        ),
        "Chassis.SetControlMode(VELOCITY)",
    )


def exercise(robot):
    model = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType")
    if int(model.model_type) != int(RobotModelType.QUANTA_X1_PRO):
        raise RuntimeError(f"expected EX001, got {model.model_type!r}")
    baseline = capture_stopped(robot, 0, "OBSERVE_BASELINE")
    enter_velocity_mode(robot)
    write_attempted = False
    stop_confirmed = False
    target = None
    primary = None
    try:
        write_attempted = True
        set_velocity(robot, 0.0, "Chassis.SetVelocity(initial-zero)")
        baseline = capture_stopped(
            robot, baseline.stamp_ns, "INITIAL_STOP"
        )
        stop_confirmed = True
        run_lease(robot, YAW_RADPS, "positive-yaw")
        stop_confirmed = False
        set_velocity(robot, 0.0, "Chassis.SetVelocity(target-zero)")
        target = capture_stopped(
            robot, baseline.stamp_ns, "YAW_TARGET_STOP"
        )
        stop_confirmed = True
        target_yaw = yaw_delta(baseline, target)
        target_drift = distance(baseline, target)
        if not 0.05 <= target_yaw <= 0.30 or target_drift > 0.05:
            primary = RuntimeError(
                "yaw target outside verified envelope: "
                f"yaw={target_yaw:+.6f}rad translation={target_drift:.6f}m"
            )
        else:
            print(
                f"YAW_TARGET PASS yaw={target_yaw:+.6f}rad "
                f"translation={target_drift:.6f}m"
            )

        # A complete positive lease is always paired with one symmetric
        # negative lease, even if the target magnitude check failed.
        stop_confirmed = False
        run_lease(robot, -YAW_RADPS, "negative-yaw-restore")
    except BaseException as error:
        if primary is None:
            primary = error
    finally:
        if write_attempted:
            reference = target.stamp_ns if target is not None else baseline.stamp_ns
            try:
                set_velocity(robot, 0.0, "Chassis.SetVelocity(final-zero)")
                final = capture_stopped(robot, reference, "FINAL_STOP")
                stop_confirmed = True
            except BaseException as stop_error:
                raise StopUnconfirmedError(
                    f"primary={primary}; final stop={stop_error}"
                ) from stop_error

    final_yaw_error = abs(yaw_delta(baseline, final))
    final_position_error = distance(baseline, final)
    if final_yaw_error > 0.15 or final_position_error > 0.08:
        restore_error = RuntimeError(
            "reverse restoration outside tolerance: "
            f"yaw={final_yaw_error:.6f}rad "
            f"translation={final_position_error:.6f}m"
        )
        if primary is None:
            primary = restore_error
    else:
        print(
            f"YAW_RESTORE PASS yaw_error={final_yaw_error:.6f}rad "
            f"position_error={final_position_error:.6f}m"
        )
    if primary is not None:
        if stop_confirmed:
            raise MotionFailedStoppedError(str(primary)) from primary
        raise StopUnconfirmedError(str(primary)) from primary
    print("YAW_MOTION_AND_RESTORE PASS")


_sdk_mode_entered = False


def enter_sdk_mode(robot):
    """切到 SDK 模式并记录占用，退出时由 restore_idle_mode 交还应用。"""
    global _sdk_mode_entered
    result = robot.system.set_work_mode(
        RobotModeParam(mode=RobotWorkMode.SDK), timeout=5.0
    )
    _sdk_mode_entered = bool(getattr(result, "is_success", False))
    return result


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

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:15551")
    parser.add_argument("--allow-write", action="store_true")
    parser.add_argument("--confirm-simulator-only", action="store_true")
    parser.add_argument("--acknowledgement", default="")
    args = parser.parse_args()
    host, separator, port = args.server.rpartition(":")
    if not separator or host.lower() not in LOOPBACK_HOSTS or not port.isdigit():
        raise SystemExit("--server must be a loopback SSH-tunnel host:port")
    if not args.allow_write or not args.confirm_simulator_only:
        raise SystemExit("demo requires explicit write and simulator gates")
    if args.acknowledgement != ACKNOWLEDGEMENT:
        raise SystemExit("demo requires --acknowledgement " + ACKNOWLEDGEMENT)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        exercise(robot)
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


def cli():
    try:
        main()
        return 0
    except StopUnconfirmedError as error:
        print(f"STOP_UNCONFIRMED: {error}", file=sys.stderr)
        return 2
    except MotionFailedStoppedError as error:
        print(f"MOTION_FAILED_STOPPED: {error}", file=sys.stderr)
        return 1
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(cli())
