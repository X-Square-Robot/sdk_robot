#!/usr/bin/env python3
"""Small typed-SDK Head/Arm demo for the EX001 ManaEnv simulator.

It never publishes ROS directly. Each action captures the live baseline,
executes one bounded target through the ordinary SDK Client, and restores the
captured baseline. Endpoint ownership and a visible simulator remain external
preconditions.
"""

import argparse
import math
import time

import x2robot_sdk
from x2robot.sdk import (
    HeadPose,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)
from x2robot.sdk.v2 import RobotModelType


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}
ACKNOWLEDGEMENT = "SIMULATOR_TYPED_HEAD_ARM_DEMO_AND_RESTORE"


def require(result, name):
    if not isinstance(result, x2robot_sdk.SdkResult) or not result.is_success:
        raise RuntimeError(f"{name}: {getattr(result, 'error', result)!r}")
    return result.data


def enter_sdk(robot):
    global _sdk_mode_entered
    request = RobotModeParam(mode=RobotWorkMode.SDK)
    for attempt in (1, 2):
        require(robot.system.set_work_mode(request, timeout=5.0),
                f"SetWorkMode[{attempt}]")
        _sdk_mode_entered = True


def pose_of(value):
    pose = tuple(float(getattr(value, name)) for name in
                 ("x", "y", "z", "qx", "qy", "qz", "qw"))
    if not all(math.isfinite(component) for component in pose):
        raise RuntimeError(f"Arm feedback contains non-finite data: {pose!r}")
    norm = math.sqrt(sum(component * component for component in pose[3:]))
    if abs(norm - 1.0) > 1e-3:
        raise RuntimeError(
            f"Arm feedback quaternion is not normalized: norm={norm:.9f}")
    # Float32 simulator feedback may differ from unit length by a few ULPs.
    # Canonicalize it at the demo boundary as defense in depth; the SDK Server
    # now performs the same simulator-only normalization before typed reads.
    return (*pose[:3], *(component / norm for component in pose[3:]))


def sdk_pose(value):
    return x2robot_sdk.Pose(
        x=value[0], y=value[1], z=value[2],
        qx=value[3], qy=value[4], qz=value[5], qw=value[6])


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def qdistance(left, right):
    dot = abs(sum(a * b for a, b in zip(left[3:], right[3:])))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def pdistance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left[:3], right[:3])))


def read_arm(arm, label):
    return pose_of(require(arm.get_end_pose(deadline_ms=3000),
                           f"{label}.GetEndPose"))


def wait_arm(arm, expected, label, timeout=8.0):
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        latest = read_arm(arm, label)
        if pdistance(latest, expected) <= 0.007 and qdistance(latest, expected) <= 0.04:
            return latest
        time.sleep(0.12)
    raise RuntimeError(
        f"{label} feedback timeout expected={expected} latest={latest} "
        f"pos_error={pdistance(latest, expected) if latest else None}")


def arm_test(
    robot,
    side,
    delta_xyz=(0.0, 0.0, 0.018),
    yaw_delta_rad=0.0,
):
    enter_sdk(robot)
    require(
        robot.robot_control.set_manipulator_control_mode(
            ManipulatorControlModeParam(
                mode=ManipulatorControlMode.MANIPULATOR_END_POSE),
            timeout=5.0),
        "SetManipulatorControlMode(END_POSE)")
    arm = (robot.left_arm_controller if side == "left"
           else robot.right_arm_controller)
    label = f"{side.upper()}_ARM"
    baseline = read_arm(arm, label)
    require(arm.set_end_pose(sdk_pose(baseline), deadline_ms=3000),
            f"{label}.SetEndPose(same)")
    baseline = wait_arm(arm, baseline, label)
    target = list(baseline)
    for index, delta in enumerate(delta_xyz):
        target[index] += delta
    if yaw_delta_rad:
        half = yaw_delta_rad / 2.0
        delta_quaternion = (0.0, 0.0, math.sin(half), math.cos(half))
        target[3:] = quaternion_multiply(
            delta_quaternion, tuple(target[3:])
        )
    target = tuple(target)
    target_write_attempted = False
    primary = None
    try:
        # Once the target RPC begins, its delivery is ambiguous until fresh
        # feedback proves otherwise.  Always issue the single baseline
        # restore even when the RPC response itself is an error/timeout.
        target_write_attempted = True
        require(arm.set_end_pose(sdk_pose(target), deadline_ms=3000),
                f"{label}.SetEndPose(bounded-step)")
        reached = wait_arm(arm, target, label)
        observed_xyz = tuple(
            reached[index] - baseline[index] for index in range(3)
        )
        observed_orientation = qdistance(reached, baseline)
        requested_translation = math.sqrt(sum(value * value for value in delta_xyz))
        if requested_translation and pdistance(reached, baseline) < max(
            0.006, requested_translation * 0.45
        ):
            raise RuntimeError(
                f"{label} translation response was too small: "
                f"requested={delta_xyz!r} observed={observed_xyz!r}"
            )
        if yaw_delta_rad and observed_orientation < max(
            0.04, abs(yaw_delta_rad) * 0.45
        ):
            raise RuntimeError(
                f"{label} orientation response was too small: "
                f"requested={yaw_delta_rad:+.6f}rad "
                f"observed={observed_orientation:.6f}rad"
            )
        print(
            f"{label} TARGET PASS requested_xyz={delta_xyz!r} "
            f"observed_xyz={observed_xyz!r} "
            f"requested_yaw={yaw_delta_rad:+.6f}rad "
            f"observed_orientation={observed_orientation:.6f}rad"
        )
    except BaseException as error:
        primary = error
    finally:
        if target_write_attempted:
            try:
                require(arm.set_end_pose(sdk_pose(baseline), deadline_ms=3000),
                        f"{label}.SetEndPose(restore)")
                restored = wait_arm(arm, baseline, label)
                print(f"{label} RESTORE PASS error={pdistance(restored, baseline):.6f}m")
            except BaseException as restore_error:
                raise RuntimeError(
                    f"{label} RESTORE_UNCONFIRMED: primary={primary}; "
                    f"restore={restore_error}") from restore_error
    if primary:
        raise primary


def read_head(robot):
    value = require(robot.head.get_pose(timeout=3.0), "Head.GetPose")
    return float(value.pitch), float(value.yaw)


def wait_head(robot, expected, timeout=6.0):
    deadline = time.monotonic() + timeout
    latest = None
    while time.monotonic() < deadline:
        latest = read_head(robot)
        if max(abs(a - b) for a, b in zip(latest, expected)) <= 0.012:
            return latest
        time.sleep(0.12)
    raise RuntimeError(f"HEAD timeout expected={expected} latest={latest}")


def head_test(robot):
    enter_sdk(robot)
    baseline = read_head(robot)
    require(robot.head.set_pose(HeadPose(pitch=baseline[0], yaw=baseline[1]),
                                timeout=3.0), "Head.SetPose(same)")
    baseline = wait_head(robot, baseline)
    target = (baseline[0], baseline[1] + 0.05)
    target_write_attempted = False
    primary = None
    try:
        # Treat call entry as a possible write.  A failed/ambiguous response
        # must not suppress the one bounded restoration attempt.
        target_write_attempted = True
        require(robot.head.set_pose(HeadPose(pitch=target[0], yaw=target[1]),
                                    timeout=3.0), "Head.SetPose(yaw+0.05)")
        reached = wait_head(robot, target)
        observed = reached[1] - baseline[1]
        if observed < 0.03:
            raise RuntimeError(f"HEAD moved only {observed:.6f}rad")
        print(f"HEAD TARGET PASS commanded=+0.050rad observed={observed:+.6f}rad")
    except BaseException as error:
        primary = error
    finally:
        if target_write_attempted:
            try:
                require(robot.head.set_pose(
                    HeadPose(pitch=baseline[0], yaw=baseline[1]), timeout=3.0),
                    "Head.SetPose(restore)")
                restored = wait_head(robot, baseline)
                print("HEAD RESTORE PASS "
                      f"pitch_error={restored[0]-baseline[0]:+.6f} "
                      f"yaw_error={restored[1]-baseline[1]:+.6f}")
            except BaseException as restore_error:
                raise RuntimeError(
                    f"HEAD RESTORE_UNCONFIRMED: primary={primary}; "
                    f"restore={restore_error}") from restore_error
    if primary:
        raise primary


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("part", choices=("head", "left-arm", "right-arm"))
    parser.add_argument("--server", default="127.0.0.1:15551")
    parser.add_argument("--arm-dx", type=float, default=0.0)
    parser.add_argument("--arm-dy", type=float, default=0.0)
    parser.add_argument("--arm-dz", type=float, default=0.018)
    parser.add_argument("--arm-yaw", type=float, default=0.0)
    parser.add_argument("--allow-write", action="store_true")
    parser.add_argument("--confirm-simulator-only", action="store_true")
    parser.add_argument("--acknowledgement", default="")
    args = parser.parse_args()
    host, separator, port = args.server.rpartition(":")
    if not separator or host.lower() not in LOOPBACK_HOSTS or not port.isdigit():
        raise SystemExit("--server must be a loopback SSH-tunnel host:port")
    if not args.allow_write or not args.confirm_simulator_only:
        raise SystemExit(
            "demo requires --allow-write and --confirm-simulator-only")
    if args.acknowledgement != ACKNOWLEDGEMENT:
        raise SystemExit(
            "demo requires --acknowledgement " + ACKNOWLEDGEMENT)
    arm_deltas = (args.arm_dx, args.arm_dy, args.arm_dz)
    if not all(math.isfinite(value) for value in (*arm_deltas, args.arm_yaw)):
        raise SystemExit("arm deltas must be finite")
    if math.sqrt(sum(value * value for value in arm_deltas)) > 0.05:
        raise SystemExit("arm translation demo step must be <= 0.05m")
    if abs(args.arm_yaw) > 0.20:
        raise SystemExit("arm yaw demo step must be <= 0.20rad")
    if args.part != "head" and not any(
        abs(value) > 0.0 for value in (*arm_deltas, args.arm_yaw)
    ):
        raise SystemExit("arm demo requires a non-zero bounded step")
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        model = require(robot.system.get_model_type(timeout=3.0), "GetModelType")
        if int(model.model_type) != int(RobotModelType.QUANTA_X1_PRO):
            raise RuntimeError(f"expected EX001, got {model.model_type!r}")
        if args.part == "head":
            head_test(robot)
        else:
            arm_test(
                robot,
                args.part.split("-")[0],
                arm_deltas,
                args.arm_yaw,
            )
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "close")


if __name__ == "__main__":
    main()
