#!/usr/bin/env python3
"""Open-source EX001 home-relative Cartesian arm acceptance.

Requires maniparena_quanta_x1_home_relative_cartesian_local and upstream
main@2b12b06 default ee. Commands and feedback are home-relative lift_link
poses, not per-message deltas. Check current scene, endpoints and ownership
before use; do not reset/restart the simulator during a move/restore window.
Defaults to a read-only stream sample. --action move/verify opts into one
bounded Z target followed by restoration, with fresh stamped feedback.
"""

import argparse
from contextlib import contextmanager
import math
import time

import x2robot_sdk
from x2robot.sdk import (
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation}: {error.get('error_name', 'UNKNOWN')}: "
            f"{error.get('error_message', '')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


@contextmanager
def managed_stream(stream):
    primary = None
    try:
        yield stream
    except BaseException as error:
        primary = error
        raise
    finally:
        try:
            result = stream.cancel()
            if result is not None:
                require(result, "stream.cancel")
        except Exception as error:
            if primary is None:
                raise
            print(f"WARN: stream cleanup failed: {error}")


class FeedbackReset(RuntimeError):
    """The saved pose no longer belongs to a confirmed feedback context."""


def pose_values(pose):
    values = tuple(float(getattr(pose, key)) for key in
                   ("x", "y", "z", "qx", "qy", "qz", "qw"))
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("Arm feedback contains non-finite values")
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if abs(norm - 1.0) > 1e-3:
        raise RuntimeError("Arm feedback quaternion is not normalized")
    return values[:3] + tuple(value / norm for value in values[3:])


def near(actual, target, position_tolerance=0.007):
    distance = math.dist(actual[:3], target[:3])
    dot = min(1.0, abs(sum(a * b for a, b in zip(actual[3:], target[3:]))))
    return distance <= position_tolerance and 2 * math.acos(dot) <= 0.04


class FreshPoseReader:
    def __init__(self, stream):
        self.stream = stream
        self.stamp = -1

    def read(self, deadline):
        while time.monotonic() < deadline:
            remaining_ms = max(1, min(1000, int((deadline - time.monotonic()) * 1000)))
            sample = require(self.stream.read(deadline_ms=remaining_ms), "Arm.EndPoseStream.read")
            if sample.frame_id != "lift_link":
                raise FeedbackReset(f"Expected lift_link feedback, got {sample.frame_id!r}")
            stamp = int(sample.stamp_ns)
            values = pose_values(sample.pose)
            if stamp < self.stamp:
                raise FeedbackReset("Arm timestamp moved backwards; possible scene/reset change")
            if stamp > self.stamp:
                self.stamp = stamp
                return values
            time.sleep(0.01)
        raise RuntimeError("Timed out waiting for advancing Arm feedback")

    def wait(self, target, timeout):
        deadline = time.monotonic() + timeout
        matched = 0
        while time.monotonic() < deadline:
            value = self.read(deadline)
            matched = matched + 1 if near(value, target) else 0
            if matched == 3:
                return value
        raise RuntimeError(f"Arm target feedback timed out: {target}")


def move_and_restore(robot, controller, side, delta_z, timeout):
    stream = require(controller.get_end_pose_stream(deadline_ms=3000), "Arm.EndPoseStream")
    with managed_stream(stream):
        reader = FreshPoseReader(stream)
        deadline = time.monotonic() + timeout
        first = reader.read(deadline)
        baseline = reader.read(deadline)
        if not near(first, baseline, position_tolerance=0.002):
            raise RuntimeError("Arm baseline is moving; stop other controllers first")
        require(enter_sdk_mode(robot), "System.SetWorkMode")
        mode = ManipulatorControlMode.MANIPULATOR_END_POSE
        require(robot.robot_control.set_manipulator_control_mode(
            ManipulatorControlModeParam(mode=mode), timeout=5.0), "SetManipulatorControlMode")
        current = require(robot.robot_control.get_manipulator_control_mode(timeout=3.0),
                          "GetManipulatorControlMode")
        if current.mode != mode:
            raise RuntimeError("Manipulator mode readback does not match END_POSE")
        # Mode transitions can take time. Recheck stability immediately before
        # capturing the home-relative pose used for both target and restoration.
        fresh = reader.read(time.monotonic() + timeout)
        if not near(fresh, baseline, position_tolerance=0.002):
            raise RuntimeError("Arm moved during mode selection")
        baseline = fresh
        target = (*baseline[:2], baseline[2] + delta_z, *baseline[3:])
        primary = None
        try:
            # Call entry means the write may have arrived even if its reply
            # fails. Issue exactly one restoration sequence in that case too.
            require(controller.set_end_pose(x2robot_sdk.Pose(*target), deadline_ms=3000),
                    f"{side}.SetEndPose(target)")
            reached = reader.wait(target, timeout)
            if math.dist(reached[:3], baseline[:3]) < abs(delta_z) * 0.45:
                raise RuntimeError("Arm feedback did not show the requested movement")
            print(f"{side}: TARGET PASS delta_z={delta_z:+.3f} m")
        except BaseException as error:
            primary = error
        if isinstance(primary, FeedbackReset):
            raise RuntimeError(
                f"RESTORE_UNCONFIRMED: {primary}; stopped writes because the saved "
                "pose's feedback context changed. Inspect the current scene before retrying."
            ) from primary
        try:
            require(controller.set_end_pose(x2robot_sdk.Pose(*baseline), deadline_ms=3000),
                    f"{side}.SetEndPose(restore)")
            reader.wait(baseline, timeout)
            print(f"{side}: RESTORE PASS")
        except BaseException as error:
            raise RuntimeError(f"RESTORE_UNCONFIRMED: primary={primary}; restore={error}") from error
        if primary is not None:
            raise primary


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

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="stream")
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="end_pose")
    parser.add_argument("--delta-z", type=float, default=0.018)
    parser.add_argument("--settle-timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    if args.action != "stream":
        if args.mode == "joint_pos":
            print("UNSUPPORTED: open-source ee SetJointPositions (100501/UNIMPLEMENTED); "
                  "no connection or writes. Use --mode end_pose or --action stream.")
            return 2
        if not math.isfinite(args.delta_z) or not 0.01 <= abs(args.delta_z) <= 0.05:
            parser.error("--delta-z magnitude must be within [0.01, 0.05] m")
    if not math.isfinite(args.settle_timeout) or not 0 < args.settle_timeout <= 120:
        parser.error("--settle-timeout must be within (0, 120] seconds")
    robot = None
    try:
        robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
        sides = ("left", "right") if args.arm == "both" else (args.arm,)
        for side in sides:
            controller = getattr(robot, f"{side}_arm_controller")
            if args.action == "stream":
                open_stream = (controller.get_end_pose_stream if args.mode == "end_pose"
                               else controller.get_joint_states_stream)
                stream = require(open_stream(deadline_ms=3000), f"{side}.stream")
                with managed_stream(stream):
                    print(require(stream.read(deadline_ms=3000), f"{side}.stream.read"))
            else:
                move_and_restore(robot, controller, side, args.delta_z, args.settle_timeout)
        print("PASS: open-source home-relative arm " + args.action)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        if robot is not None:
            try:
                restore_idle_mode(robot)
                require(robot.close(), "Robot.close")
            except Exception as error:
                print(f"FAIL: connection cleanup: {error}")
                return 1


if __name__ == "__main__":
    raise SystemExit(main())
