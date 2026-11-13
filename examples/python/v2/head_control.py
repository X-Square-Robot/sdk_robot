"""Head pose and joint stream example."""

from __future__ import annotations

import argparse
import time

from x2robot.sdk import HeadPose, RobotModeParam, RobotWorkMode

import x2robot_sdk


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation} failed [{error.get('error_name', 'UNKNOWN')}]: "
            f"{error.get('error_message', 'SDK call failed')} "
            f"(request_id={getattr(result, 'request_id', None) or '-'})")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def stream_head_joint_states(robot, seconds=None):
    """Print head joint states until Ctrl+C (or ``seconds`` elapses).

    The v1 example intentionally keeps this command alive.  A finite duration
    is useful for CI and scripted smoke tests, while the default preserves the
    interactive Ctrl+C behaviour.
    """
    print("Starting head joint states streaming...")
    print("Press Ctrl+C to stop streaming")
    stream = robot.head.get_joint_states_stream(
        timeout=None if seconds is None else max(4.0, float(seconds) + 1.0)
    )
    started = time.monotonic()
    try:
        for result in stream:
            print(f"joint_state: {require(result, 'Head.GetJointStatesStream')}")
            if seconds is not None and time.monotonic() - started >= seconds:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping streaming...")
    finally:
        close_stream(stream)


def sdk_mode(robot):
    require(enter_sdk_mode(robot), "System.SetWorkMode")


def verify(robot, _model="auto"):
    if _model == "quanta_x3":
        # quanta_x3 uses a read-only aggregate check of pose/state streams.
        pose = require(robot.head.get_pose(timeout=3.0), "Head.GetPose")
        print(f"head pose: pitch={pose.pitch}, yaw={pose.yaw}")
        stream = robot.head.get_joint_states_stream(timeout=4.0)
        try:
            sample = require(next(iter(stream)), "Head.GetJointStatesStream")
            if len(sample.position) < 2:
                raise AssertionError("head stream returned no joints")
        finally:
            close_stream(stream)
        return

    sdk_mode(robot)
    target = HeadPose(pitch=0.12, yaw=-0.18)
    require(robot.head.set_pose(target, timeout=3.0), "Head.SetPose")
    time.sleep(0.12)
    pose = require(robot.head.get_pose(timeout=3.0), "Head.GetPose")
    if abs(pose.pitch - target.pitch) > 1e-5 or abs(pose.yaw - target.yaw) > 1e-5:
        raise AssertionError("head pose readback mismatch")
    stream = robot.head.get_joint_states_stream(timeout=4.0)
    try:
        sample = require(next(iter(stream)), "Head.GetJointStatesStream")
        if len(sample.position) < 2:
            raise AssertionError("head stream returned no joints")
    finally:
        close_stream(stream)
    require(robot.head.reset(timeout=3.0), "Head.Reset")


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
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="move")
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot, getattr(robot, "model_name", "auto"))
        elif args.action == "move":
            sdk_mode(robot)
            target = HeadPose(pitch=0.0, yaw=0.5)
            require(robot.head.set_pose(target, timeout=3.0), "Head.SetPose")
            time.sleep(0.12)
            print(require(robot.head.get_pose(timeout=3.0), "Head.GetPose"))
            require(robot.head.reset(timeout=3.0), "Head.Reset")
        else:
            stream_head_joint_states(robot, args.seconds)
        print("PASS: head control")
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
