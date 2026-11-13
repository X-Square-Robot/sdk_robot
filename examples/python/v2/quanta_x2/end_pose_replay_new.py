"""Replay Quanta X2 end poses from an episode using the v2 API.

This keeps the v1 command-line contract (arm selection, waist/gripper toggles,
speed and dry-run) while replacing the legacy Robot calls with checked
``x2robot_sdk`` requests.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import x2robot_sdk
from x2robot.geometry_msgs import Point, Pose, Quaternion
from x2robot.sdk import (
    GripperPosition,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation} failed: {error.get('error_name', 'UNKNOWN')}: "
            f"{error.get('error_message', 'SDK call failed')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def load_episode(path):
    source = Path(path)
    if source.is_dir():
        source /= "episode.json"
    if not source.exists():
        raise FileNotFoundError(source)
    data = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(data.get("frames"), list) or not data["frames"]:
        raise ValueError("episode contains no frames")
    return data


def finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def valid_pose(value):
    if not isinstance(value, dict):
        return False
    position, orientation = value.get("position"), value.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return False
    values = [position.get(key) for key in ("x", "y", "z")] + [orientation.get(key) for key in ("x", "y", "z", "w")]
    return all(finite(item) for item in values) and not (
        position["x"] == position["y"] == position["z"] == orientation["x"] == orientation["y"] == orientation["z"] == 0
        and orientation["w"] == 1
    )


def pose_from_dict(value):
    position, orientation = value["position"], value["orientation"]
    return Pose(
        position=Point(x=float(position["x"]), y=float(position["y"]), z=float(position["z"])),
        orientation=Quaternion(x=float(orientation["x"]), y=float(orientation["y"]), z=float(orientation["z"]), w=float(orientation["w"])),
    )


def pose_for(frame, part):
    action = frame.get("action") or {}
    observation = frame.get("observation") or {}
    candidate = action.get(f"{part}_end_pose_action")
    if valid_pose(candidate):
        return candidate
    candidate = observation.get(f"{part}_end_pose")
    return candidate if valid_pose(candidate) else None


def gripper_for(frame, side):
    action, observation = frame.get("action") or {}, frame.get("observation") or {}
    value = action.get(f"{side}_gripper_position_action")
    if isinstance(value, dict) and finite(value.get("position")):
        return max(0.0, float(value["position"]))
    value = observation.get(f"{side}_gripper_joint_states")
    if isinstance(value, dict) and value.get("positions"):
        return max(0.0, float(value["positions"][0]))
    value = observation.get(f"{side}_gripper_position")
    if isinstance(value, dict) and finite(value.get("position")):
        return max(0.0, float(value["position"]))
    return None


def selected(arm):
    return ("left_arm", "right_arm") if arm == "both" else (f"{arm}_arm",)


def dry_run_episode(data, arm, speed, include_waist, include_gripper):
    parts = selected(arm) + (("waist",) if include_waist else ())
    grippers = ("left", "right") if arm == "both" and include_gripper else ((arm,) if include_gripper else ())
    pose_count = sum(sum(pose_for(frame, part) is not None for part in parts) for frame in data["frames"])
    gripper_count = sum(sum(gripper_for(frame, side) is not None for side in grippers) for frame in data["frames"])
    first, last = data["frames"][0].get("timestamp"), data["frames"][-1].get("timestamp")
    duration = (last - first) / speed if finite(first) and finite(last) else 0.0
    print(f"Dry-run: frames={len(data['frames'])}, pose_commands={pose_count}, gripper_commands={gripper_count}, estimated_time={max(0.0, duration):.2f}s")


def replay_end_pose(robot, data, arm, speed, include_waist, include_gripper, dry_run):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE), timeout=3.0), "RobotControl.SetManipulatorControlMode")
    parts = selected(arm) + (("waist",) if include_waist else ())
    grippers = ("left", "right") if arm == "both" and include_gripper else ((arm,) if include_gripper else ())
    frames = data["frames"]
    sent = 0
    started = time.monotonic()
    for index, frame in enumerate(frames):
        frame_sent = False
        for part in parts:
            value = pose_for(frame, part)
            if value is None:
                continue
            if not dry_run:
                service = getattr(robot, f"{part}_controller", None) if part.endswith("_arm") else robot.waist
                if part.endswith("_arm"):
                    require(service.set_end_pose(x2robot_sdk.Pose(
                        value["position"]["x"], value["position"]["y"], value["position"]["z"],
                        value["orientation"]["x"], value["orientation"]["y"], value["orientation"]["z"], value["orientation"]["w"],
                    ), deadline_ms=1000), f"{part}.SetEndPose")
                else:
                    require(service.set_end_pose(pose_from_dict(value), timeout=1.0), "Waist.SetEndPose")
            frame_sent = True
        for side in grippers:
            position = gripper_for(frame, side)
            if position is None:
                continue
            if not dry_run:
                service = robot.left_gripper if side == "left" else robot.right_gripper
                require(service.set_position(GripperPosition(position=position), timeout=1.0), f"{side.title()}Gripper.SetPosition")
            frame_sent = True
        sent += int(frame_sent)
        if index % 10 == 0 or index == len(frames) - 1:
            print(f"Progress: {(index + 1) / len(frames) * 100:5.1f}% ({index + 1}/{len(frames)})", end="\r")
        if index + 1 < len(frames):
            current, following = frame.get("timestamp"), frames[index + 1].get("timestamp")
            if finite(current) and finite(following):
                delay = (following - current) / speed
                if delay > 0:
                    time.sleep(delay)
    print(f"\nReplay completed: sent {sent}/{len(frames)} frames in {time.monotonic() - started:.2f}s")


def verify(robot, model="cx002"):
    if model not in ("cx002", "auto"):
        raise RuntimeError("this example is only for CX002/Quanta X2")
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE), timeout=3.0), "RobotControl.SetManipulatorControlMode")
    for side in ("left", "right"):
        controller = robot.left_arm_controller if side == "left" else robot.right_arm_controller
        target = x2robot_sdk.Pose(x=0.31, y=0.07 if side == "left" else -0.07, z=0.72, qw=1.0)
        require(controller.set_end_pose(target, deadline_ms=1000), f"{side}.SetEndPose")
        time.sleep(0.1)
        pose = require(controller.get_end_pose(deadline_ms=1000), f"{side}.GetEndPose")
        if any(abs(getattr(pose, field) - getattr(target, field)) > 1e-5 for field in ("x", "y", "z")):
            raise AssertionError(f"{side} pose readback mismatch")


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
    parser.add_argument("episode", nargs="?")
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--include-waist", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--include-gripper", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--verify", action="store_true")
    args = parser.parse_args(argv)
    if args.speed <= 0:
        parser.error("--speed must be > 0")
    if args.verify:
        robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
        try:
            verify(robot)
            print("PASS: Quanta X2 end-pose replay")
            return 0
        finally:
            require(robot.close(), "Robot.close")
    if not args.episode:
        parser.error("episode path is required unless --verify is used")
    data = load_episode(args.episode)
    if args.dry_run:
        dry_run_episode(data, args.arm, args.speed, args.include_waist, args.include_gripper)
        return 0
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        replay_end_pose(robot, data, args.arm, args.speed, args.include_waist, args.include_gripper, False)
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
