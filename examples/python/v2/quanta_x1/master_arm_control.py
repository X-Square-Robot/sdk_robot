"""Quanta X1 master-arm control (v2 ``SdkResult`` API).

The options intentionally mirror the desktop/X1 v1 example: read/snapshot,
set-mode, move, zero and stream.  This file is self-contained so it can be
copied to a robot without importing an example-specific controller module.
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
import x2robot_sdk
from x2robot.geometry_msgs import Point, Pose, Quaternion
from x2robot.sdk import JointPositions, ManipulatorControlMode, ManipulatorControlModeParam
from x2robot.sdk import RobotModeParam, RobotWorkMode
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


MASTER_JOINT_LOWER = np.array([-2.7925, 0.0, -2.7925, -1.5708, -1.5708, -1.9199])
MASTER_JOINT_UPPER = np.array([2.7925, 3.6652, 0.0, 1.5708, 1.5708, 1.9199])
TRAJECTORY_RATE_HZ = 200.0


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation} failed: {error.get('error_name', 'UNKNOWN')}: "
            f"{error.get('error_message', 'SDK call failed')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def get_master(robot, side):
    return robot.master_left_arm if side == "left" else robot.master_right_arm


def set_mode(master, mode):
    values = {
        "joint_pos": ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS,
        "end_pose": ManipulatorControlMode.MANIPULATOR_END_POSE,
        "gravity": ManipulatorControlMode.MANIPULATOR_GRAVITY_COMPENSATION,
    }
    if mode not in values:
        raise ValueError("mode must be joint_pos, end_pose, or gravity")
    require(master.set_control_mode(ManipulatorControlModeParam(mode=values[mode]), timeout=3.0), "MasterArm.SetControlMode")


def read_master(master, query="all"):
    methods = {
        "control_mode": master.get_control_mode,
        "joint_states": master.get_joint_states,
        "end_pose": master.get_end_pose,
        "gripper_position": master.get_gripper_position,
    }
    key = query.replace("-", "_")
    selected = methods.items() if key == "all" else ((key, methods[key]),)
    if key != "all" and key not in methods:
        raise ValueError("invalid query")
    for name, method in selected:
        print(f"{name}: {require(method(timeout=3.0), f'MasterArm.Get{name.title()}')}")


def validate_rate(rate_hz):
    if rate_hz <= 0 or rate_hz > 500:
        raise ValueError("trajectory rate must be in (0, 500]")


def validate_target_positions(target):
    if len(target) != 6:
        raise ValueError(f"expected 6 joint positions, got {len(target)}")
    q_end = np.array(target, dtype=float)
    if not np.all(np.isfinite(q_end)):
        raise ValueError("target joint positions must be finite")
    below = q_end < MASTER_JOINT_LOWER
    above = q_end > MASTER_JOINT_UPPER
    if np.any(below) or np.any(above):
        bad = int(np.where(below | above)[0][0])
        raise ValueError(
            f"joint{bad + 1} target {q_end[bad]} is outside "
            f"[{MASTER_JOINT_LOWER[bad]}, {MASTER_JOINT_UPPER[bad]}]"
        )


def _pose_from_stamped(stamped):
    pose = stamped.pose if hasattr(stamped, "pose") else stamped
    return Pose(
        position=Point(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
        ),
        orientation=Quaternion(
            x=float(pose.orientation.x),
            y=float(pose.orientation.y),
            z=float(pose.orientation.z),
            w=float(pose.orientation.w),
        ),
    )


def _zero_end_pose():
    return Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _target_end_pose(original, delta, target_text, axis="z"):
    if target_text:
        values = [float(value.strip()) for value in target_text.split(",") if value.strip()]
        if len(values) != 7:
            raise ValueError("target-pose expects x,y,z,qx,qy,qz,qw")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target-pose values must be finite")
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm < 1e-9:
            raise ValueError("target-pose quaternion must be non-zero")
        return Pose(
            position=Point(x=values[0], y=values[1], z=values[2]),
            orientation=Quaternion(
                x=values[3] / norm,
                y=values[4] / norm,
                z=values[5] / norm,
                w=values[6] / norm,
            ),
        )
    position = {
        "x": float(original.position.x),
        "y": float(original.position.y),
        "z": float(original.position.z),
    }
    position[axis] += delta
    return Pose(
        position=Point(x=position["x"], y=position["y"], z=position["z"]),
        orientation=original.orientation,
    )


def move_arm_joints_toppra(master, target_positions, rate_hz, v_max=1.0, a_max=3.0):
    """TOPPRA joint trajectory matching the v1 quanta_x1 example."""
    validate_rate(rate_hz)
    validate_target_positions(target_positions)

    current_state = require(
        master.get_joint_states(timeout=3.0), "MasterArm.GetJointStates"
    )
    q_start = np.array(current_state.position, dtype=float)
    q_end = np.array(target_positions, dtype=float)
    if q_start.shape != q_end.shape or not np.all(np.isfinite(q_start)):
        raise RuntimeError("MasterArm.GetJointStates returned invalid joint positions")
    q_start = np.clip(q_start, MASTER_JOINT_LOWER, MASTER_JOINT_UPPER)

    path = ta.SplineInterpolator([0, 1], np.stack([q_start, q_end]))
    constraints = [
        constraint.JointVelocityConstraint([v_max] * len(q_start)),
        constraint.JointAccelerationConstraint([a_max] * len(q_start)),
    ]
    trajectory = algo.TOPPRA(constraints, path).compute_trajectory(0, 0)
    if trajectory is None:
        raise RuntimeError("TOPPRA joint planning failed")

    dt = 1.0 / rate_hz
    command_count = 0
    for timestamp in np.arange(0.0, trajectory.duration, dt):
        command = np.clip(
            np.asarray(trajectory(float(timestamp)), dtype=float),
            MASTER_JOINT_LOWER,
            MASTER_JOINT_UPPER,
        )
        require(
            master.set_joint_positions(
                JointPositions(positions=command.tolist()), timeout=3.0
            ),
            "MasterArm.SetJointPositions",
        )
        command_count += 1
        time.sleep(dt)

    require(
        master.set_joint_positions(
            JointPositions(positions=q_end.tolist()), timeout=3.0
        ),
        "MasterArm.SetJointPositions(final)",
    )
    return command_count + 1


def move_arm_endpose_toppra(master, target_pose, rate_hz, v_max=2.2, a_max=0.3):
    """TOPPRA Cartesian line with SLERP orientation, matching v1."""
    validate_rate(rate_hz)
    start_pose = _pose_from_stamped(
        require(master.get_end_pose(timeout=3.0), "MasterArm.GetEndPose")
    )
    p_start = np.array(
        [start_pose.position.x, start_pose.position.y, start_pose.position.z],
        dtype=float,
    )
    p_end = np.array(
        [target_pose.position.x, target_pose.position.y, target_pose.position.z],
        dtype=float,
    )
    q_start = [
        start_pose.orientation.x,
        start_pose.orientation.y,
        start_pose.orientation.z,
        start_pose.orientation.w,
    ]
    q_end = [
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w,
    ]

    path_len = float(np.linalg.norm(p_end - p_start))
    effective_path_len = path_len if path_len > 1e-6 else 1.0
    path = ta.SplineInterpolator([0, 1], np.array([[0], [effective_path_len]]))
    constraints = [
        constraint.JointVelocityConstraint([v_max]),
        constraint.JointAccelerationConstraint([a_max]),
    ]
    trajectory = algo.TOPPRA(constraints, path).compute_trajectory(0, 0)
    if trajectory is None:
        raise RuntimeError("TOPPRA end-pose planning failed")

    slerp = Slerp([0, trajectory.duration], R.from_quat([q_start, q_end]))
    dt = 1.0 / rate_hz
    command_count = 0
    for timestamp in np.arange(0.0, trajectory.duration, dt):
        distance = float(trajectory(float(timestamp))[0])
        alpha = float(np.clip(distance / effective_path_len, 0.0, 1.0))
        position = p_start + (p_end - p_start) * alpha
        orientation = slerp(float(timestamp)).as_quat()
        pose = Pose(
            position=Point(
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
            ),
            orientation=Quaternion(
                x=float(orientation[0]),
                y=float(orientation[1]),
                z=float(orientation[2]),
                w=float(orientation[3]),
            ),
        )
        require(master.set_end_pose(pose, timeout=3.0), "MasterArm.SetEndPose")
        command_count += 1
        time.sleep(dt)

    require(master.set_end_pose(target_pose, timeout=3.0), "MasterArm.SetEndPose(final)")
    return command_count + 1


def move(master, mode, joint_index, joint_delta, pose_delta, axis,
         joint_target_text, pose_target_text, hold, return_original):
    set_mode(master, mode)
    if mode == "joint_pos":
        start = [
            float(value)
            for value in require(
                master.get_joint_states(timeout=3.0), "MasterArm.GetJointStates"
            ).position
        ]
        if joint_target_text:
            target = [float(value.strip()) for value in joint_target_text.split(",") if value.strip()]
            if len(target) != len(start):
                raise ValueError(f"target must contain {len(start)} values")
        else:
            if not 0 <= joint_index < len(start):
                raise ValueError("joint-index out of range")
            target = list(start)
            target[joint_index] += joint_delta
        validate_target_positions(target)
        count = move_arm_joints_toppra(master, target, TRAJECTORY_RATE_HZ)
        print(f"sent {count} TOPPRA joint commands")
        time.sleep(hold)
        if return_original:
            original = np.clip(
                np.array(start, dtype=float), MASTER_JOINT_LOWER, MASTER_JOINT_UPPER
            ).tolist()
            count = move_arm_joints_toppra(master, original, TRAJECTORY_RATE_HZ)
            print(f"sent {count} TOPPRA return joint commands")
            time.sleep(0.5)
    elif mode == "end_pose":
        original = _pose_from_stamped(
            require(master.get_end_pose(timeout=3.0), "MasterArm.GetEndPose")
        )
        target = _target_end_pose(original, pose_delta, pose_target_text, axis)
        print(f"target end pose: {target}")
        count = move_arm_endpose_toppra(master, target, TRAJECTORY_RATE_HZ)
        print(f"sent {count} TOPPRA end-pose commands")
        time.sleep(hold)
        if return_original:
            count = move_arm_endpose_toppra(master, original, TRAJECTORY_RATE_HZ)
            print(f"sent {count} TOPPRA return end-pose commands")
            time.sleep(0.5)
    else:
        raise ValueError("move mode must be joint_pos or end_pose")


def zero(master, mode, hold):
    set_mode(master, mode)
    if mode == "joint_pos":
        state = require(master.get_joint_states(timeout=3.0), "MasterArm.GetJointStates")
        target = [0.0] * len(state.position)
        validate_target_positions(target)
        count = move_arm_joints_toppra(master, target, TRAJECTORY_RATE_HZ)
        print(f"sent {count} TOPPRA joint zero commands")
    elif mode == "end_pose":
        count = move_arm_endpose_toppra(master, _zero_end_pose(), TRAJECTORY_RATE_HZ)
        print(f"sent {count} TOPPRA end-pose zero commands")
    else:
        raise ValueError("zero mode must be joint_pos or end_pose")
    time.sleep(hold)


def stream(master, stream_name, interval, samples):
    methods = {
        "joint_states": master.get_joint_states_stream,
        "end_pose": master.get_end_pose_stream,
        "gripper_state": master.get_gripper_state_stream,
        "gripper_joint_states": master.get_gripper_joint_states_stream,
    }
    key = stream_name.replace("-", "_")
    if key not in methods:
        raise ValueError("invalid stream")
    handle = methods[key](timeout=None)
    count = 0
    try:
        for result in handle:
            print(f"{key}: {require(result, f'MasterArm.{key}')}")
            count += 1
            if samples > 0 and count >= samples:
                break
            if interval:
                time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopping stream.")
    finally:
        close_stream(handle)


def verify(robot, model="ex001"):
    if model not in ("ex001", "quanta_x1", "quanta_x1_master", "auto"):
        raise RuntimeError("this example is only for EX001/Quanta X1")
    for side in ("left", "right"):
        read_master(get_master(robot, side))




_sdk_mode_entered = False


def enter_sdk_mode(robot):
    """切到 SDK 模式并记录占用，退出时由 restore_idle_mode 交还应用。

    主臂命令和手臂共用同一道服务端门控：没进 SDK 模式时
    MasterArm.SetControlMode / SetJointPositions 会报 "Sdk mode is not running"，
    所以控制前必须先进入 SDK 模式（不再依赖机器人恰好处于 SDK 模式）。
    """
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
    parser.add_argument("--model", default="quanta_x1")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument("--action", choices=("read", "snapshot", "set-mode", "move", "zero", "stream", "verify"), default="read")
    parser.add_argument("--mode", choices=("joint-pos", "end-pose", "gravity", "joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--query", choices=("all", "control-mode", "joint-states", "end-pose", "gripper-position"), default="all")
    parser.add_argument("--stream", choices=("joint-states", "end-pose", "gripper-state", "gripper-joint-states"), default="joint-states")
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--joint-index", type=int, default=0)
    parser.add_argument("--joint-delta", type=float, default=0.262)
    parser.add_argument("--target-positions", default="")
    parser.add_argument("--pose-delta", type=float, default=0.10)
    parser.add_argument("--axis", choices=("x", "y", "z"), default="z")
    parser.add_argument("--target-pose", default="")
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--return-original", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stream-interval", type=float, default=0.0)
    parser.add_argument("--samples", type=int, default=0)
    args = parser.parse_args(argv)
    if args.hold < 0:
        raise ValueError("hold must be >= 0")
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot, args.model)
        else:
            if args.action in ("set-mode", "move", "zero") and not args.write:
                raise RuntimeError("refusing write action: pass --write")
            mode = args.mode.replace("-", "_")
            if args.action in ("set-mode", "move", "zero"):
                # 控制前先进入 SDK 模式（主臂与手臂共用同一道门控）。
                enter_sdk_mode(robot)
            selected = ("left", "right") if args.arm == "both" else (args.arm,)
            for side in selected:
                hand = get_master(robot, side)
                if args.action in ("read", "snapshot"):
                    read_master(hand, args.query)
                elif args.action == "set-mode":
                    set_mode(hand, mode)
                elif args.action == "move":
                    move(hand, mode, args.joint_index, args.joint_delta, args.pose_delta,
                         args.axis, args.target_positions, args.target_pose,
                         args.hold, args.return_original)
                elif args.action == "zero":
                    zero(hand, mode, args.hold)
                elif args.action == "stream":
                    if args.arm == "both":
                        raise RuntimeError("stream does not support --arm both")
                    stream(hand, args.stream, args.stream_interval, args.samples)
        print("PASS: Quanta X1 master-arm control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
