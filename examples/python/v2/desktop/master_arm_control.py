"""Desktop master-arm example.

This is the v2 counterpart of ``python/v1/desktop/master_arm_control.py``.
The command names and safety switches intentionally stay the same; every RPC
is made through the standardized ``x2robot_sdk`` service and every response is
checked as an ``SdkResult``.
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
from x2robot.sdk import (
    JointPositions,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


# Keep the desktop v2 motion profile identical to the quanta_x1 example.
MASTER_JOINT_LOWER = np.array([-2.7925, 0.0, -2.7925, -1.5708, -1.5708, -1.9199])
MASTER_JOINT_UPPER = np.array([2.7925, 3.6652, 0.0, 1.5708, 1.5708, 1.9199])
JOINT_FEEDBACK_LIMIT_TOLERANCE = 1e-3
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


def master(robot, side):
    return robot.master_left_arm if side == "left" else robot.master_right_arm


def sides(value):
    if value == "both":
        return ("left", "right")
    if value not in ("left", "right"):
        raise ValueError("arm must be left, right, or both")
    return (value,)


def print_message(title, value):
    print(f"\n== {title} ==")
    print(value)


def read_master_arm(hand, query="all"):
    query = query.replace("-", "_")
    methods = {
        "control_mode": ("GetControlMode", hand.get_control_mode),
        "joint_states": ("GetJointStates", hand.get_joint_states),
        "end_pose": ("GetEndPose", hand.get_end_pose),
        "gripper_position": ("GetGripperPosition", hand.get_gripper_position),
    }
    if query not in ("all", *methods):
        raise ValueError("query must be all, control-mode, joint-states, end-pose, or gripper-position")
    selected = methods.values() if query == "all" else (methods[query.replace("-", "_")],)
    for name, method in selected:
        print_message(name, require(method(timeout=3.0), name))


def set_master_mode(hand, mode):
    modes = {
        "joint_pos": ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS,
        "end_pose": ManipulatorControlMode.MANIPULATOR_END_POSE,
        "gravity": ManipulatorControlMode.MANIPULATOR_GRAVITY_COMPENSATION,
    }
    if mode not in modes:
        raise ValueError("mode must be joint_pos, end_pose, or gravity")
    return require(
        hand.set_control_mode(
            ManipulatorControlModeParam(mode=modes[mode]), timeout=3.0
        ),
        "MasterArm.SetControlMode",
    )


def _joint_values(state):
    return [float(value) for value in state.position]


def _normalized_joint_feedback(values):
    """Return legal command values while tolerating tiny readback noise."""
    feedback = np.asarray(values, dtype=float)
    if feedback.shape != MASTER_JOINT_LOWER.shape or not np.all(np.isfinite(feedback)):
        raise RuntimeError("MasterArm.GetJointStates returned invalid joint positions")
    if np.any(feedback < MASTER_JOINT_LOWER - JOINT_FEEDBACK_LIMIT_TOLERANCE) or np.any(
        feedback > MASTER_JOINT_UPPER + JOINT_FEEDBACK_LIMIT_TOLERANCE
    ):
        bad = int(np.where(
            (feedback < MASTER_JOINT_LOWER - JOINT_FEEDBACK_LIMIT_TOLERANCE)
            | (feedback > MASTER_JOINT_UPPER + JOINT_FEEDBACK_LIMIT_TOLERANCE)
        )[0][0])
        raise RuntimeError(
            f"joint{bad + 1} feedback {feedback[bad]} is outside "
            f"[{MASTER_JOINT_LOWER[bad]}, {MASTER_JOINT_UPPER[bad]}]"
        )
    return np.clip(feedback, MASTER_JOINT_LOWER, MASTER_JOINT_UPPER)


def validate_rate(rate_hz):
    if rate_hz <= 0 or rate_hz > 500:
        raise ValueError("rate-hz must be in (0, 500]")


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


def move_arm_joints_toppra(hand, target_positions, rate_hz, v_max=1.0, a_max=3.0):
    """TOPPRA joint-space trajectory matching the quanta_x1 example."""
    validate_rate(rate_hz)
    validate_target_positions(target_positions)

    current_state = require(
        hand.get_joint_states(timeout=3.0), "MasterArm.GetJointStates"
    )
    q_start = np.array(current_state.position, dtype=float)
    q_end = np.array(target_positions, dtype=float)
    if q_start.shape != q_end.shape:
        raise RuntimeError("MasterArm.GetJointStates returned an unexpected joint count")
    q_start = _normalized_joint_feedback(q_start)

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
            hand.set_joint_positions(
                JointPositions(positions=command.tolist()), timeout=3.0
            ),
            "MasterArm.SetJointPositions",
        )
        command_count += 1
        time.sleep(dt)

    require(
        hand.set_joint_positions(
            JointPositions(positions=q_end.tolist()), timeout=3.0
        ),
        "MasterArm.SetJointPositions(final)",
    )
    return command_count + 1


def move_joints(
    hand, *, joint_index, joint_delta, target_text, rate_hz, hold, return_original
):
    set_master_mode(hand, "joint_pos")
    start_feedback = _joint_values(
        require(hand.get_joint_states(timeout=3.0), "MasterArm.GetJointStates")
    )
    # The controller can report a tiny negative value at a zero lower limit.
    # Normalize before adding a relative delta; otherwise an unrelated joint
    # makes the whole command fail validation (or gets sent out of range).
    start = _normalized_joint_feedback(start_feedback).tolist()
    if target_text:
        target = [float(item.strip()) for item in target_text.split(",") if item.strip()]
        if len(target) != len(start):
            raise ValueError(f"target must contain {len(start)} joint values")
    else:
        if not 0 <= joint_index < len(start):
            raise ValueError(f"joint-index must be in [0, {len(start) - 1}]")
        target = list(start)
        target[joint_index] += joint_delta
    validate_target_positions(target)
    print(f"start_feedback={start_feedback}\nstart_command={start}\ntarget={target}")
    count = move_arm_joints_toppra(hand, target, rate_hz)
    print(f"sent {count} TOPPRA joint commands")
    time.sleep(hold)
    if return_original:
        print("Returning to original joint command.")
        original = np.clip(
            np.array(start, dtype=float), MASTER_JOINT_LOWER, MASTER_JOINT_UPPER
        ).tolist()
        count = move_arm_joints_toppra(hand, original, rate_hz)
        print(f"sent {count} TOPPRA return joint commands")
        time.sleep(0.5)


def _pose_from_stamped(stamped):
    pose = stamped.pose if hasattr(stamped, "pose") else stamped
    return Pose(
        position=Point(x=float(pose.position.x), y=float(pose.position.y), z=float(pose.position.z)),
        orientation=Quaternion(
            x=float(pose.orientation.x), y=float(pose.orientation.y),
            z=float(pose.orientation.z), w=float(pose.orientation.w),
        ),
    )


def _pose_target(original, axis, delta, text):
    if text:
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
        if len(values) != 7:
            raise ValueError("target-pose expects x,y,z,qx,qy,qz,qw")
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm < 1e-9:
            raise ValueError("target-pose quaternion must be non-zero")
        return Pose(
            position=Point(x=values[0], y=values[1], z=values[2]),
            orientation=Quaternion(x=values[3] / norm, y=values[4] / norm,
                                   z=values[5] / norm, w=values[6] / norm),
        )
    if axis not in ("x", "y", "z"):
        raise ValueError("axis must be x, y, or z")
    position = Point(x=float(original.position.x), y=float(original.position.y), z=float(original.position.z))
    setattr(position, axis, getattr(position, axis) + delta)
    return Pose(position=position, orientation=original.orientation)


def move_arm_endpose_toppra(hand, target_pose, rate_hz, v_max=2.2, a_max=0.3):
    """TOPPRA Cartesian line with SLERP orientation, matching quanta_x1."""
    validate_rate(rate_hz)
    start_pose_msg = require(hand.get_end_pose(timeout=3.0), "MasterArm.GetEndPose")
    start_pose = _pose_from_stamped(start_pose_msg)

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
        require(hand.set_end_pose(pose, timeout=3.0), "MasterArm.SetEndPose")
        command_count += 1
        time.sleep(dt)

    require(hand.set_end_pose(target_pose, timeout=3.0), "MasterArm.SetEndPose(final)")
    return command_count + 1


def move_pose(
    hand, *, axis, delta, target_text, rate_hz, hold, return_original
):
    set_master_mode(hand, "end_pose")
    original = _pose_from_stamped(require(hand.get_end_pose(timeout=3.0), "MasterArm.GetEndPose"))
    target = _pose_target(original, axis, delta, target_text)
    print_message("target end pose", target)
    count = move_arm_endpose_toppra(hand, target, rate_hz)
    print(f"sent {count} TOPPRA end-pose commands")
    time.sleep(hold)
    if return_original:
        print("Returning to original end pose.")
        count = move_arm_endpose_toppra(hand, original, rate_hz)
        print(f"sent {count} TOPPRA return end-pose commands")
        time.sleep(0.5)


def zero_end_pose():
    return Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def zero_master(hand, mode, rate_hz, hold):
    if mode == "joint_pos":
        set_master_mode(hand, mode)
        current = _joint_values(require(hand.get_joint_states(timeout=3.0), "MasterArm.GetJointStates"))
        target = [0.0] * len(current)
        validate_target_positions(target)
        count = move_arm_joints_toppra(hand, target, rate_hz)
        print(f"sent {count} TOPPRA joint zero commands")
    elif mode == "end_pose":
        set_master_mode(hand, mode)
        count = move_arm_endpose_toppra(hand, zero_end_pose(), rate_hz)
        print(f"sent {count} TOPPRA end-pose zero commands")
    else:
        raise ValueError("mode must be joint_pos or end_pose")
    time.sleep(hold)


def stream_master_arm(hand, stream_name, interval, samples):
    methods = {
        "joint_states": hand.get_joint_states_stream,
        "end_pose": hand.get_end_pose_stream,
        "gripper_state": hand.get_gripper_state_stream,
        "gripper_joint_states": hand.get_gripper_joint_states_stream,
    }
    if stream_name not in methods:
        raise ValueError("stream must be joint_states, end_pose, gripper_state, or gripper_joint_states")
    stream = methods[stream_name](timeout=None)
    count = 0
    try:
        print("Streaming. Press Ctrl+C to stop.")
        for result in stream:
            print_message(stream_name, require(result, f"MasterArm.{stream_name}"))
            count += 1
            if samples > 0 and count >= samples:
                break
            if interval:
                time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopping stream.")
    finally:
        close_stream(stream)


def verify(robot, model="ex001"):
    if model not in ("desktop", "ex001", "auto"):
        raise RuntimeError("desktop master arm is only available on EX001")
    for side in ("left", "right"):
        read_master_arm(master(robot, side))




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
    parser.add_argument("--model", default="desktop")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="left")
    parser.add_argument("--action", choices=("read", "snapshot", "set-mode", "move", "zero", "stream", "verify"), default="read")
    parser.add_argument("--mode", choices=("joint-pos", "end-pose", "gravity", "joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--query", choices=("all", "control-mode", "joint-states", "end-pose", "gripper-position"), default="all")
    parser.add_argument("--stream", choices=("joint-states", "end-pose", "gripper-state", "gripper-joint-states"), default="joint-states")
    parser.add_argument("--write", action="store_true", help="allow set-mode/move/zero commands")
    parser.add_argument("--joint-index", type=int, default=0)
    parser.add_argument("--joint-delta", type=float, default=0.262)
    parser.add_argument("--target-positions", default="")
    parser.add_argument("--axis", default="z")
    parser.add_argument("--pose-delta", type=float, default=0.10)
    parser.add_argument("--target-pose", default="")
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--return-original", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--stream-interval", type=float, default=0.0)
    parser.add_argument("--samples", type=int, default=0)
    args = parser.parse_args(argv)
    validate_rate(TRAJECTORY_RATE_HZ)
    if args.hold < 0:
        raise ValueError("hold must be >= 0")
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot, args.model)
        else:
            if args.action in ("set-mode", "move", "zero") and not args.write:
                raise RuntimeError("refusing write action: pass --write after onsite safety confirmation")
            mode = args.mode.replace("-", "_")
            if args.action in ("set-mode", "move", "zero"):
                # 控制前先进入 SDK 模式（主臂与手臂共用同一道门控）。
                enter_sdk_mode(robot)
            for side in sides(args.arm):
                hand = master(robot, side)
                if args.action in ("read", "snapshot"):
                    read_master_arm(hand, args.query)
                elif args.action == "set-mode":
                    set_master_mode(hand, mode)
                elif args.action == "move":
                    if mode == "joint_pos":
                        move_joints(hand, joint_index=args.joint_index, joint_delta=args.joint_delta,
                                    target_text=args.target_positions, rate_hz=TRAJECTORY_RATE_HZ,
                                    hold=args.hold,
                                    return_original=args.return_original)
                    elif mode == "end_pose":
                        move_pose(hand, axis=args.axis, delta=args.pose_delta,
                                  target_text=args.target_pose, rate_hz=TRAJECTORY_RATE_HZ,
                                  hold=args.hold,
                                  return_original=args.return_original)
                    else:
                        raise ValueError("move mode must be joint-pos or end-pose")
                elif args.action == "zero":
                    zero_master(hand, mode, TRAJECTORY_RATE_HZ, args.hold)
                elif args.action == "stream":
                    if args.arm == "both":
                        raise ValueError("stream does not support --arm both")
                    stream_master_arm(hand, args.stream.replace("-", "_"), args.stream_interval, args.samples)
        print("PASS: desktop master-arm control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
