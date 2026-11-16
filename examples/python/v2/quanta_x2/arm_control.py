"""Quanta X2 seven-axis arm example: direct v2 API calls."""

from __future__ import annotations

import argparse
import time

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
import x2robot_sdk
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from toppra.constraint import JointAccelerationConstraint, JointVelocityConstraint
from x2robot.sdk import (
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)
from x2robot.geometry_msgs import Point, Pose as RosPose, Quaternion

from x2robot_sdk import Pose


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def format_numbers(values, digits=6):
    """Render a numeric sequence so stream output is useful at a glance."""
    return "[" + ", ".join(f"{float(value):.{digits}f}" for value in values) + "]"


def print_joint_state(state):
    """Print joint names and all state vectors instead of an abbreviated repr."""
    names = list(getattr(state, "name", ()))
    positions = list(getattr(state, "position", ()))
    velocities = list(getattr(state, "velocity", ()))
    efforts = list(getattr(state, "effort", ()))
    if names and len(names) == len(positions):
        joints = ", ".join(
            f"{name}={position:.6f}" for name, position in zip(names, positions)
        )
    else:
        joints = format_numbers(positions)
    print(
        "joint_pos: "
        f"{joints}; velocity={format_numbers(velocities)}; "
        f"effort={format_numbers(efforts)}"
    )


def print_end_pose(pose):
    """Print Cartesian position and quaternion fields explicitly."""
    print(
        "end_pose: "
        f"position=(x={pose.x:.6f}, y={pose.y:.6f}, z={pose.z:.6f}); "
        f"orientation=(qx={pose.qx:.6f}, qy={pose.qy:.6f}, "
        f"qz={pose.qz:.6f}, qw={pose.qw:.6f})"
    )


def confirm_motion(action, mode, arm_side, assume_yes=False):
    if assume_yes:
        return
    scope = "both arms" if action == "verify" or arm_side == "both" else f"{arm_side} arm"
    try:
        input(
            f"About to execute Quanta X2 {scope} {mode} control; the robot will begin moving. "
            "Press Enter to continue or Ctrl+C to cancel (use --yes for automation): "
        )
    except EOFError as error:
        raise RuntimeError(
            "Motion confirmation required; press Enter in an interactive terminal "
            "or pass --yes for automation"
        ) from error


def arm(robot, side):
    return robot.left_arm_controller if side == "left" else robot.right_arm_controller


def set_mode(robot, mode):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=mode), timeout=5.0), "RobotControl.SetManipulatorControlMode")


JOINT_LOWER_LIMITS = np.array(
    [-3.1067, -2.0944, -3.1067, -2.5307, -3.1067, -1.0472, -1.5708],
    dtype=float,
)
JOINT_UPPER_LIMITS = np.array(
    [3.1067, 2.0944, 3.1067, 1.0472, 3.1067, 1.0472, 1.5708],
    dtype=float,
)


def move_arm_joints_toppra(controller, target_positions, v_max=1.0, a_max=3.0):
    """Plan and execute the same bounded joint trajectory as the v1 example."""
    q_end = np.asarray(target_positions, dtype=float)
    if q_end.shape != JOINT_LOWER_LIMITS.shape or not np.all(np.isfinite(q_end)):
        raise ValueError("target must contain seven finite joint positions")
    if np.any(q_end < JOINT_LOWER_LIMITS) or np.any(q_end > JOINT_UPPER_LIMITS):
        raise ValueError("target joint position is outside the Quanta X2 limits")

    current = require(controller.get_joint_states(deadline_ms=1000), "Arm.GetJointStates")
    q_start = np.asarray(list(current.position), dtype=float)
    if q_start.shape != q_end.shape or not np.all(np.isfinite(q_start)):
        raise RuntimeError("Arm.GetJointStates returned an invalid joint position")
    q_start = np.clip(q_start, JOINT_LOWER_LIMITS, JOINT_UPPER_LIMITS)

    path = ta.SplineInterpolator([0, 1], np.stack([q_start, q_end]))
    constraints = [
        JointVelocityConstraint([v_max] * len(q_start)),
        JointAccelerationConstraint([a_max] * len(q_start)),
    ]
    trajectory = algo.TOPPRA(constraints, path).compute_trajectory(0, 0)
    if trajectory is None:
        raise RuntimeError("TOPP-RA joint planning failed")

    interval = 0.002  # 500 Hz, matching the v1 example.
    print(f"Executing TOPP-RA joint trajectory: Estimated duration {trajectory.duration:.2f}s")
    for timestamp in np.arange(0.0, trajectory.duration, interval):
        command = np.clip(
            np.asarray(trajectory(float(timestamp)), dtype=float),
            JOINT_LOWER_LIMITS,
            JOINT_UPPER_LIMITS,
        )
        require(
            controller.set_joint_positions(command.tolist(), deadline_ms=500),
            "Arm.SetJointPositions",
        )
        time.sleep(interval)

    require(
        controller.set_joint_positions(q_end.tolist(), deadline_ms=500),
        "Arm.SetJointPositions(final)",
    )


def copy_arm_pose(pose):
    return Pose(
        x=float(pose.x),
        y=float(pose.y),
        z=float(pose.z),
        qx=float(pose.qx),
        qy=float(pose.qy),
        qz=float(pose.qz),
        qw=float(pose.qw),
    )


def copy_waist_pose(stamped):
    if stamped is None or stamped.pose is None:
        raise RuntimeError("Waist.GetEndPose returned an empty PoseStamped")
    if stamped.pose.position is None or stamped.pose.orientation is None:
        raise RuntimeError("Waist.GetEndPose returned an incomplete PoseStamped")
    return RosPose(
        position=Point(
            x=float(stamped.pose.position.x),
            y=float(stamped.pose.position.y),
            z=float(stamped.pose.position.z),
        ),
        orientation=Quaternion(
            x=float(stamped.pose.orientation.x),
            y=float(stamped.pose.orientation.y),
            z=float(stamped.pose.orientation.z),
            w=float(stamped.pose.orientation.w),
        ),
    )


def move_arm_endpose_toppra(
    controller, target, other_controller=None, waist=None, v_max=2.2, a_max=0.3
):
    """Run the CX002 whole-body end-pose trajectory used by the v1 example.

    CX002's whole-body controller expects the selected arm, the other arm and
    the waist to be refreshed together.  The gRPC arm API uses the flattened
    v2 ``Pose`` type; the waist service still accepts the ROS-shaped
    ``geometry_msgs.Pose`` request and publishes it as ``PoseStamped`` on the
    server side.
    """
    current = require(controller.get_end_pose(deadline_ms=1000), "Arm.GetEndPose")
    p_start = np.array([current.x, current.y, current.z], dtype=float)
    p_end = np.array([target.x, target.y, target.z], dtype=float)
    q_start = np.array([current.qx, current.qy, current.qz, current.qw], dtype=float)
    q_end = np.array([target.qx, target.qy, target.qz, target.qw], dtype=float)
    if not all(np.all(np.isfinite(values)) for values in (p_start, p_end, q_start, q_end)):
        raise RuntimeError("Arm.GetEndPose or target contains non-finite values")
    if np.linalg.norm(q_start) < 1e-9 or np.linalg.norm(q_end) < 1e-9:
        raise ValueError("end-pose orientations must be non-zero quaternions")

    other_pose = None
    if other_controller is not None:
        other_pose = copy_arm_pose(
            require(other_controller.get_end_pose(deadline_ms=1000), "OtherArm.GetEndPose")
        )

    waist_pose = None
    if waist is not None:
        waist_pose = copy_waist_pose(
            require(waist.get_end_pose(timeout=3.0), "Waist.GetEndPose")
        )

    distance = np.linalg.norm(p_end - p_start)
    path_length = distance if distance > 1e-6 else 1.0
    path = ta.SplineInterpolator([0, 1], np.array([[0.0], [path_length]]))
    constraints = [
        JointVelocityConstraint([v_max]),
        JointAccelerationConstraint([a_max]),
    ]
    trajectory = algo.TOPPRA(constraints, path).compute_trajectory(0, 0)
    if trajectory is None:
        raise RuntimeError("TOPP-RA end-pose planning failed")

    slerp = Slerp([0, trajectory.duration], R.from_quat([q_start, q_end]))
    interval = 0.005  # 200 Hz, matching the v1 example.
    print(f"Executing TOPP-RA smooth trajectory: Estimated duration {trajectory.duration:.2f}s")
    for timestamp in np.arange(0.0, trajectory.duration, interval):
        alpha = np.clip(
            float(trajectory(float(timestamp))[0]) / path_length, 0.0, 1.0
        )
        position = p_start + (p_end - p_start) * alpha
        orientation = slerp(float(timestamp)).as_quat()
        pose = Pose(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            qx=float(orientation[0]),
            qy=float(orientation[1]),
            qz=float(orientation[2]),
            qw=float(orientation[3]),
        )
        require(controller.set_end_pose(pose, deadline_ms=500), "Arm.SetEndPose")
        if other_controller is not None:
            require(
                other_controller.set_end_pose(other_pose, deadline_ms=500),
                "OtherArm.SetEndPose",
            )
        if waist is not None:
            require(waist.set_end_pose(waist_pose, timeout=3.0), "Waist.SetEndPose")
        time.sleep(interval)

    require(controller.set_end_pose(target, deadline_ms=500), "Arm.SetEndPose(final)")
    if other_controller is not None:
        require(other_controller.set_end_pose(other_pose, deadline_ms=500), "OtherArm.SetEndPose(final)")
    if waist is not None:
        require(waist.set_end_pose(waist_pose, timeout=3.0), "Waist.SetEndPose(final)")
    time.sleep(0.1)


def move_by_joint_positions(robot, side):
    controller = arm(robot, side)
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    reset = (
        [1.57, -1.57, -1.57, -1.57, 0.0, 0.0, 0.0]
        if side == "left"
        else [-1.57, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0]
    )
    target = ([1.57, -1.57, -1.57, -1.07, 0.0, 0.0, 0.7854]
              if side == "left" else [-1.57, -1.57, 1.57, -1.07, 0.0, 0.0, 0.7854])
    print("Starting joint reset...")
    move_arm_joints_toppra(controller, reset)
    print(f"Moving {side} arm to target joint angles: {target}")
    move_arm_joints_toppra(controller, target)


def move_by_end_pose(robot, side):
    controller = arm(robot, side)
    other_controller = arm(robot, "right" if side == "left" else "left")
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
    time.sleep(1.0)
    y = 0.22 if side == "left" else -0.22
    print("Starting end pose reset...")
    move_arm_endpose_toppra(
        controller,
        Pose(x=0.4, y=y, z=0.877, qw=1.0),
        other_controller=other_controller,
        waist=robot.waist,
    )
    time.sleep(2.0)
    print("Executing smooth end-pose trajectory...")
    move_arm_endpose_toppra(
        controller,
        Pose(x=0.4, y=y, z=1.08, qw=1.0),
        other_controller=other_controller,
        waist=robot.waist,
    )


def stream_arm(robot, side, mode, seconds=None):
    controller = arm(robot, side)
    opened = require(
        controller.get_end_pose_stream(deadline_ms=3000)
        if mode == "end_pose" else controller.get_joint_states_stream(deadline_ms=3000),
        f"{side} arm stream",
    )
    end = None if seconds is None else time.monotonic() + seconds
    try:
        print("Starting arm stream; press Ctrl+C to stop")
        while end is None or time.monotonic() < end:
            sample = require(opened.read(deadline_ms=3000), f"{side} arm stream/read")
            if mode == "end_pose":
                print_end_pose(sample)
            else:
                print_joint_state(sample)
    except KeyboardInterrupt:
        print("\nStopping streaming...")
    finally:
        close_stream(opened)


def verify_one(robot, side):
    controller = arm(robot, side)
    target = [0.08] + [0.0] * 6
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    require(controller.set_joint_positions(target), f"{side} arm SetJointPositions")
    time.sleep(0.12)
    state = require(controller.get_joint_states(), f"{side} arm GetJointStates")
    if len(state.position) != 7 or any(abs(a - b) > 1e-5 for a, b in zip(state.position, target)):
        raise AssertionError(f"{side} arm joint readback mismatch")
    target_pose = Pose(x=0.31, y=0.07, z=0.72, qw=1.0)
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
    require(controller.set_end_pose(target_pose), f"{side} arm SetEndPose")
    time.sleep(0.12)
    pose = require(controller.get_end_pose(), f"{side} arm GetEndPose")
    for name in ("x", "y", "z"):
        if abs(getattr(pose, name) - getattr(target_pose, name)) > 1e-5:
            raise AssertionError(f"{side} arm pose {name} mismatch")
    stream = require(controller.get_end_pose_stream(), f"{side} arm GetEndPoseStream")
    try:
        require(stream.read(), f"{side} arm GetEndPoseStream/read")
    finally:
        close_stream(stream)


def verify(robot, model="cx002"):
    if model not in ("cx002", "auto"):
        raise RuntimeError("this example is only for CX002/Quanta X2")
    verify_one(robot, "left")
    verify_one(robot, "right")


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
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="move")
    parser.add_argument("--seconds", type=float, default=0.0, help="stream duration; 0 means until Ctrl+C")
    parser.add_argument("--yes", action="store_true", help="skip the motion confirmation prompt")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action in ("verify", "move"):
            confirm_motion(args.action, args.mode, args.arm, args.yes)
        if args.action == "verify":
            verify(robot)
        else:
            for side in (("left", "right") if args.arm == "both" else (args.arm,)):
                if args.action == "move":
                    if args.mode == "joint_pos":
                        move_by_joint_positions(robot, side)
                    else:
                        move_by_end_pose(robot, side)
                else:
                    if args.arm == "both":
                        raise RuntimeError("stream does not support --arm both")
                    stream_arm(robot, side, args.mode, args.seconds or None)
        print("PASS: Quanta X2 arm control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
