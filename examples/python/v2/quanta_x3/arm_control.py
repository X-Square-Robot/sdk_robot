"""Quanta X3 seven-axis arm example: direct v2 API calls."""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import x2robot_sdk
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from toppra.constraint import JointAccelerationConstraint, JointVelocityConstraint
from x2robot.geometry_msgs import Point, Pose as RosPose, Quaternion
from x2robot.sdk import (
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)
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
    scope = "both arms" if arm_side == "both" else f"{arm_side} arm"
    try:
        input(
            f"About to execute Quanta X3 {scope} {mode} control; the robot will begin moving. "
            "Press Enter to continue or Ctrl+C to cancel (use --yes for automation): "
        )
    except EOFError as error:
        raise RuntimeError(
            "Motion confirmation required; press Enter in an interactive terminal "
            "or pass --yes for automation"
        ) from error


def arm(robot, side):
    return robot.left_arm_controller if side == "left" else robot.right_arm_controller


def copy_arm_pose(pose):
    """Copy and validate the flattened v2 pose used to hold the other arm."""
    values = np.array(
        [pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw],
        dtype=float,
    )
    if not np.all(np.isfinite(values)) or np.linalg.norm(values[3:]) < 1e-9:
        raise RuntimeError("OtherArm.GetEndPose returned an invalid pose")
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
    """Copy the waist pose that must be refreshed with each arm command.

    The Quanta X3 whole-body controller subscribes to the waist target on
    /whole_body_controller/reasoning_target_head and expects the ROS-shaped
    geometry_msgs/Pose on the waist service.
    """
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


def set_mode(robot, mode):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=mode), timeout=5.0), "RobotControl.SetManipulatorControlMode")


JOINT_LOWER_LIMITS = np.array(
    [-3.0194, -2.0595, -3.0194, -2.5307, -3.0194, -1.0472, -1.5708],
    dtype=float,
)
JOINT_UPPER_LIMITS = np.array(
    [3.0194, 2.0595, 3.0194, 0.7854, 3.0194, 1.0472, 1.5708],
    dtype=float,
)


def trajectory_timestamps(duration, interval):
    """按实际经过时间取点；RPC 变慢时跳过过期点，不堆积补发。"""
    started = time.monotonic()
    elapsed = 0.0
    while elapsed < duration:
        yield elapsed
        delay = min(elapsed + interval, duration) - (time.monotonic() - started)
        if delay > 0.0:
            time.sleep(delay)
        elapsed = time.monotonic() - started


def move_arm_joints_toppra(controller, target_positions, v_max=1.0, a_max=3.0):
    """Plan and execute the same bounded joint trajectory as the Quanta X2 example."""
    q_end = np.asarray(target_positions, dtype=float)
    if q_end.shape != JOINT_LOWER_LIMITS.shape or not np.all(np.isfinite(q_end)):
        raise ValueError("target must contain seven finite joint positions")
    if np.any(q_end < JOINT_LOWER_LIMITS) or np.any(q_end > JOINT_UPPER_LIMITS):
        raise ValueError("target joint position is outside the Quanta X3 arm limits")

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

    interval = 0.002  # 最多 500 Hz，实际取点时间包含 RPC 耗时。
    print(f"Executing TOPP-RA joint trajectory: Estimated duration {trajectory.duration:.2f}s")
    for timestamp in trajectory_timestamps(trajectory.duration, interval):
        command = np.clip(
            np.asarray(trajectory(float(timestamp)), dtype=float),
            JOINT_LOWER_LIMITS,
            JOINT_UPPER_LIMITS,
        )
        require(
            controller.set_joint_positions(command.tolist(), deadline_ms=500),
            "Arm.SetJointPositions",
        )

    require(
        controller.set_joint_positions(q_end.tolist(), deadline_ms=500),
        "Arm.SetJointPositions(final)",
    )


def pose_error(actual, target):
    """兼容手臂扁平 Pose 和腰部 ROS Pose，返回位置/最短旋转误差。"""

    def values(pose):
        if not hasattr(pose, "qx"):
            return np.array(
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )
        return np.array([pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw])

    a, b = values(actual), values(target)
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        raise RuntimeError("End-pose feedback contains non-finite values")
    norms = np.linalg.norm(a[3:]), np.linalg.norm(b[3:])
    if min(norms) < 1e-9:
        raise RuntimeError("End-pose feedback contains an invalid quaternion")
    angle = 2 * math.acos(np.clip(abs(np.dot(a[3:], b[3:]) / np.prod(norms)), 0, 1))
    return float(np.linalg.norm(a[:3] - b[:3])), angle


def hold_end_pose(controller, target, other_controller, other_pose, waist, waist_pose):
    """持续刷新三路目标 1 秒；WBC 残余误差仅展示，不作为演示通过条件。"""
    deadline = time.monotonic() + 1.0
    while True:
        require(
            controller.set_end_pose(target, deadline_ms=500), "Arm.SetEndPose(hold)"
        )
        require(
            other_controller.set_end_pose(other_pose, deadline_ms=500),
            "OtherArm.SetEndPose(hold)",
        )
        require(waist.set_end_pose(waist_pose, timeout=0.5), "Waist.SetEndPose(hold)")
        errors = [
            pose_error(
                require(
                    controller.get_end_pose(deadline_ms=500), "Arm.GetEndPose(hold)"
                ),
                target,
            ),
            pose_error(
                require(
                    other_controller.get_end_pose(deadline_ms=500),
                    "OtherArm.GetEndPose(hold)",
                ),
                other_pose,
            ),
            pose_error(
                copy_waist_pose(
                    require(waist.get_end_pose(timeout=0.5), "Waist.GetEndPose(hold)")
                ),
                waist_pose,
            ),
        ]
        now = time.monotonic()
        if now >= deadline:
            break
        time.sleep(min(0.005, deadline - now))
    residuals = ", ".join(
        f"{name}=({position * 1000:.3f}, {math.degrees(angle):.3f})"
        for name, (position, angle) in zip(("arm", "other", "waist"), errors)
    )
    print(f"End-pose target hold complete; residual errors (mm, deg): {residuals}")


def move_arm_endpose_toppra(
    controller,
    target,
    other_controller,
    waist,
    other_pose,
    waist_pose,
    v_max=2.2,
    a_max=0.3,
):
    """Move one arm while refreshing the other arm and the waist end pose.

    Quanta X3's whole-body controller requires the three reasoning targets
    (left arm, right arm and waist) to be refreshed together in end-pose mode.
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
    interval = 0.005  # 最多 200 Hz，三路同步 RPC 共用真实轨迹时间。
    print(f"Executing TOPP-RA smooth trajectory: Estimated duration {trajectory.duration:.2f}s")
    for timestamp in trajectory_timestamps(trajectory.duration, interval):
        alpha = np.clip(
            float(trajectory(float(timestamp))[0]) / path_length, 0.0, 1.0
        )
        position = p_start + (p_end - p_start) * alpha
        orientation = slerp(float(timestamp)).as_quat()
        pose = x2robot_sdk.Pose(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            qx=float(orientation[0]),
            qy=float(orientation[1]),
            qz=float(orientation[2]),
            qw=float(orientation[3]),
        )
        require(controller.set_end_pose(pose, deadline_ms=500), "Arm.SetEndPose")
        require(
            other_controller.set_end_pose(other_pose, deadline_ms=500),
            "OtherArm.SetEndPose",
        )
        if waist is not None:
            require(waist.set_end_pose(waist_pose, timeout=3.0), "Waist.SetEndPose")

    require(controller.set_end_pose(target, deadline_ms=500), "Arm.SetEndPose(final)")
    require(
        other_controller.set_end_pose(other_pose, deadline_ms=500),
        "OtherArm.SetEndPose(final)",
    )
    if waist is not None:
        require(waist.set_end_pose(waist_pose, timeout=3.0), "Waist.SetEndPose(final)")
    hold_end_pose(controller, target, other_controller, other_pose, waist, waist_pose)


def move_by_joint_positions(robot, side):
    controller = arm(robot, side)
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS)
    # Start from measured Quanta X3 poses instead of the Quanta X2 reset pose.
    # The demonstration changes only joint 4 by 0.2 rad for a small lift.
    reset = (
        [0.002453, 1.570471, 1.570013, -0.591459, -0.002960, -0.005304, 0.002296]
        if side == "left"
        else [-0.000184, 1.568038, -1.567079, -0.591765, -0.000566, -0.005404, 0.000034]
    )
    target = list(reset)
    target[3] += 0.2
    print("Starting joint reset...")
    move_arm_joints_toppra(controller, reset)
    print(f"Moving {side} arm to target joint angles: {target}")
    move_arm_joints_toppra(controller, target)


def move_by_end_pose(robot, side):
    controller = arm(robot, side)
    other_controller = arm(robot, "right" if side == "left" else "left")
    set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
    time.sleep(1.0)
    # 一次动作共用保持点，不把 reset 后的偏差重新当作目标。
    other_pose = copy_arm_pose(
        require(other_controller.get_end_pose(deadline_ms=1000), "OtherArm.GetEndPose")
    )
    waist_pose = copy_waist_pose(
        require(robot.waist.get_end_pose(timeout=3.0), "Waist.GetEndPose")
    )
    # This measured Quanta X3 posture avoids the centered pose that can cause an
    # inverse-kinematics branch flip.  The right arm uses the mirrored y
    # coordinate; the measured quaternion is shared by both arm commands.
    if side == "left":
        reset = x2robot_sdk.Pose(
            x=0.125556,
            y=0.193477,
            z=0.857640,
            qx=0.001356,
            qy=0.002202,
            qz=0.708243,
            qw=0.705964,
        )
    else:
        reset = x2robot_sdk.Pose(
            x=0.125556,
            y=-0.193477,
            z=0.857640,
            qx=0.001356,
            qy=0.002202,
            qz=0.708243,
            qw=0.705964,
        )
    print("Starting end pose reset...")
    move_arm_endpose_toppra(
        controller, reset, other_controller, robot.waist, other_pose, waist_pose
    )
    # 沿 z 方向抬升 10 cm，保持准备位的 x/y 和姿态。
    target = x2robot_sdk.Pose(
        x=reset.x,
        y=reset.y,
        z=reset.z + 0.1,
        qx=reset.qx,
        qy=reset.qy,
        qz=reset.qz,
        qw=reset.qw,
    )
    print("Executing smooth end-pose trajectory...")
    move_arm_endpose_toppra(
        controller, target, other_controller, robot.waist, other_pose, waist_pose
    )


def stream_arm(robot, side, mode, seconds=None):
    controller = arm(robot, side)
    if mode == "end_pose":
        set_mode(robot, ManipulatorControlMode.MANIPULATOR_END_POSE)
        # Give the controller a moment to publish the first pose in the new mode.
        time.sleep(1.0)
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


def verify_one(robot, side, mode):
    controller = arm(robot, side)
    joint_mode = mode == "joint_pos"
    operation = "GetJointStates" if joint_mode else "GetEndPose"
    getter = controller.get_joint_states if joint_mode else controller.get_end_pose
    opener = controller.get_joint_states_stream if joint_mode else controller.get_end_pose_stream

    def validate(data):
        if joint_mode:
            if len(data.position) != 7 or not np.all(np.isfinite(data.position)):
                raise AssertionError(f"{side} arm returned invalid joint feedback")
        else:
            values = (data.x, data.y, data.z, data.qx, data.qy, data.qz, data.qw)
            if not np.all(np.isfinite(values)) or np.linalg.norm(values[3:]) < 1e-9:
                raise AssertionError(f"{side} arm returned invalid end-pose feedback")

    validate(require(getter(), f"{side} arm {operation}"))
    stream = require(opener(), f"{side} arm {operation}Stream")
    try:
        validate(require(stream.read(), f"{side} arm {operation}Stream/read"))
    finally:
        close_stream(stream)


def verify(robot, model="quanta_x3", *, arm_side="both", mode=None):
    if model not in ("quanta_x3", "auto"):
        raise RuntimeError("this example is only for Quanta X3")
    if arm_side not in ("left", "right", "both") or mode not in (None, "joint_pos", "end_pose"):
        raise ValueError("Unsupported arm verification selection")
    # 不带选择参数时，保留原有双臂、两类反馈的完整检查。
    modes = ("joint_pos", "end_pose") if mode is None else (mode,)
    for side in (("left", "right") if arm_side == "both" else (arm_side,)):
        for selected_mode in modes:
            verify_one(robot, side, selected_mode)


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
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and non-negative")
    if args.action == "stream" and args.arm == "both":
        parser.error("stream does not support --arm both")
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "move":
            confirm_motion(args.action, args.mode, args.arm, args.yes)
        if args.action == "verify":
            verify(robot, arm_side=args.arm, mode=args.mode)
        else:
            for side in (("left", "right") if args.arm == "both" else (args.arm,)):
                if args.action == "move":
                    if args.mode == "joint_pos":
                        move_by_joint_positions(robot, side)
                    else:
                        move_by_end_pose(robot, side)
                else:
                    stream_arm(robot, side, args.mode, args.seconds or None)
        print("PASS: Quanta X3 arm control")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
