import json
import math
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
from typing import Annotated

import grpc
import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint
import typer
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from x2robot import Robot, connect
from x2robot.geometry_msgs import Point, Pose, Quaternion
from x2robot.sdk import GripperPosition, JointPositions
from x2robot.sdk import ManipulatorControlMode, ManipulatorControlModeParam


DEFAULT_SERVER = os.environ.get("X2ROBOT_SERVER", "192.168.10.1:50051")
DEFAULT_MODEL = os.environ.get("X2ROBOT_MODEL", "desktop")

MASTER_JOINT_LOWER = np.array([-2.7925, 0.0, -2.7925, -1.5708, -1.5708, -1.9199])
MASTER_JOINT_UPPER = np.array([2.7925, 3.6652, 0.0, 1.5708, 1.5708, 1.9199])
ZERO_JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def select_master_arm(robot: Robot, side: str):
    if side == "left":
        return robot.master_left_arm
    if side == "right":
        return robot.master_right_arm
    raise ValueError("side must be left or right")


def selected_sides(arm: str) -> list[str]:
    if arm == "both":
        return ["left", "right"]
    if arm in ("left", "right"):
        return [arm]
    raise ValueError("arm must be left, right, or both")


def print_message(title: str, message) -> None:
    print(f"\n== {title} ==")
    if isinstance(message, ManipulatorControlModeParam):
        print(json.dumps({"mode": str(message.mode)}, indent=2, ensure_ascii=False))
    elif isinstance(message, GripperPosition):
        print(json.dumps({"position": float(message.position)}, indent=2, ensure_ascii=False))
    elif hasattr(message, "to_json"):
        print(json.dumps(json.loads(message.to_json()), indent=2, ensure_ascii=False))
    else:
        print(message)


def require_write(write: bool, action: str) -> None:
    if write:
        return
    print(f"Refusing {action}: pass --write after onsite safety confirmation.")
    raise typer.Exit(11)


def normalize_name(value: str) -> str:
    return value.replace("-", "_")


def validate_rate(rate_hz: float) -> None:
    if rate_hz <= 0 or rate_hz > 500:
        raise typer.BadParameter("rate-hz must be in (0, 500]")


def validate_target_positions(target: list[float]) -> None:
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


def parse_target_positions(value: str) -> list[float]:
    positions = [float(item.strip()) for item in value.split(",") if item.strip()]
    validate_target_positions(positions)
    return positions


def parse_target_pose(value: str) -> Pose:
    values = [float(item.strip()) for item in value.split(",") if item.strip()]
    if len(values) != 7:
        raise ValueError("--target-pose expects x,y,z,qx,qy,qz,qw")
    if not all(math.isfinite(item) for item in values):
        raise ValueError("--target-pose values must be finite")
    q_norm = math.sqrt(sum(item * item for item in values[3:7]))
    if q_norm < 1e-9:
        raise ValueError("--target-pose quaternion must be non-zero")
    return Pose(
        position=Point(x=values[0], y=values[1], z=values[2]),
        orientation=Quaternion(
            x=values[3] / q_norm,
            y=values[4] / q_norm,
            z=values[5] / q_norm,
            w=values[6] / q_norm,
        ),
    )


def joint_index_from_args(names: list[str], joint: str, joint_index: int) -> int:
    if joint:
        if joint not in names:
            joint_suffix = joint.split("_")[-1]
            for index, name in enumerate(names):
                if name.endswith(joint_suffix):
                    return index
            raise ValueError(f"joint {joint!r} not found in current names: {names}")
        return names.index(joint)
    if joint_index < 0 or joint_index >= len(names):
        raise ValueError(f"joint-index must be in [0, {len(names) - 1}]")
    return joint_index


def set_master_mode(master_arm, mode: str):
    if mode == "joint_pos":
        target = ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS
    elif mode == "end_pose":
        target = ManipulatorControlMode.MANIPULATOR_END_POSE
    else:
        raise ValueError("mode must be joint_pos or end_pose")

    result = master_arm.set_control_mode(ManipulatorControlModeParam(mode=target))
    print_message(f"set_control_mode({mode})", result)
    return result


def run_for_sides(robot: Robot, sides: list[str], description: str, func) -> None:
    if len(sides) == 1:
        side = sides[0]
        print(f"\n== {description}: {side} ==")
        func(side, select_master_arm(robot, side))
        return

    print(f"\n== {description}: left + right ==")
    errors: dict[str, BaseException] = {}
    with ThreadPoolExecutor(max_workers=len(sides)) as executor:
        futures = {
            executor.submit(func, side, select_master_arm(robot, side)): side
            for side in sides
        }
        for future in as_completed(futures):
            side = futures[future]
            try:
                future.result()
                print(f"{side}: done")
            except BaseException as exc:  # noqa: BLE001 - keep the other side result visible
                errors[side] = exc
                print(f"{side}: failed: {type(exc).__name__}: {exc}")
    if errors:
        raise RuntimeError(
            "; ".join(f"{side}: {type(exc).__name__}: {exc}" for side, exc in errors.items())
        )


def pose_from_message_pose(message_pose) -> Pose:
    return Pose(
        position=Point(
            x=float(message_pose.position.x),
            y=float(message_pose.position.y),
            z=float(message_pose.position.z),
        ),
        orientation=Quaternion(
            x=float(message_pose.orientation.x),
            y=float(message_pose.orientation.y),
            z=float(message_pose.orientation.z),
            w=float(message_pose.orientation.w),
        ),
    )


def joint_positions(master_arm) -> list[float]:
    return [float(value) for value in master_arm.get_joint_states().position]


def max_abs_delta(values_a: list[float], values_b: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(values_a, values_b, strict=True))


def vector_delta(values_a: list[float], values_b: list[float]) -> list[float]:
    return [a - b for a, b in zip(values_a, values_b, strict=True)]


def pose_to_dict(pose: Pose) -> dict:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def pose_position_list(pose: Pose) -> list[float]:
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z)]


def pose_position_distance(pose_a: Pose, pose_b: Pose) -> float:
    return float(np.linalg.norm(np.array(pose_position_list(pose_a)) - np.array(pose_position_list(pose_b))))


def current_pose(master_arm) -> Pose:
    return pose_from_message_pose(master_arm.get_end_pose().pose)


def make_delta_pose(start: Pose, axis: str, delta: float) -> Pose:
    target = pose_from_message_pose(start)
    setattr(target.position, axis, float(getattr(target.position, axis)) + delta)
    return target


def zero_end_pose() -> Pose:
    return Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def print_read_result(title: str, call) -> bool:
    try:
        print_message(title, call())
        return True
    except grpc.RpcError as exc:
        print(f"\n== {title} ==")
        print(f"RPC failed: {exc.code().name} - {exc.details()}")
        return False


def read_master_arm(master_arm, query: str) -> None:
    attempts = 0
    successes = 0

    if query in ("all", "control_mode"):
        attempts += 1
        successes += int(print_read_result("control mode", master_arm.get_control_mode))
    if query in ("all", "joint_states"):
        attempts += 1
        successes += int(print_read_result("joint states", master_arm.get_joint_states))
    if query in ("all", "end_pose"):
        attempts += 1
        successes += int(print_read_result("end pose", master_arm.get_end_pose))
    if query in ("all", "gripper_position"):
        attempts += 1
        successes += int(print_read_result("gripper position", master_arm.get_gripper_position))

    if attempts > 0 and successes == 0:
        raise RuntimeError("all read RPCs failed for this MasterArm")


def move_arm_joints_toppra(
    master_arm,
    target_positions: list[float],
    rate_hz: float,
    v_max: float = 1.0,
    a_max: float = 3.0,
) -> int:
    """TOPPRA joint-space trajectory from current MasterArm joints to target."""
    validate_target_positions(target_positions)

    current_state = master_arm.get_joint_states()
    q_start = np.array(current_state.position, dtype=float)
    q_end = np.array(target_positions, dtype=float)

    # Current readback can be microscopically outside configured limits near zero.
    # Keep the command path legal without changing the requested target.
    q_start = np.clip(q_start, MASTER_JOINT_LOWER, MASTER_JOINT_UPPER)

    path = ta.SplineInterpolator([0, 1], np.stack([q_start, q_end]))
    pc_vel = constraint.JointVelocityConstraint([v_max] * len(q_start))
    pc_acc = constraint.JointAccelerationConstraint([a_max] * len(q_start))
    instance = algo.TOPPRA([pc_vel, pc_acc], path)
    traj = instance.compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPPRA joint planning failed")

    dt = 1.0 / rate_hz
    command_count = 0
    for t in np.arange(0, traj.duration, dt):
        q_t = np.clip(traj(t), MASTER_JOINT_LOWER, MASTER_JOINT_UPPER)
        master_arm.set_joint_positions(JointPositions(positions=q_t.tolist()))
        command_count += 1
        time.sleep(dt)

    master_arm.set_joint_positions(JointPositions(positions=q_end.tolist()))
    return command_count + 1


def move_arm_endpose_toppra(
    master_arm,
    target_pose: Pose,
    rate_hz: float,
    v_max: float = 2.2,
    a_max: float = 0.3,
) -> int:
    """TOPPRA Cartesian position trajectory with SLERP orientation interpolation."""
    start_pose_msg = master_arm.get_end_pose()
    start_pose = pose_from_message_pose(start_pose_msg.pose)

    p_start = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z], dtype=float)
    p_end = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z], dtype=float)

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
    pc_vel = constraint.JointVelocityConstraint([v_max])
    pc_acc = constraint.JointAccelerationConstraint([a_max])
    instance = algo.TOPPRA([pc_vel, pc_acc], path)
    traj = instance.compute_trajectory(0, 0)
    if traj is None:
        raise RuntimeError("TOPPRA end-pose planning failed")

    slerp = Slerp([0, traj.duration], R.from_quat([q_start, q_end]))
    dt = 1.0 / rate_hz
    command_count = 0

    for t in np.arange(0, traj.duration, dt):
        s_t = float(traj(t)[0])
        alpha = float(np.clip(s_t / effective_path_len, 0, 1))

        pose = Pose()
        pose.position = Point(
            x=float(p_start[0] + (p_end[0] - p_start[0]) * alpha),
            y=float(p_start[1] + (p_end[1] - p_start[1]) * alpha),
            z=float(p_start[2] + (p_end[2] - p_start[2]) * alpha),
        )
        curr_q = slerp(t).as_quat()
        pose.orientation = Quaternion(
            x=float(curr_q[0]),
            y=float(curr_q[1]),
            z=float(curr_q[2]),
            w=float(curr_q[3]),
        )

        master_arm.set_end_pose(pose)
        command_count += 1
        time.sleep(dt)

    master_arm.set_end_pose(target_pose)
    return command_count + 1


def move_joint(
    master_arm,
    joint: str,
    joint_index: int,
    delta_rad: float,
    target_positions: str,
    rate_hz: float,
    hold: float,
    return_original: bool,
) -> None:
    set_master_mode(master_arm, "joint_pos")
    current = master_arm.get_joint_states()
    names = list(current.name)
    start_feedback = [float(value) for value in current.position]
    original_command = np.clip(np.array(start_feedback, dtype=float), MASTER_JOINT_LOWER, MASTER_JOINT_UPPER).tolist()

    if target_positions:
        target = parse_target_positions(target_positions)
    else:
        index = joint_index_from_args(names, joint, joint_index)
        target = list(original_command)
        target[index] += delta_rad
        validate_target_positions(target)
        print(f"Moving {names[index]} by {delta_rad} rad.")

    print(f"start_feedback={start_feedback}")
    print(f"start_command={original_command}")
    print(f"target_command={target}")

    count = move_arm_joints_toppra(master_arm, target, rate_hz)
    print(f"sent {count} TOPPRA joint commands")
    time.sleep(hold)
    after_target = joint_positions(master_arm)
    print(f"after_target_feedback={after_target}")
    print(f"after_target_minus_start={vector_delta(after_target, start_feedback)}")
    print(f"after_target_minus_target={vector_delta(after_target, target)}")
    print(f"max_abs_after_target_minus_target={max_abs_delta(after_target, target)}")

    if return_original:
        print("Returning to original joint command.")
        count = move_arm_joints_toppra(master_arm, original_command, rate_hz)
        print(f"sent {count} TOPPRA return joint commands")
        time.sleep(0.5)
        final_feedback = joint_positions(master_arm)
        print(f"final_feedback={final_feedback}")
        print(f"final_minus_start={vector_delta(final_feedback, start_feedback)}")
        print(f"max_abs_final_minus_start={max_abs_delta(final_feedback, start_feedback)}")


def move_end_pose(
    master_arm,
    axis: str,
    delta_m: float,
    target_pose_text: str,
    rate_hz: float,
    hold: float,
    return_original: bool,
) -> None:
    set_master_mode(master_arm, "end_pose")
    current = master_arm.get_end_pose()
    original = pose_from_message_pose(current.pose)
    target = parse_target_pose(target_pose_text) if target_pose_text else make_delta_pose(original, axis, delta_m)

    print_message("start end pose", current)
    print(
        "target position="
        f"({target.position.x}, {target.position.y}, {target.position.z}), "
        "orientation="
        f"({target.orientation.x}, {target.orientation.y}, {target.orientation.z}, {target.orientation.w})"
    )

    count = move_arm_endpose_toppra(master_arm, target, rate_hz)
    print(f"sent {count} TOPPRA end-pose commands")
    time.sleep(hold)
    after_target = current_pose(master_arm)
    print(f"after_target_pose={json.dumps(pose_to_dict(after_target), ensure_ascii=False)}")
    print(f"after_target_position_delta_from_start={vector_delta(pose_position_list(after_target), pose_position_list(original))}")
    print(f"after_target_distance_to_target_m={pose_position_distance(after_target, target)}")

    if return_original:
        print("Returning to original end pose.")
        count = move_arm_endpose_toppra(master_arm, original, rate_hz)
        print(f"sent {count} TOPPRA return end-pose commands")
        time.sleep(0.5)
        final_pose = current_pose(master_arm)
        print(f"final_pose={json.dumps(pose_to_dict(final_pose), ensure_ascii=False)}")
        print(f"final_position_delta_from_start={vector_delta(pose_position_list(final_pose), pose_position_list(original))}")
        print(f"final_position_distance_from_start_m={pose_position_distance(final_pose, original)}")


def zero_master_arm(master_arm, mode: str, rate_hz: float, hold: float) -> None:
    if mode == "joint_pos":
        set_master_mode(master_arm, "joint_pos")
        current = master_arm.get_joint_states()
        start_feedback = [float(value) for value in current.position]
        target = list(ZERO_JOINT_POSITIONS)

        print(f"start_feedback={start_feedback}")
        print(f"target_command={target}")
        print("Zeroing joint positions. This action does not return to the original position.")

        count = move_arm_joints_toppra(master_arm, target, rate_hz)
        print(f"sent {count} TOPPRA joint zero commands")
        time.sleep(hold)
        after_target = joint_positions(master_arm)
        print(f"after_zero_feedback={after_target}")
        print(f"after_zero_minus_start={vector_delta(after_target, start_feedback)}")
        print(f"after_zero_minus_target={vector_delta(after_target, target)}")
        print(f"max_abs_after_zero_minus_target={max_abs_delta(after_target, target)}")
        return

    if mode == "end_pose":
        set_master_mode(master_arm, "end_pose")
        current = master_arm.get_end_pose()
        original = pose_from_message_pose(current.pose)
        target = zero_end_pose()

        print_message("start end pose", current)
        print(
            "target zero position="
            f"({target.position.x}, {target.position.y}, {target.position.z}), "
            "orientation="
            f"({target.orientation.x}, {target.orientation.y}, {target.orientation.z}, {target.orientation.w})"
        )
        print("Zeroing end pose. This action does not return to the original pose.")

        count = move_arm_endpose_toppra(master_arm, target, rate_hz)
        print(f"sent {count} TOPPRA end-pose zero commands")
        time.sleep(hold)
        after_target = current_pose(master_arm)
        print(f"after_zero_pose={json.dumps(pose_to_dict(after_target), ensure_ascii=False)}")
        print(f"after_zero_position_delta_from_start={vector_delta(pose_position_list(after_target), pose_position_list(original))}")
        print(f"after_zero_distance_to_target_m={pose_position_distance(after_target, target)}")
        return

    raise ValueError("mode must be joint_pos or end_pose")


def stream_master_arm(master_arm, stream: str, interval: float, samples: int) -> None:
    print("Streaming. Press Ctrl+C to stop.")
    count = 0

    def should_stop() -> bool:
        return samples > 0 and count >= samples

    try:
        if stream == "joint_states":
            for joint_state in master_arm.get_joint_states_stream():
                print_message("joint state", joint_state)
                count += 1
                if should_stop():
                    break
                time.sleep(interval)
        elif stream == "end_pose":
            for end_pose in master_arm.get_end_pose_stream():
                print_message("end pose", end_pose)
                count += 1
                if should_stop():
                    break
                time.sleep(interval)
        elif stream == "gripper_state":
            for gripper_state in master_arm.get_gripper_state_stream():
                print_message("gripper state", gripper_state)
                count += 1
                if should_stop():
                    break
                time.sleep(interval)
        elif stream == "gripper_joint_states":
            for gripper_joint_state in master_arm.get_gripper_joint_states_stream():
                print_message("gripper joint state", gripper_joint_state)
                count += 1
                if should_stop():
                    break
                time.sleep(interval)
        else:
            raise ValueError("stream must be joint_states, end_pose, gripper_state, or gripper_joint_states")
    except KeyboardInterrupt:
        print("\nStopping stream.")


def main(
    server: Annotated[str, typer.Option(help="server address, e.g. 192.168.10.1:50051")] = DEFAULT_SERVER,
    model: Annotated[str, typer.Option(help="client model registry: auto, desktop, or quanta_x1")] = DEFAULT_MODEL,
    arm: Annotated[str, typer.Option(help="left, right, or both")] = "left",
    action: Annotated[str, typer.Option(help="read, snapshot, set-mode, move, zero, or stream")] = "read",
    mode: Annotated[str, typer.Option(help="joint-pos or end-pose for set-mode/move/zero")] = "joint_pos",
    query: Annotated[str, typer.Option(help="all, control-mode, joint-states, end-pose, gripper-position")] = "all",
    stream: Annotated[str, typer.Option(help="joint-states, end-pose, gripper-state, gripper-joint-states")] = "joint_states",
    write: Annotated[bool, typer.Option(help="required for set-mode, move, and zero")] = False,
    joint: Annotated[str, typer.Option(help="joint name; overrides --joint-index")] = "",
    joint_index: Annotated[int, typer.Option(help="zero-based joint index")] = 0,
    joint_delta: Annotated[float, typer.Option(help="joint delta in radians")] = 0.262,
    target_positions: Annotated[str, typer.Option(help="comma-separated 6 joint targets; overrides joint delta")] = "",
    axis: Annotated[str, typer.Option(help="end-pose axis: x, y, or z")] = "z",
    pose_delta: Annotated[float, typer.Option(help="end-pose position delta in meters")] = 0.02,
    target_pose: Annotated[str, typer.Option(help="absolute pose x,y,z,qx,qy,qz,qw; overrides pose delta")] = "",
    rate_hz: Annotated[float, typer.Option(help="stream command rate")] = 200.0,
    hold: Annotated[float, typer.Option(help="seconds to hold target before return")] = 2.0,
    return_original: Annotated[bool, typer.Option(help="return to original after move")] = True,
    stream_interval: Annotated[float, typer.Option(help="seconds between printed stream samples")] = 0.1,
    samples: Annotated[int, typer.Option(help="stream sample count; 0 means unlimited")] = 0,
):
    """MasterArm-only example.

    This script intentionally uses only robot.master_left_arm / robot.master_right_arm.
    Ordinary robot.left_arm / robot.right_arm belong in arm_control.py.
    """
    action = normalize_name(action)
    mode = normalize_name(mode)
    query = normalize_name(query)
    stream = normalize_name(stream)

    if arm not in ("left", "right", "both"):
        raise typer.BadParameter("arm must be left, right, or both")
    if action not in ("read", "snapshot", "set_mode", "move", "zero", "stream"):
        raise typer.BadParameter("action must be read, snapshot, set-mode, move, zero, or stream")
    if mode not in ("joint_pos", "end_pose"):
        raise typer.BadParameter("mode must be joint-pos or end-pose")
    if query not in ("all", "control_mode", "joint_states", "end_pose", "gripper_position"):
        raise typer.BadParameter("query must be all, control-mode, joint-states, end-pose, or gripper-position")
    if stream not in ("joint_states", "end_pose", "gripper_state", "gripper_joint_states"):
        raise typer.BadParameter("stream must be joint-states, end-pose, gripper-state, or gripper-joint-states")
    if axis not in ("x", "y", "z"):
        raise typer.BadParameter("axis must be x, y, or z")
    validate_rate(rate_hz)
    if hold < 0:
        raise typer.BadParameter("hold must be >= 0")
    if samples < 0:
        raise typer.BadParameter("samples must be >= 0")
    if action in ("set_mode", "move", "zero"):
        require_write(write, action)
    if action == "stream" and arm == "both":
        raise typer.BadParameter("stream does not support --arm both; run one side per terminal")

    connect_model = None if model in ("", "auto") else model
    robot = connect(f"x2://{server}", model=connect_model)
    sides = selected_sides(arm)

    if action == "snapshot":
        query = "all"

    if action in ("read", "snapshot"):
        run_for_sides(robot, sides, f"read({query})", lambda side, master_arm: read_master_arm(master_arm, query))
    elif action == "set_mode":
        run_for_sides(robot, sides, f"set_mode({mode})", lambda side, master_arm: set_master_mode(master_arm, mode))
    elif action == "move":
        if mode == "joint_pos":
            run_for_sides(
                robot,
                sides,
                "move_joint",
                lambda side, master_arm: move_joint(
                    master_arm,
                    joint,
                    joint_index,
                    joint_delta,
                    target_positions,
                    rate_hz,
                    hold,
                    return_original,
                ),
            )
        else:
            run_for_sides(
                robot,
                sides,
                "move_end_pose",
                lambda side, master_arm: move_end_pose(
                    master_arm,
                    axis,
                    pose_delta,
                    target_pose,
                    rate_hz,
                    hold,
                    return_original,
                ),
            )
    elif action == "zero":
        run_for_sides(
            robot,
            sides,
            f"zero({mode})",
            lambda side, master_arm: zero_master_arm(master_arm, mode, rate_hz, hold),
        )
    elif action == "stream":
        master_arm = select_master_arm(robot, sides[0])
        stream_master_arm(master_arm, stream, stream_interval, samples)


if __name__ == "__main__":
    try:
        typer.run(main)
    except RuntimeError as exc:
        print(f"\nerror: {exc}")
        raise SystemExit(1)
