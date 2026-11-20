import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Annotated, Any, Iterable, Literal

import typer


ArmName = Literal["left", "right", "both"]


def signal_handler(sig, frame):
    print("\nReplay interrupted.")
    sys.exit(0)


def load_episode(episode_path: Path) -> dict[str, Any]:
    if episode_path.is_dir():
        episode_file = episode_path / "episode.json"
    else:
        episode_file = episode_path

    if not episode_file.exists():
        raise FileNotFoundError(f"episode file not found: {episode_file}")

    with episode_file.open("r", encoding="utf-8") as f:
        data = json.load(f)

    frames = data.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError("episode contains no frames")

    return data


def finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def pose_is_valid(pose_data: Any) -> bool:
    if not isinstance(pose_data, dict):
        return False

    position = pose_data.get("position")
    orientation = pose_data.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return False

    values = [
        position.get("x"),
        position.get("y"),
        position.get("z"),
        orientation.get("x"),
        orientation.get("y"),
        orientation.get("z"),
        orientation.get("w"),
    ]
    if not all(finite_number(value) for value in values):
        return False

    return not (
        position["x"] == 0
        and position["y"] == 0
        and position["z"] == 0
        and orientation["x"] == 0
        and orientation["y"] == 0
        and orientation["z"] == 0
        and orientation["w"] == 1
    )


def pose_from_dict(pose_data: dict[str, Any]) -> Any:
    from x2robot.geometry_msgs import Point, Pose, Quaternion

    position = pose_data["position"]
    orientation = pose_data["orientation"]
    return Pose(
        position=Point(
            x=position["x"],
            y=position["y"],
            z=position["z"],
        ),
        orientation=Quaternion(
            x=orientation["x"],
            y=orientation["y"],
            z=orientation["z"],
            w=orientation["w"],
        ),
    )


def get_pose_data(frame: dict[str, Any], part_name: str) -> dict[str, Any] | None:
    action = frame.get("action") or {}
    observation = frame.get("observation") or {}

    action_pose = action.get(f"{part_name}_end_pose_action")
    if pose_is_valid(action_pose):
        return action_pose

    observation_pose = observation.get(f"{part_name}_end_pose")
    if pose_is_valid(observation_pose):
        return observation_pose

    return None


def get_gripper_position(frame: dict[str, Any], side: Literal["left", "right"]) -> float | None:
    action = frame.get("action") or {}
    observation = frame.get("observation") or {}

    action_position = action.get(f"{side}_gripper_position_action")
    if isinstance(action_position, dict) and finite_number(action_position.get("position")):
        return action_position["position"]

    observation_joint_states = observation.get(f"{side}_gripper_joint_states")
    if isinstance(observation_joint_states, dict) and 'positions' in observation_joint_states:
        position = observation_joint_states['positions'][0]
        if (position < 0.0):
            return 0.0
        return position

    observation_position = observation.get(f"{side}_gripper_position")
    if isinstance(observation_position, dict) and finite_number(
        observation_position.get("position")
    ):
        return observation_position["position"]

    return None


def check_result(result: Any, name: str, frame_index: int) -> bool:
    if result is None or not hasattr(result, "is_success"):
        return True

    if result.is_success:
        return True

    error = getattr(result, "error_message", "")
    print(f"\nWarning: frame {frame_index} {name} control failed: {error}")
    return False


def selected_parts(arm: ArmName, include_waist: bool) -> Iterable[str]:
    if arm in ("left", "both"):
        yield "left_arm"
    if arm in ("right", "both"):
        yield "right_arm"
    if include_waist:
        yield "waist"


def selected_grippers(
    arm: ArmName, include_gripper: bool
) -> Iterable[Literal["left", "right"]]:
    if not include_gripper:
        return
    if arm in ("left", "both"):
        yield "left"
    if arm in ("right", "both"):
        yield "right"


def replay_end_pose(
    robot: Any,
    episode_data: dict[str, Any],
    arm: ArmName,
    speed: float,
    include_waist: bool,
    include_gripper: bool,
    dry_run: bool,
) -> None:
    from x2robot.sdk import (
        GripperPosition,
        ManipulatorControlMode,
        ManipulatorControlModeParam,
        RobotModeParam,
        RobotWorkMode,
    )

    robot_model = robot.get_robot_model()
    if robot_model != "quanta_x2":
        raise RuntimeError(f"this script only supports quanta_x2, got {robot_model}")

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))
    result = robot.robot_control.set_manipulator_control_mode(
        ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE)
    )
    if not check_result(result, "manipulator mode", 0):
        raise RuntimeError("failed to set manipulator end pose control mode")

    frames = episode_data["frames"]
    total_frames = len(frames)
    parts = tuple(selected_parts(arm, include_waist))
    grippers = tuple(selected_grippers(arm, include_gripper))

    print("Replay configuration:")
    print(f"  model: {robot_model}")
    print(f"  frames: {total_frames}")
    print(f"  speed: {speed}x")
    print(f"  parts: {', '.join(parts)}")
    print(f"  grippers: {', '.join(grippers) if grippers else 'disabled'}")
    print("  pose data source: action end pose, fallback to observation end pose")
    print("  gripper data source: action gripper position, fallback to observation")
    if dry_run:
        print("  dry run: enabled")

    start_time = time.time()
    sent_frames = 0
    sent_pose_commands = 0
    sent_gripper_commands = 0

    for index, frame in enumerate(frames):
        frame_sent = False

        for part_name in parts:
            pose_data = get_pose_data(frame, part_name)
            if pose_data is None:
                continue

            if not dry_run:
                controller = getattr(robot, part_name)
                result = controller.set_end_pose(pose_from_dict(pose_data))
                if not check_result(result, part_name, index):
                    continue

            frame_sent = True
            sent_pose_commands += 1

        for side in grippers:
            position = get_gripper_position(frame, side)
            if position is None:
                continue

            # if not dry_run:
            controller = getattr(robot, f"{side}_gripper")
            #import pdb;pdb.set_trace()
            result = controller.set_position(GripperPosition(position=position))
            if not check_result(result, f"{side}_gripper", index):
                continue

            frame_sent = True
            sent_gripper_commands += 1

        if frame_sent:
            sent_frames += 1

        if index % 10 == 0 or index == total_frames - 1:
            progress = (index + 1) / total_frames * 100
            elapsed = time.time() - start_time
            print(
                f"Progress: {progress:5.1f}% ({index + 1}/{total_frames}) | "
                f"sent frames: {sent_frames} | elapsed: {elapsed:.2f}s",
                end="\r",
            )

        if index < total_frames - 1:
            current_timestamp = frame.get("timestamp")
            next_timestamp = frames[index + 1].get("timestamp")
            if finite_number(current_timestamp) and finite_number(next_timestamp):
                sleep_time = (next_timestamp - current_timestamp) / speed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    elapsed = time.time() - start_time
    print(
        f"\nReplay completed. Sent {sent_pose_commands} end-pose commands and "
        f"{sent_gripper_commands} gripper commands "
        f"from {sent_frames}/{total_frames} frames in {elapsed:.2f}s."
    )


def dry_run_episode(
    episode_data: dict[str, Any],
    arm: ArmName,
    speed: float,
    include_waist: bool,
    include_gripper: bool,
) -> None:
    frames = episode_data["frames"]
    parts = tuple(selected_parts(arm, include_waist))
    grippers = tuple(selected_grippers(arm, include_gripper))
    total_pose_commands = 0
    total_gripper_commands = 0
    valid_frames = 0

    for frame in frames:
        frame_pose_commands = sum(
            1 for part_name in parts if get_pose_data(frame, part_name) is not None
        )
        frame_gripper_commands = sum(
            1 for side in grippers if get_gripper_position(frame, side) is not None
        )
        frame_commands = frame_pose_commands + frame_gripper_commands
        if frame_commands:
            valid_frames += 1
            total_pose_commands += frame_pose_commands
            total_gripper_commands += frame_gripper_commands

    duration = 0.0
    if len(frames) > 1:
        first_timestamp = frames[0].get("timestamp")
        last_timestamp = frames[-1].get("timestamp")
        if finite_number(first_timestamp) and finite_number(last_timestamp):
            duration = max(0.0, (last_timestamp - first_timestamp) / speed)

    print("Dry-run result:")
    print(f"  frames: {len(frames)}")
    print(f"  valid frames: {valid_frames}")
    print(f"  end-pose commands: {total_pose_commands}")
    print(f"  gripper commands: {total_gripper_commands}")
    print(f"  parts: {', '.join(parts)}")
    print(f"  grippers: {', '.join(grippers) if grippers else 'disabled'}")
    print(f"  estimated replay time: {duration:.2f}s")


def main(
    episode: Annotated[
        Path,
        typer.Argument(
            help="Episode directory or episode.json path, e.g. ./pick_up_the_teddy_bear/episode_0000"
        ),
    ],
    server: Annotated[
        str,
        typer.Option(help="Robot server address, e.g. localhost:50051"),
    ] = "localhost:50051",
    arm: Annotated[
        ArmName,
        typer.Option(help="Arm end pose to replay: left, right, or both"),
    ] = "both",
    speed: Annotated[
        float,
        typer.Option(help="Replay speed multiplier, e.g. 1.0 normal, 0.5 slow, 2.0 fast"),
    ] = 1.0,
    include_waist: Annotated[
        bool,
        typer.Option(
            "--include-waist/--no-waist",
            help="Replay quanta_x2 waist_end_pose together with arm end poses",
        ),
    ] = True,
    include_gripper: Annotated[
        bool,
        typer.Option(
            "--include-gripper/--no-gripper",
            help="Replay quanta_x2 left/right gripper positions together with arm end poses",
        ),
    ] = True,
    dry_run: Annotated[
        bool,
        typer.Option(help="Parse data and timing without sending robot commands"),
    ] = False,
):
    if speed <= 0:
        raise typer.BadParameter("speed must be greater than 0")

    signal.signal(signal.SIGINT, signal_handler)

    try:
        episode_data = load_episode(episode)
    except Exception as exc:
        print(f"Failed to load episode: {exc}")
        raise typer.Exit(1) from exc

    record_model = episode_data.get("model")
    if record_model and record_model != "quanta_x2":
        print(f"Episode model is {record_model}, but this script only supports quanta_x2.")
        raise typer.Exit(1)

    if dry_run:
        dry_run_episode(episode_data, arm, speed, include_waist, include_gripper)
        return

    print(f"Connecting to x2://{server}...")
    try:
        from x2robot import connect

        robot = connect(f"x2://{server}")
    except Exception as exc:
        print(f"Failed to connect robot: {exc}")
        raise typer.Exit(1) from exc

    try:
        replay_end_pose(
            robot,
            episode_data,
            arm,
            speed,
            include_waist,
            include_gripper,
            dry_run,
        )
    except Exception as exc:
        print(f"\nReplay failed: {exc}")
        raise typer.Exit(1) from exc


if __name__ == "__main__":
    typer.run(main)
