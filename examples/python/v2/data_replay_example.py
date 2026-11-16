"""Replay Python JSON or C++ protobuf episodes.

Quanta X3 end-pose replay refreshes both arms and the waist on a common 50 Hz
timeline. Other controls follow recorded timestamps. Use --no-chassis to omit
all chassis commands. All SDK results are checked before replay continues.
Use --dry-run to validate data and print a replay plan without connecting.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import x2robot_sdk
from x2robot.geometry_msgs import Point, Pose, Quaternion
from x2robot.sdk import (
    ChassisControlMode,
    ChassisControlModeParam,
    ChassisVelocity,
    GripperPosition,
    HeadPose,
    JointPositions,
    LiftPosition,
    ManipulatorControlMode,
    ManipulatorControlModeParam,
    RobotModeParam,
    RobotWorkMode,
)


try:
    from .replay_joint_limits import prepare_joint_commands
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from replay_joint_limits import prepare_joint_commands


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation} failed: {error.get('error_name', 'UNKNOWN')}: "
            f"{error.get('error_message', 'SDK call failed')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def check_end_pose_result(result, operation):
    # SDK TIMEOUT (100504) includes gRPC DEADLINE_EXCEEDED. A missed reply
    # does not establish whether the target was executed; never resend it.
    if not result.is_success and (result.error or {}).get("error_code") == 100504:
        print(f"WARNING: {operation} timed out; execution unconfirmed; continuing without retry: "
              f"{result.error.get('error_message', 'SDK call timed out')} "
              f"(request_id={result.request_id or '-'})", file=sys.stderr, flush=True)
        return False
    require(result, operation)
    return True


def load_episode_data(path):
    episode = Path(path) / "episode.json"
    if not episode.exists():
        raise FileNotFoundError(episode)
    data = json.loads(episode.read_text(encoding="utf-8"))
    if "frames" not in data and data.get("format") == "sdk-protobuf-record-v1":
        try:
            from .data_collection.replay_input import load_protobuf_frames
        except ImportError:
            from data_collection.replay_input import load_protobuf_frames
        data["frames"] = load_protobuf_frames(episode.parent)
    print(f"Loaded {episode}: {data.get('num_frames', len(data.get('frames', [])))} frames, "
          f"duration={float(data.get('duration', 0.0)):.2f}s")
    return data


def finite(values):
    return [float(value) for value in values if value is not None and not (isinstance(value, float) and math.isnan(value))]


def model_name(robot):
    identity = getattr(robot, "model_name", None)
    if isinstance(identity, str) and identity.lower() in {"quanta_x1", "quanta_x2", "quanta_x3", "desktop"}:
        return identity.lower()
    value = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType")
    text = str(getattr(value, "model_type", value)).upper()
    if "X1" in text:
        return "quanta_x1"
    if "X2" in text:
        return "quanta_x2"
    if "X3" in text:
        return "quanta_x3"
    return text.lower()


def _yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class ChassisController(threading.Thread):
    """100 Hz proportional chassis tracker, matching the v1 replay helper."""

    def __init__(self, robot, rate=100.0, model=None):
        super().__init__(daemon=True)
        self.robot = robot
        self.period = 1.0 / rate
        self.target = None
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.running = True
        # Quanta X3 MCU caps are 0.45/0.20 m/s and 0.40 rad/s.
        # Preserve replay's existing lower forward cap and other models' limits.
        self.lateral_limit = 0.20 if model == "quanta_x3" else 0.3
        self.angular_limit = 0.40 if model == "quanta_x3" else 0.5

    def set_target(self, target):
        with self.lock:
            self.target = target

    def stop(self):
        self.running = False
        try:
            # Serialize the final zero with any in-flight tracking command.
            with self.command_lock:
                require(self.robot.chassis.set_velocity(ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0), timeout=1.0), "Chassis.SetVelocity(stop)")
        except Exception:
            pass

    def run(self):
        while self.running:
            started = time.monotonic()
            try:
                with self.lock:
                    target = self.target
                if target is None:
                    command = ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0)
                else:
                    odom = require(self.robot.chassis.get_odometry(timeout=1.0), "Chassis.GetOdometry")
                    current = odom.pose.pose
                    desired = target.get("pose", target)
                    position = desired.get("position", {})
                    orientation = desired.get("orientation", {})
                    dx = float(position.get("x", 0.0)) - float(current.position.x)
                    dy = float(position.get("y", 0.0)) - float(current.position.y)
                    dyaw = float(orientation.get("z", 0.0))  # replaced below when w is present
                    target_yaw = math.atan2(2 * (float(orientation.get("w", 1.0)) * float(orientation.get("z", 0.0)) + float(orientation.get("x", 0.0)) * float(orientation.get("y", 0.0))), 1 - 2 * (float(orientation.get("y", 0.0)) ** 2 + float(orientation.get("z", 0.0)) ** 2))
                    dyaw = target_yaw - _yaw(current.orientation)
                    while dyaw > math.pi:
                        dyaw -= 2 * math.pi
                    while dyaw < -math.pi:
                        dyaw += 2 * math.pi
                    distance = math.hypot(dx, dy)
                    if distance < 0.01 and abs(dyaw) < 0.05:
                        command = ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0)
                    else:
                        heading = math.atan2(dy, dx) - _yaw(current.orientation)
                        command = ChassisVelocity(
                            vel_x=max(-0.3, min(0.3, 2.0 * distance * math.cos(heading))),
                            vel_y=max(-self.lateral_limit, min(self.lateral_limit, 2.0 * distance * math.sin(heading))),
                            vel_yaw=max(-self.angular_limit, min(self.angular_limit, 2.0 * dyaw)),
                        )
                with self.command_lock:
                    if not self.running:
                        break
                    require(self.robot.chassis.set_velocity(command, timeout=1.0), "Chassis.SetVelocity")
            except Exception:
                # Preserve v1's best-effort loop; the main replay still reports
                # failures from individual frame commands.
                pass
            remaining = self.period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)


def _arm(robot, side):
    return robot.left_arm_controller if side == "left" else robot.right_arm_controller


# Quanta X3 G 夹爪上限 1.526 rad 向内取 float32 可表示值，兼容旧服务端的严格比较。
X3_GRIPPER_UPPER = 1.525999903678894


def gripper_replay_value(observation, key):
    state = observation[key]
    if not isinstance(state, dict):
        raise ValueError(f"{key}: invalid gripper record")
    value = state.get("position")
    if key.endswith("joint_states"):
        values = state.get("positions")
        if not isinstance(values, (list, tuple)) or not values:
            raise ValueError(f"{key}: invalid or empty gripper joint state")
        value = values[0]
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{key}: invalid gripper position {value!r}") from error
    if not math.isfinite(value):
        raise ValueError(f"{key}: non-finite gripper position {value!r}")
    return value


def gripper_replay_target(value):
    return min(max(value, 0.0), X3_GRIPPER_UPPER)


def validate_gripper_replay(data, model):
    if model != "quanta_x3":
        return
    summaries = {}
    for index, frame in enumerate(data.get("frames", []), 1):
        observation = frame.get("observation", {})
        location = f"frame={index} timestamp={frame.get('timestamp')}"
        for side in ("left", "right"):
            for suffix in ("joint_states", "position"):
                key = f"{side}_gripper_{suffix}"
                if key not in observation:
                    continue
                try:
                    value = gripper_replay_value(observation, key)
                except ValueError as error:
                    raise ValueError(f"Gripper replay: {location} {error}") from error
                summary = summaries.setdefault(side, dict(clamped=0, minimum=math.inf, maximum=-math.inf, first=""))
                summary["minimum"] = min(summary["minimum"], value)
                summary["maximum"] = max(summary["maximum"], value)
                if value < 0.0 or value > X3_GRIPPER_UPPER:
                    summary["clamped"] += 1
                    if not summary["first"]:
                        summary["first"] = f" first={key} {location} value={value!r} target={gripper_replay_target(value)!r}"
    for side, summary in summaries.items():
        print(f"Gripper replay: side={side} range=[0, 1.526] rad clamped={summary['clamped']} "
              f"recorded_min={summary['minimum']!r} recorded_max={summary['maximum']!r}{summary['first']}")


def control_grippers(robot, observation, model=None):
    for side in ("left", "right"):
        if model == "quanta_x3":
            key = f"{side}_gripper_joint_states"
            if key not in observation:
                key = f"{side}_gripper_position"
            if key not in observation:
                continue
            target = gripper_replay_target(gripper_replay_value(observation, key))
        else:
            state = observation.get(f"{side}_gripper_joint_states")
            values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
            if not values:
                position = observation.get(f"{side}_gripper_position")
                if isinstance(position, dict):
                    values = finite([position.get("position")])
            if not values:
                continue
            target = max(0.0, values[0])
        require(
            (robot.left_gripper if side == "left" else robot.right_gripper).set_position(
                GripperPosition(position=target), timeout=1.0
            ),
            f"{side.title()}Gripper.SetPosition",
        )


def control_head(robot, observation):
    state = observation.get("head_joint_states")
    values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
    if values:
        require(robot.head.set_pose(HeadPose(pitch=values[0] if len(values) > 1 else 0.0, yaw=values[1] if len(values) > 1 else values[0]), timeout=1.0), "Head.SetPose")


def control_aux(robot, observation, model, end_pose=False):
    if model == "quanta_x1":
        state = observation.get("lift_joint_states")
        values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
        if values:
            require(robot.lift.set_lift_position(LiftPosition(position=values[0]), timeout=1.0), "Lift.SetLiftPosition")
    elif model in ("quanta_x2", "quanta_x3"):
        state = observation.get("waist_joint_states")
        values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
        if values and not end_pose:
            require(robot.waist.set_joint_positions(JointPositions(positions=values), timeout=1.0), "Waist.SetJointPositions")
        if end_pose:
            pose = observation.get("waist_end_pose")
            if isinstance(pose, dict) and pose.get("position") and pose.get("orientation"):
                require(robot.waist.set_end_pose(_pose_from_dict(pose), timeout=1.0), "Waist.SetEndPose")


def _pose_from_dict(value):
    position = value["position"]
    orientation = value["orientation"]
    return Pose(
        position=Point(x=float(position.get("x", 0.0)), y=float(position.get("y", 0.0)), z=float(position.get("z", 0.0))),
        orientation=Quaternion(x=float(orientation.get("x", 0.0)), y=float(orientation.get("y", 0.0)), z=float(orientation.get("z", 0.0)), w=float(orientation.get("w", 1.0))),
    )


def _send_chassis_target(controller, frame):
    if controller is not None:
        target = frame.get("observation", {}).get("odometry")
        if target:
            controller.set_target(target)


def replay_frames(data, speed):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed must be finite and > 0")
    frames = sorted(data.get("frames", []), key=lambda frame: float(frame.get("timestamp", 0)))
    if not frames or any(not math.isfinite(float(frame.get("timestamp", 0))) for frame in frames):
        raise ValueError("replay requires observations with finite timestamps")
    return frames


OFFLINE_MODELS = {
    "quanta_x1": "quanta_x1", "ex001": "quanta_x1",
    "quanta_x2": "quanta_x2", "cx002": "quanta_x2",
    "quanta_x3": "quanta_x3",
    "desktop": "desktop",
}


def dry_run_plan(data, mode, speed=1.0, model=None, replay_chassis=True):
    """Validate the same inputs as live replay; counts describe source records, not RPCs."""
    recorded = str(data.get("model") or "").strip().lower()
    recorded = OFFLINE_MODELS.get(recorded, recorded)
    selected = OFFLINE_MODELS.get(model, model) if model else recorded
    if selected not in set(OFFLINE_MODELS.values()):
        raise ValueError("dry-run requires a known episode model; use --model quanta_x1|quanta_x2|quanta_x3|desktop for old recordings")
    if model and recorded and selected != recorded:
        raise ValueError(f"dry-run model mismatch: requested={selected!r}, recorded={recorded!r}")
    prepared, joint_stats = prepare_joint_commands(data, selected, mode)
    frames = replay_frames(prepared, speed)
    validate_gripper_replay(prepared, selected)
    if mode == "end_pose":
        validate_end_pose_replay(prepared, selected)
        if selected == "quanta_x3":
            prepare_pose_timelines(frames)

    applicable = {"head_joint_states", "left_gripper_joint_states", "right_gripper_joint_states",
                  "left_gripper_position", "right_gripper_position"}
    arm_suffix = "joint_states" if mode == "joint_pos" else "end_pose"
    applicable.update(f"{side}_arm_{arm_suffix}" for side in ("left", "right"))
    if selected == "quanta_x1":
        applicable.add("lift_joint_states")
    elif selected in {"quanta_x2", "quanta_x3"}:
        applicable.add(f"waist_{arm_suffix}")
    if replay_chassis and selected != "desktop":
        applicable.add("odometry")
    streams, timestamps = {}, {}
    for frame in frames:
        stamp = float(frame.get("timestamp", 0))
        for name in frame.get("observation", {}):
            row = streams.setdefault(name, dict(records=0, planned=0, ignored=0))
            row["records"] += 1
            row["planned" if name in applicable else "ignored"] += 1
            first, _ = timestamps.setdefault(name, (stamp, stamp))
            timestamps[name] = (first, stamp)
    for name, row in streams.items():
        first, last = timestamps[name]
        row["hz"] = (row["records"] - 1) / (last - first) if last > first else 0.0
    duration = float(frames[-1].get("timestamp", 0)) - float(frames[0].get("timestamp", 0))
    return dict(model=selected, mode=mode, speed=speed, source_frames=len(frames),
                duration_s=duration, planned_duration_s=duration / speed,
                streams=streams, joint_clamp=joint_stats)


def wait_for_frame(started, first_stamp, frame, speed):
    remaining = started + (float(frame.get("timestamp", 0)) - first_stamp) / speed - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


def start_replay_chassis(robot, frames, model, enabled):
    if not enabled or model == "desktop" or not any("odometry" in f.get("observation", {}) for f in frames):
        return None
    require(robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY), timeout=3.0), "Chassis.SetControlMode")
    chassis = ChassisController(robot, model=model)
    chassis.start()
    return chassis


def replay_by_joint_positions(robot, data, speed=1.0, model=None, replay_chassis=True):
    model = model or model_name(robot)
    data, joint_stats = prepare_joint_commands(data, model, "joint_pos")
    print("Joint clamp plan: " + json.dumps(joint_stats), flush=True)
    frames = replay_frames(data, speed)
    validate_gripper_replay(data, model)
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS), timeout=3.0), "RobotControl.SetManipulatorControlMode")
    chassis = start_replay_chassis(robot, frames, model, replay_chassis)
    started = time.monotonic()
    first_stamp = float(frames[0].get("timestamp", 0))
    try:
        for index, frame in enumerate(frames):
            wait_for_frame(started, first_stamp, frame, speed)
            observation = frame.get("observation", {})
            _send_chassis_target(chassis, frame)
            for side in ("left", "right"):
                state = observation.get(f"{side}_arm_joint_states")
                values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
                if values:
                    require(_arm(robot, side).set_joint_positions(values, deadline_ms=1000), f"{side.title()}Arm.SetJointPositions")
            control_aux(robot, observation, model)
            control_grippers(robot, observation, model)
            control_head(robot, observation)
            if index % 10 == 0:
                print(f"Progress: {index + 1}/{len(frames)}", end="\r")
    finally:
        if chassis is not None:
            chassis.stop()
            chassis.join(timeout=1.0)
    print(f"\nReplay completed in {time.monotonic() - started:.2f}s ({len(frames)} frames)")


def validate_end_pose_replay(data, model):
    if model != "quanta_x3":
        return
    required = {"left_arm_end_pose", "right_arm_end_pose", "waist_end_pose"}
    present = set()
    for frame in data.get("frames", []):
        observation = frame.get("observation", {})
        for name in required:
            pose = observation.get(name)
            if isinstance(pose, dict) and pose.get("position") and pose.get("orientation"):
                present.add(name)
    missing = sorted(required - present)
    if missing:
        raise ValueError("Quanta X3 end_pose replay requires both arms and waist; "
                         "missing recorded pose streams: " + ", ".join(missing))


POSE_STREAMS = ("left_arm_end_pose", "right_arm_end_pose", "waist_end_pose")
POSE_PERIOD = 0.02


def prepare_pose_timelines(frames):
    try:
        from .data_collection.pose_timeline import PoseTimeline
    except ImportError:
        from data_collection.pose_timeline import PoseTimeline
    timelines = {}
    for name in POSE_STREAMS:
        samples = [(float(f.get("timestamp", 0)), f["observation"][name])
                   for f in frames if name in f.get("observation", {})]
        try:
            timelines[name] = PoseTimeline(samples)
        except (ValueError, KeyError, TypeError) as error:
            raise ValueError(f"{name}: invalid recorded pose: {error}") from error
    return timelines


def replay_x3_poses(robot, frames, timelines, speed, chassis):
    """Refresh all WBC targets together; source events never dictate RPC cadence."""
    first = float(frames[0].get("timestamp", 0))
    last = float(frames[-1].get("timestamp", 0))
    duration = last - first
    started = time.monotonic()
    index, cycles, max_refresh = 0, 0, 0.0
    pose_timeouts = 0
    played = 0.0
    previous_cycle = None
    print("Synchronized Quanta X3 end_pose replay: 50 Hz, left/right/waist every cycle")
    while True:
        cycle = time.monotonic()
        if previous_cycle is not None:
            max_refresh = max(max_refresh, cycle - previous_cycle)
        previous_cycle = cycle
        stamp = last if played >= duration else first + played
        # All three targets use the SAME playback timestamp, even across
        # missing source intervals. No joint-space fallback or homing occurs.
        poses = {name: _pose_from_dict(timelines[name].at(stamp)) for name in POSE_STREAMS}
        if not check_end_pose_result(_arm(robot, "left").set_end_pose(poses[POSE_STREAMS[0]], deadline_ms=300), "LeftArm.SetEndPose"):
            pose_timeouts += 1
        if not check_end_pose_result(_arm(robot, "right").set_end_pose(poses[POSE_STREAMS[1]], deadline_ms=300), "RightArm.SetEndPose"):
            pose_timeouts += 1
        if not check_end_pose_result(robot.waist.set_end_pose(poses[POSE_STREAMS[2]], timeout=.3), "Waist.SetEndPose"):
            pose_timeouts += 1
        cycles += 1
        # Preserve discrete head/gripper events. The high-rate pose events
        # above are interpolated instead of being drained as a stale backlog.
        while index < len(frames) and float(frames[index].get("timestamp", 0)) <= stamp:
            frame = frames[index]
            _send_chassis_target(chassis, frame)
            control_grippers(robot, frame.get("observation", {}), "quanta_x3")
            control_head(robot, frame.get("observation", {}))
            index += 1
        if played >= duration:
            break
        # Advance one recorded-time step only. Communication delays stretch
        # replay instead of skipping targets or sending a catch-up burst.
        # Count RPC time toward the period and wait only for the remainder.
        # Clamp in the recorded timestamp domain to avoid a duplicate final
        # cycle from subtracting large absolute timestamps.
        next_played = min(first + cycles * POSE_PERIOD * speed, last) - first
        remaining = cycle + (next_played - played) / speed - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        played = next_played
    print(f"Replay completed: source_frames={len(frames)}, pose_cycles={cycles}, "
          f"wall={time.monotonic()-started:.3f}s, recorded={duration:.3f}s, "
          f"max_refresh={max_refresh*1000:.1f}ms, pose_timeouts={pose_timeouts}")


def replay_by_end_pose(robot, data, speed=1.0, model=None, replay_chassis=True):
    model = model or model_name(robot)
    data, joint_stats = prepare_joint_commands(data, model, "end_pose")
    print("Joint clamp plan: " + json.dumps(joint_stats), flush=True)
    frames = replay_frames(data, speed)
    validate_gripper_replay(data, model)
    validate_end_pose_replay(data, model)
    timelines = prepare_pose_timelines(frames) if model == "quanta_x3" else None
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE), timeout=3.0), "RobotControl.SetManipulatorControlMode")
    chassis = start_replay_chassis(robot, frames, model, replay_chassis)
    started = time.monotonic()
    first_stamp = float(frames[0].get("timestamp", 0))
    try:
        if timelines is not None:
            replay_x3_poses(robot, frames, timelines, speed, chassis)
            return
        for index, frame in enumerate(frames):
            wait_for_frame(started, first_stamp, frame, speed)
            observation = frame.get("observation", {})
            _send_chassis_target(chassis, frame)
            control_aux(robot, observation, model, end_pose=True)
            for side in ("left", "right"):
                value = observation.get(f"{side}_arm_end_pose")
                if isinstance(value, dict) and value.get("position") and value.get("orientation"):
                    require(_arm(robot, side).set_end_pose(_pose_from_dict(value), deadline_ms=1000), f"{side.title()}Arm.SetEndPose")
            control_grippers(robot, observation, model)
            control_head(robot, observation)
            if index % 10 == 0:
                print(f"Progress: {index + 1}/{len(frames)}", end="\r")
    finally:
        if chassis is not None:
            chassis.stop()
            chassis.join(timeout=1.0)
    print(f"\nReplay completed ({len(frames)} frames)")


def verify(robot, model="auto"):
    """Validate the replay dependencies without sending motion commands.

    Replay is deliberately an explicit, user-confirmed operation.  The
    aggregate example smoke test therefore checks the read-only model query
    and the generated command/message surface, while :func:`main` exercises
    the complete timestamped replay loop after the user supplies an episode
    and confirms motion.
    """
    detected = model_name(robot)
    if model not in ("auto", detected,
                     "ex001" if detected == "quanta_x1" else None,
                     "cx002" if detected == "quanta_x2" else None,
                     "desktop" if detected == "desktop" else None):
        raise RuntimeError(f"replay model mismatch: requested={model!r}, detected={detected!r}")
    # Construct representative messages to catch stale/generated API drift;
    # no command is sent to the robot by this check.
    ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=0.0)
    JointPositions(positions=[])
    GripperPosition(position=0.0)
    HeadPose(pitch=0.0, yaw=0.0)
    print(f"Replay contract ready: model={detected}")


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
        return True
    _sdk_mode_entered = False
    try:
        require(robot.system.set_work_mode(
            # Task shutdown (11 s) and WBC STOP (21 s) run sequentially.
            # Wait for this one request; a timeout must not trigger a retry.
            RobotModeParam(mode=RobotWorkMode.IDLE), timeout=35.0
        ), "System.SetWorkMode(IDLE)")
        return True
    except Exception as error:  # 清理失败不能掩盖示例本身的错误
        print(f"FAIL: failed to leave SDK mode: {error}")
        return False

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode_dir")
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--no-confirm", action="store_true")
    parser.add_argument("--no-chassis", action="store_true", help="Skip all chassis mode and motion commands")
    parser.add_argument("--dry-run", action="store_true", help="validate data and print replay/clipping statistics without connecting to a robot")
    parser.add_argument("--model", choices=sorted(OFFLINE_MODELS), help="dry-run only: model for old recordings without model metadata")
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and > 0")
    if args.model and not args.dry_run:
        parser.error("--model is only supported with --dry-run; live replay detects the connected model")
    if args.dry_run:
        try:
            data = load_episode_data(args.episode_dir)
            plan = dry_run_plan(data, args.mode, args.speed, args.model, not args.no_chassis)
            print("Replay dry-run: " + json.dumps(plan, ensure_ascii=False, indent=2, allow_nan=False))
            return 0
        except (OSError, ValueError, TypeError, KeyError, OverflowError) as error:
            print(f"FAIL: dry-run: {error}", file=sys.stderr)
            return 1
    data = load_episode_data(args.episode_dir)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    exit_code, interrupted = 0, False
    try:
        model = model_name(robot)
        recorded = str(data.get("model", model))
        print(f"Replay configuration: mode={args.mode}, speed={args.speed}x, model={model}, recorded={recorded}")
        if not args.no_confirm:
            input("Press Enter to start replay (Ctrl+C to cancel)...")
        if args.mode == "joint_pos":
            replay_by_joint_positions(robot, data, args.speed, model, not args.no_chassis)
        else:
            replay_by_end_pose(robot, data, args.speed, model, not args.no_chassis)
    except KeyboardInterrupt:
        print("\nReplay interrupted")
        interrupted = True
    except Exception as error:
        print(f"FAIL: {error}")
        exit_code = 1
    finally:
        try:
            if not restore_idle_mode(robot):
                exit_code = 1
        finally:
            try:
                require(robot.close(), "Robot.close")
            except Exception as error:
                print(f"FAIL: {error}")
                exit_code = 1
    if exit_code == 0 and not interrupted:
        print("PASS: data replay")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
