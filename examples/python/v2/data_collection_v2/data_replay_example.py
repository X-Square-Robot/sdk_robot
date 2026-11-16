"""Replay an episode exported from an SDK Recording rosbag.

The replay order and timing follow the v1 example: chassis is controlled by a
small 100 Hz proportional loop, then arm/waist(or lift)/gripper/head commands
are sent for each recorded frame.  All v2 calls return ``SdkResult`` and are
checked at this boundary.
"""

from __future__ import annotations

import argparse
import configparser
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
    from ..replay_joint_limits import prepare_joint_commands
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
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


class ReplayCommandWarnings:
    """Warn and count rejected motion commands without aborting later frames."""

    def __init__(self):
        self.counts = {}
        self.last_warning = {}
        self.lock = threading.Lock()

    def warn(self, operation, message):
        with self.lock:
            self.counts[operation] = self.counts.get(operation, 0) + 1
            now = time.monotonic()
            if operation not in self.last_warning or now - self.last_warning[operation] >= 5.0:
                self.last_warning[operation] = now
                print(f"WARN: {operation}: {message}; continuing replay "
                      f"(failures={self.counts[operation]})", flush=True)

    def check(self, result, operation):
        if result.is_success:
            return result.data
        error = result.error or {}
        self.warn(operation, f"{error.get('error_name', 'UNKNOWN')}: "
                  f"{error.get('error_message', 'SDK call failed')} "
                  f"(request_id={result.request_id or '-'})")
        return None

    def summary(self):
        with self.lock:
            total = sum(self.counts.values())
            print("Replay command summary: " + json.dumps(
                {'rejected': total, 'operations': self.counts}, sort_keys=True), flush=True)
            return total


def load_episode_data(path):
    episode = Path(path) / "episode.json"
    if not episode.exists():
        raise FileNotFoundError(episode)
    data = json.loads(episode.read_text(encoding="utf-8"))
    print(f"Loaded {episode}: {data.get('num_frames', len(data.get('frames', [])))} frames, "
          f"duration={float(data.get('duration', 0.0)):.2f}s")
    return data


def has_odometry(data):
    return any(frame.get('observation', {}).get('odometry') for frame in data.get('frames', []))


def wait_until_frame(frame, first_stamp, started, speed):
    remaining = started + (float(frame.get('timestamp', first_stamp))-first_stamp)/speed - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)


def replay_stream_applies(name, mode):
    return (name in {'odometry', 'head_joint_states', 'lift_joint_states',
               'left_gripper_joint_states', 'right_gripper_joint_states',
               'left_gripper_position', 'right_gripper_position'} or
               (mode == 'joint_pos' and name in {'left_arm_joint_states', 'right_arm_joint_states', 'waist_joint_states'}) or
               (mode == 'end_pose' and name in {'left_arm_end_pose', 'right_arm_end_pose', 'waist_end_pose'}))


def replay_plan(data, mode):
    counts = {}
    for frame in data.get('frames', []):
        for name in frame.get('observation', {}):
            applies = replay_stream_applies(name, mode)
            row = counts.setdefault(name, {'records': 0, 'planned': 0, 'ignored': 0})
            row['records'] += 1
            row['planned' if applies else 'ignored'] += 1
    return counts


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

    def __init__(self, robot, rate=100.0, warnings=None):
        super().__init__(daemon=True)
        self.robot = robot
        self.warnings = warnings or ReplayCommandWarnings()
        self.period = 1.0 / rate
        self.target = None
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.running = True

    def set_target(self, target):
        with self.lock:
            self.target = target

    def stop(self):
        self.running = False
        try:
            # Serialize the final zero with any in-flight tracking command.
            with self.command_lock:
                self.warnings.check(self.robot.chassis.set_velocity(ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0), timeout=1.0), "Chassis.SetVelocity(stop)")
        except Exception as error:
            self.warnings.warn("Chassis.SetVelocity(stop)", str(error))

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
                            vel_y=max(-0.3, min(0.3, 2.0 * distance * math.sin(heading))),
                            vel_yaw=max(-0.5, min(0.5, 2.0 * dyaw)),
                        )
                with self.command_lock:
                    if not self.running:
                        break
                    self.warnings.check(self.robot.chassis.set_velocity(command, timeout=1.0), "Chassis.SetVelocity")
            except Exception as error:
                self.warnings.warn("Chassis.Update", str(error))
            remaining = self.period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)


def _arm(robot, side):
    return robot.left_arm_controller if side == "left" else robot.right_arm_controller


def control_grippers(robot, observation, warnings=None):
    check = warnings.check if warnings is not None else require
    for side in ("left", "right"):
        state = observation.get(f"{side}_gripper_joint_states")
        values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
        if not values:
            position = observation.get(f"{side}_gripper_position")
            if isinstance(position, dict):
                values = finite([position.get("position")])
        if values:
            check(
                (robot.left_gripper if side == "left" else robot.right_gripper).set_position(
                    GripperPosition(position=max(0.0, values[0])), timeout=1.0
                ),
                f"{side.title()}Gripper.SetPosition",
            )


START_LIMITS = {
    'joint_rad': 0.1, 'position_m': 0.03, 'orientation_rad': 0.1,
    'lift_m': 0.02, 'gripper': 0.1, 'chassis_position_m': 0.05,
    'chassis_yaw_rad': 0.1, 'read_timeout_s': 3.0,
}


def load_start_limits(path):
    config = configparser.ConfigParser()
    with Path(path).open(encoding='utf-8') as source:
        config.read_file(source)
    if config.defaults() or config.sections() != ['start_check'] or set(config['start_check']) != set(START_LIMITS):
        raise ValueError('start-check config must contain exactly the documented [start_check] keys')
    values = {key: float(config['start_check'][key]) for key in START_LIMITS}
    if any(not math.isfinite(value) or value <= 0 for value in values.values()):
        raise ValueError('start-check limits must be finite and positive')
    if not 0.001 <= values["read_timeout_s"] <= 60:
        raise ValueError("read_timeout_s must be between 0.001 and 60")
    return values


def first_replay_commands(data, mode):
    first = {}
    for frame in data.get('frames', []):
        for name, value in frame.get('observation', {}).items():
            if replay_stream_applies(name, mode):
                first.setdefault(name, value)
    return first


def needs_manipulator(first):
    return any(name in first for name in (
        'left_arm_joint_states', 'right_arm_joint_states', 'waist_joint_states',
        'left_arm_end_pose', 'right_arm_end_pose', 'waist_end_pose'))


def _checked_vector(values):
    values = [float(value) for value in values]
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError('expected nonempty finite values')
    return values


def _start_pose(value):
    if isinstance(value, dict):
        position, orientation = value['position'], value['orientation']
        return (_checked_vector([position[k] for k in ('x', 'y', 'z')]),
                _checked_vector([orientation[k] for k in ('x', 'y', 'z', 'w')]))
    if hasattr(value, 'qx'):  # Native arm PoseStamped binding.
        return (_checked_vector([value.x, value.y, value.z]),
                _checked_vector([value.qx, value.qy, value.qz, value.qw]))
    pose = getattr(value, 'pose', value)
    return (_checked_vector([pose.position.x, pose.position.y, pose.position.z]),
            _checked_vector([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))


def _rotation_error(left, right):
    a, b = math.hypot(*left), math.hypot(*right)
    if not math.isfinite(a+b) or abs(a-1) > .01 or abs(b-1) > .01:
        raise ValueError("orientation quaternion must be finite and approximately unit length")
    return 2 * math.acos(min(1.0, abs(sum((x/a)*(y/b) for x, y in zip(left, right)))))


def check_start_state(robot, first, limits=None):
    """Read only recorded components. Any failed read/check blocks all writes."""
    limits = dict(START_LIMITS if limits is None else limits)
    errors, checks = [], []
    timeout = limits['read_timeout_s']

    def compare(name, metric, error, limit):
        passed = math.isfinite(error) and error <= limit + max(1e-9, limit*1e-6)
        row = dict(component=name, metric=metric, error=error, limit=limit, passed=passed)
        checks.append(row)
        print('Start check: ' + json.dumps(row), flush=True)
        if not passed:
            errors.append(f'{name} {metric}: {error:.6g} > {limit:.6g}')

    def vector(name, current, target, limit):
        current, target = _checked_vector(current), _checked_vector(target)
        if len(current) != len(target):
            raise ValueError(f'position count mismatch: current={len(current)}, recorded={len(target)}')
        compare(name, 'max_position_error', max(abs(x-y) for x, y in zip(current, target)), limit)

    def pose(name, current, target):
        cp, cq = _start_pose(current)
        tp, tq = _start_pose(target)
        compare(name, 'translation_m', math.dist(cp, tp), limits['position_m'])
        compare(name, 'rotation_rad', _rotation_error(cq, tq), limits['orientation_rad'])

    for name, target in first.items():
        try:
            if name in ('left_arm_joint_states', 'right_arm_joint_states', 'waist_joint_states'):
                controller = robot.waist if name.startswith('waist') else _arm(robot, name.split('_')[0])
                args = {'timeout': timeout} if name.startswith('waist') else {'deadline_ms': int(timeout*1000)}
                state = require(controller.get_joint_states(**args), name + '.GetJointStates')
                names, recorded_names = list(state.name), target.get('names', [])
                for labels, values in ((names, state.position), (recorded_names, target["positions"])):
                    if labels and (len(labels) != len(values) or len(set(labels)) != len(labels)):
                        raise ValueError("joint names must be unique and match the position count")
                if names and recorded_names and names != recorded_names:
                    raise ValueError('joint names/order differ from the SDK command order')
                vector(name, state.position, target['positions'], limits['joint_rad'])
            elif name in ('left_arm_end_pose', 'right_arm_end_pose', 'waist_end_pose'):
                controller = robot.waist if name.startswith('waist') else _arm(robot, name.split('_')[0])
                args = {'timeout': timeout} if name.startswith('waist') else {'deadline_ms': int(timeout*1000)}
                pose(name, require(controller.get_end_pose(**args), name + '.GetEndPose'), target)
            elif name == 'head_joint_states':
                state = require(robot.head.get_pose(timeout=timeout), 'Head.GetPose')
                vector(name, [state.pitch, state.yaw], target['positions'], limits['joint_rad'])
            elif name == 'lift_joint_states':
                state = require(robot.lift.get_lift_position(timeout=timeout), 'Lift.GetLiftPosition')
                vector(name, [state.position], target['positions'], limits['lift_m'])
            elif name in ('left_gripper_joint_states', 'right_gripper_joint_states',
                          'left_gripper_position', 'right_gripper_position'):
                state = require(getattr(robot, name.split('_')[0]+'_gripper').get_position(timeout=timeout), name+'.GetPosition')
                positions = target['positions'] if name.endswith('joint_states') else [target['position']]
                values = _checked_vector(positions)
                vector(name, [state.position], [max(0.0, values[0])], limits['gripper'])
            elif name == 'odometry':
                state = require(robot.chassis.get_odometry(timeout=timeout), 'Chassis.GetOdometry')
                cp, cq = _start_pose(state.pose.pose)
                tp, tq = _start_pose(target['pose'])
                # Validate both quaternions before computing the planar heading.
                _rotation_error(cq, tq)
                yaw = lambda q: math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]))
                compare(name, 'translation_m', math.dist(cp[:2], tp[:2]), limits['chassis_position_m'])
                compare(name, 'yaw_rad', abs(math.remainder(yaw(cq)-yaw(tq), 2*math.pi)), limits['chassis_yaw_rad'])
        except Exception as error:
            errors.append(f'{name}: {error}')
            print(f'Start check BLOCKED: {name}: {error}', flush=True)
    if errors:
        raise RuntimeError('Replay start check failed; no motion commands sent: ' + '; '.join(errors))
    print(f'Start check passed: {len(first)} recorded streams; unrecorded streams skipped', flush=True)
    return checks


def control_head(robot, observation, warnings=None):
    check = warnings.check if warnings is not None else require
    state = observation.get("head_joint_states")
    values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
    if values:
        check(robot.head.set_pose(HeadPose(pitch=values[0] if len(values) > 1 else 0.0, yaw=values[1] if len(values) > 1 else values[0]), timeout=1.0), "Head.SetPose")


def control_aux(robot, observation, model, end_pose=False, warnings=None):
    check = warnings.check if warnings is not None else require
    if model == "quanta_x1":
        state = observation.get("lift_joint_states")
        values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
        if values:
            check(robot.lift.set_lift_position(LiftPosition(position=values[0]), timeout=1.0), "Lift.SetLiftPosition")
    elif model in ("quanta_x2", "quanta_x3"):
        state = observation.get("waist_joint_states")
        values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
        if values and not end_pose:
            check(robot.waist.set_joint_positions(JointPositions(positions=values), timeout=1.0), "Waist.SetJointPositions")
        if end_pose:
            pose = observation.get("waist_end_pose")
            if isinstance(pose, dict) and pose.get("position") and pose.get("orientation"):
                check(robot.waist.set_end_pose(_pose_from_dict(pose), timeout=1.0), "Waist.SetEndPose")


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


def replay_by_joint_positions(robot, data, speed=1.0, model=None, start_limits=None):
    model = model or model_name(robot)
    if data.get("model") and data["model"] != model:
        raise ValueError("recorded model differs from the connected robot")
    data, joint_stats = prepare_joint_commands(data, model, "joint_pos")
    print("Joint clamp plan: " + json.dumps(joint_stats), flush=True)
    first = first_replay_commands(data, 'joint_pos')
    if not first:
        print("Nothing to replay for the selected mode; no commands sent")
        return 0
    check_start_state(robot, first, start_limits)
    warnings = ReplayCommandWarnings()
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    if needs_manipulator(first):
        require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS), timeout=3.0), "RobotControl.SetManipulatorControlMode")
    chassis = None
    if model != "desktop" and has_odometry(data):
        require(robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY), timeout=3.0), "Chassis.SetControlMode")
        chassis = ChassisController(robot, warnings=warnings)
        chassis.start()
    frames = data.get("frames", [])
    started = time.monotonic()
    first_stamp = float(frames[0].get('timestamp', 0)) if frames else 0
    try:
        for index, frame in enumerate(frames):
            wait_until_frame(frame, first_stamp, started, speed)
            observation = frame.get("observation", {})
            _send_chassis_target(chassis, frame)
            for side in ("left", "right"):
                state = observation.get(f"{side}_arm_joint_states")
                values = finite((state or {}).get("positions", [])) if isinstance(state, dict) else []
                if values:
                    warnings.check(_arm(robot, side).set_joint_positions(values, deadline_ms=1000), f"{side.title()}Arm.SetJointPositions")
            control_aux(robot, observation, model, warnings=warnings)
            control_grippers(robot, observation, warnings)
            control_head(robot, observation, warnings)
            if index % 10 == 0:
                print(f"Progress: {index + 1}/{len(frames)}", end="\r")
    finally:
        if chassis is not None:
            chassis.stop()
            chassis.join(timeout=1.0)
        rejected = warnings.summary()
    print(f"\nReplay completed in {time.monotonic() - started:.2f}s ({len(frames)} frames)")
    return rejected


def replay_by_end_pose(robot, data, speed=1.0, model=None, start_limits=None):
    model = model or model_name(robot)
    if data.get("model") and data["model"] != model:
        raise ValueError("recorded model differs from the connected robot")
    data, joint_stats = prepare_joint_commands(data, model, "end_pose")
    print("Joint clamp plan: " + json.dumps(joint_stats), flush=True)
    first = first_replay_commands(data, 'end_pose')
    if not first:
        print("Nothing to replay for the selected mode; no commands sent")
        return 0
    check_start_state(robot, first, start_limits)
    warnings = ReplayCommandWarnings()
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    if needs_manipulator(first):
        require(robot.robot_control.set_manipulator_control_mode(ManipulatorControlModeParam(mode=ManipulatorControlMode.MANIPULATOR_END_POSE), timeout=3.0), "RobotControl.SetManipulatorControlMode")
    chassis = None
    if model != "desktop" and has_odometry(data):
        require(robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY), timeout=3.0), "Chassis.SetControlMode")
        chassis = ChassisController(robot, warnings=warnings)
        chassis.start()
    frames = data.get("frames", [])
    started = time.monotonic()
    first_stamp = float(frames[0].get('timestamp', 0)) if frames else 0
    try:
        for index, frame in enumerate(frames):
            wait_until_frame(frame, first_stamp, started, speed)
            observation = frame.get("observation", {})
            _send_chassis_target(chassis, frame)
            control_aux(robot, observation, model, end_pose=True, warnings=warnings)
            for side in ("left", "right"):
                value = observation.get(f"{side}_arm_end_pose")
                if isinstance(value, dict) and value.get("position") and value.get("orientation"):
                    warnings.check(_arm(robot, side).set_end_pose(_pose_from_dict(value), deadline_ms=1000), f"{side.title()}Arm.SetEndPose")
            control_grippers(robot, observation, warnings)
            control_head(robot, observation, warnings)
            if index % 10 == 0:
                print(f"Progress: {index + 1}/{len(frames)}", end="\r")
    finally:
        if chassis is not None:
            chassis.stop()
            chassis.join(timeout=1.0)
        rejected = warnings.summary()
    print(f"\nReplay completed ({len(frames)} frames)")
    return rejected


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
    parser.add_argument("episode_dir")
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--no-confirm", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="print per-stream replay counts without connecting to a robot")
    parser.add_argument("--start-check-config", type=Path, default=Path(__file__).with_name("replay.conf"))
    parser.add_argument("--check-only", action="store_true", help="read current state and check the recorded start without sending commands")
    args = parser.parse_args(argv)
    start_limits = load_start_limits(args.start_check_config)
    if args.dry_run and args.check_only:
        parser.error("--dry-run and --check-only are mutually exclusive")
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and > 0")
    data = load_episode_data(args.episode_dir)
    if args.dry_run:
        data, joint_stats = prepare_joint_commands(data, str(data.get('model', '')), args.mode)
        print(json.dumps({'mode': args.mode, 'streams': replay_plan(data, args.mode),
                          'joint_clamp': joint_stats}, ensure_ascii=False, indent=2))
        return 0
    if not first_replay_commands(data, args.mode):
        print("Nothing to replay for the selected mode; no connection or commands needed")
        return 0
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        model = model_name(robot)
        recorded = str(data.get("model", model))
        print(f"Replay configuration: mode={args.mode}, speed={args.speed}x, model={model}, recorded={recorded}")
        if recorded != model:
            raise ValueError("recorded model differs from the connected robot")
        if args.check_only:
            prepared, _ = prepare_joint_commands(data, model, args.mode)
            check_start_state(robot, first_replay_commands(prepared, args.mode), start_limits)
            print("PASS: start check only; no motion commands sent")
            return 0
        if not args.no_confirm:
            input("Press Enter to start replay (Ctrl+C to cancel)...")
        if args.mode == "joint_pos":
            rejected = replay_by_joint_positions(robot, data, args.speed, model, start_limits)
        else:
            rejected = replay_by_end_pose(robot, data, args.speed, model, start_limits)
        print(f"WARN: replay completed with {rejected} rejected commands" if rejected else "PASS: data replay")
        return 0
    except KeyboardInterrupt:
        print("\nReplay interrupted")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
