"""Quanta X3 waist: bounded, smooth relative joint/Cartesian motion.

Default joint verification is read-only. Cartesian verify/stream enters SDK
end-pose mode to enable feedback, then restores IDLE on exit.
Cartesian motion translates or rotates one
axis in the feedback frame and refreshes both arm targets. WBC may trade off pose accuracy;
Cartesian completion means the command sequence finished, not precise arrival.
This example does not provide IK/collision checking.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import x2robot_sdk
from x2robot.geometry_msgs import Point, Pose as RosPose, Quaternion
from x2robot.sdk import (
    JointPositions, ManipulatorControlMode, ManipulatorControlModeParam,
    RobotModeParam, RobotWorkMode,
)

CONFIG = json.loads((Path(__file__).resolve().parents[1] / "config/quanta_x3/waist.json").read_text())
INTERVAL = 0.02  # At most 50 Hz; never catch up with a burst after a delayed RPC.
MAX_GAP = 0.4  # Whole-cycle budget, shared by feedback and all target writes.
WRITE_TIMEOUT = 0.3
FEEDBACK_TIMEOUT = 0.3
FEEDBACK_START_WAIT = 2.0  # Bounded retry window after entering Cartesian mode.
_sdk_mode_entered = False


def require(result, operation):
    if not result.is_success:
        raise RuntimeError(f"{operation} failed: {result.error}")
    return result.data


def finite(values, size):
    values = list(values)
    if len(values) != size or not all(math.isfinite(v) for v in values):
        raise ValueError(f"expected {size} finite values")
    return values


def fresh(message):
    stamp = message.header.stamp
    timestamp = stamp.sec + stamp.nanosec * 1e-9
    age = time.time() - timestamp
    if timestamp <= 0 or not -0.5 <= age <= 1.0:
        raise RuntimeError("feedback missing/stale or robot clock unsynchronized")


def validate_joints(values, margin=False):
    values = finite(values, 3)
    for i, value in enumerate(values):
        pad = CONFIG["joint_margin"][i] if margin else 0.0
        lower, upper = CONFIG["joint_lower"][i] + pad, CONFIG["joint_upper"][i] - pad
        if not lower <= value <= upper:
            raise ValueError(f"{CONFIG['joint_names'][i]}={value} outside [{lower}, {upper}]")
    return values


def read_joints(robot, margin=False):
    state = require(robot.waist.get_joint_states(timeout=FEEDBACK_TIMEOUT), "Waist.GetJointStates")
    fresh(state)
    names = list(state.name)
    if len(names) != 3 or set(names) != set(CONFIG["joint_names"]):
        raise RuntimeError(f"unexpected waist joint names: {names}")
    positions = finite(state.position, 3)
    return validate_joints([positions[names.index(n)] for n in CONFIG["joint_names"]], margin)


def validate_pose(pose):
    p = finite([pose.position.x, pose.position.y, pose.position.z], 3)
    q = finite([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], 4)
    if not 0.999 <= math.sqrt(sum(v * v for v in q)) <= 1.001:
        raise ValueError("feedback orientation is not a unit quaternion")
    if any(not lo <= v <= hi for v, lo, hi in zip(p, CONFIG["pose_lower"], CONFIG["pose_upper"])):
        raise ValueError("waist pose outside server command bounds")
    return p


def read_pose(robot, wait_for_feedback=False):
    deadline = time.monotonic() + FEEDBACK_START_WAIT
    while True:
        result = robot.waist.get_end_pose(timeout=FEEDBACK_TIMEOUT)
        if result.is_success:
            break
        error = result.error or {}
        if (not wait_for_feedback or not isinstance(error, dict)
                or error.get("error_name") not in ("UNAVAILABLE", "TIMEOUT")
                or time.monotonic() >= deadline):
            require(result, "Waist.GetEndPose")
        time.sleep(0.05)
    stamped = result.data
    fresh(stamped)
    validate_pose(stamped.pose)
    if not stamped.header.frame_id:
        raise RuntimeError("waist feedback has no reference frame")
    return stamped


def pose_at(source, position):
    q = source.orientation
    return RosPose(position=Point(x=position[0], y=position[1], z=position[2]),
                   orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w))


def pose_rotated(source, axis, angle):
    """Rotate around a fixed feedback-frame Y (pitch) or Z (yaw) axis.

    Left-multiply the initial quaternion; never accumulate incremental rotations.
    """
    if axis not in ("pitch", "yaw") or not math.isfinite(angle):
        raise ValueError("rotation requires pitch or yaw and a finite angle in radians")
    q = source.orientation
    s, c = math.sin(angle / 2), math.cos(angle / 2)
    y, z = (s, 0.0) if axis == "pitch" else (0.0, s)
    return RosPose(position=Point(x=source.position.x, y=source.position.y, z=source.position.z),
                   orientation=Quaternion(x=c * q.x + y * q.z - z * q.y,
                                          y=c * q.y + y * q.w + z * q.x,
                                          z=c * q.z - y * q.x + z * q.w,
                                          w=c * q.w - y * q.y - z * q.z))


def arm_hold(controller):
    source = require(controller.get_end_pose(deadline_ms=round(FEEDBACK_TIMEOUT * 1000)), "Arm.GetEndPose")
    values = finite([source.x, source.y, source.z, source.qx, source.qy, source.qz, source.qw], 7)
    if not 0.999 <= math.sqrt(sum(v * v for v in values[3:])) <= 1.001:
        raise ValueError("invalid arm hold quaternion")
    return x2robot_sdk.Pose(**dict(zip(("x", "y", "z", "qx", "qy", "qz", "qw"), values)))


def plan(start, delta, velocity, acceleration, jerk, cartesian=False):
    """Quintic time scaling: zero velocity/acceleration at both ends.

    Maxima of |s'|, |s''|, |s'''| are 1.875, 10/sqrt(3), 60.
    Stretch duration to satisfy each axis, then sample the entire path first.
    """
    start, delta = finite(start, 3), finite(delta, 3)
    duration = 2.0
    distances = [math.sqrt(sum(d * d for d in delta))] * 3 if cartesian else delta
    for d, v, a, j in zip(distances, velocity, acceleration, jerk):
        duration = max(duration, 1.875 * abs(d) / v,
                       math.sqrt(10 / math.sqrt(3) * abs(d) / a),
                       (60 * abs(d) / j) ** (1 / 3))
    steps = math.ceil(duration / INTERVAL)
    points = []
    for i in range(steps + 1):
        u = i / steps
        s = u ** 3 * (10 + u * (-15 + 6 * u))
        points.append([p + d * s for p, d in zip(start, delta)])
    return points


def check_tracking(actual, command, tolerance):
    if any(abs(a - b) > t for a, b, t in zip(actual, command, tolerance)):
        raise RuntimeError("feedback deviates from trajectory; stopping commands")


class ControlCycle:
    """Include RPC work in the period; never catch up after an overrun."""

    def __init__(self):
        self.started = None
        self.last_dispatch = None

    def begin(self):
        now = time.monotonic()
        if self.started is not None and now - self.started >= MAX_GAP:
            raise RuntimeError("control cycle stalled between samples; stopping commands")
        self.started = now

    def check(self):
        remaining = MAX_GAP - (time.monotonic() - self.started)
        if remaining < 0.001:
            raise RuntimeError("control cycle stalled; stopping commands")
        return remaining

    def write_timeout(self):
        return min(WRITE_TIMEOUT, self.check())

    def before_send(self):
        # Variable feedback latency must not bunch two adjacent targets.
        if self.last_dispatch is not None:
            remaining = INTERVAL - (time.monotonic() - self.last_dispatch)
            if remaining > 0:
                time.sleep(remaining)
        self.check()
        self.last_dispatch = time.monotonic()

    def wait(self):
        self.check()
        remaining = INTERVAL - (time.monotonic() - self.started)
        if remaining > 0:
            time.sleep(remaining)


def validate_speeds(linear_speed=None, angular_speed=None):
    for name, value in (("linear", linear_speed), ("angular", angular_speed)):
        lower, upper = CONFIG[name + "_speed_range"]
        if value is not None and (not math.isfinite(value) or not lower <= value <= upper):
            raise ValueError(f"{name} speed must be finite and within [{lower}, {upper}]")


def set_mode(robot, mode):
    global _sdk_mode_entered
    # Mark before RPC: even an ambiguous SDK-entry timeout must attempt IDLE.
    _sdk_mode_entered = True
    require(robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK), timeout=5.0),
            "System.SetWorkMode(SDK)")
    control = (ManipulatorControlMode.MANIPULATOR_JOINT_POSITIONS if mode == "joint_pos"
               else ManipulatorControlMode.MANIPULATOR_END_POSE)
    require(robot.robot_control.set_manipulator_control_mode(
        ManipulatorControlModeParam(mode=control), timeout=5.0), "SetManipulatorControlMode")


def restore_idle_mode(robot):
    global _sdk_mode_entered
    if not _sdk_mode_entered:
        return
    require(robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.IDLE), timeout=5.0),
            "System.SetWorkMode(IDLE)")
    _sdk_mode_entered = False


def validate_delta(mode, delta, rotation=None):
    delta = finite(delta, 3)
    if rotation is not None:
        axis, angle = rotation
        if mode != "end_pose" or any(delta):
            raise ValueError("rotation requires end_pose mode; do not mix translation or joint targets")
        if axis not in ("pitch", "yaw") or not math.isfinite(angle) or not 0 < abs(angle) <= CONFIG["pose_max_angle"]:
            raise ValueError("pose rotation requires pitch or yaw and 0 < abs(angle) <= 0.2 rad")
        return
    if mode not in ("joint_pos", "end_pose") or not any(delta):
        raise ValueError("select a motion mode and nonzero relative displacement")
    if mode == "joint_pos":
        if any(abs(v) > limit for v, limit in zip(delta, CONFIG["joint_max_delta"])):
            raise ValueError("joint displacement exceeds demonstration limits")
    else:
        limit = CONFIG["pose_z_max_delta"] if delta[0] == 0 and delta[1] == 0 else CONFIG["pose_max_delta"]
        if math.sqrt(sum(v * v for v in delta)) > limit:
            raise ValueError("Cartesian displacement exceeds limits: Z-only <= 0.05 m; otherwise <= 0.01 m")


def move(robot, mode, delta, rotation=None, linear_speed=None, angular_speed=None):
    delta = finite(delta, 3)
    validate_delta(mode, delta, rotation)
    validate_speeds(linear_speed, angular_speed)
    joint_velocity = CONFIG["joint_velocity"][:]
    pose_velocity = CONFIG["pose_velocity"] if linear_speed is None else linear_speed
    pose_angular_velocity = CONFIG["pose_angular_velocity"] if angular_speed is None else angular_speed
    if linear_speed is not None:
        joint_velocity[0] = linear_speed
    if angular_speed is not None:
        joint_velocity[1:] = [angular_speed, angular_speed]
    cycle = ControlCycle()
    pose_joint_excursion = CONFIG["pose_joint_max_excursion"][:]
    if rotation is not None:
        index = 1 if rotation[0] == "pitch" else 2
        pose_joint_excursion[index] = max(pose_joint_excursion[index],
                                         abs(rotation[1]) + CONFIG["joint_tracking_error"][index])
    # Refuse to take over a moving waist; reject out-of-range feedback, never clamp it.
    start = read_joints(robot, margin=(mode == "end_pose"))
    time.sleep(0.1)
    check_tracking(read_joints(robot), start, [0.0005, 0.001, 0.001])
    if mode == "joint_pos":
        validate_joints([p + d for p, d in zip(start, delta)], margin=True)
    set_mode(robot, mode)
    initial_joints = start[:]
    observed_joints = read_joints(robot, margin=(mode == "end_pose"))
    if mode == "joint_pos":
        check_tracking(observed_joints, start, [0.0005, 0.001, 0.001])
    elif any(abs(a - b) > t for a, b, t in zip(
            observed_joints, initial_joints, pose_joint_excursion)):
        raise RuntimeError("waist joint excursion exceeds Cartesian demonstration limits")
    after_mode = observed_joints
    start = after_mode
    if mode == "joint_pos":
        validate_joints([p + d for p, d in zip(start, delta)], margin=True)
        points = plan(start, delta, joint_velocity, CONFIG["joint_acceleration"], CONFIG["joint_jerk"])
        for point in points:
            validate_joints(point)
        def send(point):
            validate_joints(point)
            require(robot.waist.set_joint_positions(JointPositions(positions=point), timeout=cycle.write_timeout()),
                    "Waist.SetJointPositions")
            cycle.check()
        def monitor(previous):
            actual = read_joints(robot)
            check_tracking(actual, previous, CONFIG["joint_tracking_error"])
            return actual
        tolerance = [0.002, 0.005, 0.005]
    else:
        source_sample = read_pose(robot, wait_for_feedback=True)
        source = source_sample.pose
        start = validate_pose(source)
        if rotation is None:
            points = plan(start, delta, [pose_velocity] * 3,
                          [CONFIG["pose_acceleration"]] * 3, [CONFIG["pose_jerk"]] * 3, cartesian=True)
            def pose_for_point(point):
                return pose_at(source, point)
        else:
            axis, angle = rotation
            index = 1 if axis == "pitch" else 2
            angles = [0.0, 0.0, 0.0]
            angles[index] = angle
            points = plan([0.0] * 3, angles, [pose_angular_velocity] * 3,
                          [CONFIG["pose_angular_acceleration"]] * 3,
                          [CONFIG["pose_angular_jerk"]] * 3, cartesian=True)
            def pose_for_point(point):
                return pose_rotated(source, axis, point[index])
        for point in points:
            validate_pose(pose_for_point(point))
        left = arm_hold(robot.left_arm_controller)
        right = arm_hold(robot.right_arm_controller)
        def send(point):
            pose = pose_for_point(point)
            validate_pose(pose)
            # All three WBC reasoning targets must be refreshed every cycle.
            require(robot.left_arm_controller.set_end_pose(left, deadline_ms=int(cycle.write_timeout() * 1000)), "LeftArm.SetEndPose")
            require(robot.right_arm_controller.set_end_pose(right, deadline_ms=int(cycle.write_timeout() * 1000)), "RightArm.SetEndPose")
            require(robot.waist.set_end_pose(pose, timeout=cycle.write_timeout()), "Waist.SetEndPose")
            cycle.check()
        def monitor(previous):
            nonlocal observed_joints
            observed_joints = read_joints(robot, margin=True)
            if any(abs(a - b) > t for a, b, t in zip(
                    observed_joints, initial_joints, pose_joint_excursion)):
                raise RuntimeError("waist joint excursion exceeds Cartesian demonstration limits")
            sample = read_pose(robot)
            if sample.header.frame_id != source_sample.header.frame_id:
                raise RuntimeError("waist reference frame changed during motion")
            # WBC optimizes several tasks together. Pose error is diagnostic,
            # while measured joint limits/excursion still bound this demo.
            return validate_pose(sample.pose)
    print(f"Planned {len(points)} samples, at least {(len(points) - 1) * INTERVAL:.2f}s; delta={delta}; rotation={rotation}")
    print(f"Velocity limits: joints [m/s, rad/s, rad/s]={joint_velocity}; pose={pose_velocity} m/s, {pose_angular_velocity} rad/s")
    previous, trajectory_started = points[0], time.monotonic()
    for point in points:
        cycle.begin()
        monitor(previous)
        cycle.before_send()
        send(point)
        previous = point
        cycle.wait()
    print(f"Trajectory samples completed in {time.monotonic() - trajectory_started:.2f}s")
    if mode == "end_pose":
        # Keep all three targets alive for a bounded interval, without chasing
        # residual error or waiting for an accuracy WBC does not guarantee.
        deadline = time.monotonic() + 1.0
        while True:
            cycle.begin()
            actual = monitor(previous)
            cycle.check()
            if time.monotonic() >= deadline:
                break
            cycle.before_send()
            send(previous)
            cycle.wait()
        displacement = [a - b for a, b in zip(observed_joints, initial_joints)]
        target_pose = pose_for_point(previous)
        target_position = validate_pose(target_pose)
        print(f"Cartesian command sequence completed; target xyz(m)={target_position}; feedback xyz(m)={actual}")
        if rotation is not None:
            q = target_pose.orientation
            print(f"Target quaternion xyzw={[q.x, q.y, q.z, q.w]}; rotation={rotation}")
        print(f"Measured waist joint delta [m, rad, rad]: {displacement}")
        print("NOTE: WBC balances multiple tasks; completion does not imply precise target arrival.")
        if all(abs(v) <= t for v, t in zip(displacement, [0.0005, 0.001, 0.001])):
            print("WARNING: no net waist joint movement observed; command acceptance alone does not confirm motion.")
        return
    # Joint-position mode retains its feedback convergence requirement.
    deadline = time.monotonic() + 2.0
    while True:
        cycle.begin()
        actual = monitor(previous)
        cycle.check()
        if all(abs(a - b) <= t for a, b, t in zip(actual, previous, tolerance)):
            return
        if time.monotonic() >= deadline:
            raise RuntimeError("waist did not reach the trajectory endpoint")
        cycle.before_send()
        send(previous)
        cycle.wait()


def stream_waist(robot, mode, seconds):
    method = robot.waist.get_end_pose_stream if mode == "end_pose" else robot.waist.get_joint_states_stream
    stream = method(timeout=None if seconds == 0 else seconds + 1)
    end = time.monotonic() + seconds if seconds else math.inf
    try:
        for result in stream:
            print(require(result, "Waist stream"))
            if time.monotonic() >= end:
                break
    finally:
        close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
        if close:
            close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--action", choices=("verify", "move", "stream"), default="verify")
    parser.add_argument("--mode", choices=("joint_pos", "end_pose"), default="joint_pos")
    parser.add_argument("--joint-delta", type=float, nargs=3, metavar=("LIFT_M", "PITCH_RAD", "YAW_RAD"))
    parser.add_argument("--pose-delta", type=float, nargs=3, metavar=("X_M", "Y_M", "Z_M"))
    parser.add_argument("--pose-rotation", nargs=2, metavar=("pitch|yaw", "ANGLE_RAD"),
                        help="rotate one feedback-frame axis, keeping the initial XYZ target")
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--linear-speed", type=float, default=None,
                        help="linear velocity limit in m/s [0.001, 0.04]; default joint=0.02, pose=0.01")
    parser.add_argument("--angular-speed", type=float, default=None,
                        help="pitch/yaw velocity limit in rad/s [0.005, 0.12]; default=0.06")
    parser.add_argument("--yes", action="store_true", help="confirm motion or Cartesian mode entry after checking the scene")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and non-negative")
    try:
        validate_speeds(args.linear_speed, args.angular_speed)
        if args.action != "move" and (args.linear_speed is not None or args.angular_speed is not None):
            raise ValueError("speed options require --action move")
    except ValueError as error:
        parser.error(str(error))
    delta, rotation = None, None
    if args.action == "move":
        if args.pose_rotation is not None:
            try:
                rotation = (args.pose_rotation[0], float(args.pose_rotation[1]))
            except ValueError:
                parser.error("rotation angle must be a finite number in radians")
        delta = args.joint_delta if args.mode == "joint_pos" else args.pose_delta
        if sum(value is not None for value in (args.joint_delta, args.pose_delta, args.pose_rotation)) != 1:
            parser.error("move requires exactly one of --joint-delta, --pose-delta or --pose-rotation")
        if delta is None and rotation is None:
            parser.error("delta must match the selected mode")
        delta = [0.0] * 3 if delta is None else delta
        try:
            validate_delta(args.mode, delta, rotation)
        except ValueError as error:
            parser.error(str(error))
    if (args.action == "move" or args.mode == "end_pose") and not args.yes:
        try:
            operation = ("smooth waist motion" if args.action == "move" else
                         "SDK end-pose feedback (mode entry may move the robot)")
            input(f"Check clear space and emergency stop. Press Enter for {operation}, Ctrl+C to cancel: ")
        except (EOFError, KeyboardInterrupt):
            return 1
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    status = 0
    try:
        if robot.model_name != "quanta_x3":
            raise RuntimeError(f"expected Quanta X3, connected to {robot.model_name}")
        if args.action == "move":
            move(robot, args.mode, delta, rotation, args.linear_speed, args.angular_speed)
        elif args.action == "stream":
            if args.mode == "end_pose":
                read_joints(robot)
                set_mode(robot, args.mode)
                read_pose(robot, wait_for_feedback=True)
            stream_waist(robot, args.mode, args.seconds)
        else:
            print("joint_pos:", read_joints(robot))
            if args.mode == "end_pose":
                set_mode(robot, args.mode)
                print("end_pose:", read_pose(robot, wait_for_feedback=True))
    except (Exception, KeyboardInterrupt) as error:
        print(f"FAIL: {str(error) or 'interrupted'}")
        status = 1
    finally:
        try:
            restore_idle_mode(robot)
        except Exception as error:
            print(f"FAIL: could not restore IDLE: {error}")
            status = 1
        finally:
            try:
                require(robot.close(), "Robot.close")
            except Exception as error:
                print(f"FAIL: close: {error}")
                status = 1
    if status == 0:
        print("PASS: Quanta X3 waist control")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
