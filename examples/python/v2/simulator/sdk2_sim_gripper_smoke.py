#!/usr/bin/env python3
"""Guarded SDK2 left/right gripper smoke test for the EX001 ManaEnv simulator.

The default mode is read-only.  A mutating mode requires one explicit action,
``--allow-write``, ``--confirm-simulator-only``, and that action's exact
acknowledgement.  The authoritative feedback is the complete, timestamped
selected gripper's ``GetJointStatesStream`` result.  ``GetPosition`` is used
only as an independent cross-check.

Before a mutating run, externally prove the active SDK release uses
``runtime_profile=simulator`` and prove exact owner/type/QoS relationships for
the selected side's command and state topics.  The script does not inspect the
ROS graph, publish raw ROS messages, or distinguish a simulator from a real
robot on its own.  Left and right are independent tests.  Do not run a mutating
mode while any previous control attempt is classified ``RESTORE_UNCONFIRMED``.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import x2robot_sdk
from x2robot.sdk import GripperPosition, RobotModeParam, RobotWorkMode
from x2robot.sdk.v2 import RobotModelType


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}

GRIPPER_LOWER_RAD = 0.0
GRIPPER_UPPER_RAD = 1.53
DEFAULT_DELTA_RAD_TEXT = "0.03"
REVIEWED_DELTA_RAD_BY_TEXT = {
    "0.03": 0.03,
    "0.10": 0.10,
    "0.20": 0.20,
    "0.50": 0.50,
    "1.00": 1.00,
}
EXCURSION_MARGIN_RAD = 0.05
HOLD_STREAM_DEADLINE_GRACE_SECONDS = 0.25

BASELINE_SAMPLE_COUNT = 5
CONFIRMATION_SAMPLE_COUNT = 3
BASELINE_STABILITY_TOLERANCE_RAD = 0.003
POSITION_TOLERANCE_RAD = 0.006
GET_CROSSCHECK_TOLERANCE_RAD = 0.006
SAME_VALUE_MAX_EXCURSION_RAD = 0.08
FEEDBACK_LIMIT_TOLERANCE_RAD = 0.003

SAME_VALUE_ACKNOWLEDGEMENT = "SIMULATOR_LEFT_GRIPPER_SAME_VALUE"
RIGHT_SAME_VALUE_ACKNOWLEDGEMENT = "SIMULATOR_RIGHT_GRIPPER_SAME_VALUE"


@dataclass(frozen=True)
class GripperSide:
    side: str
    controller_attribute: str
    controller_name: str
    joint_name: str
    command_topic: str
    state_topic: str
    same_value_acknowledgement: str


GRIPPER_SIDES = {
    "left": GripperSide(
        side="left",
        controller_attribute="left_gripper",
        controller_name="LeftGripperController",
        joint_name="left_arm_gripper",
        command_topic="/left_gripper_controller/commands",
        state_topic="/left_gripper/joint_states",
        same_value_acknowledgement=SAME_VALUE_ACKNOWLEDGEMENT,
    ),
    "right": GripperSide(
        side="right",
        controller_attribute="right_gripper",
        controller_name="RightGripperController",
        joint_name="right_arm_gripper",
        command_topic="/right_gripper_controller/commands",
        state_topic="/right_gripper/joint_states",
        same_value_acknowledgement=RIGHT_SAME_VALUE_ACKNOWLEDGEMENT,
    ),
}


@dataclass(frozen=True)
class GripperJointSample:
    position_rad: float
    stamp_ns: int
    names: tuple[str, ...]


@dataclass(frozen=True)
class StableWindow:
    samples: tuple[GripperJointSample, ...]

    @property
    def latest(self) -> GripperJointSample:
        return self.samples[-1]


@dataclass
class SafetyState:
    actuator_write_attempted: bool = False
    restoration_confirmed: bool = False


class RestoreUnconfirmedError(RuntimeError):
    """A failure occurred after at least one gripper Set RPC began."""


class TargetFailedRestoredError(RuntimeError):
    """A bounded write failed, but exact baseline restoration was confirmed."""


class WriteNotRunError(RuntimeError):
    """A pre-write gate rejected the requested action before mode or Set."""


def require(result, operation: str):
    if not isinstance(result, x2robot_sdk.SdkResult):
        raise TypeError(
            f"{operation}: expected SdkResult, got {type(result).__name__}"
        )
    if not result.is_success:
        error = result.error or {}
        name = error.get("error_name", "UNKNOWN")
        message = error.get("error_message", "SDK call failed")
        raise RuntimeError(f"{operation}: {name}: {message}")
    return result.data


def close_stream(stream) -> None:
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close is not None:
        close()


def selected_gripper(robot, side: GripperSide):
    return getattr(robot, side.controller_attribute)


def parse_gripper_joint_sample(
    value, side: GripperSide
) -> GripperJointSample:
    names = tuple(str(name) for name in value.name)
    positions = tuple(float(position) for position in value.position)
    if not names or len(names) != len(positions):
        raise RuntimeError(
            f"{side.side}-gripper JointState must contain equally sized, non-empty "
            f"name and position arrays; names={names!r}, positions={positions!r}"
        )
    if any(not name.strip() for name in names):
        raise RuntimeError(
            f"{side.side}-gripper JointState contains an empty joint name: {names!r}"
        )
    if len(set(names)) != len(names):
        raise RuntimeError(
            f"{side.side}-gripper JointState contains duplicate names: {names!r}"
        )
    if names != (side.joint_name,):
        raise RuntimeError(
            f"{side.side}-gripper JointState must identify exactly "
            f"{side.joint_name!r}; names={names!r}, positions={positions!r}"
        )
    if not all(math.isfinite(position) for position in positions):
        raise RuntimeError(
            f"{side.side}-gripper JointState contains non-finite positions: "
            f"{positions!r}"
        )

    position_rad = positions[0]
    if not (
        GRIPPER_LOWER_RAD - FEEDBACK_LIMIT_TOLERANCE_RAD
        <= position_rad
        <= GRIPPER_UPPER_RAD + FEEDBACK_LIMIT_TOLERANCE_RAD
    ):
        raise RuntimeError(
            f"{side.side}-gripper JointState is outside EX001 limits: "
            f"position={position_rad:.9f}rad, "
            f"limits=[{GRIPPER_LOWER_RAD}, {GRIPPER_UPPER_RAD}]"
        )

    header = getattr(value, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise RuntimeError(
            f"{side.side}-gripper JointState has an invalid timestamp: "
            f"sec={sec}, nanosec={nanosec}"
        )
    stamp_ns = sec * 1_000_000_000 + nanosec
    if stamp_ns <= 0:
        raise RuntimeError(
            f"{side.side}-gripper JointState has no positive simulation timestamp: "
            f"{stamp_ns}"
        )
    return GripperJointSample(
        position_rad=position_rad,
        stamp_ns=stamp_ns,
        names=names,
    )


def assert_stable(
    samples: list[GripperJointSample], label: str, side: GripperSide
) -> StableWindow:
    if not samples:
        raise RuntimeError(
            f"{label}: no {side.side}-gripper JointState samples"
        )
    expected_names = samples[0].names
    if any(sample.names != expected_names for sample in samples[1:]):
        raise RuntimeError(
            f"{label}: {side.side}-gripper joint identity changed within one window: "
            f"{[sample.names for sample in samples]!r}"
        )
    span = max(sample.position_rad for sample in samples) - min(
        sample.position_rad for sample in samples
    )
    if span > BASELINE_STABILITY_TOLERANCE_RAD:
        raise RuntimeError(
            f"{label}: {side.side}-gripper feedback is not stable; "
            f"span={span:.9f}rad exceeds "
            f"{BASELINE_STABILITY_TOLERANCE_RAD:.9f}rad"
        )
    return StableWindow(tuple(samples))


def capture_stable_window(
    gripper,
    side: GripperSide,
    *,
    sample_count: int,
    timeout: float,
    label: str,
    newer_than_ns: int = 0,
    expected_names: tuple[str, ...] | None = None,
) -> StableWindow:
    stream = gripper.get_joint_states_stream(timeout=timeout)
    samples: list[GripperJointSample] = []
    previous_stamp_ns = newer_than_ns
    try:
        iterator = iter(stream)
        while len(samples) < sample_count:
            try:
                value = require(
                    next(iterator),
                    f"{side.controller_name}.GetJointStatesStream",
                )
            except StopIteration as error:
                raise RuntimeError(
                    f"{label}: {side.side}-gripper JointState stream ended after "
                    f"{len(samples)}/{sample_count} samples"
                ) from error
            sample = parse_gripper_joint_sample(value, side)
            if sample.stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"{label}: {side.side}-gripper JointState timestamp did not "
                    f"advance; previous={previous_stamp_ns}, "
                    f"current={sample.stamp_ns}"
                )
            if expected_names is not None and sample.names != expected_names:
                raise RuntimeError(
                    f"{label}: {side.side}-gripper joint identity changed; "
                    f"expected={expected_names!r}, current={sample.names!r}"
                )
            previous_stamp_ns = sample.stamp_ns
            samples.append(sample)
    finally:
        close_stream(stream)
    return assert_stable(samples, label, side)


def wait_for_stable_position(
    gripper,
    side: GripperSide,
    *,
    expected_rad: float,
    baseline_rad: float,
    expected_names: tuple[str, ...],
    newer_than_ns: int,
    timeout: float,
    label: str,
    direction: int = 0,
    max_excursion_rad: float = SAME_VALUE_MAX_EXCURSION_RAD,
) -> StableWindow:
    stream = gripper.get_joint_states_stream(timeout=timeout)
    consecutive: list[GripperJointSample] = []
    previous_stamp_ns = newer_than_ns
    latest: GripperJointSample | None = None
    try:
        iterator = iter(stream)
        while True:
            try:
                value = require(
                    next(iterator),
                    f"{side.controller_name}.GetJointStatesStream",
                )
            except StopIteration as error:
                latest_text = (
                    "none"
                    if latest is None
                    else f"{latest.position_rad:.9f}rad"
                )
                raise RuntimeError(
                    f"{label}: {side.side}-gripper stream ended before stable "
                    f"convergence; expected={expected_rad:.9f}rad, "
                    f"latest={latest_text}"
                ) from error
            sample = parse_gripper_joint_sample(value, side)
            latest = sample
            if sample.stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"{label}: {side.side}-gripper JointState timestamp did not "
                    f"advance; previous={previous_stamp_ns}, "
                    f"current={sample.stamp_ns}"
                )
            if sample.names != expected_names:
                raise RuntimeError(
                    f"{label}: {side.side}-gripper joint identity changed; "
                    f"expected={expected_names!r}, current={sample.names!r}"
                )
            previous_stamp_ns = sample.stamp_ns

            excursion = abs(sample.position_rad - baseline_rad)
            if excursion > max_excursion_rad:
                raise RuntimeError(
                    f"{label}: {side.side} gripper exceeded the bounded envelope; "
                    f"baseline={baseline_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad, "
                    f"excursion={excursion:.9f}rad, "
                    f"maximum={max_excursion_rad:.9f}rad"
                )
            if (
                direction > 0
                and sample.position_rad
                < baseline_rad - POSITION_TOLERANCE_RAD
            ):
                raise RuntimeError(
                    f"{label}: {side.side} gripper moved opposite the positive "
                    f"target direction; baseline={baseline_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad"
                )
            if (
                direction < 0
                and sample.position_rad
                > baseline_rad + POSITION_TOLERANCE_RAD
            ):
                raise RuntimeError(
                    f"{label}: {side.side} gripper moved opposite the negative "
                    f"target direction; baseline={baseline_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad"
                )

            if abs(sample.position_rad - expected_rad) <= POSITION_TOLERANCE_RAD:
                consecutive.append(sample)
                if len(consecutive) > CONFIRMATION_SAMPLE_COUNT:
                    consecutive.pop(0)
                if len(consecutive) == CONFIRMATION_SAMPLE_COUNT:
                    return assert_stable(consecutive, label, side)
            else:
                consecutive.clear()
    finally:
        close_stream(stream)


def hold_verified_position(
    gripper,
    side: GripperSide,
    *,
    expected_rad: float,
    baseline_rad: float,
    expected_names: tuple[str, ...],
    newer_than_ns: int,
    hold_seconds: float,
    timeout: float,
    direction: int,
    max_excursion_rad: float,
) -> StableWindow:
    """Continuously prove the target until a monotonic hold deadline.

    This deliberately consumes every received frame instead of sleeping.  A
    stopped stream, stale/invalid frame, target drift, opposite movement, or
    envelope violation fails into the caller's single restoration path.
    """

    if not math.isfinite(hold_seconds) or hold_seconds <= 0.0:
        raise RuntimeError("DELTA_HOLD requires a positive finite duration")

    expires_at = time.monotonic() + hold_seconds
    # Bound a blocked ``next()`` by the hold plus one small final-frame grace.
    # A stream deadline is always a failure: old samples never prove that the
    # target remained valid through a feedback outage.
    stream = gripper.get_joint_states_stream(
        timeout=min(
            timeout,
            hold_seconds + HOLD_STREAM_DEADLINE_GRACE_SECONDS,
        )
    )
    samples: list[GripperJointSample] = []
    previous_stamp_ns = newer_than_ns
    try:
        iterator = iter(stream)
        while time.monotonic() < expires_at:
            try:
                result = next(iterator)
            except StopIteration as error:
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side}-gripper stream ended before "
                    "the monotonic hold deadline"
                ) from error
            value = require(
                result,
                f"{side.controller_name}.GetJointStatesStream",
            )

            sample = parse_gripper_joint_sample(value, side)
            if sample.stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side}-gripper JointState timestamp "
                    f"did not advance; previous={previous_stamp_ns}, "
                    f"current={sample.stamp_ns}"
                )
            if sample.names != expected_names:
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side}-gripper joint identity changed; "
                    f"expected={expected_names!r}, current={sample.names!r}"
                )
            previous_stamp_ns = sample.stamp_ns

            excursion = abs(sample.position_rad - baseline_rad)
            if excursion > max_excursion_rad:
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side} gripper exceeded the bounded "
                    f"envelope; baseline={baseline_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad, "
                    f"excursion={excursion:.9f}rad, "
                    f"maximum={max_excursion_rad:.9f}rad"
                )
            if (
                direction > 0
                and sample.position_rad
                < baseline_rad - POSITION_TOLERANCE_RAD
            ):
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side} gripper moved opposite the "
                    f"positive target direction; baseline={baseline_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad"
                )
            if (
                direction < 0
                and sample.position_rad
                > baseline_rad + POSITION_TOLERANCE_RAD
            ):
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side} gripper moved opposite the "
                    f"negative target direction; baseline={baseline_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad"
                )
            if abs(sample.position_rad - expected_rad) > POSITION_TOLERANCE_RAD:
                raise RuntimeError(
                    f"DELTA_HOLD: {side.side} gripper left the verified "
                    f"target; expected={expected_rad:.9f}rad, "
                    f"current={sample.position_rad:.9f}rad"
                )
            samples.append(sample)
    finally:
        close_stream(stream)

    if not samples:
        raise RuntimeError(
            f"DELTA_HOLD: no fresh {side.side}-gripper frame was verified"
        )
    return StableWindow(tuple(samples))


def read_one_joint_sample(
    gripper,
    side: GripperSide,
    *,
    timeout: float,
    expected_names: tuple[str, ...],
) -> GripperJointSample:
    return capture_stable_window(
        gripper,
        side,
        sample_count=1,
        timeout=timeout,
        label="pre-restore feedback",
        expected_names=expected_names,
    ).latest


def cross_check_get_position(
    gripper,
    side: GripperSide,
    authoritative: GripperJointSample,
    label: str,
) -> float:
    value = require(
        gripper.get_position(timeout=3.0),
        f"{side.controller_name}.GetPosition",
    )
    position_rad = float(value.position)
    if not math.isfinite(position_rad):
        raise RuntimeError(f"{label}: GetPosition returned {position_rad!r}")
    if not (
        GRIPPER_LOWER_RAD - FEEDBACK_LIMIT_TOLERANCE_RAD
        <= position_rad
        <= GRIPPER_UPPER_RAD + FEEDBACK_LIMIT_TOLERANCE_RAD
    ):
        raise RuntimeError(
            f"{label}: GetPosition is outside EX001 limits: "
            f"{position_rad:.9f}rad"
        )
    difference = abs(position_rad - authoritative.position_rad)
    if difference > GET_CROSSCHECK_TOLERANCE_RAD:
        raise RuntimeError(
            f"{label}: GetPosition disagrees with named JointState feedback; "
            f"get={position_rad:.9f}rad, "
            f"stream={authoritative.position_rad:.9f}rad, "
            f"difference={difference:.9f}rad"
        )
    print(
        f"{label} GET_CROSSCHECK PASS get={position_rad:.9f}rad "
        f"stream={authoritative.position_rad:.9f}rad "
        f"difference={difference:.9f}rad"
    )
    return position_rad


def set_gripper_position(
    gripper,
    side: GripperSide,
    position_rad: float,
    operation: str,
) -> None:
    if not math.isfinite(position_rad):
        raise RuntimeError(
            f"{operation}: refusing non-finite target {position_rad!r}"
        )
    if not GRIPPER_LOWER_RAD <= position_rad <= GRIPPER_UPPER_RAD:
        raise RuntimeError(
            f"{operation}: target {position_rad:.9f}rad is outside "
            f"[{GRIPPER_LOWER_RAD}, {GRIPPER_UPPER_RAD}]"
        )
    require(
        gripper.set_position(
            GripperPosition(position=position_rad),
            timeout=3.0,
        ),
        operation,
    )


def enter_sdk_mode_idempotently(robot) -> None:
    global _sdk_mode_entered
    request = RobotModeParam(mode=RobotWorkMode.SDK)
    for attempt in (1, 2):
        require(
            robot.system.set_work_mode(request, timeout=5.0),
            f"System.SetWorkMode(SDK) attempt {attempt}",
        )
        _sdk_mode_entered = True
        print(f"SDK_MODE PASS attempt={attempt}/2")


def choose_delta_target(
    baseline_rad: float, delta_rad: float
) -> tuple[float, int]:
    if not math.isfinite(baseline_rad):
        raise RuntimeError(f"baseline is non-finite: {baseline_rad!r}")
    if not (
        GRIPPER_LOWER_RAD - FEEDBACK_LIMIT_TOLERANCE_RAD
        <= baseline_rad
        <= GRIPPER_UPPER_RAD + FEEDBACK_LIMIT_TOLERANCE_RAD
    ):
        raise RuntimeError(
            f"baseline {baseline_rad:.9f}rad is outside EX001 limits"
        )
    distance_to_lower = baseline_rad - GRIPPER_LOWER_RAD
    distance_to_upper = GRIPPER_UPPER_RAD - baseline_rad
    direction = 1 if distance_to_lower <= distance_to_upper else -1
    target_rad = baseline_rad + direction * delta_rad
    if not GRIPPER_LOWER_RAD < target_rad < GRIPPER_UPPER_RAD:
        raise RuntimeError(
            f"computed target {target_rad:.9f}rad is not strictly inside "
            f"[{GRIPPER_LOWER_RAD}, {GRIPPER_UPPER_RAD}]"
        )
    if not math.isclose(
        abs(target_rad - baseline_rad),
        delta_rad,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(
            f"computed target delta is not exactly {delta_rad:.2f}rad: "
            f"baseline={baseline_rad:.9f}rad, "
            f"target={target_rad:.9f}rad"
        )
    return target_rad, direction


def delta_acknowledgement(side: GripperSide, delta_rad_text: str) -> str:
    return (
        f"SIMULATOR_{side.side.upper()}_GRIPPER_DELTA_"
        f"{delta_rad_text}RAD_AND_RESTORE"
    )


def validate_server_address(address: str) -> str:
    normalized = address.strip()
    for prefix in ("grpc://", "x2://"):
        if normalized.startswith(prefix):
            normalized = normalized[len(prefix) :]
            break

    if normalized.startswith("["):
        closing = normalized.find("]")
        if closing < 0 or normalized[closing + 1 : closing + 2] != ":":
            raise SystemExit("--server must be a loopback host:port endpoint")
        host = normalized[1:closing]
        port_text = normalized[closing + 2 :]
    else:
        host, separator, port_text = normalized.rpartition(":")
        if not separator:
            raise SystemExit("--server must include a TCP port")

    if host.lower() not in LOOPBACK_HOSTS:
        raise SystemExit(
            "--server must use a loopback SSH tunnel; "
            f"refusing non-loopback host {host!r}"
        )
    try:
        port = int(port_text)
    except ValueError as error:
        raise SystemExit("--server port must be numeric") from error
    if not 1 <= port <= 65535:
        raise SystemExit("--server port must be in [1, 65535]")
    return normalized


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--server",
        default="127.0.0.1:15551",
        help="loopback SSH-tunnel endpoint only",
    )
    parser.add_argument(
        "--side",
        choices=tuple(GRIPPER_SIDES),
        default="left",
        help="gripper side; left remains the backward-compatible default",
    )
    action = parser.add_mutually_exclusive_group()
    action.add_argument(
        "--same-value-only",
        action="store_true",
        help="write the selected gripper's captured absolute position and verify it",
    )
    action.add_argument(
        "--apply-delta",
        action="store_true",
        help="same-value gate, one reviewed delta, and baseline restoration",
    )
    parser.add_argument(
        "--delta-rad",
        choices=tuple(REVIEWED_DELTA_RAD_BY_TEXT),
        default=DEFAULT_DELTA_RAD_TEXT,
        help="reviewed gripper demo delta; only used with --apply-delta",
    )
    parser.add_argument(
        "--allow-write",
        action="store_true",
        help="required explicit write gate for either mutating action",
    )
    parser.add_argument(
        "--confirm-simulator-only",
        action="store_true",
        help="confirm the target is the watched simulator, never a real robot",
    )
    parser.add_argument(
        "--acknowledgement",
        default="",
        help="exact action-specific acknowledgement",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=8.0,
        help="bounded feedback deadline in seconds",
    )
    parser.add_argument(
        "--target-hold-seconds",
        type=float,
        default=0.0,
        help=(
            "after fresh delta feedback, hold the verified target for up to "
            "5 seconds before restoration; requires --apply-delta"
        ),
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    args.server = validate_server_address(args.server)
    side = GRIPPER_SIDES[args.side]
    delta_rad_text = getattr(args, "delta_rad_text", None)
    if delta_rad_text not in REVIEWED_DELTA_RAD_BY_TEXT:
        if isinstance(args.delta_rad, str):
            delta_rad_text = args.delta_rad
        else:
            delta_rad_text = next(
                (
                    text
                    for text, value in REVIEWED_DELTA_RAD_BY_TEXT.items()
                    if math.isclose(
                        float(args.delta_rad), value, rel_tol=0.0, abs_tol=1e-12
                    )
                ),
                "",
            )
    if delta_rad_text not in REVIEWED_DELTA_RAD_BY_TEXT:
        raise SystemExit(
            "--delta-rad must be one of 0.03, 0.10, 0.20, 0.50, or 1.00"
        )
    delta_rad = REVIEWED_DELTA_RAD_BY_TEXT[delta_rad_text]
    args.delta_rad_text = delta_rad_text
    args.delta_rad = delta_rad
    mutating = args.same_value_only or args.apply_delta
    if mutating and not args.allow_write:
        raise SystemExit("mutating mode requires --allow-write")
    if mutating and not args.confirm_simulator_only:
        raise SystemExit("mutating mode requires --confirm-simulator-only")
    if not mutating and (args.allow_write or args.confirm_simulator_only):
        raise SystemExit(
            "--allow-write/--confirm-simulator-only require a mutating action"
        )

    expected_ack = ""
    if args.same_value_only:
        expected_ack = side.same_value_acknowledgement
    elif args.apply_delta:
        expected_ack = delta_acknowledgement(side, delta_rad_text)
    if mutating and args.acknowledgement != expected_ack:
        raise SystemExit(
            "selected action requires --acknowledgement " + expected_ack
        )
    if not mutating and args.acknowledgement:
        raise SystemExit("--acknowledgement requires a mutating action")
    if not args.apply_delta and delta_rad_text != DEFAULT_DELTA_RAD_TEXT:
        raise SystemExit("non-default --delta-rad requires --apply-delta")
    if not 2.0 <= args.settle_timeout <= 30.0:
        raise SystemExit("--settle-timeout must be in [2.0, 30.0] seconds")
    if not math.isfinite(args.target_hold_seconds):
        raise SystemExit("--target-hold-seconds must be finite")
    if not 0.0 <= args.target_hold_seconds <= 5.0:
        raise SystemExit("--target-hold-seconds must be in [0.0, 5.0] seconds")
    if args.target_hold_seconds > 0.0 and not args.apply_delta:
        raise SystemExit("--target-hold-seconds requires --apply-delta")
    if (
        args.target_hold_seconds > 0.0
        and args.settle_timeout
        < args.target_hold_seconds + HOLD_STREAM_DEADLINE_GRACE_SECONDS
    ):
        raise SystemExit(
            "--settle-timeout must be at least --target-hold-seconds + "
            f"{HOLD_STREAM_DEADLINE_GRACE_SECONDS:.2f} seconds"
        )


def capture_baseline(
    gripper,
    side: GripperSide,
    timeout: float,
    label: str,
) -> StableWindow:
    window = capture_stable_window(
        gripper,
        side,
        sample_count=BASELINE_SAMPLE_COUNT,
        timeout=timeout,
        label=label,
    )
    latest = window.latest
    print(
        f"{label} PASS samples={len(window.samples)} names={latest.names!r} "
        f"stamp_ns={latest.stamp_ns} position={latest.position_rad:.9f}rad"
    )
    cross_check_get_position(gripper, side, latest, label)
    return window


def exercise(robot, args: argparse.Namespace, safety: SafetyState) -> int:
    side = GRIPPER_SIDES[args.side]
    gripper = selected_gripper(robot, side)
    model = require(
        robot.system.get_model_type(timeout=3.0),
        "System.GetModelType",
    )
    if int(model.model_type) != int(RobotModelType.QUANTA_X1_PRO):
        raise RuntimeError(
            f"expected EX001/QUANTA_X1_PRO, got {model.model_type!r}"
        )
    print(f"MODEL PASS model={model.model_type!r} transport={robot.transport}")
    print(
        "SAFETY NOTE: active simulator profile, endpoint ownership, visible "
        "scene, and absence of a restoration hold are external prerequisites"
    )
    print(
        f"SIDE side={side.side} command_topic={side.command_topic} "
        f"state_topic={side.state_topic} joint={side.joint_name}"
    )

    observed = capture_baseline(
        gripper, side, args.settle_timeout, "OBSERVE_BASELINE"
    )
    if not args.same_value_only and not args.apply_delta:
        print("OBSERVE_ONLY PASS: no mode or Set RPC was sent")
        return 0

    baseline = observed.latest
    if not (
        GRIPPER_LOWER_RAD
        <= baseline.position_rad
        <= GRIPPER_UPPER_RAD
    ):
        raise WriteNotRunError(
            f"captured {side.side}-gripper baseline is not a legal command "
            f"target: {baseline.position_rad:.9f}rad"
        )

    target_rad: float | None = None
    direction = 0
    max_excursion_rad = SAME_VALUE_MAX_EXCURSION_RAD
    if args.apply_delta:
        try:
            target_rad, direction = choose_delta_target(
                baseline.position_rad, args.delta_rad
            )
        except RuntimeError as error:
            raise WriteNotRunError(
                f"requested {args.delta_rad_text}rad delta has no legal "
                f"target from baseline {baseline.position_rad:.9f}rad: {error}"
            ) from error
        max_excursion_rad = max(
            SAME_VALUE_MAX_EXCURSION_RAD,
            args.delta_rad + EXCURSION_MARGIN_RAD,
        )
        direction_name = (
            "open/positive" if direction > 0 else "close/negative"
        )
        print(
            f"TARGET_PLAN baseline={baseline.position_rad:.9f}rad "
            f"target={target_rad:.9f}rad "
            f"delta={direction * args.delta_rad:+.2f}rad "
            f"direction={direction_name} "
            f"maximum_excursion={max_excursion_rad:.9f}rad"
        )

    print("WRITE_GATES PASS: explicit flags and acknowledgement accepted")
    enter_sdk_mode_idempotently(robot)

    prewrite = capture_baseline(
        gripper, side, args.settle_timeout, "PREWRITE_BASELINE"
    )
    if prewrite.latest.names != observed.latest.names:
        raise RuntimeError(
            f"{side.side}-gripper joint identity changed during the SDK-mode gate; "
            f"before={observed.latest.names!r}, "
            f"after={prewrite.latest.names!r}"
        )
    if (
        abs(prewrite.latest.position_rad - observed.latest.position_rad)
        > BASELINE_STABILITY_TOLERANCE_RAD
    ):
        raise RuntimeError(
            f"{side.side} gripper changed during the SDK-mode gate; "
            f"before={observed.latest.position_rad:.9f}rad, "
            f"after={prewrite.latest.position_rad:.9f}rad"
        )
    safety.actuator_write_attempted = True
    same_value_error: BaseException | None = None
    try:
        set_gripper_position(
            gripper,
            side,
            baseline.position_rad,
            f"{side.controller_name}.SetPosition(same-value)",
        )
    except BaseException as error:
        same_value_error = error

    try:
        same_value = wait_for_stable_position(
            gripper,
            side,
            expected_rad=baseline.position_rad,
            baseline_rad=baseline.position_rad,
            expected_names=baseline.names,
            newer_than_ns=prewrite.latest.stamp_ns,
            timeout=args.settle_timeout,
            label="SAME_VALUE",
        )
        cross_check_get_position(
            gripper, side, same_value.latest, "SAME_VALUE"
        )
        safety.restoration_confirmed = True
        print(
            f"SAME_VALUE PASS samples={len(same_value.samples)} "
            f"stamp_ns={same_value.latest.stamp_ns} "
            f"position={same_value.latest.position_rad:.9f}rad"
        )
    except BaseException as feedback_error:
        if same_value_error is not None:
            raise RestoreUnconfirmedError(
                f"same-value write failed ({same_value_error}); three-frame "
                f"baseline confirmation also failed ({feedback_error})"
            ) from feedback_error
        raise RestoreUnconfirmedError(
            "same-value write began, but three-frame exact baseline feedback "
            f"was not confirmed: {feedback_error}"
        ) from feedback_error

    if same_value_error is not None:
        raise TargetFailedRestoredError(
            "same-value Set RPC failed, but three fresh baseline frames "
            f"confirmed restoration: {same_value_error}"
        ) from same_value_error
    if args.same_value_only:
        return 0

    if target_rad is None:
        raise RuntimeError("delta target was not planned before the write gate")
    print(
        f"TARGET baseline={baseline.position_rad:.9f}rad "
        f"target={target_rad:.9f}rad "
        f"delta={direction * args.delta_rad:+.2f}rad "
        f"maximum_excursion={max_excursion_rad:.9f}rad"
    )

    primary_error: BaseException | None = None
    target_attempted = False
    safety.restoration_confirmed = False
    try:
        target_attempted = True
        set_gripper_position(
            gripper,
            side,
            target_rad,
            f"{side.controller_name}.SetPosition(delta)",
        )
        target = wait_for_stable_position(
            gripper,
            side,
            expected_rad=target_rad,
            baseline_rad=baseline.position_rad,
            expected_names=baseline.names,
            newer_than_ns=same_value.latest.stamp_ns,
            timeout=args.settle_timeout,
            label="DELTA_TARGET",
            direction=direction,
            max_excursion_rad=max_excursion_rad,
        )
        cross_check_get_position(
            gripper, side, target.latest, "DELTA_TARGET"
        )
        print(
            f"DELTA_TARGET PASS samples={len(target.samples)} "
            f"stamp_ns={target.latest.stamp_ns} "
            f"position={target.latest.position_rad:.9f}rad"
        )
        if args.target_hold_seconds > 0.0:
            print(
                "NUMERIC_TARGET_HOLD START "
                f"seconds={args.target_hold_seconds:.3f} "
                f"target={target_rad:.9f}rad",
                flush=True,
            )
            held = hold_verified_position(
                gripper,
                side,
                expected_rad=target_rad,
                baseline_rad=baseline.position_rad,
                expected_names=baseline.names,
                newer_than_ns=target.latest.stamp_ns,
                hold_seconds=args.target_hold_seconds,
                timeout=args.settle_timeout,
                direction=direction,
                max_excursion_rad=max_excursion_rad,
            )
            cross_check_get_position(
                gripper, side, held.latest, "DELTA_HOLD"
            )
            print(
                f"NUMERIC_TARGET_HOLD PASS samples={len(held.samples)} "
                f"stamp_ns={held.latest.stamp_ns} "
                f"position={held.latest.position_rad:.9f}rad"
            )
    except BaseException as error:
        primary_error = error
    finally:
        if target_attempted:
            restore_reference_ns = same_value.latest.stamp_ns
            try:
                restore_reference = read_one_joint_sample(
                    gripper,
                    side,
                    timeout=min(2.0, args.settle_timeout),
                    expected_names=baseline.names,
                )
                restore_reference_ns = max(
                    restore_reference_ns,
                    restore_reference.stamp_ns,
                )
            except BaseException as reference_error:
                print(
                    f"warning: pre-restore {side.side}-gripper feedback unavailable; "
                    f"sending the single restore command: {reference_error}",
                    file=sys.stderr,
                )

            try:
                # This is one explicit recovery write, never a retry of the
                # target command.  No further actuator write is sent on error.
                set_gripper_position(
                    gripper,
                    side,
                    baseline.position_rad,
                    f"{side.controller_name}.SetPosition(restore)",
                )
                restored = wait_for_stable_position(
                    gripper,
                    side,
                    expected_rad=baseline.position_rad,
                    baseline_rad=baseline.position_rad,
                    expected_names=baseline.names,
                    newer_than_ns=restore_reference_ns,
                    timeout=args.settle_timeout,
                    label="RESTORE",
                    max_excursion_rad=max_excursion_rad,
                )
                cross_check_get_position(
                    gripper, side, restored.latest, "RESTORE"
                )
                safety.restoration_confirmed = True
                print(
                    f"RESTORE PASS samples={len(restored.samples)} "
                    f"stamp_ns={restored.latest.stamp_ns} "
                    f"position={restored.latest.position_rad:.9f}rad"
                )
            except BaseException as restore_error:
                if primary_error is not None:
                    raise RestoreUnconfirmedError(
                        f"target test failed ({primary_error}); exact baseline "
                        f"restoration also failed ({restore_error})"
                    ) from restore_error
                raise RestoreUnconfirmedError(
                    f"exact baseline restoration failed: {restore_error}"
                ) from restore_error

    if primary_error is not None:
        raise TargetFailedRestoredError(
            "delta target did not pass, but exact baseline restoration was "
            f"confirmed: {primary_error}"
        ) from primary_error
    print(
        "NUMERIC_DELTA_AND_RESTORE PASS: visual confirmation remains external"
    )
    return 0


_sdk_mode_entered = False


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


def main() -> int:
    args = parse_args()
    validate_args(args)
    side = GRIPPER_SIDES[args.side]
    safety = SafetyState()

    robot = require(
        x2robot_sdk.connect(args.server, force_grpc=True),
        "connect",
    )
    try:
        try:
            return exercise(robot, args, safety)
        finally:
            restore_idle_mode(robot)
            closed = robot.close()
            if (
                isinstance(closed, x2robot_sdk.SdkResult)
                and not closed.is_success
            ):
                raise RuntimeError(f"Robot.close failed: {closed.error}")
    except BaseException as error:
        if safety.actuator_write_attempted:
            if isinstance(error, RestoreUnconfirmedError):
                raise
            if safety.restoration_confirmed:
                if isinstance(error, TargetFailedRestoredError):
                    raise
                raise TargetFailedRestoredError(
                    f"failure after a {side.side}-gripper actuator write, with exact "
                    f"baseline restoration confirmed: {error}"
                ) from error
            raise RestoreUnconfirmedError(
                f"failure after a {side.side}-gripper actuator write; do not retry or "
                "perform SDK lifecycle actions "
                f"(restore_feedback=not_confirmed): {error}"
            ) from error
        raise


def cli() -> int:
    try:
        return main()
    except WriteNotRunError as error:
        print(f"WRITE_NOT_RUN: {error}", file=sys.stderr)
        return 1
    except RestoreUnconfirmedError as error:
        print(f"RESTORE_UNCONFIRMED: {error}", file=sys.stderr)
        return 2
    except TargetFailedRestoredError as error:
        print(f"TARGET_FAILED_RESTORED: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(
            "INTERRUPTED during selected-gripper smoke test; inspect the "
            "last output before any retry",
            file=sys.stderr,
        )
        return 130
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(cli())
