#!/usr/bin/env python3
"""Guarded SDK2 Lift smoke test for the EX001 ManaEnv simulator.

The default mode is read-only.  A mutating mode requires one explicit action,
``--allow-write``, ``--confirm-simulator-only``, and that action's exact
acknowledgement.  ``--canonical-zero-delta`` is the only mode that may start
from a tiny numerical undershoot at the lower stop: every fresh baseline frame
must be inside a dedicated micrometre-scale read tolerance, after which the
script sends the literal legal target ``0.0`` before the bounded delta.  It
never clamps or reflects the negative feedback into the command API.

The authoritative feedback is the public SDK
``LiftController.GetJointStatesStream`` result, not ``GetLiftPosition``.
``GetLiftPosition`` is used only as an independent cross-check.

Before a mutating run, externally prove the active SDK release uses
``runtime_profile=simulator`` and prove exact owner/type/QoS relationships for
``/lift_position_controller/commands`` and ``/lift/joint_states``.  The script
does not inspect the ROS graph, publish raw ROS messages, or distinguish a
simulator from a real robot on its own.  Do not run a mutating mode while any
previous control attempt is classified ``RESTORE_UNCONFIRMED``.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

import x2robot_sdk
from x2robot.sdk import LiftPosition, RobotModeParam, RobotWorkMode
from x2robot.sdk.v2 import RobotModelType


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}

LIFT_JOINT_NAME = "lift_joint"
LIFT_LOWER_M = 0.0
LIFT_UPPER_M = 0.78
DELTA_M = 0.005
MIN_DEMO_DELTA_M = 0.005
MAX_DEMO_DELTA_M = 0.200
MAX_SETTLE_TIMEOUT_SECONDS = 90.0
PROGRESS_REPORT_INTERVAL_NS = 5_000_000_000

# Simulator feedback can settle a few micrometres below the authored 0 m joint
# limit.  This is a read-side gate only: every command remains strictly inside
# [LIFT_LOWER_M, LIFT_UPPER_M], and canonical zero is always sent as literal
# 0.0.  Keep this substantially tighter than the ordinary 2 mm convergence
# tolerance so a displaced Lift cannot be mistaken for the lower stop.
CANONICAL_ZERO_TARGET_M = 0.0
CANONICAL_ZERO_FEEDBACK_TOLERANCE_M = 0.000005
CANONICAL_ZERO_STABILITY_TOLERANCE_M = 0.000002

BASELINE_SAMPLE_COUNT = 5
CONFIRMATION_SAMPLE_COUNT = 3
BASELINE_STABILITY_TOLERANCE_M = 0.001
POSITION_TOLERANCE_M = 0.002
GET_CROSSCHECK_TOLERANCE_M = 0.002
# Keep a narrow allowance above the largest explicitly supported demo target
# for feedback settling/overshoot.  This is independent of the Server's wider
# physical limit and does not authorize targets above MAX_DEMO_DELTA_M.
MAX_TEST_EXCURSION_M = 0.210

SAME_VALUE_ACKNOWLEDGEMENT = "SIMULATOR_LIFT_SAME_VALUE"
DELTA_ACKNOWLEDGEMENT = "SIMULATOR_LIFT_DELTA_0.005M_AND_RESTORE"
CANONICAL_ZERO_DELTA_ACKNOWLEDGEMENT = (
    "SIMULATOR_LIFT_CANONICAL_ZERO_THEN_DELTA_0.005M_AND_RESTORE"
)


@dataclass(frozen=True)
class LiftJointSample:
    position_m: float
    stamp_ns: int
    names: tuple[str, ...]


@dataclass(frozen=True)
class StableWindow:
    samples: tuple[LiftJointSample, ...]

    @property
    def latest(self) -> LiftJointSample:
        return self.samples[-1]


@dataclass
class SafetyState:
    actuator_write_attempted: bool = False
    restoration_confirmed: bool = False


class RestoreUnconfirmedError(RuntimeError):
    """A failure occurred after at least one Lift actuator Set RPC began."""


class TargetFailedRestoredError(RuntimeError):
    """The bounded target failed, but exact baseline restoration was confirmed."""


class NoLegalRestoreError(RuntimeError):
    """The observed Lift baseline cannot legally be sent back as a command."""


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


def normalize_joint_name(name: str) -> str:
    return name.rsplit("/", 1)[-1]


def parse_lift_joint_sample(value) -> LiftJointSample:
    names = tuple(str(name) for name in value.name)
    positions = tuple(float(position) for position in value.position)
    if not names or len(names) != len(positions):
        raise RuntimeError(
            "Lift JointState must contain equally sized, non-empty name and "
            f"position arrays; names={names!r}, positions={positions!r}"
        )
    if len(set(names)) != len(names):
        raise RuntimeError(f"Lift JointState contains duplicate names: {names!r}")
    if not all(math.isfinite(position) for position in positions):
        raise RuntimeError(
            f"Lift JointState contains non-finite positions: {positions!r}"
        )

    matches = [
        index
        for index, name in enumerate(names)
        if normalize_joint_name(name) == LIFT_JOINT_NAME
    ]
    if len(matches) != 1:
        raise RuntimeError(
            "Lift JointState must map exactly one name to lift_joint; "
            f"names={names!r}, matches={matches!r}"
        )
    position_m = positions[matches[0]]

    header = getattr(value, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise RuntimeError(
            f"Lift JointState has an invalid timestamp: sec={sec}, nanosec={nanosec}"
        )
    stamp_ns = sec * 1_000_000_000 + nanosec
    if stamp_ns <= 0:
        raise RuntimeError(
            f"Lift JointState has no positive simulation timestamp: {stamp_ns}"
        )
    return LiftJointSample(
        position_m=position_m,
        stamp_ns=stamp_ns,
        names=names,
    )


def assert_stable(
    samples: list[LiftJointSample],
    label: str,
    *,
    tolerance_m: float = BASELINE_STABILITY_TOLERANCE_M,
) -> StableWindow:
    if not samples:
        raise RuntimeError(f"{label}: no Lift JointState samples")
    span = max(sample.position_m for sample in samples) - min(
        sample.position_m for sample in samples
    )
    if span > tolerance_m:
        raise RuntimeError(
            f"{label}: Lift feedback is not stable; span={span:.9f}m exceeds "
            f"{tolerance_m:.9f}m"
        )
    return StableWindow(tuple(samples))


def capture_stable_window(
    robot,
    *,
    sample_count: int,
    timeout: float,
    label: str,
    newer_than_ns: int = 0,
) -> StableWindow:
    stream = robot.lift.get_joint_states_stream(timeout=timeout)
    samples: list[LiftJointSample] = []
    previous_stamp_ns = newer_than_ns
    try:
        iterator = iter(stream)
        while len(samples) < sample_count:
            try:
                value = require(
                    next(iterator),
                    "LiftController.GetJointStatesStream",
                )
            except StopIteration as error:
                raise RuntimeError(
                    f"{label}: Lift JointState stream ended after "
                    f"{len(samples)}/{sample_count} samples"
                ) from error
            sample = parse_lift_joint_sample(value)
            if sample.stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"{label}: Lift JointState timestamp did not advance; "
                    f"previous={previous_stamp_ns}, current={sample.stamp_ns}"
                )
            previous_stamp_ns = sample.stamp_ns
            samples.append(sample)
    finally:
        close_stream(stream)
    return assert_stable(samples, label)


def wait_for_stable_position(
    robot,
    *,
    expected_m: float,
    baseline_m: float,
    newer_than_ns: int,
    timeout: float,
    label: str,
    direction: int = 0,
    position_tolerance_m: float = POSITION_TOLERANCE_M,
    stability_tolerance_m: float = BASELINE_STABILITY_TOLERANCE_M,
) -> StableWindow:
    stream = robot.lift.get_joint_states_stream(timeout=timeout)
    consecutive: list[LiftJointSample] = []
    previous_stamp_ns = newer_than_ns
    last_report_stamp_ns = newer_than_ns
    latest: LiftJointSample | None = None
    try:
        iterator = iter(stream)
        while True:
            try:
                result = next(iterator)
            except StopIteration as error:
                latest_text = "none" if latest is None else f"{latest.position_m:.9f}m"
                raise RuntimeError(
                    f"{label}: Lift stream ended before stable convergence; "
                    f"expected={expected_m:.9f}m, latest={latest_text}"
                ) from error
            try:
                value = require(
                    result,
                    "LiftController.GetJointStatesStream",
                )
            except Exception as error:
                latest_text = (
                    "none" if latest is None else f"{latest.position_m:.9f}m"
                )
                raise RuntimeError(
                    f"{label}: feedback stream failed before stable "
                    f"convergence; expected={expected_m:.9f}m, "
                    f"latest={latest_text}: {error}"
                ) from error
            sample = parse_lift_joint_sample(value)
            latest = sample
            if sample.stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"{label}: Lift JointState timestamp did not advance; "
                    f"previous={previous_stamp_ns}, current={sample.stamp_ns}"
                )
            previous_stamp_ns = sample.stamp_ns

            if (
                sample.stamp_ns - last_report_stamp_ns
                >= PROGRESS_REPORT_INTERVAL_NS
            ):
                print(
                    f"{label} PROGRESS stamp_ns={sample.stamp_ns} "
                    f"position={sample.position_m:.9f}m "
                    f"remaining={expected_m - sample.position_m:+.9f}m"
                )
                last_report_stamp_ns = sample.stamp_ns

            excursion = abs(sample.position_m - baseline_m)
            if excursion > MAX_TEST_EXCURSION_M:
                raise RuntimeError(
                    f"{label}: Lift exceeded the bounded envelope; "
                    f"baseline={baseline_m:.9f}m, current={sample.position_m:.9f}m, "
                    f"excursion={excursion:.9f}m"
                )
            if direction > 0 and sample.position_m < baseline_m - POSITION_TOLERANCE_M:
                raise RuntimeError(
                    f"{label}: Lift moved opposite the positive target direction; "
                    f"baseline={baseline_m:.9f}m, current={sample.position_m:.9f}m"
                )
            if direction < 0 and sample.position_m > baseline_m + POSITION_TOLERANCE_M:
                raise RuntimeError(
                    f"{label}: Lift moved opposite the negative target direction; "
                    f"baseline={baseline_m:.9f}m, current={sample.position_m:.9f}m"
                )

            if abs(sample.position_m - expected_m) <= position_tolerance_m:
                consecutive.append(sample)
                if len(consecutive) > CONFIRMATION_SAMPLE_COUNT:
                    consecutive.pop(0)
                if len(consecutive) == CONFIRMATION_SAMPLE_COUNT:
                    return assert_stable(
                        consecutive,
                        label,
                        tolerance_m=stability_tolerance_m,
                    )
            else:
                consecutive.clear()
    finally:
        close_stream(stream)


def read_one_joint_sample(robot, timeout: float = 2.0) -> LiftJointSample:
    return capture_stable_window(
        robot,
        sample_count=1,
        timeout=timeout,
        label="pre-restore feedback",
    ).latest


def cross_check_get_position(
    robot,
    authoritative: LiftJointSample,
    label: str,
    *,
    tolerance_m: float = GET_CROSSCHECK_TOLERANCE_M,
) -> float:
    value = require(
        robot.lift.get_lift_position(timeout=3.0),
        "LiftController.GetLiftPosition",
    )
    position_m = float(value.position)
    if not math.isfinite(position_m):
        raise RuntimeError(f"{label}: GetLiftPosition returned {position_m!r}")
    difference = abs(position_m - authoritative.position_m)
    if difference > tolerance_m:
        raise RuntimeError(
            f"{label}: GetLiftPosition disagrees with lift_joint feedback; "
            f"get={position_m:.9f}m, stream={authoritative.position_m:.9f}m, "
            f"difference={difference:.9f}m"
        )
    print(
        f"{label} GET_CROSSCHECK PASS get={position_m:.9f}m "
        f"stream={authoritative.position_m:.9f}m difference={difference:.9f}m"
    )
    return position_m


def set_lift_position(
    robot,
    position_m: float,
    operation: str,
) -> None:
    if not math.isfinite(position_m):
        raise RuntimeError(f"{operation}: refusing non-finite target {position_m!r}")
    if not LIFT_LOWER_M <= position_m <= LIFT_UPPER_M:
        raise RuntimeError(
            f"{operation}: target {position_m:.9f}m is outside "
            f"[{LIFT_LOWER_M}, {LIFT_UPPER_M}]"
        )
    require(
        robot.lift.set_lift_position(
            LiftPosition(position=position_m),
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
    baseline_m: float, delta_m: float = DELTA_M
) -> tuple[float, int]:
    if not math.isfinite(baseline_m):
        raise RuntimeError(f"baseline is non-finite: {baseline_m!r}")
    if not LIFT_LOWER_M <= baseline_m <= LIFT_UPPER_M:
        raise RuntimeError(
            f"baseline {baseline_m:.9f}m is outside EX001 limits"
        )
    distance_to_lower = baseline_m - LIFT_LOWER_M
    distance_to_upper = LIFT_UPPER_M - baseline_m
    direction = 1 if distance_to_lower <= distance_to_upper else -1
    target_m = baseline_m + direction * delta_m
    if not math.isfinite(target_m):
        raise RuntimeError(f"computed target is non-finite: {target_m!r}")
    if not LIFT_LOWER_M < target_m < LIFT_UPPER_M:
        raise RuntimeError(
            f"computed target {target_m:.9f}m is not strictly inside "
            f"[{LIFT_LOWER_M}, {LIFT_UPPER_M}]"
        )
    if not math.isclose(
        abs(target_m - baseline_m),
        delta_m,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(
            f"computed target delta is not exactly {delta_m:.3f}m: "
            f"baseline={baseline_m:.9f}m, target={target_m:.9f}m"
        )
    return target_m, direction


def require_legal_restore_target(
    sample: LiftJointSample, label: str
) -> None:
    if not LIFT_LOWER_M <= sample.position_m <= LIFT_UPPER_M:
        raise NoLegalRestoreError(
            f"{label}: captured Lift baseline {sample.position_m:.9f}m "
            f"cannot be restored through strict command limits "
            f"[{LIFT_LOWER_M}, {LIFT_UPPER_M}]"
        )


def require_canonical_zero_window(
    window: StableWindow, label: str
) -> None:
    """Require fresh stable feedback close enough to explicitly command 0 m.

    ``capture_stable_window``/``wait_for_stable_position`` own timestamp
    freshness.  This additional check deliberately validates every sample,
    not merely the latest one, and never changes a feedback or command value.
    """

    if not window.samples:
        raise NoLegalRestoreError(f"{label}: no Lift samples for canonical zero")
    maximum_error = max(abs(sample.position_m) for sample in window.samples)
    span = max(sample.position_m for sample in window.samples) - min(
        sample.position_m for sample in window.samples
    )
    if maximum_error > CANONICAL_ZERO_FEEDBACK_TOLERANCE_M:
        raise NoLegalRestoreError(
            f"{label}: Lift feedback is not inside the canonical-zero read "
            f"window +/-{CANONICAL_ZERO_FEEDBACK_TOLERANCE_M:.9f}m; "
            f"maximum_error={maximum_error:.9f}m"
        )
    if span > CANONICAL_ZERO_STABILITY_TOLERANCE_M:
        raise NoLegalRestoreError(
            f"{label}: Lift feedback is not stable enough for canonical zero; "
            f"span={span:.9f}m exceeds "
            f"{CANONICAL_ZERO_STABILITY_TOLERANCE_M:.9f}m"
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
    action = parser.add_mutually_exclusive_group()
    action.add_argument(
        "--same-value-only",
        action="store_true",
        help="write the captured absolute Lift position once and verify feedback",
    )
    action.add_argument(
        "--apply-delta",
        action="store_true",
        help="same-value gate, one bounded delta, and exact baseline restoration",
    )
    action.add_argument(
        "--canonical-zero-delta",
        action="store_true",
        help=(
            "for a micrometre-scale lower-stop undershoot only: explicitly "
            "establish legal 0.0m, apply one bounded positive delta, and "
            "restore literal 0.0m"
        ),
    )
    parser.add_argument(
        "--allow-write",
        action="store_true",
        help="required explicit write gate for a mutating action",
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
        "--delta-m",
        type=float,
        default=DELTA_M,
        help="bounded Lift demo delta in metres (0.005 to 0.200)",
    )
    parser.add_argument(
        "--target-hold-seconds",
        type=float,
        default=0.0,
        help="hold a confirmed target for 0 to 10 seconds before restoration",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    args.server = validate_server_address(args.server)
    args.delta_m = float(getattr(args, "delta_m", DELTA_M))
    mutating = (
        args.same_value_only
        or args.apply_delta
        or args.canonical_zero_delta
    )
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
        expected_ack = SAME_VALUE_ACKNOWLEDGEMENT
    elif args.apply_delta:
        expected_ack = (
            f"SIMULATOR_LIFT_DELTA_{args.delta_m:.3f}M_AND_RESTORE"
        )
    elif args.canonical_zero_delta:
        expected_ack = (
            "SIMULATOR_LIFT_CANONICAL_ZERO_THEN_DELTA_"
            f"{args.delta_m:.3f}M_AND_RESTORE"
        )
    if mutating and args.acknowledgement != expected_ack:
        raise SystemExit(
            "selected action requires --acknowledgement " + expected_ack
        )
    if not mutating and args.acknowledgement:
        raise SystemExit("--acknowledgement requires a mutating action")
    if not 2.0 <= args.settle_timeout <= MAX_SETTLE_TIMEOUT_SECONDS:
        raise SystemExit(
            "--settle-timeout must be in "
            f"[2.0, {MAX_SETTLE_TIMEOUT_SECONDS:.1f}] seconds"
        )
    if not math.isfinite(args.delta_m) or not (
        MIN_DEMO_DELTA_M <= args.delta_m <= MAX_DEMO_DELTA_M
    ):
        raise SystemExit(
            "--delta-m must be finite and in "
            f"[{MIN_DEMO_DELTA_M:.3f}, {MAX_DEMO_DELTA_M:.3f}]m"
        )
    args.target_hold_seconds = float(
        getattr(args, "target_hold_seconds", 0.0)
    )
    if not math.isfinite(args.target_hold_seconds) or not (
        0.0 <= args.target_hold_seconds <= 10.0
    ):
        raise SystemExit(
            "--target-hold-seconds must be finite and in [0, 10] seconds"
        )


def capture_baseline(robot, timeout: float, label: str) -> StableWindow:
    window = capture_stable_window(
        robot,
        sample_count=BASELINE_SAMPLE_COUNT,
        timeout=timeout,
        label=label,
    )
    latest = window.latest
    print(
        f"{label} PASS samples={len(window.samples)} names={latest.names!r} "
        f"stamp_ns={latest.stamp_ns} position={latest.position_m:.9f}m"
    )
    cross_check_get_position(robot, latest, label)
    return window


def exercise(robot, args: argparse.Namespace, safety: SafetyState) -> int:
    delta_m = float(getattr(args, "delta_m", DELTA_M))
    canonical_zero_delta = bool(
        getattr(args, "canonical_zero_delta", False)
    )
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

    observed = capture_baseline(robot, args.settle_timeout, "OBSERVE_BASELINE")
    if not args.same_value_only and not args.apply_delta and not canonical_zero_delta:
        print("OBSERVE_ONLY PASS: no mode or Set RPC was sent")
        return 0

    if canonical_zero_delta:
        # This does not make the observed value a legal target.  It proves only
        # that the Lift is already resting within the reviewed read-side window
        # from which a separately acknowledged literal 0.0 m command is safe.
        require_canonical_zero_window(observed, "OBSERVE_BASELINE")
    else:
        # A slightly negative or otherwise out-of-command-range reading remains
        # useful read-only evidence, but it can never be reflected into the
        # strict command API.  Fail before even entering SDK mode on that
        # baseline unless the explicit canonical-zero action was selected.
        require_legal_restore_target(observed.latest, "OBSERVE_BASELINE")
    print("WRITE_GATES PASS: explicit flags and acknowledgement accepted")
    enter_sdk_mode_idempotently(robot)

    prewrite = capture_baseline(robot, args.settle_timeout, "PREWRITE_BASELINE")
    if (
        abs(prewrite.latest.position_m - observed.latest.position_m)
        > BASELINE_STABILITY_TOLERANCE_M
    ):
        raise RuntimeError(
            "Lift changed during the local SDK-mode gate; "
            f"before={observed.latest.position_m:.9f}m, "
            f"after={prewrite.latest.position_m:.9f}m"
        )
    baseline = prewrite.latest
    if canonical_zero_delta:
        require_canonical_zero_window(prewrite, "PREWRITE_BASELINE")
    else:
        require_legal_restore_target(baseline, "PREWRITE_BASELINE")

    safety.actuator_write_attempted = True
    initial_set_error: BaseException | None = None
    initial_label = "CANONICAL_ZERO" if canonical_zero_delta else "SAME_VALUE"
    initial_target_m = (
        CANONICAL_ZERO_TARGET_M
        if canonical_zero_delta
        else baseline.position_m
    )
    initial_operation = (
        "LiftController.SetLiftPosition(canonical-zero)"
        if canonical_zero_delta
        else "LiftController.SetLiftPosition(same-value)"
    )
    try:
        # canonical-zero always sends the literal legal 0.0 target.  The
        # observed negative micrometre-scale value is never passed here.
        set_lift_position(robot, initial_target_m, initial_operation)
    except BaseException as error:
        initial_set_error = error

    try:
        same_value = wait_for_stable_position(
            robot,
            expected_m=initial_target_m,
            baseline_m=baseline.position_m,
            newer_than_ns=baseline.stamp_ns,
            timeout=args.settle_timeout,
            label=initial_label,
            position_tolerance_m=(
                CANONICAL_ZERO_FEEDBACK_TOLERANCE_M
                if canonical_zero_delta
                else POSITION_TOLERANCE_M
            ),
            stability_tolerance_m=(
                CANONICAL_ZERO_STABILITY_TOLERANCE_M
                if canonical_zero_delta
                else BASELINE_STABILITY_TOLERANCE_M
            ),
        )
        if canonical_zero_delta:
            require_canonical_zero_window(same_value, initial_label)
        cross_check_get_position(
            robot,
            same_value.latest,
            initial_label,
            tolerance_m=(
                CANONICAL_ZERO_FEEDBACK_TOLERANCE_M
                if canonical_zero_delta
                else GET_CROSSCHECK_TOLERANCE_M
            ),
        )
        safety.restoration_confirmed = True
        print(
            f"{initial_label} PASS samples={len(same_value.samples)} "
            f"stamp_ns={same_value.latest.stamp_ns} "
            f"position={same_value.latest.position_m:.9f}m "
            f"command={initial_target_m:.9f}m"
        )
    except BaseException as feedback_error:
        if initial_set_error is not None:
            raise RestoreUnconfirmedError(
                f"{initial_label} Set failed ({initial_set_error}); fresh "
                f"stable feedback also failed ({feedback_error})"
            ) from feedback_error
        raise RestoreUnconfirmedError(
            f"{initial_label} Set began, but fresh stable feedback was not "
            f"confirmed: {feedback_error}"
        ) from feedback_error

    if initial_set_error is not None:
        raise TargetFailedRestoredError(
            f"{initial_label} Set RPC failed, but fresh stable legal "
            f"reference feedback was confirmed: {initial_set_error}"
        ) from initial_set_error
    if args.same_value_only:
        return 0

    if canonical_zero_delta:
        # From this point onward the legal command reference is exactly 0.0 m,
        # regardless of the tiny raw feedback undershoot that admitted the
        # explicit canonical-zero action.
        baseline = LiftJointSample(
            position_m=CANONICAL_ZERO_TARGET_M,
            stamp_ns=same_value.latest.stamp_ns,
            names=same_value.latest.names,
        )

    target_m, direction = choose_delta_target(
        baseline.position_m, delta_m
    )
    direction_name = "up/positive" if direction > 0 else "down/negative"
    print(
        f"TARGET baseline={baseline.position_m:.9f}m "
        f"target={target_m:.9f}m delta={direction * delta_m:+.3f}m "
        f"direction={direction_name}"
    )

    primary_error: BaseException | None = None
    target_attempted = False
    safety.restoration_confirmed = False
    try:
        target_attempted = True
        set_lift_position(
            robot,
            target_m,
            "LiftController.SetLiftPosition(delta)",
        )
        target = wait_for_stable_position(
            robot,
            expected_m=target_m,
            baseline_m=baseline.position_m,
            newer_than_ns=same_value.latest.stamp_ns,
            timeout=args.settle_timeout,
            label="DELTA_TARGET",
            direction=direction,
        )
        cross_check_get_position(robot, target.latest, "DELTA_TARGET")
        print(
            f"DELTA_TARGET PASS samples={len(target.samples)} "
            f"stamp_ns={target.latest.stamp_ns} "
            f"position={target.latest.position_m:.9f}m"
        )
        target_hold_seconds = float(
            getattr(args, "target_hold_seconds", 0.0)
        )
        if target_hold_seconds > 0.0:
            print(
                f"TARGET_HOLD seconds={target_hold_seconds:.1f}; "
                "the single position target remains active"
            )
            time.sleep(target_hold_seconds)
    except BaseException as error:
        primary_error = error
    finally:
        if target_attempted:
            restore_reference_ns = same_value.latest.stamp_ns
            try:
                restore_reference = read_one_joint_sample(
                    robot,
                    timeout=min(2.0, args.settle_timeout),
                )
                restore_reference_ns = max(
                    restore_reference_ns,
                    restore_reference.stamp_ns,
                )
            except BaseException as reference_error:
                print(
                    "warning: pre-restore Lift feedback unavailable; "
                    f"sending the single restore command: {reference_error}",
                    file=sys.stderr,
                )

            try:
                set_lift_position(
                    robot,
                    baseline.position_m,
                    "LiftController.SetLiftPosition(restore)",
                )
                restored = wait_for_stable_position(
                    robot,
                    expected_m=baseline.position_m,
                    baseline_m=baseline.position_m,
                    newer_than_ns=restore_reference_ns,
                    timeout=args.settle_timeout,
                    label="RESTORE",
                    position_tolerance_m=(
                        CANONICAL_ZERO_FEEDBACK_TOLERANCE_M
                        if canonical_zero_delta
                        else POSITION_TOLERANCE_M
                    ),
                    stability_tolerance_m=(
                        CANONICAL_ZERO_STABILITY_TOLERANCE_M
                        if canonical_zero_delta
                        else BASELINE_STABILITY_TOLERANCE_M
                    ),
                )
                if canonical_zero_delta:
                    require_canonical_zero_window(restored, "RESTORE")
                cross_check_get_position(
                    robot,
                    restored.latest,
                    "RESTORE",
                    tolerance_m=(
                        CANONICAL_ZERO_FEEDBACK_TOLERANCE_M
                        if canonical_zero_delta
                        else GET_CROSSCHECK_TOLERANCE_M
                    ),
                )
                safety.restoration_confirmed = True
                print(
                    f"RESTORE PASS samples={len(restored.samples)} "
                    f"stamp_ns={restored.latest.stamp_ns} "
                    f"position={restored.latest.position_m:.9f}m"
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
    if canonical_zero_delta:
        print("CANONICAL_ZERO_DELTA_AND_RESTORE PASS")
    else:
        print("DELTA_AND_RESTORE PASS")
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
            if isinstance(closed, x2robot_sdk.SdkResult) and not closed.is_success:
                raise RuntimeError(f"Robot.close failed: {closed.error}")
    except BaseException as error:
        if safety.actuator_write_attempted:
            if isinstance(error, RestoreUnconfirmedError):
                raise
            if safety.restoration_confirmed:
                if isinstance(error, TargetFailedRestoredError):
                    raise
                raise TargetFailedRestoredError(
                    "failure after a Lift actuator write, with exact baseline "
                    f"restoration confirmed: {error}"
                ) from error
            raise RestoreUnconfirmedError(
                "failure after a Lift actuator write; do not retry or perform "
                f"SDK lifecycle actions (restore_feedback=not_confirmed): {error}"
            ) from error
        raise


def cli() -> int:
    try:
        return main()
    except NoLegalRestoreError as error:
        print(
            f"WRITE_NOT_RUN/NO_LEGAL_RESTORE: {error}",
            file=sys.stderr,
        )
        return 1
    except RestoreUnconfirmedError as error:
        print(f"RESTORE_UNCONFIRMED: {error}", file=sys.stderr)
        return 2
    except TargetFailedRestoredError as error:
        print(f"TARGET_FAILED_RESTORED: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("INTERRUPTED before any Lift actuator write", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(cli())
