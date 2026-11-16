#!/usr/bin/env python3
"""Guarded typed-SDK chassis velocity demo for the EX001 ManaEnv simulator.

The default mode is read-only and only observes fresh, stationary odometry.
``--apply-forward`` uses the ordinary SDK Client API to command a fixed,
bounded forward movement.  It requires explicit write/simulator gates and the
exact acknowledgement below.  Before enabling it, externally prove that the
active Server uses ``runtime_profile=simulator`` and that ``/sdk2_server`` is
the only command publisher while ``/sx001_ros_communicator`` owns the matching
``/chassis/cmd_vel`` subscriber and odometry publisher.

The Server lease is 300 ms and ManaEnv's command timeout is 250 ms.  This
client refreshes every 80 ms and aborts if the observed start-to-start interval
exceeds 150 ms.  Every mutating run sends
an explicit zero first and in ``finally``.  A successful zero RPC is not enough:
three fresh, consecutive, near-zero odometry samples are required.  Do not run
while any simulator control attempt is under an open stop/restore hold.  Yaw
velocity is intentionally not offered: live evidence showed command delivery
without corresponding yaw or wheel-velocity response, so it remains
unsupported rather than being presented as a working control path.

The current ManaEnv asset responds much more strongly than an earlier scene:
``0.20 m/s`` for ``2.50 s`` exceeded ``0.30 m`` before the lease completed.
The reviewed demonstration therefore uses ``0.08 m/s`` for ``1.00 s`` and a
strict ``0.15 m`` maximum observed excursion.  The symmetric reverse lease and
fresh stopped-state checks are unchanged.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable

import x2robot_sdk
from x2robot.sdk import (
    ChassisControlMode,
    ChassisControlModeParam,
    ChassisVelocity,
    RobotModeParam,
    RobotWorkMode,
)
from x2robot.sdk.v2 import RobotModelType


LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}

FORWARD_VELOCITY_MPS = 0.08
FORWARD_DURATION_SECONDS = 1.0
REFRESH_PERIOD_SECONDS = 0.08
MAX_REFRESH_INTERVAL_SECONDS = 0.15

BASELINE_SAMPLE_COUNT = 3
STOP_SAMPLE_COUNT = 3
LINEAR_STOP_TOLERANCE_MPS = 0.02
ANGULAR_STOP_TOLERANCE_RADPS = 0.03
MIN_VISIBLE_FORWARD_M = 0.05
MAX_ALLOWED_FORWARD_M = 0.15
MAX_LATERAL_DRIFT_M = 0.05
MAX_YAW_DRIFT_RAD = 0.10
MAX_RESTORE_POSITION_ERROR_M = 0.05
MAX_RESTORE_YAW_ERROR_RAD = 0.10

FORWARD_ACKNOWLEDGEMENT = "SIMULATOR_CHASSIS_FORWARD_0.08MPS_1.00S_WITH_LEASE_AND_STOP"
FORWARD_RESTORE_ACKNOWLEDGEMENT = (
    "SIMULATOR_CHASSIS_FORWARD_0.08MPS_1.00S_REVERSE_RESTORE_AND_STOP"
)


@dataclass(frozen=True)
class OdometrySample:
    x_m: float
    y_m: float
    yaw_rad: float
    linear_x_mps: float
    linear_y_mps: float
    angular_z_radps: float
    stamp_ns: int


@dataclass
class SafetyState:
    velocity_write_attempted: bool = False
    stop_feedback_confirmed: bool = False


class OdometryEnvelopeMonitor:
    """Observe typed odometry concurrently without delaying lease refreshes."""

    def __init__(self, robot, origin: OdometrySample, timeout: float):
        self._origin = origin
        self._stream = robot.chassis.get_odometry_stream(timeout=timeout)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._error: BaseException | None = None
        self._sample_count = 0
        self._last_stamp_ns = origin.stamp_ns
        self._thread = threading.Thread(
            target=self._run,
            name="sdk2-chassis-odom-envelope",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        try:
            iterator = iter(self._stream)
            while not self._stop.is_set():
                try:
                    value = require(
                        next(iterator), "ChassisController.GetOdometryStream(envelope)"
                    )
                except StopIteration as error:
                    if self._stop.is_set():
                        return
                    raise RuntimeError("odometry envelope stream ended") from error
                sample = parse_odometry(value)
                with self._lock:
                    if sample.stamp_ns <= self._last_stamp_ns:
                        raise RuntimeError(
                            "odometry envelope timestamp did not advance; "
                            f"previous={self._last_stamp_ns}, "
                            f"current={sample.stamp_ns}"
                        )
                    validate_maximum_excursion(self._origin, sample)
                    self._last_stamp_ns = sample.stamp_ns
                    self._sample_count += 1
        except BaseException as error:
            with self._lock:
                self._error = error

    def check(self, require_sample: bool = False) -> None:
        with self._lock:
            error = self._error
            sample_count = self._sample_count
        if error is not None:
            raise RuntimeError(f"odometry envelope monitor failed: {error}") from error
        if require_sample and sample_count == 0:
            raise RuntimeError("odometry envelope monitor received no fresh sample")

    def close(self) -> None:
        self._stop.set()
        close_stream(self._stream)
        self._thread.join(timeout=0.20)


class StopUnconfirmedError(RuntimeError):
    """A velocity RPC began, but a fresh stopped state was not confirmed."""


class MotionFailedStoppedError(RuntimeError):
    """The movement did not pass, but a fresh stopped state was confirmed."""


def require(result, operation: str):
    if not isinstance(result, x2robot_sdk.SdkResult):
        raise TypeError(f"{operation}: expected SdkResult, got {type(result).__name__}")
    if not result.is_success:
        error = result.error or {}
        name = error.get("error_name", "UNKNOWN")
        message = error.get("error_message", "SDK call failed")
        request_id = result.request_id or error.get("request_id") or "-"
        raise RuntimeError(f"{operation}: {name}: {message} (request_id={request_id})")
    return result.data


def close_stream(stream) -> None:
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close is not None:
        close()


def parse_odometry(value) -> OdometrySample:
    header = getattr(value, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise RuntimeError(
            f"odometry has an invalid timestamp: sec={sec}, nanosec={nanosec}"
        )
    stamp_ns = sec * 1_000_000_000 + nanosec
    if stamp_ns <= 0:
        raise RuntimeError("odometry has no positive simulation timestamp")

    try:
        position = value.pose.pose.position
        twist = value.twist.twist
        orientation = value.pose.pose.orientation
        values = (
            float(position.x),
            float(position.y),
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.angular.z),
        )
    except (AttributeError, TypeError, ValueError) as error:
        raise RuntimeError(
            "odometry is missing the expected pose/twist fields"
        ) from error
    if not all(math.isfinite(item) for item in values):
        raise RuntimeError(f"odometry contains non-finite values: {values!r}")
    x_m, y_m, qx, qy, qz, qw, linear_x, linear_y, angular_z = values
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not 0.95 <= quaternion_norm <= 1.05:
        raise RuntimeError(
            f"odometry orientation quaternion norm is invalid: {quaternion_norm}"
        )
    yaw_rad = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return OdometrySample(
        x_m=x_m,
        y_m=y_m,
        yaw_rad=yaw_rad,
        linear_x_mps=linear_x,
        linear_y_mps=linear_y,
        angular_z_radps=angular_z,
        stamp_ns=stamp_ns,
    )


def is_stopped(sample: OdometrySample) -> bool:
    return (
        math.hypot(sample.linear_x_mps, sample.linear_y_mps)
        <= LINEAR_STOP_TOLERANCE_MPS
        and abs(sample.angular_z_radps) <= ANGULAR_STOP_TOLERANCE_RADPS
    )


def capture_odometry(
    robot,
    *,
    sample_count: int,
    timeout: float,
    label: str,
    newer_than_ns: int = 0,
    require_stopped: bool = False,
) -> tuple[OdometrySample, ...]:
    stream = robot.chassis.get_odometry_stream(timeout=timeout)
    samples: list[OdometrySample] = []
    previous_stamp_ns = newer_than_ns
    try:
        iterator = iter(stream)
        while len(samples) < sample_count:
            try:
                value = require(next(iterator), "ChassisController.GetOdometryStream")
            except StopIteration as error:
                raise RuntimeError(
                    f"{label}: odometry stream ended after "
                    f"{len(samples)}/{sample_count} samples"
                ) from error
            sample = parse_odometry(value)
            if sample.stamp_ns <= previous_stamp_ns:
                raise RuntimeError(
                    f"{label}: odometry timestamp did not advance; "
                    f"previous={previous_stamp_ns}, current={sample.stamp_ns}"
                )
            previous_stamp_ns = sample.stamp_ns
            if require_stopped and not is_stopped(sample):
                samples.clear()
                continue
            samples.append(sample)
    finally:
        close_stream(stream)
    return tuple(samples)


def observe_stationary_baseline(robot, timeout: float) -> OdometrySample:
    samples = capture_odometry(
        robot,
        sample_count=BASELINE_SAMPLE_COUNT,
        timeout=timeout,
        label="OBSERVE_BASELINE",
        require_stopped=True,
    )
    latest = samples[-1]
    print(
        "OBSERVE_BASELINE PASS "
        f"samples={len(samples)} stamp_ns={latest.stamp_ns} "
        f"pose=({latest.x_m:.6f},{latest.y_m:.6f},{latest.yaw_rad:.6f}) "
        f"velocity=({latest.linear_x_mps:.6f},"
        f"{latest.linear_y_mps:.6f},{latest.angular_z_radps:.6f})"
    )
    return latest


def set_velocity(robot, forward_mps: float, operation: str) -> None:
    if not math.isfinite(forward_mps) or abs(forward_mps) > FORWARD_VELOCITY_MPS:
        raise RuntimeError(f"{operation}: refusing forward velocity {forward_mps!r}")
    require(
        robot.chassis.set_velocity(
            ChassisVelocity(vel_x=forward_mps, vel_y=0.0, vel_yaw=0.0),
            timeout=3.0,
        ),
        operation,
    )


def enter_velocity_mode(robot) -> None:
    require(
        enter_sdk_mode(robot),
        "System.SetWorkMode(SDK)",
    )
    require(
        robot.chassis.set_control_mode(
            ChassisControlModeParam(mode=ChassisControlMode.VELOCITY),
            timeout=3.0,
        ),
        "ChassisController.SetControlMode(VELOCITY)",
    )
    actual = require(
        robot.chassis.get_control_mode(timeout=3.0),
        "ChassisController.GetControlMode",
    )
    if int(actual.mode) != int(ChassisControlMode.VELOCITY):
        raise RuntimeError(f"chassis mode round-trip mismatch: {actual.mode!r}")
    print("CONTROL_MODE PASS work_mode=SDK chassis_mode=VELOCITY")


def confirm_stopped_after_zero(
    robot,
    *,
    newer_than_ns: int,
    timeout: float,
    label: str,
) -> OdometrySample:
    samples = capture_odometry(
        robot,
        sample_count=STOP_SAMPLE_COUNT,
        timeout=timeout,
        label=label,
        newer_than_ns=newer_than_ns,
        require_stopped=True,
    )
    latest = samples[-1]
    print(
        f"{label} PASS samples={len(samples)} stamp_ns={latest.stamp_ns} "
        f"velocity=({latest.linear_x_mps:.6f},"
        f"{latest.linear_y_mps:.6f},{latest.angular_z_radps:.6f})"
    )
    return latest


def run_velocity_lease(
    robot,
    *,
    forward_mps: float,
    label: str,
    envelope_check: Callable[[bool], None],
    duration: float = FORWARD_DURATION_SECONDS,
    refresh_period: float = REFRESH_PERIOD_SECONDS,
    max_refresh_interval: float = MAX_REFRESH_INTERVAL_SECONDS,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    start = clock()
    deadline = start + duration
    next_send = start
    previous_send: float | None = None
    sends = 0
    while True:
        envelope_check(False)
        now = clock()
        if now >= deadline:
            break
        if now < next_send:
            sleep(next_send - now)
            now = clock()
        if now >= deadline:
            break
        if previous_send is not None:
            interval = now - previous_send
            if interval > max_refresh_interval:
                raise RuntimeError(
                    "velocity refresh deadline missed; "
                    f"interval={interval:.6f}s limit={max_refresh_interval:.6f}s"
                )
        set_velocity(
            robot,
            forward_mps,
            f"ChassisController.SetVelocity({label} #{sends + 1})",
        )
        envelope_check(False)
        previous_send = now
        sends += 1
        next_send += refresh_period
    if sends < 2:
        raise RuntimeError(f"velocity lease sent only {sends} command(s)")
    envelope_check(True)
    print(
        f"LEASE_REFRESH PASS sends={sends} period={refresh_period:.3f}s "
        f"duration={duration:.2f}s forward_velocity={forward_mps:+.2f}m/s "
        f"phase={label}"
    )
    return sends


def displacement_m(start: OdometrySample, end: OdometrySample) -> float:
    return math.hypot(end.x_m - start.x_m, end.y_m - start.y_m)


def validate_maximum_excursion(origin: OdometrySample, sample: OdometrySample) -> float:
    excursion = displacement_m(origin, sample)
    if excursion > MAX_ALLOWED_FORWARD_M:
        raise RuntimeError(
            f"maximum chassis excursion exceeded: {excursion:.6f}m > "
            f"{MAX_ALLOWED_FORWARD_M:.2f}m"
        )
    return excursion


def yaw_delta_rad(start: OdometrySample, end: OdometrySample) -> float:
    return math.atan2(
        math.sin(end.yaw_rad - start.yaw_rad),
        math.cos(end.yaw_rad - start.yaw_rad),
    )


def motion_in_start_frame(
    start: OdometrySample, end: OdometrySample
) -> tuple[float, float]:
    dx = end.x_m - start.x_m
    dy = end.y_m - start.y_m
    cosine = math.cos(start.yaw_rad)
    sine = math.sin(start.yaw_rad)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def validate_forward_target(start: OdometrySample, target: OdometrySample) -> float:
    forward_m, lateral_m = motion_in_start_frame(start, target)
    yaw_drift = abs(yaw_delta_rad(start, target))
    if not MIN_VISIBLE_FORWARD_M <= forward_m <= MAX_ALLOWED_FORWARD_M:
        raise RuntimeError(
            f"forward target change {forward_m:.6f}m is outside "
            f"[{MIN_VISIBLE_FORWARD_M:.2f}, {MAX_ALLOWED_FORWARD_M:.2f}]m"
        )
    if abs(lateral_m) > MAX_LATERAL_DRIFT_M or yaw_drift > MAX_YAW_DRIFT_RAD:
        raise RuntimeError(
            "forward-only target exceeded drift limits: "
            f"lateral={lateral_m:.6f}m, yaw={yaw_drift:.6f}rad"
        )
    print(
        f"FORWARD_TARGET PASS forward={forward_m:.6f}m "
        f"lateral={lateral_m:.6f}m yaw_drift={yaw_drift:.6f}rad"
    )
    return forward_m


def exercise(robot, args: argparse.Namespace, safety: SafetyState) -> int:
    model = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType")
    if int(model.model_type) != int(RobotModelType.QUANTA_X1_PRO):
        raise RuntimeError(f"expected EX001/QUANTA_X1_PRO, got {model.model_type!r}")
    print(f"MODEL PASS model={model.model_type!r} transport={robot.transport}")

    observed = observe_stationary_baseline(robot, args.settle_timeout)
    if not args.apply_forward:
        print("OBSERVE_ONLY PASS: no lifecycle, mode, or Set RPC was sent")
        return 0

    print("WRITE_GATES PASS: explicit flags and acknowledgement accepted")
    enter_velocity_mode(robot)

    safety.velocity_write_attempted = True
    try:
        set_velocity(robot, 0.0, "ChassisController.SetVelocity(initial-zero)")
        initial_stopped = confirm_stopped_after_zero(
            robot,
            newer_than_ns=observed.stamp_ns,
            timeout=args.settle_timeout,
            label="INITIAL_STOP",
        )
        safety.stop_feedback_confirmed = True
    except BaseException as error:
        raise StopUnconfirmedError(
            f"initial zero began, but fresh stopped odometry was not confirmed: {error}"
        ) from error

    envelope = OdometryEnvelopeMonitor(
        robot,
        initial_stopped,
        timeout=min(30.0, max(15.0, args.settle_timeout * 2.0)),
    )
    envelope.start()
    primary_error: BaseException | None = None
    target_stopped: OdometrySample | None = None
    final_stopped: OdometrySample | None = None
    safety.stop_feedback_confirmed = False
    try:
        run_velocity_lease(
            robot,
            forward_mps=FORWARD_VELOCITY_MPS,
            label="forward",
            envelope_check=envelope.check,
        )
        if args.restore_position:
            set_velocity(robot, 0.0, "ChassisController.SetVelocity(target-zero)")
            target_stopped = confirm_stopped_after_zero(
                robot,
                newer_than_ns=initial_stopped.stamp_ns,
                timeout=args.settle_timeout,
                label="FORWARD_TARGET_STOP",
            )
            validate_forward_target(initial_stopped, target_stopped)
            run_velocity_lease(
                robot,
                forward_mps=-FORWARD_VELOCITY_MPS,
                label="reverse-position-restore",
                envelope_check=envelope.check,
            )
    except BaseException as error:
        primary_error = error
    finally:
        try:
            try:
                set_velocity(robot, 0.0, "ChassisController.SetVelocity(final-zero)")
                final_stopped = confirm_stopped_after_zero(
                    robot,
                    newer_than_ns=(
                        target_stopped.stamp_ns
                        if target_stopped is not None
                        else initial_stopped.stamp_ns
                    ),
                    timeout=args.settle_timeout,
                    label="FINAL_STOP",
                )
                safety.stop_feedback_confirmed = True
            except BaseException as stop_error:
                if primary_error is not None:
                    raise StopUnconfirmedError(
                        f"motion failed ({primary_error}); final zero/stop "
                        f"confirmation also failed ({stop_error})"
                    ) from stop_error
                raise StopUnconfirmedError(
                    f"final zero began, but fresh stopped odometry was not confirmed: "
                    f"{stop_error}"
                ) from stop_error
            envelope.check(True)
        finally:
            envelope.close()

    if primary_error is not None:
        raise MotionFailedStoppedError(
            "motion lease failed, but final stopped odometry was confirmed: "
            f"{primary_error}"
        ) from primary_error
    assert final_stopped is not None
    if args.restore_position:
        restore_error = abs(yaw_delta_rad(initial_stopped, final_stopped))
        position_error = displacement_m(initial_stopped, final_stopped)
        if (
            restore_error > MAX_RESTORE_YAW_ERROR_RAD
            or position_error > MAX_RESTORE_POSITION_ERROR_M
        ):
            raise MotionFailedStoppedError(
                "fresh final stop was confirmed, but reverse restoration was "
                f"outside tolerance: yaw_error={restore_error:.6f}rad, "
                f"position_error={position_error:.6f}m"
            )
        print(
            f"POSITION_RESTORE PASS position_error={position_error:.6f}m "
            f"yaw_error={restore_error:.6f}rad"
        )
    else:
        try:
            validate_forward_target(initial_stopped, final_stopped)
        except RuntimeError as error:
            raise MotionFailedStoppedError(
                f"fresh final stop was confirmed, but forward target failed: {error}"
            ) from error
    print("FORWARD_MOTION_AND_STOP PASS")
    return 0


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
    parser.add_argument("--server", default="127.0.0.1:15551")
    parser.add_argument(
        "--apply-forward",
        action="store_true",
        help="run the fixed +0.20m/s, 2.50s forward lease and stop",
    )
    parser.add_argument(
        "--restore-position",
        action="store_true",
        help="after the target stop, run the symmetric reverse lease",
    )
    parser.add_argument("--allow-write", action="store_true")
    parser.add_argument("--confirm-simulator-only", action="store_true")
    parser.add_argument("--acknowledgement", default="")
    parser.add_argument("--settle-timeout", type=float, default=8.0)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    args.server = validate_server_address(args.server)
    if args.apply_forward:
        if not args.allow_write:
            raise SystemExit("--apply-forward requires --allow-write")
        if not args.confirm_simulator_only:
            raise SystemExit("--apply-forward requires --confirm-simulator-only")
        expected_ack = (
            FORWARD_RESTORE_ACKNOWLEDGEMENT
            if args.restore_position
            else FORWARD_ACKNOWLEDGEMENT
        )
        if args.acknowledgement != expected_ack:
            raise SystemExit(
                "selected forward action requires --acknowledgement " + expected_ack
            )
    elif args.restore_position:
        raise SystemExit("--restore-position requires --apply-forward")
    elif args.allow_write or args.confirm_simulator_only or args.acknowledgement:
        raise SystemExit("write gates/acknowledgement require --apply-forward")
    if not 2.0 <= args.settle_timeout <= 30.0:
        raise SystemExit("--settle-timeout must be in [2.0, 30.0] seconds")


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

def main() -> int:
    args = parse_args()
    validate_args(args)
    safety = SafetyState()
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        try:
            return exercise(robot, args, safety)
        finally:
            restore_idle_mode(robot)
            require(robot.close(), "Robot.close")
    except BaseException as error:
        if safety.velocity_write_attempted:
            if isinstance(error, StopUnconfirmedError):
                raise
            if safety.stop_feedback_confirmed:
                if isinstance(error, MotionFailedStoppedError):
                    raise
                raise MotionFailedStoppedError(
                    "failure after velocity write, with fresh stopped odometry "
                    f"confirmed: {error}"
                ) from error
            raise StopUnconfirmedError(
                "failure after velocity write; explicit zero plus fresh stopped "
                f"odometry were not confirmed: {error}"
            ) from error
        raise


def cli() -> int:
    try:
        return main()
    except StopUnconfirmedError as error:
        print(f"STOP_UNCONFIRMED: {error}", file=sys.stderr)
        return 2
    except MotionFailedStoppedError as error:
        print(f"MOTION_FAILED_STOPPED: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("INTERRUPTED before any velocity write", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(cli())
