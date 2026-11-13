"""Verify desktop arm, gripper and camera feedback without issuing commands."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import threading
import time

import x2robot_sdk


def require(result):
    if not result.is_success:
        raise RuntimeError(f"{result.error} (request_id={result.request_id})")
    return result.data


def finite(values):
    values = [float(value) for value in values]
    if not values or not all(math.isfinite(value) for value in values):
        raise AssertionError(f"empty or non-finite values: {values}")
    return values


def joint_fields(value, expected_names=None, dof=None):
    names = list(value.name)
    positions = finite(value.position)
    if dof is not None and len(positions) != dof:
        raise AssertionError(f"expected {dof} joints, got {len(positions)}: {names}")
    if expected_names is not None and names != expected_names:
        raise AssertionError(f"wrong joint identities/order: {names}")
    if names and len(names) != len(positions):
        raise AssertionError("joint names and positions have different lengths")
    result = {"name": names, "position": positions}
    for field in ("velocity", "effort"):
        values = list(getattr(value, field))
        if values and len(values) != len(positions):
            raise AssertionError(f"{field} has wrong length")
        result[field] = finite(values) if values else []
    return result


def pose_fields(value):
    if hasattr(value, "x"):
        xyz = [value.x, value.y, value.z]
        xyzw = [value.qx, value.qy, value.qz, value.qw]
        frame = value.frame_id
    else:
        pose = value.pose
        xyz = [pose.position.x, pose.position.y, pose.position.z]
        xyzw = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        frame = value.header.frame_id
    xyz, xyzw = finite(xyz), finite(xyzw)
    if abs(math.sqrt(sum(x * x for x in xyzw)) - 1.0) > 0.01:
        raise AssertionError(f"invalid quaternion: {xyzw}")
    return {"frame_id": frame, "position_xyz_m": xyz, "quaternion_xyzw": xyzw}


def position_fields(value):
    return {"position": finite([value.position])[0]}


def wrench_fields(value):
    return {"frame_id": value.frame_id, "force": finite(value.force), "torque": finite(value.torque)}


def motor_fields(value):
    # Native arm bindings use a list of dictionaries; generic services use
    # generated message fields. Do not silently accept an empty status list.
    motors = list(value.motors if hasattr(value, "motors") else value.joint_motor_status)
    if not motors:
        raise AssertionError("empty motor status")
    return {"all_joints_healthy": bool(value.all_joints_healthy), "motors": [str(m) for m in motors]}


def frame_fields(frame):
    if frame.size <= 0 or len(frame.data) != frame.size:
        raise AssertionError("empty/truncated camera payload")
    return {"format": frame.format, "bytes": frame.size, "seq": frame.seq, "stamp_ns": frame.stamp_ns}


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def collect_stream(open_stream, validate, samples, native):
    stream = require(open_stream()) if native else open_stream()
    count = 0
    first_at = last_at = None
    first = last = None
    try:
        iterator = None if native else iter(stream)
        for _ in range(samples):
            result = stream.read(deadline_ms=3000) if native else next(iterator)
            fields = validate(require(result))
            now = time.monotonic()
            if first_at is None:
                first_at, first = now, fields
            last_at, last = now, fields
            count += 1
    finally:
        close_stream(stream)
    elapsed = last_at - first_at
    return {"samples": count, "arrival_hz": (count - 1) / elapsed if elapsed > 0 else None,
            "first": first, "last": last}


def collect_camera(open_stream, samples, deadline_ms=5000):
    done = threading.Event()
    frames, arrivals, errors = [], [], []

    def receive(result):
        if done.is_set():
            return
        try:
            frames.append(frame_fields(require(result)))
            arrivals.append(time.monotonic())
            if len(frames) >= samples:
                done.set()
        except Exception as error:
            errors.append(error)
            done.set()

    stream = require(open_stream(receive, deadline_ms=deadline_ms))
    try:
        if not done.wait(max(5.0, samples / 5.0)):
            raise TimeoutError(f"camera delivered {len(frames)}/{samples} samples")
        if errors:
            raise errors[0]
    finally:
        close_stream(stream)
    stamps = [frame["stamp_ns"] for frame in frames]
    if stamps[-1] <= stamps[0]:
        raise AssertionError("camera timestamps did not advance")
    elapsed = arrivals[-1] - arrivals[0]
    return {"samples": len(frames), "arrival_hz": (len(frames) - 1) / elapsed if elapsed > 0 else None,
            "first": frames[0], "last": frames[-1]}


def cases(robot, samples, camera_deadline_ms=5000):
    """Explicit allow-list: never discover or execute arbitrary robot services."""
    result = []

    def single(label, call, validate):
        result.append((label, lambda: validate(require(call()))))

    def stream(label, call, validate, native=False):
        result.append((label, lambda: collect_stream(call, validate, samples, native)))

    for side in ("left", "right"):
        arm = getattr(robot, f"{side}_arm_controller")
        joints = lambda value: joint_fields(value, dof=6)
        for suffix, validator in (("joint_states", joints), ("end_pose", pose_fields),
                                  ("wrench_ext_world", wrench_fields), ("wrench_ext_local", wrench_fields)):
            method = getattr(arm, f"get_{suffix}")
            single(f"slave_{side}.{suffix}", lambda method=method: method(deadline_ms=3000), validator)
            method = getattr(arm, f"get_{suffix}_stream")
            stream(f"slave_{side}.{suffix}_stream", lambda method=method: method(deadline_ms=3000), validator, True)
        single(f"slave_{side}.motor_status", lambda arm=arm: arm.get_motor_status(deadline_ms=3000), motor_fields)

        master = getattr(robot, f"master_{side}_arm")
        names = [f"master_{side}_arm_joint{i}" for i in range(1, 7)]
        joints = lambda value, names=names: joint_fields(value, expected_names=names, dof=6)
        single(f"master_{side}.control_mode", lambda master=master: master.get_control_mode(timeout=3.0),
               lambda value: {"mode": str(value.mode)})
        for suffix, validator in (("joint_states", joints), ("end_pose", pose_fields), ("gripper_position", position_fields)):
            method = getattr(master, f"get_{suffix}")
            single(f"master_{side}.{suffix}", lambda method=method: method(timeout=3.0), validator)
        for suffix, validator in (("joint_states", joints), ("end_pose", pose_fields),
                                  ("gripper_state", position_fields), ("gripper_joint_states", joint_fields)):
            method = getattr(master, f"get_{suffix}_stream")
            stream(f"master_{side}.{suffix}_stream", lambda method=method: method(timeout=max(5.0, samples / 5.0)), validator)

        gripper = getattr(robot, f"{side}_gripper")
        single(f"gripper_{side}.position", lambda gripper=gripper: gripper.get_position(timeout=3.0), position_fields)
        single(f"gripper_{side}.motor_status", lambda gripper=gripper: gripper.get_motor_status(timeout=3.0), motor_fields)
        for suffix, validator in (("position", position_fields), ("joint_states", joint_fields)):
            method = getattr(gripper, f"get_{suffix}_stream")
            stream(f"gripper_{side}.{suffix}_stream", lambda method=method: method(timeout=max(5.0, samples / 5.0)), validator)

    cameras = (("front_rgb", robot.head_camera.get_rgb_image, robot.head_camera.get_rgb_video_stream),
               ("front_depth", robot.head_camera.get_depth_image, robot.head_camera.get_depth_video_stream),
               ("left_wrist", robot.left_arm_camera.get_raw_image, robot.left_arm_camera.get_video_stream),
               ("right_wrist", robot.right_arm_camera.get_raw_image, robot.right_arm_camera.get_video_stream))
    for name, getter, streamer in cameras:
        single(f"camera_{name}.image", lambda getter=getter: getter(deadline_ms=camera_deadline_ms), frame_fields)
        result.append((f"camera_{name}.stream", lambda streamer=streamer: collect_camera(streamer, samples, camera_deadline_ms)))
    return result


def verify(robot, samples=10, camera_deadline_ms=5000):
    if samples < 2:
        raise ValueError("samples must be at least 2")
    results = []
    for name, check in cases(robot, samples, camera_deadline_ms):
        started = time.monotonic()
        try:
            entry = {"name": name, "status": "PASS", "data": check()}
        except Exception as error:
            entry = {"name": name, "status": "FAIL", "error": str(error)}
        entry["elapsed_s"] = round(time.monotonic() - started, 4)
        results.append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    failed = sum(entry["status"] == "FAIL" for entry in results)
    summary = {"model": "desktop", "camera_deadline_ms": camera_deadline_ms,
               "total": len(results), "passed": len(results) - failed,
               "failed": failed, "write_calls": 0, "results": results}
    print(f"desktop read-only: total={len(results)} pass={summary['passed']} fail={failed}", flush=True)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--camera-deadline-ms", type=int, default=5000, help="camera startup timeout; does not throttle streams")
    parser.add_argument("--output", type=Path, help="save JSON validation report")
    parser.add_argument("--fail-on-error", action="store_true",
                        help="return non-zero after all checks; default keeps the batch running")
    args = parser.parse_args(argv)
    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if args.camera_deadline_ms <= 0:
        parser.error("--camera-deadline-ms must be positive")
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True))
    try:
        report = verify(robot, args.samples, args.camera_deadline_ms)
        report["server"] = args.server
        if args.output:
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return 1 if args.fail_on_error and report["failed"] else 0
    finally:
        require(robot.close())


if __name__ == "__main__":
    raise SystemExit(main())
