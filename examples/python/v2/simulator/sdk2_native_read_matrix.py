#!/usr/bin/env python3
"""Bounded, strictly read-only typed Client matrix for SDK2 ManaEnv-native.

This program sends no mode, heartbeat, lifecycle, or actuator Set RPC.  It is
an acceptance harness for the ordinary SDK Client contract, not a simulator
adapter.  Run it only through a loopback SSH tunnel after endpoint ownership
has been proved externally.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import threading
import time
from collections import deque
from collections.abc import Iterable

import x2robot_sdk
from x2robot.sdk.v2 import RobotModelType


LOOPBACK = {"127.0.0.1", "localhost", "::1"}
EXPECTED_JOINTS = {
    "Head": ("head_pitch_joint", "head_yaw_joint"),
    "LeftArm": tuple(f"left_arm_joint{index}" for index in range(1, 7)),
    "RightArm": tuple(f"right_arm_joint{index}" for index in range(1, 7)),
    "LeftGripper": ("left_arm_gripper",),
    "RightGripper": ("right_arm_gripper",),
    "Lift": ("lift_joint",),
}
EXPECTED_ARM_END_POSE_FRAME = "lift_link"
EXPECTED_TF_ANCHOR = ("map", "odom")
EXPECTED_ODOM_CHILD_FRAME = "base_link"
EXPECTED_FRAMES = {
    "Chassis.GetOdometry": "odom",
    "Imu.GetChassisImu": "imu_link",
    "Radar.GetLaserScan": "a_d_laser",
    "DepthPoints.GetChassisDepthPoints": "camera_chassis_front_depth_optical_frame",
}
EXPECTED_CAMERA_FORMATS = {
    "HeadCamera.GetRGBImage": "jpeg",
    "HeadCamera.GetDepthImage": "png",
    "LeftArmCamera.GetRawImage": "jpeg",
    "RightArmCamera.GetRawImage": "jpeg",
}
QUATERNION_NORM_TOLERANCE = 1e-3
MAX_STREAM_CATCHUP_SAMPLES = 16
MAX_CAMERA_CALLBACK_SAMPLES = 64
JOINT_POSITION_CROSSCHECK_TOLERANCE = 0.02
JOINT_VELOCITY_CROSSCHECK_TOLERANCE = 0.1
JOINT_EFFORT_CROSSCHECK_TOLERANCE = 0.1
POSE_POSITION_CROSSCHECK_TOLERANCE = 0.02
POSE_ORIENTATION_CROSSCHECK_TOLERANCE = 0.02
ODOMETRY_CROSSCHECK_TOLERANCE = 0.05
IMU_CROSSCHECK_TOLERANCE = 0.05
SCAN_CROSSCHECK_TOLERANCE = 0.01
CLOUD_CROSSCHECK_TOLERANCE = 0.02


def endpoint(value: str) -> str:
    text = value.strip()
    for prefix in ("grpc://", "x2://"):
        if text.startswith(prefix):
            text = text[len(prefix) :]
    host, separator, port_text = text.rpartition(":")
    if not separator or host.strip("[]").lower() not in LOOPBACK:
        raise argparse.ArgumentTypeError("server must be a loopback host:port")
    try:
        port = int(port_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("server port must be numeric") from error
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("server port must be in 1..65535")
    return text


def require(result, operation: str):
    if not isinstance(result, x2robot_sdk.SdkResult):
        raise TypeError(f"{operation}: expected SdkResult, got {type(result).__name__}")
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation}: {error.get('error_name', 'UNKNOWN')}")
    return result.data


def stamp_ns(value) -> int:
    direct = getattr(value, "stamp_ns", None)
    if direct is not None:
        return int(direct)
    header = getattr(value, "header", None)
    stamp = getattr(header, "stamp", None)
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    return sec * 1_000_000_000 + nanosec


def finite(values: Iterable[float], label: str) -> None:
    checked = [float(value) for value in values]
    if not checked or not all(math.isfinite(value) for value in checked):
        raise RuntimeError(f"{label}: empty or non-finite values")


def close_stream(stream, operation: str) -> None:
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close is None:
        raise RuntimeError(f"{operation}: stream has no close/cancel method")
    result = close()
    if isinstance(result, x2robot_sdk.SdkResult):
        require(result, f"{operation}.close")


def assert_close_values(
    left: Iterable[float],
    right: Iterable[float],
    tolerance: float,
    label: str,
) -> None:
    left_values = tuple(float(item) for item in left)
    right_values = tuple(float(item) for item in right)
    if len(left_values) != len(right_values):
        raise RuntimeError(
            f"{label}: shape mismatch {len(left_values)} != {len(right_values)}"
        )
    difference = max(
        (abs(left_item - right_item) for left_item, right_item in zip(left_values, right_values)),
        default=0.0,
    )
    if difference > tolerance:
        raise RuntimeError(
            f"{label}: max difference {difference:.9g} exceeds {tolerance:.9g}"
        )


def assert_quaternion_close(
    left: Iterable[float], right: Iterable[float], tolerance: float, label: str
) -> None:
    left_values = tuple(float(item) for item in left)
    right_values = tuple(float(item) for item in right)
    if len(left_values) != 4 or len(right_values) != 4:
        raise RuntimeError(f"{label}: quaternion shape mismatch")
    # q and -q encode the same rotation.
    direct = max(abs(a - b) for a, b in zip(left_values, right_values))
    negated = max(abs(a + b) for a, b in zip(left_values, right_values))
    if min(direct, negated) > tolerance:
        raise RuntimeError(
            f"{label}: quaternion difference {min(direct, negated):.9g} "
            f"exceeds {tolerance:.9g}"
        )


def check_joint(
    value,
    label: str,
    expected_names: tuple[str, ...],
    expected_frame: str | None = None,
) -> dict:
    names = tuple(str(name) for name in value.name)
    positions = [float(position) for position in value.position]
    if names != expected_names or len(positions) != len(expected_names):
        raise RuntimeError(
            f"{label}: identity/order {names!r} != {expected_names!r}"
        )
    finite(positions, label)
    dynamics: dict[str, tuple[float, ...]] = {}
    for field_name in ("velocity", "effort"):
        field_values = tuple(
            float(item) for item in getattr(value, field_name, ())
        )
        if len(field_values) != len(expected_names):
            raise RuntimeError(f"{label}: invalid {field_name} length")
        finite(field_values, f"{label}.{field_name}")
        dynamics[field_name] = field_values
    timestamp = stamp_ns(value)
    if timestamp <= 0:
        raise RuntimeError(f"{label}: missing positive source timestamp")
    header = getattr(value, "header", None)
    frame_id = str(
        getattr(value, "frame_id", getattr(header, "frame_id", ""))
    )
    if expected_frame is not None and frame_id != expected_frame:
        raise RuntimeError(
            f"{label}: frame_id {frame_id!r} != {expected_frame!r}"
        )
    return {
        "effort": dynamics["effort"],
        "frame_id": frame_id,
        "names": names,
        "positions": tuple(positions),
        "stamp_ns": timestamp,
        "velocity": dynamics["velocity"],
    }


def check_pose(value, label: str, expected_frame: str) -> dict:
    pose = value.pose
    if isinstance(pose.position, (list, tuple)):
        position = tuple(pose.position)
        orientation = tuple(pose.orientation)
    else:
        position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
    if len(position) != 3 or len(orientation) != 4:
        raise RuntimeError(f"{label}: invalid pose shape")
    numbers = position + orientation
    finite(numbers, label)
    quaternion_norm = math.sqrt(sum(float(item) ** 2 for item in numbers[3:]))
    if abs(quaternion_norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise RuntimeError(f"{label}: invalid quaternion norm")
    timestamp = stamp_ns(value)
    if timestamp <= 0:
        raise RuntimeError(f"{label}: missing positive source timestamp")
    header = getattr(value, "header", None)
    frame_id = getattr(value, "frame_id", getattr(header, "frame_id", ""))
    if str(frame_id) != expected_frame:
        raise RuntimeError(
            f"{label}: frame_id {frame_id!r} != {expected_frame!r}"
        )
    return {
        "frame_id": str(frame_id),
        "orientation": tuple(float(item) for item in orientation),
        "position": tuple(float(item) for item in position),
        "stamp_ns": timestamp,
    }


def check_gripper_position(value, label: str) -> dict:
    position = float(value.position)
    finite((position,), label)
    timestamp = stamp_ns(value)
    if timestamp <= 0:
        raise RuntimeError(f"{label}: missing positive source timestamp")
    header = getattr(value, "header", None)
    frame_id = str(getattr(header, "frame_id", ""))
    return {
        "frame_id": frame_id,
        "position": position,
        "stamp_ns": timestamp,
    }


def stream_pair(stream, operation: str, checker) -> tuple[dict, dict]:
    try:
        iterator = iter(stream)
        first = checker(require(next(iterator), operation), operation)
        second = checker(require(next(iterator), operation), operation)
    finally:
        close_stream(stream, operation)
    if int(second["stamp_ns"]) <= int(first["stamp_ns"]):
        raise RuntimeError(f"{operation}: source timestamp did not advance")
    if first.get("names") != second.get("names"):
        raise RuntimeError(f"{operation}: joint identity/order changed")
    if set(first.get("identities", {})) != set(second.get("identities", {})):
        raise RuntimeError(f"{operation}: frame identity set changed")
    if first.get("identities") and any(
        int(second["identities"][identity])
        <= int(first["identities"][identity])
        for identity in first["identities"]
    ):
        raise RuntimeError(f"{operation}: a transform timestamp did not advance")
    return first, second


def unary_with_joint_stream(
    *,
    unary_call,
    stream_factory,
    stream_operation: str,
    expected_names: tuple[str, ...],
) -> tuple[object, tuple[dict, dict]]:
    """Bracket an unstamped unary body read with two joint stream frames."""

    stream = stream_factory()
    try:
        iterator = iter(stream)
        first = check_joint(
            require(next(iterator), stream_operation),
            stream_operation,
            expected_names,
        )
        unary = unary_call()
        second = check_joint(
            require(next(iterator), stream_operation),
            stream_operation,
            expected_names,
        )
    finally:
        close_stream(stream, stream_operation)
    if int(second["stamp_ns"]) <= int(first["stamp_ns"]):
        raise RuntimeError(f"{stream_operation}: source timestamp did not advance")
    if (
        first["names"],
        first["frame_id"],
    ) != (
        second["names"],
        second["frame_id"],
    ):
        raise RuntimeError(f"{stream_operation}: joint identity/order changed")
    return unary, (first, second)


def unary_stream_crosscheck(
    *,
    unary_call,
    unary_operation: str,
    stream_factory,
    stream_operation: str,
    stream_read,
    checker,
    compare,
) -> tuple[dict, tuple[dict, ...]]:
    """Read one unary sample while its corresponding stream remains open.

    The first stream sample precedes the unary call.  We then consume a
    bounded number of strictly newer stream samples until one reaches or
    passes the unary source timestamp.  This proves both RPC surfaces are
    fresh and lets ``compare`` bind the unary value to the nearest streamed
    sample from the same source timeline.
    """

    stream = stream_factory()
    samples: list[dict] = []
    try:
        first = checker(stream_read(stream), stream_operation)
        samples.append(first)
        unary = checker(unary_call(), unary_operation)
        if int(unary["stamp_ns"]) < int(first["stamp_ns"]):
            raise RuntimeError(
                f"{unary_operation}: unary timestamp predates the already "
                f"open {stream_operation} sample"
            )

        while (
            len(samples) < 2
            or int(samples[-1]["stamp_ns"]) < int(unary["stamp_ns"])
        ):
            if len(samples) >= MAX_STREAM_CATCHUP_SAMPLES:
                raise RuntimeError(
                    f"{stream_operation}: did not catch up to unary timestamp "
                    f"within {MAX_STREAM_CATCHUP_SAMPLES} samples"
                )
            current = checker(stream_read(stream), stream_operation)
            if int(current["stamp_ns"]) <= int(samples[-1]["stamp_ns"]):
                raise RuntimeError(
                    f"{stream_operation}: source timestamp did not advance"
                )
            samples.append(current)
    finally:
        close_stream(stream, stream_operation)

    nearest = min(
        samples,
        key=lambda sample: abs(
            int(sample["stamp_ns"]) - int(unary["stamp_ns"])
        ),
    )
    compare(unary, nearest, unary_operation)
    return unary, tuple(samples)


def crosschecked_detail(unary: dict, samples: tuple[dict, ...]) -> dict:
    detail = {
        "stream_first_stamp_ns": samples[0]["stamp_ns"],
        "stream_last_stamp_ns": samples[-1]["stamp_ns"],
        "stream_samples": len(samples),
        "unary_stamp_ns": unary["stamp_ns"],
    }
    for key in (
        "child_frame_id",
        "frame_id",
        "format",
        "height",
        "names",
        "width",
    ):
        if key in unary:
            value = unary[key]
            detail[key] = list(value) if isinstance(value, tuple) else value
    return detail


def pair_detail(pair: tuple[dict, dict]) -> dict:
    first, second = pair
    result = {
        "first_stamp_ns": first["stamp_ns"],
        "second_stamp_ns": second["stamp_ns"],
    }
    if "names" in second:
        result["names"] = list(second["names"])
    if "frame_id" in second:
        result["frame_id"] = second["frame_id"]
    if "child_frame_id" in second:
        result["child_frame_id"] = second["child_frame_id"]
    return result


def check_odometry(value, label: str, expected_frame: str) -> dict:
    pose = value.pose.pose
    twist = value.twist.twist
    position = (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    )
    orientation = (
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    linear = (
        float(twist.linear.x),
        float(twist.linear.y),
        float(twist.linear.z),
    )
    angular = (
        float(twist.angular.x),
        float(twist.angular.y),
        float(twist.angular.z),
    )
    finite(position + orientation + linear + angular, label)
    finite(tuple(value.pose.covariance), f"{label}.pose_covariance")
    finite(tuple(value.twist.covariance), f"{label}.twist_covariance")
    quaternion_norm = math.sqrt(sum(item**2 for item in orientation))
    if abs(quaternion_norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise RuntimeError(f"{label}: invalid orientation quaternion")
    timestamp = stamp_ns(value)
    frame_id = str(value.header.frame_id)
    child_frame_id = str(value.child_frame_id)
    if (
        timestamp <= 0
        or frame_id != expected_frame
        or child_frame_id != EXPECTED_ODOM_CHILD_FRAME
    ):
        raise RuntimeError(f"{label}: invalid frame identity")
    return {
        "child_frame_id": child_frame_id,
        "frame_id": frame_id,
        "angular": angular,
        "linear": linear,
        "orientation": orientation,
        "position": position,
        "stamp_ns": timestamp,
    }


def check_imu(value, label: str, expected_frame: str) -> dict:
    orientation = (
        float(value.orientation.x),
        float(value.orientation.y),
        float(value.orientation.z),
        float(value.orientation.w),
    )
    angular_velocity = (
        float(value.angular_velocity.x),
        float(value.angular_velocity.y),
        float(value.angular_velocity.z),
    )
    linear_acceleration = (
        float(value.linear_acceleration.x),
        float(value.linear_acceleration.y),
        float(value.linear_acceleration.z),
    )
    numbers = orientation + angular_velocity + linear_acceleration
    finite(numbers, label)
    finite(tuple(value.orientation_covariance), f"{label}.orientation_covariance")
    finite(
        tuple(value.angular_velocity_covariance),
        f"{label}.angular_velocity_covariance",
    )
    finite(
        tuple(value.linear_acceleration_covariance),
        f"{label}.linear_acceleration_covariance",
    )
    quaternion_norm = math.sqrt(sum(float(item) ** 2 for item in numbers[:4]))
    if abs(quaternion_norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise RuntimeError(f"{label}: invalid orientation quaternion")
    timestamp = stamp_ns(value)
    frame_id = str(value.header.frame_id)
    if timestamp <= 0 or frame_id != expected_frame:
        raise RuntimeError(f"{label}: invalid header")
    return {
        "angular_velocity": angular_velocity,
        "frame_id": frame_id,
        "linear_acceleration": linear_acceleration,
        "orientation": orientation,
        "stamp_ns": timestamp,
    }


def check_scan(value, label: str, expected_frame: str) -> dict:
    ranges = tuple(float(item) for item in value.ranges)
    if not ranges:
        raise RuntimeError(f"{label}.ranges: empty values")
    metadata = (
        float(value.angle_min),
        float(value.angle_max),
        float(value.angle_increment),
        float(value.time_increment),
        float(value.scan_time),
        float(value.range_min),
        float(value.range_max),
    )
    finite(metadata, label)
    if (
        metadata[1] < metadata[0]
        or metadata[2] <= 0.0
        or metadata[3] < 0.0
        or metadata[4] < 0.0
        or metadata[5] < 0.0
        or metadata[6] <= metadata[5]
    ):
        raise RuntimeError(f"{label}: invalid angular/range metadata")
    range_min = float(value.range_min)
    range_max = float(value.range_max)
    for item in ranges:
        if math.isnan(item) or item == -math.inf:
            raise RuntimeError(f"{label}: NaN or negative-infinite range")
        # Positive infinity is the ROS LaserScan representation for a valid
        # beam with no return.  Finite returns still have to respect the
        # message's declared measurement interval.
        if math.isfinite(item) and (
            item < range_min or item > range_max
        ):
            raise RuntimeError(f"{label}: range outside declared limits")
    intensities = tuple(float(item) for item in value.intensities)
    if intensities:
        if len(intensities) != len(ranges):
            raise RuntimeError(f"{label}: intensity/range length mismatch")
        finite(intensities, f"{label}.intensities")
    timestamp = stamp_ns(value)
    frame_id = str(value.header.frame_id)
    if timestamp <= 0 or frame_id != expected_frame:
        raise RuntimeError(f"{label}: invalid header")
    return {
        "frame_id": frame_id,
        "intensities": intensities,
        "metadata": metadata,
        "ranges": ranges,
        "samples": len(ranges),
        "stamp_ns": timestamp,
    }


def check_cloud(value, label: str, expected_frame: str) -> dict:
    if int(value.width) <= 0 or int(value.height) <= 0 or not value.data:
        raise RuntimeError(f"{label}: empty cloud")
    fields: dict[str, tuple[int, int, int]] = {}
    for field in value.fields:
        name = str(field.name)
        if not name or name in fields:
            raise RuntimeError(f"{label}: empty or duplicate field name {name!r}")
        fields[name] = (int(field.offset), int(field.datatype), int(field.count))
    if not {"x", "y", "z"}.issubset(fields):
        raise RuntimeError(f"{label}: missing x/y/z layout")
    # sensor_msgs/PointField.FLOAT32 == 7.  The decoder below intentionally
    # rejects implicit datatype conversion and multi-count xyz fields.
    for axis in ("x", "y", "z"):
        offset, datatype, count = fields[axis]
        if offset < 0 or datatype != 7 or count != 1:
            raise RuntimeError(f"{label}: invalid {axis} PointField layout")

    width = int(value.width)
    height = int(value.height)
    point_step = int(value.point_step)
    row_step = int(value.row_step)
    xyz_end = max(fields[axis][0] for axis in ("x", "y", "z")) + 4
    if point_step < xyz_end or row_step < width * point_step:
        raise RuntimeError(f"{label}: invalid point_step/row_step")
    raw = bytes(value.data)
    if len(raw) < row_step * height:
        raise RuntimeError(f"{label}: truncated point cloud payload")
    endian = ">" if value.is_bigendian else "<"
    total_points = width * height
    sample_count = min(total_points, 256)
    sample_indices = tuple(
        min(total_points - 1, index * total_points // sample_count)
        for index in range(sample_count)
    )
    sampled_xyz: list[tuple[float, float, float]] = []
    for index in sample_indices:
        row, column = divmod(index, width)
        base = row * row_step + column * point_step
        xyz = tuple(
            struct.unpack_from(f"{endian}f", raw, base + fields[axis][0])[0]
            for axis in ("x", "y", "z")
        )
        if not all(math.isfinite(item) for item in xyz):
            raise RuntimeError(
                f"{label}: non-finite xyz at bounded sample point {index}"
            )
        sampled_xyz.append(xyz)
    timestamp = stamp_ns(value)
    frame_id = str(value.header.frame_id)
    if timestamp <= 0 or frame_id != expected_frame:
        raise RuntimeError(f"{label}: invalid header")
    return {
        "field_layout": tuple(
            (name, *layout) for name, layout in sorted(fields.items())
        ),
        "frame_id": frame_id,
        "height": height,
        "is_bigendian": bool(value.is_bigendian),
        "point_step": point_step,
        "row_step": row_step,
        "sample_xyz": tuple(sampled_xyz),
        "stamp_ns": timestamp,
        "width": width,
    }


def check_camera(value, label: str, expected_format: str) -> dict:
    payload = bytes(getattr(value, "data", b""))
    size = int(getattr(value, "size", len(payload)))
    timestamp = stamp_ns(value)
    image_format = str(getattr(value, "format", ""))
    if (
        size <= 0
        or size != len(payload)
        or timestamp <= 0
        or image_format != expected_format
    ):
        raise RuntimeError(f"{label}: empty, untimestamped, or unidentified frame")
    return {
        "bytes": size,
        "digest": hashlib.sha256(payload).hexdigest(),
        "format": image_format,
        "seq": int(getattr(value, "seq", 0)),
        "stamp_ns": timestamp,
    }


def check_tf(value, label: str) -> dict:
    identities: dict[tuple[str, str], int] = {}
    child_frames: set[str] = set()
    anchor_stamp_ns: int | None = None
    for item in value.transforms:
        parent = str(item.header.frame_id)
        child = str(item.child_frame_id)
        identity = (parent, child)
        if (
            not parent
            or not child
            or parent == child
            or child in child_frames
            or identity in identities
        ):
            raise RuntimeError(
                f"{label}: empty, self-referential, or duplicate child frame"
            )
        translation = item.transform.translation
        rotation = item.transform.rotation
        finite(
            (
                translation.x,
                translation.y,
                translation.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
            label,
        )
        quaternion_norm = math.sqrt(
            rotation.x**2 + rotation.y**2 + rotation.z**2 + rotation.w**2
        )
        if abs(quaternion_norm - 1.0) > QUATERNION_NORM_TOLERANCE:
            raise RuntimeError(f"{label}: invalid quaternion for {identity!r}")
        timestamp = stamp_ns(item)
        if timestamp <= 0:
            raise RuntimeError(f"{label}: invalid timestamp for {identity!r}")
        child_frames.add(child)
        identities[identity] = timestamp
        if identity == EXPECTED_TF_ANCHOR:
            if anchor_stamp_ns is not None:
                raise RuntimeError(
                    f"{label}: duplicate dynamic TF anchor {EXPECTED_TF_ANCHOR!r}"
                )
            anchor_stamp_ns = timestamp
    if anchor_stamp_ns is None:
        raise RuntimeError(
            f"{label}: missing dynamic TF anchor {EXPECTED_TF_ANCHOR!r}"
        )
    return {
        "identities": identities,
        # Correlate successive dynamic-tree messages on the same exact anchor
        # used by the Server source-time gate.  Non-anchor timestamps may
        # legitimately advance on independent schedules.
        "stamp_ns": anchor_stamp_ns,
        "transforms": len(identities),
    }


def assert_scan_ranges_close(
    left: Iterable[float],
    right: Iterable[float],
    tolerance: float,
    label: str,
) -> None:
    left_values = tuple(float(item) for item in left)
    right_values = tuple(float(item) for item in right)
    if len(left_values) != len(right_values):
        raise RuntimeError(
            f"{label}: shape mismatch {len(left_values)} != {len(right_values)}"
        )
    for index, (left_item, right_item) in enumerate(
        zip(left_values, right_values)
    ):
        if left_item == math.inf and right_item == math.inf:
            continue
        if left_item == math.inf or right_item == math.inf:
            raise RuntimeError(f"{label}: no-return mask mismatch at {index}")
        if abs(left_item - right_item) > tolerance:
            raise RuntimeError(
                f"{label}: difference at {index} exceeds {tolerance:.9g}"
            )


def compare_joint(unary: dict, streamed: dict, label: str) -> None:
    if (
        unary["names"],
        unary["frame_id"],
    ) != (
        streamed["names"],
        streamed["frame_id"],
    ):
        raise RuntimeError(f"{label}: unary/stream joint identity mismatch")
    assert_close_values(
        unary["positions"],
        streamed["positions"],
        JOINT_POSITION_CROSSCHECK_TOLERANCE,
        f"{label}.position unary/stream",
    )
    assert_close_values(
        unary["velocity"],
        streamed["velocity"],
        JOINT_VELOCITY_CROSSCHECK_TOLERANCE,
        f"{label}.velocity unary/stream",
    )
    assert_close_values(
        unary["effort"],
        streamed["effort"],
        JOINT_EFFORT_CROSSCHECK_TOLERANCE,
        f"{label}.effort unary/stream",
    )


def compare_pose(unary: dict, streamed: dict, label: str) -> None:
    if unary["frame_id"] != streamed["frame_id"]:
        raise RuntimeError(f"{label}: unary/stream frame mismatch")
    assert_close_values(
        unary["position"],
        streamed["position"],
        POSE_POSITION_CROSSCHECK_TOLERANCE,
        f"{label}.position unary/stream",
    )
    assert_quaternion_close(
        unary["orientation"],
        streamed["orientation"],
        POSE_ORIENTATION_CROSSCHECK_TOLERANCE,
        f"{label}.orientation unary/stream",
    )


def compare_gripper_position(unary: dict, streamed: dict, label: str) -> None:
    if unary["frame_id"] != streamed["frame_id"]:
        raise RuntimeError(f"{label}: unary/stream gripper frame mismatch")
    assert_close_values(
        (unary["position"],),
        (streamed["position"],),
        0.006,
        f"{label}.position unary/stream",
    )


def compare_odometry(unary: dict, streamed: dict, label: str) -> None:
    if (
        unary["frame_id"],
        unary["child_frame_id"],
    ) != (
        streamed["frame_id"],
        streamed["child_frame_id"],
    ):
        raise RuntimeError(f"{label}: unary/stream odometry identity mismatch")
    assert_close_values(
        unary["position"],
        streamed["position"],
        ODOMETRY_CROSSCHECK_TOLERANCE,
        f"{label}.position unary/stream",
    )
    assert_quaternion_close(
        unary["orientation"],
        streamed["orientation"],
        POSE_ORIENTATION_CROSSCHECK_TOLERANCE,
        f"{label}.orientation unary/stream",
    )
    assert_close_values(
        unary["linear"] + unary["angular"],
        streamed["linear"] + streamed["angular"],
        ODOMETRY_CROSSCHECK_TOLERANCE,
        f"{label}.twist unary/stream",
    )


def compare_imu(unary: dict, streamed: dict, label: str) -> None:
    if unary["frame_id"] != streamed["frame_id"]:
        raise RuntimeError(f"{label}: unary/stream IMU frame mismatch")
    assert_quaternion_close(
        unary["orientation"],
        streamed["orientation"],
        POSE_ORIENTATION_CROSSCHECK_TOLERANCE,
        f"{label}.orientation unary/stream",
    )
    assert_close_values(
        unary["angular_velocity"] + unary["linear_acceleration"],
        streamed["angular_velocity"] + streamed["linear_acceleration"],
        IMU_CROSSCHECK_TOLERANCE,
        f"{label}.motion unary/stream",
    )


def compare_scan(unary: dict, streamed: dict, label: str) -> None:
    if unary["frame_id"] != streamed["frame_id"]:
        raise RuntimeError(f"{label}: unary/stream scan frame mismatch")
    assert_close_values(
        unary["metadata"],
        streamed["metadata"],
        1e-6,
        f"{label}.metadata unary/stream",
    )
    assert_scan_ranges_close(
        unary["ranges"],
        streamed["ranges"],
        SCAN_CROSSCHECK_TOLERANCE,
        f"{label}.ranges unary/stream",
    )
    assert_close_values(
        unary["intensities"],
        streamed["intensities"],
        SCAN_CROSSCHECK_TOLERANCE,
        f"{label}.intensities unary/stream",
    )


def compare_cloud(unary: dict, streamed: dict, label: str) -> None:
    # ManaEnv emits an unorganized depth cloud after removing invalid points.
    # The number of retained points can legitimately change by one or more
    # between adjacent source timestamps, so width and its derived row_step
    # are frame data rather than a stable schema contract.  Compare those
    # fields and sampled XYZ only when unary and stream identify the exact
    # same source frame.  Distinct frames must still agree on every fixed
    # layout property; check_cloud() has already validated each frame's own
    # dimensions, payload bounds, finite XYZ sample, and timestamp.
    schema_keys = (
        "field_layout",
        "frame_id",
        "height",
        "is_bigendian",
        "point_step",
    )
    if any(unary[key] != streamed[key] for key in schema_keys):
        raise RuntimeError(f"{label}: unary/stream cloud schema mismatch")
    if int(unary["stamp_ns"]) == int(streamed["stamp_ns"]):
        if (unary["width"], unary["row_step"]) != (
            streamed["width"],
            streamed["row_step"],
        ):
            raise RuntimeError(
                f"{label}: equal source timestamp has different cloud layout"
            )
        assert_close_values(
            (
                component
                for point in unary["sample_xyz"]
                for component in point
            ),
            (
                component
                for point in streamed["sample_xyz"]
                for component in point
            ),
            CLOUD_CROSSCHECK_TOLERANCE,
            f"{label}.xyz unary/stream",
        )


def compare_camera(unary: dict, streamed: dict, label: str) -> None:
    if unary["format"] != streamed["format"]:
        raise RuntimeError(f"{label}: unary/stream camera format mismatch")
    if (
        int(unary["stamp_ns"]) == int(streamed["stamp_ns"])
        and unary["digest"] != streamed["digest"]
    ):
        raise RuntimeError(
            f"{label}: equal source timestamp has different unary/stream bytes"
        )


def camera_unary_stream_crosscheck(
    *,
    unary_call,
    unary_operation: str,
    stream_call,
    stream_operation: str,
    expected_format: str,
    deadline_ms: int,
    timeout: float,
) -> tuple[dict, tuple[dict, ...]]:
    """Collect callback video frames around a unary camera read.

    Native camera streams are callback-based rather than iterators.  The
    callback never raises into C++; it records the first validation failure,
    wakes the waiter, and lets the owning thread close the stream exactly once.
    """

    condition = threading.Condition()
    samples: deque[dict] = deque(maxlen=MAX_CAMERA_CALLBACK_SAMPLES)
    callback_errors: list[BaseException] = []
    callback_count = 0

    def callback(result) -> None:
        nonlocal callback_count
        try:
            sample = check_camera(
                require(result, stream_operation),
                stream_operation,
                expected_format,
            )
            error: BaseException | None = None
        except BaseException as caught:
            sample = None
            error = caught
        with condition:
            callback_count += 1
            if error is not None:
                if not callback_errors:
                    callback_errors.append(error)
            elif sample is not None:
                samples.append(sample)
            condition.notify_all()

    stream = require(
        stream_call(callback, deadline_ms=deadline_ms),
        stream_operation,
    )
    expires_at = time.monotonic() + timeout
    try:
        with condition:
            while not samples and not callback_errors:
                remaining = expires_at - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"{stream_operation}: timed out waiting for first callback"
                    )
                condition.wait(remaining)
            if callback_errors:
                raise RuntimeError(
                    f"{stream_operation}: callback failed: {callback_errors[0]}"
                ) from callback_errors[0]
            first_stamp_ns = int(samples[0]["stamp_ns"])
            callbacks_before_unary = callback_count

        unary = check_camera(
            require(unary_call(), unary_operation),
            unary_operation,
            expected_format,
        )
        if int(unary["stamp_ns"]) < first_stamp_ns:
            raise RuntimeError(
                f"{unary_operation}: unary timestamp predates the open video stream"
            )

        with condition:
            while True:
                if callback_errors:
                    raise RuntimeError(
                        f"{stream_operation}: callback failed: {callback_errors[0]}"
                    ) from callback_errors[0]
                current_samples = tuple(samples)
                has_post_unary_callback = callback_count > callbacks_before_unary
                has_two_fresh_frames = (
                    len(current_samples) >= 2
                    and all(
                        int(current_samples[index]["stamp_ns"])
                        > int(current_samples[index - 1]["stamp_ns"])
                        for index in range(1, len(current_samples))
                    )
                )
                caught_up = (
                    current_samples
                    and int(current_samples[-1]["stamp_ns"])
                    >= int(unary["stamp_ns"])
                )
                if has_post_unary_callback and has_two_fresh_frames and caught_up:
                    break
                remaining = expires_at - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError(
                        f"{stream_operation}: callback stream did not produce "
                        "two advancing frames and catch up to the unary timestamp"
                    )
                condition.wait(remaining)
    finally:
        close_stream(stream, stream_operation)

    current_samples = tuple(samples)
    nearest = min(
        current_samples,
        key=lambda sample: abs(
            int(sample["stamp_ns"]) - int(unary["stamp_ns"])
        ),
    )
    compare_camera(unary, nearest, unary_operation)
    return unary, current_samples


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:15551", type=endpoint)
    parser.add_argument("--timeout", default=4.0, type=float)
    args = parser.parse_args()
    if not 2.0 <= args.timeout <= 15.0:
        raise SystemExit("timeout must be in 2..15 seconds")

    records: list[dict] = []

    def passed(name: str, detail) -> None:
        records.append({"name": name, "result": "PASS", "detail": detail})

    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        deadline_ms = int(args.timeout * 1000)
        model = require(robot.system.get_model_type(timeout=args.timeout), "System.GetModelType")
        expected_model = int(RobotModelType.QUANTA_X1_PRO)
        if int(model.model_type) != expected_model:
            raise RuntimeError(
                f"System.GetModelType: {int(model.model_type)} != EX001/{expected_model}"
            )
        passed("System.GetModelType", {"model_type": expected_model})

        head, head_pair = unary_with_joint_stream(
            unary_call=lambda: require(
                robot.head.get_pose(timeout=args.timeout), "Head.GetPose"
            ),
            stream_factory=lambda: robot.head.get_joint_states_stream(
                timeout=args.timeout
            ),
            stream_operation="Head.GetJointStatesStream",
            expected_names=EXPECTED_JOINTS["Head"],
        )
        finite((head.pitch, head.yaw), "Head.GetPose")
        latest_head = head_pair[1]["positions"]
        if abs(float(head.pitch) - latest_head[0]) > 0.02 or abs(float(head.yaw) - latest_head[1]) > 0.02:
            raise RuntimeError("Head.GetPose: disagrees with named stream")
        passed(
            "Head.GetPose+GetJointStatesStream",
            {
                **pair_detail(head_pair),
                "pitch": float(head.pitch),
                "yaw": float(head.yaw),
            },
        )

        for name, arm in (
            ("LeftArm", robot.left_arm_controller),
            ("RightArm", robot.right_arm_controller),
        ):
            joint_unary, joint_samples = unary_stream_crosscheck(
                unary_call=lambda arm=arm: require(
                    arm.get_joint_states(deadline_ms=deadline_ms),
                    f"{name}.GetJointStates",
                ),
                unary_operation=f"{name}.GetJointStates",
                stream_factory=lambda arm=arm: require(
                    arm.get_joint_states_stream(deadline_ms=deadline_ms),
                    f"{name}.GetJointStatesStream",
                ),
                stream_operation=f"{name}.GetJointStatesStream",
                stream_read=lambda stream: require(
                    stream.read(deadline_ms=deadline_ms),
                    f"{name}.GetJointStatesStream",
                ),
                checker=lambda value, label, name=name: check_joint(
                    value, label, EXPECTED_JOINTS[name]
                ),
                compare=compare_joint,
            )
            passed(
                f"{name}.GetJointStates+GetJointStatesStream",
                crosschecked_detail(joint_unary, joint_samples),
            )
            pose_unary, pose_samples = unary_stream_crosscheck(
                unary_call=lambda arm=arm: require(
                    arm.get_end_pose(deadline_ms=deadline_ms),
                    f"{name}.GetEndPose",
                ),
                unary_operation=f"{name}.GetEndPose",
                stream_factory=lambda arm=arm: require(
                    arm.get_end_pose_stream(deadline_ms=deadline_ms),
                    f"{name}.GetEndPoseStream",
                ),
                stream_operation=f"{name}.GetEndPoseStream",
                stream_read=lambda stream: require(
                    stream.read(deadline_ms=deadline_ms),
                    f"{name}.GetEndPoseStream",
                ),
                checker=lambda value, label: check_pose(
                    value, label, EXPECTED_ARM_END_POSE_FRAME
                ),
                compare=compare_pose,
            )
            passed(
                f"{name}.GetEndPose+GetEndPoseStream",
                crosschecked_detail(pose_unary, pose_samples),
            )

        for name, gripper in (
            ("LeftGripper", robot.left_gripper),
            ("RightGripper", robot.right_gripper),
        ):
            position_unary, position_samples = unary_stream_crosscheck(
                unary_call=lambda gripper=gripper, name=name: require(
                    gripper.get_position(timeout=args.timeout),
                    f"{name}.GetPosition",
                ),
                unary_operation=f"{name}.GetPosition",
                stream_factory=lambda gripper=gripper: gripper.get_position_stream(
                    timeout=args.timeout
                ),
                stream_operation=f"{name}.GetPositionStream",
                stream_read=lambda stream, name=name: require(
                    next(iter(stream)), f"{name}.GetPositionStream"
                ),
                checker=check_gripper_position,
                compare=compare_gripper_position,
            )
            joint_pair = stream_pair(
                gripper.get_joint_states_stream(timeout=args.timeout),
                f"{name}.GetJointStatesStream",
                lambda value, label, name=name: check_joint(
                    value, label, EXPECTED_JOINTS[name]
                ),
            )
            latest_joint_position = float(joint_pair[1]["positions"][0])
            if (
                abs(position_unary["position"] - latest_joint_position) > 0.006
                or abs(position_samples[-1]["position"] - latest_joint_position)
                > 0.006
            ):
                raise RuntimeError(
                    f"{name}: GetPosition unary/position stream disagrees "
                    "with the named JointState stream"
                )
            passed(
                f"{name}.GetPosition+GetPositionStream+GetJointStatesStream",
                {
                    **crosschecked_detail(position_unary, position_samples),
                    "joint_first_stamp_ns": joint_pair[0]["stamp_ns"],
                    "joint_last_stamp_ns": joint_pair[1]["stamp_ns"],
                    "names": list(joint_pair[1]["names"]),
                    "position": position_unary["position"],
                },
            )

        lift, lift_pair = unary_with_joint_stream(
            unary_call=lambda: require(
                robot.lift.get_lift_position(timeout=args.timeout),
                "Lift.GetLiftPosition",
            ),
            stream_factory=lambda: robot.lift.get_joint_states_stream(
                timeout=args.timeout
            ),
            stream_operation="Lift.GetJointStatesStream",
            expected_names=EXPECTED_JOINTS["Lift"],
        )
        finite((lift.position,), "Lift.GetLiftPosition")
        if abs(float(lift.position) - lift_pair[1]["positions"][0]) > 0.006:
            raise RuntimeError("Lift.GetLiftPosition: disagrees with named stream")
        passed(
            "Lift.GetLiftPosition+GetJointStatesStream",
            {**pair_detail(lift_pair), "position": float(lift.position)},
        )

        odometry_unary, odometry_samples = unary_stream_crosscheck(
            unary_call=lambda: require(
                robot.chassis.get_odometry(timeout=args.timeout),
                "Chassis.GetOdometry",
            ),
            unary_operation="Chassis.GetOdometry",
            stream_factory=lambda: robot.chassis.get_odometry_stream(
                timeout=args.timeout
            ),
            stream_operation="Chassis.GetOdometryStream",
            stream_read=lambda stream: require(
                next(iter(stream)), "Chassis.GetOdometryStream"
            ),
            checker=lambda value, label: check_odometry(
                value, label, EXPECTED_FRAMES["Chassis.GetOdometry"]
            ),
            compare=compare_odometry,
        )
        passed(
            "Chassis.GetOdometry+GetOdometryStream",
            crosschecked_detail(odometry_unary, odometry_samples),
        )

        imu_unary, imu_samples = unary_stream_crosscheck(
            unary_call=lambda: require(
                robot.imu.get_chassis_imu(timeout=args.timeout),
                "Imu.GetChassisImu",
            ),
            unary_operation="Imu.GetChassisImu",
            stream_factory=lambda: robot.imu.get_chassis_imu_stream(
                timeout=args.timeout
            ),
            stream_operation="Imu.GetChassisImuStream",
            stream_read=lambda stream: require(
                next(iter(stream)), "Imu.GetChassisImuStream"
            ),
            checker=lambda value, label: check_imu(
                value, label, EXPECTED_FRAMES["Imu.GetChassisImu"]
            ),
            compare=compare_imu,
        )
        passed(
            "Imu.GetChassisImu+GetChassisImuStream",
            crosschecked_detail(imu_unary, imu_samples),
        )

        scan_unary, scan_samples = unary_stream_crosscheck(
            unary_call=lambda: require(
                robot.radar.get_laser_scan(timeout=args.timeout),
                "Radar.GetLaserScan",
            ),
            unary_operation="Radar.GetLaserScan",
            stream_factory=lambda: robot.radar.get_laser_scan_stream(
                timeout=args.timeout
            ),
            stream_operation="Radar.GetLaserScanStream",
            stream_read=lambda stream: require(
                next(iter(stream)), "Radar.GetLaserScanStream"
            ),
            checker=lambda value, label: check_scan(
                value, label, EXPECTED_FRAMES["Radar.GetLaserScan"]
            ),
            compare=compare_scan,
        )
        passed(
            "Radar.GetLaserScan+GetLaserScanStream",
            {
                **crosschecked_detail(scan_unary, scan_samples),
                "samples": scan_unary["samples"],
            },
        )

        cloud_unary, cloud_samples = unary_stream_crosscheck(
            unary_call=lambda: require(
                robot.depth_points.get_chassis_depth_points(timeout=args.timeout),
                "DepthPoints.GetChassisDepthPoints",
            ),
            unary_operation="DepthPoints.GetChassisDepthPoints",
            stream_factory=lambda: robot.depth_points.get_chassis_depth_points_stream(
                timeout=args.timeout
            ),
            stream_operation="DepthPoints.GetChassisDepthPointsStream",
            stream_read=lambda stream: require(
                next(iter(stream)), "DepthPoints.GetChassisDepthPointsStream"
            ),
            checker=lambda value, label: check_cloud(
                value,
                label,
                EXPECTED_FRAMES["DepthPoints.GetChassisDepthPoints"],
            ),
            compare=compare_cloud,
        )
        passed(
            "DepthPoints.GetChassisDepthPoints+GetChassisDepthPointsStream",
            {
                **crosschecked_detail(cloud_unary, cloud_samples),
                "sampled_xyz": len(cloud_unary["sample_xyz"]),
            },
        )

        for name, unary_call, stream_name, stream_call in (
            (
                "HeadCamera.GetRGBImage",
                robot.head_camera.get_rgb_image,
                "HeadCamera.GetRGBVideoStream",
                robot.head_camera.get_rgb_video_stream,
            ),
            (
                "HeadCamera.GetDepthImage",
                robot.head_camera.get_depth_image,
                "HeadCamera.GetDepthVideoStream",
                robot.head_camera.get_depth_video_stream,
            ),
            (
                "LeftArmCamera.GetRawImage",
                robot.left_arm_camera.get_raw_image,
                "LeftArmCamera.GetVideoStream",
                robot.left_arm_camera.get_video_stream,
            ),
            (
                "RightArmCamera.GetRawImage",
                robot.right_arm_camera.get_raw_image,
                "RightArmCamera.GetVideoStream",
                robot.right_arm_camera.get_video_stream,
            ),
        ):
            frame_unary, frame_samples = camera_unary_stream_crosscheck(
                unary_call=lambda unary_call=unary_call: unary_call(
                    deadline_ms=deadline_ms
                ),
                unary_operation=name,
                stream_call=stream_call,
                stream_operation=stream_name,
                expected_format=EXPECTED_CAMERA_FORMATS[name],
                deadline_ms=deadline_ms,
                timeout=args.timeout,
            )
            passed(
                f"{name}+{stream_name.rsplit('.', 1)[-1]}",
                {
                    **crosschecked_detail(frame_unary, frame_samples),
                    "bytes": frame_unary["bytes"],
                },
            )

        dynamic_stream = robot.tf.subscribe_tf(timeout=args.timeout)
        tf_pair = stream_pair(dynamic_stream, "Tf.SubscribeTf", check_tf)
        passed(
            "Tf.SubscribeTf",
            {
                **pair_detail(tf_pair),
                "identities": [
                    f"{parent}->{child}"
                    for parent, child in sorted(tf_pair[1]["identities"])
                ],
                "transforms": tf_pair[1]["transforms"],
            },
        )
    finally:
        require(robot.close(), "Robot.close")

    print(json.dumps({"result": "PASS", "writes_sent": 0, "records": records}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
