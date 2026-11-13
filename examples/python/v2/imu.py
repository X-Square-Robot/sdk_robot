"""Chassis IMU one-shot and streaming example (v1 behavior, v2 API)."""

from __future__ import annotations

import argparse
import math

import x2robot_sdk


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {(error.get('error_message') or 'SDK call failed')} (request_id={result.request_id or '-'})")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def quaternion_to_euler(x, y, z, w):
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    return roll, pitch, math.atan2(siny, cosy)


def display_imu_data(value):
    if value is None:
        print("Warning: empty IMU data message")
        return
    if value.orientation is not None:
        q = value.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        print(f"Orientation (Euler): Roll={math.degrees(roll):.2f}°, Pitch={math.degrees(pitch):.2f}°, Yaw={math.degrees(yaw):.2f}°")
    else:
        print("Orientation (Euler): Not available")
    if value.angular_velocity is not None:
        v = value.angular_velocity
        print(f"Angular Velocity (rad/s): x={v.x:.3f}, y={v.y:.3f}, z={v.z:.3f}")
    else:
        print("Angular Velocity (rad/s): Not available")
    if value.linear_acceleration is not None:
        a = value.linear_acceleration
        print(f"Linear Acceleration (m/s²): x={a.x:.3f}, y={a.y:.3f}, z={a.z:.3f}")
    else:
        print("Linear Acceleration (m/s²): Not available")
    print("================================================")


def read_single(robot):
    value = require(robot.imu.get_chassis_imu(timeout=5.0), "Imu.GetChassisImu")
    display_imu_data(value)
    return value


def read_stream(robot, seconds=None):
    import time
    stream = robot.imu.get_chassis_imu_stream(
        timeout=None if seconds is None else max(4.0, seconds + 1.0)
    )
    count = 0
    try:
        end = None if seconds is None else time.monotonic() + max(0.1, seconds)
        for result in stream:
            display_imu_data(require(result, "Imu.GetChassisImuStream"))
            count += 1
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(stream)
    if not count:
        raise RuntimeError("IMU stream returned no samples")


def verify(robot, _model="auto"):
    value = read_single(robot)
    if value.orientation is None and value.angular_velocity is None and value.linear_acceleration is None:
        raise AssertionError("IMU returned no measurements")
    read_stream(robot, 0.5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--action", choices=("single", "stream", "verify"), default="single")
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "single":
            read_single(robot)
        elif args.action == "stream":
            read_stream(robot, args.seconds)
        else:
            verify(robot, getattr(robot, "model_name", "auto"))
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        require(robot.close(), "Robot.Close")


if __name__ == "__main__":
    raise SystemExit(main())
