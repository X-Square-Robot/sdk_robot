"""Read and verify Quanta X3 3D lidar ``sensor_msgs/PointCloud2`` samples."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import x2robot_sdk

_REPO_ROOT = str(Path(__file__).resolve().parents[4])
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from examples.python.v2._point_cloud import display_depth_points, parse_point_cloud2


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {(error.get('error_message') or 'SDK call failed')} (request_id={result.request_id or '-'})")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def require_quanta_x3(robot):
    if getattr(robot, "model_name", None) != "quanta_x3":
        raise RuntimeError("this example is only for Quanta X3")


def read_single(robot):
    message = require(robot.depth_points.get_lidar_points(timeout=8.0), "DepthPoints.GetLidarPoints")
    display_depth_points(message, parse_point_cloud2(message))
    return message


def read_stream(robot, seconds=None):
    stream = robot.depth_points.get_lidar_points_stream(
        timeout=None if seconds is None else max(4.0, seconds + 1.0)
    )
    count = 0
    try:
        import time
        deadline = None if seconds is None else time.monotonic() + max(0.1, seconds)
        for result in stream:
            message = require(result, "DepthPoints.GetLidarPointsStream")
            display_depth_points(message, parse_point_cloud2(message))
            count += 1
            if deadline is not None and time.monotonic() >= deadline:
                break
    finally:
        close_stream(stream)
    if count == 0:
        raise RuntimeError("lidar point stream returned no samples")


def verify(robot, _model="auto"):
    require_quanta_x3(robot)
    message = read_single(robot)
    if message.width * message.height == 0:
        raise AssertionError("empty lidar point cloud")
    read_stream(robot, 0.5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--action", choices=("single", "stream", "verify"), default="single")
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        require_quanta_x3(robot)
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
