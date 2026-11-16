"""Read and decode chassis depth ``sensor_msgs/PointCloud2`` samples."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import x2robot_sdk

_REPO_ROOT = str(Path(__file__).resolve().parents[3])
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


def read_single(robot):
    message = require(robot.depth_points.get_chassis_depth_points(timeout=8.0), "DepthPoints.GetChassisDepthPoints")
    points = parse_point_cloud2(message)
    display_depth_points(message, points)
    return message


def read_stream(robot, seconds=None):
    stream = robot.depth_points.get_chassis_depth_points_stream(
        timeout=None if seconds is None else max(4.0, seconds + 1.0)
    )
    count = 0
    try:
        import time
        deadline = None if seconds is None else time.monotonic() + max(0.1, seconds)
        for result in stream:
            message = require(result, "DepthPoints.GetChassisDepthPointsStream")
            display_depth_points(message, parse_point_cloud2(message))
            count += 1
            if deadline is not None and time.monotonic() >= deadline:
                break
    finally:
        close_stream(stream)
    if count == 0:
        raise RuntimeError("depth point stream returned no samples")


def verify(robot, _model="auto"):
    message = read_single(robot)
    if message.width * message.height == 0:
        raise AssertionError("empty point cloud")
    # 单读和订阅都解析点云，避免仅验证收到数据而遗漏格式异常。
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
