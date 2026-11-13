#!/usr/bin/env python3
"""Python gRPC camera-stream performance benchmark.

Usage:
    python3 examples/python/v2/bench_camera.py [address] [camera] [seconds]

Example:
    python3 examples/python/v2/bench_camera.py 127.0.0.1:55051 left_wrist 10

Supported cameras: head_rgb, head_depth, head_left_eye, head_right_eye,
left_wrist, left_elbow, right_wrist, right_elbow.
"""

from __future__ import annotations

import argparse
import os
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import x2robot_sdk

# Keep this in sync with CameraBenchPrefix in x2robot/sdk/camera_bench_frame.hpp.
_BENCH_MAGIC = 0x3148434E45423258  # Little-endian bytes for "X2BENCH1".
_BENCH_PREFIX = struct.Struct("<QQq")  # magic, seq, steady_clock dispatch_ns


def require(result):
    if not result.is_success:
        raise RuntimeError((result.error or {}).get("error_message", "SDK call failed"))
    return result.data


@dataclass
class BenchResult:
    frames: int = 0
    byte_count: int = 0
    elapsed_s: float = 0.0
    gaps: int = 0
    lost: int = 0
    latencies_us: list[float] = field(default_factory=list)

    @property
    def fps(self) -> float:
        return self.frames / self.elapsed_s if self.elapsed_s else 0.0

    @property
    def mbps(self) -> float:
        return self.byte_count / self.elapsed_s / 1e6 if self.elapsed_s else 0.0

    @property
    def mean_us(self) -> float | None:
        return sum(self.latencies_us) / len(self.latencies_us) if self.latencies_us else None

    @property
    def p99_us(self) -> float | None:
        if not self.latencies_us:
            return None
        ordered = sorted(self.latencies_us)
        return ordered[min(len(ordered) - 1, int(0.99 * len(ordered)))]


def _stream_opener(robot: x2robot_sdk.Robot, camera: str) -> Callable:
    """Return the Python stream method for the requested camera."""
    methods = {
        "head_rgb": robot.head_camera.get_rgb_video_stream,
        "head_depth": robot.head_camera.get_depth_video_stream,
        "head_left_eye": robot.head_camera.get_left_eye_video_stream,
        "head_right_eye": robot.head_camera.get_right_eye_video_stream,
        "left_wrist": robot.left_arm_camera.get_video_stream,
        "left_elbow": robot.left_arm_camera.get_elbow_video_stream,
        "right_wrist": robot.right_arm_camera.get_video_stream,
        "right_elbow": robot.right_arm_camera.get_elbow_video_stream,
    }
    return methods[camera]


def bench(address: str, camera: str, seconds: float) -> BenchResult:
    result = BenchResult()
    lock = threading.Lock()
    first = True
    last_seq = 0

    def on_frame(frame_result: x2robot_sdk.SdkResult[x2robot_sdk.CameraFrame]) -> None:
        nonlocal first, last_seq
        if not frame_result.is_success:
            return
        frame = frame_result.data
        monotonic_now_ns = time.monotonic_ns()
        data = frame.data

        with lock:
            result.frames += 1
            result.byte_count += len(data)

            stamped_payload = len(data) >= _BENCH_PREFIX.size
            if stamped_payload:
                magic, seq, dispatch_ns = _BENCH_PREFIX.unpack_from(data)
                stamped_payload = magic == _BENCH_MAGIC
            if stamped_payload:
                result.latencies_us.append((monotonic_now_ns - dispatch_ns) / 1000.0)
            elif frame.stamp_ns > 0:
                # ROS2/proto timestamps use the system clock; the container and
                # host share the same system clock.
                result.latencies_us.append((time.time_ns() - frame.stamp_ns) / 1000.0)
                seq = frame.seq
            else:
                return
            if not first and seq != last_seq + 1:
                result.gaps += 1
                if seq > last_seq:
                    result.lost += seq - last_seq - 1
            first = False
            last_seq = seq

    with require(x2robot_sdk.connect(address)) as robot:
        stream = require(_stream_opener(robot, camera)(on_frame))
        try:
            time.sleep(seconds)
        finally:
            stream.close()
    # Match the C++ benchmark: connection and stream setup/teardown are not
    # included in the measurement window.
    result.elapsed_s = seconds
    return result


def report(name: str, result: BenchResult) -> None:
    mean_us = result.mean_us
    p99_us = result.p99_us
    latency = (
        f"latency mean {mean_us:6.1f} p99 {p99_us:7.1f} us"
        if mean_us is not None and p99_us is not None
        else "latency n/a (camera header.stamp is not set)"
    )
    print(
        f"  {name:<22} frames {result.frames:7d}  {result.fps:.1f} fps  "
        f"{result.mbps:6.2f} MB/s  {latency}  "
        f"gaps {result.gaps} lost {result.lost}"
    )


def write_result(path: str, grpc: BenchResult) -> None:
    def line(name: str, value: BenchResult) -> str:
        mean = value.mean_us or 0.0
        p99 = value.p99_us or 0.0
        return (
            f"{name} frames={value.frames} fps={value.fps:.1f} "
            f"mbps={value.mbps:.2f} mean_us={mean:.1f} "
            f"p99_us={p99:.1f} lost={value.lost}\n"
        )

    Path(path).write_text(line("grpc", grpc), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("address", nargs="?", default="127.0.0.1:55051")
    parser.add_argument(
        "camera",
        nargs="?",
        default="left_wrist",
        choices=(
            "head_rgb",
            "head_depth",
            "head_left_eye",
            "head_right_eye",
            "left_wrist",
            "left_elbow",
            "right_wrist",
            "right_elbow",
        ),
    )
    parser.add_argument("seconds", nargs="?", type=float, default=10.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("seconds must be greater than 0")
    return args


def main() -> int:
    args = parse_args()
    print(f"\nPython camera stream benchmark ({args.seconds:g} s, {args.camera})\n")

    grpc = bench(args.address, args.camera, args.seconds)
    report("gRPC stream", grpc)

    if output := os.getenv("X2BENCH_OUT"):
        write_result(output, grpc)

    return 0 if grpc.frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
