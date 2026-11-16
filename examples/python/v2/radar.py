"""Radar/LaserScan example with the same detailed output as v1."""

from __future__ import annotations

import argparse
import math
import time

import x2robot_sdk

RADAR_FREQUENCY = 10
RADAR_ANGLE_INCREMENT = 0.02
RADAR_MIN_RANGE = 0.05
RADAR_MAX_RANGE = 25.0


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {(error.get('error_message') or 'SDK call failed')} (request_id={result.request_id or '-'})")
    return result.data


def expected_unimplemented(result):
    error = result.error or {}
    return error.get("error_code") == 100501 or error.get("error_name") == "UNIMPLEMENTED"


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def display_scan_data(scan):
    print(f"Angle Range: [{scan.angle_min:.3f}, {scan.angle_max:.3f}] rad")
    print(f"Angle Increment: {scan.angle_increment:.6f} rad (Spec: {RADAR_ANGLE_INCREMENT})")
    print(f"Time Increment: {scan.time_increment:.9f} s")
    if scan.scan_time > 0:
        print(f"Scan Time: {scan.scan_time:.3f} s (Frequency: {1.0 / scan.scan_time:.1f} Hz, Spec: {RADAR_FREQUENCY} Hz)")
    else:
        print(f"Scan Time: {scan.scan_time:.3f} s")
    print(f"Range Limits: [{scan.range_min:.2f}, {scan.range_max:.2f}] m (Spec: [{RADAR_MIN_RANGE}, {RADAR_MAX_RANGE}])")
    print(f"Number of Points: {len(scan.ranges)}")
    if scan.ranges:
        print(f"Sample Ranges (first 10): {[f'{value:.2f}' for value in scan.ranges[:10]]}")
        valid = [value for value in scan.ranges if not math.isnan(value)]
        print(f"Min Range in Scan: {min(valid):.2f} m" if valid else "Min Range in Scan: N/A (all values are NaN)")
        print(f"Max Range in Scan: {max(valid):.2f} m" if valid else "Max Range in Scan: N/A (all values are NaN)")
    if scan.intensities:
        print(f"Sample Intensities (first 10): {[f'{value:.2f}' for value in scan.intensities[:10]]}")


def read_stream(robot, seconds=None):
    stream = robot.radar.get_laser_scan_stream(timeout=None if seconds is None else max(4.0, seconds + 1.0))
    end = None if seconds is None else time.monotonic() + max(0.1, seconds)
    count = 0
    try:
        for result in stream:
            scan = require(result, "Radar.GetLaserScanStream")
            if scan:
                display_scan_data(scan)
                print("-" * 80)
                count += 1
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(stream)
    if not count:
        raise RuntimeError("radar stream returned no samples")


def verify(robot, _model="auto"):
    if _model == "quanta_x3":
        result = robot.radar.get_laser_scan(timeout=5.0)
        if result.is_success or not expected_unimplemented(result):
            raise AssertionError("quanta_x3 Radar.GetLaserScan was not withdrawn as expected")
        print("Radar.GetLaserScan is not public for Quanta X3; skipped")
        return
    scan = require(robot.radar.get_laser_scan(timeout=5.0), "Radar.GetLaserScan")
    display_scan_data(scan)
    if not scan.ranges:
        raise AssertionError("radar returned no ranges")
    read_stream(robot, 0.5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", choices=("single", "stream", "verify"), default="single")
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "single":
            display_scan_data(require(robot.radar.get_laser_scan(timeout=5.0), "Radar.GetLaserScan"))
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
