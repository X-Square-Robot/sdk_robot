#!/usr/bin/env python3
"""Read full joint states through the SDK2 Client; no mode or actuator writes.

Compatible with the delivered 6b0203f Client. State uses timeout in seconds.
The stream is an iterator of SdkResult, not a native arm stream handle.
"""

import argparse
import json
import math
from pathlib import Path
import sys


def require(result, operation):
    if not result.is_success:
        raise RuntimeError(
            f"{operation}: {json.dumps(result.error, ensure_ascii=False)} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def frame_record(result, operation, index):
    state = require(result, operation)
    names = list(state.name)
    if not names or len(set(names)) != len(names) or any(not name for name in names):
        raise RuntimeError(f"{operation}: joint names are empty or duplicated")
    values = {field: list(getattr(state, field)) for field in ("position", "velocity", "effort")}
    for field, array in values.items():
        # ROS JointState permits an unavailable field to be an empty array.
        if len(array) not in (0, len(names)) or not all(math.isfinite(x) for x in array):
            raise RuntimeError(f"{operation}: invalid {field} length or non-finite value")
    if not any(values.values()):
        raise RuntimeError(f"{operation}: all joint value arrays are empty")
    header = state.header
    stamp = header.stamp if header is not None else None
    return {
        "operation": operation,
        "index": index,
        "request_id": result.request_id,
        "joint_count": len(names),
        "frame_id": header.frame_id if header is not None else None,
        "stamp_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) if stamp is not None else None,
        "name": names,
        **values,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:15551")
    parser.add_argument("--action", choices=("once", "stream", "both"), default="both")
    parser.add_argument("--count", type=int, default=5, help="Number of stream frames (default: 5)")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Seconds per RPC; for stream this covers the entire stream")
    parser.add_argument("--output", type=Path, help="Save results, including errors, as JSON")
    args = parser.parse_args(argv)
    if not 1 <= args.count <= 10000:
        parser.error("--count must be within [1, 10000]")
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 300:
        parser.error("--timeout must be within (0, 300] seconds")

    report = {"server": args.server, "action": args.action, "status": "FAIL", "frames": [], "errors": []}
    robot = None
    stream = None
    try:
        import x2robot_sdk

        robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
        if args.action in ("once", "both"):
            response = robot.state.get_all_joint_states(timeout=args.timeout)
            frame = frame_record(response, "State.GetAllJointStates", 1)
            report["frames"].append(frame)
            print(json.dumps(frame, ensure_ascii=False, allow_nan=False))
        if args.action in ("stream", "both"):
            stream = robot.state.get_all_joint_states_stream(timeout=args.timeout)
            iterator = iter(stream)
            for index in range(1, args.count + 1):
                try:
                    response = next(iterator)
                except StopIteration as error:
                    raise RuntimeError(f"Stream ended after {index - 1}/{args.count} frames") from error
                frame = frame_record(response, "State.GetAllJointStatesStream", index)
                report["frames"].append(frame)
                print(json.dumps(frame, ensure_ascii=False, allow_nan=False))
    except (Exception, KeyboardInterrupt) as error:
        report["errors"].append(str(error) or type(error).__name__)
    finally:
        if stream is not None:
            try:
                result = stream.cancel()
                if result is not None:
                    require(result, "stream.cancel")
            except Exception as error:
                report["errors"].append(f"stream cleanup: {error}")
            # Release the iterator as well as requesting cancellation. The
            # finite RPC deadline also bounds old Client stream lifetimes.
            stream = None
            iterator = None
        if robot is not None:
            try:
                require(robot.close(), "Robot.close")
            except Exception as error:
                report["errors"].append(f"connection cleanup: {error}")

    report["status"] = "FAIL" if report["errors"] else "PASS"
    if args.output is not None:
        try:
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        except Exception as error:
            report["errors"].append(f"save output: {error}")
            report["status"] = "FAIL"
    for error in report["errors"]:
        print(f"FAIL: {error}", file=sys.stderr)
    print(f"{report['status']}: all joint states ({len(report['frames'])} frame(s))")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
