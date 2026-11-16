"""TOF sensor example: single reads, both sensors and concurrent streams."""

from __future__ import annotations

import argparse
import math
import threading
import time

import x2robot_sdk


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


def display_tof_data(value, label, is_stream=False):
    radiation = {0: "ULTRASOUND", 1: "INFRARED"}.get(value.radiation_type, f"Unknown({value.radiation_type})")
    print(f"{label}:")
    print(f"  - Distance: {value.range:.3f} m")
    print(f"  - Radiation Type: {radiation}")
    print(f"  - Field of View: {value.field_of_view:.3f} rad ({math.degrees(value.field_of_view):.1f}°)")
    print(f"  - Min Range: {value.min_range:.2f} m")
    print(f"  - Max Range: {value.max_range:.2f} m")
    if value.range < value.min_range:
        print("  - Status: Distance too close (less than minimum range)")
    elif value.range > value.max_range:
        print("  - Status: Out of measurement range")
    else:
        print("  - Status: OK")
    print("================================================")


def read_all_sensors(robot):
    for index in (1, 2):
        method = getattr(robot.tof, f"get_chassis_tof_{index}")
        try:
            display_tof_data(require(method(timeout=5.0), f"Tof.GetChassisTof{index}"), f"Sensor {index}")
        except Exception as error:
            print(f"Sensor {index}: Read failed: {error}")


def stream_sensor(robot, index, seconds=None):
    method = getattr(robot.tof, f"get_chassis_tof_{index}_stream")
    stream = method(timeout=None if seconds is None else max(4.0, seconds + 1.0))
    count = 0
    end = None if seconds is None else time.monotonic() + max(0.1, seconds)
    try:
        for result in stream:
            display_tof_data(require(result, f"Tof.GetChassisTof{index}Stream"), f"Sensor {index} Stream", True)
            count += 1
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(stream)
    return count


def verify(robot, _model="auto"):
    if _model == "quanta_x3":
        for index in (1, 2):
            result = getattr(robot.tof, f"get_chassis_tof_{index}")(timeout=5.0)
            if result.is_success or not expected_unimplemented(result):
                raise AssertionError(f"quanta_x3 Tof.GetChassisTof{index} was not withdrawn as expected")
        print("Tof.GetChassisTof1/2 is not public for Quanta X3; skipped")
        return
    for index in (1, 2):
        value = require(getattr(robot.tof, f"get_chassis_tof_{index}")(timeout=5.0), f"Tof.GetChassisTof{index}")
        display_tof_data(value, f"Sensor {index}")
        if not stream_sensor(robot, index, 0.5):
            raise AssertionError(f"Sensor {index} stream returned no data")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", choices=("sensor-1", "sensor-2", "sensor-1-stream", "sensor-2-stream", "both", "both-stream", "verify"), default="sensor-1")
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot, getattr(robot, "model_name", "auto"))
        elif args.action == "both":
            read_all_sensors(robot)
        elif args.action == "both-stream":
            threads = [threading.Thread(target=stream_sensor, args=(robot, index, args.seconds), daemon=True) for index in (1, 2)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
        elif args.action.endswith("-stream"):
            stream_sensor(robot, int(args.action.split("-")[1]), args.seconds)
        else:
            index = int(args.action.split("-")[1])
            display_tof_data(require(getattr(robot.tof, f"get_chassis_tof_{index}")(timeout=5.0), f"Tof.GetChassisTof{index}"), f"Sensor {index}")
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
