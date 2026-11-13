"""Read whole-robot status."""

from __future__ import annotations

import argparse
import json

import x2robot_sdk
from x2robot.sdk import GetRobotStatusRequest

STATUS_GROUPS = ("energy", "motion", "execution", "safety", "health")


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def print_fields(title, section, fields):
    print(f"\n=== {title} ===")
    section = section or {}
    for field in fields:
        value = section.get(field)
        print(f"{field}: {json.dumps(value, ensure_ascii=False, indent=2) if isinstance(value, (dict, list)) else value}")


def selected_groups(fields):
    """Return the status groups requested by an optional field filter."""
    if not fields:
        return STATUS_GROUPS
    groups = []
    for item in fields:
        group = item.split(".", 1)[0]
        if group not in STATUS_GROUPS:
            raise ValueError(f"unknown robot status group: {group}")
        if group not in groups:
            groups.append(group)
    return tuple(groups)


def verify(robot, _model="auto", fields=None):
    payload = require(robot.robot_status.get_robot_status(GetRobotStatusRequest(fields=fields or []), timeout=3.0), "RobotStatus.GetRobotStatus")
    status = json.loads(payload)
    for name in selected_groups(fields):
        value = status.get(name)
        if value is None:
            raise AssertionError(f"status has no {name} group")
        wanted = [item.split(".", 1)[1] for item in (fields or []) if item.startswith(name + ".")]
        print_fields(name, value, wanted or list(value))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--fields", nargs="*", help="optional category or category.field filters")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        verify(robot, fields=args.fields)
        print("PASS: robot status")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
