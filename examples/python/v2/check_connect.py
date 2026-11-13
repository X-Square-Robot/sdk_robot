"""Smallest possible v2 connection example."""

from __future__ import annotations

import argparse

import x2robot_sdk
from x2robot.sdk import PingRequest


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


def verify(robot, _model="auto"):
    payload = "Hello, X2Robot!"
    pong = require(robot.benchmark.ping(PingRequest(payload=payload), timeout=3.0), "Benchmark.Ping")
    if payload not in pong.payload:
        raise AssertionError(f"unexpected pong payload: {pong.payload!r}")
    print(f"connected through {robot.transport}: {pong.payload}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="localhost:50051")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        verify(robot)
        print("Connection to X2Robot SDK server successful!")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
