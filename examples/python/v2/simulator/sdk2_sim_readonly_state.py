#!/usr/bin/env python3
"""Read representative ManaEnv state through the ordinary typed SDK2 Client.

This example issues no lifecycle, mode, heartbeat, or Set call.
"""

from __future__ import annotations

import argparse

import x2robot_sdk


def require(result: x2robot_sdk.SdkResult, operation: str):
    if not result.is_success:
        error = result.error or {}
        name = error.get("error_name", "UNKNOWN")
        message = error.get("error_message", "SDK call failed")
        request_id = result.request_id or error.get("request_id") or "-"
        raise RuntimeError(f"{operation}: {name}: {message} (request_id={request_id})")
    return result.data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="127.0.0.1:15551")
    args = parser.parse_args()

    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        model = require(
            robot.system.get_model_type(timeout=3.0),
            "System.GetModelType",
        )
        head = require(robot.head.get_pose(timeout=3.0), "Head.GetPose")
        left = require(
            robot.left_arm_controller.get_joint_states(deadline_ms=3000),
            "LeftArm.GetJointStates",
        )
        right = require(
            robot.right_arm_controller.get_joint_states(deadline_ms=3000),
            "RightArm.GetJointStates",
        )
        left_gripper = require(
            robot.left_gripper.get_position(timeout=3.0),
            "LeftGripper.GetPosition",
        )
        right_gripper = require(
            robot.right_gripper.get_position(timeout=3.0),
            "RightGripper.GetPosition",
        )
        lift = require(
            robot.lift.get_lift_position(timeout=3.0),
            "Lift.GetLiftPosition",
        )
        odometry = require(
            robot.chassis.get_odometry(timeout=3.0),
            "Chassis.GetOdometry",
        )
        point = odometry.pose.pose.position
        print(f"model={model.model_type!r}")
        print(f"head_pitch={float(head.pitch):.6f} head_yaw={float(head.yaw):.6f}")
        print(f"left_arm_position={list(left.position)}")
        print(f"right_arm_position={list(right.position)}")
        print(
            "gripper_position_rad="
            f"left:{float(left_gripper.position):.9f} "
            f"right:{float(right_gripper.position):.9f}"
        )
        print(f"lift_position_m={float(lift.position):.9f}")
        print(
            "odometry_position_m="
            f"x:{float(point.x):.6f} y:{float(point.y):.6f} "
            f"z:{float(point.z):.6f}"
        )
        print("PASS: representative typed reads; writes_sent=0")
        return 0
    finally:
        require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
