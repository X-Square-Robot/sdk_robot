"""Run the v2 examples against one server connection."""

from __future__ import annotations

import argparse
import importlib
from dataclasses import dataclass

import x2robot_sdk
from x2robot.sdk import HeartBeatRequest
from x2robot.sdk.v2 import RobotModelType

DEFAULT_ADDRESS = "127.0.0.1:55051"


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {error.get('error_message', 'SDK call failed')}")
    return result.data


@dataclass(frozen=True)
class Case:
    file: str
    models: frozenset[str] = frozenset(("ex001", "cx002"))


CASES = (
    Case("audio.py"), Case("camera.py"), Case("chassis_control.py"),
    Case("check_connect.py"), Case("data_collection/data_collector.py"),
    Case("data_collection_example.py"), Case("data_replay_example.py"),
    Case("depth_points.py"), Case("desktop/arm_control.py", frozenset(("ex001",))),
    Case("desktop/gripper_control.py", frozenset(("ex001",))),
    Case("desktop/master_arm_control.py", frozenset(("ex001",))),
    Case("head_control.py"), Case("imu.py"), Case("navigation.py"),
    Case("quanta_x1/align_master_slave_demo.py", frozenset(("ex001",))),
    Case("quanta_x1/arm_control.py", frozenset(("ex001",))),
    Case("quanta_x1/gripper_control.py", frozenset(("ex001",))),
    Case("quanta_x1/lift_control.py", frozenset(("ex001",))),
    Case("quanta_x1/master_arm_control.py", frozenset(("ex001",))),
    Case("quanta_x2/arm_control.py", frozenset(("cx002",))),
    Case("quanta_x2/end_pose_replay_new.py", frozenset(("cx002",))),
    Case("quanta_x2/gripper_control.py", frozenset(("cx002",))),
    Case("quanta_x2/waist_control.py", frozenset(("cx002",))),
    Case("radar.py"), Case("robot_control.py"), Case("robot_status.py"),
    Case("system.py"), Case("tf.py"), Case("tof.py"), Case("ultrasonic.py"),
)

# Keep Quanta X3's aggregate checks read-only. Navigation write RPCs are
# exposed, but the aggregate verifier must not start mapping/localization or
# alter a map; arm, lift, chassis and other motion-writing examples remain out
# of the automatic run as well.
QUANTA_X3_CASES = (
    Case("check_connect.py", frozenset(("quanta_x3",))),
    Case("audio.py", frozenset(("quanta_x3",))),
    Case("camera.py", frozenset(("quanta_x3",))),
    Case("data_collection/data_collector.py", frozenset(("quanta_x3",))),
    Case("data_collection_example.py", frozenset(("quanta_x3",))),
    Case("data_replay_example.py", frozenset(("quanta_x3",))),
    Case("depth_points.py", frozenset(("quanta_x3",))),
    Case("quanta_x3/lidar_points.py", frozenset(("quanta_x3",))),
    Case("head_control.py", frozenset(("quanta_x3",))),
    Case("imu.py", frozenset(("quanta_x3",))),
    Case("navigation.py", frozenset(("quanta_x3",))),
    Case("radar.py", frozenset(("quanta_x3",))),
    Case("robot_status.py", frozenset(("quanta_x3",))),
    Case("tf.py", frozenset(("quanta_x3",))),
    Case("tof.py", frozenset(("quanta_x3",))),
    Case("ultrasonic.py", frozenset(("quanta_x3",))),
)


def detect_model(robot):
    info = require(robot.system.get_static_info(timeout=3.0), "System.GetStaticInfo")
    text = str(info.model).lower()
    if "cx002" in text or "quanta_x2" in text:
        return "cx002"
    if "desktop" in text:
        return "desktop"
    if "ex001" in text or "quanta_x1" in text:
        return "ex001"
    value = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType").model_type
    return {
        RobotModelType.QUANTA_X1_PRO: "ex001",
        RobotModelType.QUANTA_X2: "cx002",
        RobotModelType.QUANTA_X3: "quanta_x3",
    }.get(value, "unknown")


def module_name(file):
    name = file.removesuffix(".py").replace("/", ".")
    return f"examples.python.v2.{name}" if __package__ else name


def run(address, requested_model="auto"):
    robot = require(x2robot_sdk.connect(address, force_grpc=True), "connect")
    try:
        model = detect_model(robot) if requested_model == "auto" else requested_model
        if model == "desktop":
            # Desktop has its own explicit feedback-only matrix. Do not run
            # the full-robot cases or their command-writing verify() methods.
            module = importlib.import_module(module_name("desktop/verify_readonly.py"))
            module.verify(robot)
            # Desktop matrix reports item failures but does not abort the
            # caller midway through unrelated component checks.
            return True
        require(robot.state.heart_beat(HeartBeatRequest(client_id="sdk-example2"), timeout=3.0), "State.HeartBeat")
        cases = QUANTA_X3_CASES if model == "quanta_x3" else CASES
        if model == "quanta_x3":
            print(
                "quanta_x3 aggregate uses safe read cases; navigation writes, "
                "arm, lift, chassis, and other motion-writing examples are excluded."
            )
        passed = 0
        skipped = 0
        failed = []
        print(f"v2 examples: model={model} server={address}")
        for case in cases:
            if model not in case.models:
                skipped += 1
                print(f"SKIP {case.file}: not applicable to {model}")
                continue
            module = None
            try:
                module = importlib.import_module(module_name(case.file))
                module.verify(robot, model)
                passed += 1
                print(f"PASS {case.file}")
            except Exception as error:  # keep running to show every failing example
                failed.append((case.file, error))
                print(f"FAIL {case.file}: {error}")
            finally:
                # ``verify()`` reuses the same helpers as the standalone
                # examples and may enter SDK mode for a write-side smoke
                # check.  Its caller owns the shared connection, so return
                # the robot to IDLE after every case, before the next module
                # (or the final Robot.close()).
                restore = getattr(module, "restore_idle_mode", None)
                if callable(restore):
                    try:
                        restore(robot)
                    except Exception as error:
                        failed.append((case.file, error))
                        print(f"FAIL {case.file}: failed to restore IDLE: {error}")
        print(f"summary: total={len(cases)} pass={passed} model_skip={skipped} fail={len(failed)}")
        return not failed
    finally:
        require(robot.close(), "Robot.close")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default=DEFAULT_ADDRESS)
    parser.add_argument("--model", choices=("auto", "desktop", "ex001", "cx002", "quanta_x3"), default="auto")
    args = parser.parse_args(argv)
    return 0 if run(args.server, args.model) else 1


if __name__ == "__main__":
    raise SystemExit(main())
