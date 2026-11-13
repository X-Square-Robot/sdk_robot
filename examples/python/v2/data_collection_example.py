"""Record synchronized robot data with the v2 ``SdkResult`` API.

The episode format and the interactive flow intentionally match
``python/v1/data_collection_example.py``.  The only boundary change is the
connection: ``x2robot_sdk.connect`` returns an ``SdkResult`` and the v2
``DataCollector`` unwraps/checks every stream item internally.
"""

from __future__ import annotations

import argparse
import select
import signal
import sys
import time

import x2robot_sdk

try:
    from .data_collection.collection_config import CollectionConfig
    from .data_collection.data_collector import DataCollector
except ImportError:  # direct execution from an exported examples directory
    from data_collection.collection_config import CollectionConfig
    from data_collection.data_collector import DataCollector


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(
            f"{operation} failed: {error.get('error_name', 'UNKNOWN')}: "
            f"{error.get('error_message', 'SDK call failed')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def model_name(robot):
    """返回机型档案名：优先用客户端解析出的身份，枚举只作兜底。

    GetModelType 只给公开产品族——服务端把 desktop 也折叠成 QUANTA_X1_PRO，
    只看枚举会把 desktop 当成 quanta_x1，于是按 x1 去拉 head/lift 等并不存在
    的流，真机直接 UNIMPLEMENTED。连接时客户端已用设备自报的型号字符串解析
    出档案名（robot.model_name），这里优先采用。
    """
    resolved = str(getattr(robot, "model_name", "") or "").strip().lower()
    if resolved:
        return resolved
    value = require(robot.system.get_model_type(timeout=3.0), "System.GetModelType")
    text = str(getattr(value, "model_type", value)).upper()
    if "X1" in text:
        return "quanta_x1"
    if "X2" in text:
        return "quanta_x2"
    if "X3" in text:
        return "quanta_x3"
    return text.lower()


def classify_empty_streams(empty_streams):
    """Split failed/empty self-check streams into optional and required ones.

    Mirrors the collector's own validation: master-arm tracking data is only a
    warning (not every bench has master arms), every other enabled stream must
    deliver frames without a stream error.
    """
    optional = [name for name in empty_streams if name.startswith("master_")]
    required = [name for name in empty_streams if not name.startswith("master_")]
    return optional, required


def describe_model_source(robot):
    """Return (detected profile, human readable source) for startup logging."""
    resolved = str(getattr(robot, "model_name", "") or "").strip()
    if resolved:
        return resolved.lower(), f"client identity {resolved!r}"
    return "(enum fallback)", "System.GetModelType enum"


def create_collection_config_for_quanta_x1(binocular=False):
    config = CollectionConfig(
        slave_joint_names=[
            "left_arm_joint_states", "right_arm_joint_states", "lift_joint_states",
            "left_gripper_joint_states", "right_gripper_joint_states", "head_joint_states",
        ],
        enable_left_arm_end_pose=True,
        enable_right_arm_end_pose=True,
        enable_odometry=True,
        enable_master_arm_data=True,
        enable_wrench_ext_world=True,
        enable_wrench_ext_local=True,
    )
    if binocular:
        config.enable_head_left_eye_stream = True
        config.enable_head_right_eye_stream = True
        config.enable_left_arm_elbow_stream = True
        config.enable_right_arm_elbow_stream = True
    else:
        config.enable_head_rgb_stream = True
    config.enable_left_arm_rgb_stream = True
    config.enable_right_arm_rgb_stream = True
    return config


def create_collection_config_for_quanta_x2():
    return CollectionConfig(
        slave_joint_names=[
            "left_arm_joint_states", "right_arm_joint_states",
            "left_gripper_joint_states", "right_gripper_joint_states",
            "waist_joint_states", "head_joint_states",
        ],
        enable_head_rgb_stream=True,
        enable_left_arm_rgb_stream=True,
        enable_right_arm_rgb_stream=True,
        enable_left_arm_end_pose=True,
        enable_right_arm_end_pose=True,
        enable_odometry=True,
        enable_waist_end_pose=True,
    )


def create_collection_config_for_quanta_x3():
    """Return the read-only collection profile for quanta_x3.

    Record both joint state and all three Cartesian pose streams so the
    episode supports joint and end-pose replay. Pose feedback must already
    be publishing in the robot's current mode; collection never changes it.
    quanta_x3 also exposes four H.265 camera streams and the 21-joint body state
    split into arm, gripper, waist and head streams. The legacy collector
    also supports a number of older-model sources (RGB/Depth images,
    master arms, wrench and tactile data); leaving those fields disabled is
    intentional because they are outside the quanta_x3 public contract.
    """
    return CollectionConfig(
        slave_joint_names=[
            "left_arm_joint_states", "right_arm_joint_states",
            "left_gripper_joint_states", "right_gripper_joint_states",
            "waist_joint_states", "head_joint_states",
        ],
        enable_head_left_eye_stream=True,
        enable_head_right_eye_stream=True,
        enable_left_arm_rgb_stream=True,
        enable_right_arm_rgb_stream=True,
        enable_left_arm_end_pose=True,
        enable_right_arm_end_pose=True,
        enable_waist_end_pose=True,
        enable_odometry=True,
        enable_chassis_imu=True,
        enable_ultrasonic_sensors=True,
    )


def create_collection_config_for_desktop():
    return CollectionConfig(
        slave_joint_names=[
            "left_arm_joint_states", "right_arm_joint_states",
            "left_gripper_joint_states", "right_gripper_joint_states",
        ],
        enable_head_rgb_stream=True,
        enable_left_arm_rgb_stream=True,
        enable_right_arm_rgb_stream=True,
        enable_left_arm_end_pose=True,
        enable_right_arm_end_pose=True,
    )


def _stop_on_enter(collector):
    """Keep v1's Enter-to-stop loop while still printing one-second stats."""
    while True:
        try:
            if select.select([sys.stdin], [], [], 1.0)[0]:
                input()
                return False
            collector.print_stats()
        except KeyboardInterrupt:
            return True


def verify(robot, model="auto"):
    """Run the non-destructive adapter check used by ``verify_all``.

    Starting a recording is intentionally not part of the aggregate smoke
    test: it creates files and starts a set of high-rate streams.  The full
    recording path is exercised by :func:`main`; here we delegate to the
    collector's own contract check so this example remains safe to run on a
    live robot.
    """
    # Keep this import local so direct execution from an exported examples
    # directory continues to work without making the package layout part of
    # the public API.
    try:
        from .data_collection.data_collector import verify as verify_collector
    except ImportError:
        from data_collection.data_collector import verify as verify_collector
    verify_collector(robot, model)
    if model in ("auto", "quanta_x3"):
        config = create_collection_config_for_quanta_x3()
        print(
            "quanta_x3 collection profile ready: "
            f"joints={','.join(config.slave_joint_names)}; "
            f"cameras={','.join(config.get_camera_names())}; "
            f"sensors={','.join(config.get_enabled_sensors())}"
        )


def record_one(collector, *, task, non_interactive=False, duration=0.0):
    print(f"\nStarting episode: {task}")
    collector.start_recording(task=task)
    interrupted = False
    if non_interactive:
        end = time.monotonic() + max(0.0, duration)
        while time.monotonic() < end:
            time.sleep(min(1.0, max(0.01, end - time.monotonic())))
            collector.print_stats()
    else:
        print("Recording in progress; press Enter to stop (Ctrl+C interrupts).")
        interrupted = _stop_on_enter(collector)
    info = collector.stop_recording() if collector.is_recording else None
    if info:
        print(
            f"Episode {info.get('episode_id')} saved: "
            f"{info.get('num_frames', 0)} frames, "
            f"{info.get('duration', 0.0):.2f}s at {info.get('episode_dir')}"
        )
    return interrupted, info


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument(
        "--model",
        choices=("auto", "quanta_x1", "quanta_x2", "quanta_x3", "desktop", "ex001", "cx002"),
        default="auto",
    )
    parser.add_argument("--output-dir", default="./collected_data")
    parser.add_argument("--target-hz", type=float, default=30.0)
    parser.add_argument("--keep-raw-data", action="store_true")
    parser.add_argument("--binocular", action="store_true")
    parser.add_argument("--no-master-arm-data", action="store_true",
                        help="skip master-arm streams when no teleoperation master is connected")
    parser.add_argument("--episodes", type=int, default=0, help="number of episodes; 0 keeps v1 interactive continuation")
    parser.add_argument("--duration", type=float, default=0.0, help="non-interactive episode duration in seconds")
    parser.add_argument("--non-interactive", action="store_true")
    parser.add_argument("--no-video-storage", action="store_true")
    parser.add_argument(
        "--self-check",
        action="store_true",
        help="open every configured stream for --duration seconds (default 5), print "
             "per-stream frame counts and the first RPC error, save nothing, then exit "
             "(exit code 3 when an enabled stream produced no frame)",
    )
    args = parser.parse_args(argv)

    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    collector = None
    try:
        detected = model_name(robot) if args.model == "auto" else args.model
        if detected in ("ex001", "quanta_x1"):
            config = create_collection_config_for_quanta_x1(args.binocular)
            model_label = "quanta_x1"
        elif detected in ("cx002", "quanta_x2"):
            config = create_collection_config_for_quanta_x2()
            model_label = "quanta_x2"
        elif detected == "desktop":
            config = create_collection_config_for_desktop()
            model_label = "desktop"
        elif detected == "quanta_x3":
            config = create_collection_config_for_quanta_x3()
            model_label = "quanta_x3"
        else:
            raise RuntimeError(f"unsupported model {detected!r}; use --model to select a known profile")

        if args.no_master_arm_data:
            config.enable_master_arm_data = False
        if args.model == "auto":
            identity, source = describe_model_source(robot)
            print(f"Client reports model {identity!r} ({source}); profile={model_label}")
        print(f"Connecting to {args.server}; model={model_label}")
        collector = DataCollector(
            robot=robot,
            output_dir=args.output_dir,
            target_hz=args.target_hz,
            collection_config=config,
            image_quality=95,
            downsample_joint_states=True,
            use_video_storage=not args.no_video_storage,
            keep_raw_data=args.keep_raw_data,
        )
        print(f"Data collector ready: output={collector.output_dir}, target_hz={collector.target_hz} Hz")
        print(f"  cameras: {', '.join(config.get_camera_names()) or '(none)'}")
        print(f"  enabled sensors: {', '.join(config.get_enabled_sensors()) or '(none)'}")
        if args.self_check:
            seconds = args.duration if args.duration > 0 else 5.0
            print(f"Self-check: opening all configured streams for {seconds:.1f}s; nothing is saved.")
            report = collector.probe(seconds)
            failed = collector.print_stream_report(report)
            # Master-arm data is optional in the collector's own validation
            # (tracking arms are not present on every bench), so it only warns.
            optional, required = classify_empty_streams(failed)
            if optional:
                print(f"SELF-CHECK WARNING: optional master-arm stream(s) failed or had no frame: "
                      f"{', '.join(optional)}")
            if required:
                print(f"SELF-CHECK FAILED: {len(required)} required stream(s) failed or produced no frame: "
                      f"{', '.join(required)}", flush=True)
                return 3
            print("SELF-CHECK OK: every required enabled stream produced frames without stream errors")
            return 0
        episode_limit = args.episodes if args.episodes > 0 else None
        completed = 0
        while episode_limit is None or completed < episode_limit:
            if args.non_interactive:
                task = f"sdk_v2_episode_{completed:04d}"
            else:
                task = input("Enter task name (Enter uses pick_trash): ").strip() or "pick_trash"
                input("Prepare the robot, then press Enter to start recording...")
            interrupted, _ = record_one(
                collector,
                task=task,
                non_interactive=args.non_interactive,
                duration=args.duration,
            )
            completed += 1
            if interrupted or args.non_interactive or episode_limit is not None:
                if episode_limit is None or completed >= episode_limit or interrupted:
                    break
            if args.non_interactive:
                # A non-interactive run has no console to answer the
                # continuation prompt; --episodes decides the episode count.
                continue
            if input("Continue recording next episode? (y/n, default n): ").strip().lower() != "y":
                break
        print(f"Data collection completed: {collector.episode_count} episode(s) in {collector.output_dir}")
        return 0
    except KeyboardInterrupt:
        print("\nRecording interrupted")
        return 0
    except Exception as error:
        # Keep the failure diagnosable from a single run: the message carries
        # the per-stream RPC error, and the table shows which streams had frames.
        print(f"FAIL: {error}", flush=True)
        if collector is not None:
            try:
                collector.print_stream_report()
            except Exception as report_error:  # noqa: BLE001 - never mask the real failure
                print(f"(per-stream report unavailable: {report_error})", flush=True)
        return 1
    finally:
        # Drain workers even when validation reset is_recording before failing.
        try:
            if collector is not None and collector.is_recording:
                collector.stop_recording()
        finally:
            try:
                if collector is not None:
                    # Cancel every subscription and join its worker before the
                    # shared client closes the connection.  Otherwise the
                    # cancelled streams surface as daemon-thread tracebacks
                    # while the interpreter is finalizing.
                    collector.close()
            finally:
                require(robot.close(), "Robot.close")


if __name__ == "__main__":
    raise SystemExit(main())
