"""TF bridge example: static, stream and tree actions (v1 behavior)."""

from __future__ import annotations

import argparse
import threading
import time

import x2robot_sdk

COLLECT_SECONDS = 3.0


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {(error.get('error_message') or 'SDK call failed')} (request_id={result.request_id or '-'})")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def format_transform(value):
    translation = value.transform.translation
    rotation = value.transform.rotation
    return (
        f"{value.header.frame_id} -> {value.child_frame_id} "
        f"t=({translation.x:.4f}, {translation.y:.4f}, {translation.z:.4f}) "
        f"q=({rotation.x:.4f}, {rotation.y:.4f}, {rotation.z:.4f}, {rotation.w:.4f})"
    )


def collect_for(stream_fn, seconds, on_message):
    stop = threading.Event()
    errors = []

    def worker():
        stream = None
        try:
            stream = stream_fn(timeout=None)
            for result in stream:
                if stop.is_set():
                    break
                on_message(require(result, "TfBridge stream"))
        except Exception as error:
            errors.append(error)
        finally:
            if stream is not None:
                close_stream(stream)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    time.sleep(max(0.1, seconds))
    stop.set()
    thread.join(timeout=2.0)
    if errors:
        raise errors[0]


def show_static(robot, seconds=COLLECT_SECONDS):
    edges = {}
    transforms = {}
    collect_for(robot.tf.subscribe_tf_static, seconds, lambda message: transforms.update({item.child_frame_id: item for item in message.transforms}))
    for value in transforms.values():
        print(f"[static] {format_transform(value)}")
    print(f"Received {len(transforms)} static transforms.")


def show_stream(robot, seconds=None):
    stream = robot.tf.subscribe_tf(timeout=None if seconds is None else max(4.0, seconds + 1.0))
    end = None if seconds is None else time.monotonic() + max(0.1, seconds)
    try:
        for result in stream:
            message = require(result, "TfBridge.SubscribeTf")
            for value in message.transforms:
                print(f"[tf] {format_transform(value)}")
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(stream)


def show_tree(robot, seconds=COLLECT_SECONDS):
    edges = {}
    threads = [
        threading.Thread(target=collect_for, args=(robot.tf.subscribe_tf_static, seconds, lambda m: edges.update({t.child_frame_id: t.header.frame_id for t in m.transforms})), daemon=True),
        threading.Thread(target=collect_for, args=(robot.tf.subscribe_tf, seconds, lambda m: edges.update({t.child_frame_id: t.header.frame_id for t in m.transforms})), daemon=True),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    children = {}
    for child, parent in edges.items():
        children.setdefault(parent, []).append(child)
    roots = [parent for parent in children if parent not in edges]

    def render(frame, prefix=""):
        values = sorted(children.get(frame, []))
        for index, child in enumerate(values):
            last = index == len(values) - 1
            print(f"{prefix}{'└── ' if last else '├── '}{child}")
            render(child, prefix + ("    " if last else "│   "))

    for root in sorted(roots):
        print(root)
        render(root)
    print(f"Total frames: {len(set(edges) | set(children))}, roots: {sorted(roots)}")


def verify(robot, _model="auto"):
    show_static(robot, 0.5)
    dynamic = []
    collect_for(robot.tf.subscribe_tf, 0.5, dynamic.append)
    if not dynamic:
        raise AssertionError("TF dynamic stream returned no messages")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--action", choices=("static", "stream", "tree", "verify"), default="static")
    parser.add_argument("--seconds", type=float, default=None,
                        help="stream duration; omit for Ctrl+C (static/tree use 3s by default)")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "static":
            show_static(robot, COLLECT_SECONDS if args.seconds is None else args.seconds)
        elif args.action == "stream":
            show_stream(robot, args.seconds)
        elif args.action == "tree":
            show_tree(robot, COLLECT_SECONDS if args.seconds is None else args.seconds)
        else:
            verify(robot)
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
