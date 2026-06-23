"""TF bridge example.

Consume the robot's raw /tf and /tf_static streams via the SDK
(robot.tf.subscribe_tf() / robot.tf.subscribe_tf_static()).

Each stream yields tf2_msgs.TfMessage frames; every frame carries a list of
geometry_msgs.TransformStamped (one per parent->child edge).

Actions:
  static  Print the full set of latched static transforms, then exit.
  stream  Continuously print dynamic transforms until Ctrl+C.
  tree    Build and print the full TF tree topology.
"""

import threading
import time
from typing import Annotated

import typer

from x2robot import connect

# Default window (seconds) to accumulate transforms. /tf and /tf_static are
# published by MULTIPLE nodes (robot_state_publisher, each camera, hal, ...),
# each sending its own TfMessage. A single frame only carries one publisher's
# transforms, so we must accumulate over a short window to see the whole tree.
COLLECT_SECONDS = 3.0


def collect_for(stream_fn, seconds: float, on_message):
    """Consume a server stream in a background thread for `seconds`, calling
    on_message(tf_msg) for each frame received within the window."""
    stop = threading.Event()

    def worker():
        try:
            for tf_msg in stream_fn():
                if stop.is_set():
                    break
                on_message(tf_msg)
        except Exception:
            pass

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    time.sleep(seconds)
    stop.set()


def format_transform(t) -> str:
    tr = t.transform.translation
    rot = t.transform.rotation
    return (
        f"{t.header.frame_id} -> {t.child_frame_id}  "
        f"t=({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f})  "
        f"q=({rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f})"
    )


def print_frame(tf_msg, label: str):
    for t in tf_msg.transforms:
        print(f"[{label}] {format_transform(t)}")


def collect_edges(tf_msg, edges: dict):
    for t in tf_msg.transforms:
        edges[t.child_frame_id] = t.header.frame_id


# --------------------------------------------------------------------------- #
# Pure-SDK actions (no ROS dependency)
# --------------------------------------------------------------------------- #
def show_static(robot):
    """Collect static transforms over a short window.

    /tf_static is latched but published by multiple nodes, each as its own
    message, so we accumulate for a few seconds to capture all of them.
    """
    print(f"Collecting static transforms (/tf_static) for {COLLECT_SECONDS:g}s...")
    print("=" * 80)
    edges = {}  # child -> TransformStamped (dedup by child, keep latest)
    collect_for(
        robot.tf.subscribe_tf_static,
        COLLECT_SECONDS,
        lambda msg: edges.update({t.child_frame_id: t for t in msg.transforms}),
    )
    for t in edges.values():
        print(f"[static] {format_transform(t)}")
    print("=" * 80)
    print(f"Received {len(edges)} static transforms.")


def show_stream(robot):
    print("Streaming dynamic transforms (/tf)... Press Ctrl+C to stop.")
    print("=" * 80)
    for tf_msg in robot.tf.subscribe_tf():
        print_frame(tf_msg, "tf")


def show_tree(robot):
    print(f"Building TF tree from /tf_static + /tf (collecting {COLLECT_SECONDS:g}s)...")
    edges: dict = {}  # child_frame -> parent_frame

    # Both topics have multiple publishers; accumulate over a window so every
    # publisher's transforms are captured. Run the two streams concurrently.
    t_static = threading.Thread(
        target=collect_for,
        args=(robot.tf.subscribe_tf_static, COLLECT_SECONDS, lambda m: collect_edges(m, edges)),
        daemon=True,
    )
    t_dynamic = threading.Thread(
        target=collect_for,
        args=(robot.tf.subscribe_tf, COLLECT_SECONDS, lambda m: collect_edges(m, edges)),
        daemon=True,
    )
    t_static.start()
    t_dynamic.start()
    t_static.join()
    t_dynamic.join()

    if not edges:
        print("No transforms received.")
        return

    children: dict = {}
    for child, parent in edges.items():
        children.setdefault(parent, []).append(child)
    roots = [p for p in children if p not in edges]

    def render(frame: str, prefix: str = ""):
        kids = sorted(children.get(frame, []))
        for i, child in enumerate(kids):
            last = i == len(kids) - 1
            branch = "└── " if last else "├── "
            child_prefix = "    " if last else "│   "
            print(f"{prefix}{branch}{child}")
            render(child, prefix + child_prefix)

    print("=" * 80)
    for root in sorted(roots):
        print(root)
        render(root)
    print("=" * 80)
    total_frames = len(set(edges.keys()) | set(children.keys()))
    print(f"Total frames: {total_frames}, roots: {sorted(roots)}")


def main(
    action: Annotated[
        str, typer.Option(help="Action: static, stream, tree")
    ] = "static",
    server: Annotated[
        str, typer.Option(help="Server address, e.g., localhost:50051")
    ] = "localhost:50051",
):
    robot = connect(f"x2://{server}")
    print("Connected to robot.")

    try:
        if action == "static":
            show_static(robot)
        elif action == "stream":
            show_stream(robot)
        elif action == "tree":
            show_tree(robot)
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: static, stream, tree")
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {type(e).__name__}: {e}")


if __name__ == "__main__":
    typer.run(main)
