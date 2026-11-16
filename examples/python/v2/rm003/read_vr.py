"""Read one VR world/related stream without entering control mode."""
import argparse
import math

import x2robot_sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("address", help="MiniPC host:port, e.g. 192.168.1.10:55051")
    parser.add_argument("--part", choices=("left", "right", "head"), default="head")
    parser.add_argument("--frame", choices=("world", "related"), default="world")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=10,
                        help="total stream deadline in seconds")
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")

    connected = x2robot_sdk.connect(args.address)
    if not connected.is_success:
        raise RuntimeError(connected.error)
    with connected.data as robot:
        vr = getattr(robot, "vr", None)
        if vr is None:
            raise RuntimeError("Connected device does not support VR pose reading")
        part = "head" if args.part == "head" else f"{args.part}_arm"
        related = "_related" if args.frame == "related" else ""
        method = f"get_{part}_pose{related}_stream"
        reader = getattr(vr, method, None)
        if not callable(reader):
            raise RuntimeError(f"Connected device does not support {method}")
        stream = reader(timeout=args.timeout)
        try:
            for index in range(args.count):
                try:
                    sample = next(stream)
                except StopIteration as error:
                    raise RuntimeError(f"VR ended after {index}/{args.count} samples") from error
                if not sample.is_success:
                    raise RuntimeError(f"{sample.error}; request_id={sample.request_id}")
                msg = sample.data
                p, q = msg.pose.position, msg.pose.orientation
                print({"part": args.part, "frame": args.frame,
                       "stamp": [msg.header.stamp.sec, msg.header.stamp.nanosec],
                       "frame_id": msg.header.frame_id,
                       "position": [p.x, p.y, p.z],
                       "quaternion_xyzw": [q.x, q.y, q.z, q.w]})
        finally:
            stream.cancel()


if __name__ == "__main__":
    main()
