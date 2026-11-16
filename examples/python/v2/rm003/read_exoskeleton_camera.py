"""Save encoded frames from an ExoHand camera without entering control mode."""
import argparse
import json
import math
from pathlib import Path

import x2robot_sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("address", help="host:port, for example 192.168.1.10:55051")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=5, help="total stream deadline in seconds")
    parser.add_argument("--output", type=Path, default=Path("exo_camera_frames"))
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    connected = x2robot_sdk.connect(args.address)
    if not connected.is_success:
        raise RuntimeError(connected.error)
    with connected.data as robot:
        stream = getattr(robot.exo_hand, f"get_{args.side}_camera_stream")(timeout=args.timeout)
        try:
            for index in range(args.count):
                try:
                    sample = next(stream)
                except StopIteration as error:
                    raise RuntimeError(f"camera ended after {index}/{args.count} frames") from error
                if not sample.is_success:
                    raise RuntimeError(sample.error)
                image = sample.data
                if not image.data:
                    raise RuntimeError("camera returned an empty payload")
                codec = image.format.lower()
                if "jpeg" in codec or "jpg" in codec:
                    suffix = ".jpg"
                elif "png" in codec:
                    suffix = ".png"
                else:
                    raise RuntimeError(f"expected JPEG/PNG, received format {image.format!r}")
                args.output.mkdir(parents=True, exist_ok=True)
                path = args.output / f"{args.side}_{index:04d}{suffix}"
                path.write_bytes(image.data)
                metadata = {"frame_id": image.header.frame_id, "sec": image.header.stamp.sec,
                            "nanosec": image.header.stamp.nanosec, "format": image.format,
                            "bytes": len(image.data)}
                path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
                print(path, metadata)
        finally:
            stream.cancel()


if __name__ == "__main__":
    main()
