"""Read exoskeleton JointState without entering SDK control mode."""
import argparse

import x2robot_sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("address", help="host:port, for example 192.168.1.10:55051")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--stream", action="store_true")
    parser.add_argument("--count", type=int, default=10)
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")
    connected = x2robot_sdk.connect(args.address)
    if not connected.is_success:
        raise RuntimeError(connected.error)
    with connected.data as robot:
        method = f"get_{args.side}_joint_state"
        if args.stream:
            stream = getattr(robot.exo_hand, method + "_stream")(timeout=5)
            try:
                for _, sample in zip(range(args.count), stream):
                    show(sample)
            finally:
                stream.cancel()
        else:
            show(getattr(robot.exo_hand, method)(timeout=1))


def show(sample):
    if not sample.is_success:
        raise RuntimeError(sample.error)
    state = sample.data
    print("stamp:", state.header.stamp, "frame_id:", state.header.frame_id)
    print("name:", list(state.name), "position:", list(state.position))
    print("velocity:", list(state.velocity), "effort:", list(state.effort))


if __name__ == "__main__":
    main()
