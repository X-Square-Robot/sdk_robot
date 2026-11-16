"""Check SDK import, device connection and public method availability."""
import argparse
from importlib.metadata import PackageNotFoundError, version

import x2robot_sdk


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("address", help="MiniPC host:port, e.g. 192.168.1.10:55051")
    args = parser.parse_args()
    try:
        sdk_version = version("x2robot-sdk")
    except PackageNotFoundError:
        sdk_version = getattr(x2robot_sdk, "__version__", "source build")
    print("SDK:", sdk_version, "module:", x2robot_sdk.__file__)
    connected = x2robot_sdk.connect(args.address)
    if not connected.is_success:
        raise RuntimeError(connected.error)
    with connected.data as robot:
        methods = {
            "exo_hand": ["get_left_joint_state", "get_right_joint_state",
                         "get_left_joint_state_stream", "get_right_joint_state_stream",
                         "get_left_camera_stream", "get_right_camera_stream"],
            "vr": ["get_left_arm_pose_stream", "get_right_arm_pose_stream",
                   "get_head_pose_stream", "get_left_arm_pose_related_stream",
                   "get_right_arm_pose_related_stream", "get_head_pose_related_stream"],
        }
        for service, names in methods.items():
            handle = getattr(robot, service, None)
            if handle is None:
                raise RuntimeError(f"Connected device does not support {service}")
            for name in names:
                if not callable(getattr(handle, name, None)):
                    raise RuntimeError(f"SDK is missing {service}.{name}")
        print("Device connection and 12 local SDK methods: OK")


if __name__ == "__main__":
    main()
