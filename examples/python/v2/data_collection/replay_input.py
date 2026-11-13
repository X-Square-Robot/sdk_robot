"""Read replay observations from the C++ append-only protobuf format."""

import struct

from x2robot.geometry_msgs import PoseStamped
from x2robot.nav_msgs import Odometry
from x2robot.sensor_msgs import JointState
from x2robot.sdk import GripperPosition


def pose_dict(pose):
    if pose is None or pose.position is None or pose.orientation is None:
        raise ValueError("recorded pose is missing position or orientation")
    return {"position": {k: getattr(pose.position, k) for k in ("x", "y", "z")},
            "orientation": {k: getattr(pose.orientation, k) for k in ("x", "y", "z", "w")}}


def load_protobuf_frames(directory):
    records = []
    for path in sorted(directory.glob("stream_*.pb")):
        name = path.stem[len("stream_"):]
        if name.endswith("joint_states"):
            message_type = JointState
        elif name.endswith("end_pose"):
            message_type = PoseStamped
        elif name in ("left_gripper_position", "right_gripper_position"):
            message_type = GripperPosition
        elif name == "odometry":
            message_type = Odometry
        else:
            continue
        with path.open("rb") as source:
            while header := source.read(12):
                if len(header) != 12:
                    raise ValueError(f"{name}: truncated record header")
                stamp, size = struct.unpack("<QI", header)
                if size > 256 * 1024 * 1024:
                    raise ValueError(f"{name}: oversized protobuf record")
                payload = source.read(size)
                if len(payload) != size:
                    raise ValueError(f"{name}: truncated record payload")
                try:
                    message = message_type.FromString(payload)
                except Exception as error:
                    raise ValueError(f"{name}: invalid protobuf record") from error
                if message_type is JointState:
                    value = {"positions": list(message.position)}
                elif message_type is PoseStamped:
                    value = pose_dict(message.pose)
                elif message_type is Odometry:
                    value = {"pose": pose_dict(message.pose.pose if message.pose is not None else None)}
                else:
                    value = {"position": message.position}
                records.append((stamp, name, value))
    records.sort(key=lambda record: record[0])
    if not records:
        raise ValueError("episode has no replay observations")
    first = records[0][0]
    return [{"timestamp": (stamp - first) / 1e9, "observation": {name: value}}
            for stamp, name, value in records]
