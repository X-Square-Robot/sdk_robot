import struct
from typing import Annotated, List, NamedTuple
import typer
from x2robot import connect
from x2robot.sensor_msgs import PointCloud2, PointFieldConstants

# Mapping from PointField data types to Python struct format characters
DATATYPE_TO_STRUCT = {
    PointFieldConstants.INT8: "b",
    PointFieldConstants.UINT8: "B",
    PointFieldConstants.INT16: "h",
    PointFieldConstants.UINT16: "H",
    PointFieldConstants.INT32: "i",
    PointFieldConstants.UINT32: "I",
    PointFieldConstants.FLOAT32: "f",
    PointFieldConstants.FLOAT64: "d",
}

class Point(NamedTuple):
    x: float
    y: float
    z: float

def parse_point_cloud2(msg: PointCloud2) -> List[Point]:
    """
    Parses a PointCloud2 message to extract the coordinates of all points.

    Args:
        msg: The PointCloud2 message.

    Returns:
        A list of Point objects containing all points.
    """
    points = []
    # Find the offset and data type for x, y, z fields
    offsets = {}
    formats = {}
    for field in msg.fields:
        if field.name in ["x", "y", "z"]:
            offsets[field.name] = field.offset
            struct_char = DATATYPE_TO_STRUCT.get(field.datatype)
            if not struct_char:
                raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
            formats[field.name] = struct_char

    if not all(k in offsets for k in ["x", "y", "z"]):
        # If the essential fields are missing, return an empty list.
        # This can happen if the stream sends an empty or metadata-only message.
        print("Warning: Received PointCloud2 message without x, y, and z fields. Skipping.")
        return []
    endian = ">" if msg.is_bigendian else "<"
    byte_data = bytes(msg.data)
    for i in range(msg.width * msg.height):
        point_start = i * msg.point_step
        point_data_bytes = byte_data[point_start : point_start + msg.point_step]

        x = struct.unpack_from(f"{endian}{formats['x']}", point_data_bytes, offsets["x"])[0]
        y = struct.unpack_from(f"{endian}{formats['y']}", point_data_bytes, offsets["y"])[0]
        z = struct.unpack_from(f"{endian}{formats['z']}", point_data_bytes, offsets["z"])[0]

        points.append(Point(x, y, z))

    return points


def display_depth_points(msg: PointCloud2, points: List[Point], max_points_to_show: int = 10):
    """
    Displays the depth point cloud data.

    Args:
        msg: The original PointCloud2 message.
        points: The list of parsed points.
        max_points_to_show: The maximum number of points to display.
    """
    print("\n Depth Point Cloud Data:")
    print("=" * 80)

    # Basic Information
    print(f"  - Dimensions: {msg.width}x{msg.height}")
    print(f"  - Total Points: {len(points)}")
    print(f"  - Is Dense: {'Yes' if msg.is_dense else 'No'}")
    print(f"  - Point Step: {msg.point_step} bytes")
    print(f"  - Row Step: {msg.row_step} bytes")
    print(f"  - Is Big Endian: {'Yes' if msg.is_bigendian else 'No'}")

    # Display the first few points
    if points:
        print(f"\n Displaying first {min(len(points), max_points_to_show)} points (x, y, z):")
        for i, point in enumerate(points[:max_points_to_show]):
            print(f"  - Point {i+1}: ({point.x:.4f}, {point.y:.4f}, {point.z:.4f})")


def main(
    action: Annotated[
        str, typer.Option(help="Action to perform: single, stream")
    ] = "single",
    server: Annotated[
        str, typer.Option(help="Server address, e.g., localhost:50051")
    ] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    print("Connected to the robot, reading depth point cloud data...")
    print("=" * 80)

    try:
        if action == "stream":
            print("Starting depth points stream... Press Ctrl+C to stop.\n")
            for point_cloud_msg in robot.depth_points.get_chassis_depth_points_stream():
                points = parse_point_cloud2(point_cloud_msg)
                display_depth_points(point_cloud_msg, points)
                print("\n" + "-" * 80)
        elif action == "single":
            print("Performing a single read...")
            point_cloud_msg = robot.depth_points.get_chassis_depth_points()
            points = parse_point_cloud2(point_cloud_msg)
            display_depth_points(point_cloud_msg, points)
            print("\n" + "=" * 80)
            print("Single read completed.")
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: single, stream")

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {type(e).__name__}: {e}")


if __name__ == "__main__":
    typer.run(main)
