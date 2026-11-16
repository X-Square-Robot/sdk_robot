"""Decode and display PointCloud2 samples without SDK dependencies."""

from __future__ import annotations

import struct
import warnings
from collections import namedtuple

Point = namedtuple("Point", "x y z")
DATATYPE_TO_STRUCT = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}


def parse_point_cloud2(message):
    fields = {}
    for field in message.fields:
        if field.name not in ("x", "y", "z"):
            continue
        code = DATATYPE_TO_STRUCT.get(field.datatype)
        if (field.name in fields or field.count != 1 or code is None
                or field.offset < 0 or field.offset + struct.calcsize(code) > message.point_step):
            raise ValueError("PointCloud2 parse failed: invalid coordinate field")
        fields[field.name] = field
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 parse failed: missing coordinates")
    endian = ">" if message.is_bigendian else "<"
    raw = bytes(message.data)
    row_bytes = message.width * message.point_step
    row_step = message.row_step
    if message.height == 1 and row_bytes == len(raw) and row_step != row_bytes:
        warnings.warn(f"PointCloud2: single-row packed data uses {row_bytes} bytes; "
                      f"ignoring stale row_step={row_step}", RuntimeWarning, stacklevel=2)
        row_step = row_bytes
    if (message.width <= 0 or message.height <= 0 or message.point_step <= 0
            or row_step < row_bytes or row_step * message.height != len(raw)):
        raise ValueError("PointCloud2 parse failed: invalid dimensions or byte count")
    points = []
    for index in range(message.width * message.height):
        row, column = divmod(index, message.width)
        start = row * row_step + column * message.point_step
        values = []
        for name in ("x", "y", "z"):
            field = fields[name]
            code = DATATYPE_TO_STRUCT.get(field.datatype)
            if code is None:
                raise ValueError(f"unsupported PointField datatype: {field.datatype}")
            values.append(struct.unpack_from(f"{endian}{code}", raw, start + field.offset)[0])
        points.append(Point(*values))
    return points


def display_depth_points(message, points, max_points_to_show=10):
    print("\nDepth Point Cloud Data:")
    print("=" * 80)
    print(f"  - Dimensions: {message.width}x{message.height}")
    print(f"  - Total Points: {len(points)}")
    print(f"  - Is Dense: {'Yes' if message.is_dense else 'No'}")
    print(f"  - Point Step: {message.point_step} bytes")
    print(f"  - Row Step: {message.row_step} bytes")
    print(f"  - Is Big Endian: {'Yes' if message.is_bigendian else 'No'}")
    for index, point in enumerate(points[:max_points_to_show]):
        print(f"  - Point {index + 1}: ({point.x:.4f}, {point.y:.4f}, {point.z:.4f})")
