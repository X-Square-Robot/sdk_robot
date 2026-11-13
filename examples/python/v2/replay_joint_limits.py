"""Shared joint-command bounds for ordinary and Recording-v2 replay."""

import configparser
import math
from pathlib import Path
import struct


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config/replay_joint_limits.conf"
JOINT_STREAMS = {"left_arm_joint_states", "right_arm_joint_states", "waist_joint_states"}


def applies(stream, mode):
    return stream in {"head_joint_states", "lift_joint_states"} or (mode == "joint_pos" and stream in JOINT_STREAMS)


def _float32_inside(value, lower):
    """Nearest float32 endpoint inside a double-precision configured interval."""
    rounded = struct.unpack("!f", struct.pack("!f", value))[0]
    if not math.isfinite(rounded):
        raise ValueError("joint bound is not representable as float32")
    if (lower and rounded < value) or (not lower and rounded > value):
        bits = struct.unpack("!I", struct.pack("!f", rounded))[0]
        if rounded == 0:
            bits = 1 if lower else 0x80000001
        else:
            bits += 1 if (rounded > 0) == lower else -1
        rounded = struct.unpack("!f", struct.pack("!I", bits))[0]
    return rounded


class JointLimits:
    def __init__(self, model, path=DEFAULT_CONFIG):
        self.model = model
        self.bounds = {}
        config = configparser.ConfigParser(interpolation=None)
        with Path(path).open(encoding="utf-8") as source:
            config.read_file(source)
        if not config.sections():
            raise ValueError("empty joint limits configuration")
        for section in config.sections():
            entry = config[section]
            if set(entry) != {"lower", "upper", "wire"} or entry["wire"] not in {"float32", "float64"}:
                raise ValueError(f"invalid joint limit configuration: {section}")
            lower = [float(v.strip()) for v in entry["lower"].split(",")]
            upper = [float(v.strip()) for v in entry["upper"].split(",")]
            if (
                not lower
                or len(lower) != len(upper)
                or any(not math.isfinite(lo) or not math.isfinite(hi) or lo > hi for lo, hi in zip(lower, upper))
            ):
                raise ValueError(f"invalid joint bounds: {section}")
            if entry["wire"] == "float32":
                lower = [_float32_inside(v, True) for v in lower]
                upper = [_float32_inside(v, False) for v in upper]
            if any(lo > hi for lo, hi in zip(lower, upper)):
                raise ValueError(f"joint interval has no representable command: {section}")
            self.bounds[section] = tuple(zip(lower, upper))

    def clamp(self, stream, values):
        bounds = self.bounds.get(f"{self.model}.{stream}")
        if bounds is None:
            raise ValueError(f"No joint command limits for model={self.model!r} stream={stream}")
        if not isinstance(values, (list, tuple)) or len(values) != len(bounds):
            raise ValueError(f"{stream}: expected {len(bounds)} finite joint positions")
        values = [float(value) for value in values]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{stream}: joint positions must be finite")
        return [max(lo, min(hi, value)) for value, (lo, hi) in zip(values, bounds)]


def prepare_joint_commands(data, model, mode, path=DEFAULT_CONFIG):
    """Validate before switching modes; copy only observations that need clipping.

    Unused joint streams and Cartesian poses are untouched. Statistics cover
    the complete plan, not commands acknowledged by the robot.
    """
    if mode not in {"joint_pos", "end_pose"}:
        raise ValueError("mode must be joint_pos or end_pose")
    limits = JointLimits(model, path)
    frames, summaries = [], {}
    for index, frame in enumerate(data.get("frames", [])):
        observation = frame.get("observation", {})
        changes = {}
        for stream, state in observation.items():
            if not applies(stream, mode):
                continue
            try:
                values = state.get("positions") if isinstance(state, dict) else None
                clipped = limits.clamp(stream, values)
            except (ValueError, TypeError, OverflowError) as error:
                raise ValueError(f"Joint replay frame={index} timestamp={frame.get('timestamp')}: {error}") from error
            row = summaries.setdefault(stream, dict(records=0, clamped_records=0, joints_clamped=[0] * len(clipped)))
            row["records"] += 1
            changed = [target != float(value) for target, value in zip(clipped, values)]
            row["joints_clamped"] = [count + int(hit) for count, hit in zip(row["joints_clamped"], changed)]
            if any(changed):
                row["clamped_records"] += 1
                changes[stream] = {**state, "positions": clipped}
        frames.append({**frame, "observation": {**observation, **changes}} if changes else frame)
    return {**data, "frames": frames}, dict(model=model, streams=summaries)
