"""Cartesian interpolation shared by collection alignment and replay."""

import bisect
import math


class PoseTimeline:
    def __init__(self, samples):
        self.samples = sorted(samples, key=lambda sample: sample[0])
        if not self.samples:
            raise ValueError("empty pose timeline")
        self.times = []
        self.values = []
        for stamp, pose in self.samples:
            p, q = pose["position"], pose["orientation"]
            position = [float(p.get(k, 0.0)) for k in ("x", "y", "z")]
            quaternion = [float(q.get(k, 1.0 if k == "w" else 0.0)) for k in ("x", "y", "z", "w")]
            if not all(math.isfinite(v) for v in [stamp, *position, *quaternion]):
                raise ValueError("non-finite pose or timestamp")
            norm = math.sqrt(sum(v * v for v in quaternion))
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError("invalid pose quaternion length")
            self.times.append(stamp)
            self.values.append((position, [v / norm for v in quaternion]))

    def at(self, stamp):
        index = bisect.bisect_right(self.times, stamp)
        if index == 0:
            position, quaternion = self.values[0]
        elif index == len(self.times):
            position, quaternion = self.values[-1]
        else:
            first, last = index - 1, index
            ratio = (stamp - self.times[first]) / (self.times[last] - self.times[first])
            p0, q0 = self.values[first]
            p1, q1 = self.values[last]
            position = [a + ratio * (b - a) for a, b in zip(p0, p1)]
            dot = sum(a * b for a, b in zip(q0, q1))
            if dot < 0:
                q1, dot = [-v for v in q1], -dot
            if dot > 0.9995:
                quaternion = [a + ratio * (b - a) for a, b in zip(q0, q1)]
            else:
                theta = math.acos(max(-1.0, min(1.0, dot)))
                a = math.sin((1 - ratio) * theta) / math.sin(theta)
                b = math.sin(ratio * theta) / math.sin(theta)
                quaternion = [a * x + b * y for x, y in zip(q0, q1)]
            norm = math.sqrt(sum(v * v for v in quaternion))
            quaternion = [v / norm for v in quaternion]
        return {"position": dict(zip(("x", "y", "z"), position)),
                "orientation": dict(zip(("x", "y", "z", "w"), quaternion))}


def interpolate_poses(samples, timestamps):
    if not samples:
        return []
    timeline = PoseTimeline(samples)
    return [timeline.at(stamp) for stamp in timestamps]
