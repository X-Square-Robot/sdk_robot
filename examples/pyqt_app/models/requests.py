from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class ArmControlRequest:
    server: str
    action: str
    mode: str
    arm: str
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_qx: float = -0.0076
    target_qy: float = 0.0868
    target_qz: float = 0.0868
    target_qw: float = 0.9924
