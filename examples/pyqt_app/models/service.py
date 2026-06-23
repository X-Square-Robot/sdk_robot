from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class ServiceResult:
    success: bool
    summary: str
    details: list[str] = field(default_factory=list)
