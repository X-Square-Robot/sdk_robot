from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class PageSpec:
    key: str
    label: str
    page_cls: type
    group: str
    badge: str
