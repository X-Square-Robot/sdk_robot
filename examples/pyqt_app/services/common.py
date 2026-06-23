from __future__ import annotations

from typing import Callable

from ..models.service import ServiceResult

LogFn = Callable[[str], None]
CancelFn = Callable[[], bool]

__all__ = ["CancelFn", "LogFn", "ServiceResult"]
