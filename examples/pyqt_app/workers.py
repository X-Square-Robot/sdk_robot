from __future__ import annotations

import traceback
from typing import Callable

from PyQt6.QtCore import QThread, pyqtSignal

from .models.service import ServiceResult


class TaskWorker(QThread):
    started_task = pyqtSignal()
    log_message = pyqtSignal(str)
    task_succeeded = pyqtSignal(object)
    task_failed = pyqtSignal(str)
    task_finished = pyqtSignal()

    def __init__(self, task: Callable[[Callable[[str], None], Callable[[], bool]], ServiceResult]):
        super().__init__()
        self._task = task

    def run(self) -> None:
        self.started_task.emit()
        try:
            result = self._task(self.log_message.emit, self.isInterruptionRequested)
            self.task_succeeded.emit(result)
        except Exception:
            self.task_failed.emit(traceback.format_exc())
        finally:
            self.task_finished.emit()

    def request_stop(self) -> None:
        self.requestInterruption()
