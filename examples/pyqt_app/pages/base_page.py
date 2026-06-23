from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt6.QtWidgets import QHBoxLayout, QLabel, QMessageBox, QPushButton, QSizePolicy, QVBoxLayout, QWidget

from ..models.service import ServiceResult
from ..workers import TaskWorker

if TYPE_CHECKING:
    from ..ui.window import AppWindow


class BasePage(QWidget):
    title = ""
    subtitle = ""
    group = "system"

    def __init__(self, main_window: "AppWindow") -> None:
        super().__init__()
        self.main_window = main_window
        self.worker: TaskWorker | None = None
        self.run_button = QPushButton("运行")
        self.stop_button = QPushButton("停止/取消")
        self.stop_button.setEnabled(False)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)

        header = QWidget()
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(4)

        eyebrow = QLabel(self.group.upper())
        eyebrow.setObjectName("pageEyebrow")
        header_layout.addWidget(eyebrow)

        title = QLabel(self.title)
        title.setObjectName("pageTitle")
        header_layout.addWidget(title)

        if self.subtitle:
            subtitle = QLabel(self.subtitle)
            subtitle.setObjectName("pageSubtitle")
            subtitle.setWordWrap(True)
            header_layout.addWidget(subtitle)

        layout.addWidget(header)

        content = self.build_content()
        content.setObjectName("pageBody")
        content.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        layout.addWidget(content)
        layout.addStretch(1)

    def build_content(self) -> QWidget:
        raise NotImplementedError

    def get_server(self) -> str:
        return self.main_window.server_input.text().strip() or "localhost:50051"

    def append_log(self, message: str) -> None:
        self.main_window.append_log(message)

    def set_base_running_state(self, running: bool) -> None:
        self.run_button.setEnabled(not running)
        self.stop_button.setEnabled(running)

    def set_running_state(self, running: bool) -> None:
        self.set_base_running_state(running)

    def start_worker(self, task) -> None:
        if self.worker is not None and self.worker.isRunning():
            return

        self.main_window.log_output.clear()
        self.worker = TaskWorker(task)
        self.worker.started_task.connect(lambda: self.set_running_state(True))
        self.worker.log_message.connect(self.append_log)
        self.worker.task_succeeded.connect(self._handle_success)
        self.worker.task_failed.connect(self._handle_failure)
        self.worker.task_finished.connect(self._handle_finished)
        self.worker.start()

    def stop_worker(self) -> None:
        if self.worker is None:
            return
        self.append_log("Stop requested.")
        self.worker.request_stop()

    def shutdown(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.stop_worker()
            self.worker.wait(2000)

    def build_button_row(self) -> QWidget:
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.run_button)
        layout.addWidget(self.stop_button)
        layout.addStretch(1)
        return row

    def _handle_success(self, result: ServiceResult) -> None:
        self.append_log(result.summary)
        if result.details:
            for detail in result.details:
                self.append_log(detail)

    def _handle_failure(self, error_text: str) -> None:
        self.append_log(error_text.rstrip())
        QMessageBox.critical(self, "Task failed", "任务执行失败，详细错误已写入日志。")

    def _handle_finished(self) -> None:
        if self.sender() is not self.worker:
            return
        self.set_running_state(False)
        self.worker = None
