from __future__ import annotations

import sys
import traceback
from pathlib import Path

from PyQt6.QtWidgets import QApplication, QMessageBox

from .ui.window import AppWindow

CRASH_LOG = Path.home() / ".x2robot_crash.log"


def _write_crash_log(text: str) -> None:
    try:
        CRASH_LOG.write_text(text)
    except Exception:
        pass


def _install_excepthook(window: AppWindow | None = None) -> None:
    def handle_exception(exc_type, exc_value, exc_traceback) -> None:
        error_text = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        _write_crash_log(error_text.rstrip())
        if window is not None:
            window.append_log(error_text.rstrip())
            QMessageBox.critical(window, "Unhandled error", "出现未处理异常，详细信息已写入日志。")
        else:
            QMessageBox.critical(None, "启动崩溃", f"启动时崩溃：\n{error_text}")

    sys.excepthook = handle_exception


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("X2 Examples GUI")
    app.setStyle("Fusion")

    _install_excepthook()  # 在创建窗口前注册，确保初始化崩溃也能捕获

    window = AppWindow()
    _install_excepthook(window)  # 更新为带窗口引用的版本
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
