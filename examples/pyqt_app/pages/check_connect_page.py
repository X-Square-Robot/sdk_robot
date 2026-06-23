from __future__ import annotations

from PyQt6.QtWidgets import QFormLayout, QGroupBox, QLabel, QWidget

from ..controllers.robot_pages import CheckConnectController
from .base_page import BasePage


class CheckConnectPage(BasePage):
    title = "Check Connect"
    subtitle = "验证当前 SDK server 是否可达，并查看基础 ping 返回。"

    def __init__(self, main_window) -> None:
        super().__init__(main_window)
        self.controller = CheckConnectController(self)
        self.run_button.clicked.connect(self.controller.run)
        self.stop_button.clicked.connect(self.stop_worker)

    def build_content(self) -> QWidget:
        box = QGroupBox("连接检测")
        form = QFormLayout(box)
        form.addRow("说明", QLabel("对当前 server 执行 ping 检查。"))
        form.addRow(self.build_button_row())
        return box
