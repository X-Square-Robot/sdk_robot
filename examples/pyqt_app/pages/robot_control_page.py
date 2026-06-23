from __future__ import annotations

from PyQt6.QtWidgets import QComboBox, QFormLayout, QGroupBox, QWidget

from ..controllers.robot_pages import RobotControlController
from .base_page import BasePage


class RobotControlPage(BasePage):
    title = "Robot Control"
    subtitle = "执行 stop、recover、homing 等基础系统控制动作。"

    def __init__(self, main_window) -> None:
        super().__init__(main_window)
        self.controller = RobotControlController(self)
        self.run_button.clicked.connect(self.controller.run)
        self.stop_button.clicked.connect(self.stop_worker)

    def build_content(self) -> QWidget:
        box = QGroupBox("机器人控制")
        form = QFormLayout(box)
        self.action_combo = QComboBox()
        self.action_combo.addItems(["homing", "stop", "recover"])
        form.addRow("动作", self.action_combo)
        form.addRow(self.build_button_row())
        return box
