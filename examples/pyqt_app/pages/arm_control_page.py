from __future__ import annotations

from PyQt6.QtWidgets import QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QLabel, QWidget

from ..controllers.robot_pages import ArmControlController
from .base_page import BasePage


class ArmControlPage(BasePage):
    title = "Arm Control"
    subtitle = "面向单臂操作的控制页，支持 move 和 stream 两种动作。"

    def __init__(self, main_window) -> None:
        super().__init__(main_window)
        self.controller = ArmControlController(self)
        self.run_button.clicked.connect(self.controller.run)
        self.stop_button.clicked.connect(self.stop_worker)
        self.action_combo.currentTextChanged.connect(self._update_pose_visibility)
        self.mode_combo.currentTextChanged.connect(self._update_pose_visibility)
        self._update_pose_visibility()

    def build_content(self) -> QWidget:
        box = QGroupBox("机械臂控制")
        self.form = QFormLayout(box)

        self.action_combo = QComboBox()
        self.action_combo.addItems(["move", "stream"])
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["joint_pos", "end_pose"])
        self.arm_combo = QComboBox()
        self.arm_combo.addItems(["left", "right"])

        self.form.addRow("动作", self.action_combo)
        self.form.addRow("模式", self.mode_combo)
        self.form.addRow("机械臂", self.arm_combo)

        self.pose_group = QGroupBox("目标位姿 (end_pose)")
        pose_form = QFormLayout(self.pose_group)
        self.pos_x_spin = self._make_double_spin(-2.0, 2.0, 0.0, "m")
        self.pos_y_spin = self._make_double_spin(-2.0, 2.0, 0.0, "m")
        self.pos_z_spin = self._make_double_spin(-2.0, 2.0, 0.0, "m")
        self.ori_qx_spin = self._make_double_spin(-1.0, 1.0, -0.0076)
        self.ori_qy_spin = self._make_double_spin(-1.0, 1.0, 0.0868)
        self.ori_qz_spin = self._make_double_spin(-1.0, 1.0, 0.0868)
        self.ori_qw_spin = self._make_double_spin(-1.0, 1.0, 0.9924)
        pose_form.addRow("pos x", self.pos_x_spin)
        pose_form.addRow("pos y", self.pos_y_spin)
        pose_form.addRow("pos z", self.pos_z_spin)
        pose_form.addRow("ori qx", self.ori_qx_spin)
        pose_form.addRow("ori qy", self.ori_qy_spin)
        pose_form.addRow("ori qz", self.ori_qz_spin)
        pose_form.addRow("ori qw", self.ori_qw_spin)
        self.form.addRow(self.pose_group)

        self.form.addRow(QLabel("说明：move 会执行目标轨迹，请确认周围安全。"))
        self.form.addRow(self.build_button_row())
        return box

    def _update_pose_visibility(self) -> None:
        visible = self.action_combo.currentText() == "move" and self.mode_combo.currentText() == "end_pose"
        self.pose_group.setVisible(visible)

    @staticmethod
    def _make_double_spin(min_val: float, max_val: float, default: float, suffix: str = "") -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(6)
        spin.setSingleStep(0.001)
        spin.setValue(default)
        if suffix:
            spin.setSuffix(suffix)
        return spin
