from __future__ import annotations

from PyQt6.QtWidgets import QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QLabel, QLineEdit, QPushButton, QSpinBox, QWidget

from ..controllers.robot_pages import AlignMasterSlaveController
from .base_page import BasePage


class AlignMasterSlavePage(BasePage):
    title = "Align Master-Slave Arms"
    subtitle = "将从臂移动到目标位姿后，再触发主臂对齐流程。"

    PRESETS: dict[str, dict] = {
        "ex001": {
            "slave_server": "10.100.21.60:50051",
            "master_server": "10.100.28.209:50051",
            "slave_arm": "right",
            "align_block": True,
            "align_timeout": 10,
            "note": "EX001 主从臂",
        },
    }

    def __init__(self, main_window) -> None:
        super().__init__(main_window)
        self.controller = AlignMasterSlaveController(self)
        self.preset_combo.currentIndexChanged.connect(self._apply_preset)
        self.step1_button.clicked.connect(self.controller.run_step1)
        self.stop_button.clicked.connect(self.stop_worker)

    def build_content(self) -> QWidget:
        box = QGroupBox("主从臂对齐")
        form = QFormLayout(box)

        self.preset_combo = QComboBox()
        self.preset_combo.addItem("-- 手动输入 --")
        for name, cfg in self.PRESETS.items():
            note = cfg.get("note", "")
            label = f"{name}  {note}" if note else name
            self.preset_combo.addItem(label)
        form.addRow("预设配置", self.preset_combo)

        self.slave_server_input = QLineEdit()
        self.slave_server_input.setPlaceholderText("默认同全局 server")
        self.master_server_input = QLineEdit()
        self.master_server_input.setPlaceholderText("默认同全局 server")

        self.slave_arm_combo = QComboBox()
        self.slave_arm_combo.addItems(["left", "right"])

        self.align_block_check = QCheckBox("阻塞等待对齐完成")
        self.align_block_check.setChecked(True)

        self.align_timeout_spin = QSpinBox()
        self.align_timeout_spin.setRange(1, 60)
        self.align_timeout_spin.setValue(10)
        self.align_timeout_spin.setSuffix(" 秒")

        form.addRow("Slave Server", self.slave_server_input)
        form.addRow("Master Server", self.master_server_input)
        form.addRow("Slave Arm", self.slave_arm_combo)
        form.addRow("", self.align_block_check)
        form.addRow("Timeout", self.align_timeout_spin)

        pose_group = QGroupBox("从臂目标位姿")
        pose_form = QFormLayout(pose_group)
        self.pos_x_spin = self._make_double_spin(-2.0, 2.0, 0.0, "m")
        self.pos_y_spin = self._make_double_spin(-2.0, 2.0, 0.0, "m")
        self.pos_z_spin = self._make_double_spin(-2.0, 2.0, 0.2, "m")
        pose_form.addRow("pos x", self.pos_x_spin)
        pose_form.addRow("pos y", self.pos_y_spin)
        pose_form.addRow("pos z", self.pos_z_spin)
        form.addRow(pose_group)

        step1_label = QLabel("Step 1: 移动从臂到目标位姿，完成后自动断开等待 SDK 自动 stop")
        step1_label.setStyleSheet("font-weight: 600;")
        form.addRow(step1_label)
        self.step1_button = QPushButton("移动从臂")
        form.addRow(self.step1_button)

        form.addRow(self.stop_button)
        return box

    def _apply_preset(self, index: int) -> None:
        if index == 0:
            return
        preset_name = list(self.PRESETS.keys())[index - 1]
        cfg = self.PRESETS[preset_name]
        self.slave_server_input.setText(cfg.get("slave_server", ""))
        self.master_server_input.setText(cfg.get("master_server", ""))
        arm = cfg.get("slave_arm", "left")
        self.slave_arm_combo.setCurrentIndex(0 if arm == "left" else 1)
        self.align_block_check.setChecked(cfg.get("align_block", True))
        self.align_timeout_spin.setValue(cfg.get("align_timeout", 10))

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
