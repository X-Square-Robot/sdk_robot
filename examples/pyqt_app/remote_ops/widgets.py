from __future__ import annotations

from PyQt6.QtWidgets import QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSpinBox, QVBoxLayout, QWidget

from .models import QUICK_COMMANDS, SSH_PRESETS


class SshBarWidget(QGroupBox):
    def __init__(self) -> None:
        super().__init__("SSH 连接")
        layout = QHBoxLayout(self)

        layout.addWidget(QLabel("预设"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItem("-- 手动 --")
        for preset in SSH_PRESETS:
            self.preset_combo.addItem(preset.label)
        layout.addWidget(self.preset_combo)

        layout.addWidget(QLabel("Host"))
        self.host_input = QLineEdit("10.100.28.209")
        self.host_input.setMaximumWidth(140)
        layout.addWidget(self.host_input)

        layout.addWidget(QLabel("Port"))
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(22)
        self.port_spin.setMaximumWidth(70)
        layout.addWidget(self.port_spin)

        layout.addWidget(QLabel("User"))
        self.user_input = QLineEdit("xr")
        self.user_input.setMaximumWidth(100)
        layout.addWidget(self.user_input)

        layout.addWidget(QLabel("Pwd"))
        self.pwd_input = QLineEdit()
        self.pwd_input.setEchoMode(QLineEdit.EchoMode.Password)
        self.pwd_input.setMaximumWidth(120)
        layout.addWidget(self.pwd_input)

        self.connect_button = QPushButton("连接")
        layout.addWidget(self.connect_button)
        layout.addStretch(1)


class ContainerPanelWidget(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Docker 容器")
        layout = QVBoxLayout(self)
        self.container_combo = QComboBox()
        self.container_combo.setMinimumWidth(180)
        layout.addWidget(self.container_combo)

        self.refresh_button = QPushButton("刷新容器列表")
        layout.addWidget(self.refresh_button)

        self.logs_button = QPushButton("查看容器日志 (tail 200)")
        layout.addWidget(self.logs_button)
        layout.addStretch(1)


class CommandPanelWidget(QGroupBox):
    def __init__(self) -> None:
        super().__init__("命令")
        layout = QVBoxLayout(self)

        toggle_row = QWidget()
        toggle_layout = QHBoxLayout(toggle_row)
        toggle_layout.setContentsMargins(0, 0, 0, 0)
        self.container_exec_cb = QCheckBox("在容器内执行")
        self.container_exec_cb.setChecked(True)
        self.container_exec_cb.setToolTip("勾选后命令会通过 docker exec 在选中容器内执行，取消则在宿主机执行")
        toggle_layout.addWidget(self.container_exec_cb)
        toggle_layout.addWidget(QLabel("source"))
        self.source_input = QLineEdit("source setup.sh")
        self.source_input.setPlaceholderText("可选：容器内前置 source 命令")
        self.source_input.setMaximumWidth(200)
        toggle_layout.addWidget(self.source_input)
        toggle_layout.addStretch(1)
        layout.addWidget(toggle_row)

        cmd_row = QWidget()
        cmd_layout = QHBoxLayout(cmd_row)
        cmd_layout.setContentsMargins(0, 0, 0, 0)
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("输入要在容器内执行的命令…")
        cmd_layout.addWidget(self.cmd_input, 1)
        self.exec_button = QPushButton("执行")
        cmd_layout.addWidget(self.exec_button)
        layout.addWidget(cmd_row)

        quick_label = QLabel("常用命令")
        quick_label.setStyleSheet("font-weight: 600; margin-top: 8px;")
        layout.addWidget(quick_label)

        quick_grid = QWidget()
        quick_layout = QHBoxLayout(quick_grid)
        quick_layout.setContentsMargins(0, 0, 0, 0)
        quick_layout.setSpacing(6)
        self.quick_buttons: list[QPushButton] = []
        for quick_command in QUICK_COMMANDS:
            button = QPushButton(quick_command.label)
            button.setToolTip(f"{'[容器内]' if quick_command.in_container else '[宿主机]'} {quick_command.command}")
            button.setProperty("command", quick_command.command)
            button.setProperty("in_container", quick_command.in_container)
            self.quick_buttons.append(button)
            quick_layout.addWidget(button)
        quick_layout.addStretch(1)
        layout.addWidget(quick_grid)
        layout.addStretch(1)
