from __future__ import annotations

from PyQt6.QtWidgets import QGroupBox, QHBoxLayout, QVBoxLayout, QWidget

from ..remote_ops import RemoteOpsController
from ..remote_ops.widgets import CommandPanelWidget, ContainerPanelWidget, SshBarWidget
from .base_page import BasePage


class RemoteOpsPage(BasePage):
    title = "Remote Ops"
    subtitle = "通过 SSH 对远端主机和 Docker 容器做运维排查与命令执行。"
    group = "ops"

    def __init__(self, main_window) -> None:
        super().__init__(main_window)
        self.controller = RemoteOpsController(self)
        self.ssh_bar.connect_button.clicked.connect(self.controller.on_connect_toggle)
        self.ssh_bar.preset_combo.currentIndexChanged.connect(self.controller.apply_preset)
        self.container_panel.refresh_button.clicked.connect(self.controller.refresh_containers)
        self.container_panel.logs_button.clicked.connect(self.controller.show_container_logs)
        self.command_panel.cmd_input.returnPressed.connect(self.controller.exec_command_from_input)
        self.command_panel.exec_button.clicked.connect(self.controller.exec_command_from_input)
        for button in self.command_panel.quick_buttons:
            command = button.property("command")
            in_container = bool(button.property("in_container"))
            button.clicked.connect(lambda _checked, c=command, ic=in_container: self.controller.run_quick_command(c, ic))
        self.stop_button.clicked.connect(self.stop_worker)

    def build_content(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.ssh_bar = SshBarWidget()
        layout.addWidget(self.ssh_bar)

        panel = QWidget()
        panel_layout = QHBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(12)

        self.container_panel = ContainerPanelWidget()
        self.command_panel = CommandPanelWidget()
        panel_layout.addWidget(self.container_panel)
        panel_layout.addWidget(self.command_panel, 1)

        layout.addWidget(panel)
        return root

    def set_running_state(self, running: bool) -> None:
        self.controller.set_running_state(running)

    def stop_worker(self) -> None:
        self.controller.stop()
        super().stop_worker()

    def shutdown(self) -> None:
        self.controller.shutdown()
        super().shutdown()
