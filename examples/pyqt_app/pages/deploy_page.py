from __future__ import annotations

from PyQt6.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget

from ..deploy_ops import DeployController
from ..deploy_ops.widgets import DeploySshBar, DeploySourcePanel, DeployStepsPanel, DeployTargetPanel
from .base_page import BasePage


class DeployPage(BasePage):
    title = "Deploy"
    subtitle = "将本地编译产物 rsync 到机器人并替换容器内文件，分步执行可观察日志。"
    group = "ops"

    def __init__(self, main_window) -> None:
        super().__init__(main_window)
        self.controller = DeployController(self)

        self.ssh_bar.connect_button.clicked.connect(self.controller.on_connect_toggle)
        self.ssh_bar.preset_combo.currentIndexChanged.connect(self.controller.apply_preset)
        self.source_panel.browse_button.clicked.connect(self.controller.browse_source_dir)
        self.source_panel.scan_button.clicked.connect(self.controller.scan_subdirs)
        self.target_panel.refresh_button.clicked.connect(self.controller.refresh_containers)
        self.steps_panel.rsync_button.clicked.connect(self.controller.do_rsync)
        self.steps_panel.replace_button.clicked.connect(self.controller.do_container_replace)
        self.steps_panel.logs_button.clicked.connect(self.controller.do_view_logs)
        self.stop_button.clicked.connect(self.stop_worker)

    def build_content(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.ssh_bar = DeploySshBar()
        layout.addWidget(self.ssh_bar)

        panels = QWidget()
        panels_layout = QHBoxLayout(panels)
        panels_layout.setContentsMargins(0, 0, 0, 0)
        panels_layout.setSpacing(12)

        self.source_panel = DeploySourcePanel()
        self.target_panel = DeployTargetPanel()
        panels_layout.addWidget(self.source_panel, 1)
        panels_layout.addWidget(self.target_panel, 1)
        layout.addWidget(panels)

        self.steps_panel = DeployStepsPanel()
        layout.addWidget(self.steps_panel)

        return root

    def set_running_state(self, running: bool) -> None:
        self.controller.set_running_state(running)

    def stop_worker(self) -> None:
        self.controller.stop()
        super().stop_worker()

    def shutdown(self) -> None:
        self.controller.shutdown()
        super().shutdown()
