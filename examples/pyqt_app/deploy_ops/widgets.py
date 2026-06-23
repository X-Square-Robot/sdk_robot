from __future__ import annotations

import os

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .models import DEPLOY_PRESETS


class DeploySshBar(QGroupBox):
    def __init__(self) -> None:
        super().__init__("SSH 连接")
        layout = QHBoxLayout(self)

        layout.addWidget(QLabel("预设"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItem("-- 手动 --")
        for preset in DEPLOY_PRESETS:
            self.preset_combo.addItem(preset.label)
        layout.addWidget(self.preset_combo)

        layout.addWidget(QLabel("Host"))
        self.host_input = QLineEdit("10.100.20.101")
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


class DeploySourcePanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("部署源（本地）")
        layout = QVBoxLayout(self)

        dir_row = QWidget()
        dir_layout = QHBoxLayout(dir_row)
        dir_layout.setContentsMargins(0, 0, 0, 0)
        dir_layout.addWidget(QLabel("本地目录"))
        self.dir_input = QLineEdit("/home/shangyizhou/code/real_ws/install")
        dir_layout.addWidget(self.dir_input, 1)
        self.browse_button = QPushButton("浏览…")
        dir_layout.addWidget(self.browse_button)
        layout.addWidget(dir_row)

        self.subdir_list = QListWidget()
        self.subdir_list.setMaximumHeight(160)
        layout.addWidget(self.subdir_list)

        scan_row = QWidget()
        scan_layout = QHBoxLayout(scan_row)
        scan_layout.setContentsMargins(0, 0, 0, 0)
        self.scan_button = QPushButton("扫描子目录")
        scan_layout.addWidget(self.scan_button)
        scan_layout.addStretch(1)
        layout.addWidget(scan_row)


class DeployTargetPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("部署目标（容器）")
        layout = QVBoxLayout(self)

        layout.addWidget(QLabel("容器名"))
        container_row = QWidget()
        container_layout = QHBoxLayout(container_row)
        container_layout.setContentsMargins(0, 0, 0, 0)
        self.container_combo = QComboBox()
        self.container_combo.setEditable(True)
        self.container_combo.setCurrentText("ex001_master-sdk_server-1")
        container_layout.addWidget(self.container_combo, 1)
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setToolTip("通过 SSH 查询 docker ps -a 获取容器列表")
        self.refresh_button.setMaximumWidth(60)
        container_layout.addWidget(self.refresh_button)
        layout.addWidget(container_row)

        layout.addWidget(QLabel("容器内路径"))
        self.container_path_input = QLineEdit("/opt/xr/bot")
        layout.addWidget(self.container_path_input)

        layout.addWidget(QLabel("Host 临时目录"))
        self.temp_dir_input = QLineEdit("/home/xr/temp")
        layout.addWidget(self.temp_dir_input)

        layout.addStretch(1)


class DeployStepsPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("部署步骤")
        layout = QHBoxLayout(self)

        self.rsync_button = QPushButton("Step 1: Rsync 同步")
        self.rsync_button.setToolTip("将本地选中的子目录通过 rsync 拷贝到机器人 Host 临时目录")
        layout.addWidget(self.rsync_button)

        layout.addWidget(QLabel("→"))

        self.replace_button = QPushButton("Step 2: 容器替换")
        self.replace_button.setToolTip("停止容器 → 删除旧文件 → docker cp 新文件 → 重启容器")
        layout.addWidget(self.replace_button)

        layout.addWidget(QLabel("→"))

        self.logs_button = QPushButton("Step 3: 查看日志")
        self.logs_button.setToolTip("查看容器最新 200 行日志")
        layout.addWidget(self.logs_button)

        layout.addStretch(1)
