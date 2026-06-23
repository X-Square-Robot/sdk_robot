from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QButtonGroup, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QRadioButton, QVBoxLayout, QWidget

from ..models.service import ServiceResult
from ..services.robot_info import run_get_model_type, run_set_control_mode, run_stream_query, run_unary_query
from .base_page import BasePage


class RobotInfoPage(BasePage):
    title = "Robot Info"
    subtitle = "通过 gRPC 查询 / 设置 Master 臂的控制模式、关节状态、末端位姿和夹爪位置。"
    group = "system"

    def __init__(self, main_window) -> None:
        self._streaming_method: str | None = None
        self.action_buttons: list[QPushButton] = []
        super().__init__(main_window)
        self.run_button.setVisible(False)
        self.stop_button.clicked.connect(self.stop_worker)

    def build_content(self) -> QWidget:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        # ── arm selection ──────────────────────────────────────
        arm_group = QGroupBox("Arm Selection")
        arm_layout = QHBoxLayout(arm_group)
        self.arm_group = QButtonGroup(self)
        left_radio = QRadioButton("Master Left Arm")
        right_radio = QRadioButton("Master Right Arm")
        left_radio.setChecked(True)
        self.arm_group.addButton(left_radio, 0)
        self.arm_group.addButton(right_radio, 1)
        arm_layout.addWidget(left_radio)
        arm_layout.addWidget(right_radio)
        arm_layout.addStretch(1)
        layout.addWidget(arm_group)

        # ── system queries ──────────────────────────────────────
        sys_group = QGroupBox("System Queries")
        sys_layout = QHBoxLayout(sys_group)
        sys_btn = QPushButton("GetModelType")
        sys_btn.setToolTip("获取机器人型号（不依赖 application node）")
        sys_btn.clicked.connect(self._do_get_model_type)
        self.action_buttons.append(sys_btn)
        sys_layout.addWidget(sys_btn)
        sys_layout.addStretch(1)
        layout.addWidget(sys_group)

        # ── unary queries ──────────────────────────────────────
        unary_group = QGroupBox("Unary Queries")
        unary_layout = QGridLayout(unary_group)
        unary_layout.setSpacing(8)

        unary_methods = [
            ("GetControlMode", "查询当前控制模式"),
            ("GetJointStates", "查询关节状态"),
            ("GetEndPose", "查询末端位姿"),
            ("GetGripperPosition", "查询夹爪开度"),
        ]
        for col, (method, tooltip) in enumerate(unary_methods):
            btn = QPushButton(method)
            btn.setToolTip(tooltip)
            btn.clicked.connect(lambda _checked, m=method: self._do_unary(m))
            self.action_buttons.append(btn)
            unary_layout.addWidget(btn, 0, col)
        layout.addWidget(unary_group)

        # ── stream queries ─────────────────────────────────────
        stream_group = QGroupBox("Stream Queries")
        stream_layout = QGridLayout(stream_group)
        stream_layout.setSpacing(8)

        stream_methods = [
            "GetJointStatesStream",
            "GetEndPoseStream",
            "GetGripperStateStream",
        ]
        for col, method in enumerate(stream_methods):
            btn = QPushButton("▶ " + method)
            btn.setToolTip(f"持续监听 {method}")
            btn.clicked.connect(lambda _checked, m=method: self._do_stream(m))
            self.action_buttons.append(btn)
            stream_layout.addWidget(btn, 0, col)

        layout.addWidget(stream_group)

        # ── set control mode ───────────────────────────────────
        set_group = QGroupBox("Set Control Mode")
        set_layout = QHBoxLayout(set_group)
        set_layout.setSpacing(8)

        set_btn_13 = QPushButton("Set ENDPOSE_TELEOP (mode=13)")
        set_btn_13.setToolTip("切换到末端位姿遥操模式")
        set_btn_13.clicked.connect(lambda: self._do_set_control_mode(13))
        self.action_buttons.append(set_btn_13)
        set_layout.addWidget(set_btn_13)

        set_btn_14 = QPushButton("Set JOINT_TELEOP (mode=14)")
        set_btn_14.setToolTip("切换到关节遥操模式")
        set_btn_14.clicked.connect(lambda: self._do_set_control_mode(14))
        self.action_buttons.append(set_btn_14)
        set_layout.addWidget(set_btn_14)

        set_layout.addStretch(1)
        layout.addWidget(set_group)

        layout.addWidget(self.build_button_row())
        return root

    def selected_arm(self) -> str:
        return "left" if self.arm_group.checkedId() == 0 else "right"

    # ── actions ────────────────────────────────────────────────

    def _do_get_model_type(self) -> None:
        server = self.get_server()

        def task(log, _cancelled):
            log(f"$ grpcurl -d '{{}}' {server} xr.sdk.System/GetModelType")
            result = run_get_model_type(server)
            return result

        self.start_worker(task)

    def _do_unary(self, method: str) -> None:
        server = self.get_server()
        arm = self.selected_arm()

        def task(log, _cancelled):
            log(f"$ grpcurl -d '{{}}' {server} xr.sdk.Master{'Left' if arm == 'left' else 'Right'}Arm/{method}")
            result = run_unary_query(server, arm, method)
            return result

        self.start_worker(task)

    def _do_set_control_mode(self, mode: int) -> None:
        server = self.get_server()
        arm = self.selected_arm()
        label = "ENDPOSE_TELEOP" if mode == 13 else "JOINT_TELEOP"

        def task(log, _cancelled):
            log(f"$ grpcurl -d '{{\"mode\": {mode}}}' {server} .../SetControlMode  ({label})")
            result = run_set_control_mode(server, arm, mode)
            return result

        self.start_worker(task)

    def _do_stream(self, method: str) -> None:
        server = self.get_server()
        arm = self.selected_arm()

        def task(log, is_cancelled):
            log(f"$ grpcurl -d '{{}}' {server} xr.sdk.Master{'Left' if arm == 'left' else 'Right'}Arm/{method}")
            return run_stream_query(server, arm, method, log, is_cancelled)

        self._streaming_method = method
        self.start_worker(task)

    # ── lifecycle ──────────────────────────────────────────────

    def set_running_state(self, running: bool) -> None:
        self.set_base_running_state(running)
        for btn in self.action_buttons:
            btn.setEnabled(not running)
        if not running:
            self._streaming_method = None

    def stop_worker(self) -> None:
        if self._streaming_method:
            self.main_window.append_log(f"Stopping {self._streaming_method} …")
        super().stop_worker()

    def shutdown(self) -> None:
        if self._streaming_method and self.worker is not None and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(2000)
        super().shutdown()
