from __future__ import annotations

from ..models.requests import ArmControlRequest
from ..services import run_arm_control, run_check_connect, run_move_slave_arm, run_robot_control


class CheckConnectController:
    def __init__(self, page) -> None:
        self.page = page

    def run(self) -> None:
        server = self.page.get_server()
        self.page.start_worker(lambda log, _cancelled: run_check_connect(server, log))


class RobotControlController:
    def __init__(self, page) -> None:
        self.page = page

    def run(self) -> None:
        server = self.page.get_server()
        action = self.page.action_combo.currentText()
        self.page.start_worker(lambda log, _cancelled: run_robot_control(server, action, log))


class ArmControlController:
    def __init__(self, page) -> None:
        self.page = page

    def run(self) -> None:
        request = ArmControlRequest(
            server=self.page.get_server(),
            action=self.page.action_combo.currentText(),
            mode=self.page.mode_combo.currentText(),
            arm=self.page.arm_combo.currentText(),
            target_x=self.page.pos_x_spin.value(),
            target_y=self.page.pos_y_spin.value(),
            target_z=self.page.pos_z_spin.value(),
            target_qx=self.page.ori_qx_spin.value(),
            target_qy=self.page.ori_qy_spin.value(),
            target_qz=self.page.ori_qz_spin.value(),
            target_qw=self.page.ori_qw_spin.value(),
        )
        self.page.start_worker(lambda log, cancelled: run_arm_control(request, log, cancelled))


class AlignMasterSlaveController:
    def __init__(self, page) -> None:
        self.page = page

    def run_step1(self) -> None:
        slave_server = self.page.slave_server_input.text().strip() or self.page.get_server()
        self.page.start_worker(
            lambda log, _cancelled: run_move_slave_arm(
                slave_server=slave_server,
                slave_arm=self.page.slave_arm_combo.currentText(),
                log=log,
                target_x=self.page.pos_x_spin.value(),
                target_y=self.page.pos_y_spin.value(),
                target_z=self.page.pos_z_spin.value(),
            )
        )
