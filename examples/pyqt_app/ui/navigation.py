from __future__ import annotations

from ..models.page_spec import PageSpec
from ..pages import AlignMasterSlavePage, ArmControlPage, CheckConnectPage, DeployPage, RemoteOpsPage, RobotControlPage, RobotInfoPage

PAGE_SPECS = [
    PageSpec("check_connect", "Check Connect", CheckConnectPage, "system", "PING"),
    PageSpec("robot_control", "Robot Control", RobotControlPage, "system", "CTRL"),
    PageSpec("arm_control", "Arm Control", ArmControlPage, "system", "ARM"),
    PageSpec("align_master_slave", "Align Master-Slave", AlignMasterSlavePage, "system", "SYNC"),
    PageSpec("robot_info", "Robot Info", RobotInfoPage, "system", "INFO"),
    PageSpec("remote_ops", "Remote Ops", RemoteOpsPage, "ops", "SSH"),
    PageSpec("deploy", "Deploy", DeployPage, "ops", "DEPLOY"),
]
