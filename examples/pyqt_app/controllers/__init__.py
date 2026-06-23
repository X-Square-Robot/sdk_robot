from .lifecycle import shutdown_pages
from .robot_pages import AlignMasterSlaveController, ArmControlController, CheckConnectController, RobotControlController

__all__ = [
    "AlignMasterSlaveController",
    "ArmControlController",
    "CheckConnectController",
    "RobotControlController",
    "shutdown_pages",
]
