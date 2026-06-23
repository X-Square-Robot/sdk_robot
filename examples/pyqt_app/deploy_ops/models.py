from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class DeployPreset:
    label: str
    host: str
    user: str = "xr"
    port: int = 22
    container_name: str = "ex001_master-sdk_server-1"
    container_path: str = "/opt/xr/bot"
    temp_dir: str = "/home/xr/temp"


DEPLOY_PRESETS = [
    DeployPreset("主臂 ex001-53 (10.100.28.209)", "10.100.28.209"),
    DeployPreset("从臂 ex001-61 (10.100.21.60)", "10.100.21.60"),
]
