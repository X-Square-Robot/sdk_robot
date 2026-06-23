from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class QuickCommand:
    label: str
    command: str
    in_container: bool


@dataclass(slots=True)
class SshPreset:
    label: str
    host: str
    user: str = "xr"
    port: int = 22


QUICK_COMMANDS = [
    QuickCommand("ros2 /sdk/status", "ros2 topic echo /sdk/status --once 2>/dev/null", True),
    QuickCommand(
        "ros2 /mode_ctrl",
        "ros2 topic echo /realtime_controller_manager/mode_controller_enabled_status --once 2>/dev/null",
        True,
    ),
    QuickCommand("docker ps", "docker ps --format 'table {{.Names}}\\t{{.Status}}' 2>/dev/null", False),
    QuickCommand("docker logs", "docker logs --tail 80 sdk_server-1 2>&1", False),
]

SSH_PRESETS = [
    SshPreset("主臂 ex001-53 (10.100.28.209)", "10.100.28.209"),
    SshPreset("从臂 ex001-61 (10.100.21.60)", "10.100.21.60"),
]
