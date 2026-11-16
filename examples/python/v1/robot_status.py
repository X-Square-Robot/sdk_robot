"""Example of get_robot_status with separate energy, motion, execution, safety,
and health sections.

Each print_* function explicitly lists every field in its section, matching
agent_status.md §4.2 for easy schema comparison. Unavailable fields display as None.
"""

from __future__ import annotations

import json
from typing import Any, Annotated, Optional

import typer

from x2robot import connect


def _fmt(value: Any) -> str:
    """Format nested objects as indented JSON and other values as strings."""
    if isinstance(value, (dict, list)):
        return json.dumps(value, ensure_ascii=False, indent=2)
    return str(value)


def _print_fields(title: str, section: Optional[dict], fields: list[str]) -> None:
    print(f"\n=== {title} ===")
    section = section or {}
    for key in fields:
        print(f"{key}: {_fmt(section.get(key))}")


def print_energy(energy: Optional[dict]) -> None:
    _print_fields(
        "energy",
        energy,
        [
            "battery_level",
            "is_charging",
            "battery_charging_method",
            "battery_temperature",
            "est_runtime_left",
            "has_power_fault",
        ],
    )


def print_motion(motion: Optional[dict]) -> None:
    _print_fields(
        "motion",
        motion,
        [
            "localization",
            "current_pose",
            "is_moving",
            "linear_velocity",
            "angular_velocity",
            "is_navigating",
            "navigation_target",
            "distance_to_goal",
            "is_path_blocked",
            "nearest_obstacle_distance",
            "blocked_reason",
        ],
    )


def print_execution(execution: Optional[dict]) -> None:
    _print_fields(
        "execution",
        execution,
        [
            "operation_mode",
            "control_owner",
            "is_speaking",
            "audio_output_volume",
            "arm_state_left",
            "arm_state_right",
            "end_effector_pose",
            "gripper_force",
            "gripper_width",
            "is_holding_object",
            "system_uptime",
        ],
    )


def print_safety(safety: Optional[dict]) -> None:
    _print_fields(
        "safety",
        safety,
        [
            "emergency_stop_active",
            "safety_still",
            "fault_alerts",
            "component_health",
        ],
    )


def print_health(health: Optional[dict]) -> None:
    _print_fields(
        "health",
        health,
        [
            "battery_soh",
            "network_connected",
            "network_latency",
            "cpu_usage",
            "memory_usage",
            "disk_usage",
            "active_error_count",
            "highest_error_level",
            "active_error_codes",
        ],
    )


def print_robot_status(data: dict) -> None:
    print_energy(data.get("energy"))
    print_motion(data.get("motion"))
    print_execution(data.get("execution"))
    print_safety(data.get("safety"))
    print_health(data.get("health"))


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    result = robot.get_robot_status()
    if not result.is_success:
        err = result.error or {}
        print(f"get robot status failed: {err.get('error_code')}, {err.get('error_message')}")
        return

    print_robot_status(result.data or {})


if __name__ == "__main__":
    typer.run(main)
