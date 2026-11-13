"""Chassis control example copied from the v1 scenarios, using v2 results."""

from __future__ import annotations

import argparse
import signal
import sys
import termios
import threading
import time
import tty
from select import select

import x2robot_sdk
from x2robot.sdk import (
    ChassisControlMode,
    ChassisControlModeParam,
    ChassisPosition,
    ChassisVelocity,
    CoordinateSystemMode,
    CoordinateSystemModeParam,
    NavigationMode,
    NavigationModeParam,
    RobotModeParam,
    RobotWorkMode,
    SaveMapParam,
    StartLocalizationParam,
)

VELOCITY_PUBLISH_RATE = 20.0
KEY_TIMEOUT = 0.1


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {(error.get('error_message') or 'SDK call failed')} (request_id={result.request_id or '-'})")
    return result.data


def get_key(settings=None, timeout=0):
    fd = sys.stdin.fileno()
    old = settings if settings is not None else termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select([sys.stdin], [], [], timeout) if timeout else ([sys.stdin], [], [])
        key = sys.stdin.read(1) if ready else ""
        if key == "\x03":
            raise KeyboardInterrupt
        return key.lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def save_terminal_settings():
    return None if sys.platform == "win32" else termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    if settings is not None and sys.platform != "win32":
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def set_mode(robot, mode):
    require(robot.chassis.set_control_mode(ChassisControlModeParam(mode=mode), timeout=3.0), "Chassis.SetControlMode")


def move_to_global_position(robot):
    current = require(robot.chassis.get_global_position(timeout=3.0), "Chassis.GetGlobalPosition")
    print(f"current global position: x={current.x}, y={current.y}, yaw={current.yaw}")
    set_mode(robot, ChassisControlMode.GLOBAL)
    target = ChassisPosition(x=1.2, y=-0.2, yaw=0.0)
    require(robot.chassis.move_to_global_position(target, timeout=5.0), "Chassis.MoveToGlobalPosition")
    time.sleep(2.0)
    current = require(robot.chassis.get_global_position(timeout=3.0), "Chassis.GetGlobalPosition")
    print(f"current global position: x={current.x}, y={current.y}, yaw={current.yaw}")


def move_to_relative_position(robot, cancel=False):
    current = require(robot.chassis.get_global_position(timeout=3.0), "Chassis.GetGlobalPosition")
    print(f"current global position: x={current.x}, y={current.y}, yaw={current.yaw}")
    require(robot.chassis.set_virtual_zero_point(current, timeout=3.0), "Chassis.SetVirtualZeroPoint")
    set_mode(robot, ChassisControlMode.RELATIVE)
    print("move to relative position 0.85 meters forward")
    require(robot.chassis.move_to_relative_position(ChassisPosition(x=0.85), timeout=5.0), "Chassis.MoveToRelativePosition")
    if not cancel:
        time.sleep(2.0)
    current = require(robot.chassis.get_relative_position(timeout=3.0), "Chassis.GetRelativePosition")
    print(f"current relative position: x={current.x}, y={current.y}, yaw={current.yaw}")
    if cancel:
        require(robot.navigation.cancel_navigation(timeout=3.0), "Navigation.CancelNavigation")


def move_by_velocity(robot):
    set_mode(robot, ChassisControlMode.VELOCITY)
    for _ in range(300):
        require(robot.chassis.set_velocity(ChassisVelocity(vel_x=0.3), timeout=1.0), "Chassis.SetVelocity")
        time.sleep(0.01)
    time.sleep(1.0)
    for yaw in (-0.4, 0.4):
        print(f"rotate yaw to {yaw:+.1f} rad/s")
        for _ in range(800):
            require(robot.chassis.set_velocity(ChassisVelocity(vel_yaw=yaw), timeout=1.0), "Chassis.SetVelocity")
            time.sleep(0.01)
        time.sleep(1.0)
    for _ in range(100):
        require(robot.chassis.set_velocity(ChassisVelocity(), timeout=1.0), "Chassis.SetVelocity(stop)")
        time.sleep(0.001)


def get_chassis_odometry(robot):
    odometry = require(robot.chassis.get_odometry(timeout=3.0), "Chassis.GetOdometry")
    print(odometry.pose.pose.orientation)
    print(odometry.twist.twist.linear)
    print(odometry.twist.twist.angular)
    print(odometry.pose.pose.position)


def move_by_map(robot, cancel=False):
    require(robot.navigation.set_navigation_mode(NavigationModeParam(mode=NavigationMode.BUILT_IN_NAVIGATION), timeout=3.0), "Navigation.SetNavigationMode")
    require(robot.chassis.set_trajectory_coord_system_mode(CoordinateSystemModeParam(coordinate_system_mode=CoordinateSystemMode.MAP), timeout=3.0), "Chassis.SetTrajectoryCoordSystemMode")
    require(robot.navigation.start_mapping(timeout=5.0), "Navigation.StartMapping")
    print("move around to build map...")
    move_by_velocity(robot)
    require(robot.navigation.stop_mapping(SaveMapParam(map_name="test"), timeout=10.0), "Navigation.StopMapping")
    require(robot.navigation.start_localization(StartLocalizationParam(map_name="test", use_init_pose=False), timeout=5.0), "Navigation.StartLocalization")
    time.sleep(2.0)
    move_to_relative_position(robot, cancel=cancel)
    get_chassis_odometry(robot)


def stop_chassis(robot):
    for _ in range(30):
        require(robot.chassis.set_velocity(ChassisVelocity(), timeout=1.0), "Chassis.SetVelocity(stop)")
        time.sleep(0.01)


class VelocitySendThread(threading.Thread):
    """Keep sending the current velocity while keyboard state changes."""

    def __init__(self, robot, rate):
        super().__init__(daemon=True)
        self.robot = robot
        self.state = [0.0] * 5
        self.condition = threading.Condition()
        self.done = False
        self.timeout = 1.0 / rate
        self.start()

    def update(self, vx, vy, vyaw, speed, turn):
        with self.condition:
            self.state = [vx, vy, vyaw, speed, turn]
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done:
            with self.condition:
                self.condition.wait(self.timeout)
                vx, vy, yaw, speed, turn = self.state
            try:
                require(self.robot.chassis.set_velocity(ChassisVelocity(vel_x=vx * speed, vel_y=vy * speed, vel_yaw=yaw * turn), timeout=1.0), "Chassis.SetVelocity")
            except Exception:
                pass
        try:
            require(self.robot.chassis.set_velocity(ChassisVelocity(), timeout=1.0), "Chassis.SetVelocity(stop)")
        except Exception:
            pass


def move_by_keyboard(robot):
    set_mode(robot, ChassisControlMode.VELOCITY)
    stop_chassis(robot)
    bindings = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 0, 1), "d": (0, 0, -1)}
    x = y = yaw = 0.0
    speed, turn = 0.35, 0.40
    settings = save_terminal_settings()
    sender = VelocitySendThread(robot, VELOCITY_PUBLISH_RATE)
    sender.update(x, y, yaw, speed, turn)
    print("w/s forward/back, a/d rotate, i/k speed, j/l turn, space stop, q quit")
    try:
        while True:
            key = get_key(settings, KEY_TIMEOUT)
            if key in bindings:
                x, y, yaw = bindings[key]
            elif key == "i":
                speed = min(1.0, speed + 0.02)
            elif key == "k":
                speed = max(0.0, speed - 0.02)
            elif key == "j":
                turn = min(2.0, turn + 0.02)
            elif key == "l":
                turn = max(0.0, turn - 0.02)
            elif key in (" ", "q"):
                x = y = yaw = 0.0
                if key == "q":
                    break
            else:
                x = y = yaw = 0.0
            sender.update(x, y, yaw, speed, turn)
    finally:
        sender.stop()
        restore_terminal_settings(settings)


def verify(robot, _model="auto"):
    require(enter_sdk_mode(robot), "System.SetWorkMode")
    set_mode(robot, ChassisControlMode.VELOCITY)
    require(robot.chassis.set_velocity(ChassisVelocity(), timeout=3.0), "Chassis.SetVelocity")
    get_chassis_odometry(robot)


_sdk_mode_entered = False


def enter_sdk_mode(robot):
    """切到 SDK 模式并记录占用，退出时由 restore_idle_mode 交还应用。"""
    global _sdk_mode_entered
    result = robot.system.set_work_mode(
        RobotModeParam(mode=RobotWorkMode.SDK), timeout=5.0
    )
    _sdk_mode_entered = bool(getattr(result, "is_success", False))
    return result


def restore_idle_mode(robot):
    """只交还本示例占用的 SDK 模式：/application/task_info.type 回到 0。"""
    global _sdk_mode_entered
    if not _sdk_mode_entered:
        return
    _sdk_mode_entered = False
    try:
        robot.system.set_work_mode(
            RobotModeParam(mode=RobotWorkMode.IDLE), timeout=5.0
        )
    except Exception as error:  # 清理失败不能掩盖示例本身的错误
        print(f"WARN: failed to leave SDK mode: {error}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--control_mode", "--control-mode", choices=("map", "map_cancel", "keyboard", "verify"), default="keyboard")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        require(enter_sdk_mode(robot), "System.SetWorkMode")
        if args.control_mode == "verify":
            verify(robot)
        elif args.control_mode == "map":
            move_by_map(robot)
        elif args.control_mode == "map_cancel":
            move_by_map(robot, cancel=True)
        else:
            move_by_keyboard(robot)
        return 0
    except KeyboardInterrupt:
        print("\nReceived interrupt signal, stopping chassis...")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        restore_idle_mode(robot)
        require(robot.close(), "Robot.Close")


if __name__ == "__main__":
    raise SystemExit(main())
