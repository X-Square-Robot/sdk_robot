from typing import Annotated
from x2robot.sdk import CoordinateSystemMode, CoordinateSystemModeParam
import typer
from x2robot import Robot, connect
from x2robot.sdk import ChassisControlMode, ChassisControlModeParam, ChassisPosition, ChassisVelocity
from x2robot.sdk import RobotModeParam, RobotWorkMode
import time
from x2robot.sdk import SaveMapParam
from x2robot.sdk import NavigationMode, NavigationModeParam
import sys
import termios
import tty
import signal
import threading
from select import select

# Default publish rate for velocity commands (Hz). Velocity mode requires at least 10Hz.
VELOCITY_PUBLISH_RATE = 20.0
KEY_TIMEOUT = 0.1  # Seconds to wait for key input before checking again


def get_key(settings=None, timeout=0):
    """Get single key input on Linux/Ubuntu platforms.

    When timeout > 0, returns '' if no key is pressed within timeout (non-blocking).
    When timeout is 0 or None, blocks until a key is pressed.

    Note: In raw mode, Ctrl+C is read as a normal character (ASCII code 0x03).
    Special handling is required to support normal interrupt functionality.
    """
    fd = sys.stdin.fileno()
    old_settings = settings if settings is not None else termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if timeout and timeout > 0:
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            ch = sys.stdin.read(1) if rlist else ''
        else:
            ch = sys.stdin.read(1)
        # Ctrl+C in raw mode is character '\x03'
        if ch == '\x03':  # Ctrl+C
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            raise KeyboardInterrupt("User pressed Ctrl+C")
        if ch:
            return ch.lower()
        return ''
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


def save_terminal_settings():
    """Save current terminal settings for later restore."""
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    """Restore terminal settings."""
    if sys.platform == 'win32' or settings is None:
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def move_to_global_position(robot: Robot):
    # need to set control mode to global first
    current_position = robot.chassis.get_global_position()
    print(f"current global position: x={current_position.x}, y={current_position.y}, yaw={current_position.yaw}")
    robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.GLOBAL))
    robot.chassis.move_to_global_position(ChassisPosition(x=1.2, y=-0.2, yaw=0.0))
    time.sleep(2.0)
    current_position = robot.chassis.get_global_position()
    print(f"current global position: x={current_position.x}, y={current_position.y}, yaw={current_position.yaw}")

def move_to_relative_position(robot: Robot):
    # need to set control mode to relative first and set virtual zero point first
    current_position = robot.chassis.get_global_position()
    print(f"current global position: x={current_position.x}, y={current_position.y}, yaw={current_position.yaw}")
    robot.chassis.set_virtual_zero_point(current_position)
    robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.RELATIVE))
    print(f"move to relative position 0.85 meters forward")
    robot.chassis.move_to_relative_position(ChassisPosition(x=0.85, y=0.0, yaw=0.0))
    time.sleep(2.0)
    current_position = robot.chassis.get_relative_position()
    print(f"current relative position: x={current_position.x}, y={current_position.y}, yaw={current_position.yaw}")

def move_by_velocity(robot: Robot):
    # need to set control mode to velocity first
    robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY))
    # velocity mode must send command in a rate of at least 10Hz
    # rotate yaw is negative, clockwise
    for i in range(300):
        cur_velocity = ChassisVelocity(vel_x=0.3, vel_y=0.0, vel_yaw=0)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.01)
    time.sleep(1.0)
    for i in range(800):
        cur_velocity = ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=-0.4)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.01)
    time.sleep(1.0)
    print("rotate yaw to positive 0.4 rad/s")
    for i in range(800):
        cur_velocity = ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=0.4)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.01)
    time.sleep(1.0)
    # stop, set all velocities to 0
    for i in range(100):
        cur_velocity = ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=0.0)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.001)
    time.sleep(1.0)

def get_chassis_odometry(robot: Robot):
    current_odometry = robot.chassis.get_odometry()
    current_orientation = current_odometry.pose.pose.orientation
    print(current_orientation)
    current_velocity = current_odometry.twist.twist.linear
    print(current_velocity)
    current_angular_velocity = current_odometry.twist.twist.angular
    print(current_angular_velocity)
    current_position = current_odometry.pose.pose.position
    print(current_position)

def move_by_map(robot: Robot):
    result = robot.navigation.set_navigation_mode(NavigationModeParam(mode=NavigationMode.BUILT_IN_NAVIGATION))
    print(f"set built-in navigation mode success: {result.is_success}")

    coord_system_mode = CoordinateSystemModeParam(coordinate_system_mode=CoordinateSystemMode.MAP)
    result = robot.chassis.set_trajectory_coord_system_mode(coord_system_mode)
    print(f"set trajectory coord system mode success: {result.is_success}")

    result = robot.navigation.start_mapping();
    print(f"start mapping success: {result.is_success}")

    print(f"move around to build map...")
    # move around to build map
    move_by_velocity(robot)

    time.sleep(1.0)

    map_name = "test"
    result = robot.navigation.stop_mapping(SaveMapParam(map_name=map_name))
    print(f"stop mapping success: {result.is_success}")

    result = robot.navigation.start_localization(SaveMapParam(map_name=map_name))
    print(f"start localization success: {result.is_success}")

    time.sleep(2.0)

    move_to_relative_position(robot)

    get_chassis_odometry(robot)

def stop_chassis(robot: Robot):
    """Stop chassis movement"""
    # send velocity command to stop chassis
    for i in range(30):
        cur_velocity = ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=0.0)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.01)


class VelocitySendThread(threading.Thread):
    """Background thread that continuously sends velocity commands at a fixed rate.

    Similar to teleop_twist_keyboard's PublishThread: key presses only update
    the target velocity state; this thread keeps sending commands for smooth control.
    """

    def __init__(self, robot: Robot, rate: float):
        super().__init__(daemon=True)
        self.robot = robot
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_yaw = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = 1.0 / rate if rate > 0 else 0.1
        self.start()

    def update(self, vel_x: float, vel_y: float, vel_yaw: float, speed: float, turn: float):
        with self.condition:
            self.vel_x = vel_x
            self.vel_y = vel_y
            self.vel_yaw = vel_yaw
            self.speed = speed
            self.turn = turn
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done:
            with self.condition:
                self.condition.wait(self.timeout)
                vx = self.vel_x * self.speed
                vy = self.vel_y * self.speed
                vyaw = self.vel_yaw * self.turn
            try:
                self.robot.chassis.set_velocity(
                    ChassisVelocity(vel_x=vx, vel_y=vy, vel_yaw=vyaw)
                )
            except Exception:
                pass
        # Send stop when exiting
        try:
            self.robot.chassis.set_velocity(
                ChassisVelocity(vel_x=0, vel_y=0, vel_yaw=0)
            )
        except Exception:
            pass


def move_by_keyboard(robot: Robot):
    """Control chassis velocity in real-time via keyboard.

    Uses a background thread to continuously send velocity commands (like
    teleop_twist_keyboard), so holding a key produces smooth continuous
    movement. Key release (timeout) or space stops the robot.
    """
    robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY))
    print("Stopping chassis...")
    stop_chassis(robot)

    # Direction state: x in [-1,0,1], y in [-1,0,1], th (yaw) in [-1,0,1]
    x, y, th = 0.0, 0.0, 0.0
    speed = 0.35
    turn = 0.40
    speed_limit, turn_limit = 1.0, 2.0

    move_bindings = {
        'w': (1, 0, 0),
        's': (-1, 0, 0),
        'a': (0, 0, 1),
        'd': (0, 0, -1),
    }
    speed_step = 0.02
    turn_step = 0.02

    print("=" * 60)
    print("Keyboard Control Chassis Velocity")
    print("=" * 60)
    print("Direction Control (hold key for continuous movement):")
    print("  w - Forward    s - Backward")
    print("  a - Turn left  d - Turn right")
    print("Speed Adjustment: i/k (linear), j/l (angular)")
    print("  space - Stop   q - Quit")
    print("=" * 60)
    print(f"Current: speed={speed:.2f} m/s, turn={turn:.2f} rad/s")
    print("Waiting for key input...")

    settings = save_terminal_settings()
    send_thread = VelocitySendThread(robot, VELOCITY_PUBLISH_RATE)
    send_thread.update(x, y, th, speed, turn)

    try:
        while True:
            key = get_key(settings, KEY_TIMEOUT)
            if key in move_bindings:
                x, y, th = move_bindings[key]
            elif key == 'i':
                speed = round(min(speed_limit, speed + speed_step), 2)
                print(f"speed={speed:.2f} m/s, turn={turn:.2f} rad/s")
            elif key == 'k':
                speed = round(max(0.0, speed - speed_step), 2)
                print(f"speed={speed:.2f} m/s, turn={turn:.2f} rad/s")
            elif key == 'j':
                turn = round(min(turn_limit, turn + turn_step), 2)
                print(f"speed={speed:.2f} m/s, turn={turn:.2f} rad/s")
            elif key == 'l':
                turn = round(max(0.0, turn - turn_step), 2)
                print(f"speed={speed:.2f} m/s, turn={turn:.2f} rad/s")
            elif key == ' ' or key == '\x20':
                x, y, th = 0.0, 0.0, 0.0
            elif key == 'q':
                break
            else:
                # Timeout (key released) or unknown key: stop
                x, y, th = 0.0, 0.0, 0.0

            send_thread.update(x, y, th, speed, turn)
    except KeyboardInterrupt:
        print("\nReceived interrupt signal, stopping chassis...")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        send_thread.stop()
        restore_terminal_settings(settings)


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    control_mode: Annotated[str, typer.Option(help="control mode: map, keyboard")] = "keyboard",
):
    robot = connect(f"x2://{server}")

    robot.system.set_work_mode(RobotModeParam(mode=RobotWorkMode.SDK))

    # Note: In keyboard control mode, Ctrl+C is handled in get_key()
    # Set signal handler as backup (though it may not trigger in raw mode)
    def signal_handler(signum, frame):
        print("\nReceived interrupt signal, exiting...")
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    if control_mode == "map":
        print("This example is going to move forward for 1m and then rotate the chassis for 8 seconds by 0.4 rad/s clockwise and counter-clockwise")
        print("Please make sure there is at least 2m distance between the robot and the obstacle!!!")
        print("Please make sure there is enough space around the robot to move!!!")
        if not input("continue? (y/n): ").lower() == "y":
            return
        move_by_map(robot)
    elif control_mode == "keyboard":
        print("Ready to start keyboard control, please ensure there is enough space!!!")
        if not input("continue? (y/n): ").lower() == "y":
            return
        move_by_keyboard(robot)
    else:
        print(f"unknown control mode: {control_mode}, please choose from map, or keyboard")
        return

if __name__ == "__main__":
    typer.run(main)
