from typing import Annotated
from x2robot.sdk import CoordinateSystemMode, CoordinateSystemModeParam
import typer
from x2robot import Robot, connect
from x2robot.sdk import ChassisControlMode, ChassisControlModeParam, ChassisPosition, ChassisVelocity
import time
from x2robot.sdk import SaveMapParam
from x2robot.sdk import NavigationMode, NavigationModeParam
import sys
import termios
import tty
import signal

def get_key():
    """Get single key input on Linux/Ubuntu platforms

    Note: In raw mode, Ctrl+C is read as a normal character (ASCII code 0x03)
    Special handling is required to support normal interrupt functionality
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # Ctrl+C in raw mode is character '\x03'
        if ch == '\x03':  # Ctrl+C
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            # Raise KeyboardInterrupt exception
            raise KeyboardInterrupt("User pressed Ctrl+C")
        elif ch:
            return ch.lower()
        else:
            return None
    finally:
        # Ensure terminal settings are restored (unless already restored due to Ctrl+C)
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except:
            pass  # If already restored, ignore the error

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

    result = robot.navigation.start_mapping()
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

def move_by_keyboard(robot: Robot):
    """Control chassis velocity in real-time via keyboard

    Velocity mode requires continuous command sending (at least 10Hz),
    so continuous sending is used here.
    When arrow keys are pressed, velocity commands are continuously sent,
    and when released or other keys are pressed, movement stops.
    """
    # 设置速度控制模式
    robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY))
    
    # First ensure the robot is stopped
    print("Stopping chassis...")
    stop_chassis(robot)
    
    vel_x = 0.25
    vel_yaw = 0.3
    
    print("=" * 60)
    print("Keyboard Control Chassis Velocity")
    print("=" * 60)
    print("Direction Control:")
    print("  w - Forward")
    print("  s - Backward")
    print("  a - Turn left (counter-clockwise)")
    print("  d - Turn right (clockwise)")
    print("Speed Adjustment:")
    print("  i - Increase forward speed")
    print("  k - Decrease forward speed")
    print("  j - Increase rotation speed")
    print("  l - Decrease rotation speed")
    print("  space - Stop")
    print("  q - Quit")
    print("=" * 60)
    print(f"Current speed: vel_x={vel_x:.2f} m/s, vel_yaw={vel_yaw:.2f} rad/s")
    print("Waiting for key input...")
    
    current_vel_x = 0.0
    current_vel_y = 0.0
    current_vel_yaw = 0.0
    
    try:
        while True:
            key = get_key()
            if key is None:
                continue 
            
            if key == 'w':
                # Forward: continuously send velocity commands
                print(f"Forward: vel_x={vel_x:.2f}")
                current_vel_x = vel_x
                current_vel_y = 0.0
                current_vel_yaw = 0.0
                # Continuously send commands until another key is pressed
                for _ in range(30):  # Send commands for 1 second
                    cur_velocity = ChassisVelocity(vel_x=current_vel_x, vel_y=current_vel_y, vel_yaw=current_vel_yaw)
                    robot.chassis.set_velocity(cur_velocity)
                    time.sleep(0.01)
                # Stop
                stop_chassis(robot)
                current_vel_x = 0.0
                
            elif key == 's':
                # Backward
                print(f"Backward: vel_x={-vel_x:.2f}")
                current_vel_x = -vel_x
                current_vel_y = 0.0
                current_vel_yaw = 0.0
                for _ in range(30):
                    cur_velocity = ChassisVelocity(vel_x=current_vel_x, vel_y=current_vel_y, vel_yaw=current_vel_yaw)
                    robot.chassis.set_velocity(cur_velocity)
                    time.sleep(0.01)
                stop_chassis(robot)
                current_vel_x = 0.0
                
            elif key == 'a':
                # Turn left (counter-clockwise, positive angular velocity)
                print(f"Turn left: vel_yaw={vel_yaw:.2f}")
                current_vel_x = 0.0
                current_vel_y = 0.0
                current_vel_yaw = vel_yaw
                for _ in range(30):
                    cur_velocity = ChassisVelocity(vel_x=current_vel_x, vel_y=current_vel_y, vel_yaw=current_vel_yaw)
                    robot.chassis.set_velocity(cur_velocity)
                    time.sleep(0.01)
                stop_chassis(robot)
                current_vel_yaw = 0.0
                
            elif key == 'd':
                # Turn right (clockwise, negative angular velocity)
                print(f"Turn right: vel_yaw={-vel_yaw:.2f}")
                current_vel_x = 0.0
                current_vel_y = 0.0
                current_vel_yaw = -vel_yaw
                for _ in range(30):
                    cur_velocity = ChassisVelocity(vel_x=current_vel_x, vel_y=current_vel_y, vel_yaw=current_vel_yaw)
                    robot.chassis.set_velocity(cur_velocity)
                    time.sleep(0.01)
                stop_chassis(robot)
                current_vel_yaw = 0.0
                
            elif key == 'i':
                vel_x += 0.05
                vel_x = max(0.0, min(vel_x, 1.0))  # Limit between 0-1
                print(f"Forward speed increased to: {vel_x:.2f} m/s")

            elif key == 'k':
                vel_x -= 0.05
                vel_x = max(0.0, min(vel_x, 1.0))
                print(f"Forward speed decreased to: {vel_x:.2f} m/s")

            elif key == 'j':
                vel_yaw += 0.05
                vel_yaw = max(0.0, min(vel_yaw, 2.0))  # Limit between 0-2
                print(f"Rotation speed increased to: {vel_yaw:.2f} rad/s")

            elif key == 'l':
                vel_yaw -= 0.05
                vel_yaw = max(0.0, min(vel_yaw, 2.0))
                print(f"Rotation speed decreased to: {vel_yaw:.2f} rad/s")

            elif key == ' ' or key == '\x20':  # Space key
                print("Stop")
                stop_chassis(robot)
                current_vel_x = 0.0
                current_vel_y = 0.0
                current_vel_yaw = 0.0

            elif key == 'q':
                print("Quit")
                stop_chassis(robot)
                break
            else:
                # ignore other invalid keys
                pass
                
    except KeyboardInterrupt:
        print("\nReceived interrupt signal, stopping chassis...")
        stop_chassis(robot)
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
        stop_chassis(robot)


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    control_mode: Annotated[str, typer.Option(help="control mode: map, keyboard")] = "keyboard",
):
    robot = connect(f"x2://{server}")

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
