from typing import Annotated
from client.x2robot.sdk import CoordinateSystemMode, CoordinateSystemModeParam
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
    """Linux/Ubuntu平台获取单个按键输入
    
    注意：在 raw 模式下，Ctrl+C 会被当作普通字符读取（ASCII 码 0x03）
    需要特殊处理以支持正常的中断功能
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # Ctrl+C 在 raw 模式下是字符 '\x03'
        if ch == '\x03':  # Ctrl+C
            # 恢复终端设置
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            # 抛出 KeyboardInterrupt 异常
            raise KeyboardInterrupt("用户按下 Ctrl+C")
        elif ch:
            return ch.lower()
        else:
            return None
    finally:
        # 确保终端设置被恢复（除非已经因为 Ctrl+C 恢复了）
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except:
            pass  # 如果已经恢复过了，忽略错误

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
        cur_velocity = ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=-0.4)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.01)
    time.sleep(1.0)

    for i in range(300):
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
    """停止底盘运动"""
    # 发送停止命令至少100次（1秒），确保机器人完全停止
    for i in range(20):
        cur_velocity = ChassisVelocity(vel_x=0.0, vel_y=0.0, vel_yaw=0.0)
        robot.chassis.set_velocity(cur_velocity)
        time.sleep(0.01)

def move_by_keyboard(robot: Robot):
    """通过键盘实时控制底盘速度
    
    速度模式需要持续发送命令（至少10Hz），所以这里采用持续发送的方式
    按下方向键时持续发送速度命令，松开或按下其他键时停止
    """
    # 设置速度控制模式
    robot.chassis.set_control_mode(ChassisControlModeParam(mode=ChassisControlMode.VELOCITY))
    
    # 先确保机器人停止
    print("正在停止底盘...")
    stop_chassis(robot)
    
    vel_x = 0.25
    vel_yaw = 0.3
    
    print("=" * 60)
    print("键盘控制底盘速度")
    print("=" * 60)
    print("方向控制：")
    print("  w - 前进")
    print("  s - 后退")
    print("  a - 左转（逆时针）")
    print("  d - 右转（顺时针）")
    print("速度调整：")
    print("  i - 增加前进速度")
    print("  k - 减少前进速度")
    print("  j - 增加旋转速度")
    print("  l - 减少旋转速度")
    print("  space - 停止")
    print("  q - 退出")
    print("=" * 60)
    print(f"当前速度: vel_x={vel_x:.2f} m/s, vel_yaw={vel_yaw:.2f} rad/s")
    print("等待按键输入...")
    
    current_vel_x = 0.0
    current_vel_y = 0.0
    current_vel_yaw = 0.0
    
    try:
        while True:
            key = get_key()
            if key is None:
                continue  # 忽略无效按键
            
            if key == 'w':
                # 前进：持续发送速度命令
                print(f"前进: vel_x={vel_x:.2f}")
                current_vel_x = vel_x
                current_vel_y = 0.0
                current_vel_yaw = 0.0
                # 持续发送命令直到按下其他键
                for _ in range(30):  # 发送1秒的命令
                    cur_velocity = ChassisVelocity(vel_x=current_vel_x, vel_y=current_vel_y, vel_yaw=current_vel_yaw)
                    robot.chassis.set_velocity(cur_velocity)
                    time.sleep(0.01)
                # 停止
                stop_chassis(robot)
                current_vel_x = 0.0
                
            elif key == 's':
                # 后退
                print(f"后退: vel_x={-vel_x:.2f}")
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
                # 左转（逆时针，正角速度）
                print(f"左转: vel_yaw={vel_yaw:.2f}")
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
                # 右转（顺时针，负角速度）
                print(f"右转: vel_yaw={-vel_yaw:.2f}")
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
                vel_x = max(0.0, min(vel_x, 1.0))  # 限制在0-1之间
                print(f"前进速度增加到: {vel_x:.2f} m/s")
                
            elif key == 'k':
                vel_x -= 0.05
                vel_x = max(0.0, min(vel_x, 1.0))
                print(f"前进速度减少到: {vel_x:.2f} m/s")
                
            elif key == 'j':
                vel_yaw += 0.05
                vel_yaw = max(0.0, min(vel_yaw, 2.0))  # 限制在0-2之间
                print(f"旋转速度增加到: {vel_yaw:.2f} rad/s")
                
            elif key == 'l':
                vel_yaw -= 0.05
                vel_yaw = max(0.0, min(vel_yaw, 2.0))
                print(f"旋转速度减少到: {vel_yaw:.2f} rad/s")
                
            elif key == ' ' or key == '\x20':  # 空格键
                print("停止")
                stop_chassis(robot)
                current_vel_x = 0.0
                current_vel_y = 0.0
                current_vel_yaw = 0.0
                
            elif key == 'q':
                print("退出")
                stop_chassis(robot)
                break
            else:
                # 忽略其他无效按键
                pass
                
    except KeyboardInterrupt:
        print("\n收到中断信号，停止底盘...")
        stop_chassis(robot)
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
        stop_chassis(robot)


def main(
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
    control_mode: Annotated[str, typer.Option(help="control mode: map, keyboard")] = "keyboard",
):
    robot = connect(f"x2://{server}")

    # 注意：在键盘控制模式下，Ctrl+C 会在 get_key() 中处理
    # 这里设置信号处理器作为备用（虽然 raw 模式下可能不会触发）
    def signal_handler(signum, frame):
        print("\n收到中断信号，退出...")
        exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    if control_mode == "map":
        print("this example is going to rotate the chassis for 3s by 0.4 rad/s, please make sure there is enough space around the robot")
        if not input("continue? (y/n): ").lower() == "y":
            return
        move_by_map(robot)
    elif control_mode == "keyboard":
        print("准备开始键盘控制，请确保有足够的空间")
        if not input("continue? (y/n): ").lower() == "y":
            return
        move_by_keyboard(robot)
    else:
        print(f"unknown control mode: {control_mode}, please choose from map, or keyboard")
        return

if __name__ == "__main__":
    typer.run(main)
