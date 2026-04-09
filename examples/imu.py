import math
from typing import Annotated
import typer
from x2robot import connect

def quaternion_to_euler(x, y, z, w):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw) in radians.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def display_imu_data(imu_data):
    """
    Displays simplified IMU sensor data.
    """
    if not imu_data:
        print("Warning: Received empty IMU data message. Skipping.")
        return

    # Orientation (Euler Angles)
    if imu_data.is_set("orientation"):
        q = imu_data.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        print(f"Orientation (Euler): Roll={math.degrees(roll):.2f}°, Pitch={math.degrees(pitch):.2f}°, Yaw={math.degrees(yaw):.2f}°")
    else:
        print("Orientation (Euler): Not available")

    if imu_data.is_set("angular_velocity"):
        av = imu_data.angular_velocity
        print(f"Angular Velocity (rad/s):  x={av.x:.3f}, y={av.y:.3f}, z={av.z:.3f}")
    else:
        print("Angular Velocity (rad/s): Not available")

    if imu_data.is_set("linear_acceleration"):
        la = imu_data.linear_acceleration
        print(f"Linear Acceleration (m/s²): x={la.x:.3f}, y={la.y:.3f}, z={la.z:.3f}")
    else:
        print("Linear Acceleration (m/s²): Not available")

    print("================================================")

def read_and_display_imu(robot):
    """
    Reads and displays data from the IMU sensor.
    """
    try:
        imu_data = robot.imu.get_chassis_imu()
        display_imu_data(imu_data)
    except Exception as e:
        print(f" Read failed: {e}")

def main(
    action: Annotated[
        str, typer.Option(help="Action to perform: single, stream")
    ] = "single",
    server: Annotated[
        str, typer.Option(help="Server address, e.g., localhost:50051")
    ] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    print("Connected to robot, reading IMU sensor data...")
    print("=" * 80)

    try:
        if action == "stream":
            print("Starting IMU stream... Press Ctrl+C to stop.\n")
            for imu_data in robot.imu.get_chassis_imu_stream():
                display_imu_data(imu_data)
        elif action == "single":
            print("Performing a single read...")
            read_and_display_imu(robot)
            print("\n" + "=" * 80)
            print("Single read completed.")
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: single, stream")

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {type(e).__name__}: {e}")


if __name__ == "__main__":
    typer.run(main)
