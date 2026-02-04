import math
from typing import Annotated
import typer
from x2robot import connect

RADAR_FREQUENCY = 10  # Hz
RADAR_ANGLE_INCREMENT = 0.02  # rad
RADAR_MIN_RANGE = 0.05  # m
RADAR_MAX_RANGE = 25.0  # m

def display_scan_data(scan_data):
    print(f"Angle Range: [{scan_data.angle_min:.3f}, {scan_data.angle_max:.3f}] rad")
    print(f"Angle Increment: {scan_data.angle_increment:.6f} rad (Spec: {RADAR_ANGLE_INCREMENT})")
    print(f"Time Increment: {scan_data.time_increment:.9f} s")
    if scan_data.scan_time > 0:
        print(f"Scan Time: {scan_data.scan_time:.3f} s (Frequency: {1.0/scan_data.scan_time:.1f} Hz, Spec: {RADAR_FREQUENCY} Hz)")
    else:
        print(f"Scan Time: {scan_data.scan_time:.3f} s")
    print(f"Range Limits: [{scan_data.range_min:.2f}, {scan_data.range_max:.2f}] m (Spec: [{RADAR_MIN_RANGE}, {RADAR_MAX_RANGE}])")
    print(f"Number of Points: {len(scan_data.ranges)}")
    
    if len(scan_data.ranges) > 0:
        print(f"Sample Ranges (first 10): {[f'{r:.2f}' for r in scan_data.ranges[:10]]}")
        valid_ranges = [r for r in scan_data.ranges if not math.isnan(r)]
        if valid_ranges:
            print(f"Min Range in Scan: {min(valid_ranges):.2f} m")
            print(f"Max Range in Scan: {max(valid_ranges):.2f} m")
        else:
            print("Min/Max Range in Scan: N/A (all values are NaN)")
    
    if len(scan_data.intensities) > 0:
        print(f"Sample Intensities (first 10): {[f'{i:.2f}' for i in scan_data.intensities[:10]]}")

def main(
    action: Annotated[
        str, typer.Option(help="Action to perform: single, stream")
    ] = "single",
    server: Annotated[str, typer.Option(help="server address, e.g., localhost:50051")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    print("Connected to robot, reading radar data...")
    print("-" * 80)

    try:
        if action == "stream":
            print("Starting radar stream... Press Ctrl+C to stop.\n")
            for scan_data in robot.radar.get_laser_scan_stream():
                if scan_data:
                    display_scan_data(scan_data)
                    print("-" * 80)
                else:
                    print("Warning: Received empty scan data. Skipping.")
        elif action == "single":
            print("Performing a single read...")
            scan_data = robot.radar.get_laser_scan()
            display_scan_data(scan_data)
            print("-" * 80)
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
