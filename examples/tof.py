import time
import math
import threading
from typing import Annotated
import typer
from x2robot import connect

def display_tof_data(range_data, label: str, is_stream: bool = False):
    print(f"{label}:")

    # Radiation Type Mapping
    radiation_types = {
        0: "ULTRASOUND",
        1: "INFRARED",
    }
    radiation = radiation_types.get(range_data.radiation_type, f"Unknown({range_data.radiation_type})")

    print(f"  - Distance: {range_data.range:.3f} m")
    print(f"  - Radiation Type: {radiation}")
    print(f"  - Field of View: {range_data.field_of_view:.3f} rad ({math.degrees(range_data.field_of_view):.1f}°)")
    print(f"  - Min Range: {range_data.min_range:.2f} m")
    print(f"  - Max Range: {range_data.max_range:.2f} m")

    # Status Judgment
    if range_data.range < range_data.min_range:
        print(f"  - Status: Distance too close (less than minimum range)")
    elif range_data.range > range_data.max_range:
        print(f"  - Status: Out of measurement range")
    else:
        print(f"  - Status: OK")
    
    print("================================================")


def read_all_sensors(sensor_methods):
    for label, method in sensor_methods.items():
        try:
            range_data = method()
            display_tof_data(range_data, label)
        except Exception as e:
            print(f"{label}:")
            print(f"Read failed: {e}")


def main(
    action: Annotated[
        str, typer.Option(help="Action to perform: sensor-1, sensor-2, sensor-1-stream, sensor-2-stream, both, both-stream")
    ] = "sensor-1",
    server: Annotated[
        str, typer.Option(help="Server address, e.g., localhost:50051")
    ] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    print("Connected to robot, reading TOF sensor data...")
    print("=" * 80)

    try:
        if action == "sensor-1":
            print("Reading from Sensor 1...")
            range_data = robot.tof.get_chassis_tof_1()
            display_tof_data(range_data, "Sensor 1")
        elif action == "sensor-2":
            print("Reading from Sensor 2...")
            range_data = robot.tof.get_chassis_tof_2()
            display_tof_data(range_data, "Sensor 2")
        elif action == "both":
            print("Reading from both sensors (single shot)...")
            sensor_methods = {
                "Sensor 1": robot.tof.get_chassis_tof_1,
                "Sensor 2": robot.tof.get_chassis_tof_2,
            }
            read_all_sensors(sensor_methods)
        elif action == "sensor-1-stream":
            print("Starting stream for Sensor 1... Press Ctrl+C to stop.")
            for range_data in robot.tof.get_chassis_tof_1_stream():
                display_tof_data(range_data, "Sensor 1 Stream", is_stream=True)
        elif action == "sensor-2-stream":
            print("Starting stream for Sensor 2... Press Ctrl+C to stop.")
            for range_data in robot.tof.get_chassis_tof_2_stream():
                display_tof_data(range_data, "Sensor 2 Stream", is_stream=True)
        elif action == "both-stream":
            print("Starting streams for both sensors concurrently... Press Ctrl+C to stop.")
            
            def stream_worker(method, label):
                try:
                    for range_data in method():
                        display_tof_data(range_data, label, is_stream=True)
                except Exception as e:
                    print(f"{label} stream error: {e}")

            t1 = threading.Thread(target=stream_worker, args=(robot.tof.get_chassis_tof_1_stream, "Sensor 1 Stream"), daemon=True)
            t2 = threading.Thread(target=stream_worker, args=(robot.tof.get_chassis_tof_2_stream, "Sensor 2 Stream"), daemon=True)
            t1.start()
            t2.start()
            while True:
                time.sleep(1)
        else:
            print(f"Unknown action: {action}")
            print("Valid actions: sensor-1, sensor-2, sensor-1-stream, sensor-2-stream, both, both-stream")

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {type(e).__name__}: {e}")


if __name__ == "__main__":
    typer.run(main)
