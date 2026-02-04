import time
import math
import threading
from typing import Annotated
import typer
from x2robot import connect

def display_ultrasonic_data(range_data, label: str):
    print(f"\n{label}:")
    
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
            display_ultrasonic_data(range_data, label)
        except Exception as e:
            print(f"\n{label}:")
            print(f"Read failed: {e}")

def main(
    action: Annotated[
        str, typer.Option(help="Action to perform: sensor-1..4[-stream], all, all-stream")
    ] = "sensor-1",
    server: Annotated[
        str, typer.Option(help="Server address, e.g., localhost:50051")
    ] = "localhost:50051",
):
    robot = connect(f"x2://{server}")

    print("Connected to robot, reading ultrasonic sensor data...")
    print("=" * 80)

    # Define single-shot and streaming methods for 4 channels; indices 0..3 correspond to sensors 1..4
    poll_methods = [
        robot.ultrasonic.get_chassis_ultrasonic_1,
        robot.ultrasonic.get_chassis_ultrasonic_2,
        robot.ultrasonic.get_chassis_ultrasonic_3,
        robot.ultrasonic.get_chassis_ultrasonic_4,
    ]
    stream_methods = [
        robot.ultrasonic.get_chassis_ultrasonic_1_stream,
        robot.ultrasonic.get_chassis_ultrasonic_2_stream,
        robot.ultrasonic.get_chassis_ultrasonic_3_stream,
        robot.ultrasonic.get_chassis_ultrasonic_4_stream,
    ]

    def label_of(i: int) -> str:
        return f"Sensor {i+1}"

    try:
        if action == "all":
            # Single shot read from all 4 sensors
            print("Reading from all 4 ultrasonic sensors (single shot)...")
            read_all_sensors({label_of(i): poll_methods[i] for i in range(4)})
            return

        if action == "all-stream":
            # Concurrently start streams for all 4 sensors
            print("Starting streams for all 4 sensors... Press Ctrl+C to stop.")
            def stream_worker(method, label):
                try:
                    for r in method():
                        display_ultrasonic_data(r, label)
                except Exception as e:
                    print(f"{label} stream error: {e}")
            threads = [
                threading.Thread(target=stream_worker, args=(stream_methods[i], label_of(i) + " Stream"), daemon=True)
                for i in range(4)
            ]
            for t in threads:
                t.start()
            while True:
                time.sleep(1)

        # Parse sensor-N or sensor-N-stream
        if not action.startswith("sensor-"):
            print(f"Unknown action: {action}")
            print("Valid actions: sensor-1..4, sensor-1..4-stream, all, all-stream")
            return

        parts = action.split("-")
        if len(parts) < 2:
            print("Invalid action format. Should be sensor-N or sensor-N-stream")
            return
        try:
            idx = int(parts[1]) - 1
        except ValueError:
            print("Invalid action format. N should be 1..4")
            return
        if idx < 0 or idx >= 4:
            print("Invalid action format. N is out of bounds, should be 1..4")
            return

        is_stream = len(parts) >= 3 and parts[2] == "stream"
        if is_stream:
            print(f"Starting stream for {label_of(idx)}... Press Ctrl+C to stop.")
            for r in stream_methods[idx]():
                display_ultrasonic_data(r, label_of(idx) + " Stream")
        else:
            display_ultrasonic_data(poll_methods[idx](), label_of(idx))

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {type(e).__name__}: {e}")


if __name__ == "__main__":
    typer.run(main)
