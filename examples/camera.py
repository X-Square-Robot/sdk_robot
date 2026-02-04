import typer
from typing_extensions import Annotated
from enum import Enum
import cv2
import numpy as np
from x2robot import connect
from x2robot.sensor_msgs import CompressedImage


def show_image(image: CompressedImage, source: str, is_stream: bool = False):
    """
    Decodes and displays a CompressedImage using OpenCV and prints metadata.
    """
    if not image or not image.data:
        print(f"Warning: Empty image received from {source}; skipping.")
        return

    try:
        image_bytes = bytes(image.data)
        is_depth = "depth" in image.format.lower()

        if is_depth:
            # Strip header before trying to decode.
            image_bytes = image_bytes[12:]
        
        np_arr = np.frombuffer(image_bytes, np.uint8)
        
        if is_depth:
            frame = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        else:
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is not None:
            display_frame = frame
            if is_depth:
                # Normalize the 16-bit depth image to an 8-bit grayscale image for visualization
                display_frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                # Apply a colormap for better visualization
                display_frame = cv2.applyColorMap(display_frame, cv2.COLORMAP_JET)

            cv2.imshow(source, display_frame)
            if is_stream:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    raise KeyboardInterrupt("Stream stopped by user ('q' pressed).")
            else:
                print("-> Press any key in the image window or close it to continue.")
                while cv2.getWindowProperty(source, cv2.WND_PROP_VISIBLE) >= 1:
                    if cv2.waitKey(100) != -1:
                        break
        else:
            print(f"Could not decode image from {source}.")
    except Exception as e:
        print(f"OpenCV Error: {e}")


class HeadAction(str, Enum):
    rgb_image = "rgb-image"
    depth_image = "depth-image"
    rgb_stream = "rgb-stream"
    depth_stream = "depth-stream"


class ArmAction(str, Enum):
    raw_image = "raw-image"
    stream = "stream"


app = typer.Typer(help="A CLI to interact with the robot's cameras and display images.")


@app.command()
def head(
    action: Annotated[HeadAction, typer.Argument(help="The action to perform with the head camera.")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")
    print(f"Connected to robot. Action: '{action.value}' on head camera.")

    try:
        if action == HeadAction.rgb_image:
            image = robot.head_camera.get_rgb_image()
            show_image(image, "Head RGB")
        elif action == HeadAction.depth_image:
            image = robot.head_camera.get_depth_image()
            show_image(image, "Head Depth")
        elif action == HeadAction.rgb_stream:
            print("Starting RGB video stream... Press 'q' or Ctrl+C to stop.")
            for image in robot.head_camera.get_rgb_video_stream():
                show_image(image, "Head RGB Stream", is_stream=True)
        elif action == HeadAction.depth_stream:
            print("Starting Depth video stream... Press 'q' or Ctrl+C to stop.")
            for image in robot.head_camera.get_depth_video_stream():
                show_image(image, "Head Depth Stream", is_stream=True)
    except KeyboardInterrupt:
        print("\n\n Stream stopped by user.")
    except Exception as e:
        print(f"\n An error occurred: {type(e).__name__}: {e}")
    finally:
        cv2.destroyAllWindows()


@app.command()
def left_arm(
    action: Annotated[ArmAction, typer.Argument(help="The action to perform.")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")
    print(f"Connected to robot. Action: '{action.value}' on left arm camera.")

    try:
        if action == ArmAction.raw_image:
            image = robot.left_arm_camera.get_raw_image()
            show_image(image, "Left Arm")
        elif action == ArmAction.stream:
            print("Starting raw video stream... Press 'q' in the window or Ctrl+C to stop.")
            for image in robot.left_arm_camera.get_video_stream():
                show_image(image, "Left Arm Stream", is_stream=True)
    except KeyboardInterrupt:
        print("\n\n Stream stopped by user.")
    except Exception as e:
        print(f"\n An error occurred: {type(e).__name__}: {e}")
    finally:
        cv2.destroyAllWindows()


@app.command()
def right_arm(
    action: Annotated[ArmAction, typer.Argument(help="The action to perform.")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")
    print(f"Connected to robot. Action: '{action.value}' on right arm camera.")

    try:
        if action == ArmAction.raw_image:
            image = robot.right_arm_camera.get_raw_image()
            show_image(image, "Right Arm")
        elif action == ArmAction.stream:
            print("Starting raw video stream... Press 'q' or Ctrl+C to stop.")
            for image in robot.right_arm_camera.get_video_stream():
                show_image(image, "Right Arm Stream", is_stream=True)
    except KeyboardInterrupt:
        print("\n\n Stream stopped by user.")
    except Exception as e:
        print(f"\n An error occurred: {type(e).__name__}: {e}")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    app()
