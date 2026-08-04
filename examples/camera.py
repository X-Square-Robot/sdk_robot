import os
import queue
import threading
import time
from enum import Enum
from typing import Optional

import cv2
import grpc
import numpy as np
import typer
from typing_extensions import Annotated

from x2robot import connect
from x2robot.sensor_msgs import CompressedImage
from x2robot.utils import H26xStreamDecoder


# H.26x/H.264/H.265 video packets cannot be decoded with cv2.imdecode,
# which only supports JPEG/PNG and other still-image formats; route them to PyAV via format.
_VIDEO_CODEC_HINTS = ("h264", "h265", "h26x", "hevc", "avc")


def is_video_packet(image: CompressedImage) -> bool:
    fmt = (image.format or "").lower()
    return any(hint in fmt for hint in _VIDEO_CODEC_HINTS)


_global_decoder: Optional[H26xStreamDecoder] = None


def get_decoder() -> H26xStreamDecoder:
    global _global_decoder
    if _global_decoder is None:
        _global_decoder = H26xStreamDecoder()
    return _global_decoder


def decode_image(image: CompressedImage) -> Optional[np.ndarray]:
    if not image or not image.data:
        return None

    image_bytes = bytes(image.data)
    fmt = (image.format or "").lower()

    # Depth maps and JPEG paths are unchanged
    if "depth" in fmt:
        image_bytes = image_bytes[12:]
        np_arr = np.frombuffer(image_bytes, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    if not is_video_packet(image):
        np_arr = np.frombuffer(image_bytes, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Video packets: feed the stream decoder; return ndarray only when a new frame is produced.
    decoder = get_decoder()
    return decoder.feed(image_bytes, codec_hint=fmt)


def to_display_frame(frame: np.ndarray, is_depth: bool) -> np.ndarray:
    """Convert a decoded frame to an 8-bit BGR image for display/saving (colorize depth maps)."""
    if not is_depth:
        return frame
    normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def fetch_first_video_frame(stream_fn, timeout: float = 15.0) -> Optional[np.ndarray]:
    """Feed packets from a video stream until the first frame is decoded.

    H.26x "single-frame" requests often return non-keyframes (no parameter sets,
    no reference frames) that cannot be decoded in isolation. Instead, read from
    the video stream until parameter sets (VPS/SPS/PPS) and a keyframe arrive,
    then return the first successfully decoded frame (BGR).

    Note: the underlying gRPC unary_stream blocks on the generator when the first
    packet is slow to arrive unless an RPC timeout is set (local deadline checks
    never run). Pass timeout through to the stream call so gRPC raises
    DEADLINE_EXCEEDED when no data arrives.
    """
    decoder = H26xStreamDecoder()          # Isolated decoder; does not affect the global one
    deadline = time.monotonic() + timeout
    received_packets = 0
    try:
        for image in stream_fn(timeout=timeout):
            received_packets += 1
            frame = decoder.feed(bytes(image.data), codec_hint=image.format)
            if frame is not None:
                return frame
            if time.monotonic() > deadline:
                print(
                    f"Warning: timeout ({timeout}s) waiting for IDR/decoded frame "
                    f"after receiving {received_packets} packets"
                )
                return None
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.DEADLINE_EXCEEDED:
            if received_packets:
                print(
                    f"Warning: timeout ({timeout}s) waiting for IDR/decoded frame "
                    f"after receiving {received_packets} packets"
                )
            else:
                print(f"Warning: timeout ({timeout}s) waiting for video stream data")
        else:
            print(f"Stream error: {e.code().name}: {e.details()}")
    except Exception as e:  # noqa: BLE001
        print(f"Error while fetching first video frame: {e}")
    return None


def show_image(image: Optional[CompressedImage] = None,
               source: str = "Image",
               is_stream: bool = False,
               frame: Optional[np.ndarray] = None):
    # Already-decoded frame (from fetch_first_video_frame)
    if frame is not None:
        cv2.imshow(source, frame)
        if is_stream:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt("Stream stopped by user ('q' pressed).")
        else:
            print("-> Press any key in the image window or close it to continue.")
            while cv2.getWindowProperty(source, cv2.WND_PROP_VISIBLE) >= 1:
                if cv2.waitKey(100) != -1:
                    break
        return

    if not image or not image.data:
        print(f"Warning: Empty image received from {source}; skipping.")
        return

    try:
        decoded = decode_image(image)   # Supports H.265 via PyAV
        if decoded is None:
            print(f"Could not decode image from {source}.")
            return

        is_depth = "depth" in (image.format or "").lower()
        display_frame = to_display_frame(decoded, is_depth)  # Normalize depth maps for display
        cv2.imshow(source, display_frame)

        if is_stream:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt("Stream stopped by user ('q' pressed).")
        else:
            print("-> Press any key in the image window or close it to continue.")
            while cv2.getWindowProperty(source, cv2.WND_PROP_VISIBLE) >= 1:
                if cv2.waitKey(100) != -1:
                    break
    except KeyboardInterrupt:
        raise
    except Exception as e:
        print(f"OpenCV Error: {e}")


def _iter_stream_with_idle_timeout(stream_fn, idle_timeout: float):
    """Consume the gRPC stream in a background thread; yield frames on the main thread with idle timeout.

    gRPC unary_stream timeout applies to the entire stream, which is unsuitable for
    long-running live playback (the stream is cut off at the deadline). Use idle-timeout
    semantics instead: keep playing as long as a packet arrives within idle_timeout,
    and treat the stream as stalled only after idle_timeout seconds with no data.
    """
    frames: "queue.Queue" = queue.Queue(maxsize=16)
    sentinel = object()
    holder = {}

    def worker():
        try:
            for item in stream_fn():
                frames.put(item)
        except Exception as e:  # noqa: BLE001 — handled/reported on the main thread
            holder["error"] = e
        finally:
            frames.put(sentinel)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()

    while True:
        try:
            item = frames.get(timeout=idle_timeout)
        except queue.Empty:
            raise TimeoutError(
                f"no stream data received for {idle_timeout:.0f}s "
                "(camera not publishing?)"
            )
        if item is sentinel:
            if "error" in holder:
                raise holder["error"]
            return
        yield item


def display_video_stream(stream_fn, label: str, idle_timeout: float = 8.0):
    """Play an H.26x video stream with idle timeout, status hints, and error reporting."""
    print(f"Starting {label}... waiting for data (press 'q' or Ctrl+C to stop).")
    video_decoder = H26xStreamDecoder()
    shown = False
    waited_for_keyframe = False
    try:
        for image in _iter_stream_with_idle_timeout(stream_fn, idle_timeout):
            if not image or not image.data:
                continue
            image_bytes = bytes(image.data)
            if is_video_packet(image):
                frame = video_decoder.feed(image_bytes, codec_hint=image.format)
            else:
                frame = decode_image(image)
            if frame is None:
                # Data received but no keyframe yet; log once to avoid spam
                if not shown and not waited_for_keyframe:
                    print("Receiving data, waiting for keyframe to decode...")
                    waited_for_keyframe = True
                continue

            is_depth = "depth" in (image.format or "").lower()
            display_frame = to_display_frame(frame, is_depth)
            if not shown:
                h, w = display_frame.shape[:2]
                print(f"Streaming {label} ({w}x{h}). Press 'q' in the window to stop.")
                shown = True
            cv2.imshow(label, display_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt("Stream stopped by user ('q' pressed).")
    except TimeoutError as e:
        print(f"Warning: {e}")
    except grpc.RpcError as e:
        print(f"Stream error: {e.code().name}: {e.details()}")


class HeadAction(str, Enum):
    rgb_image = "rgb-image"
    depth_image = "depth-image"
    left_eye_image = "left-eye-image"
    right_eye_image = "right-eye-image"
    rgb_stream = "rgb-stream"
    depth_stream = "depth-stream"
    left_eye_stream = "left-eye-stream"
    right_eye_stream = "right-eye-stream"


class ArmAction(str, Enum):
    raw_image = "raw-image"          # Wrist
    elbow_image = "elbow-image"      # Elbow
    stream = "stream"                # Wrist video stream
    elbow_stream = "elbow-stream"    # Elbow video stream


app = typer.Typer(help="A CLI to interact with the robot's cameras and display images.")


@app.command()
def head(
    action: Annotated[HeadAction, typer.Argument(help="The action to perform with the head camera.")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")
    print(f"Connected to robot. Action: '{action.value}' on head camera.")

    cam = robot.head_camera
    # Binocular left/right eye cameras use H.26x; single packets cannot decode alone, fetch first frame from stream.
    unary = {
        HeadAction.rgb_image: (cam.get_rgb_image, "Head RGB"),
        HeadAction.depth_image: (cam.get_depth_image, "Head Depth"),
        HeadAction.left_eye_image: (
            lambda: fetch_first_video_frame(cam.get_left_eye_video_stream),
            "Head Left Eye",
        ),
        HeadAction.right_eye_image: (
            lambda: fetch_first_video_frame(cam.get_right_eye_video_stream),
            "Head Right Eye",
        ),
    }
    streams = {
        HeadAction.rgb_stream: (cam.get_rgb_video_stream, "Head RGB Stream"),
        HeadAction.depth_stream: (cam.get_depth_video_stream, "Head Depth Stream"),
        HeadAction.left_eye_stream: (cam.get_left_eye_video_stream, "Head Left-Eye Stream"),
        HeadAction.right_eye_stream: (cam.get_right_eye_video_stream, "Head Right-Eye Stream"),
    }

    try:
        if action in unary:
            fn, label = unary[action]
            result = fn()
            if isinstance(result, np.ndarray):
                show_image(source=label, frame=result)
            else:
                show_image(image=result, source=label)
        else:
            fn, label = streams[action]
            display_video_stream(fn, label)
    except KeyboardInterrupt:
        print("\n\n Stream stopped by user.")
    except Exception as e:
        print(f"\n An error occurred: {type(e).__name__}: {e}")
    finally:
        cv2.destroyAllWindows()


def _arm_command(cam, arm_label: str, action: ArmAction):
    # Configurations with elbow cameras use H.26x; single-frame requests fetch the first frame from stream.
    unary = {
        ArmAction.raw_image: (
            lambda: fetch_first_video_frame(cam.get_video_stream),
            f"{arm_label} Wrist",
        ),
        ArmAction.elbow_image: (
            lambda: fetch_first_video_frame(cam.get_elbow_video_stream),
            f"{arm_label} Elbow",
        ),
    }
    streams = {
        ArmAction.stream: (cam.get_video_stream, f"{arm_label} Wrist Stream"),
        ArmAction.elbow_stream: (cam.get_elbow_video_stream, f"{arm_label} Elbow Stream"),
    }
    try:
        if action in unary:
            fn, label = unary[action]
            result = fn()
            if isinstance(result, np.ndarray):
                show_image(source=label, frame=result)
            else:
                show_image(image=result, source=label)
        else:
            fn, label = streams[action]
            display_video_stream(fn, label)
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
    _arm_command(robot.left_arm_camera, "Left Arm", action)


@app.command()
def right_arm(
    action: Annotated[ArmAction, typer.Argument(help="The action to perform.")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    robot = connect(f"x2://{server}")
    print(f"Connected to robot. Action: '{action.value}' on right arm camera.")
    _arm_command(robot.right_arm_camera, "Right Arm", action)


# Capture one frame from every camera. Cross-platform:
#   - Monocular configuration: head RGB/depth + left/right wrist.
#   - Binocular configuration: left/right eye + left/right wrist + left/right elbow.
# H.26x-encoded frames cannot be decoded with cv2; only format and byte count are printed (save/display skipped).
def _all_captures(robot):
    return [
        ("head_rgb",        lambda: robot.head_camera.get_rgb_image()),
        ("head_depth",      lambda: robot.head_camera.get_depth_image()),
        ("head_left_eye",   lambda: robot.head_camera.get_left_eye_image()),
        ("head_right_eye",  lambda: robot.head_camera.get_right_eye_image()),
        ("left_wrist",      lambda: robot.left_arm_camera.get_raw_image()),
        ("left_elbow",      lambda: robot.left_arm_camera.get_elbow_image()),
        ("right_wrist",     lambda: robot.right_arm_camera.get_raw_image()),
        ("right_elbow",     lambda: robot.right_arm_camera.get_elbow_image()),
    ]


@app.command("all")
def all_cameras(
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
    save_dir: Annotated[Optional[str], typer.Option(help="If set, decodable frames are written here as PNG.")] = None,
    show: Annotated[bool, typer.Option(help="Display each decodable frame in a window.")] = False,
):
    """Snapshot every camera on the robot, skipping ones unavailable on this platform."""
    robot = connect(f"x2://{server}")
    print(f"Connected to robot ({robot.robot_model}). Reading all cameras...\n")

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    ok, unavailable, failed = 0, 0, 0
    for name, fetch in _all_captures(robot):
        try:
            image = fetch()
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.UNAVAILABLE:
                unavailable += 1
                print(f"[skip] {name:15s} not available on this platform")
            else:
                failed += 1
                print(f"[err ] {name:15s} {e.code().name}: {e.details()}")
            continue
        except Exception as e:  # noqa: BLE001
            failed += 1
            print(f"[err ] {name:15s} {type(e).__name__}: {e}")
            continue

        if not image or not image.data:
            failed += 1
            print(f"[warn] {name:15s} empty image")
            continue

        ok += 1
        size = len(image.data)
        if is_video_packet(image):
            # H.26x single packets report metadata only; continuous streams use the video decoder.
            print(f"[ ok ] {name:15s} {image.format:12s} {size:>8d} bytes (encoded video packet)")
            continue

        frame = decode_image(image)
        if frame is None:
            print(f"[ ok ] {name:15s} {image.format:12s} {size:>8d} bytes (undecodable by cv2)")
            continue

        is_depth = "depth" in (image.format or "").lower()
        print(f"[ ok ] {name:15s} {image.format:12s} {size:>8d} bytes -> {frame.shape}")
        display_frame = to_display_frame(frame, is_depth)
        if save_dir:
            path = os.path.join(save_dir, f"{name}.png")
            cv2.imwrite(path, display_frame)
            print(f"       saved {path}")
        if show:
            cv2.imshow(name, display_frame)

    print(f"\nDone. ok={ok}  unavailable(skipped)={unavailable}  failed={failed}")
    if show and ok:
        print("Press any key in a window to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    app()
