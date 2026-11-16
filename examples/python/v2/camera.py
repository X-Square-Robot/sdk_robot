#!/usr/bin/env python3
"""Camera example: capture, decode, save, and display single frames or streams.

The example keeps the v1 head/left-arm/right-arm/all scenarios while using the
new ``SdkResult`` interface. SDK callbacks only enqueue image results; decoding,
file saving, and GUI operations run on the main thread.

Examples (run from the repository root or an exported artifacts directory):

    python camera.py head rgb-image --server 127.0.0.1:55051 --show
    python camera.py left-arm stream --server 127.0.0.1:55051 --show --seconds 10
    python camera.py all --server 127.0.0.1:55051 --save-dir ./camera_output

``opencv-python`` is used to display JPEG/PNG/depth images; H.264/H.265 also
requires the optional ``av`` package (used by
``x2robot.utils.H26xStreamDecoder``). A single camera is displayed by default;
``all`` only captures frames. Headless environments are detected automatically,
and ``--no-show`` can be used explicitly. ``--save-dir`` saves images and
``--seconds`` limits stream duration for testing.
"""

from __future__ import annotations

import argparse
import os
import queue
import sys
import time
from pathlib import Path
from typing import Any

import x2robot_sdk


class ExampleSkip(RuntimeError):
    """Camera is not present in the selected robot profile."""


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        name = error.get("error_name", "UNKNOWN")
        if name in ("UNAVAILABLE", "UNIMPLEMENTED"):
            raise ExampleSkip(f"{operation}: {error.get('error_message', name)}")
        raise RuntimeError(
            f"{operation} failed: {name}: {error.get('error_message', 'SDK call failed')} "
            f"(request_id={result.request_id or '-'})"
        )
    return result.data


def close_stream(stream):
    require(stream.close(), "CameraStream.close")


_VIDEO_CODEC_HINTS = ("h264", "h265", "h26x", "hevc", "avc")


def _cv_modules():
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError:
        return None, None
    return cv2, np


def is_video_packet(frame: Any) -> bool:
    fmt = str(getattr(frame, "format", "") or "").lower()
    return any(item in fmt for item in _VIDEO_CODEC_HINTS)


def _is_depth(frame: Any) -> bool:
    return "depth" in str(getattr(frame, "format", "") or "").lower()


def _jpeg_or_png(data: bytes) -> bool:
    return data.startswith(b"\xff\xd8") or data.startswith(b"\x89PNG\r\n\x1a\n")


def decode_frame(frame: Any, decoder: Any = None):
    """Decode one standardized ``CameraFrame`` into a BGR ndarray.

    ROS ``compressedDepth`` prepends a 12-byte transport header. It is removed
    only when the payload is not already a normal JPEG/PNG, so the simulator's
    valid JPEG frames and real compressed-depth frames both work.
    """

    data = bytes(getattr(frame, "data", b"") or b"")
    if not data:
        return None
    fmt = str(getattr(frame, "format", "") or "").lower()
    cv2, np = _cv_modules()
    if cv2 is None or np is None:
        return None

    if is_video_packet(frame):
        if decoder is False:
            return None
        if decoder is None:
            try:
                from x2robot.utils import H26xStreamDecoder

                decoder = H26xStreamDecoder()
            except ImportError:
                return None
        return decoder.feed(data, codec_hint=fmt)

    payload = data
    if _is_depth(frame) and not _jpeg_or_png(payload) and len(payload) > 12:
        payload = payload[12:]
    return cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED)


def display_frame(frame, source: str, *, depth: bool = False) -> bool:
    """Display a decoded ndarray; return False when GUI support is absent."""

    cv2, _ = _cv_modules()
    if cv2 is None:
        print(f"{source}: opencv-python is not installed; cannot display")
        return False
    # Qt/GTK OpenCV builds may abort the process (instead of raising) when no
    # display exists. Check the environment before calling imshow so the same
    # example remains usable in CI, SSH sessions and robot containers.
    if os.environ.get("DISPLAY") is None and os.environ.get("WAYLAND_DISPLAY") is None:
        print(f"{source}: no DISPLAY/WAYLAND_DISPLAY; continuing headless")
        return False
    shown = frame
    if depth and getattr(frame, "ndim", 0) == 2:
        shown = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        shown = cv2.applyColorMap(shown, cv2.COLORMAP_JET)
    try:
        cv2.imshow(source, shown)
        return True
    except Exception as error:  # headless OpenCV builds raise here
        print(f"{source}: GUI unavailable ({error}); continuing headless")
        return False


def _display_image(frame, source: str, *, wait: bool, decoded: Any = None) -> bool:
    if decoded is None:
        decoded = decode_frame(frame)
    if decoded is None:
        print(f"{source}: received {len(bytes(frame.data))} bytes ({frame.format}), decode unavailable")
        return False
    shown = display_frame(decoded, source, depth=_is_depth(frame))
    if shown:
        cv2, _ = _cv_modules()
        if wait:
            print("Press any key or close the window to continue; press q to quit a stream.")
            while cv2.getWindowProperty(source, cv2.WND_PROP_VISIBLE) >= 1:
                if cv2.waitKey(100) != -1:
                    break
            cv2.destroyWindow(source)
        else:
            cv2.waitKey(1)
    return shown


def _save_frame(frame: Any, decoded: Any, path: Path) -> Path:
    """Save decoded image when possible, otherwise preserve the encoded bytes."""

    cv2, _ = _cv_modules()
    path.parent.mkdir(parents=True, exist_ok=True)
    if decoded is not None and cv2 is not None:
        output = path.with_suffix(".png")
        if not cv2.imwrite(str(output), decoded):
            raise OSError(f"cannot write {output}")
        return output
    suffix = ".bin"
    fmt = str(getattr(frame, "format", "") or "").lower()
    if "jpeg" in fmt or "jpg" in fmt:
        suffix = ".jpg"
    elif "png" in fmt:
        suffix = ".png"
    elif "h265" in fmt or "hevc" in fmt:
        suffix = ".h265"
    elif "h264" in fmt or "avc" in fmt:
        suffix = ".h264"
    output = path.with_suffix(suffix)
    output.write_bytes(bytes(frame.data))
    return output


def fetch_first_video_frame(stream_fn, label, timeout: float = 8.0) -> tuple[Any, Any]:
    """Decode the first frame of a video stream for a unary image fallback.

    Binocular cameras commonly return an H.26x packet from ``GetImage``.  A
    packet by itself may not contain VPS/SPS/PPS, so the v1 example starts the
    corresponding stream and waits for a decodable keyframe.  Keep that
    behavior in v2 while still exposing the standardized callback result.
    """
    packets: queue.Queue[Any] = queue.Queue(maxsize=64)
    overflow = []

    def on_frame(result: Any) -> None:
        try:
            packets.put_nowait(result)
        except queue.Full:
            if not overflow:
                overflow.append(True)

    opened = require(stream_fn(on_frame), f"{label}.GetStream")
    decoder: Any = None
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if overflow:
                raise RuntimeError(f"{label}: decoder cannot keep up with the camera stream")
            try:
                result = packets.get(timeout=min(0.5, max(0.01, deadline - time.monotonic())))
            except queue.Empty:
                continue
            frame = require(result, f"{label}.Stream")
            if decoder is None and is_video_packet(frame):
                try:
                    from x2robot.utils import H26xStreamDecoder

                    decoder = H26xStreamDecoder(frame.format)
                except ImportError:
                    decoder = False
            decoded = decode_frame(frame, decoder)
            if decoded is not None:
                return frame, decoded
        raise TimeoutError(f"{label}: no decodable video frame in {timeout:.1f}s")
    finally:
        close_stream(opened)


def capture_image(image_fn, stream_fn, label, *, show=True, save_dir=None):
    frame = require(image_fn(), f"{label}.GetImage")
    data = bytes(frame.data)
    if not data:
        raise RuntimeError(f"{label}: empty image")
    decoded = decode_frame(frame)
    if decoded is None and is_video_packet(frame):
        # Match v1's unary-camera behavior: use the stream to obtain a keyframe
        # when the unary packet is not independently decodable.
        print(f"{label}: unary packet is encoded video; waiting for a stream keyframe")
        frame, decoded = fetch_first_video_frame(stream_fn, label)
        data = bytes(frame.data)
    print(f"{label}: format={frame.format} bytes={len(data)} seq={frame.seq} stamp_ns={frame.stamp_ns}")
    if save_dir is not None:
        print(f"  saved {_save_frame(frame, decoded, save_dir / label.lower().replace(' ', '_'))}")
    if show and decoded is not None:
        _display_image(frame, label, wait=True, decoded=decoded)
    elif show:
        print(f"{label}: frame could not be decoded; install opencv-python/av or use --save-dir")
    return frame


def display_video_stream(stream_fn, label, *, seconds=None, show=True, save_dir=None):
    packets: queue.Queue[Any] = queue.Queue(maxsize=64)
    overflow = []

    def on_frame(result: Any) -> None:
        try:
            packets.put_nowait(result)
        except queue.Full:
            # Losing a video reference frame breaks H.26x decoding. Report it
            # explicitly instead of silently dropping packets.
            if not overflow:
                overflow.append(True)

    opened = require(stream_fn(on_frame), f"{label}.GetStream")
    decoder = None
    count = 0
    shown = False
    last_packet = time.monotonic()
    deadline = None if seconds is None else time.monotonic() + max(0.1, seconds)
    try:
        while deadline is None or time.monotonic() < deadline:
            if overflow:
                raise RuntimeError(f"{label}: decoding/display cannot keep up with the camera stream")
            try:
                wait = 0.5 if deadline is None else min(0.5, max(0.01, deadline - time.monotonic()))
                result = packets.get(timeout=wait)
            except queue.Empty:
                if time.monotonic() - last_packet > 8.0:
                    raise TimeoutError(f"{label}: no stream data received for 8s")
                continue
            last_packet = time.monotonic()
            frame = require(result, f"{label}.Stream")
            if not frame.data:
                raise RuntimeError(f"{label}: empty stream image")
            count += 1
            if is_video_packet(frame) and decoder is None:
                try:
                    from x2robot.utils import H26xStreamDecoder

                    decoder = H26xStreamDecoder(frame.format)
                except ImportError:
                    decoder = False
            decoded = decode_frame(frame, decoder)
            if save_dir is not None and (count == 1 or count % 30 == 0):
                path = _save_frame(frame, decoded, save_dir / f"{label.lower().replace(' ', '_')}_{count:06d}")
                print(f"  saved {path}")
            if show and decoded is not None:
                shown = display_frame(decoded, label, depth=_is_depth(frame)) or shown
                cv2, _ = _cv_modules()
                if shown and cv2 is not None and cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        if count == 0:
            raise TimeoutError(f"{label}: no camera data received")
        print(f"{label}: stream packets={count}")
        return count
    finally:
        close_stream(opened)
        if shown:
            cv2, _ = _cv_modules()
            if cv2 is not None:
                cv2.destroyWindow(label)


def head(robot, action, *, show=True, seconds=None, save_dir=None):
    cam = robot.head_camera
    images = {
        "rgb-image": (cam.get_rgb_image, cam.get_rgb_video_stream, "Head RGB"),
        "depth-image": (cam.get_depth_image, cam.get_depth_video_stream, "Head Depth"),
        "left-eye-image": (cam.get_left_eye_image, cam.get_left_eye_video_stream, "Head Left Eye"),
        "right-eye-image": (cam.get_right_eye_image, cam.get_right_eye_video_stream, "Head Right Eye"),
    }
    streams = {
        "rgb-stream": (cam.get_rgb_video_stream, "Head RGB Stream"),
        "depth-stream": (cam.get_depth_video_stream, "Head Depth Stream"),
        "left-eye-stream": (cam.get_left_eye_video_stream, "Head Left Eye Stream"),
        "right-eye-stream": (cam.get_right_eye_video_stream, "Head Right Eye Stream"),
    }
    if action in images:
        capture_image(*images[action], show=show, save_dir=save_dir)
    elif action in streams:
        display_video_stream(*streams[action], show=show, seconds=seconds, save_dir=save_dir)
    else:
        raise ValueError(f"unknown head action {action!r}; choose from {', '.join((*images, *streams))}")


def left_arm(robot, action, *, show=True, seconds=None, save_dir=None):
    cam = robot.left_arm_camera
    if action == "raw-image":
        capture_image(cam.get_raw_image, cam.get_video_stream, "Left Wrist", show=show, save_dir=save_dir)
    elif action == "elbow-image":
        capture_image(cam.get_elbow_image, cam.get_elbow_video_stream, "Left Elbow", show=show, save_dir=save_dir)
    elif action == "stream":
        display_video_stream(cam.get_video_stream, "Left Wrist Stream", show=show, seconds=seconds, save_dir=save_dir)
    elif action == "elbow-stream":
        display_video_stream(cam.get_elbow_video_stream, "Left Elbow Stream", show=show, seconds=seconds, save_dir=save_dir)
    else:
        raise ValueError("left-arm action must be raw-image, elbow-image, stream or elbow-stream")


def right_arm(robot, action, *, show=True, seconds=None, save_dir=None):
    cam = robot.right_arm_camera
    if action == "raw-image":
        capture_image(cam.get_raw_image, cam.get_video_stream, "Right Wrist", show=show, save_dir=save_dir)
    elif action == "elbow-image":
        capture_image(cam.get_elbow_image, cam.get_elbow_video_stream, "Right Elbow", show=show, save_dir=save_dir)
    elif action == "stream":
        display_video_stream(cam.get_video_stream, "Right Wrist Stream", show=show, seconds=seconds, save_dir=save_dir)
    elif action == "elbow-stream":
        display_video_stream(cam.get_elbow_video_stream, "Right Elbow Stream", show=show, seconds=seconds, save_dir=save_dir)
    else:
        raise ValueError("right-arm action must be raw-image, elbow-image, stream or elbow-stream")


def all_captures(robot):
    return [
        ("head_rgb", robot.head_camera.get_rgb_image),
        ("head_depth", robot.head_camera.get_depth_image),
        ("head_left_eye", robot.head_camera.get_left_eye_image),
        ("head_right_eye", robot.head_camera.get_right_eye_image),
        ("left_wrist", robot.left_arm_camera.get_raw_image),
        ("left_elbow", robot.left_arm_camera.get_elbow_image),
        ("right_wrist", robot.right_arm_camera.get_raw_image),
        ("right_elbow", robot.right_arm_camera.get_elbow_image),
    ]


def all_cameras(robot, *, show=False, save_dir=None):
    """Capture all cameras like v1, skipping unconfigured cameras."""
    ok = unavailable = failed = 0
    for label, fetch in all_captures(robot):
        try:
            image = require(fetch(), f"{label}.GetImage")
            if not image.data:
                raise RuntimeError("empty image")
            decoded = None if is_video_packet(image) else decode_frame(image)
            print(f"[ok] {label}: {image.format} {len(image.data)} bytes")
            if save_dir is not None:
                print(f"  saved {_save_frame(image, decoded, save_dir / label)}")
            if show and decoded is not None:
                _display_image(image, label, wait=True, decoded=decoded)
            ok += 1
        except ExampleSkip as error:
            print(f"[skip] {label}: {error}")
            unavailable += 1
        except Exception as error:
            print(f"[error] {label}: {error}")
            failed += 1
    # quanta_x3 intentionally exposes only four H.265 streams.  Its historical
    # unary image methods are all rejected, so use the public streams as the
    # aggregate camera check when every snapshot was skipped.
    if ok == 0 and unavailable == len(all_captures(robot)) and failed == 0 and getattr(robot, "model_name", "") == "quanta_x3":
        print("[info] quanta_x3 snapshots are not public; checking the four public video streams")
        for label, stream in (
            ("Head Left Eye Stream", robot.head_camera.get_left_eye_video_stream),
            ("Head Right Eye Stream", robot.head_camera.get_right_eye_video_stream),
            ("Left Wrist Stream", robot.left_arm_camera.get_video_stream),
            ("Right Wrist Stream", robot.right_arm_camera.get_video_stream),
        ):
            try:
                count = display_video_stream(stream, label, show=show, seconds=1.0, save_dir=save_dir)
                print(f"[stream] {label}: {count} packets")
                ok += 1
            except ExampleSkip as error:
                print(f"[skip] {label}: {error}")
                unavailable += 1
            except Exception as error:
                print(f"[error] {label}: {error}")
                failed += 1
    print(f"Done. ok={ok} unavailable={unavailable} failed={failed}")
    return ok > 0 and failed == 0


def _verify_quanta_x3_streams(robot):
    """Verify the four public Quanta X3 H.265 camera streams.

    Quanta X3 intentionally publishes stream RPCs only.  Its unary snapshot and
    head RGB/depth routes are not part of the public capability matrix, so do
    not probe those routes as part of the safe aggregate verification.
    """
    streams = (
        ("Head Left Eye Stream", robot.head_camera.get_left_eye_video_stream),
        ("Head Right Eye Stream", robot.head_camera.get_right_eye_video_stream),
        ("Left Wrist Stream", robot.left_arm_camera.get_video_stream),
        ("Right Wrist Stream", robot.right_arm_camera.get_video_stream),
    )
    for label, stream in streams:
        count = display_video_stream(stream, label, show=False, seconds=0.5)
        if count <= 0:
            raise AssertionError(f"{label} returned no packets")


def verify(robot, _model="auto"):
    """Non-interactive contract check used by verify_all.py."""
    if _model == "quanta_x3":
        _verify_quanta_x3_streams(robot)
        return
    skipped = 0
    for action in ("rgb-image", "depth-image", "rgb-stream", "depth-stream"):
        try:
            head(robot, action, show=False, seconds=0.5)
        except ExampleSkip as error:
            skipped += 1
            print(f"[skip] head/{action}: {error}")
    for action in ("raw-image", "stream"):
        for side, fetch in (("left-arm", left_arm), ("right-arm", right_arm)):
            try:
                fetch(robot, action, show=False, seconds=0.5)
            except ExampleSkip as error:
                skipped += 1
                print(f"[skip] {side}/{action}: {error}")
    if not all_cameras(robot):
        raise AssertionError("camera snapshot validation failed")
    if skipped:
        print(f"Camera verification completed with {skipped} quanta_x3 capability skips")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", choices=("head", "left-arm", "right-arm", "all", "verify"))
    parser.add_argument("action", nargs="?")
    parser.add_argument("--server", default="127.0.0.1:55051")
    parser.add_argument("--show", action=argparse.BooleanOptionalAction, default=None,
                        help="single camera displays by default; all captures only unless --show")
    parser.add_argument("--save-dir", type=Path)
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    args = parser.parse_args(argv)
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be greater than zero")
    if args.group in ("head", "left-arm", "right-arm") and args.action is None:
        parser.error(f"{args.group} requires an action")
    show = args.group not in ("all", "verify") if args.show is None else args.show
    if show and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("No desktop display; running headless. Use --save-dir to save images.")
        show = False
    robot = None
    try:
        robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
        if args.group == "head":
            head(robot, args.action, show=show, seconds=args.seconds, save_dir=args.save_dir)
        elif args.group == "left-arm":
            left_arm(robot, args.action, show=show, seconds=args.seconds, save_dir=args.save_dir)
        elif args.group == "right-arm":
            right_arm(robot, args.action, show=show, seconds=args.seconds, save_dir=args.save_dir)
        elif args.group == "all":
            return 0 if all_cameras(robot, show=show, save_dir=args.save_dir) else 1
        else:
            verify(robot, getattr(robot, "model_name", "auto"))
        return 0
    except KeyboardInterrupt:
        print("Stream stopped by user.")
        return 0
    except Exception as error:  # noqa: BLE001 - command-line boundary
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            require(robot.close(), "Robot.close")
        cv2, _ = _cv_modules()
        if cv2 is not None and show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
