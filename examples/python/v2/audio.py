"""Audio play/replay/stop and microphone recording (v1 behavior, v2 API)."""

from __future__ import annotations

import argparse
import io
import time
import urllib.request
from urllib.parse import urlparse
import wave
from pathlib import Path

import x2robot_sdk
from x2robot.sdk import (
    GetAudioStreamRequest,
    GetSpeechHistoryRequest,
    PlayAudioRequest,
    StopAudioRequest,
    VoicePromptPriority,
)


def require(result, operation):
    if not result.is_success:
        error = result.error or {}
        raise RuntimeError(f"{operation} failed: {(error.get('error_message') or 'SDK call failed')} (request_id={result.request_id or '-'})")
    return result.data


def close_stream(stream):
    close = getattr(stream, "close", None) or getattr(stream, "cancel", None)
    if close:
        close()


def print_result(value):
    print(f"[{'OK' if getattr(value, 'success', True) else 'FAIL'}] message={getattr(value, 'message', '')!r}")
    if getattr(value, "play_id", 0):
        print(f"       play_id={value.play_id}")
    if getattr(value, "resource_id", ""):
        print(f"       resource_id={value.resource_id}")


def audio_request(source, volume, priority, block, cache, resource_id, timeout):
    path = Path(source)
    kwargs = {
        "volume": volume,
        "priority": VoicePromptPriority(priority),
        "block": block,
        "cache": cache,
        "resource_id": resource_id or (source if source.startswith(("http://", "https://")) else ""),
        "timeout_s": timeout,
    }
    if path.is_file():
        kwargs.update(audio_bytes=path.read_bytes(), file_name=path.name, format=path.suffix.lstrip(".") or "wav")
    elif source.startswith(("http://", "https://")):
        # PlayAudioRequest carries bytes; download URLs on the client just as
        # the v1 facade did, then send the exact same request for local files.
        parsed = urlparse(source)
        with urllib.request.urlopen(source, timeout=timeout) as response:
            payload = response.read()
        name = Path(parsed.path).name or "audio"
        suffix = Path(name).suffix.lstrip(".") or "wav"
        kwargs.update(audio_bytes=payload, file_name=name, format=suffix)
    else:
        raise FileNotFoundError(f"audio source does not exist: {source}")
    return PlayAudioRequest(**kwargs)


def play(robot, source, args):
    print(f"Connected. Playing: {source}")
    value = require(robot.audio.play_audio(audio_request(source, args.volume, args.priority, args.block, args.cache, args.resource_id, args.timeout), timeout=args.timeout), "Audio.PlayAudio")
    print_result(value)
    return value


def replay(robot, resource_id, args):
    print(f"Connected. Replaying: {resource_id}")
    value = require(robot.audio.play_audio(PlayAudioRequest(resource_id=resource_id, volume=args.volume, priority=VoicePromptPriority(args.priority), block=args.block, timeout_s=args.timeout), timeout=args.timeout), "Audio.PlayAudio")
    print_result(value)
    return value


def stop(robot, play_id):
    value = require(robot.audio.stop_audio(StopAudioRequest(play_id=play_id), timeout=5.0), "Audio.StopAudio")
    print_result(value)


def stream_microphone(robot, seconds=None, output=None):
    iterator = robot.audio.get_audio_stream(
        GetAudioStreamRequest(sample_rate=16000),
        timeout=None if seconds is None else max(4.0, seconds + 1.0),
    )
    pcm = bytearray()
    info = None
    count = 0
    end = None if seconds is None else time.monotonic() + max(0.1, seconds)
    try:
        for result in iterator:
            value = require(result, "Audio.GetAudioStream")
            info = info or value.audio_info
            if value.audio_data:
                pcm.extend(value.audio_data.data)
            count += 1
            print(f"\rFrames: {count}, Recorded: {len(pcm)} bytes", end="", flush=True)
            if end is not None and time.monotonic() >= end:
                break
    finally:
        close_stream(iterator)
    print()
    if output and pcm:
        with wave.open(output, "wb") as file:
            file.setnchannels(info.channels if info else 1)
            file.setsampwidth(max(1, (info.bit_depth if info else 16) // 8))
            file.setframerate(info.sample_rate if info else 16000)
            file.writeframes(bytes(pcm))
        print(f"Saved to: {output}")
    elif not pcm:
        print("No audio data received, nothing saved.")
    return count, len(pcm)


def _verify_read_only(robot):
    """Verify Quanta X3 audio reads without starting playback.

    Quanta X3 publishes microphone frames and playback/status history queries.  The
    playback control RPCs remain intentionally closed, so an aggregate check
    must never upload or play a generated sound.
    """
    playback = require(
        robot.audio.get_audio_playback_status(timeout=3.0),
        "Audio.GetAudioPlaybackStatus",
    )
    history = require(
        robot.audio.get_speech_history(GetSpeechHistoryRequest(limit=10), timeout=3.0),
        "Audio.GetSpeechHistory",
    )
    count, byte_count = stream_microphone(robot, 0.5, None)
    if count == 0 or byte_count == 0:
        raise AssertionError("Audio.GetAudioStream returned no PCM data")
    print(f"playback_status={getattr(playback, 'json', playback)}")
    print(f"speech_history={getattr(history, 'json', history)}")
    print(f"audio_stream_frames={count} bytes={byte_count}")


def verify(robot, _model="auto"):
    if _model == "quanta_x3":
        _verify_read_only(robot)
        return

    sample = io.BytesIO()
    with wave.open(sample, "wb") as file:
        file.setnchannels(1); file.setsampwidth(2); file.setframerate(16000); file.writeframes(b"\0\0" * 160)
    value = require(robot.audio.play_audio(PlayAudioRequest(audio_bytes=sample.getvalue(), file_name="sdk_example2.wav", format="wav", volume=60, block=True, cache=False, timeout_s=5), timeout=10), "Audio.PlayAudio")
    print_result(value)
    stream_microphone(robot, 0.5, None)
    stop(robot, value.play_id)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", choices=("play", "replay", "stop", "stream", "verify"), default="play")
    parser.add_argument("value", nargs="?", help="source path/URL, resource id, or play id")
    parser.add_argument("--server", default="localhost:50051")
    parser.add_argument("--volume", type=int, default=80)
    parser.add_argument("--priority", type=int, choices=(0, 1, 2), default=2)
    parser.add_argument("--block", action="store_true")
    parser.add_argument("--cache", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--resource-id", default="")
    parser.add_argument("--timeout", type=float, default=600.0)
    parser.add_argument("--seconds", type=float, default=None,
                        help="optional stream duration; omit to stream until Ctrl+C")
    parser.add_argument("--output", default="./audio_output.wav")
    args = parser.parse_args(argv)
    robot = require(x2robot_sdk.connect(args.server, force_grpc=True), "connect")
    try:
        if args.action == "verify":
            verify(robot, getattr(robot, "model_name", "auto"))
        elif args.action == "play":
            if not args.value:
                parser.error("play requires a local file or URL")
            play(robot, args.value, args)
        elif args.action == "replay":
            if not args.value:
                parser.error("replay requires a resource id")
            replay(robot, args.value, args)
        elif args.action == "stop":
            stop(robot, int(args.value or 0))
        else:
            stream_microphone(robot, args.seconds, args.output)
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except Exception as error:
        print(f"FAIL: {error}")
        return 1
    finally:
        require(robot.close(), "Robot.Close")


if __name__ == "__main__":
    raise SystemExit(main())
