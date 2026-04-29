import typer
import wave
from typing_extensions import Annotated
from x2robot import connect


app = typer.Typer(help="A CLI to interact with the robot's audio system.")


def _print_result(result):
    status = "OK" if result.success else "FAIL"
    print(f"[{status}] message={result.message!r}")
    # StopAudioResult 没有这些字段,用 getattr 兼容
    play_id = getattr(result, "play_id", 0)
    resource_id = getattr(result, "resource_id", "")
    if play_id:
        print(f"       play_id={play_id}")
    if resource_id:
        print(f"       resource_id={resource_id}")


@app.command()
def play(
    source: Annotated[str, typer.Argument(help="Local file path or http(s):// URL")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
    volume: Annotated[int, typer.Option(help="Volume 1-100")] = 80,
    priority: Annotated[int, typer.Option(help="0=urgent, 1=high, 2=normal")] = 2,
    block: Annotated[bool, typer.Option(help="Wait until playback finishes")] = False,
    cache: Annotated[bool, typer.Option(help="Cache on server for replay")] = True,
    resource_id: Annotated[str, typer.Option(help="Optional cache-key hint")] = "",
    timeout: Annotated[float, typer.Option(help="block=True timeout seconds")] = 600.0,
):
    """Upload (from file or URL) and play audio on the robot."""
    robot = connect(f"x2://{server}")
    print(f"Connected. Playing: {source}")
    result = robot.audio.play_audio(
        source=source,
        volume=volume,
        priority=priority,
        block=block,
        cache=cache,
        resource_id=resource_id or None,
        timeout=timeout,
    )
    _print_result(result)


@app.command()
def replay(
    resource_id: Annotated[str, typer.Argument(help="Cached resource id returned from play_audio")],
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
    volume: Annotated[int, typer.Option(help="Volume 1-100")] = 80,
    priority: Annotated[int, typer.Option(help="0=urgent, 1=high, 2=normal")] = 2,
    block: Annotated[bool, typer.Option(help="Wait until playback finishes")] = False,
    timeout: Annotated[float, typer.Option(help="block=True timeout seconds")] = 600.0,
):
    """Replay a previously cached resource by its id."""
    robot = connect(f"x2://{server}")
    print(f"Connected. Replaying: {resource_id}")
    result = robot.audio.replay(
        resource_id=resource_id,
        volume=volume,
        priority=priority,
        block=block,
        timeout=timeout,
    )
    _print_result(result)


@app.command()
def stop(
    play_id: Annotated[int, typer.Argument(help="Play id to stop (0 = stop all)")] = 0,
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
):
    """Stop a specific playback task or all tasks."""
    robot = connect(f"x2://{server}")
    result = robot.audio.stop_audio(play_id=play_id)
    _print_result(result)


@app.command()
def stream(
    server: Annotated[str, typer.Option(help="Server address")] = "localhost:50051",
    output: Annotated[str, typer.Option(help="Output WAV file path")] = "./audio_output.wav",
):
    """Stream microphone audio and save to a WAV file."""
    robot = connect(f"x2://{server}")
    print("Connected. Starting audio stream... Press Ctrl+C to stop.\n")

    pcm_buffer = bytearray()
    audio_info = None
    frame_count = 0

    try:
        for audio in robot.audio.get_audio_stream():
            frame_count += 1

            if audio_info is None:
                audio_info = audio.audio_info
                print(f"Audio format: channels={audio_info.channels}, "
                      f"sample_rate={audio_info.sample_rate}Hz, "
                      f"bit_depth={audio_info.bit_depth}, "
                      f"format={audio_info.sample_format}")
                print()

            if audio.audio_data and audio.audio_data.data:
                pcm_buffer.extend(audio.audio_data.data)

            bytes_per_sec = audio_info.sample_rate * audio_info.channels * (audio_info.bit_depth // 8)
            duration = len(pcm_buffer) / bytes_per_sec if bytes_per_sec else 0
            print(f"\rFrames: {frame_count}, Recorded: {duration:.1f}s, Size: {len(pcm_buffer)} bytes",
                  end="", flush=True)

    except KeyboardInterrupt:
        print(f"\n\nStopped by user. Total frames: {frame_count}")

    if audio_info and len(pcm_buffer) > 0:
        sample_width = audio_info.bit_depth // 8
        with wave.open(output, "wb") as wf:
            wf.setnchannels(audio_info.channels)
            wf.setsampwidth(sample_width)
            wf.setframerate(audio_info.sample_rate)
            wf.writeframes(bytes(pcm_buffer))
        print(f"Saved to: {output}")
    else:
        print("No audio data received, nothing saved.")


if __name__ == "__main__":
    app()
