#!/usr/bin/env python3
"""Stream a ROS2 bag (directory, MCAP or SDK tar) to per-topic JSON and MP4.

No ROS installation or LeRobot dependency is required. See requirements.txt.
H.264/H.265 fragments are parsed across messages. Undecodable leading frames
are counted, never replaced with black frames or duplicated to invent an FPS.
"""
from __future__ import annotations
import argparse
from collections import deque
from contextlib import contextmanager
import dataclasses
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import tarfile
import tempfile


def json_value(value):
    if dataclasses.is_dataclass(value):
        return {f.name: json_value(getattr(value, f.name)) for f in dataclasses.fields(value) if f.name != '__msgtype__'}
    if hasattr(value, 'tolist'):
        return json_value(value.tolist())
    if isinstance(value, (list, tuple)):
        return [json_value(x) for x in value]
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, bytes):
        return list(value)
    return value


def topic_name(topic):
    return re.sub(r'[^a-zA-Z0-9_-]', '_', topic).strip('_')[:100] + '_' + hashlib.sha256(topic.encode()).hexdigest()[:10]


def joints_by_name(data):
    """Index aggregate feedback on the client without assuming a model or joint order."""
    names = data.get('name', [])
    if any(not isinstance(name, str) or not name for name in names) or len(set(names)) != len(names):
        raise ValueError('aggregate joint names must be nonempty and unique')
    fields = {}
    for field in ('position', 'velocity', 'effort'):
        values = data.get(field, [])
        if values:
            if len(values) != len(names):
                raise ValueError('aggregate joint ' + field + ' length differs from name')
            fields[field] = values
    return {name: {field: values[i] for field, values in fields.items()} for i, name in enumerate(names)}


@contextmanager
def bag_source(path):
    path = Path(path).resolve()
    if not path.is_file() or path.suffix == '.mcap':
        yield path
        return
    with tempfile.TemporaryDirectory(prefix='sdk-rosbag-') as directory:
        root = Path(directory)
        with tarfile.open(path, 'r:*') as archive:
            total = 0
            for entry in archive:
                name = Path(entry.name)
                if name.is_absolute() or '..' in name.parts or not (entry.isfile() or entry.isdir()):
                    raise ValueError('archive must contain only relative regular files/directories')
                total += entry.size
                if entry.size < 0 or total > 128 * 1024**3:
                    raise ValueError('archive exceeds extraction limit')
                dest = root / name
                if dest.exists() and not entry.isdir():
                    raise ValueError('duplicate archive member')
                if entry.isdir():
                    dest.mkdir(parents=True, exist_ok=True)
                else:
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    with archive.extractfile(entry) as source, dest.open('xb') as output:
                        shutil.copyfileobj(source, output, 1 << 20)
        candidates = list(root.rglob('metadata.yaml'))
        if len(candidates) != 1:
            raise ValueError('SDK archive must contain exactly one rosbag metadata.yaml')
        yield candidates[0].parent


class JsonTopic:
    def __init__(self, path, topic, msgtype):
        self.aggregate_joints = topic == '/joint_states' and msgtype == 'sensor_msgs/msg/JointState'
        self.output = path.open('x', encoding='utf-8')
        self.output.write(json.dumps({'topic': topic, 'type': msgtype}, ensure_ascii=False)[:-1] + ',"messages":[')
        self.count = 0

    def write(self, stamp, data):
        record = {'timestamp_ns': int(stamp), 'data': json_value(data)}
        if self.aggregate_joints:
            record['joints_by_name'] = joints_by_name(record['data'])
        if self.count:
            self.output.write(',\n')
        json.dump(record, self.output, ensure_ascii=False, allow_nan=False)
        self.count += 1

    def close(self):
        self.output.write(']}\n')
        self.output.close()


class VideoTopic:
    def __init__(self, path, topic, msgtype):
        import av
        self.av = av
        self.path = path
        self.container = None
        self.stream = None
        self.decoder = None
        self.codec = None
        self.spans = deque()
        self.frames = JsonTopic(path.with_suffix('.frames.json'), topic, msgtype)
        self.first_stamp = None
        self.last_pts = -1
        self.last_source_stamp = None
        self.input_messages = 0
        self.decode_errors = 0
        self.empty_messages = 0
        self.fallback_timestamps = 0
        self.adjusted_timestamps = 0

    def emit(self, frame, stamp):
        stamp = int(stamp)
        if self.container is None:
            self.container = self.av.open(str(self.path), mode='w')
            self.stream = self.container.add_stream('libx264', rate=30)
            self.stream.width, self.stream.height = frame.width, frame.height
            self.stream.pix_fmt = 'yuv420p'
            self.stream.time_base = Fraction(1, 1_000_000)
            self.stream.codec_context.time_base = Fraction(1, 1_000_000)
            self.stream.options = {'crf': '18', 'preset': 'fast'}
            self.first_stamp = stamp
        if (frame.width, frame.height) != (self.stream.width, self.stream.height):
            raise ValueError('camera resolution changed within recording')
        raw_pts = (stamp - self.first_stamp) // 1000
        pts = max(self.last_pts + 1, raw_pts)
        self.adjusted_timestamps += int(pts != raw_pts)
        self.last_pts = pts
        frame.pts, frame.time_base = pts, Fraction(1, 1_000_000)
        for packet in self.stream.encode(frame):
            self.container.mux(packet)
        self.frames.write(stamp, {'frame_index': self.frames.count, 'video_pts_us': pts})

    def packets(self, packets):
        for packet in packets:
            stamp = self.spans[0][1] if self.spans else self.last_source_stamp
            consumed = packet.size
            while consumed and self.spans:
                size, source_stamp = self.spans[0]
                take = min(size, consumed)
                consumed -= take
                if take == size:
                    self.spans.popleft()
                else:
                    self.spans[0][0] -= take
            packet.pts = packet.dts = int(stamp)
            packet.time_base = Fraction(1, 1_000_000_000)
            try:
                decoded = self.decoder.decode(packet)
            except self.av.error.InvalidDataError:
                self.decode_errors += 1
                continue
            for frame in decoded:
                if frame.pts is not None and frame.time_base is not None:
                    frame_stamp = int(frame.pts * frame.time_base * 1_000_000_000)
                else:
                    self.fallback_timestamps += 1
                    frame_stamp = stamp
                self.emit(frame, frame_stamp)

    def write(self, stamp, msg):
        import numpy as np
        self.input_messages += 1
        self.last_source_stamp = int(stamp)
        payload = bytes(msg.data)
        if not payload:
            self.empty_messages += 1
            return
        fmt = str(getattr(msg, 'format', '')).lower()
        codec = 'hevc' if 'h265' in fmt or 'hevc' in fmt else 'h264' if 'h264' in fmt or 'avc' in fmt else None
        if codec:
            if self.decoder is None:
                self.codec = codec
                self.decoder = self.av.CodecContext.create(codec, 'r')
            if self.codec != codec:
                raise ValueError('camera codec changed within recording')
            self.spans.append([len(payload), int(stamp)])
            self.packets(self.decoder.parse(payload))
        elif hasattr(msg, 'encoding'):
            encoding = msg.encoding.lower()
            channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(encoding)
            if channels is None:
                raise ValueError(f'unsupported raw image encoding: {encoding}')
            pixels = np.frombuffer(payload, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
            if channels != 1:
                pixels = pixels.reshape(msg.height, msg.width, channels)
            frame = self.av.VideoFrame.from_ndarray(pixels, format={'rgb8':'rgb24','bgr8':'bgr24','mono8':'gray','rgba8':'rgba','bgra8':'bgra'}[encoding])
            self.emit(frame, stamp)
        else:
            decoder = self.av.CodecContext.create('png' if 'png' in fmt else 'mjpeg', 'r')
            for frame in decoder.decode(self.av.Packet(payload)):
                self.emit(frame, stamp)

    def close(self):
        try:
            if self.decoder:
                self.packets(self.decoder.parse(b''))
                try:
                    frames = self.decoder.decode(None)
                except self.av.error.InvalidDataError:
                    self.decode_errors += 1
                    frames = []
                for frame in frames:
                    if frame.pts is not None and frame.time_base is not None:
                        stamp = int(frame.pts * frame.time_base * 1_000_000_000)
                    else:
                        self.fallback_timestamps += 1
                        stamp = self.last_source_stamp
                    self.emit(frame, stamp)
            if self.container:
                for packet in self.stream.encode():
                    self.container.mux(packet)
        finally:
            if self.container:
                self.container.close()
            self.frames.close()
        if self.frames.count == 0:
            raise ValueError(f'no decodable frames for {self.path.name}; inspect camera format/keyframes')


def convert(source, output, replay_format='none', replay_model=None):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.rosbag-convert-', dir=output.parent))
    writers = {}
    replay = None
    summary = {'schema_version': 1, 'topics': {}, 'timestamp_unit': 'nanoseconds', 'non_finite_float': 'null'}
    try:
        with bag_source(source) as bag, AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
            if replay_format != 'none':
                # Also support importlib-based users/tests of this standalone script.
                import sys
                module_dir = str(Path(__file__).resolve().parent)
                if module_dir not in sys.path:
                    sys.path.insert(0, module_dir)
                from replay_export import ReplayExporter, recording_metadata
                if replay_format not in {'json', 'both'}:
                    raise ValueError('replay format must be none, json or both')
                replay = ReplayExporter(stage, reader.connections, recording_metadata(bag),
                                        replay_model, protobuf=replay_format == 'both')
            for connection in reader.connections:
                topic = connection.topic
                if topic in writers:
                    if summary['topics'][topic]['type'] != connection.msgtype:
                        raise ValueError('conflicting message types on one topic')
                    continue
                video = connection.msgtype in {'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/Image'}
                name = topic_name(topic) + ('.mp4' if video else '.json')
                # A registered camera may publish no messages (for example while asleep).
                # Open its decoder only when data arrives; do not invent an empty MP4.
                writers[topic] = None if video else JsonTopic(stage/name, topic, connection.msgtype)
                summary['topics'][topic] = {'type': connection.msgtype, 'file': name, 'messages': 0, 'video': video}
            for connection, stamp, raw in reader.messages():
                msg = reader.deserialize(raw, connection.msgtype)
                if writers[connection.topic] is None:
                    writers[connection.topic] = VideoTopic(stage/summary['topics'][connection.topic]['file'],
                                                          connection.topic, connection.msgtype)
                writers[connection.topic].write(stamp, msg)
                summary['topics'][connection.topic]['messages'] += 1
                if replay is not None:
                    replay.write(connection.topic, stamp, json_value(msg) if connection.topic in replay.routes else {})
            if not any(entry['messages'] for entry in summary['topics'].values()):
                raise ValueError('bag contains no messages')
            for topic, writer in list(writers.items()):
                del writers[topic]
                if writer is None:
                    summary['topics'][topic].update(file=None, decoded_frames=0, skipped_reason='no_messages')
                    continue
                writer.close()
                if isinstance(writer, VideoTopic):
                    summary['topics'][topic].update(decoded_frames=writer.frames.count, decode_errors=writer.decode_errors,
                        empty_messages=writer.empty_messages, first_frame_timestamp_ns=writer.first_stamp,
                        timestamp_fallbacks=writer.fallback_timestamps, adjusted_timestamps=writer.adjusted_timestamps,
                        frame_timestamps=writer.path.with_suffix('.frames.json').name)
            if replay is not None:
                summary['replay'] = replay.close()
            (stage/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        stage.rename(output)
        return summary
    finally:
        if replay is not None:
            replay.close()
        for writer in writers.values():
            try:
                if writer is not None:
                    writer.close()
            except Exception:
                pass
        if stage.exists():
            shutil.rmtree(stage)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag-path', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--replay-format', choices=['none', 'json', 'both'], default='none',
                        help='also export replay/episode.json and optionally C++ stream_*.pb (both requires the SDK Python package)')
    parser.add_argument('--replay-model', choices=['quanta_x1', 'quanta_x3', 'quanta_x2'],
                        help='required for replay export of a plain bag without an SDK manifest')
    args = parser.parse_args()
    result = convert(args.bag_path, args.output_dir, args.replay_format, args.replay_model)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
