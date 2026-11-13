#!/usr/bin/env python3
"""Record selected robot topics, stop, download the closed bag, optionally parse it.

No data stream subscriptions or robot motion commands are sent by this example.
Keep --session-id to query or stop a recording after a client interruption.
"""
import argparse
import configparser
import json
import math
import re
import select
from pathlib import Path
import subprocess
import sys
import time
import threading
import uuid


def checked(result):
    if not result.is_success:
        raise RuntimeError(str(result.error))
    status = result.data
    if getattr(status, 'state', None) == 'FAILED':
        raise RuntimeError(f'{status.error_code}: {status.error_message}')
    return status


def wait_for(robot, session_id, states, timeout, status_timeout=30):
    from x2robot.sdk.v2 import RecordingSessionRequest
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        status = checked(robot.recording.get_recording_status(RecordingSessionRequest(session_id=session_id),
                                                            timeout=min(status_timeout, max(0.001, end - time.monotonic()))))
        if status.state in states:
            return status
        if status.state in {'DELETING', 'DELETED'}:
            raise RuntimeError(f'session {session_id} is {status.state}; use a new session ID or retry --action delete')
        time.sleep(0.2)
    raise TimeoutError(f'session {session_id} is still pending; query/stop with the same --session-id')


class DownloadProgress:
    """Throttled client-side output. 100% bytes does not imply verified success."""
    def __init__(self):
        self.started = time.monotonic()
        self.last_print = None
        print('download_started: waiting for metadata', flush=True)

    def __call__(self, received, total, verified):
        now = time.monotonic()
        if not verified and self.last_print is not None and now - self.last_print < 1:
            return
        self.last_print = now
        seconds = max(now - self.started, 0.001)
        speed = received / seconds
        percent = 100 * received / total if total else 100
        eta = f'{(total - received) / speed:.0f}s' if speed else '--'
        state = 'download_complete' if verified else 'download_progress'
        suffix = ' | MD5 verified' if verified else ' | awaiting verification' if received == total else ''
        print(f'{state}={percent:.1f}% {received / (1 << 20):.1f}/{total / (1 << 20):.1f} MiB'
              f' | {speed / (1 << 20):.1f} MiB/s | ETA {eta}{suffix}', flush=True)


class RecordingUpdates:
    def __init__(self):
        self._lock = threading.Lock()
        self._error = None

    def callback(self, result):
        # Callback work stays short; report failure on the example's main thread.
        try:
            status = checked(result)
            print('recording_state='+status.state, flush=True)
            if getattr(status, 'error_code', ''):
                raise RuntimeError(f'{status.error_code}: {status.error_message}')
            if status.state in {'STOPPING', 'FINALIZING', 'READY'}:
                raise RuntimeError(f'recording ended early (state={status.state})')
        except Exception as error:
            with self._lock:
                if self._error is None:
                    self._error = error

    def check(self):
        with self._lock:
            if self._error is not None:
                raise self._error


def record_for(updates, duration):
    """Wait locally; StartRecording owns the single 5-second status poller."""
    end = time.monotonic() + duration
    if not duration:
        print('Recording; press Enter to stop: ', end='', flush=True)
    while not duration or time.monotonic() < end:
        updates.check()
        if duration:
            time.sleep(max(0, min(0.1, end - time.monotonic())))
        elif select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            return
    updates.check()

PRESETS = ('quanta_x1', 'quanta_x1_dual_head_camera', 'quanta_x2', 'quanta_x3')


def validate_items(items):
    if not isinstance(items, list) or not 1 <= len(items) <= 512:
        raise ValueError('select between 1 and 512 item IDs; use [all] explicitly for all items')
    if any(not isinstance(item, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', item) for item in items):
        raise ValueError('each item must be a nonempty logical ID')
    if len(set(items)) != len(items):
        raise ValueError('duplicate item IDs')
    if 'all' in items and items != ['all']:
        raise ValueError('all must be the only item')


def config_path(args, model='quanta_x1'):
    if args.config is not None:
        return args.config
    preset = args.preset or model
    if preset not in PRESETS:
        raise ValueError(f'no default recording preset for {model}; use --config or --items')
    return Path(__file__).resolve().parent / 'config' / (preset + '.conf')


def read_items_config(path):
    config = configparser.ConfigParser(interpolation=None, delimiters=('=',), comment_prefixes=('#',),
                                       empty_lines_in_values=False)
    config.optionxform = str
    with path.open(encoding='utf-8') as source:
        config.read_file(source)
    if config.defaults() or 'preset' not in config or set(config['preset']) != {'name', 'model'}:
        raise ValueError(f'{path}: expected [preset] name/model without DEFAULT values')
    name, model = config['preset']['name'], config['preset']['model']
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', name) or model not in {'quanta_x1', 'quanta_x2', 'quanta_x3'}:
        raise ValueError(f'{path}: invalid preset name or model')
    items = []
    for section in config.sections():
        if section == 'preset':
            continue
        if not section.startswith('item.') or set(config[section]) != {'enabled', 'description'}:
            raise ValueError(f'{path}: expected [item.ID] enabled/description')
        item_id = section[len('item.'):]
        validate_items([item_id])
        enabled, description = config[section]['enabled'], config[section]['description']
        if item_id == 'all' or enabled not in {'true', 'false'} or not description.strip():
            raise ValueError(f'{path}: invalid item definition: {item_id}')
        items.append({'id': item_id, 'enabled': enabled == 'true', 'description': description})
    if not 1 <= len(items) <= 512:
        raise ValueError(f'{path}: expected between 1 and 512 item definitions')
    return {'preset': name, 'model': model, 'items': items}


def selected_items(robot, args):
    if args.items is not None:
        validate_items(args.items)
        return args.items
    path = config_path(args, robot.model_name)
    config = read_items_config(path)
    if config['model'] != robot.model_name:
        raise ValueError(f"preset model {config['model']} differs from robot model {robot.model_name}")
    items = [item['id'] for item in config['items'] if item['enabled']]
    validate_items(items)
    print('config='+str(path), flush=True)
    return items


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', default='127.0.0.1:55051')
    parser.add_argument('--config', type=Path, help='recording .conf with item descriptions and enabled flags')
    parser.add_argument('--preset', choices=PRESETS, help='built-in preset; default: connected model (list: quanta_x1)')
    parser.add_argument('--items', nargs='+', default=None, help='explicit item IDs; overrides configuration selection')
    parser.add_argument('--session-id', default=None)
    parser.add_argument('--duration', type=float, default=10, help='seconds; 0 waits for Enter')
    parser.add_argument('--output-dir', type=Path, default=Path('data_collection_v2'))
    cleanup = parser.add_mutually_exclusive_group()
    cleanup.add_argument('--cleanup-after-download', dest='cleanup_after_download', action='store_true',
                         help='delete remote bag/archive after verified local download (default)')
    cleanup.add_argument('--keep-robot-data', dest='cleanup_after_download', action='store_false',
                         help='retain remote bag/archive after download')
    parser.set_defaults(cleanup_after_download=None)
    parser.add_argument('--raw-only', action='store_true', help='download rosbag tar without parsing')
    parser.add_argument('--replay-format', choices=['none', 'json', 'both'], default='both',
                        help='parsed output also includes Python/C++ replay data; ignored with --raw-only')
    parser.add_argument('--action', choices=['list', 'collect', 'start', 'stop', 'status', 'download', 'delete'], default='collect')
    parser.add_argument('--status-timeout', type=float, default=30, help='timeout per status query in seconds; default 30')
    parser.add_argument('--finalize-timeout', type=float, default=600)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not math.isfinite(args.finalize_timeout) or args.duration < 0 or args.finalize_timeout <= 0:
        parser.error('duration must be nonnegative and finalize-timeout positive')
    if not math.isfinite(args.status_timeout) or args.status_timeout <= 0:
        parser.error('status-timeout must be finite and positive')
    if args.action in {'stop', 'download', 'delete'} and not args.session_id:
        parser.error('--session-id is required for stop/download/delete')
    if args.session_id is not None and not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', args.session_id):
        parser.error('--session-id must contain 1–96 letters, digits, underscores or hyphens')
    if args.cleanup_after_download is not None and args.action not in {'collect', 'download'}:
        parser.error('--cleanup-after-download/--keep-robot-data require collect or download')
    if args.cleanup_after_download is None:
        args.cleanup_after_download = args.action in {'collect', 'download'}
    if args.action == 'list':
        print(json.dumps(read_items_config(config_path(args)), ensure_ascii=False, indent=2))
        return
    import x2robot_sdk as sdk
    from x2robot.sdk.v2 import StartRecordingRequest, RecordingSessionRequest
    session_id = args.session_id or (uuid.uuid4().hex if args.action in {'start', 'collect'} else '')
    if session_id:
        print('session_id='+session_id, flush=True)
    with checked(sdk.connect(args.address)) as robot:
        request = RecordingSessionRequest(session_id=session_id)
        if args.action == 'delete':
            print(checked(robot.recording.delete_recording(request, timeout=args.status_timeout)).to_json())
            return
        if args.action == 'status':
            print(checked(robot.recording.get_recording_status(request, timeout=args.status_timeout)).to_json())
            return
        if args.action in {'collect', 'start'}:
            args.items = selected_items(robot, args)
            print(f'items={args.items}', flush=True)
            # Print the ID before START, so timeout recovery never needs a new ID.
            updates = RecordingUpdates()
            checked(robot.recording.start_recording(StartRecordingRequest(session_id=session_id, data_items=args.items),
                                                   timeout=10, callback=updates.callback, poll_interval=5.0,
                                                   status_timeout=args.status_timeout))
            if args.action == 'start':
                print(wait_for(robot, session_id, {'RECORDING'}, 30, args.status_timeout).to_json())
                return
            try:
                wait_for(robot, session_id, {'RECORDING'}, 30, args.status_timeout)
                record_for(updates, args.duration)
            except BaseException:
                # Preserve the original resource/network failure even when STOP also fails.
                try:
                    checked(robot.recording.stop_recording(request, timeout=10))
                except Exception as error:
                    print(f'Stop failed: {error}', file=sys.stderr)
                raise
            else:
                checked(robot.recording.stop_recording(request, timeout=10))
        elif args.action == 'stop':
            checked(robot.recording.stop_recording(request, timeout=10))
        status = wait_for(robot, session_id, {'READY'}, args.finalize_timeout, args.status_timeout)
        robot.recording.cancel_recording_callback(session_id)
        print(status.to_json(), flush=True)
        if args.action == 'stop':
            return
        args.output_dir.mkdir(parents=True, exist_ok=True)
        bag_path = args.output_dir / (session_id + '.tar')
        checked(robot.files.download_file(status.file_id, bag_path, timeout=args.finalize_timeout, progress=DownloadProgress()))
        cleanup_error = None
        if args.cleanup_after_download:
            try:
                deleted = checked(robot.recording.delete_recording(request, timeout=args.status_timeout))
                if deleted.state != 'DELETED':
                    raise RuntimeError(f'unexpected cleanup state: {deleted.state}')
                print('remote_cleanup=DELETED session_id='+session_id, flush=True)
            except Exception as error:
                cleanup_error = RuntimeError(f'remote cleanup failed; verified local file retained at {bag_path}: {error}')
                print(str(cleanup_error), file=sys.stderr, flush=True)
    print('downloaded='+str(bag_path), flush=True)
    if not args.raw_only:
        converter = Path(__file__).resolve().parents[4] / 'tools/rosbag_to_json_mp4/convert_rosbag.py'
        subprocess.run([sys.executable, str(converter), '--bag-path', str(bag_path), '--output-dir', str(args.output_dir/session_id),
                        '--replay-format', args.replay_format], check=True)
    if cleanup_error is not None:
        raise cleanup_error


if __name__ == '__main__':
    main()
