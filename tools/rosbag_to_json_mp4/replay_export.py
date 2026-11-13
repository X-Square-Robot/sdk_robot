"""Export ROS feedback to the existing Python and C++ replay contracts.

Profiles describe the SDK's public joint order. No resampling, interpolation,
motion RPCs, or conversion of arbitrary recorded commands into motion targets.
"""
import json
import math
from pathlib import Path
import struct


def recording_metadata(bag):
    from ruamel.yaml import YAML
    path = bag / 'sdk_manifest.yaml'
    return (YAML(typ='safe').load(path.read_text()) or {}).get('recording', {}) if path.is_file() else {}


def finite(values):
    if not values or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError('replay requires nonempty finite position values')


def joint_state(data, profile, aggregate):
    names, positions = data.get('name', []), data.get('position', [])
    if names and (len(set(names)) != len(names) or len(names) != len(positions)):
        raise ValueError('joint names must be unique and match the position array')
    expected = profile.get('joint_names')
    if expected and names:
        if not set(expected) <= set(names):
            if aggregate:
                # An aggregate message may omit an entire uninstalled part.
                if not set(expected) & set(names):
                    return None
            raise ValueError('incomplete or unexpected replay joint names: ' + ','.join(expected))
        indices = [names.index(name) for name in expected]
    elif aggregate:
        raise ValueError('aggregate joint states require joint names')
    else:
        indices = list(range(len(positions)))
    result = {'header': data.get('header', {}), 'name': [names[i] for i in indices] if names else []}
    for field in ('position', 'velocity', 'effort'):
        values = data.get(field, [])
        if values and len(values) != len(positions):
            raise ValueError('joint ' + field + ' length differs from position')
        result[field] = [values[i] for i in indices] if values else []
        if result[field]:
            finite(result[field])
    finite(result['position'])
    if profile.get('joint_count') and len(result['position']) != profile['joint_count']:
        raise ValueError('replay joint count does not match the selected model')
    return result


class ReplayExporter:
    def __init__(self, output, connections, metadata, model=None, protobuf=False):
        recorded_model = metadata.get('model')
        # Read archives produced by earlier development builds; emit only public names.
        recorded_model = {'ex001': 'quanta_x1', 'cx002': 'quanta_x2', 'ex002': 'quanta_x3'}.get(recorded_model, recorded_model)
        if model and recorded_model and model != recorded_model:
            raise ValueError('requested replay model differs from the SDK recording manifest')
        model = model or recorded_model
        if model not in {'quanta_x1', 'quanta_x3', 'quanta_x2'}:
            raise ValueError('replay export requires an SDK manifest or --replay-model quanta_x1/quanta_x3/quanta_x2')
        profile = json.loads((Path(__file__).parent/'config'/model/'replay.json').read_text())
        self.types = None
        if protobuf:
            # Reuse the SDK's generated types; do not maintain another protobuf schema.
            from x2robot.sensor_msgs import JointState
            from x2robot.geometry_msgs import PoseStamped
            from x2robot.nav_msgs import Odometry
            from x2robot.sdk.v2 import GripperPosition
            self.types = {'joints': JointState, 'head': JointState, 'pose': PoseStamped,
                          'odom': Odometry, 'scalar': GripperPosition}
        self.directory = output/'replay'
        self.directory.mkdir()
        self.json = (self.directory/'episode.json').open('x', encoding='utf-8')
        self.json.write(json.dumps({'schema_version': 1, 'episode_id': metadata.get('sessionId', 'rosbag'),
                                   'model': profile['replay_model'], 'source_model': model,
                                   'task': 'SDK rosbag replay'}, ensure_ascii=False)[:-1] + ',"frames":[')
        self.files, self.streams, self.routes = {}, {}, {}
        self.first_ns = self.last_ns = None
        self.frames = 0
        self.missing = {}
        self.ignored = {}
        self.closed = False
        accepted = set(metadata.get('acceptedItems', metadata.get('accepted_items', [])))
        topics = {c.topic: c.msgtype for c in connections}
        for name, entry in profile['streams'].items():
            allowed = not accepted or entry.get('item_id', name) in accepted
            aggregate_allowed = allowed or ('joint_states' in accepted and entry.get('aggregate_default', False))
            topic = next((t for t in entry['topics'] if allowed and t in topics), None)
            aggregate = topic is None and aggregate_allowed and entry.get('aggregate_topic') in topics
            if aggregate:
                topic = entry['aggregate_topic']
            if topic is None:
                continue
            if topics[topic] != entry['message_type']:
                raise ValueError('unexpected replay message type on ' + topic)
            self.routes.setdefault(topic, []).append((name, entry, aggregate))

    def write(self, topic, stamp, data):
        if topic not in self.routes:
            self.ignored[topic] = self.ignored.get(topic, 0) + 1
            return
        observation = {}
        for name, profile, aggregate in self.routes[topic]:
            kind = profile['kind']
            if kind == 'joints':
                value = joint_state(data, profile, aggregate)
                if value is None:
                    self.missing[name] = self.missing.get(name, 0) + 1
                    continue
            elif kind == 'head':
                value = {'header': data.get('header', {}), 'name': ['head_pitch_joint', 'head_yaw_joint'],
                         'position': [data['pitch_position'], data['yaw_position']],
                         'velocity': [data['pitch_velocity'], data['yaw_velocity']],
                         'effort': [data['pitch_torque'], data['yaw_torque']]}
                finite(value['position'])
            elif kind == 'scalar':
                value = {'position': data['data']}
                finite([value['position']])
            else:
                value = data
                pose = data['pose']['pose'] if kind == 'odom' else data['pose']
                finite([pose['position'][key] for key in ('x', 'y', 'z')])
                q = [pose['orientation'][key] for key in ('x', 'y', 'z', 'w')]
                finite(q)
                if sum(v*v for v in q) < 1e-12:
                    raise ValueError('replay pose has an invalid zero quaternion')
            if kind in {'joints', 'head'}:
                observation[name] = {'names': value.get('name', []), 'positions': value['position'],
                                     'velocities': value.get('velocity', []), 'efforts': value.get('effort', [])}
            elif kind == 'pose':
                observation[name] = value['pose']
            elif kind == 'odom':
                observation[name] = {'pose': value['pose']['pose'], 'twist': value['twist']['twist']}
            else:
                observation[name] = value
            if self.types:
                payload = bytes(self.types[kind].from_dict(value))
                if name not in self.files:
                    self.files[name] = (self.directory/f'stream_{name}.pb').open('xb')
                self.files[name].write(struct.pack('<QI', int(stamp), len(payload)))
                self.files[name].write(payload)
            stats = self.streams.setdefault(name, {'topic': topic, 'records': 0, 'first_timestamp_ns': int(stamp)})
            stats['records'] += 1
            stats['last_timestamp_ns'] = int(stamp)
        if not observation:
            return
        if self.first_ns is None:
            self.first_ns = int(stamp)
        if self.last_ns is not None and stamp < self.last_ns:
            raise ValueError('replay input timestamps are not monotonic')
        self.last_ns = int(stamp)
        if self.frames:
            self.json.write(',\n')
        json.dump({'timestamp_ns': int(stamp), 'timestamp': (int(stamp)-self.first_ns)/1e9,
                   'observation': observation, 'action': {}}, self.json, ensure_ascii=False, allow_nan=False)
        self.frames += 1

    def close(self):
        if self.closed:
            return
        self.closed = True
        result = {'num_frames': self.frames,
                  'duration': (self.last_ns-self.first_ns)/1e9 if self.frames else 0,
                  'streams': self.streams, 'missing_aggregate_parts': self.missing,
                  'topics_without_replay_commands': self.ignored,
                  'formats': ['episode.json', 'stream_*.pb'] if self.types else ['episode.json']}
        self.json.write('],'+json.dumps(result, ensure_ascii=False, allow_nan=False)[1:]+'\n')
        self.json.close()
        for file in self.files.values():
            file.close()
        return result
