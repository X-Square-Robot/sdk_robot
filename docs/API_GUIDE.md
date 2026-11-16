# SDK2 API guide

[English](API_GUIDE.md) | [中文](API_GUIDE_CN.md)

## Python: connect, check the result, close

```python
import x2robot_sdk


def require(result):
    if not result.is_success:
        raise RuntimeError(f"request_id={result.request_id}, error={result.error}")
    return result.data


robot = require(x2robot_sdk.connect("192.168.10.1:55051", force_grpc=True))
try:
    info = require(robot.system.get_static_info(timeout=3.0))
    print(info.model)
finally:
    require(robot.close())
```

This example performs no mode change or motion. Request/response messages are
imported from `x2robot.sdk` / `x2robot.sdk.v2`; do not confuse these message types
with the legacy `x2robot.connect` facade. Check `is_success` before reading `data`.
Errors include `error_code`, `error_name`, `error_message`, and a request ID for
matching server logs. A deadline does not establish whether a motion was applied.

## C++ and Python equivalents

| Operation | Python | C++ |
| --- | --- | --- |
| Connect | `x2robot_sdk.connect(address, force_grpc=True)` | `Robot::Create(address, true)` |
| Check result | `is_success`, `data`, `error` | `Ok()`, `Value()`, `Error()` |
| Static information | `robot.system.get_static_info()` | `robot->GetSystem().GetStaticInfo()` |
| Waist joints | `robot.waist.get_joint_states()` | `robot->GetWaistController().GetJointStates()` |
| Waist target | `robot.waist.set_end_pose(...)` | `robot->GetWaistController().SetEndPose(...)` |
| Close | `robot.close()` | `robot->Close()` |

C++ headers are under `include/x2robot/sdk/` and `include/v2/` in the release.
Use generated message types rather than building an independent wire format.
Python timeout arguments are seconds; C++ methods accept chrono durations.
Streaming calls must be cancelled/closed when no longer needed, including on
errors. See each sensor example for the stream object's lifecycle.

## Motion, units and modes

Positions/distances use metres and angles use radians unless the interface
explicitly documents a different unit. A pose contains position and a quaternion;
use the documented reference frame and a normalized quaternion.
Work mode and manipulator control mode must match the operation. A successful
mode RPC alone does not guarantee that the first feedback sample is available.

For Quanta X3, waist joints are lift (m), pitch (rad), yaw (rad). End-pose Z is
used for lift targets and orientation for pitch/yaw; X/Y are not independent
horizontal waist actuators. Start from measured feedback, preserve unused
components, and keep both arm targets refreshed during waist end-pose control.
WBC solves multiple weighted tasks, so actual displacement can differ from the
target and can couple several joints. Use the bounded examples and the model's
configured limits, not target precision as proof that an RPC succeeded.

## Collection and replay

```bash
python examples/python/v2/data_collection_example.py --server 192.168.10.1:55051
python examples/python/v2/data_replay_example.py ./collected_data/episode_0000 \
  --server 192.168.10.1:55051 --mode end_pose --no-chassis
./bin/data_replay_example ./collected_data/episode_0000 \
  --server 192.168.10.1:55051 --mode end_pose --no-chassis
```

Collection does not take over the robot's work mode. Before recording, establish
the feedback-producing mode and complete VR authorization if using teleoperation.
Check the per-stream report and saved files. A first-response timeout or missing
pose stream is a recording failure, not proof of a valid end-pose dataset.
Quanta X3 end-pose replay requires left-arm, right-arm and waist pose streams; it
does not silently fall back to joint replay when those streams are missing.

The synchronized replay refreshes three targets at a nominal 50 Hz. RPC time is
part of the cycle, and slow cycles stretch playback rather than send catch-up
bursts. A 300 ms end-pose RPC timeout produces a warning and continues to the next
cycle without resending that failed instruction; other failures stop replay.
Chassis velocities are clamped per axis by the example; `--no-chassis` disables
chassis commands. Always inspect the recording and attend the first replay.

## Troubleshooting

- Connection failure: check the server's actual IP, port and version, then inspect
  the structured error and request ID. A proxy/VPN can change the route.
- No first stream frame: confirm the robot is publishing that capability and has
  completed its mode/authorization transition before starting collection.
- `UNIMPLEMENTED`: verify the model supports the capability and package/server
  versions agree; it is not a reason to repeat motion requests.
- Mode cleanup timeout: inspect the actual task state before another action.
  A timed-out response can coexist with a completed transition to idle.
- C++ loader errors: retain the complete matching `include/` and `lib/` bundle.
  Do not mix system protobuf/gRPC headers with a different bundled ABI.
