# SDK client examples

The exported examples are grouped by language:

- `cpp/`: C++ examples for the current standardized API and their standalone
  `CMakeLists.txt`; every executable is a small self-contained source;
- `python/v1/`: v1-compatible examples for the historical `x2robot`
  facade;
- `python/v2/`: the same scenarios as v1, using the standardized `SdkResult`
  API. Shared scenarios stay at the root; model-specific examples are grouped
  in `desktop/`, `quanta_x1/`, `quanta_x2/`, and `quanta_x3/`.

For example, the model-specific files are laid out as:

```text
python/v2/desktop/{arm_control,gripper_control,master_arm_control}.py
python/v2/quanta_x1/{arm_control,gripper_control,lift_control,master_arm_control}.py
python/v2/quanta_x2/{arm_control,gripper_control,waist_control}.py
python/v2/quanta_x3/{arm_control,gripper_control,waist_control}.py
```

Quanta X3 reuses the root Python v2 and C++ examples for shared capabilities
such as head, chassis, system, audio, navigation and read-only calls. Its
`quanta_x3/` directories contain only the arm, gripper and waist controls; this
model uses waist control for lift, pitch and yaw. See [Quanta X3 examples](QUANTA_X3_README.md) for
the commands and trajectories.

Build all C++ examples from the extracted artifact root:

```bash
cmake -S examples/cpp -B examples/cpp/build
cmake --build examples/cpp/build -j
```

For example, the C++ counterpart of `python/v2/camera.py` is built as
`camera`:

```bash
examples/cpp/build/camera 127.0.0.1:55051
examples/cpp/build/data_collection_example 127.0.0.1:55051 ./vr_pose.txt
```

If the examples directory was moved away from the artifact root, pass it explicitly:

```bash
cmake -S examples/cpp -B examples/cpp/build \
  -DX2ROBOT_SDK_ROOT=/path/to/sdk-client-artifact
```

Install and run the matching Python wheel:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install wheel/*-cp310-cp310-manylinux_2_35_x86_64.whl  # Python 3.10
python examples/python/v2/camera.py head rgb-image --server 127.0.0.1:55051 --show
```

Use the same relative path in `python/v1/` and `python/v2/` to compare an old
example with its new API implementation. C++ counterparts call the native SDK
directly. `bench_*` and `verify_all` are additional performance/validation tools.

## Desktop / Quanta X1 C++ master-arm motion

`desktop_master_arm_control` and `quanta_x1_master_arm_control` share the
slave-arm C++ TOPPRA profile from `linear_toppra_trajectory.hpp`. This is the
existing triangular/trapezoidal solution for a straight two-point path,
checked against the Python slave-arm example's `toppra` planner. No separate
quintic profile is used. The default minimum duration is 4 seconds:

```bash
examples/cpp/build/desktop_master_arm_control <server:55051> left move joint_pos --duration 8
examples/cpp/build/desktop_master_arm_control <server:55051> left move end_pose
examples/cpp/build/desktop_master_arm_control <server:55051> left move end_pose --z-delta -0.05 --duration 8
```

The velocity/acceleration constraints match the Python slave-arm examples:
joints 1.0 rad/s and 3.0 rad/s²; Cartesian translation 2.2 m/s and 0.3 m/s².
Joint commands are sampled at 500 Hz and pose commands at 200 Hz.
`--duration` accepts a finite minimum in `(0, 120]` seconds. It stretches the
TOPPRA timeline when requested; plans exceeding 120 seconds are rejected.
RPC delays stretch execution without skipping points or sending catch-up bursts.

`move joint_pos` defaults to +0.262 rad on J1. `--joint-delta` accepts signed
J1 offsets in `[-0.262, 0.262]` rad only for `move joint_pos`; other joints keep
their measured positions. Feedback and target limit checks remain active.

`move end_pose` defaults to **Z +0.10 m (10 cm)** from the current pose, with
X, Y and orientation unchanged. `--z-delta` accepts signed offsets in
`[-0.10, 0.10]` m only for `move end_pose`. This numeric bound does not
establish reachability; choose a target within the robot's workspace.
For orientation changes, shortest-path SLERP follows elapsed trajectory time,
as in the Python slave-arm example. The v2 Python Desktop and Quanta X1
master-arm examples also default to `--pose-delta 0.10` on the Z axis.

`zero` retains its existing target (all joints zero, or Cartesian origin with
identity orientation). `both` runs left then right and stops on the first
failure. `gravity` is only valid with `set_mode`.

After the final command, feedback is checked for up to 3 seconds (joint error
≤0.01 rad, or position error ≤0.002 m and rotation error ≤0.02 rad).
Invalid feedback, RPC rejection, interruption or failure to reach the target
returns an error; exit cleanup restores IDLE. Physical tracking must be
validated separately from the planned command trajectory.
