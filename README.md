# X2Robot SDK 2.0

[English](README.md) | [中文](README_CN.md)

SDK2 provides a C++ client and Python bindings for robot control, sensor streams,
data collection and replay. Python uses `x2robot_sdk`; C++ uses
`x2robot::sdk::Robot`. Both use the same native client and structured RPC results.
Available capabilities depend on the robot model and installed server version.

## Requirements

- Release baseline: Linux x86_64, Ubuntu 22.04 or a compatible newer glibc system.
- Python wheels: CPython 3.10, 3.11, 3.12 and 3.13; select the matching ABI.
- C++ example compilation: a C++17 compiler, CMake 3.16+ and build tools.
- A reachable SDK2 server, usually `ROBOT_IP:55051`; use its configured port.
- Camera decoding/display uses FFmpeg; Python examples also need their declared
  dependencies. A GUI is needed only for display options such as `--show`.

The released client bundle is x86_64. Do not install it on an ARM robot computer.
The robot server is deployed separately. Keep SDK and robot software compatible.

## Install a release

Extract the complete SDK2 package into a dedicated directory and run these
commands from that directory. A CI download contains this directory as `artifacts/`.

```bash
sha256sum -c SHA256SUMS
python3.10 -m venv .venv
.venv/bin/python -m pip install wheel/*-cp310-cp310-manylinux_2_35_x86_64.whl
.venv/bin/python -m pip install -r examples/python/requirements-examples.txt
```

For Python 3.11/3.12/3.13 replace both `python3.10` and the `cp310-cp310` wheel
selector. Do not install all ABI wheels together. Image examples may require
system packages `ffmpeg` and `libgl1`.

```text
include/                    Public C++ and generated protocol/dependency headers
lib/                        SDK and bundled non-system shared libraries
bin/                        Compiled C++ examples
wheel/                      Python wheels, one per supported ABI
examples/cpp/               C++ example sources and CMake build
examples/python/v2/         Standardized Python API examples
examples/python/v1/         SDK2's legacy Python API compatibility examples
docs/                       SDK2 guides in English and Chinese
tools/                      Data conversion utilities; see tools/USER_GUIDE.md
versions.env                x2robot tag/commit and submodule revisions for this build
SHA256SUMS                  Full package checksums
```

The GitHub source export contains examples, tools, headers and documentation.
It does not contain `lib/`, `bin/` or wheels by default. Use the matching complete
release package from [GitHub Releases](https://github.com/X-Square-Robot/sdk_robot/releases)
to run or rebuild it; headers alone are insufficient to link.

In `versions.env`, `x2robot=` identifies the source build: the full tag for a
tag-triggered build (for example, `v2.0.0rc11`), otherwise the full commit SHA.
The other repository fields record the corresponding submodule commits.

## URDF resources

Download the ZIP for your robot model (`quanta_x1`, `quanta_x2`, or `quanta_x3`)
from the matching [GitHub Release](https://github.com/X-Square-Robot/sdk_robot/releases),
along with its `SHA256SUMS`. Only models with a published ZIP are available.
URDF resources are separate Release attachments; the GitHub source tree does not
contain an `urdf/` directory or internal resource metadata.

Verify the downloaded ZIP with `sha256sum --ignore-missing -c SHA256SUMS`, then
extract it to a local directory. Keep the URDF and its mesh directories together
and select the variant matching your robot's hardware. Resource versions may
differ from the SDK version; use the filenames and notes on the selected Release.

## First connection: read only

Replace the address with the robot's actual server address. These commands read
static information and joint feedback without changing the robot's work mode:

```bash
.venv/bin/python examples/python/v2/system.py --server 192.168.10.1:55051 --action static
.venv/bin/python examples/python/v2/all_joint_states.py --server 192.168.10.1:55051 --action once
```

For Python API usage, structured errors and C++ equivalents see the
[API guide](docs/API_GUIDE.md). `--help` on a control example describes its
specific parameters; a `verify` action can change modes or move the robot,
so it is not a universal read-only check.

## Build C++ examples from the package

```bash
cmake -S examples/cpp -B build-examples -DCMAKE_BUILD_TYPE=Release
cmake --build build-examples --parallel 4
```

Keep `include/` and `lib/` from the same release. If building sources exported
separately, set `-DX2ROBOT_SDK_ROOT=/absolute/path/to/the/complete/package`.
See [C++ examples](examples/cpp/README.md) for each binary's CLI syntax; Python
and C++ argument conventions are not interchangeable.

## Examples, collection and replay

| Task | Python v2 | C++ |
| --- | --- | --- |
| System and sensors | `system.py`, `robot_status.py`, `camera.py`, `imu.py` | Same-name sources in `examples/cpp/` |
| Model-specific controls | `desktop/`, `quanta_x1/`, `quanta_x2/`, `quanta_x3/` | Matching model directories |
| Collection | `data_collection_example.py` | `data_collection_example` |
| Replay | `data_replay_example.py` | `data_replay_example` |

See [Python examples](examples/python/v2/README_CN.md),
[Quanta X3 examples](examples/QUANTA_X3_README.md), and the
[collection/replay notes](docs/API_GUIDE.md#collection-and-replay).
Movement and replay require an attended robot with clear space. Inspect the
recording and requested motion before confirming. WBC end-pose control can couple
the arms and waist; target and feedback positions need not match exactly.
