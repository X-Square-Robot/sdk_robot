# X2Robot SDK 2.0

[English](README.md) | [中文](README_CN.md)

SDK2 提供 C++ 客户端和 Python 绑定，支持机器人控制、传感器流、数据采集和回放。
Python 使用 `x2robot_sdk`，C++ 使用 `x2robot::sdk::Robot`，两者复用同一原生客户端
和结构化 RPC 结果。具体能力取决于机型与安装的服务端版本。

## 环境要求

- 发布基线：Linux x86_64，Ubuntu 22.04 或兼容的更新 glibc 系统。
- Python wheel：CPython 3.10、3.11、3.12、3.13，按当前解释器 ABI 选择。
- 编译 C++ 示例：C++17 编译器、CMake 3.16 及以上版本和构建工具。
- 能访问 SDK2 服务，地址通常为 `机器人IP:55051`，以实际配置端口为准。
- 相机解码、显示需要 FFmpeg；Python 示例需安装声明的依赖，只有 `--show` 等显示功能需要图形桌面。

发布的客户端包为 x86_64，不可直接安装到 ARM 机器人计算机。机器人服务端单独部署，
使用前确认 SDK 与整机软件版本兼容。

## 安装发布包

将完整 SDK2 压缩包解压到独立目录，在该目录执行以下命令。
CI 下载的产物中，这个目录名为 `artifacts/`。

```bash
sha256sum -c SHA256SUMS
python3.10 -m venv .venv
.venv/bin/python -m pip install wheel/*-cp310-cp310-manylinux_2_35_x86_64.whl
.venv/bin/python -m pip install -r examples/python/requirements-examples.txt
```

使用 Python 3.11、3.12、3.13 时，同时替换解释器和 `cp310-cp310` wheel 选择条件，
不要一次安装全部 ABI 的 wheel。图像示例可能还需要系统包 `ffmpeg` 和 `libgl1`。

```text
include/                    C++ 公共头文件、生成的协议头文件及依赖头文件
lib/                        SDK 和非系统动态依赖库
bin/                        已编译 C++ 示例
wheel/                      各 Python ABI 对应的 wheel
examples/cpp/               C++ 示例源码及 CMake 入口
examples/python/v2/         标准化 Python API 示例
examples/python/v1/         SDK2 内的旧 Python API 兼容层示例
docs/                       SDK2 中英文说明
tools/                      数据转换工具，见 tools/USER_GUIDE_CN.md
versions.env                本次构建的 x2robot tag/commit 及子仓 commit
SHA256SUMS                  完整发布包校验和
```

GitHub 源码导出默认包含示例、工具、头文件和文档，
不含 `lib/`、`bin/` 和 wheel。
运行或重新编译时，请从 [GitHub Releases](https://github.com/X-Square-Robot/sdk_robot/releases)
取得相同版本的完整发布包；只有头文件不能完成链接。

`versions.env` 中的 `x2robot=` 标识构建来源：tag 触发构建时记录完整 tag
（例如 `v2.0.0rc11`），其他构建记录完整 commit SHA；其余仓库字段记录对应子仓 commit。

## URDF 资源

从对应版本的 [GitHub Release](https://github.com/X-Square-Robot/sdk_robot/releases)
下载所用机型（`quanta_x1`、`quanta_x2` 或 `quanta_x3`）的 ZIP 和 `SHA256SUMS`。
只有已上传 ZIP 的机型提供资源下载。URDF 通过独立 Release 附件分发，
GitHub 源码目录不包含 `urdf/` 或内部资源元数据。

运行 `sha256sum --ignore-missing -c SHA256SUMS` 校验已下载的 ZIP，再解压到本地目录。
保持 URDF 与网格目录的相对位置，并选择与机器人硬件一致的变体。
资源版本可能与 SDK 版本不同，以所选 Release 的附件名和说明为准。

## 首次连接：只读检查

将地址替换成实际机器人服务地址。以下命令读取静态信息和关节反馈，不切换工作模式：

```bash
.venv/bin/python examples/python/v2/system.py --server 192.168.10.1:55051 --action static
.venv/bin/python examples/python/v2/all_joint_states.py --server 192.168.10.1:55051 --action once
```

Python 调用、结构化错误及 C++ 对应方法见 [API 使用说明](docs/API_GUIDE_CN.md)。
控制示例的参数以各自 `--help` 为准；`verify` 可能包含切模式或运动，不能一概作为只读检查。

## 用发布包编译 C++ 示例

```bash
cmake -S examples/cpp -B build-examples -DCMAKE_BUILD_TYPE=Release
cmake --build build-examples --parallel 4
```

`include/` 和 `lib/` 必须来自同一发布版本。单独下载示例源码时，配置
`-DX2ROBOT_SDK_ROOT=/完整发布包的绝对路径`。
具体命令见 [C++ 示例](examples/cpp/README.md)，两种语言的命令行参数形式不完全相同。

## 示例、录制和回放

| 功能 | Python v2 | C++ |
| --- | --- | --- |
| 系统和传感器 | `system.py`、`robot_status.py`、`camera.py`、`imu.py` | `examples/cpp/` 下同名源码 |
| 机型控制 | `desktop/`、`quanta_x1/`、`quanta_x2/`、`quanta_x3/` | 对应机型目录 |
| 采集 | `data_collection_example.py` | `data_collection_example` |
| 回放 | `data_replay_example.py` | `data_replay_example` |

参阅 [Python 示例](examples/python/v2/README_CN.md)、
[Quanta X3 示例](examples/QUANTA_X3_README.md) 和
[录制回放说明](docs/API_GUIDE_CN.md#录制和回放)。
运动和回放时需现场看护、活动空间清空，执行前核对数据与动作意图。
WBC 末端控制可能引起双臂和腰部联动，目标位姿与实际反馈不保证完全一致。
