# SDK2 全关节状态 Client 调用

使用 `all_joint_states.py` 调用文档中的两个只读接口：

| RPC | Python 调用 | 返回类型 |
| --- | --- | --- |
| State.GetAllJointStates | robot.state.get_all_joint_states(timeout=10.0) | SdkResult；成功时 data 为 JointState |
| State.GetAllJointStatesStream | robot.state.get_all_joint_states_stream(timeout=10.0) | 迭代器；每一帧是 SdkResult |

脚本不切换工作模式、控制模式或发送动作。兼容已交付的 Client `6b0203f`，
在原有 `x2robot_sdk` 虚拟环境中运行；无需替换 wheel、Server 或手臂脚本。
测试地址默认 `127.0.0.1:15551`，用 `--server` 指定实际 SDK Server 的 gRPC 地址。

## 调用命令

解压测试包后，在包含 `all_joint_states.py` 的目录执行。仓库直接运行时，
脚本路径为 `examples/python/v2/all_joint_states.py`。

```bash
# 单次读取
python3 all_joint_states.py --server 127.0.0.1:15551 --action once

# 读取 5 帧流数据
python3 all_joint_states.py --server 127.0.0.1:15551 --action stream --count 5

# 两个接口依次调用，保存结果供回传
python3 all_joint_states.py --server 127.0.0.1:15551 --action both --count 5 --output joint_states_result.json
```

默认 `--action both --count 5 --timeout 10`。`timeout` 单位为秒；流的 timeout
是整个流 RPC 的期限，增加 count 时可调整，但脚本限制不超过 300 秒。

## 输出和判断

每帧以 JSON 输出 `operation`、`request_id`、`joint_count`、`frame_id`、
`stamp_ns`、`name`、`position`、`velocity`、`effort`。`name[i]` 对应其他非空数组的
第 i 项；不假定关节个数或顺序。ROS JointState 允许速度、力矩等未提供字段为空数组，
脚本保留空数组，不补零。转动关节位置为 rad、移动关节为 m；速度分别为 rad/s、m/s，
effort 分别为 N·m、N；数据是否提供取决于服务端反馈。

`both --count 5` 成功时先输出 1 帧单次结果，再输出 5 帧流结果，最后显示：

```text
PASS: all joint states (6 frame(s))
```

退出码 0 表示接口调用成功、收到所需帧数且数组结构与数值基本校验通过；
不代表动作、全量物理关节覆盖或数据新鲜度已验收。时间戳按服务端原值显示，
可能属于仿真时钟；不按主机墙钟比较。返回错误、流提前结束或数据非法时退出码 1，
参数不合法时退出码 2。SDK 错误及 request_id 会显示在输出中。

提供 `--output` 时，成功或失败结果都会保存到 JSON（会覆盖同名文件）。
测试反馈请附命令、终端输出与该 JSON。连接失败时先核对服务是否启动和端口，
不要通过反复切换控制模式处理只读接口错误。

## 自己写 Client 代码

连接成功后，单次结果检查 `result.is_success`，再取 `result.data`。
流应使用 `for result in robot.state.get_all_joint_states_stream(timeout=10.0)`，
对每一帧分别检查 `result.is_success`。State 流不是手臂原生流句柄，
不使用 `.data.read(deadline_ms=...)`。完整的连接、错误处理和资源释放见脚本。

## 接口来源

已交付 Client 提交 `6b0203f09db1d3481f791401bd6917dd5be01f48`：

- Python 协议定义：`x2robot/sdk/v2/__init__.py` 的 StateStub，4028、4043 行。
- Python 统一结果包装：`native/python/x2robot_sdk/services.py` 的 ServiceClient。
- C++ Client：`native/include/x2robot/sdk/service/core.hpp`，69、74 行。
- Server：`sdk2/include/transport/grpc/core_services.hpp` 的 StateGrpcService。

验证范围：使用已交付 wheel 做离线模拟返回与序列化检查；此脚本包不包含新 wheel，
不改变冻结 Bundle，也不代表已经在测试机完成在线读取验收。
