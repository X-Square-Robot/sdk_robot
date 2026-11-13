# SDK2 API 使用说明

[English](API_GUIDE.md) | [中文](API_GUIDE_CN.md)

## Python：连接、检查结果、关闭

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

此例不切模式、不发送运动。消息类型从 `x2robot.sdk` / `x2robot.sdk.v2` 导入，
与旧版 `x2robot.connect` 兼容入口区分使用。先检查 `is_success`，再读 `data`。
错误包含 `error_code`、`error_name`、`error_message` 等字段，request ID 可关联服务端日志。
请求超时不能证明运动指令没有执行。

## C++ 与 Python 对应关系

| 操作 | Python | C++ |
| --- | --- | --- |
| 连接 | `x2robot_sdk.connect(address, force_grpc=True)` | `Robot::Create(address, true)` |
| 结果 | `is_success`、`data`、`error` | `Ok()`、`Value()`、`Error()` |
| 静态信息 | `robot.system.get_static_info()` | `robot->GetSystem().GetStaticInfo()` |
| 腰部关节 | `robot.waist.get_joint_states()` | `robot->GetWaistController().GetJointStates()` |
| 腰部末端 | `robot.waist.set_end_pose(...)` | `robot->GetWaistController().SetEndPose(...)` |
| 关闭 | `robot.close()` | `robot->Close()` |

C++ 头文件位于产物 `include/x2robot/sdk/` 和 `include/v2/`。
使用生成的协议消息类型，避免自行拼接协议。Python 的 timeout 单位为秒，C++ 使用 chrono 时长。
流式接口在结束及异常时都需要取消或关闭，具体生命周期参考对应传感器示例。

## 运动、单位与模式

除接口单独注明外，距离使用米、角度使用弧度。位姿由 position 和四元数组成，
使用对应参考坐标系并保证四元数归一化。工作模式、机械臂控制模式要与操作匹配；
切模式 RPC 成功不代表反馈首包已到达。

Quanta X3 的腰部关节依次是升降（m）、俯仰（rad）、偏航（rad）。
末端 Z 用于升降目标，姿态用于俯仰、偏航，X/Y 不是独立的腰部水平平移轴。
以当前反馈为起点，保持不修改的分量；腰部末端控制时同步刷新双臂目标。
WBC 采用多任务加权求解，实际位移可能偏离目标并伴随多关节联动。
遵循示例和机型配置中的限制，不以精确到达目标作为 RPC 是否成功的判断。

## 录制和回放

```bash
python examples/python/v2/data_collection_example.py --server 192.168.10.1:55051
python examples/python/v2/data_replay_example.py ./collected_data/episode_0000 \
  --server 192.168.10.1:55051 --mode end_pose --no-chassis
./bin/data_replay_example ./collected_data/episode_0000 \
  --server 192.168.10.1:55051 --mode end_pose --no-chassis
```

录制程序不接管工作模式。开始前先确保反馈已发布；使用遥操时，先完成遥操和 VR 授权。
结束后检查逐流统计与保存文件。首包超时、末端流缺失不能视作有效末端录制。
Quanta X3 末端回放要求左臂、右臂、腰部三路位姿，不会在缺少末端数据时悄悄降级为关节回放。

同步回放以标称 50 Hz 刷新三路目标，RPC 耗时计入周期；延迟较大时延长回放时间，
不突发补发积压目标。单次末端 RPC 超时为 300 ms，超时提示 warning 后继续下一周期，
不重发该条失败指令；其他调用失败停止回放。示例按轴裁剪底盘速度，
`--no-chassis` 关闭底盘指令。首次回放前检查数据，执行时现场看护。

## 常见排查

- 连接失败：核对实际服务 IP、端口、版本及错误 request ID，检查代理/VPN 是否改变路由。
- 流首包超时：确认该机型支持该能力，话题正在发布，且切模式/授权已经完成。
- `UNIMPLEMENTED`：核对机型支持情况和客户端/服务端兼容性，不重复尝试运动指令。
- 退出模式超时：继续动作前确认实际任务状态；响应超时也可能已完成待机切换。
- C++ 动态库错误：保留同版本完整 `include/`、`lib/`，不要混用不同 ABI 的系统 protobuf/gRPC。
