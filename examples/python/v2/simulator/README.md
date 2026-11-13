# SDK2 仿真示例

示例从 sdk_client 旧 MR !76 迁入 x2robot workspace；原 9 个文件与 Server Bundle 历史副本一致。
旧脚本用于各自记录的历史组合，不能将其 PASS 或默认限制套到新镜像。

## 开源默认 ee 双臂

`opensource_arm_control.py` 对应 `maniparena_quanta_x1_home_relative_cartesian_local`，
继承 !158 的新鲜反馈及恢复检查，坐标改为 home-relative，默认只读。
先按配套 SOP 完成镜像、实例、场景、唯一端点、读取检查。禁止同时使用键盘或 reset。

```bash
python -m examples.python.v2.simulator.opensource_arm_control --arm both
python -m examples.python.v2.simulator.opensource_arm_control --arm left --action move --delta-z 0.018
python -m examples.python.v2.simulator.opensource_arm_control --arm right --action move --delta-z 0.018
```

每侧要求三个连续新样本确认目标和恢复。恢复未确认时停止所有后续动作并保留现场。
末端指令为相对启动/reset home 的目标，重复发送不会累加；关节写入和 Arm.Reset 不支持。
新实例必须重新获取基线，旧基线不可跨 reset 或重启恢复。

内部 `manaenv_local` 示例见下节；不要将开源相对值当作内部绝对位姿。

## 内部 ManaEnv 手臂测试

`manaenv_arm_control.py` 适用于内部 EX001 `manaenv_local`（p92375）的绝对
`lift_link` 末端接口，兼容已交付的 SDK2 Client `6b0203f`。它不适用于开源
home-relative 仿真或真机；坐标系名称相同不能证明部署相同。执行动作前按内部
部署 SOP 核对当前 scene、PID、服务 endpoint 及读取结果，并停止其他控制源。

旧 Client 包的 `quanta_x1/arm_control.py` 在默认 `move` 和 `verify` 中忽略
`--mode end_pose`，先设置关节模式并调用不支持的 `SetJointPositions`，因此返回
`100501/UNIMPLEMENTED`。本示例作为内部仿真专用替换文件；仓库中通用真机示例保持独立。

在本仓库运行：

```bash
python3 examples/python/v2/simulator/manaenv_arm_control.py --server 127.0.0.1:15551 --mode end_pose
```

替换旧 Client 包时，先备份 `artifacts/examples/python/v2/quanta_x1/arm_control.py`，
再将本文件复制到该路径，文件名改为 `arm_control.py`。从 `artifacts/examples/python/v2`
目录仍可使用原命令：

```bash
python3 quanta_x1/arm_control.py --server 127.0.0.1:15551 --mode end_pose
```

- 默认 `--action move --arm left --mode end_pose`。`verify` 同样只测试所选模式与手臂；
  `--arm both` 按左、右依次执行。
- 末端测试先读取时间戳递增且稳定的位姿，设置 SDK/END_POSE 并回读模式，重新确认
  位姿后沿 Z 移动默认 `+0.018 m`，保持原姿态，然后恢复到本次读取的绝对位姿。
  `--delta-z` 可为负数，绝对值限制为 0.01–0.05 m。
- 目标和恢复各需连续 3 个时间戳递增的反馈符合位置 7 mm、角度 0.04 rad 的误差范围。
  只有动作和恢复均确认成功才返回 0；`--settle-timeout` 默认每阶段 10 秒。
- 目标请求报错或被中断也尝试一次恢复。恢复失败报 `RESTORE_UNCONFIRMED`，退出码 1，
  不继续下一只手臂。时间戳倒退或坐标系改变时停止写入，需检查当前场景后重新测试。
- `--mode joint_pos` 配合 `move`/`verify` 在连接前明确拒绝，退出码 2，提示内部
  `SetJointPositions` 不支持；不切换模式、不发送关节写入，也不将其计为通过。
- `--action stream --mode joint_pos` 读取一帧关节状态；`--action stream --mode end_pose`
  读取一帧末端状态。两者不设置工作模式或控制模式，完成后取消流。

成功输出示例：

```text
left: TARGET PASS delta_z=+0.018 m
left: RESTORE PASS
PASS: internal ManaEnv arm move
```

离线回归（不需要安装 SDK，不连接设备）：

```bash
python3 -m unittest discover -s tests -p 'test_manaenv_arm_control.py' -v
```

这是 Python 示例修正，无需重装 wheel 或更新 Server。离线回归和已交付 wheel
导入检查不等于现场动作验收；旧 Bundle 的镜像身份与历史验收记录不随脚本更新改变。
