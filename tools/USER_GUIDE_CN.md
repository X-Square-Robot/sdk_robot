# 数据转换工具

[English](USER_GUIDE.md) | [中文](USER_GUIDE_CN.md)

以下命令在 SDK 发布包根目录执行。两个工具均处理本地文件，不连接机器人、不上传数据。

## Quanta X3 SDK 采集数据转 LeRobot

`convert_sdk_quanta_x3_to_lerobot.py` 读取 Python SDK 采集的 episode，导出 21 关节数据集。
校验需要 SDK 示例使用的图像依赖；正式转换还需在环境中安装 NumPy 和 LeRobot。

```bash
python3 tools/convert_sdk_quanta_x3_to_lerobot.py --input-dir collected_data --validate-only
python3 tools/convert_sdk_quanta_x3_to_lerobot.py \
  --input-dir collected_data --output-dir lerobot_data --repo-id local/quanta_x3
```

输出目录必须尚不存在。action 表示采集器记录的“下一观测状态”标签，不是实际下发的运动指令。
升降关节使用米，其余关节位置使用弧度。完整参数见 `--help`。

## ROS2 录制数据转 JSON/MP4 与回放数据

```bash
python3 -m pip install -r tools/rosbag_to_json_mp4/requirements.txt
python3 tools/rosbag_to_json_mp4/convert_rosbag.py \
  --bag-path /录制数据路径 --output-dir converted_data
```

输入支持 ROS2 bag 目录、MCAP 文件或 SDK 录制 tar 包，不要求安装 ROS。
加 `--replay-format json` 可导出 Python 回放数据；加 `--replay-format both` 可同时导出
Python 和 C++ 回放数据，后者还需要安装 SDK Python 包。
没有 SDK manifest 的录制需按实际机型指定 `--replay-model quanta_x1`、`quanta_x2` 或 `quanta_x3`。
`replay_export.py`、`requirements.txt` 和 `config/` 是配套文件，需与转换器一起保留。
转换过程不会执行回放或发送运动指令。
