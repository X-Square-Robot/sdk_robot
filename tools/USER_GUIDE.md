# Data conversion tools

[English](USER_GUIDE.md) | [中文](USER_GUIDE_CN.md)

Run these commands from the SDK package root. Both tools process local files;
they do not connect to a robot or upload data.

## Quanta X3 SDK episodes to LeRobot

`convert_sdk_quanta_x3_to_lerobot.py` reads Python SDK collector episodes and
exports a 21-joint dataset. Validation needs the image dependencies used by the
SDK examples; conversion also needs NumPy and LeRobot installed in the environment.

```bash
python3 tools/convert_sdk_quanta_x3_to_lerobot.py --input-dir collected_data --validate-only
python3 tools/convert_sdk_quanta_x3_to_lerobot.py \
  --input-dir collected_data --output-dir lerobot_data --repo-id local/quanta_x3
```

Choose a new output directory. Actions are the collector's next-observed-state
labels, not recorded motion commands. The lift position is in metres; other joint
positions are in radians. Use `--help` for the available arguments.

## ROS2 recording to JSON/MP4 and replay data

```bash
python3 -m pip install -r tools/rosbag_to_json_mp4/requirements.txt
python3 tools/rosbag_to_json_mp4/convert_rosbag.py \
  --bag-path /path/to/recording --output-dir converted_data
```

Input may be a ROS2 bag directory, MCAP file or SDK recording tar archive. A ROS
installation is not required. Add `--replay-format json` to export Python replay
data, or `--replay-format both` for Python and C++ replay data; `both` also requires
the SDK Python package. For recordings without an SDK manifest, specify
`--replay-model quanta_x1`, `quanta_x2` or `quanta_x3` matching the robot.
`replay_export.py`, `requirements.txt` and `config/` are companion files and must
remain beside the converter. Conversion itself performs no replay or motion.
