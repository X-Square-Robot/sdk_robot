#pragma once

// C++ counterpart of python/v1/data_collection/collection_config.py.  Keep
// this type local to the example: it is a user-facing recording policy, not a
// second SDK/protocol contract.  The collector maps each enabled name to the
// corresponding generated-proto stream on Robot.

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

struct CollectionConfig {
  std::vector<std::string> slave_joint_names;
  std::vector<std::string> slave_action_names;

  bool enable_head_rgb_stream{false};
  bool enable_head_depth_stream{false};
  bool enable_left_arm_rgb_stream{false};
  bool enable_right_arm_rgb_stream{false};
  bool enable_head_left_eye_stream{false};
  bool enable_head_right_eye_stream{false};
  bool enable_left_arm_elbow_stream{false};
  bool enable_right_arm_elbow_stream{false};

  bool enable_left_arm_end_pose{false};
  bool enable_right_arm_end_pose{false};
  bool enable_wrench_ext_world{false};
  bool enable_wrench_ext_local{false};
  bool enable_waist_end_pose{false};
  bool enable_odometry{false};
  bool enable_pose{false};
  bool enable_chassis_imu{false};
  bool enable_depth_points{false};
  bool enable_head_depth_video{false};
  bool enable_laser_scan{false};
  bool enable_left_gripper_position{false};
  bool enable_right_gripper_position{false};
  bool enable_left_gripper_tactile{false};
  bool enable_right_gripper_tactile{false};
  bool enable_left_hand_tactile{false};
  bool enable_right_hand_tactile{false};
  bool enable_tof_sensors{false};
  bool enable_ultrasonic_sensors{false};
  bool enable_master_arm_data{false};

  int target_hz{30};
  int image_quality{95};
  bool downsample_joint_states{true};
  bool use_video_storage{true};
  bool keep_raw_data{false};

  std::vector<std::string> EnabledSensors() const {
    std::vector<std::string> result;
    const auto add = [&result](bool enabled, const char* name) {
      if (enabled) result.emplace_back(name);
    };
    add(enable_head_rgb_stream, "head_rgb_stream");
    add(enable_head_depth_stream, "head_depth_stream");
    add(enable_left_arm_rgb_stream, "left_arm_rgb_stream");
    add(enable_right_arm_rgb_stream, "right_arm_rgb_stream");
    add(enable_head_left_eye_stream, "head_left_eye_stream");
    add(enable_head_right_eye_stream, "head_right_eye_stream");
    add(enable_left_arm_elbow_stream, "left_arm_elbow_stream");
    add(enable_right_arm_elbow_stream, "right_arm_elbow_stream");
    add(enable_left_arm_end_pose, "left_arm_end_pose");
    add(enable_right_arm_end_pose, "right_arm_end_pose");
    add(enable_wrench_ext_world, "wrench_ext_world");
    add(enable_wrench_ext_local, "wrench_ext_local");
    add(enable_waist_end_pose, "waist_end_pose");
    add(enable_odometry, "odometry");
    add(enable_pose, "pose");
    add(enable_chassis_imu, "chassis_imu");
    add(enable_depth_points, "depth_points");
    add(enable_head_depth_video, "head_depth_video");
    add(enable_laser_scan, "laser_scan");
    add(enable_left_gripper_position, "left_gripper_position");
    add(enable_right_gripper_position, "right_gripper_position");
    add(enable_left_gripper_tactile, "left_gripper_tactile");
    add(enable_right_gripper_tactile, "right_gripper_tactile");
    add(enable_left_hand_tactile, "left_hand_tactile");
    add(enable_right_hand_tactile, "right_hand_tactile");
    add(enable_tof_sensors, "tof_sensors");
    add(enable_ultrasonic_sensors, "ultrasonic_sensors");
    add(enable_master_arm_data, "master_arm_data");
    return result;
  }

  std::vector<std::string> CameraNames() const {
    std::vector<std::string> result;
    if (enable_head_rgb_stream) result.emplace_back("head_rgb_stream");
    if (enable_head_depth_stream || enable_head_depth_video) result.emplace_back("head_depth_stream");
    if (enable_left_arm_rgb_stream) result.emplace_back("left_arm_rgb_stream");
    if (enable_right_arm_rgb_stream) result.emplace_back("right_arm_rgb_stream");
    if (enable_head_left_eye_stream) result.emplace_back("head_left_eye_stream");
    if (enable_head_right_eye_stream) result.emplace_back("head_right_eye_stream");
    if (enable_left_arm_elbow_stream) result.emplace_back("left_arm_elbow_stream");
    if (enable_right_arm_elbow_stream) result.emplace_back("right_arm_elbow_stream");
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
  }

  // 机型键归一化：既能吃客户端解析出的档案名（Robot::ModelName()，来自设备
  // 自报型号字符串，例如 desktop / quanta_x2），也兼容命令行里的历史写法
  // x1 / x2 / desktop。认不出来返回空串，调用方必须显式要求用户传机型，
  // 不能再默默按 x2 采集（公开产物里不出现内部机型名）。
  static std::string NormalizeModelKey(const std::string& value) {
    std::string key;
    key.reserve(value.size());
    for (const char raw : value) {
      const auto c = static_cast<unsigned char>(raw);
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
      key.push_back(static_cast<char>(std::tolower(c)));
    }
    if (key == "x1" || key == "quanta_x1") return "x1";
    if (key == "x2" || key == "quanta_x2") return "x2";
    if (key == "x3" || key == "quanta_x3") return "x3";
    if (key == "desktop") return "desktop";
    return {};
  }

  static CollectionConfig ForModel(const std::string& model_key, bool binocular = false) {
    if (model_key == "x1") return ForQuantaX1(binocular);
    if (model_key == "x3") return ForQuantaX3();
    if (model_key == "desktop") return ForDesktop();
    return ForQuantaX2();
  }

  static CollectionConfig ForQuantaX1(bool binocular = false) {
    CollectionConfig c;
    c.slave_joint_names = {"left_arm_joint_states", "right_arm_joint_states",
                           "lift_joint_states", "left_gripper_joint_states",
                           "right_gripper_joint_states", "head_joint_states"};
    if (binocular) {
      c.enable_head_left_eye_stream = true;
      c.enable_head_right_eye_stream = true;
      c.enable_left_arm_elbow_stream = true;
      c.enable_right_arm_elbow_stream = true;
    } else {
      c.enable_head_rgb_stream = true;
    }
    c.enable_left_arm_rgb_stream = true;
    c.enable_right_arm_rgb_stream = true;
    c.enable_left_arm_end_pose = true;
    c.enable_right_arm_end_pose = true;
    c.enable_odometry = true;
    c.enable_master_arm_data = true;
    c.enable_wrench_ext_world = true;
    c.enable_wrench_ext_local = true;
    return c;
  }

  static CollectionConfig ForQuantaX2() {
    CollectionConfig c;
    c.slave_joint_names = {"left_arm_joint_states", "right_arm_joint_states",
                           "left_gripper_joint_states", "right_gripper_joint_states",
                           "waist_joint_states", "head_joint_states"};
    c.enable_head_rgb_stream = true;
    c.enable_left_arm_rgb_stream = true;
    c.enable_right_arm_rgb_stream = true;
    c.enable_left_arm_end_pose = true;
    c.enable_right_arm_end_pose = true;
    c.enable_odometry = true;
    c.enable_waist_end_pose = true;
    return c;
  }

  static CollectionConfig ForDesktop() {
    CollectionConfig c;
    c.slave_joint_names = {"left_arm_joint_states", "right_arm_joint_states",
                           "left_gripper_joint_states", "right_gripper_joint_states"};
    c.enable_head_rgb_stream = true;
    c.enable_left_arm_rgb_stream = true;
    c.enable_right_arm_rgb_stream = true;
    c.enable_left_arm_end_pose = true;
    c.enable_right_arm_end_pose = true;
    return c;
  }

  static CollectionConfig ForQuantaX3() {
    CollectionConfig c;
    c.slave_joint_names = {"left_arm_joint_states", "right_arm_joint_states",
                           "left_gripper_joint_states", "right_gripper_joint_states",
                           "waist_joint_states", "head_joint_states"};
    // 关节与末端两种回放都需要保留原始反馈；采集只订阅，不切换控制模式。
    c.enable_left_arm_end_pose = true;
    c.enable_right_arm_end_pose = true;
    c.enable_waist_end_pose = true;
    c.enable_head_left_eye_stream = true;
    c.enable_head_right_eye_stream = true;
    c.enable_left_arm_rgb_stream = true;
    c.enable_right_arm_rgb_stream = true;
    c.enable_odometry = true;
    c.enable_chassis_imu = true;
    c.enable_ultrasonic_sensors = true;
    return c;
  }
};
