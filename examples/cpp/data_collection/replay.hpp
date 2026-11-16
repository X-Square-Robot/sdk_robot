#pragma once

// Replay support for the append-only episode format emitted by
// data_collector.hpp.  It deliberately parses generated protobuf messages and
// sends them through the public v2 Robot facade; there is no legacy service or
// transport call hidden in this example.
//
// Diagnostics contract (added for SX001-125): every replayed stream reports
// ``records`` (frames in the episode), ``applied`` (commands actually accepted
// by the robot), ``rejected`` (parse/validation/RPC failures) and ``ignored``
// (records the selected mode does not consume).  A replay that "looks like it
// lost motion" is therefore provable from one run: either the episode has few
// records for that stream, or the commands were rejected/slower than the
// recorded timing.  ``--dry-run`` prints the same plan without a robot.
// Quanta X3 end-pose replay resamples three pose streams together and reports
// pose cycles, auxiliary command count, elapsed time and maximum cycle gap.

#include "data_collector.hpp"
#include "pose_timeline.hpp"
#include "joint_limits.hpp"
#include "sdk_mode_cleanup.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <unordered_map>

namespace sdk_examples {

struct ReplayRecord {
  uint64_t timestamp_ns{0};
  std::string stream;
  std::string payload;
};

struct ReplayEpisode {
  std::string episode_id;
  std::string task;
  double duration{0.0};
  std::vector<ReplayRecord> records;
  std::string gripper_load_error;
};

// Scan applicable joint records before any mode/target writes. Clamp only the
// outgoing commands; original payloads, pose streams and recordings stay intact.
inline void ValidateReplayJointCommands(const ReplayEpisode& episode, const ReplayJointLimits& limits, bool joint_mode, bool report = true)
{
    struct Summary { size_t records{0}, clamped_records{0}; std::vector<size_t> joints_clamped; };
    std::map<std::string, Summary> summaries;
    for (const auto& record : episode.records)
    {
        if (!ReplayJointStream(record.stream, joint_mode)) continue;
        sensor_msgs::JointState state;
        if (!state.ParseFromString(record.payload)) throw std::runtime_error(record.stream + ": invalid joint state payload");
        const std::vector<double> values(state.position().begin(), state.position().end());
        const auto targets = limits.Clamp(record.stream, values);
        auto& row = summaries[record.stream];
        ++row.records;
        row.joints_clamped.resize(values.size());
        bool changed = false;
        for (size_t i = 0; i < values.size(); ++i)
            if (targets[i] != values[i]) { ++row.joints_clamped[i]; changed = true; }
        row.clamped_records += changed;
    }
    if (!report) return;
    for (const auto& [stream, row] : summaries)
    {
        std::cout << "Joint clamp plan: model=" << limits.Model() << " stream=" << stream
                  << " records=" << row.records << " clamped_records=" << row.clamped_records << " joints_clamped=[";
        for (size_t i = 0; i < row.joints_clamped.size(); ++i) std::cout << (i ? "," : "") << row.joints_clamped[i];
        std::cout << "]\n";
    }
}

inline bool IsGripperReplayStream(std::string_view stream)
{
    return stream == "left_gripper_joint_states" || stream == "right_gripper_joint_states" ||
           stream == "left_gripper_position" || stream == "right_gripper_position";
}

// Quanta X3 G 夹爪设计范围为 [0, 1.526] rad；向内取 float32 上界，兼容旧服务端的严格比较。
constexpr double kX3ReplayGripperUpper = 1.525999903678894;

inline float X3ReplayGripperTarget(double value)
{
    return static_cast<float>(std::clamp(value, 0.0, kX3ReplayGripperUpper));
}

inline bool ReadReplayGripper(const ReplayRecord& record, double* value, std::string* error)
{
    if (record.stream.find("joint_states") != std::string::npos)
    {
        sensor_msgs::JointState state;
        if (!state.ParseFromString(record.payload) || state.position_size() == 0)
        {
            *error = "invalid or empty gripper joint state";
            return false;
        }
        *value = state.position(0);
    }
    else
    {
        ::xr::sdk::v2::GripperPosition position;
        if (!position.ParseFromString(record.payload))
        {
            *error = "invalid GripperPosition payload";
            return false;
        }
        *value = position.position();
    }
    if (!std::isfinite(*value))
    {
        *error = "non-finite gripper position";
        return false;
    }
    return true;
}

inline bool ValidateX3GripperReplay(const ReplayEpisode& episode)
{
    if (!episode.gripper_load_error.empty())
    {
        std::cerr << "Replay: " << episode.gripper_load_error << '\n';
        return false;
    }
    struct Summary
    {
        size_t clamped{0};
        double minimum{std::numeric_limits<double>::infinity()};
        double maximum{-std::numeric_limits<double>::infinity()};
        std::string first;
    };
    std::map<std::string, Summary> summaries;
    for (size_t index = 0; index < episode.records.size(); ++index)
    {
        const auto& record = episode.records[index];
        if (!IsGripperReplayStream(record.stream)) continue;
        const std::string location = record.stream + " record=" + std::to_string(index + 1) +
                                     " timestamp_ns=" + std::to_string(record.timestamp_ns);
        double value = 0.0;
        std::string error;
        if (!ReadReplayGripper(record, &value, &error))
        {
            std::cerr << "Replay: " << location << ": " << error << '\n';
            return false;
        }
        auto& summary = summaries[record.stream.substr(0, record.stream.find('_'))];
        summary.minimum = std::min(summary.minimum, value);
        summary.maximum = std::max(summary.maximum, value);
        if (value < 0.0 || value > kX3ReplayGripperUpper)
        {
            if (summary.clamped++ == 0)
            {
                std::ostringstream first;
                first << location << std::setprecision(17) << " value=" << value
                      << " target=" << X3ReplayGripperTarget(value);
                summary.first = first.str();
            }
        }
    }
    for (const auto& [side, summary] : summaries)
    {
        std::ostringstream message;
        message << std::setprecision(17) << "Gripper replay: side=" << side << " range=[0, 1.526] rad"
                << " clamped=" << summary.clamped << " recorded_min=" << summary.minimum
                << " recorded_max=" << summary.maximum;
        if (!summary.first.empty()) message << " first=" << summary.first;
        std::cout << message.str() << '\n';
    }
    return true;
}

inline bool ReadU32(std::istream& in, uint32_t& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(value));
  return static_cast<bool>(in);
}

inline bool ReadU64(std::istream& in, uint64_t& value) {
  in.read(reinterpret_cast<char*>(&value), sizeof(value));
  return static_cast<bool>(in);
}

inline std::string ManifestString(const std::string& text, const std::string& key) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (std::regex_search(text, match, expression) && match.size() > 1) return match[1].str();
  return {};
}

inline double ManifestNumber(const std::string& text, const std::string& key) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*([-+0-9.eE]+)");
  std::smatch match;
  if (std::regex_search(text, match, expression) && match.size() > 1) {
    try { return std::stod(match[1].str()); } catch (...) { return 0.0; }
  }
  return 0.0;
}

// Per-stream accounting shared by the live replay and ``--dry-run``.
struct ReplayStreamStats {
  uint64_t records{0};
  uint64_t planned{0};  // frames the selected mode would consume
  uint64_t applied{0};
  uint64_t rejected{0};
  uint64_t ignored{0};
  uint64_t first_ns{0};
  uint64_t last_ns{0};
  std::string first_error;
};

struct ReplayPlan {
  std::map<std::string, ReplayStreamStats> streams;
  uint64_t records{0};
  uint64_t applicable{0};
  uint64_t ignored{0};
  uint64_t first_ns{0};
  uint64_t last_ns{0};
};

enum class ApplyOutcome { kApplied, kIgnored, kFailed };

inline bool ReplayMode(std::string_view mode, bool* joint_mode) {
  const bool joint = mode == "joint_pos" || mode == "joint";
  const bool pose = mode == "end_pose" || mode == "pose";
  if (!joint && !pose) return false;
  *joint_mode = joint;
  return true;
}

inline bool ValidateX3EndPoseReplay(const ReplayEpisode& episode) {
  bool complete = true;
  for (const auto* name : {"left_arm_end_pose", "right_arm_end_pose", "waist_end_pose"}) {
    if (std::none_of(episode.records.begin(), episode.records.end(),
                    [name](const ReplayRecord& record) { return record.stream == name && !record.payload.empty(); })) {
      std::cerr << "Replay: missing recorded pose stream: " << name
                << "; Quanta X3 end_pose replay requires both arms and waist\n";
      complete = false;
    }
  }
  return complete;
}

// A record is consumed by the selected mode.  The arm command space is either
// joint positions or an end pose, but gripper/lift/head are independent single
// DoF end effectors: they are commanded in both modes.  Skipping them in
// end_pose mode made end_pose replay obviously "lose actions" (the gripper
// never moved) even though the recording contained the frames.
inline bool ReplayRecordApplies(std::string_view stream, bool joint_mode, bool replay_chassis = true) {
  if (stream == "odometry") return replay_chassis;
  if (stream == "left_arm_joint_states" || stream == "right_arm_joint_states") {
    return joint_mode;
  }
  if (stream.find("gripper_joint_states") != std::string_view::npos) return true;
  if (stream == "lift_joint_states") return true;
  if (stream == "waist_joint_states") return joint_mode;
  if (stream == "head_joint_states") return true;
  if (stream.find("end_pose") != std::string_view::npos) return !joint_mode;
  if (stream == "left_gripper_position" || stream == "right_gripper_position") return true;
  return false;
}

inline std::optional<ReplayEpisode> LoadEpisode(const fs::path& episode_dir) {
  std::ifstream manifest(episode_dir / "episode.json");
  if (!manifest) {
    std::cerr << "Replay: missing " << (episode_dir / "episode.json") << '\n';
    return std::nullopt;
  }
  const std::string manifest_text((std::istreambuf_iterator<char>(manifest)), {});
  ReplayEpisode episode;
  episode.episode_id = ManifestString(manifest_text, "episode_id");
  episode.task = ManifestString(manifest_text, "task");
  episode.duration = ManifestNumber(manifest_text, "duration");

  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(episode_dir, ec)) {
    if (ec || !entry.is_regular_file()) continue;
    const auto filename = entry.path().filename().string();
    if (filename.rfind("stream_", 0) != 0 || entry.path().extension() != ".pb") continue;
    const std::string stream = filename.substr(7, filename.size() - 10);
    std::ifstream in(entry.path(), std::ios::binary);
    const auto load_error = [&](std::string detail) {
      if (IsGripperReplayStream(stream)) episode.gripper_load_error = stream + ": " + detail;
    };
    if (!in) {
      load_error("cannot read stream");
      continue;
    }
    while (true) {
      uint64_t timestamp = 0;
      uint32_t size = 0;
      if (!ReadU64(in, timestamp)) {
        if (in.gcount() != 0) load_error("truncated record timestamp");
        break;
      }
      if (!ReadU32(in, size)) {
        load_error("truncated record size at timestamp_ns=" + std::to_string(timestamp));
        break;
      }
      if (size > 256U * 1024U * 1024U) {
        std::cerr << "Replay: refusing oversized record in " << entry.path() << '\n';
        return std::nullopt;
      }
      std::string payload(size, '\0');
      in.read(payload.data(), static_cast<std::streamsize>(size));
      if (!in) {
        load_error("truncated payload at timestamp_ns=" + std::to_string(timestamp));
        break;  // 其他机型仍沿用忽略未写完尾帧的行为。
      }
      episode.records.push_back({timestamp, stream, std::move(payload)});
    }
  }
  std::sort(episode.records.begin(), episode.records.end(),
            [](const ReplayRecord& a, const ReplayRecord& b) {
              return a.timestamp_ns < b.timestamp_ns;
            });
  if (episode.records.empty()) {
    std::cerr << "Replay: episode has no complete protobuf records\n";
    return std::nullopt;
  }
  return episode;
}

// Dry-run analysis: what a replay would consume, per stream, without a robot.
inline ReplayPlan AnalyzeEpisode(const ReplayEpisode& episode, bool joint_mode, bool replay_chassis = true) {
  ReplayPlan plan;
  for (const auto& record : episode.records) {
    auto& stream = plan.streams[record.stream];
    ++stream.records;
    ++plan.records;
    if (stream.first_ns == 0 || record.timestamp_ns < stream.first_ns) stream.first_ns = record.timestamp_ns;
    if (record.timestamp_ns > stream.last_ns) stream.last_ns = record.timestamp_ns;
    if (plan.first_ns == 0 || record.timestamp_ns < plan.first_ns) plan.first_ns = record.timestamp_ns;
    if (record.timestamp_ns > plan.last_ns) plan.last_ns = record.timestamp_ns;
    if (ReplayRecordApplies(record.stream, joint_mode, replay_chassis)) {
      ++plan.applicable;
      ++stream.planned;
    } else {
      ++plan.ignored;
      ++stream.ignored;
    }
  }
  return plan;
}

inline double StreamHz(const ReplayStreamStats& stream) {
  if (stream.records < 2 || stream.last_ns <= stream.first_ns) return 0.0;
  return static_cast<double>(stream.records - 1) * 1e9 /
         static_cast<double>(stream.last_ns - stream.first_ns);
}

inline void PrintReplayPlan(const ReplayPlan& plan, bool joint_mode, double speed,
                            double max_lag_ms) {
  const char* mode = joint_mode ? "joint_pos" : "end_pose";
  const double recorded = plan.first_ns == 0 || plan.last_ns <= plan.first_ns
                              ? 0.0
                              : static_cast<double>(plan.last_ns - plan.first_ns) / 1e9;
  std::cout << "Replay plan: mode=" << mode << " speed=" << speed << "x"
            << " records=" << plan.records << " applicable=" << plan.applicable
            << " ignored=" << plan.ignored << " recorded_duration=" << recorded << "s"
            << " max_lag=" << (max_lag_ms > 0 ? std::to_string(max_lag_ms) + "ms" : "none")
            << "\n";
  std::cout << "  " << std::left << std::setw(28) << "stream" << std::right << std::setw(9)
            << "records" << std::setw(9) << "hz" << std::setw(9) << "planned"
            << std::setw(9) << "applied" << std::setw(9) << "rejected"
            << std::setw(9) << "ignored" << "\n";
  for (const auto& [name, stream] : plan.streams) {
    std::cout << "  " << std::left << std::setw(28) << name << std::right << std::setw(9)
              << stream.records << std::setw(9) << std::fixed << std::setprecision(1)
              << StreamHz(stream) << std::setprecision(6) << std::setw(9) << stream.planned
              << std::setw(9) << stream.applied << std::setw(9) << stream.rejected
              << std::setw(9) << stream.ignored << "\n";
  }
}

inline double QuaternionYaw(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

struct ReplayBehavior {
  bool velocity_chassis_tracking{false};
  bool warn_and_continue{false};
  bool recorded_parts_only{false};
};

class EpisodeReplayer {
 public:
  explicit EpisodeReplayer(x2robot::sdk::Robot& robot, ReplayBehavior options = {}) : robot_(robot), options_(options) {}

  bool Replay(const ReplayEpisode& episode, std::string mode, double speed = 1.0,
              std::atomic<bool>* stop = nullptr, double max_lag_ms = 0.0, bool replay_chassis = true) {
    if (!std::isfinite(speed) || speed <= 0.0) {
      std::cerr << "Replay: speed must be > 0\n";
      return false;
    }
    bool joint_mode = false;
    if (!ReplayMode(mode, &joint_mode)) {
      std::cerr << "Replay: mode must be joint_pos or end_pose\n";
      return false;
    }
    try {
      joint_limits_.emplace(robot_.ModelName());
      ValidateReplayJointCommands(episode, *joint_limits_, joint_mode);
    } catch (const std::exception& error) {
      std::cerr << "Replay: " << error.what() << '\n';
      return false;
    }
    const bool is_x3 = robot_.ModelName() == "quanta_x3";
    // Legacy X3 episodes synchronize three pose streams; v2 accepts recorded subsets.
    const bool synchronized_x3_poses = is_x3 && !options_.recorded_parts_only && !joint_mode;
    // 整段检查必须先于切模式和任何控制命令，原始录制数据保持不变。
    if (is_x3 && !ValidateX3GripperReplay(episode)) return false;
    if (synchronized_x3_poses && !ValidateX3EndPoseReplay(episode)) return false;
    std::array<ReplayPoseTimeline, 3> timelines;
    uint64_t first_ns = 0, last_ns = 0;
    bool have_first = false;
    if (synchronized_x3_poses) {
      for (const auto& record : episode.records) {
        if (!ReplayRecordApplies(record.stream, false, replay_chassis)) continue;
        if (!have_first) {
          first_ns = record.timestamp_ns;
          have_first = true;
        }
        last_ns = record.timestamp_ns;
        const int index = ReplayPoseIndex(record.stream);
        if (index < 0) continue;
        geometry_msgs::PoseStamped pose;
        if (!pose.ParseFromString(record.payload) ||
            !timelines[index].Add(static_cast<double>(record.timestamp_ns - first_ns) / 1e9, pose.pose())) {
          std::cerr << "Replay: " << record.stream << ": invalid recorded pose\n";
          return false;
        }
      }
    }

    // Match Python v1's setup sequence.  SDK work mode is required for write
    // operations, while all reads in the collector remain mode independent.
    // NoteWorkMode 记录这次占用，退出时由调用方 RestoreIdleModeOnExit 交还
    // IDLE（/application/task_info.type 回到 0）。
    ::xr::sdk::v2::RobotModeParam work_mode;
    work_mode.set_mode(::xr::sdk::v2::SDK);
    auto work_result = x2robot::examples::NoteWorkMode(
        work_mode, robot_.GetSystem().SetWorkMode(work_mode, x2robot::sdk::Deadline{1000}));
    if (!work_result.Ok()) {
      std::cerr << "Replay: SetWorkMode failed: " << work_result.ErrorValue().error_message << '\n';
      return false;
    }
    const bool has_manipulator = std::any_of(episode.records.begin(), episode.records.end(),
        [&](const ReplayRecord& record) {
          return ReplayRecordApplies(record.stream, joint_mode) &&
              (record.stream == "left_arm_joint_states" || record.stream == "right_arm_joint_states" ||
               record.stream == "waist_joint_states" || record.stream == "left_arm_end_pose" ||
               record.stream == "right_arm_end_pose" || record.stream == "waist_end_pose");
        });
    if (!options_.recorded_parts_only || has_manipulator) {
      ::xr::sdk::v2::ManipulatorControlModeParam control_mode;
      control_mode.set_mode(joint_mode ? ::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS
                                       : ::xr::sdk::v2::MANIPULATOR_END_POSE);
      auto mode_result = robot_.GetRobotControl().SetManipulatorControlMode(control_mode,
                                                                             x2robot::sdk::Deadline{1000});
      if (!mode_result.Ok()) {
        std::cerr << "Replay: SetManipulatorControlMode failed: "
                  << mode_result.ErrorValue().error_message << '\n';
        return false;
      }
    }

    const bool velocity_tracking = is_x3 || options_.velocity_chassis_tracking;
    const bool has_odometry = replay_chassis && std::any_of(episode.records.begin(), episode.records.end(),
        [](const ReplayRecord& record) { return record.stream == "odometry"; });
    if (has_odometry && velocity_tracking) {
      ::xr::sdk::v2::ChassisControlModeParam chassis_mode;
      chassis_mode.set_mode(::xr::sdk::v2::VELOCITY);
      auto result = robot_.GetChassisController().SetControlMode(
          chassis_mode, x2robot::sdk::Deadline{1000});
      if (!result.Ok()) {
        std::cerr << "Replay: SetControlMode failed: " << result.ErrorValue().error_message << '\n';
        return false;
      }
    }
    const auto stop_chassis = [&] {
      if (!replay_chassis) return true;
      if (velocity_tracking && !has_odometry) return true;
      ::xr::sdk::v2::ChassisVelocity velocity;
      return robot_.GetChassisController().SetVelocity(velocity, x2robot::sdk::Deadline{500}).Ok();
    };
    if (synchronized_x3_poses) {
      const bool ok = ReplayX3Poses(episode, timelines, first_ns, last_ns, speed, stop, replay_chassis);
      const bool stopped = stop_chassis();
      return ok && stopped;
    }
    bool command_failed = false;

    ReplayPlan plan = AnalyzeEpisode(episode, joint_mode, replay_chassis);
    PrintReplayPlan(plan, joint_mode, speed, max_lag_ms);

    const double max_lag_ns = max_lag_ms > 0.0 ? max_lag_ms * 1e6 : 0.0;
    uint64_t first_applicable_ns = 0;
    uint64_t applied_total = 0;
    uint64_t late_records = 0;
    uint64_t dropped = 0;
    double max_lag_seen_ms = 0.0;
    const auto wall_start = std::chrono::steady_clock::now();
    for (const auto& record : episode.records) {
      if (stop && stop->load(std::memory_order_acquire)) break;
      if (!ReplayRecordApplies(record.stream, joint_mode, replay_chassis)) continue;
      if (first_applicable_ns == 0) first_applicable_ns = record.timestamp_ns;

      if (record.timestamp_ns > first_applicable_ns) {
        const double target_seconds =
            static_cast<double>(record.timestamp_ns - first_applicable_ns) / 1e9 / speed;
        const auto target = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                              std::chrono::duration<double>(target_seconds));
        const auto now = std::chrono::steady_clock::now();
        if (now < target) {
          std::this_thread::sleep_until(target);
        } else {
          // Behind schedule: the blocking command RPC is slower than the
          // recorded cadence.  Count it so a jerky replay is measurable, and
          // optionally drop stale frames instead of stampeding the controller.
          const double lag_ms =
              std::chrono::duration<double, std::milli>(now - target).count();
          ++late_records;
          max_lag_seen_ms = std::max(max_lag_seen_ms, lag_ms);
          if (max_lag_ns > 0.0 && lag_ms * 1e6 > max_lag_ns) {
            ++dropped;
            ++plan.streams[record.stream].rejected;
            continue;
          }
        }
      }

      std::string error;
      const ApplyOutcome outcome = Apply(record, joint_mode, &error);
      auto& stream = plan.streams[record.stream];
      if (outcome == ApplyOutcome::kApplied) {
        ++stream.applied;
        ++applied_total;
      } else if (outcome == ApplyOutcome::kFailed) {
        ++stream.rejected;
        if (stream.first_error.empty()) {
          stream.first_error = error;
          if (options_.warn_and_continue)
            std::cerr << "WARN: " << record.stream << " command rejected: " << error
                      << "; continuing replay (further failures counted in final summary)\n";
        }
        // Legacy X3 keeps fail-fast; v2 explicitly opts into warning and continuation.
        if (is_x3 && !options_.warn_and_continue) {
          command_failed = true;
          break;
        }
      } else {
        ++stream.ignored;
      }
    }
    const bool chassis_stopped = stop_chassis();
    if (!chassis_stopped && options_.warn_and_continue)
      std::cerr << "WARN: chassis stop command rejected\n";
    const auto wall_end = std::chrono::steady_clock::now();
    const double wall_seconds =
        std::chrono::duration<double>(wall_end - wall_start).count();
    const double recorded_seconds =
        plan.first_ns == 0 || plan.last_ns <= plan.first_ns
            ? 0.0
            : static_cast<double>(plan.last_ns - plan.first_ns) / 1e9;
    PrintReplayPlan(plan, joint_mode, speed, max_lag_ms);
    std::cout << "Replayed " << applied_total << " commands from " << episode.episode_id
              << " (wall=" << std::fixed << std::setprecision(2) << wall_seconds
              << "s, recorded=" << recorded_seconds << "s"
              << (recorded_seconds > 0 ? ", ratio=" + std::to_string(wall_seconds / recorded_seconds) + "x" : "")
              << ", late_records=" << late_records << ", max_lag=" << std::setprecision(1)
              << max_lag_seen_ms << "ms, dropped=" << dropped << ")\n";
    if (late_records > 0 && dropped == 0) {
      std::cout << "  note: replay stayed behind the recorded cadence; the robot receives commands "
                   "late but none were dropped (use --max-lag-ms to convert lag into explicit drops)\n";
    }
    for (const auto& [name, stream] : plan.streams) {
      if (!stream.first_error.empty()) {
        std::cerr << "Replay: " << name << " first rejection: " << stream.first_error << '\n';
      }
    }
    return !command_failed && (!is_x3 || chassis_stopped || options_.warn_and_continue) && applied_total > 0;
  }

 private:
  bool ReplayX3Poses(const ReplayEpisode& episode, const std::array<ReplayPoseTimeline, 3>& timelines,
                    uint64_t first_ns, uint64_t last_ns, double speed, std::atomic<bool>* stop, bool replay_chassis) {
    constexpr double period = 0.02;
    const std::array<std::string, 3> names{"left_arm_end_pose", "right_arm_end_pose", "waist_end_pose"};
    const double duration = static_cast<double>(last_ns - first_ns) / 1e9;
    const auto started = std::chrono::steady_clock::now();
    auto previous_cycle = started;
    size_t index = 0, cycles = 0, auxiliary = 0, pose_timeouts = 0;
    double played = 0, max_refresh = 0;
    const auto seconds = [](auto a, auto b) { return std::chrono::duration<double>(a - b).count(); };
    const auto send = [&](const ReplayRecord& record) {
      if (stop && stop->load(std::memory_order_acquire)) return false;
      std::string error;
      int32_t pose_error_code = 0;
      if (Apply(record, false, &error, x2robot::sdk::Deadline{300}, &pose_error_code) == ApplyOutcome::kApplied) return true;
      if (pose_error_code == x2robot::sdk::errc::kTimeout) {
        ++pose_timeouts;
        std::cerr << "WARNING: " << record.stream
                  << " timed out; execution unconfirmed; continuing without retry: " << error << '\n';
        return true;
      }
      std::cerr << "Replay: " << record.stream << " first rejection: " << error << '\n';
      return false;
    };
    std::cout << "Synchronized Quanta X3 end_pose replay: 50 Hz, left/right/waist every cycle\n";
    while (true) {
      if (stop && stop->load(std::memory_order_acquire)) return false;
      const auto cycle = std::chrono::steady_clock::now();
      if (cycles) {
        max_refresh = std::max(max_refresh, seconds(cycle, previous_cycle));
      }
      previous_cycle = cycle;
      const uint64_t stamp = played >= duration ? last_ns : first_ns + static_cast<uint64_t>(played * 1e9);
      for (size_t part = 0; part < 3; ++part) {
        geometry_msgs::PoseStamped pose;
        *pose.mutable_pose() = timelines[part].At(played);
        if (!send({stamp, names[part], pose.SerializeAsString()})) return false;
      }
      ++cycles;
      while (index < episode.records.size() && episode.records[index].timestamp_ns <= stamp) {
        const auto& record = episode.records[index++];
        if (ReplayPoseIndex(record.stream) >= 0 || !ReplayRecordApplies(record.stream, false, replay_chassis)) continue;
        if (!send(record)) return false;
        ++auxiliary;
      }
      if (played >= duration) break;
      // Advance one recorded-time step only. Slow successful RPCs stretch
      // replay instead of skipping targets or sending a catch-up burst.
      // RPC time counts toward this period; wait only for the remainder.
      const double next_played = std::min(cycles * period * speed, duration);
      const double remaining_step = (next_played - played) / speed;
      std::this_thread::sleep_until(cycle + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(remaining_step)));
      played = next_played;
    }
    std::cout << "Synchronized replay complete: pose_cycles=" << cycles << " auxiliary_commands=" << auxiliary
              << " wall=" << seconds(std::chrono::steady_clock::now(), started) << "s recorded=" << duration
              << "s max_refresh=" << max_refresh * 1000 << "ms pose_timeouts=" << pose_timeouts << '\n';
    return true;
  }

  ApplyOutcome Apply(const ReplayRecord& record, bool joint_mode, std::string* error,
                     x2robot::sdk::Deadline deadline = x2robot::sdk::Deadline{1000}, int32_t* pose_error_code = nullptr) {
    const auto fail = [error](std::string message) {
      if (error) *error = std::move(message);
      return ApplyOutcome::kFailed;
    };
    if (robot_.ModelName() == "quanta_x3" && IsGripperReplayStream(record.stream)) {
      double value = 0.0;
      std::string detail;
      if (!ReadReplayGripper(record, &value, &detail)) return fail(detail);
      ::xr::sdk::v2::GripperPosition command;
      command.set_position(X3ReplayGripperTarget(value));
      auto result = record.stream.rfind("left_", 0) == 0
          ? robot_.GetLeftGripperController().SetPosition(command, deadline)
          : robot_.GetRightGripperController().SetPosition(command, deadline);
      if (!result.Ok()) return fail(result.ErrorValue().error_message);
      return ApplyOutcome::kApplied;
    }
    if (record.stream == "odometry") {
      nav_msgs::Odometry odom;
      if (!odom.ParseFromString(record.payload)) return fail("cannot parse nav_msgs/Odometry payload");
      if (robot_.ModelName() == "quanta_x3" || options_.velocity_chassis_tracking) {
        // X3 不公开全局位置写接口，按 Python 示例用里程计反馈计算限幅速度。
        const auto feedback = robot_.GetChassisController().GetOdometry(deadline);
        if (!feedback.Ok()) return fail(feedback.ErrorValue().error_message);
        const auto& current = feedback.Value().pose().pose();
        const auto& desired = odom.pose().pose();
        const double yaw = QuaternionYaw(current.orientation());
        const double dx = desired.position().x() - current.position().x();
        const double dy = desired.position().y() - current.position().y();
        const double dyaw = std::remainder(QuaternionYaw(desired.orientation()) - yaw, 2.0 * std::acos(-1.0));
        if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dyaw)) {
          return fail("non-finite chassis replay error");
        }
        ::xr::sdk::v2::ChassisVelocity velocity;
        if (std::hypot(dx, dy) >= 0.01 || std::abs(dyaw) >= 0.05) {
          velocity.set_vel_x(std::clamp(2.0 * (dx * std::cos(yaw) + dy * std::sin(yaw)), -0.3, 0.3));
          // Quanta X3 MCU: vx +/-0.45 m/s, vy +/-0.20 m/s, wz +/-0.40 rad/s.
          // Keep replay's existing lower forward cap of 0.30 m/s.
          velocity.set_vel_y(std::clamp(2.0 * (-dx * std::sin(yaw) + dy * std::cos(yaw)), -0.20, 0.20));
          velocity.set_vel_yaw(std::clamp(2.0 * dyaw, -0.40, 0.40));
        }
        auto result = robot_.GetChassisController().SetVelocity(velocity, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      ::xr::sdk::v2::ChassisPosition target;
      target.set_x(odom.pose().pose().position().x());
      target.set_y(odom.pose().pose().position().y());
      target.set_yaw(QuaternionYaw(odom.pose().pose().orientation()));
      auto result = robot_.GetChassisController().MoveToGlobalPosition(target, deadline);
      if (!result.Ok()) return fail(result.ErrorValue().error_message);
      return ApplyOutcome::kApplied;
    }

    if (record.stream.find("joint_states") != std::string::npos ||
        record.stream.find("joint_state") != std::string::npos) {
      sensor_msgs::JointState state;
      if (!state.ParseFromString(record.payload)) return fail("cannot parse sensor_msgs/JointState payload");
      if (ReplayJointStream(record.stream, joint_mode)) {
        try {
          const auto targets = joint_limits_->Clamp(record.stream, {state.position().begin(), state.position().end()});
          for (size_t i = 0; i < targets.size(); ++i) state.set_position(i, targets[i]);
        } catch (const std::exception& error) { return fail(error.what()); }
      }
      if (record.stream == "left_arm_joint_states") {
        if (!joint_mode) return ApplyOutcome::kIgnored;
        ::xr::sdk::v2::JointPositions command;
        for (double value : state.position()) command.add_positions(value);
        auto result = robot_.GetLeftArmController().SetJointPositions(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      if (record.stream == "right_arm_joint_states") {
        if (!joint_mode) return ApplyOutcome::kIgnored;
        ::xr::sdk::v2::JointPositions command;
        for (double value : state.position()) command.add_positions(value);
        auto result = robot_.GetRightArmController().SetJointPositions(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      // Grippers are single-DoF end effectors: an end-pose arm command does not
      // move them, so they must be replayed in both modes (SX001-125).
      if (record.stream == "left_gripper_joint_states") {
        if (!state.position_size()) return fail("empty gripper joint state");
        ::xr::sdk::v2::GripperPosition command;
        command.set_position(static_cast<float>(state.position(0)));
        auto result = robot_.GetLeftGripperController().SetPosition(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      if (record.stream == "right_gripper_joint_states") {
        if (!state.position_size()) return fail("empty gripper joint state");
        ::xr::sdk::v2::GripperPosition command;
        command.set_position(static_cast<float>(state.position(0)));
        auto result = robot_.GetRightGripperController().SetPosition(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      if (record.stream == "lift_joint_states") {
        if (!state.position_size()) return fail("empty lift joint state");
        ::xr::sdk::v2::LiftPosition command;
        command.set_position(static_cast<float>(state.position(0)));
        auto result = robot_.GetLiftController().SetLiftPosition(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      if (record.stream == "waist_joint_states") {
        if (!joint_mode) return ApplyOutcome::kIgnored;
        ::xr::sdk::v2::JointPositions command;
        for (double value : state.position()) command.add_positions(value);
        auto result = robot_.GetWaistController().SetJointPositions(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      if (record.stream == "head_joint_states") {
        if (!state.position_size()) return fail("empty head joint state");
        ::xr::sdk::v2::HeadPose command;
        command.set_pitch(static_cast<float>(state.position(0)));
        if (state.position_size() > 1) command.set_yaw(static_cast<float>(state.position(1)));
        auto result = robot_.GetHeadController().SetPose(command, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      return ApplyOutcome::kIgnored;
    }

    if (record.stream.find("end_pose") != std::string::npos) {
      geometry_msgs::PoseStamped stamped;
      if (!stamped.ParseFromString(record.payload)) return fail("cannot parse geometry_msgs/PoseStamped payload");
      if (joint_mode) return ApplyOutcome::kIgnored;
      const auto pose_result = [&](const auto& result) {
        if (result.Ok()) return ApplyOutcome::kApplied;
        if (pose_error_code) *pose_error_code = result.ErrorValue().error_code;
        return fail(result.ErrorValue().error_message);
      };
      if (record.stream == "left_arm_end_pose") {
        return pose_result(robot_.GetLeftArmController().SetEndPose(stamped.pose(), deadline));
      }
      if (record.stream == "right_arm_end_pose") {
        return pose_result(robot_.GetRightArmController().SetEndPose(stamped.pose(), deadline));
      }
      if (record.stream == "waist_end_pose") {
        return pose_result(robot_.GetWaistController().SetEndPose(stamped.pose(), deadline));
      }
      return ApplyOutcome::kIgnored;
    }

    if (record.stream == "left_gripper_position" || record.stream == "right_gripper_position") {
      ::xr::sdk::v2::GripperPosition position;
      if (!position.ParseFromString(record.payload)) return fail("cannot parse GripperPosition payload");
      if (record.stream == "left_gripper_position") {
        auto result = robot_.GetLeftGripperController().SetPosition(position, deadline);
        if (!result.Ok()) return fail(result.ErrorValue().error_message);
        return ApplyOutcome::kApplied;
      }
      auto result = robot_.GetRightGripperController().SetPosition(position, deadline);
      if (!result.Ok()) return fail(result.ErrorValue().error_message);
      return ApplyOutcome::kApplied;
    }
    return ApplyOutcome::kIgnored;
  }

  x2robot::sdk::Robot& robot_;
  ReplayBehavior options_;
  std::optional<ReplayJointLimits> joint_limits_;
};

}  // namespace sdk_examples
