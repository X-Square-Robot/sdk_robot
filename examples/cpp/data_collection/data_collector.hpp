#pragma once

// C++ counterpart of the Python v1 DataCollector (video export requires ffmpeg).
//
// The collector intentionally records the generated protobuf payloads instead
// of introducing an example-only copy of every ROS message.  Each stream is
// opened on StartRecording (which is also the subscription boundary on the
// server), has its own reader thread, and is cancelled/joined on stop.  The
// resulting episode is self contained and can be consumed by
// data_replay_example: every `stream_*.pb` file is a sequence of
// [timestamp_ns:u64][payload_size:u32][protobuf bytes]. Cameras are exported
// as H.264 MP4 files with timestamped JSONL sample indexes. Encoded input is
// spooled to disk during capture so encoding never blocks a camera callback.

#include "collection_config.hpp"
#include "x2robot/sdk/robot.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

namespace sdk_examples {

namespace fs = std::filesystem;
using x2robot::sdk::Deadline;
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;

inline uint64_t WallClockNs() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

inline void WriteU32(std::ostream& out, uint32_t value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

inline void WriteU64(std::ostream& out, uint64_t value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}

inline std::string JsonEscape(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size() + 8);
  for (const char c : value) {
    switch (c) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped.push_back(c); break;
    }
  }
  return escaped;
}

struct EpisodeInfo {
  std::string episode_id;
  std::string task;
  fs::path episode_dir;
  uint64_t num_records{0};
  double duration{0.0};
};

// As in Python, finish the MP4 after stopping acquisition. Keep compressed
// camera input on disk rather than encoding on the gRPC reader thread.
//
// Failure diagnostics (SX001-150): the reported "decode failure" on rc4 is
// actually this exporter failing.  The error now carries the tail of ffmpeg's
// own stderr, the retained ``*.mp4.input`` spool and a copy-pasteable ffmpeg
// command, and the episode manifest receives the same text through
// ``SetStreamError``.  The container codec is sniffed from the Annex-B NAL
// headers because a generic "h26x" format hint cannot tell H.264 from H.265.
class CameraVideoWriter {
 public:
  CameraVideoWriter(fs::path output_path, int target_hz)
      : output_path_(std::move(output_path)), raw_path_(output_path_.string() + ".input"),
        log_path_(output_path_.string() + ".ffmpeg.log"),
        target_hz_(std::max(1, target_hz)) {}

  CameraVideoWriter(const CameraVideoWriter&) = delete;
  CameraVideoWriter& operator=(const CameraVideoWriter&) = delete;

  bool Write(const x2robot::sdk::client::CameraFrameView& frame, std::string* error) {
    if (closed_ || failed_) return Fail("video writer is closed or failed", error);
    if (frame.data == nullptr || frame.size == 0) return Fail("camera frame is empty", error);
    const std::string codec = ResolveCodec(frame, error);
    if (codec.empty()) return false;  // ResolveCodec already reported the reason.
    if (!raw_.is_open()) {
      codec_ = codec;
      raw_.open(raw_path_, std::ios::binary | std::ios::trunc);
      if (!raw_) return Fail("cannot open " + raw_path_.string(), error);
    }
    if (codec != codec_) {
      return Fail("camera encoding changed during recording: " + codec_ + " -> " + codec, error);
    }
    raw_.write(reinterpret_cast<const char*>(frame.data), static_cast<std::streamsize>(frame.size));
    if (!raw_) return Fail("camera input write failed", error);
    ++sample_count_;
    return true;
  }

  bool IsPacketStream() const { return codec_ == "h264" || codec_ == "hevc"; }
  bool HasSamples() const { return sample_count_ != 0; }

  bool Close(std::string* error = nullptr) {
    if (closed_) return !failed_;
    closed_ = true;
    if (raw_.is_open()) {
      raw_.close();
      if (!raw_) return Fail("camera input flush failed", error);
    }
    if (failed_) {
      return Fail("camera recording failed; compressed input retained at " + raw_path_.string(),
                  error);
    }
    if (!HasSamples()) return true;
    if (!RunFfmpeg(FfmpegArgs(), error)) {
      if (!ambiguous_packet_hint_ || !IsPacketStream()) return false;
      // The format hint could not name the codec and the sniffed one did not
      // demux: try the other packet codec on the retained spool.  This keeps a
      // mislabelled "h26x" recording exportable instead of losing the episode.
      const std::string first_failure = error != nullptr ? *error : std::string();
      codec_ = codec_ == "h264" ? "hevc" : "h264";
      std::cerr << "DataCollector: export failed with the sniffed codec; retrying as " << codec_
                << " (format hint was generic)\n";
      std::string retry_error;
      if (!RunFfmpeg(FfmpegArgs(), &retry_error)) {
        if (error != nullptr) *error = first_failure + " | retry as " + codec_ + ": " + retry_error;
        return false;
      }
      std::cerr << "DataCollector: camera payload exported as " << codec_
                << " (the format hint did not name the codec)\n";
    }
    std::error_code ec;
    fs::remove(raw_path_, ec);
    fs::remove(log_path_, ec);
    return true;
  }

 private:
  // Still-image formats map directly.  Packet formats map to h264/hevc; an
  // unknown or generic hint (for example "h26x") returns "" so the Annex-B
  // payload decides instead of assuming HEVC.
  static std::string DeclaredCodec(std::string_view format) {
    std::string lower(format);
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (lower.find("h264") != std::string::npos || lower.find("avc") != std::string::npos) return "h264";
    if (lower.find("h265") != std::string::npos || lower.find("hevc") != std::string::npos) return "hevc";
    if (lower.find("png") != std::string::npos) return "png";
    if (lower.find("jpeg") != std::string::npos || lower.find("jpg") != std::string::npos) return "mjpeg";
    return {};
  }

  // Annex-B NAL scan.  HEVC VPS/SPS/PPS are 0x40/0x42/0x44 followed by the
  // two-byte NAL header tail 0x01 (layer 0, temporal_id_plus1 = 1); H.264 SPS
  // is 0x67 with a known profile_idc in the next byte.  Requiring those markers
  // avoids the false "H.264 PPS" reading of HEVC IDR slices (0x28), which is
  // what a naive low-5-bit test produces.
  static bool KnownH264Profile(uint8_t profile) {
    switch (profile) {
      case 44: case 66: case 77: case 88: case 100: case 110: case 118: case 122:
      case 128: case 134: case 135: case 138: case 139: case 244:
        return true;
      default:
        return false;
    }
  }

  static std::string SniffAnnexBCodec(const uint8_t* data, size_t size) {
    if (data == nullptr || size < 5) return {};
    bool hevc = false;
    bool h264_sps = false;
    bool h264_pps = false;
    bool h264_idr = false;
    size_t i = 0;
    while (i + 3 < size) {
      size_t prefix = 0;
      if (i + 4 <= size && data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 0 &&
          data[i + 3] == 1) {
        prefix = 4;
      } else if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
        prefix = 3;
      }
      if (prefix == 0) {
        ++i;
        continue;
      }
      const size_t header = i + prefix;
      if (header >= size) break;
      const uint8_t byte = data[header];
      const uint8_t next = header + 1 < size ? data[header + 1] : 0;
      const unsigned hevc_type = (byte >> 1) & 0x3F;
      if ((hevc_type == 32 || hevc_type == 33 || hevc_type == 34) && next == 0x01) hevc = true;
      if ((byte & 0x80) == 0) {  // H.264 forbidden_zero_bit
        const unsigned h264_type = byte & 0x1F;
        if (h264_type == 7 && KnownH264Profile(next)) h264_sps = true;
        if (h264_type == 8) h264_pps = true;
        if (h264_type == 5) h264_idr = true;
      }
      i = header + 1;
    }
    if (hevc) return "hevc";
    if (h264_sps || (h264_pps && h264_idr)) return "h264";
    return {};
  }

  static std::string HexPreview(const uint8_t* data, size_t size) {
    static const char* kDigits = "0123456789abcdef";
    std::string out;
    const size_t count = std::min<size_t>(size, 12);
    out.reserve(count * 2 + 8);
    for (size_t i = 0; i < count; ++i) {
      out.push_back(kDigits[data[i] >> 4]);
      out.push_back(kDigits[data[i] & 0x0F]);
      if (i + 1 != count) out.push_back(' ');
    }
    if (size > count) out += " ...";
    return out;
  }

  std::string ResolveCodec(const x2robot::sdk::client::CameraFrameView& frame,
                           std::string* error) {
    const std::string declared = DeclaredCodec(frame.format);
    const bool packet_hint = declared == "h264" || declared == "hevc" || declared.empty();
    if (!packet_hint) return declared;
    const std::string sniffed = SniffAnnexBCodec(frame.data, frame.size);
    if (declared.empty()) {
      // "h26x" (or an empty hint) cannot tell the two codecs apart; remember it
      // so a failed export can retry with the other one before giving up.
      ambiguous_packet_hint_ = true;
      if (!sniffed.empty()) return sniffed;
      Fail("cannot identify camera bitstream: format='" + std::string(frame.format) +
               "' first_bytes=" + HexPreview(frame.data, frame.size) +
               "; expected Annex-B H.264/H.265 (00 00 01 with SPS/PPS/VPS) or JPEG/PNG. "
               "Inspect the retained *.input with 'ffprobe'/'xxd'.",
           error);
      return {};
    }
    if (!sniffed.empty() && sniffed != declared) {
      std::cerr << "DataCollector: camera format='" << frame.format << "' declares " << declared
                << " but the Annex-B payload is " << sniffed
                << "; following the payload (this is the rc4 h26x->hevc mismatch)\n";
      return sniffed;
    }
    return declared;
  }

  std::vector<std::string> FfmpegArgs() const {
    std::vector<std::string> args = {"ffmpeg", "-nostdin", "-loglevel", "error", "-y"};
    if (IsPacketStream()) {
      args.insert(args.end(), {"-f", codec_, "-r", std::to_string(target_hz_)});
    } else {
      args.insert(args.end(), {"-f", "image2pipe", "-vcodec", codec_, "-framerate",
                               std::to_string(target_hz_)});
    }
    args.insert(args.end(), {"-i", raw_path_.string(), "-an", "-c:v", "libx264", "-pix_fmt",
                             "yuv420p", "-preset", "medium", "-crf", "23", "-movflags",
                             "+faststart", output_path_.string()});
    return args;
  }

  static std::string ShellJoin(const std::vector<std::string>& args) {
    std::string out;
    for (const auto& arg : args) {
      if (!out.empty()) out.push_back(' ');
      if (arg.find_first_of(" \t\"'") == std::string::npos) {
        out += arg;
      } else {
        out.push_back('\'');
        for (const char c : arg) {
          if (c == '\'') out += "'\\''";
          else out.push_back(c);
        }
        out.push_back('\'');
      }
    }
    return out;
  }

  // ffmpeg writes its reason to stderr; on rc4 that text went to the terminal
  // but never into the collector error/manifest.  Keep it in a log next to the
  // payload and fold the tail into the reported error.
  static std::string LogTail(const fs::path& path, size_t max_bytes) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    const std::string text((std::istreambuf_iterator<char>(in)), {});
    std::string tail = text.size() > max_bytes ? text.substr(text.size() - max_bytes) : text;
    std::string collapsed;
    collapsed.reserve(tail.size());
    bool previous_space = false;
    for (const char c : tail) {
      const bool space = c == '\n' || c == '\r' || c == '\t' || c == ' ';
      if (space) {
        if (!previous_space) collapsed.push_back(' ');
        previous_space = true;
      } else {
        collapsed.push_back(c);
        previous_space = false;
      }
    }
    while (!collapsed.empty() && collapsed.back() == ' ') collapsed.pop_back();
    return collapsed;
  }

  bool RunFfmpeg(const std::vector<std::string>& args, std::string* error) {
    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.data()));
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    // Capture both streams so the failure text is available to the collector
    // (and therefore to episode.json) instead of only to the terminal.
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log_path_.c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, log_path_.c_str(),
                                     O_WRONLY | O_CREAT | O_APPEND, 0644);
    pid_t child = -1;
    const int spawn_result =
        ::posix_spawnp(&child, "ffmpeg", &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    if (spawn_result != 0) {
      return Fail("cannot start ffmpeg: " + std::string(std::strerror(spawn_result)) +
                      "; reproduce: " + ShellJoin(args),
                  error);
    }
    int status = 0;
    pid_t waited;
    do {
      waited = ::waitpid(child, &status, 0);
    } while (waited < 0 && errno == EINTR);
    std::error_code ec;
    const bool ok = waited >= 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0 &&
                    fs::exists(output_path_, ec) && fs::file_size(output_path_, ec) > 0 && !ec;
    if (ok) return true;
    fs::remove(output_path_, ec);
    const std::string tail = LogTail(log_path_, 3000);
    std::string message = "ffmpeg video export failed";
    if (waited < 0) {
      message += ": waitpid failed";
    } else if (WIFEXITED(status)) {
      message += ": exit code " + std::to_string(WEXITSTATUS(status));
    } else {
      message += ": terminated by signal";
    }
    message += "; ffmpeg stderr tail: " + (tail.empty() ? std::string("(empty)") : tail);
    message += "; compressed input retained at " + raw_path_.string() + "; full log: " +
               log_path_.string();
    message += "; reproduce: " + ShellJoin(args);
    std::cerr << "DataCollector: " << message << '\n';
    return Fail(message, error);
  }

  bool Fail(const std::string& message, std::string* error) {
    failed_ = true;
    if (error) *error = message;
    return false;
  }

  fs::path output_path_;
  fs::path raw_path_;
  fs::path log_path_;
  int target_hz_;
  std::string codec_;
  std::ofstream raw_;
  uint64_t sample_count_{0};
  bool failed_{false};
  bool closed_{false};
  bool ambiguous_packet_hint_{false};
};

class DataCollector {
 public:
  DataCollector(Robot& robot, fs::path output_dir, CollectionConfig config = {})
      : robot_(robot), output_dir_(std::move(output_dir)), config_(std::move(config)) {}

  ~DataCollector() { StopRecording(); }

  DataCollector(const DataCollector&) = delete;
  DataCollector& operator=(const DataCollector&) = delete;

  bool IsRecording() const noexcept { return recording_.load(std::memory_order_acquire); }
  const fs::path& OutputDir() const noexcept { return output_dir_; }
  const CollectionConfig& Config() const noexcept { return config_; }
  std::size_t EpisodeCount() const noexcept { return episode_count_; }

  bool StartRecording(std::string task = "default_task") {
    if (IsRecording()) {
      std::cerr << "DataCollector: recording is already active\n";
      return false;
    }
    std::error_code ec;
    fs::create_directories(output_dir_, ec);
    if (ec) {
      std::cerr << "DataCollector: cannot create output directory " << output_dir_
                << ": " << ec.message() << '\n';
      return false;
    }

    episode_dir_ = output_dir_ / ("episode_" + FourDigits(episode_count_));
    for (int suffix = 1; fs::exists(episode_dir_); ++suffix) {
      episode_dir_ = output_dir_ / ("episode_" + FourDigits(episode_count_) + "_" +
                                    std::to_string(suffix));
    }
    fs::create_directories(episode_dir_, ec);
    if (ec) {
      std::cerr << "DataCollector: cannot create episode directory: " << ec.message() << '\n';
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      task_ = std::move(task);
      started_ns_ = WallClockNs();
      record_count_ = 0;
      stream_counts_.clear();
      stream_errors_.clear();
      optional_streams_.clear();
      configured_streams_.clear();
      camera_streams_.clear();
      stream_first_ns_.clear();
      stream_last_ns_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(camera_index_mutex_);
      video_files_.clear();
    }
    recording_.store(true, std::memory_order_release);

    // The order mirrors Python v1: slave state/action streams, images, poses,
    // chassis/sensors, then master/wrench streams. Actions are intentionally
    // represented by the corresponding generated state stream when a server
    // has no separate action RPC; no fake protobuf contract is introduced.
    StartConfiguredStreams();
    if (workers_.empty()) {
      recording_.store(false, std::memory_order_release);
      std::cerr << "DataCollector: no stream enabled in CollectionConfig\n";
      return false;
    }
    std::cout << "Started recording " << episode_dir_ << " (task=" << task_ << ")\n";
    return true;
  }

  std::optional<EpisodeInfo> StopRecording() {
    if (!IsRecording() && workers_.empty()) return std::nullopt;
    recording_.store(false, std::memory_order_release);
    const auto stopped_ns = WallClockNs();
    for (auto& worker : workers_) {
      if (worker.joinable()) worker.join();
    }
    workers_.clear();

    // Enabled pose streams are required for Cartesian replay. Preserve partial
    // files for diagnosis, but never report a pose-less episode as successful.
    bool poses_complete = true;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      for (const auto& [enabled, name] : std::vector<std::pair<bool, std::string>>{
               {config_.enable_left_arm_end_pose, "left_arm_end_pose"},
               {config_.enable_right_arm_end_pose, "right_arm_end_pose"},
               {config_.enable_waist_end_pose, "waist_end_pose"}}) {
        if (!enabled) continue;
        if (!stream_counts_.count(name) || stream_counts_.at(name) == 0) {
          stream_errors_.emplace(name, "required pose stream produced no frames; "
                                      "pose feedback must be active before recording (collector does not switch modes)");
        }
        if (stream_errors_.count(name)) poses_complete = false;
      }
    }

    EpisodeInfo info;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      info.episode_id = episode_dir_.filename().string();
      info.task = task_;
      info.episode_dir = episode_dir_;
      info.num_records = record_count_;
      const auto ended_ns = stopped_ns;
      info.duration = started_ns_ == 0 ? 0.0
                                      : static_cast<double>(ended_ns - started_ns_) / 1e9;
    }
    if (!WriteManifest(info)) {
      std::cerr << "DataCollector: failed to write episode manifest\n";
      return std::nullopt;
    }
    PrintRecordingReport(info);
    ++episode_count_;
    std::cout << "Saved " << info.num_records << " records to " << info.episode_dir
              << " (" << std::fixed << std::setprecision(2) << info.duration << " s)\n";
    if (!poses_complete) {
      std::cerr << "DataCollector: FAIL: required pose stream missing or failed; "
                   "partial data and errors preserved in " << info.episode_dir << '\n';
      return std::nullopt;
    }
    return info;
  }

  void PrintStats(std::ostream& out = std::cout) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    out << "records=" << record_count_;
    for (const auto& [name, count] : stream_counts_) out << " " << name << "=" << count;
    out << '\n';
  }

  // Post-episode evidence for SX001-125/SX001-120: per-stream frame count and
  // the measured capture rate, plus the streams that produced no frame at all.
  // A jerky replay is then attributable to the recording (rate too low / a
  // stream never opened) instead of being guessed from the video.
  void PrintRecordingReport(const EpisodeInfo& info) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    std::cout << "DataCollector: " << info.episode_id << " records=" << record_count_
              << " duration=" << std::fixed << std::setprecision(2) << info.duration << "s\n";
    for (const auto& [name, count] : stream_counts_) {
      const double hz = EffectiveHz(count, stream_first_ns_.count(name) ? stream_first_ns_.at(name) : 0,
                                    stream_last_ns_.count(name) ? stream_last_ns_.at(name) : 0);
      std::cout << "  " << std::left << std::setw(28) << name << std::right << " records="
                << std::setw(6) << count << " hz=" << std::setprecision(1) << std::setw(6) << hz;
      if (camera_streams_.count(name) != 0) std::cout << " (video encoded at "
                                                     << config_.target_hz << " Hz)";
      std::cout << '\n';
      if (camera_streams_.count(name) != 0 && hz > 0.0 &&
          hz < static_cast<double>(config_.target_hz) * 0.8) {
        std::cout << "  warning: camera " << name << " captured " << std::setprecision(1) << hz
                  << " Hz but the MP4 is encoded at " << config_.target_hz
                  << " Hz; playback speed will not match wall clock\n";
      }
    }
    std::vector<std::string> missing_master_streams;
    for (const auto& name : configured_streams_) {
      if (stream_counts_.find(name) == stream_counts_.end()) {
        if (optional_streams_.count(name) && !stream_errors_.count(name)) {
          missing_master_streams.push_back(name);
          continue;
        }
        std::cout << "  stream without any record: " << name << " (rpc error: "
                  << (stream_errors_.count(name) ? stream_errors_.at(name) : std::string("none"))
                  << ")\n";
      }
    }
    if (!missing_master_streams.empty()) {
      std::cout << "DataCollector: WARNING: " << MissingMasterData() << " Streams:";
      for (const auto& name : missing_master_streams) std::cout << ' ' << name;
      std::cout << '\n';
    }
    std::cout << std::setprecision(6);
  }

 private:
  static std::string FourDigits(std::size_t value) {
    std::ostringstream out;
    out << std::setw(4) << std::setfill('0') << value;
    return out.str();
  }

  template <typename Open>
  void LaunchProto(const std::string& name, Open open, bool optional = false) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      configured_streams_.push_back(name);
      if (optional) optional_streams_.insert(name);
    }
    workers_.emplace_back([this, name, open = std::move(open), optional]() mutable {
      CollectProto(name, std::move(open), optional);
    });
  }

  template <typename Open>
  void CollectProto(const std::string& name, Open open, bool optional) {
    const fs::path file_path = episode_dir_ / ("stream_" + name + ".pb");
    std::ofstream out(file_path, std::ios::binary | std::ios::trunc);
    if (!out) {
      SetStreamError(name, "cannot open " + file_path.string());
      return;
    }
    auto opened = open();
    // A teleoperation master may be absent or connected after recording starts.
    // Keep waiting for optional master data instead of permanently abandoning
    // the worker after its first-frame timeout (SX001-161).
    while (!opened.Ok() && optional &&
           opened.ErrorValue().error_code == x2robot::sdk::errc::kTimeout) {
      if (!IsRecording()) return;
      opened = open();
    }
    if (!opened.Ok()) {
      SetStreamError(name, opened.ErrorValue().error_message);
      return;
    }
    auto stream = std::move(opened.Value());
    uint64_t last_ns = 0;
    uint64_t idle_timeouts = 0;
    const uint64_t period_ns = config_.downsample_joint_states && config_.target_hz > 0
                                   ? static_cast<uint64_t>(1e9 / config_.target_hz)
                                   : 0;
    while (IsRecording()) {
      auto item = stream->Read(Deadline{1000});
      if (!item.Ok()) {
        const auto error = item.ErrorValue();
        // A read timeout is an idle read window, not a dead subscription.  A
        // transient network stall used to end this stream silently and only
        // surface as "no data" during validation (SX001-140).
        if (IsRecording() && error.error_code == x2robot::sdk::errc::kTimeout) {
          if (++idle_timeouts % 30 == 0 && !optional) {
            std::cerr << "DataCollector: " << name << ": no data for " << idle_timeouts
                      << "s (still recording)\n";
          }
          continue;
        }
        if (IsRecording()) SetStreamError(name, error.error_message);
        break;
      }
      idle_timeouts = 0;
      const uint64_t now_ns = WallClockNs();
      // Downsample high-frequency state streams while retaining the first
      // sample. Other streams naturally run at their source rate.
      if (period_ns != 0 && IsJointStream(name) && last_ns != 0 && now_ns - last_ns < period_ns) {
        continue;
      }
      last_ns = now_ns;
      std::string payload;
      if (!SerializeRecord(name, item.Value(), &payload)) {
        SetStreamError(name, "non-finite gripper position");
        break;
      }
      WriteU64(out, now_ns);
      WriteU32(out, static_cast<uint32_t>(payload.size()));
      out.write(payload.data(), static_cast<std::streamsize>(payload.size()));
      if (!out) {
        SetStreamError(name, "write failed");
        break;
      }
      Count(name, now_ns);
    }
    (void)stream->Cancel();
  }

  template <typename Message>
  static bool SerializeRecord(const std::string&, const Message& value, std::string* payload) {
    *payload = value.SerializeAsString();
    return true;
  }

  static bool SerializeRecord(const std::string& name,
                              const ::xr::sdk::v2::GripperPosition& value,
                              std::string* payload) {
    if (!IsJointStream(name)) {
      *payload = value.SerializeAsString();
      return true;
    }
    if (!std::isfinite(value.position())) return false;
    // CX002 supplies only a scalar position. Preserve its units in the
    // existing replay record, without inventing velocity/effort or a ROS stamp.
    sensor_msgs::JointState state;
    state.add_name(name.substr(0, name.size() - std::string("_joint_states").size()));
    state.add_position(value.position());
    *payload = state.SerializeAsString();
    return true;
  }

  template <typename Open>
  void LaunchCamera(const std::string& name, Open open) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      configured_streams_.push_back(name);
      camera_streams_.insert(name);
    }
    workers_.emplace_back([this, name, open = std::move(open)]() mutable {
      CollectCamera(name, std::move(open));
    });
  }

  template <typename Open>
  void CollectCamera(const std::string& name, Open open) {
    const std::string video_stem = CameraFileStem(name);
    const fs::path video_path = episode_dir_ / (video_stem + ".mp4");
    const fs::path index_path = episode_dir_ / ("camera_" + name + ".jsonl");
    std::ofstream index(index_path, std::ios::out | std::ios::trunc);
    if (!index) {
      SetStreamError(name, "cannot open " + index_path.string());
      return;
    }
    CameraVideoWriter video(video_path, config_.target_hz);
    uint64_t frame_index = 0;
    auto callback = [this, name, video_stem, &index, &video, &frame_index](
                        const SdkResult<x2robot::sdk::client::CameraFrameView>& result) {
      if (!result.Ok()) {
        if (IsRecording()) SetStreamError(name, result.ErrorValue().error_message);
        return;
      }
      if (!IsRecording()) return;
      const auto& frame = result.Value();
      std::string video_error;
      if (!video.Write(frame, &video_error)) {
        SetStreamError(name, video_error);
        return;
      }
      const uint64_t stamp_ns = frame.stamp_ns > 0 ? frame.stamp_ns : WallClockNs();
      const uint64_t sequence = frame_index++;
      // Camera callbacks run on the gRPC reader thread; serialize only the
      // small index line and video-file map under the stream's mutex.
      {
        std::lock_guard<std::mutex> lock(camera_index_mutex_);
        index << "{\"timestamp_ns\":" << stamp_ns
              << ",\"sequence\":" << sequence << ",\"format\":\""
              << JsonEscape(std::string(frame.format)) << "\",\"path\":\""
              << JsonEscape(video_stem + ".mp4") << "\",\"frame_index\":" << sequence << "}\n";
        if (!index) {
          SetStreamError(name, "camera index write failed");
          return;
        }
      }
      Count(name, stamp_ns);
    };
    auto opened = open(std::move(callback));
    if (!opened.Ok()) {
      SetStreamError(name, opened.ErrorValue().error_message);
      return;
    }
    // The camera stream owns its reader thread. Keep the handle alive until
    // StopRecording changes the flag, then destruction cancels the RPC.
    while (IsRecording()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    opened.Value().reset();
    std::string video_error;
    if (!video.Close(&video_error)) {
      SetStreamError(name, video_error);
    } else if (video.HasSamples()) {
      std::lock_guard<std::mutex> lock(camera_index_mutex_);
      video_files_[video_stem] = video_stem + ".mp4";
    }
  }

  static std::string CameraFileStem(const std::string& name) {
    static const std::map<std::string, std::string> names = {
        {"head_rgb_stream", "head_camera"},
        {"head_depth_stream", "head_depth_camera"},
        {"left_arm_rgb_stream", "left_arm_camera"},
        {"right_arm_rgb_stream", "right_arm_camera"},
        {"head_left_eye_stream", "head_left_eye_camera"},
        {"head_right_eye_stream", "head_right_eye_camera"},
        {"left_arm_elbow_stream", "left_arm_elbow_camera"},
        {"right_arm_elbow_stream", "right_arm_elbow_camera"},
    };
    const auto it = names.find(name);
    return it == names.end() ? name : it->second;
  }

  static bool IsJointStream(const std::string& name) {
    return name.find("joint_states") != std::string::npos ||
           name.find("joint_state") != std::string::npos;
  }

  void Count(const std::string& name, uint64_t stamp_ns = 0) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++record_count_;
    ++stream_counts_[name];
    if (stamp_ns != 0) {
      if (stream_first_ns_[name] == 0 || stamp_ns < stream_first_ns_[name]) {
        stream_first_ns_[name] = stamp_ns;
      }
      if (stamp_ns > stream_last_ns_[name]) stream_last_ns_[name] = stamp_ns;
    }
  }

  // Effective capture rate per stream over the measured interval.  A recording
  // whose joint stream is 30 Hz and whose end-pose stream is 500 Hz is a
  // different replay problem from a recording that silently lost frames.
  static double EffectiveHz(uint64_t count, uint64_t first_ns, uint64_t last_ns) {
    if (count < 2 || last_ns <= first_ns) return 0.0;
    return static_cast<double>(count - 1) * 1e9 / static_cast<double>(last_ns - first_ns);
  }

  void SetStreamError(const std::string& name, std::string message) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (stream_errors_.find(name) == stream_errors_.end()) {
      stream_errors_.emplace(name, std::move(message));
      std::cerr << "DataCollector: " << name << ": " << stream_errors_.at(name) << '\n';
    }
  }

  void StartConfiguredStreams() {
    const auto has = [this](const std::string& name) {
      return std::find(config_.slave_joint_names.begin(), config_.slave_joint_names.end(), name) !=
             config_.slave_joint_names.end();
    };

    if (has("left_arm_joint_states")) {
      LaunchProto("left_arm_joint_states", [this] {
        return robot_.GetLeftArmController().GetJointStatesStream(Deadline{5000});
      });
    }
    if (has("right_arm_joint_states")) {
      LaunchProto("right_arm_joint_states", [this] {
        return robot_.GetRightArmController().GetJointStatesStream(Deadline{5000});
      });
    }
    if (has("lift_joint_states")) {
      LaunchProto("lift_joint_states", [this] {
        return robot_.GetLiftController().GetJointStatesStream(Deadline{5000});
      });
    }
    if (has("waist_joint_states")) {
      LaunchProto("waist_joint_states", [this] {
        return robot_.GetWaistController().GetJointStatesStream(Deadline{5000});
      });
    }
    if (has("left_gripper_joint_states")) {
      if (robot_.ModelName() == "quanta_x2") {
        LaunchProto("left_gripper_joint_states", [this] {
          return robot_.GetLeftGripperController().GetPositionStream(Deadline{5000});
        });
      } else {
        LaunchProto("left_gripper_joint_states", [this] {
          return robot_.GetLeftGripperController().GetJointStatesStream(Deadline{5000});
        });
      }
    }
    if (has("right_gripper_joint_states")) {
      if (robot_.ModelName() == "quanta_x2") {
        LaunchProto("right_gripper_joint_states", [this] {
          return robot_.GetRightGripperController().GetPositionStream(Deadline{5000});
        });
      } else {
        LaunchProto("right_gripper_joint_states", [this] {
          return robot_.GetRightGripperController().GetJointStatesStream(Deadline{5000});
        });
      }
    }
    if (has("head_joint_states")) {
      LaunchProto("head_joint_states", [this] {
        return robot_.GetHeadController().GetJointStatesStream(Deadline{5000});
      });
    }

    if (config_.enable_head_rgb_stream) {
      LaunchCamera("head_rgb_stream", [this](auto callback) {
        return robot_.GetHeadCamera().GetRGBVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_head_depth_stream || config_.enable_head_depth_video) {
      LaunchCamera("head_depth_stream", [this](auto callback) {
        return robot_.GetHeadCamera().GetDepthVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_left_arm_rgb_stream) {
      LaunchCamera("left_arm_rgb_stream", [this](auto callback) {
        return robot_.GetLeftArmCamera().GetVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_right_arm_rgb_stream) {
      LaunchCamera("right_arm_rgb_stream", [this](auto callback) {
        return robot_.GetRightArmCamera().GetVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_head_left_eye_stream) {
      LaunchCamera("head_left_eye_stream", [this](auto callback) {
        return robot_.GetHeadCamera().GetLeftEyeVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_head_right_eye_stream) {
      LaunchCamera("head_right_eye_stream", [this](auto callback) {
        return robot_.GetHeadCamera().GetRightEyeVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_left_arm_elbow_stream) {
      LaunchCamera("left_arm_elbow_stream", [this](auto callback) {
        return robot_.GetLeftArmCamera().GetElbowVideoStream(std::move(callback), Deadline{5000});
      });
    }
    if (config_.enable_right_arm_elbow_stream) {
      LaunchCamera("right_arm_elbow_stream", [this](auto callback) {
        return robot_.GetRightArmCamera().GetElbowVideoStream(std::move(callback), Deadline{5000});
      });
    }

    if (config_.enable_left_arm_end_pose) {
      LaunchProto("left_arm_end_pose", [this] {
        return robot_.GetLeftArmController().GetEndPoseStream(Deadline{5000});
      });
    }
    if (config_.enable_right_arm_end_pose) {
      LaunchProto("right_arm_end_pose", [this] {
        return robot_.GetRightArmController().GetEndPoseStream(Deadline{5000});
      });
    }
    if (config_.enable_wrench_ext_world) {
      LaunchProto("left_arm_wrench_ext_world", [this] {
        return robot_.GetLeftArmController().GetWrenchExtWorldStream(Deadline{5000});
      });
      LaunchProto("right_arm_wrench_ext_world", [this] {
        return robot_.GetRightArmController().GetWrenchExtWorldStream(Deadline{5000});
      });
    }
    if (config_.enable_wrench_ext_local) {
      LaunchProto("left_arm_wrench_ext_local", [this] {
        return robot_.GetLeftArmController().GetWrenchExtLocalStream(Deadline{5000});
      });
      LaunchProto("right_arm_wrench_ext_local", [this] {
        return robot_.GetRightArmController().GetWrenchExtLocalStream(Deadline{5000});
      });
    }
    if (config_.enable_waist_end_pose) {
      LaunchProto("waist_end_pose", [this] {
        return robot_.GetWaistController().GetEndPoseStream(Deadline{5000});
      });
    }
    if (config_.enable_odometry) {
      LaunchProto("odometry", [this] {
        return robot_.GetChassisController().GetOdometryStream(Deadline{5000});
      });
    }
    if (config_.enable_pose) {
      LaunchProto("pose", [this] {
        return robot_.GetChassisController().GetPoseStream(Deadline{5000});
      });
    }
    if (config_.enable_chassis_imu) {
      LaunchProto("chassis_imu", [this] {
        return robot_.GetImu().GetChassisImuStream(Deadline{5000});
      });
    }
    if (config_.enable_depth_points) {
      LaunchProto("depth_points", [this] {
        return robot_.GetDepthPoints().GetChassisDepthPointsStream(Deadline{5000});
      });
    }
    if (config_.enable_laser_scan) {
      LaunchProto("laser_scan", [this] {
        return robot_.GetRadarService().GetLaserScanStream(Deadline{5000});
      });
    }
    if (config_.enable_left_gripper_position) {
      LaunchProto("left_gripper_position", [this] {
        return robot_.GetLeftGripperController().GetPositionStream(Deadline{5000});
      });
    }
    if (config_.enable_right_gripper_position) {
      LaunchProto("right_gripper_position", [this] {
        return robot_.GetRightGripperController().GetPositionStream(Deadline{5000});
      });
    }
    if (config_.enable_left_gripper_tactile) {
      LaunchProto("left_gripper_tactile", [this] {
        return robot_.GetLeftGripperTactile().GetTactileSensorDataStream(Deadline{5000});
      });
    }
    if (config_.enable_right_gripper_tactile) {
      LaunchProto("right_gripper_tactile", [this] {
        return robot_.GetRightGripperTactile().GetTactileSensorDataStream(Deadline{5000});
      });
    }
    if (config_.enable_left_hand_tactile) {
      LaunchProto("left_hand_tactile", [this] {
        return robot_.GetLeftHandTactile().GetTactileSensorDataStream(Deadline{5000});
      });
    }
    if (config_.enable_right_hand_tactile) {
      LaunchProto("right_hand_tactile", [this] {
        return robot_.GetRightHandTactile().GetTactileSensorDataStream(Deadline{5000});
      });
    }
    if (config_.enable_tof_sensors) {
      LaunchProto("tof_1", [this] {
        return robot_.GetTof().GetChassisTof1Stream(Deadline{5000});
      });
      LaunchProto("tof_2", [this] {
        return robot_.GetTof().GetChassisTof2Stream(Deadline{5000});
      });
    }
    if (config_.enable_ultrasonic_sensors) {
      LaunchProto("ultrasonic_1", [this] {
        return robot_.GetUltrasonic().GetChassisUltrasonic1Stream(Deadline{5000});
      });
      LaunchProto("ultrasonic_2", [this] {
        return robot_.GetUltrasonic().GetChassisUltrasonic2Stream(Deadline{5000});
      });
      LaunchProto("ultrasonic_3", [this] {
        return robot_.GetUltrasonic().GetChassisUltrasonic3Stream(Deadline{5000});
      });
      LaunchProto("ultrasonic_4", [this] {
        return robot_.GetUltrasonic().GetChassisUltrasonic4Stream(Deadline{5000});
      });
    }
    if (config_.enable_master_arm_data) {
      LaunchProto("master_left_arm_joint_state", [this] {
        return robot_.GetMasterLeftArm().GetJointStatesStream(Deadline{1000});
      }, true);
      LaunchProto("master_right_arm_joint_state", [this] {
        return robot_.GetMasterRightArm().GetJointStatesStream(Deadline{1000});
      }, true);
      LaunchProto("master_left_arm_end_pose", [this] {
        return robot_.GetMasterLeftArm().GetEndPoseStream(Deadline{1000});
      }, true);
      LaunchProto("master_right_arm_end_pose", [this] {
        return robot_.GetMasterRightArm().GetEndPoseStream(Deadline{1000});
      }, true);
      LaunchProto("master_left_gripper_joint_state", [this] {
        return robot_.GetMasterLeftArm().GetGripperJointStatesStream(Deadline{1000});
      }, true);
      LaunchProto("master_right_gripper_joint_state", [this] {
        return robot_.GetMasterRightArm().GetGripperJointStatesStream(Deadline{1000});
      }, true);
    }
  }

  bool WriteManifest(const EpisodeInfo& info) const {
    std::ofstream out(info.episode_dir / "episode.json", std::ios::trunc);
    if (!out) return false;
    out << "{\n"
        << "  \"episode_id\": \"" << JsonEscape(info.episode_id) << "\",\n"
        << "  \"task\": \"" << JsonEscape(info.task) << "\",\n"
        << "  \"duration\": " << std::setprecision(10) << info.duration << ",\n"
        << "  \"num_records\": " << info.num_records << ",\n"
        << "  \"target_hz\": " << config_.target_hz << ",\n"
        << "  \"format\": \"sdk-protobuf-record-v1\",\n"
        << "  \"storage_format\": \"video\",\n"
        << "  \"video_files\": {";
    bool first_video = true;
    for (const auto& [name, path] : video_files_) {
      if (!first_video) out << ",";
      first_video = false;
      out << "\"" << JsonEscape(name) << "\":\"" << JsonEscape(path) << "\"";
    }
    out << "},\n"
        << "  \"streams\": [\n";
    bool first = true;
    for (const auto& [name, count] : stream_counts_) {
      if (!first) out << ",\n";
      first = false;
      const uint64_t stream_first = stream_first_ns_.count(name) ? stream_first_ns_.at(name) : 0;
      const uint64_t stream_last = stream_last_ns_.count(name) ? stream_last_ns_.at(name) : 0;
      out << "    {\"name\":\"" << JsonEscape(name) << "\",\"records\":" << count
          << ",\"first_ns\":" << stream_first << ",\"last_ns\":" << stream_last
          << ",\"hz\":" << std::setprecision(4) << EffectiveHz(count, stream_first, stream_last) << "}";
      out << std::setprecision(10);
    }
    out << "\n  ],\n  \"errors\": [";
    first = true;
    for (const auto& [name, message] : stream_errors_) {
      if (!first) out << ",";
      first = false;
      out << "{\"stream\":\"" << JsonEscape(name) << "\",\"message\":\""
          << JsonEscape(message) << "\"}";
    }
    out << "],\n  \"warnings\": [";
    first = true;
    for (const auto& name : optional_streams_) {
      if (stream_counts_.count(name) || stream_errors_.count(name)) continue;
      if (!first) out << ",";
      first = false;
      out << "{\"stream\":\"" << JsonEscape(name) << "\",\"message\":\""
          << JsonEscape(MissingMasterData()) << "\"}";
    }
    out << "]\n}\n";
    return static_cast<bool>(out);
  }

  static const char* MissingMasterData() {
    return "no master-arm data received; check the teleoperation master connection and ROS topic publisher. "
           "If no master is needed, use --no-master-arm-data. No samples were recorded for these streams.";
  }

  Robot& robot_;
  fs::path output_dir_;
  CollectionConfig config_;
  fs::path episode_dir_;
  std::string task_;
  std::atomic<bool> recording_{false};
  std::vector<std::thread> workers_;
  mutable std::mutex stats_mutex_;
  mutable std::mutex camera_index_mutex_;
  std::map<std::string, uint64_t> stream_counts_;
  std::map<std::string, std::string> stream_errors_;
  std::map<std::string, std::string> video_files_;
  std::vector<std::string> configured_streams_;
  std::set<std::string> camera_streams_;
  std::set<std::string> optional_streams_;
  std::map<std::string, uint64_t> stream_first_ns_;
  std::map<std::string, uint64_t> stream_last_ns_;
  uint64_t record_count_{0};
  uint64_t started_ns_{0};
  std::size_t episode_count_{0};
};

}  // namespace sdk_examples
