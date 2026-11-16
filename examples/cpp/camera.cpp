// Matches Python v1 camera.py: single frames, continuous playback, per-camera snapshots, and file
// output.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using x2robot::sdk::CameraId;
using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using x2robot::sdk::client::CameraFrameCallback;
using x2robot::sdk::client::CameraFrameView;
using x2robot::sdk::client::VideoStream;
using namespace std::chrono_literals;

namespace {
std::atomic_bool g_stop{false};
void OnSignal(int signal) {
  // camera 是只读示例，不会注册 SDK 模式清理钩子（没有 watchdog 直接 _Exit），
  // 所以这里自己置停止标志：第一次 Ctrl+C/SIGTERM 让取流循环正常退出并
  // Close()，第二次信号再由 RequestExitOnCtrlC 强制退出。
  g_stop.store(true);
  x2robot::examples::RequestExitOnCtrlC(signal);
}
}  // namespace

namespace {

struct Frame {
  std::string format;
  std::vector<std::uint8_t> bytes;
  std::uint64_t sequence{0};
  std::string error;
};

void CopyFirst(Frame& frame, const SdkResult<CameraFrameView>& result) {
  if (!result.Ok()) {
    frame.error = result.ErrorValue().error_name + ": " +
                  result.ErrorValue().error_message + " request_id=" + result.request_id;
    return;
  }
  const auto& value = result.Value();
  frame.format.assign(value.format.data(), value.format.size());
  if (value.data != nullptr && value.size != 0) {
    frame.bytes.assign(value.data, value.data + value.size);
  }
  frame.sequence = value.seq;
  // ROS spells this "16UC1; compressedDepth png". Normalize the format before
  // matching, otherwise its capital D leaves the transport header in the PNG
  // sent to ffplay (and in saved depth images).
  std::string format = frame.format;
  for (char& c : format) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (format.find("depth") != std::string::npos && frame.bytes.size() >= 20 &&
      frame.bytes[12] == 0x89 && frame.bytes[13] == 'P' && frame.bytes[14] == 'N' &&
      frame.bytes[15] == 'G' && frame.bytes[16] == '\r' && frame.bytes[17] == '\n' &&
      frame.bytes[18] == 0x1a && frame.bytes[19] == '\n') {
    frame.bytes.erase(frame.bytes.begin(), frame.bytes.begin() + 12);
  }
}

CameraId ParseCamera(const std::string& name) {
  if (name == "head_depth") return CameraId::kHeadDepth;
  if (name == "head_left_eye") return CameraId::kHeadLeftEye;
  if (name == "head_right_eye") return CameraId::kHeadRightEye;
  if (name == "left_wrist") return CameraId::kLeftWrist;
  if (name == "left_elbow") return CameraId::kLeftElbow;
  if (name == "right_wrist") return CameraId::kRightWrist;
  if (name == "right_elbow") return CameraId::kRightElbow;
  return CameraId::kHeadRgb;
}

SdkResult<void> GetImage(Robot& robot, CameraId id, const CameraFrameCallback& callback) {
  switch (id) {
    case CameraId::kHeadRgb: return robot.GetHeadCamera().GetRGBImage(callback);
    case CameraId::kHeadDepth: return robot.GetHeadCamera().GetDepthImage(callback);
    case CameraId::kHeadLeftEye: return robot.GetHeadCamera().GetLeftEyeImage(callback);
    case CameraId::kHeadRightEye: return robot.GetHeadCamera().GetRightEyeImage(callback);
    case CameraId::kLeftWrist: return robot.GetLeftArmCamera().GetRawImage(callback);
    case CameraId::kLeftElbow: return robot.GetLeftArmCamera().GetElbowImage(callback);
    case CameraId::kRightWrist: return robot.GetRightArmCamera().GetRawImage(callback);
    case CameraId::kRightElbow: return robot.GetRightArmCamera().GetElbowImage(callback);
  }
  return robot.GetHeadCamera().GetRGBImage(callback);
}

SdkResult<VideoStream> GetStream(Robot& robot, CameraId id, CameraFrameCallback callback) {
  switch (id) {
    case CameraId::kHeadRgb: return robot.GetHeadCamera().GetRGBVideoStream(std::move(callback));
    case CameraId::kHeadDepth: return robot.GetHeadCamera().GetDepthVideoStream(std::move(callback));
    case CameraId::kHeadLeftEye: return robot.GetHeadCamera().GetLeftEyeVideoStream(std::move(callback));
    case CameraId::kHeadRightEye: return robot.GetHeadCamera().GetRightEyeVideoStream(std::move(callback));
    case CameraId::kLeftWrist: return robot.GetLeftArmCamera().GetVideoStream(std::move(callback));
    case CameraId::kLeftElbow: return robot.GetLeftArmCamera().GetElbowVideoStream(std::move(callback));
    case CameraId::kRightWrist: return robot.GetRightArmCamera().GetVideoStream(std::move(callback));
    case CameraId::kRightElbow: return robot.GetRightArmCamera().GetElbowVideoStream(std::move(callback));
  }
  return robot.GetHeadCamera().GetRGBVideoStream(std::move(callback));
}

std::string Extension(std::string format) {
  for (char& c : format) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (format.find("jpeg") != std::string::npos || format.find("jpg") != std::string::npos) return ".jpg";
  if (format.find("png") != std::string::npos || format.find("depth") != std::string::npos) return ".png";
  if (format.find("h265") != std::string::npos || format.find("hevc") != std::string::npos) return ".h265";
  if (format.find("h264") != std::string::npos || format.find("avc") != std::string::npos) return ".h264";
  return ".bin";
}

bool Save(const Frame& frame, const std::string& directory, const char* stem) {
  std::filesystem::path dir(directory);
  std::filesystem::create_directories(dir);
  const auto path = dir / (std::string(stem) + Extension(frame.format));
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char*>(frame.bytes.data()),
               static_cast<std::streamsize>(frame.bytes.size()));
  if (!output) {
    std::cerr << "cannot write " << path << '\n';
    return false;
  }
  std::cout << "saved " << path << '\n';
  return true;
}

enum class PlayerMode { kImage, kStream };

FILE* OpenPlayer(const Frame& frame, PlayerMode mode) {
  if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
    std::cerr << "--show requested but no DISPLAY/WAYLAND_DISPLAY; use --no-show for headless capture\n";
    return nullptr;
  }
  if (std::system("command -v ffplay >/dev/null 2>&1") != 0) {
    std::cerr << "--show requested but ffplay is not installed\n";
    return nullptr;
  }
  std::string format = frame.format;
  for (char& c : format) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  // No -nostdin here: that is an ffmpeg-CLI option, while ffplay 4.4 (Ubuntu
  // 22.04) has no such flag and exits immediately with
  // "Failed to set value '-autoexit' for option 'nostdin': Option not found".
  // With it the player died on the first write, so --show never opened a window
  // even when DISPLAY and ffplay were both available (SX001-139/SX001-149).
  // ffplay does not read keyboard commands from stdin before FFmpeg 5.1, so
  // dropping the flag does not steal input.
  // A snapshot must stay visible after its one frame reaches EOF (SX001-167).
  // Streams still exit at EOF so --seconds and normal shutdown remain bounded.
  std::string command = "ffplay -loglevel error ";
  if (mode == PlayerMode::kStream) command += "-autoexit ";
  if (format.find("h265") != std::string::npos || format.find("hevc") != std::string::npos) {
    command += "-f hevc ";
  } else if (format.find("h264") != std::string::npos || format.find("avc") != std::string::npos) {
    command += "-f h264 ";
  } else if (format.find("jpeg") != std::string::npos || format.find("jpg") != std::string::npos) {
    command += "-f mjpeg ";
  } else if (format.find("png") != std::string::npos || format.find("depth") != std::string::npos) {
    // The codec is already known. Avoid ffplay's long image2pipe probe, which
    // delays low-rate depth playback and can leave pclose waiting after EOF.
    command += "-f image2pipe -vcodec png -probesize 32 -analyzeduration 0 ";
  }
  command += "-i - >/dev/null";  // Keep decoder/display errors visible on stderr.
  return ::popen(command.c_str(), "w");
}

bool Play(FILE* pipe, const Frame& frame) {
  const auto written = std::fwrite(frame.bytes.data(), 1, frame.bytes.size(), pipe);
  return written == frame.bytes.size() && std::fflush(pipe) == 0;
}

bool Show(const Frame& frame) {
  FILE* pipe = OpenPlayer(frame, PlayerMode::kImage);
  if (!pipe) return false;
  std::cout << "image preview: close the window or press Q/Esc to continue\n" << std::flush;
  const bool ok = Play(pipe, frame);
  const int status = ::pclose(pipe);
  if (!ok || status != 0) std::cerr << "image display failed; check ffplay and display settings\n";
  return ok && status == 0;
}

// Display in this example is an ffplay child process, not OpenCV highgui: the
// C++ SDK has no GUI dependency. Print the display verdict before connecting so
// a missing window can be told apart from a missing stream without waiting for
// the first frame (SX001-139/SX001-149 reports mixed both symptoms).
void ReportDisplayMode(bool show) {
  if (!show) {
    std::cout << "display: off (send --show to display frames)\n";
    return;
  }
  const char* display = std::getenv("DISPLAY");
  const char* wayland = std::getenv("WAYLAND_DISPLAY");
  if (display == nullptr && wayland == nullptr) {
    std::cout << "display: requested but no DISPLAY/WAYLAND_DISPLAY in this session; "
                 "use --no-show for headless capture\n";
    return;
  }
  if (std::system("command -v ffplay >/dev/null 2>&1") != 0) {
    std::cout << "display: requested but ffplay is not installed; "
                 "install ffmpeg/ffplay or use --no-show\n";
    return;
  }
  std::cout << "display: on (ffplay";
  if (display != nullptr) std::cout << ", DISPLAY=" << display;
  if (wayland != nullptr) std::cout << ", WAYLAND_DISPLAY=" << wayland;
  std::cout << ")\n";
}

bool IsVideo(const Frame& frame) {
  std::string format = frame.format;
  for (char& c : format) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return format.find("h264") != std::string::npos || format.find("h265") != std::string::npos ||
         format.find("h26x") != std::string::npos || format.find("hevc") != std::string::npos ||
         format.find("avc") != std::string::npos;
}

bool StartAtVideoParameters(Frame& frame, std::vector<std::uint8_t>& pending) {
  const auto extension = Extension(frame.format);
  if (extension != ".h264" && extension != ".h265") return true;
  // 订阅从 GOP 中间开始时，先等待 SPS/VPS；保留跨消息分片的起始码。
  pending.insert(pending.end(), frame.bytes.begin(), frame.bytes.end());
  for (std::size_t i = 0; i + 3 < pending.size(); ++i) {
    if (pending[i] != 0 || pending[i + 1] != 0 || pending[i + 2] != 1) continue;
    const auto type = extension == ".h265" ? (pending[i + 3] >> 1) & 0x3f
                                            : pending[i + 3] & 0x1f;
    if (type != (extension == ".h265" ? 32 : 7)) continue;
    frame.bytes.assign(pending.begin() + i, pending.end());
    pending.clear();
    return true;
  }
  if (pending.size() > 4) pending.erase(pending.begin(), pending.end() - 4);
  return false;
}

bool FetchFirstVideoFrame(Robot& robot, CameraId id, Frame& image) {
  // As in Python v1, if a single packet contains H.26x data, start the corresponding stream and
  // wait for the parameter sets and a keyframe. ffmpeg is only a runtime decoder; the C++ SDK does
  // not need to link any additional video libraries.
  if (std::system("command -v ffmpeg >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1") != 0) {
    std::cerr << "H.26x image decoding requires ffmpeg and timeout\n";
    return false;
  }
  char temporary[] = "/tmp/sdk-camera-XXXXXX";
  if (::mkdtemp(temporary) == nullptr) {
    std::cerr << "cannot create camera decoder temporary directory\n";
    return false;
  }
  const auto png = std::filesystem::path(temporary) / "frame.png";
  const auto log = std::filesystem::path(temporary) / "decoder.log";
  auto cleanup = [&] {
    std::error_code ignored;
    std::filesystem::remove(png, ignored);
    std::filesystem::remove(log, ignored);
    std::filesystem::remove(temporary, ignored);
  };
  std::string input_format;
  if (Extension(image.format) == ".h265") input_format = "-f hevc ";
  else if (Extension(image.format) == ".h264") input_format = "-f h264 ";
  // Paths are created by mkdtemp, never from a camera payload or CLI input.
  const std::string command = "timeout -k 1s 8s ffmpeg -hide_banner -loglevel error -y "
      "-probesize 32768 -analyzeduration 0 " + input_format +
      "-i pipe:0 -frames:v 1 -f image2 -update 1 " + png.string() + " 2>" + log.string();
  FILE* decoder = ::popen(command.c_str(), "w");
  if (!decoder) { cleanup(); return false; }
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<Frame> packets;
  bool overflow = false;
  auto opened = GetStream(robot, id, [&](const SdkResult<CameraFrameView>& result) {
    Frame frame;
    CopyFirst(frame, result);
    std::lock_guard<std::mutex> lock(mutex);
    if (packets.size() >= 64) overflow = true;
    else packets.push_back(std::move(frame));
    ready.notify_one();
  });
  bool success = opened.Ok();
  if (!success) {
    std::cerr << "GetVideoStream failed: " << opened.ErrorValue().error_message
              << " request_id=" << opened.request_id << '\n';
  } else {
    auto stream = std::move(opened.Value());
    bool parameters_seen = false;
    std::vector<std::uint8_t> pending_parameters;
    const auto until = std::chrono::steady_clock::now() + 8s;
    std::cout << "waiting for video parameters and a keyframe (8s timeout)...\n";
    while (!g_stop.load() && std::chrono::steady_clock::now() < until) {
      // The encoder creates this file only once it has decoded its first frame.
      if (std::filesystem::exists(png)) break;
      Frame packet;
      {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait_for(lock, 100ms, [&] { return overflow || !packets.empty(); });
        if (overflow) { success = false; break; }
        if (packets.empty()) continue;
        packet = std::move(packets.front());
        packets.pop_front();
      }
      if (!packet.error.empty() || packet.bytes.empty()) {
        std::cerr << "camera stream failed: " << packet.error << '\n';
        success = false;
        break;
      }
      if (!parameters_seen) {
        parameters_seen = StartAtVideoParameters(packet, pending_parameters);
        if (!parameters_seen) continue;
      }
      // timeout also bounds a blocked pipe write if the decoder stalls.
      if (!Play(decoder, packet)) break;
      image.sequence = packet.sequence;
    }
    stream.reset();
  }
  // Closing stdin flushes a pending decoder frame; wait before reading the PNG.
  const int status = ::pclose(decoder);
  std::ifstream file(png, std::ios::binary);
  std::vector<std::uint8_t> decoded((std::istreambuf_iterator<char>(file)), {});
  success = success && !g_stop.load() && status == 0 && decoded.size() > 8 &&
            decoded[0] == 0x89 && decoded[1] == 'P' && decoded[2] == 'N' && decoded[3] == 'G';
  if (success) {
    image.format = "png";
    image.bytes = std::move(decoded);
  } else {
    std::cerr << "could not decode a camera keyframe within 8 seconds: decoder_status="
              << status << " decoded_bytes=" << decoded.size() << '\n';
    std::ifstream errors(log);
    std::cerr << errors.rdbuf();
  }
  cleanup();
  return success;
}

bool ParseSeconds(const char* argument, double& seconds) {
  char* end = nullptr;
  seconds = std::strtod(argument, &end);
  return end != argument && *end == '\0' && std::isfinite(seconds) && seconds > 0.0;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, OnSignal);
  // timeout/kill 发的是 SIGTERM；没有 watchdog 的只读示例也要能正常停下来。
  std::signal(SIGTERM, OnSignal);
  std::signal(SIGPIPE, SIG_IGN);  // Closing ffplay's window must not kill the SDK process.
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "usage: " << argv[0]
                << " head <rgb-image|depth-image|left-eye-image|right-eye-image|rgb-stream|depth-stream|left-eye-stream|right-eye-stream>\n"
                << "       " << argv[0] << " left-arm|right-arm <raw-image|elbow-image|stream|elbow-stream>\n"
                << "       " << argv[0] << " all\n"
                << "options: --server address --show|--no-show --save-dir dir --seconds N\n"
                << "legacy positional form: [address] [camera] [output-dir]\n"
                << "streams run until Ctrl+C unless --seconds is supplied\n"
                << "--show displays frames through ffplay and needs DISPLAY/WAYLAND_DISPLAY "
                   "plus ffplay; without them frames are still received and counted\n"
                << "use --seconds N --no-show to verify the stream rate without a GUI\n";
      return 0;
    }
  }
  std::string address = "127.0.0.1:55051";
  std::string camera_name = "head_rgb";
  std::string output;
  // Streams run until Ctrl+C by default.  --seconds is an optional bounded
  // test mode (useful in CI and shell scripts).
  double seconds = 0.0;
  bool show = std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
  bool explicit_show = false;
  std::string action = "head";
  int positional = 0;
  // v1 has three subcommands: head, left_arm and right_arm, plus all.  Keep
  // the old positional form accepted by the previous C++ example as well.
  if (argc > 1 && (std::string(argv[1]) == "head" || std::string(argv[1]) == "left_arm" ||
                   std::string(argv[1]) == "right_arm" || std::string(argv[1]) == "all" ||
                   std::string(argv[1]) == "left-arm" || std::string(argv[1]) == "right-arm")) {
    action = argv[1];
    if (action == "left-arm") action = "left_arm";
    if (action == "right-arm") action = "right_arm";
    // Accept both documented forms (`head rgb-image address`) and the older
    // shell-friendly form (`head address rgb-image`).  Options are parsed in
    // this branch too; previously --output/--show were silently ignored.
    std::vector<std::string> words;
    for (int i = 2; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--show") {
        show = true; explicit_show = true;
      } else if (argument == "--no-show") {
        show = false; explicit_show = true;
      } else if (argument == "--server" && i + 1 < argc) {
        address = argv[++i];
      } else if (argument == "--seconds" && i + 1 < argc) {
        if (!ParseSeconds(argv[++i], seconds)) {
          std::cerr << "--seconds must be a positive finite number\n";
          return 2;
        }
      } else if ((argument == "--output" || argument == "--save-dir") &&
                 i + 1 < argc) {
        output = argv[++i];
      } else if (!argument.empty() && argument[0] != '-') {
        words.push_back(argument);
      } else {
        std::cerr << "unknown option or missing option value: " << argument << '\n';
        return 2;
      }
    }
    auto is_head_action = [](const std::string& value) {
      return value == "rgb-image" || value == "depth-image" ||
             value == "left-eye-image" || value == "right-eye-image" ||
             value == "rgb-stream" || value == "depth-stream" ||
             value == "left-eye-stream" || value == "right-eye-stream";
    };
    auto is_arm_action = [](const std::string& value) {
      return value == "raw-image" || value == "elbow-image" ||
             value == "stream" || value == "elbow-stream";
    };
    if (action != "all") {
      std::string requested;
      bool positional_address = false;
      for (const auto& word : words) {
        if ((action == "head" && is_head_action(word)) ||
            (action != "head" && is_arm_action(word))) {
          if (!requested.empty()) {
            std::cerr << "specify only one camera action\n";
            return 2;
          }
          requested = word;
        } else if (word.find(':') != std::string::npos && !positional_address) {
          address = word;
          positional_address = true;
        } else {
          std::cerr << "unknown " << action << " action: " << word << " (see --help)\n";
          return 2;
        }
      }
      if (requested.empty()) {
        std::cerr << action << " requires a camera action (see --help)\n";
        return 2;
      }
      if (action == "head") {
        if (requested == "rgb-image") camera_name = "head_rgb";
        else if (requested == "depth-image") camera_name = "head_depth";
        else if (requested == "left-eye-image") camera_name = "head_left_eye";
        else if (requested == "right-eye-image") camera_name = "head_right_eye";
        else if (requested == "rgb-stream") { camera_name = "head_rgb"; action = "stream"; }
        else if (requested == "depth-stream") { camera_name = "head_depth"; action = "stream"; }
        else if (requested == "left-eye-stream") { camera_name = "head_left_eye"; action = "stream"; }
        else if (requested == "right-eye-stream") { camera_name = "head_right_eye"; action = "stream"; }
      } else {
        const bool left = action == "left_arm";
        if (requested == "raw-image") camera_name = left ? "left_wrist" : "right_wrist";
        else if (requested == "elbow-image") camera_name = left ? "left_elbow" : "right_elbow";
        else if (requested == "stream") { camera_name = left ? "left_wrist" : "right_wrist"; action = "stream"; }
        else if (requested == "elbow-stream") { camera_name = left ? "left_elbow" : "right_elbow"; action = "stream"; }
      }
    } else if (!words.empty()) {
      if (words.size() != 1 || words.front().find(':') == std::string::npos) {
        std::cerr << "all accepts only an optional host:port address (see --help)\n";
        return 2;
      }
      address = words.front();
    }
  } else {
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--show") { show = true; explicit_show = true; }
      else if (argument == "--no-show") { show = false; explicit_show = true; }
      else if (argument == "--server" && i + 1 < argc) address = argv[++i];
      else if (argument == "--camera" && i + 1 < argc) camera_name = argv[++i];
      else if (argument == "--seconds" && i + 1 < argc) {
        if (!ParseSeconds(argv[++i], seconds)) {
          std::cerr << "--seconds must be a positive finite number\n";
          return 2;
        }
      }
      else if ((argument == "--output" || argument == "--save-dir") && i + 1 < argc) output = argv[++i];
      else if (!argument.empty() && argument[0] != '-') {
        if (positional == 0) {
          if (argument.find(':') == std::string::npos) {
            std::cerr << "unknown subcommand or invalid address: " << argument << '\n';
            return 2;
          }
          address = argument;
        }
        else if (positional == 1) camera_name = argument;
        else if (positional == 2) output = argument;
        else { std::cerr << "too many positional arguments\n"; return 2; }
        ++positional;
      } else {
        std::cerr << "unknown option or missing option value: " << argument << '\n';
        return 2;
      }
    }
  }
  if (camera_name != "head_rgb" && camera_name != "head_depth" && camera_name != "head_left_eye" &&
      camera_name != "head_right_eye" && camera_name != "left_wrist" && camera_name != "left_elbow" &&
      camera_name != "right_wrist" && camera_name != "right_elbow") {
    std::cerr << "unknown camera: " << camera_name << '\n';
    return 2;
  }
  if (action == "all" && !explicit_show) show = false;

  if (action == "all") {
    std::cout << "server=" << address << " action=all\n";
  } else {
    std::cout << "server=" << address << " camera=" << camera_name << " action=" << action
              << '\n';
  }
  ReportDisplayMode(show);

  auto connected = Robot::Create(address, true);
  if (!connected.Ok()) {
    std::cerr << "Robot.Create failed: " << connected.ErrorValue().error_message << '\n';
    return 1;
  }
  auto robot = std::move(connected.Value());
  const bool x3 = robot->ModelName() == "quanta_x3";
  bool ok = true;
  auto capture = [&](CameraId id, const std::string& stem) {
    Frame image;
    if (x3) {
      // X3 仅公开视频流；采集图片时从流中解码首帧，不能全部跳过后返回成功。
      if (!FetchFirstVideoFrame(*robot, id, image)) return false;
      std::cout << stem << " format=" << image.format << " bytes=" << image.bytes.size() << '\n';
      bool result = output.empty() || Save(image, output, stem.c_str());
      if (show && !Show(image)) result = false;
      return result;
    }
    auto image_result = GetImage(*robot, id, [&](const SdkResult<CameraFrameView>& result) { CopyFirst(image, result); });
    if (!image_result.Ok()) {
      const auto& error = image_result.ErrorValue();
      if (action == "all" && (error.error_name == "UNAVAILABLE" || error.error_name == "UNIMPLEMENTED")) {
        std::cout << "SKIP " << stem << ": " << error.error_message << '\n';
        return true;
      }
      std::cerr << "GetImage failed: " << error.error_name << ": " << error.error_message
                << " request_id=" << image_result.request_id << '\n';
      return false;
    }
    if (!image.error.empty()) { std::cerr << image.error << '\n'; return false; }
    if (image.bytes.empty()) { std::cerr << "GetImage returned no bytes\n"; return false; }
    std::cout << stem << " format=" << image.format << " bytes=" << image.bytes.size() << '\n';
    if (IsVideo(image) && (action != "all" || show) && !FetchFirstVideoFrame(*robot, id, image)) return false;
    bool result = output.empty() || Save(image, output, stem.c_str());
    if (show && !Show(image)) result = false;
    return result;
  };
  auto stream_camera = [&](CameraId id, const std::string& label) {
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Frame> packets;
    bool overflow = false;
    auto opened = GetStream(*robot, id, [&](const SdkResult<CameraFrameView>& result) {
      Frame frame;
      CopyFirst(frame, result);  // Callback views expire on return; decode/play on main thread.
      std::lock_guard<std::mutex> lock(mutex);
      if (packets.size() >= 64) overflow = true;
      else packets.push_back(std::move(frame));
      ready.notify_one();
    });
    if (!opened.Ok()) { std::cerr << "GetVideoStream failed: " << opened.ErrorValue().error_message << '\n'; return false; }
    auto stream = std::move(opened.Value());
    FILE* player = nullptr;
    std::uint64_t frames = 0;
    bool success = true;
    const auto started = std::chrono::steady_clock::now();
    auto last_packet = started;
    auto last_report = started;
    const auto until = started + std::chrono::duration<double>(seconds);
    while (!g_stop.load() &&
           (seconds <= 0.0 || std::chrono::steady_clock::now() < until)) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait_for(lock, 100ms, [&] { return !packets.empty() || overflow; });
        if (overflow) {
          std::cerr << "camera consumer is too slow; stream buffer is full\n";
          success = false;
          break;
        }
        if (packets.empty()) {
          if (std::chrono::steady_clock::now() - last_packet > 8s) {
            std::cerr << "no camera data for 8 seconds\n";
            success = false;
            break;
          }
          continue;
        }
        frame = std::move(packets.front());
        packets.pop_front();
      }
      if (!frame.error.empty() || frame.bytes.empty()) {
        std::cerr << "camera stream failed: " << frame.error << '\n';
        success = false;
        break;
      }
      last_packet = std::chrono::steady_clock::now();
      ++frames;
      if (frames == 1) {
        std::cout << label << " format=" << frame.format << " bytes=" << frame.bytes.size() << '\n';
        if (show) player = OpenPlayer(frame, PlayerMode::kStream);
      }
      // 每秒报一次帧数：不用等 Ctrl+C 就能确认取流是否持续（现场判断
      // "只拿到一帧" 到底是流断了还是显示链路断了）。
      if (last_packet - last_report >= 1s) {
        last_report = last_packet;
        std::cout << label << " frames=" << frames << '\n';
      }
      if (!output.empty() && !Save(frame, output, (label + "_" + std::to_string(frames)).c_str())) {
        success = false;
        break;
      }
      // 显示链路（ffplay）失败不是相机流故障：关掉播放器继续取流。以前这里
      // 直接 break，会把 SDK stream 一起取消，表现就是"只拿到一两帧就断开"。
      if (player && !Play(player, frame)) {
        std::cerr << label << ": video display stopped; continuing to receive frames "
                                      "(use --no-show to skip display)\n";
        (void)::pclose(player);
        player = nullptr;
      }
    }
    stream.reset();
    if (player) (void)::pclose(player);
    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::cout << label << " stream frames=" << frames;
    if (elapsed > 0.0) std::cout << " avg_fps=" << static_cast<double>(frames) / elapsed;
    std::cout << '\n';
    if (frames == 0) {
      std::cerr << label << ": no frames received before the stream ended; check that this "
                             "camera exists on the robot model and that the server version "
                             "matches the client\n";
    }
    return success && frames != 0;
  };
  auto id = ParseCamera(camera_name);
  if (action == "all") {
    std::vector<std::pair<CameraId, std::string>> cameras = {
      {CameraId::kHeadRgb, "head_rgb"}, {CameraId::kHeadDepth, "head_depth"},
      {CameraId::kHeadLeftEye, "head_left_eye"}, {CameraId::kHeadRightEye, "head_right_eye"},
      {CameraId::kLeftWrist, "left_wrist"}, {CameraId::kLeftElbow, "left_elbow"},
      {CameraId::kRightWrist, "right_wrist"}, {CameraId::kRightElbow, "right_elbow"}};
    if (x3) cameras = {
      {CameraId::kHeadLeftEye, "head_left_eye"}, {CameraId::kHeadRightEye, "head_right_eye"},
      {CameraId::kLeftWrist, "left_wrist"}, {CameraId::kRightWrist, "right_wrist"}};
    for (const auto& [camera, stem] : cameras) ok = capture(camera, stem) && ok;
  } else if (action == "stream") ok = stream_camera(id, camera_name);
  else ok = capture(id, camera_name);

  if (!robot->Close().Ok()) ok = false;
  return ok ? 0 : 1;
}
