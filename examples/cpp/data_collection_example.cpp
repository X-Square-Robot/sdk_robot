#include "data_collection/data_collector.hpp"
#include "data_collection/options.hpp"
#include "interrupt.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>

namespace {
using x2robot::examples::ExitRequestedBySignal;

bool ReadLine(std::string& line, const std::function<void()>& on_wait = {}) {
  line.clear();
  while (!ExitRequestedBySignal()) {
    if (on_wait) on_wait();
    // in_avail() only reports the C++ stream buffer, not terminal readiness.
    // Use poll/read for every prompt so piped lines cannot be hidden by cin's
    // read-ahead buffer. Reading one byte also preserves subsequent answers.
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const int ready = ::poll(&input, 1, 100);
    if (ready == 0 || (ready < 0 && errno == EINTR)) continue;
    if (ready < 0) {
      std::perror("stdin poll failed");
      return false;
    }
    char ch;
    const auto count = ::read(STDIN_FILENO, &ch, 1);
    if (count < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      std::perror("stdin read failed");
      return false;
    }
    if (count == 0) return !line.empty();
    if (ch == '\n') return true;
    line += ch;
  }
  return false;
}

void Usage(const char* program) {
  std::cout << "Usage: " << program
            << " [server] [output-dir] [auto|x1|x2|x3|desktop] [options]\n"
               "  Options may appear anywhere on the command line:\n"
               "    --binocular       EX001/ex001 construction: use the head left/right eye and\n"
               "                      elbow cameras instead of head_rgb.  Required on EX001 Orin:\n"
               "                      that build publishes no head_rgb stream\n"
               "                      (sdk2/config/ex001/camera_orin.yml), so without this flag the\n"
               "                      head camera subscription fails.\n"
               "    --seconds N       record one episode of N seconds without waiting for stdin\n"
               "    --once            record a single episode and exit\n"
               "    --no-master-arm-data  skip master-arm streams when no teleoperation master is connected\n"
               "  auto (default) 用客户端解析出的机型（设备自报型号）选采集配置；\n"
               "Interactive mode mirrors python/v1/data_collection_example.py.\n"
               "Examples:\n"
               "  " << program << " 10.100.20.173:55051 --binocular\n"
               "  " << program << " 192.168.10.1:55051 ./collected_data desktop --seconds 20\n";
}
}

int main(int argc, char** argv) {
  sdk_examples::DataCollectionOptions options;
  std::string parse_error;
  if (!sdk_examples::ParseDataCollectionArgs(argc, argv, &options, &parse_error)) {
    std::cerr << "Invalid arguments: " << parse_error << '\n';
    Usage(argv[0]);
    return 2;
  }
  if (options.help) {
    Usage(argv[0]);
    return 0;
  }
  const std::string& address = options.address;
  const std::filesystem::path output = options.output;
  const std::string& requested_model = options.model;
  const bool binocular = options.binocular;
  const bool once = options.once;
  const int seconds = options.seconds;

  x2robot::examples::InstallGracefulExitOnCtrlC();
  std::cout << "Connecting to robot " << address << "...\n";
  auto connected = x2robot::sdk::Robot::Create(address, true);
  if (!connected.Ok()) {
    std::cerr << "Robot.Create failed: " << connected.ErrorValue().error_message << '\n';
    return 1;
  }
  auto robot = std::move(connected.Value());
  // 默认按客户端解析出的机型选配置：桌面机以前因为默认 x2，会去拉 head/waist/
  // chassis 这些该机型没有的流，日志刷 "在当前机型未配置"。
  std::string model_key = CollectionConfig::NormalizeModelKey(requested_model);
  const bool auto_detected = model_key.empty();
  if (auto_detected) model_key = CollectionConfig::NormalizeModelKey(robot->ModelName());
  if (model_key.empty()) {
    std::cerr << "Unsupported model '" << requested_model << "' (client reports '"
              << robot->ModelName() << "'); pass x1, x2, x3 or desktop explicitly.\n";
    (void)robot->Close();
    return 2;
  }
  std::cout << "model=" << model_key << (auto_detected ? " (auto from client)" : " (explicit)")
            << " binocular=" << (binocular ? "yes" : "no") << "\n";
  if (model_key == "x1" && !binocular) {
    std::cout << "note: EX001 Orin builds expose head eye/elbow streams instead of head_rgb; "
                 "pass --binocular when the scene camera is missing.\n";
  }
  CollectionConfig config = CollectionConfig::ForModel(model_key, binocular);
  if (options.no_master_arm_data) config.enable_master_arm_data = false;
  sdk_examples::DataCollector collector(*robot, output, config);

  bool keep_recording = true;
  while (keep_recording && !ExitRequestedBySignal()) {
    std::string task = "pick_trash";
    if (seconds == 0) {
      std::cout << "\nReady to record Episode " << collector.EpisodeCount()
                << ". Enter task name (Enter for pick_trash): " << std::flush;
      std::string entered;
      if (!ReadLine(entered)) break;
      if (!entered.empty()) task = entered;
      std::cout << "Prepare the robot, then press Enter to start..." << std::flush;
      if (!ReadLine(entered)) break;
    } else {
      task = "cpp_noninteractive";
    }
    if (!collector.StartRecording(task)) {
      (void)robot->Close();
      return 1;
    }
    std::cout << "Recording in progress. Press Enter to stop (or Ctrl+C).\n" << std::flush;
    auto next_stats = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    const auto print_stats = [&] {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_stats) {
        collector.PrintStats();
        std::cout.flush();
        next_stats = now + std::chrono::seconds(1);
      }
    };
    if (seconds == 0) {
      std::string ignored;
      (void)ReadLine(ignored, print_stats);
    } else {
      const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
      while (!ExitRequestedBySignal() && std::chrono::steady_clock::now() < end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        print_stats();
      }
    }
    const auto info = collector.StopRecording();
    if (!info) {
      (void)robot->Close();
      return ExitRequestedBySignal() ? 130 : 1;
    }
    std::cout << "Episode " << info->episode_id << " complete: " << info->num_records
              << " records, " << info->episode_dir << "\n";
    if (seconds > 0 || once || ExitRequestedBySignal()) break;
    std::cout << "Continue recording next episode? (y/n, default n): " << std::flush;
    std::string answer;
    if (!ReadLine(answer) || answer != "y") keep_recording = false;
  }
  (void)robot->Close();
  return ExitRequestedBySignal() ? 130 : 0;
}
