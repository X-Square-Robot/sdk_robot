#include "data_collection/data_collector.hpp"
#include "interrupt.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};
void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

// Low-level, non-interactive collector.  It is useful in scripts and in CI:
//   data_collection <address> <output-dir> <seconds> [x1|x2|x3|desktop] [binocular]
// The richer interactive workflow lives in data_collection_example.
int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::filesystem::path output = argc > 2 ? argv[2] : "./collected_data";
  const int seconds = argc > 3 ? std::max(1, std::atoi(argv[3])) : 10;
  const std::string requested_model = argc > 4 ? argv[4] : "auto";
  const bool binocular = argc > 5 && std::string(argv[5]) == "binocular";

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);
  auto connected = x2robot::sdk::Robot::Create(address, true);
  if (!connected.Ok()) {
    std::cerr << "Robot.Create failed: " << connected.ErrorValue().error_message << '\n';
    return 1;
  }
  auto robot = std::move(connected.Value());
  // 默认按客户端解析出的机型选配置（与 data_collection_example 一致）。
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
            << "\n";
  CollectionConfig config = CollectionConfig::ForModel(model_key, binocular);
  sdk_examples::DataCollector collector(*robot, output, config);
  if (!collector.StartRecording("cpp_data_collection")) return 1;

  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  while (!g_stop.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    collector.PrintStats();
  }
  const auto info = collector.StopRecording();
  const auto closed = robot->Close();
  if (!closed.Ok()) {
    std::cerr << "Robot.Close failed: " << closed.ErrorValue().error_message << '\n';
    return 1;
  }
  return info ? 0 : 1;
}
