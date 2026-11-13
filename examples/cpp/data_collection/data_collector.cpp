#include "data_collector.hpp"
#include "interrupt.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_stop{false};
void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

// Standalone collector target retained for compatibility with the original
// artifact.  It intentionally shares the exact implementation used by the
// public data_collection/data_collection_example binaries.
int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::filesystem::path output = argc > 2 ? argv[2] : "./collected_data";
  const int seconds = argc > 3 ? std::max(1, std::atoi(argv[3])) : 10;
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  auto connected = x2robot::sdk::Robot::Create(address, true);
  if (!connected.Ok()) {
    std::cerr << "Robot.Create failed: " << connected.ErrorValue().error_message << '\n';
    return 1;
  }
  auto robot = std::move(connected.Value());
  const auto model_key = CollectionConfig::NormalizeModelKey(robot->ModelName());
  if (model_key.empty()) {
    std::cerr << "Unsupported model '" << robot->ModelName() << "'\n";
    (void)robot->Close();
    return 2;
  }
  sdk_examples::DataCollector collector(*robot, output, CollectionConfig::ForModel(model_key));
  if (!collector.StartRecording("cpp_data_collector")) return 1;
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  while (!g_stop.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    collector.PrintStats();
  }
  const auto info = collector.StopRecording();
  (void)robot->Close();
  return info ? 0 : 1;
}
