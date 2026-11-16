// Counterpart of python/v1/radar.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <algorithm>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using LaserScan = ::sensor_msgs::LaserScan;

namespace {
std::atomic_bool stop{false};
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

void Display(const LaserScan& scan) {
  std::cout << "Angle Range: [" << scan.angle_min() << ", " << scan.angle_max() << "] rad\n"
            << "Angle Increment: " << scan.angle_increment() << " rad (Spec: 0.02)\n"
            << "Time Increment: " << scan.time_increment() << " s\n";
  if (scan.scan_time() > 0)
    std::cout << "Scan Time: " << scan.scan_time() << " s (Frequency: "
              << 1.0 / scan.scan_time() << " Hz, Spec: 10 Hz)\n";
  else std::cout << "Scan Time: " << scan.scan_time() << " s\n";
  std::cout << "Range Limits: [" << scan.range_min() << ", " << scan.range_max()
            << "] m (Spec: [0.05, 25])\n"
            << "Number of Points: " << scan.ranges_size() << '\n';
  const int n = std::min(10, scan.ranges_size());
  if (n > 0) {
    std::cout << "Sample Ranges (first 10): [";
    for (int i = 0; i < n; ++i) std::cout << (i ? ", " : "") << scan.ranges(i);
    std::cout << "]\n";
    float min_value = std::numeric_limits<float>::infinity();
    float max_value = -std::numeric_limits<float>::infinity();
    for (float value : scan.ranges()) {
      if (!std::isnan(value)) { min_value = std::min(min_value, value); max_value = std::max(max_value, value); }
    }
    if (std::isfinite(min_value)) std::cout << "Min Range in Scan: " << min_value
                                             << "\nMax Range in Scan: " << max_value << '\n';
    else std::cout << "Min/Max Range in Scan: N/A (all values are NaN)\n";
  }
  const int ni = std::min(10, scan.intensities_size());
  if (ni > 0) {
    std::cout << "Sample Intensities (first 10): [";
    for (int i = 0; i < ni; ++i) std::cout << (i ? ", " : "") << scan.intensities(i);
    std::cout << "]\n";
  }
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "single";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto service = robot->GetRadarService();
  bool ok = true;
  if (robot->ModelName() == "quanta_x3" && (action == "single" || action == "stream"))
  {
      // 此机型已收回 LaserScan；只接受能力门禁的明确响应，不能把超时或无数据当作跳过。
      auto scan = service.GetLaserScan(std::chrono::seconds{5});
      ok = !scan.Ok() && scan.ErrorValue().error_code == 100501 &&
           scan.ErrorValue().error_name == "UNIMPLEMENTED";
      if (ok)
      {
          std::cout << "Radar.GetLaserScan is not public for Quanta X3; skipped\n";
      }
      else
      {
          std::cerr << "Radar capability check failed: expected 100501 UNIMPLEMENTED\n";
          check(scan, "Radar.GetLaserScan");
      }
  }
  else if (action == "single") {
    std::cout << "Performing a single read...\n";
    auto scan = service.GetLaserScan(std::chrono::seconds{5});
    ok = check(scan, "Radar.GetLaserScan");
    if (ok) { Display(scan.Value()); std::cout << "Single read completed.\n"; }
  } else if (action == "stream") {
    std::cout << "Starting radar stream... Press Ctrl+C to stop.\n";
    auto opened = service.GetLaserScanStream(std::chrono::seconds{5});
    ok = check(opened, "Radar.GetLaserScanStream");
    if (ok) {
      auto stream = std::move(opened.Value());
      while (!stop) {
        auto scan = stream->Read(std::chrono::seconds{2});
        if (!scan.Ok()) { ok = check(scan, "Radar stream read"); break; }
        Display(scan.Value()); std::cout << std::string(80, '-') << '\n';
      }
      ok = check(stream->Cancel(), "Radar stream cancel") && ok;
    }
  } else { std::cerr << "Valid actions: single, stream\n"; ok = false; }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
