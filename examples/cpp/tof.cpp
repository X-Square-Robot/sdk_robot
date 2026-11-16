// Counterpart of python/v1/tof.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <chrono>
#include <cmath>
#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using Range = ::sensor_msgs::Range;
using namespace std::chrono_literals;

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

void Display(const Range& value, const std::string& label) {
  const char* radiation = value.radiation_type() == 0 ? "ULTRASOUND" :
                          value.radiation_type() == 1 ? "INFRARED" : "UNKNOWN";
  std::cout << label << ":\n  - Distance: " << value.range() << " m\n"
            << "  - Radiation Type: " << radiation << "\n"
            << "  - Field of View: " << value.field_of_view() << " rad ("
            << value.field_of_view() * 180.0 / 3.141592653589793 << "°)\n"
            << "  - Min Range: " << value.min_range() << " m\n"
            << "  - Max Range: " << value.max_range() << " m\n";
  std::cout << (value.range() < value.min_range() ? "  - Status: Distance too close\n" :
                value.range() > value.max_range() ? "  - Status: Out of measurement range\n" :
                                                     "  - Status: OK\n")
            << "================================================\n";
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "sensor-1";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto service = robot->GetTof();
  bool ok = true;
  auto read = [&](int index, bool stream) {
    if (robot->ModelName() == "quanta_x3")
    {
        // 此机型没有 TOF，只把能力门禁的明确拒绝视为符合预期。
        auto item = index == 1 ? service.GetChassisTof1(5s) : service.GetChassisTof2(5s);
        if (!item.Ok() && item.ErrorValue().error_code == 100501 &&
            item.ErrorValue().error_name == "UNIMPLEMENTED")
        {
            std::cout << "Tof.GetChassisTof" << index << " is not public for Quanta X3; skipped\n";
            return true;
        }
        std::cerr << "Tof capability check failed: expected 100501 UNIMPLEMENTED\n";
        check(item, "Tof.Get");
        return false;
    }
    if (stream) {
      auto opened = index == 1 ? service.GetChassisTof1Stream(5s) : service.GetChassisTof2Stream(5s);
      if (!check(opened, "Tof stream")) { stop = true; return false; }
      auto handle = std::move(opened.Value());
      bool stream_ok = true;
      while (!stop) {
        auto item = handle->Read(2s);
        if (!item.Ok()) { stream_ok = check(item, "Tof stream read"); stop = true; break; }
        Display(item.Value(), "Sensor " + std::to_string(index) + " Stream");
      }
      return check(handle->Cancel(), "Tof stream cancel") && stream_ok;
    }
    auto item = index == 1 ? service.GetChassisTof1(5s) : service.GetChassisTof2(5s);
    if (!check(item, "Tof.Get")) return false;
    Display(item.Value(), "Sensor " + std::to_string(index)); return true;
  };
  if (action == "sensor-1") ok = read(1, false);
  else if (action == "sensor-2") ok = read(2, false);
  else if (action == "sensor-1-stream") ok = read(1, true);
  else if (action == "sensor-2-stream") ok = read(2, true);
  else if (action == "both") { ok = read(1, false) && read(2, false); }
  else if (action == "both-stream") {
    std::cout << "Starting streams for both sensors...\n";
    bool one_ok = false, two_ok = false;
    std::thread one([&] { one_ok = read(1, true); });
    std::thread two([&] { two_ok = read(2, true); });
    one.join(); two.join();
    ok = one_ok && two_ok;
  } else { std::cerr << "Valid actions: sensor-1, sensor-2, sensor-1-stream, sensor-2-stream, both, both-stream\n"; ok = false; }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
