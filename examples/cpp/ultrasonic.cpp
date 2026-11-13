// Counterpart of python/v1/ultrasonic.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <chrono>
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
  std::cout << '\n' << label << ":\n  - Distance: " << value.range() << " m\n"
            << "  - Radiation Type: " << radiation << "\n"
            << "  - Field of View: " << value.field_of_view() << " rad ("
            << value.field_of_view() * 180.0 / 3.141592653589793 << "°)\n"
            << "  - Min Range: " << value.min_range() << " m\n"
            << "  - Max Range: " << value.max_range() << " m\n"
            << (value.range() < value.min_range() ? "  - Status: Distance too close\n" :
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
  auto service = robot->GetUltrasonic();
  bool ok = true;
  auto read = [&](int index, bool stream) {
    auto get = [&](int i) { switch (i) {
      case 1: return service.GetChassisUltrasonic1(5s);
      case 2: return service.GetChassisUltrasonic2(5s);
      case 3: return service.GetChassisUltrasonic3(5s);
      default: return service.GetChassisUltrasonic4(5s); } };
    if (stream) {
      auto opened = [&]() {
        switch (index) {
          case 1: return service.GetChassisUltrasonic1Stream(5s);
          case 2: return service.GetChassisUltrasonic2Stream(5s);
          case 3: return service.GetChassisUltrasonic3Stream(5s);
          default: return service.GetChassisUltrasonic4Stream(5s);
        }}();
      if (!check(opened, "Ultrasonic stream")) { stop = true; return false; }
      auto handle = std::move(opened.Value());
      bool stream_ok = true;
      while (!stop) {
        auto item = handle->Read(2s); if (!item.Ok()) { stream_ok = check(item, "Ultrasonic stream read"); stop = true; break; }
        Display(item.Value(), "Sensor " + std::to_string(index) + " Stream");
      }
      return check(handle->Cancel(), "Ultrasonic stream cancel") && stream_ok;
    }
    auto item = get(index); if (!check(item, "Ultrasonic.Get")) return false;
    Display(item.Value(), "Sensor " + std::to_string(index)); return true;
  };
  if (action == "all") { for (int i = 1; i <= 4; ++i) ok = read(i, false) && ok; }
  else if (action == "all-stream") {
    bool results[4]{};
    std::thread a([&] { results[0] = read(1, true); });
    std::thread b([&] { results[1] = read(2, true); });
    std::thread c([&] { results[2] = read(3, true); });
    std::thread d([&] { results[3] = read(4, true); });
    a.join(); b.join(); c.join(); d.join();
    ok = results[0] && results[1] && results[2] && results[3];
  } else if (action.rfind("sensor-", 0) == 0) {
    const auto dash = action.find('-', 7);
    const int index = std::stoi(action.substr(7, dash == std::string::npos ? dash : dash - 7));
    if (index < 1 || index > 4) { std::cerr << "Sensor index must be 1..4\n"; ok = false; }
    else ok = read(index, dash != std::string::npos);
  } else { std::cerr << "Valid actions: sensor-1..4[-stream], all, all-stream\n"; ok = false; }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
