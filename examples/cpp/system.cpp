// Counterpart of python/v1/system.py.
#include "x2robot/sdk/robot.hpp"
#include "sdk_mode_cleanup.hpp"

#include <chrono>
#include <iostream>
#include <string>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  ::xr::sdk::v2::RobotModeParam mode; mode.set_mode(::xr::sdk::v2::SDK);
  bool ok = check(x2robot::examples::NoteWorkMode(robot, mode, robot->GetSystem().SetWorkMode(mode, std::chrono::seconds{3})), "System.SetWorkMode");
  std::cout << (ok ? "true" : "false") << '\n';
  auto system = robot->GetSystem();
  auto info = system.GetStaticInfo(std::chrono::seconds{3});
  auto dynamic = system.GetDynamicInfo(std::chrono::seconds{3});
  auto model = system.GetModelType(std::chrono::seconds{3});
  ok = check(info, "System.GetStaticInfo") && check(dynamic, "System.GetDynamicInfo") &&
       check(model, "System.GetModelType") && ok;
  if (info.Ok()) {
    std::cout << "static_info: model=" << info.Value().model()
              << " robot_id=" << info.Value().robot_id()
              << " device_sn=" << info.Value().device_sn()
              << " software_version=" << info.Value().software_version() << '\n';
  }
  if (dynamic.Ok()) {
    const auto& d = dynamic.Value();
    std::cout << "dynamic_info: is_charging: " << d.power_status().is_charging()
              << ", battery_level: " << d.power_status().value() << "%"
              << " cpu_usage: " << d.runtime_info().cpu_load_percent() << "%"
              << " gpu_usage: " << d.runtime_info().gpu_load_percent() << "%"
              << " memory_usage: " << d.runtime_info().memory_usage_mb() << "MB"
              << ", core_temp: " << d.runtime_info().core_temp_celsius() << "℃\n";
  }
  if (model.Ok()) std::cout << "model_type: "
                            << ::xr::sdk::v2::RobotModelType_Name(model.Value().model_type()) << '\n';
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
