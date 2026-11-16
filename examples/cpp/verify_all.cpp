#include "x2robot/sdk/robot.hpp"

#include <chrono>
#include <iostream>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  std::cerr << operation << " failed: " << result.ErrorValue().error_message << '\n';
  return false;
}

int main(int argc, char** argv) {
  auto connected = Robot::Create(argc > 1 ? argv[1] : "127.0.0.1:55051", true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  ::xr::sdk::v2::PingRequest ping;
  ping.set_payload("verify-all");
  auto pong = robot->GetBenchmark().Ping(ping, std::chrono::seconds{3});
  auto model = robot->GetSystem().GetModelType(std::chrono::seconds{3});
  ::xr::sdk::v2::GetRobotStatusRequest status_request;
  auto status = robot->GetRobotStatusService().GetRobotStatus(
      status_request, std::chrono::seconds{3});
  bool ok = check(pong, "Benchmark.Ping") && check(model, "System.GetModelType") &&
            check(status, "RobotStatus.GetRobotStatus");
  if (ok) std::cout << "model=" << ::xr::sdk::v2::RobotModelType_Name(
                            model.Value().model_type()) << "\n";
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
