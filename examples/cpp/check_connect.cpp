#include "x2robot/sdk/robot.hpp"

#include <chrono>
#include <iostream>
#include <string>

using namespace x2robot::sdk;

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  std::cerr << operation << ": " << result.ErrorValue().error_message << '\n';
  return false;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  ::xr::sdk::v2::PingRequest request;
  request.set_payload("Hello, X2Robot!");
  auto response = robot->GetBenchmark().Ping(request, std::chrono::seconds(3));
  const bool ok = check(response, "Benchmark.Ping") &&
                  response.Value().payload() == "Pong to: Hello, X2Robot!";
  if (ok) std::cout << "transport=" << robot->Transport() << " payload="
                    << response.Value().payload() << '\n';
  check(robot->Close(), "Robot.Close");
  return ok ? 0 : 1;
}
