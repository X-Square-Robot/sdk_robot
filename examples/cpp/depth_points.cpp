// Counterpart of python/v1/depth_points.py.  PointCloud2 is a generated
// protobuf message; its data field is decoded here without introducing a
// second C++ point-cloud type.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"
#include "point_cloud.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using PointCloud2 = ::sensor_msgs::PointCloud2;

namespace {
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

using x2robot::examples::point_cloud::Display;

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "single";
  const std::string source = argc > 3 ? argv[3] : "chassis";
  if (source != "chassis" && source != "lidar") {
    std::cerr << "Valid point sources: chassis, lidar\n";
    return 2;
  }
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto service = robot->GetDepthPoints();
  bool ok = true;
  if (action == "single") {
    std::cout << "Performing a single read...\n";
    auto cloud = source == "lidar" ? service.GetLidarPoints(std::chrono::seconds{5})
                                   : service.GetChassisDepthPoints(std::chrono::seconds{5});
    ok = check(cloud, source == "lidar" ? "DepthPoints.GetLidarPoints" : "DepthPoints.GetChassisDepthPoints");
    if (ok) { ok = Display(cloud.Value()); if (ok) std::cout << "Single read completed.\n"; }
  } else if (action == "stream") {
    std::cout << "Starting depth points stream... Press Ctrl+C to stop.\n";
    auto opened = source == "lidar" ? service.GetLidarPointsStream(std::chrono::seconds{5})
                                    : service.GetChassisDepthPointsStream(std::chrono::seconds{5});
    ok = check(opened, source == "lidar" ? "DepthPoints.GetLidarPointsStream" : "DepthPoints.GetChassisDepthPointsStream");
    if (ok) {
      auto stream = std::move(opened.Value());
      while (!x2robot::examples::ExitRequestedBySignal()) {
        auto cloud = stream->Read(std::chrono::seconds{2});
        if (x2robot::examples::ExitRequestedBySignal()) break;
        if (!cloud.Ok()) { ok = check(cloud, "DepthPoints stream read"); break; }
        if (!Display(cloud.Value())) { ok = false; break; }
      }
      ok = check(stream->Cancel(), "DepthPoints stream cancel") && ok;
    }
  } else {
    std::cerr << "Valid actions: single, stream\n"; ok = false;
  }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
