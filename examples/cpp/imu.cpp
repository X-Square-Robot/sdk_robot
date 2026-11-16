// Counterpart of python/v1/imu.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"

#include <algorithm>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cmath>
#include <iostream>
#include <string>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using ImuMessage = ::sensor_msgs::Imu;

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

void QuaternionToEuler(const ::geometry_msgs::Quaternion& q, double* roll, double* pitch,
                       double* yaw) {
  const double sinr = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  *roll = std::atan2(sinr, cosr);
  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  *pitch = std::asin(std::clamp(sinp, -1.0, 1.0));
  const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  *yaw = std::atan2(siny, cosy);
}

void Display(const ImuMessage& imu) {
  constexpr double kDeg = 180.0 / 3.14159265358979323846;
  if (imu.has_orientation()) {
    double roll{}, pitch{}, yaw{};
    QuaternionToEuler(imu.orientation(), &roll, &pitch, &yaw);
    std::cout << "Orientation (Euler): Roll=" << roll * kDeg << "°, Pitch=" << pitch * kDeg
              << "°, Yaw=" << yaw * kDeg << "°\n";
  } else {
    std::cout << "Orientation (Euler): Not available\n";
  }
  if (imu.has_angular_velocity()) {
    const auto& v = imu.angular_velocity();
    std::cout << "Angular Velocity (rad/s): x=" << v.x() << ", y=" << v.y() << ", z=" << v.z()
              << '\n';
  } else std::cout << "Angular Velocity (rad/s): Not available\n";
  if (imu.has_linear_acceleration()) {
    const auto& a = imu.linear_acceleration();
    std::cout << "Linear Acceleration (m/s²): x=" << a.x() << ", y=" << a.y() << ", z=" << a.z()
              << '\n';
  } else std::cout << "Linear Acceleration (m/s²): Not available\n";
  std::cout << std::string(48, '=') << '\n';
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "single";
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  auto service = robot->GetImu();
  bool ok = true;
  if (action == "single") {
    std::cout << "Performing a single read...\n";
    auto value = service.GetChassisImu(std::chrono::seconds{5});
    ok = check(value, "Imu.GetChassisImu");
    if (ok) { Display(value.Value()); std::cout << "Single read completed.\n"; }
  } else if (action == "stream") {
    std::cout << "Starting IMU stream... Press Ctrl+C to stop.\n";
    auto opened = service.GetChassisImuStream(std::chrono::seconds{5});
    ok = check(opened, "Imu.GetChassisImuStream");
    if (ok) {
      auto stream = std::move(opened.Value());
      while (!stop) {
        auto value = stream->Read(std::chrono::seconds{2});
        if (!value.Ok()) { ok = check(value, "Imu stream read"); break; }
        Display(value.Value());
      }
      ok = check(stream->Cancel(), "Imu stream cancel") && ok;
    }
  } else {
    std::cerr << "Valid actions: single, stream\n"; ok = false;
  }
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
