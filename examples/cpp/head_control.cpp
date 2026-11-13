// Counterpart of python/v1/head_control.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;

namespace { std::atomic_bool stop{false}; void OnSignal(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); } }

template <typename T>
bool check(const SdkResult<T>& result, const char* operation) {
  if (result.Ok()) return true;
  const auto& e = result.ErrorValue();
  std::cerr << operation << " failed [" << e.error_name << "]: " << e.error_message << '\n';
  return false;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "move";
  std::signal(SIGINT, OnSignal);
  auto connected = Robot::Create(address, true);
  if (!check(connected, "Robot.Create")) return 1;
  auto robot = std::move(connected.Value());
  bool ok = true;
  auto head = robot->GetHeadController();
  if (action == "move") {
    ::xr::sdk::v2::RobotModeParam mode; mode.set_mode(::xr::sdk::v2::SDK);
    ok = check(x2robot::examples::NoteWorkMode(robot, mode, robot->GetSystem().SetWorkMode(mode, 3s)), "System.SetWorkMode") && ok;
    ::xr::sdk::v2::HeadPose target; target.set_pitch(0.0f); target.set_yaw(0.5f);
    std::cout << "set head pose to yaw=0.5\n";
    ok = check(head.SetPose(target, 3s), "Head.SetPose") && ok;
    std::this_thread::sleep_for(1s);
    auto pose = head.GetPose(3s);
    ok = check(pose, "Head.GetPose") && ok;
    if (pose.Ok()) std::cout << "cur_head_pose: pitch=" << pose.Value().pitch()
                               << ", yaw=" << pose.Value().yaw() << '\n';
    std::cout << "reset head\n";
    ok = check(head.Reset(3s), "Head.Reset") && ok;
    std::this_thread::sleep_for(1s);
    pose = head.GetPose(3s); ok = check(pose, "Head.GetPose(after reset)") && ok;
    if (pose.Ok()) std::cout << "cur_head_pose: pitch=" << pose.Value().pitch()
                               << ", yaw=" << pose.Value().yaw() << '\n';
  } else if (action == "stream") {
    std::cout << "Starting head joint states streaming (Ctrl+C to stop)...\n";
    auto opened = head.GetJointStatesStream(5s);
    ok = check(opened, "Head.GetJointStatesStream") && ok;
    if (opened.Ok()) {
      auto stream = std::move(opened.Value());
      while (!stop.load()) {
        auto state = stream->Read(2s);
        if (!state.Ok()) { if (!stop.load()) ok = check(state, "Head joint stream read"); break; }
        std::cout << "joint_state: " << state.Value().ShortDebugString() << '\n';
      }
      ok = check(stream->Cancel(), "Head joint stream cancel") && ok;
    }
  } else { std::cerr << "Valid actions: move, stream\n"; ok = false; }
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
