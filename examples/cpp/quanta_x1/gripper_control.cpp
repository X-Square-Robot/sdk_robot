// Counterpart of python/v1/quanta_x1/gripper_control.py.
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

namespace {
std::atomic_bool stop{false};
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}

template <typename T>
bool check(const SdkResult<T>& result, const char* name) {
  if (result.Ok()) return true;
  std::cerr << name << " failed: " << result.ErrorValue().error_message << '\n';
  return false;
}

template <typename Gripper, typename RobotPtr>
bool Run(Gripper gripper, RobotPtr& robot, const std::string& action) {
  if (action == "stream") {
    bool ok = true;
    auto opened = gripper.GetJointStatesStream(5s);
    if (!check(opened, "Gripper.GetJointStatesStream")) return false;
    auto stream = std::move(opened.Value());
    std::cout << "Starting gripper data streaming (Ctrl+C to stop)...\n";
    while (!stop) {
      auto value = stream->Read(2s);
      if (!check(value, "Gripper stream read")) { ok = false; break; }
      std::cout << "joint_state: " << value.Value().ShortDebugString() << '\n';
    }
    return check(stream->Cancel(), "Gripper.JointStatesStream.Cancel") && ok;
  }
  if (action != "move") {
    std::cerr << "Valid actions: move, stream\n";
    return false;
  }

  ::xr::sdk::v2::RobotModeParam sdk;
  sdk.set_mode(::xr::sdk::v2::SDK);
  bool ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
  ::xr::sdk::v2::ManipulatorControlModeParam mode;
  mode.set_mode(::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
  ok = check(robot->GetRobotControl().SetManipulatorControlMode(mode, 3s),
             "RobotControl.SetManipulatorControlMode") && ok;

  auto initial = gripper.GetPosition(3s);
  ok = check(initial, "Gripper.GetPosition") && ok;
  if (initial.Ok()) std::cout << initial.Value().ShortDebugString() << '\n';

  for (float position : {0.0f, 0.3f, 0.6f}) {
    ::xr::sdk::v2::GripperPosition target;
    target.set_position(position);
    std::cout << "set gripper position to " << position << '\n';
    ok = check(gripper.SetPosition(target, 3s), "Gripper.SetPosition") && ok;
    std::this_thread::sleep_for(position == 0.0f ? 1500ms : 1000ms);
    auto current = gripper.GetPosition(3s);
    ok = check(current, "Gripper.GetPosition") && ok;
    if (current.Ok()) std::cout << current.Value().ShortDebugString() << '\n';
  }
  return ok;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string action = argc > 2 ? argv[2] : "move";
  const std::string side = argc > 3 ? argv[3] : "left";
  std::signal(SIGINT, signal_handler);
  auto created = Robot::Create(address, true);
  if (!check(created, "Robot.Create")) return 1;
  auto robot = std::move(created.Value());
  bool ok = side == "right"
                ? Run(robot->GetRightGripperController(), robot, action)
                : Run(robot->GetLeftGripperController(), robot, action);
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
