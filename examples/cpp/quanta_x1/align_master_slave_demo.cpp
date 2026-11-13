// Counterpart of python/v1/quanta_x1/align_master_slave_demo.py.
#include "x2robot/sdk/robot.hpp"
#include "sdk_mode_cleanup.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using x2robot::sdk::Robot;
using x2robot::sdk::SdkResult;
using namespace std::chrono_literals;

template <typename T>
bool check(const SdkResult<T>& result, const char* name) {
  if (result.Ok()) return true;
  std::cerr << name << " failed: " << result.ErrorValue().error_message << '\n';
  return false;
}

template <typename Arm, typename RobotPtr>
bool Run(Arm arm, RobotPtr& robot, const std::string& side) {
  std::cout << "moving slave " << side << " arm to a preset end pose...\n";
  ::xr::sdk::v2::RobotModeParam sdk;
  sdk.set_mode(::xr::sdk::v2::SDK);
  bool ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
  ::xr::sdk::v2::ManipulatorControlModeParam mode;
  mode.set_mode(::xr::sdk::v2::MANIPULATOR_END_POSE);
  ok = check(robot->GetRobotControl().SetManipulatorControlMode(mode, 3s),
             "RobotControl.SetManipulatorControlMode") && ok;
  ::geometry_msgs::Pose target;
  target.mutable_position()->set_z(0.2);
  target.mutable_orientation()->set_w(1.0);
  ok = check(arm.SetEndPose(target, 5s), "Arm.SetEndPose") && ok;
  std::this_thread::sleep_for(500ms);
  if (ok) std::cout << "Slave arm move completed.\n";
  return ok;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string side = argc > 2 ? argv[2] : "left";
  auto created = Robot::Create(address, true);
  if (!check(created, "Robot.Create")) return 1;
  auto robot = std::move(created.Value());
  bool ok = side == "right"
                ? Run(robot->GetRightArmController(), robot, side)
                : Run(robot->GetLeftArmController(), robot, side);
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok;
  return ok ? 0 : 1;
}
