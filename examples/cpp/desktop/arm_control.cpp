// Desktop arm_control counterpart of python/v1/desktop/arm_control.py.
#include "x2robot/sdk/robot.hpp"
#include "interrupt.hpp"
#include "sdk_mode_cleanup.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
using x2robot::sdk::Robot; using x2robot::sdk::SdkResult; using namespace std::chrono_literals;
namespace {
std::atomic_bool stop{false};
// 与 quanta_x1 手臂示例统一：200 步 x 10 ms 的线性插值轨迹，避免把目标
// 位姿/关节一步下发造成跳变。
constexpr int kTrajectorySteps = 200;
constexpr auto kTrajectoryStepInterval = 10ms;
void signal_handler(int signal) { x2robot::examples::RequestExitOnCtrlC(signal); }
}
template <typename T>
bool check(const SdkResult<T>& result, const char* name) {
  if (result.Ok()) return true;
  std::cerr << name << " failed: " << result.ErrorValue().error_message << '\n';
  return false;
}

template <typename Arm, typename RobotPtr>
bool Run(Arm arm, RobotPtr& robot,
         const std::string& action, const std::string& mode) {
  bool ok = true;
  if (action == "move") {
    ::xr::sdk::v2::RobotModeParam sdk;
    sdk.set_mode(::xr::sdk::v2::SDK);
    ok = check(x2robot::examples::NoteWorkMode(robot, sdk, robot->GetSystem().SetWorkMode(sdk, 3s)), "System.SetWorkMode");
    ::xr::sdk::v2::ManipulatorControlModeParam control;
    control.set_mode(mode == "end_pose"
                         ? ::xr::sdk::v2::MANIPULATOR_END_POSE
                         : ::xr::sdk::v2::MANIPULATOR_JOINT_POSITIONS);
    ok = check(robot->GetRobotControl().SetManipulatorControlMode(control, 3s),
               "RobotControl.SetManipulatorControlMode") && ok;

    if (mode == "end_pose") {
      auto current = arm.GetEndPose(3s);
      ok = check(current, "Arm.GetEndPose") && ok;
      ::geometry_msgs::Pose target;
      target.mutable_position()->set_z(0.2);
      target.mutable_orientation()->set_w(1.0);
      if (current.Ok()) {
        for (int i = 1; i <= kTrajectorySteps; ++i) {
          const double ratio = static_cast<double>(i) / kTrajectorySteps;
          auto pose = current.Value().pose();
          pose.mutable_position()->set_x(
              current.Value().pose().position().x() * (1 - ratio) +
              target.position().x() * ratio);
          pose.mutable_position()->set_y(
              current.Value().pose().position().y() * (1 - ratio) +
              target.position().y() * ratio);
          pose.mutable_position()->set_z(
              current.Value().pose().position().z() * (1 - ratio) +
              target.position().z() * ratio);
          pose.mutable_orientation()->CopyFrom(target.orientation());
          if (!check(arm.SetEndPose(pose, 1s), "Arm.SetEndPose")) return false;
          std::this_thread::sleep_for(kTrajectoryStepInterval);
        }
      }
    } else {
      auto current = arm.GetJointStates(3s);
      ok = check(current, "Arm.GetJointStates") && ok;
      ::xr::sdk::v2::JointPositions target;
      const int joint_count = current.Ok() ? current.Value().position_size() : 6;
      for (int i = 0; i < joint_count; ++i) {
        target.add_positions(i == 0 ? -0.1486 : 0.0);
      }
      if (current.Ok()) {
        for (int i = 1; i <= kTrajectorySteps; ++i) {
          ::xr::sdk::v2::JointPositions positions;
          const double ratio = static_cast<double>(i) / kTrajectorySteps;
          for (int j = 0; j < target.positions_size(); ++j) {
            positions.add_positions(current.Value().position(j) * (1 - ratio) +
                                    target.positions(j) * ratio);
          }
          if (!check(arm.SetJointPositions(positions, 1s),
                     "Arm.SetJointPositions")) return false;
          std::this_thread::sleep_for(kTrajectoryStepInterval);
        }
      }
    }
    std::cout << "Movement Completed\n";
    return ok;
  }

  if (action == "stream") {
    if (mode == "end_pose") {
      auto opened = arm.GetEndPoseStream(5s);
      ok = check(opened, "Arm.GetEndPoseStream");
      if (opened.Ok()) {
        auto stream = std::move(opened.Value());
        while (!stop) {
          auto value = stream->Read(2s);
          if (!check(value, "Arm stream read")) { ok = false; break; }
          std::cout << "end_pose: " << value.Value().ShortDebugString() << '\n';
        }
        ok = check(stream->Cancel(), "Arm.EndPoseStream.Cancel") && ok;
      }
    } else {
      auto opened = arm.GetJointStatesStream(5s);
      ok = check(opened, "Arm.GetJointStatesStream");
      if (opened.Ok()) {
        auto stream = std::move(opened.Value());
        while (!stop) {
          auto value = stream->Read(2s);
          if (!check(value, "Arm stream read")) { ok = false; break; }
          std::cout << "joint_state: " << value.Value().ShortDebugString() << '\n';
        }
        ok = check(stream->Cancel(), "Arm.JointStatesStream.Cancel") && ok;
      }
    }
    return ok;
  }

  std::cerr << "Valid actions: move, stream; modes: joint_pos, end_pose\n";
  return false;
}

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:55051";
  const std::string arm_name = argc > 2 ? argv[2] : "left";
  const std::string action = argc > 3 ? argv[3] : "move";
  const std::string mode = argc > 4 ? argv[4] : "joint_pos";
  std::signal(SIGINT, signal_handler);
  auto created = Robot::Create(address, true);
  if (!check(created, "Robot.Create")) return 1;
  auto robot = std::move(created.Value());
  bool ok = arm_name == "right"
                ? Run(robot->GetRightArmController(), robot, action, mode)
                : Run(robot->GetLeftArmController(), robot, action, mode);
  ok = x2robot::examples::RestoreIdleModeOnExit(robot) && ok;
  ok = check(robot->Close(), "Robot.Close") && ok; return ok ? 0 : 1;
}
